"""Tests of tools/loop_graph.py on synthetic captures; no recorded data or mapper runs required."""
import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import loop_graph as lg
import map_quality_lib


def translation(x, y=0.0, z=0.0):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def chain_capture(n=5, spacing=1.0, tries=None, settings=None, nodes_per_segment=3):
    """n segments along x (spacing apart, unrotated), one window of `nodes_per_segment` nodes each, 0.3 s apart."""
    segs, poses, times = [], [], []
    offsets = [0]
    for k in range(n):
        t0 = k * 1.0
        segs.append(dict(first=k, last=k, pose=translation(k * spacing), center=np.zeros(3), path_begin=k * spacing,
                         path_end=(k + 1) * spacing, t_begin=t0, t_end=t0 + 1.0))
        for m in range(nodes_per_segment):
            poses.append(translation(k * spacing + m * spacing / nodes_per_segment))
            times.append(t0 + m * 0.3)
        offsets.append(offsets[-1] + nodes_per_segment)
    cfg = {"loop_closure_edge_sigma_m": "0.05", "loop_closure_edge_sigma_deg": "0.3", "loop_closure_odometry_sigma_m": "0.1",
           "loop_closure_odometry_sigma_deg": "0.5", "loop_closure_kernel_scale": "0"}
    cfg.update(settings or {})
    t = np.array(times)
    return lg.Capture(cfg, segs, np.array(offsets), np.array(poses), t, 1.7e9 + t, tries or {})


def try_of(correction, H=None):
    return dict(C=correction, H=np.eye(6) if H is None else H, center=np.zeros(3), overlap_before=0.5, overlap_after=0.8, shift=0.1,
                angle=0.0, information_ratio=0.0, accepted=True, asked=True, registered=True)


def capture_bytes(version, readings=None, blocks=None, fine_time=False):
    """A capture file as dMapper writes it (loop_closure_capture_file.cpp): one setting, two windows of two nodes, one
    segment, the result poses, the loop count and, from format 2 on, the windows' readings (None: absent). blocks: None for
    no tries, else one try per entry (segment 0 -> k, k from 1), registered when its entry (a list of 3x3 per source window,
    format 3 only) is not empty. fine_time (format 4): False for no curves, else one eligible run with one window's curves."""
    out = bytearray(b"DMLCCAP1" + struct.pack("<I", version))
    q = lambda n: struct.pack("<Q", n)  # noqa: E731
    d = lambda *v: struct.pack("<%dd" % len(v), *v)  # noqa: E731
    out += q(1) + q(4) + b"name" + q(1) + b"1"
    out += q(2)
    for w in range(2):
        out += q(2) + d(*np.eye(4).T.ravel()) + d(*translation(0.5 + w).T.ravel())  # poses
        out += q(2) + d(*np.eye(4).T.ravel()) * 2                                    # motion-model poses
        out += q(2) + d(w * 1.0, 1.7e9, w * 1.0 + 0.4, 1.7e9 + 0.4)                    # stamps
        out += q(0) + q(0)                                                           # IMU angles, accelerations
        out += d(*[0.0] * 27)                                                        # gyro bias and velocity states
    out += q(1) + struct.pack("<ii", 0, 1) + d(*np.eye(4).T.ravel()) + d(0, 0, 0) + d(0, 1, 0, 1.4)
    out += q(len(blocks or []))
    for k, b in enumerate(blocks or [], start=1):
        out += struct.pack("<iiBB", 0, k, 0, int(len(b) > 0)) + d(0.5, 0.8) + d(*translation(0.1 * k).T.ravel())
        out += d(*np.arange(36.0)) + d(0.1, 0.2, 0.3) + d(0, 0, 0) + struct.pack("<BB", 1, 1)
    out += q(2) + (q(2) + d(*np.eye(4).T.ravel()) * 2) * 2
    out += struct.pack("<i", 0)
    if version >= 2:
        out += q(2)
        for r in readings:
            if r is None:
                out += struct.pack("<B", 0)
                continue
            ratio, axis, rest, move, dur = r
            out += struct.pack("<B", 1) + d(ratio, *axis) + struct.pack("<B", rest) + d(*move, dur)
    if version >= 3:
        out += q(len(blocks or []))
        for b in blocks or []:
            out += q(len(b)) + b"".join(d(*np.asarray(m, float).T.ravel()) for m in b)
    if version >= 4:
        out += struct.pack("<B", int(bool(fine_time)))
        if fine_time:
            out += q(1) + d(*translation(0.25).T.ravel())                                       # segment poses
            out += q(1) + struct.pack("<iiiiiB", 0, 1, 1, 0, 1, 0) + d(1.5, 0.0, 1.4) + d(1, 0, 0) + d(0, 1, 0)  # eligible run
            out += q(1) + struct.pack("<ii", 0, 1) + d(2.25) + q(3) + d(-0.1, 0.0, 0.1) + struct.pack("<i", 42)
            out += q(1) + struct.pack("<ii", 9, 3) + q(3) + d(0.25, 0.01, 0.25) + q(3) + struct.pack("<3i", 0, 9, 1)
            out += q(0) + q(0)                                                                  # no held-out score
    return bytes(out)


W_LOOP = np.diag([400.0] * 3 + [1 / np.deg2rad(0.3) ** 2] * 3)


class LoopGraphTest(unittest.TestCase):
    def test_chain_with_one_loop_edge_matches_hand_computed(self):
        # five segments 1 m apart; the loop edge says segment 4 is 0.4 m closer to segment 0 (3.6 m).
        # odometry: four edges in series, weight 100 each (compliance 0.04); loop weight 400 (compliance 0.0025).
        cap = chain_capture(tries={(0, 4, False): try_of(translation(-0.4))})
        g = lg.LoopGraph(cap, [(0, 4, False)], loop_omega={(0, 4, False): W_LOOP})
        sol = g.solve()
        taken = 0.4 * 0.04 / (0.04 + 0.0025)  # the part of the misclosure the odometry chain takes up
        expected_x = np.arange(5) - taken * np.arange(5) / 4
        np.testing.assert_allclose(sol.poses[:, 0, 3], expected_x, atol=1e-9)
        np.testing.assert_allclose(sol.poses[:, 1:3, 3], 0, atol=1e-8)  # finite-difference Jacobians
        np.testing.assert_allclose(sol.poses[:, :3, :3], np.broadcast_to(np.eye(3), (5, 3, 3)), atol=1e-8)
        # cost: odometry 4 edges of stretch taken/4 at weight 100, loop residual (4-taken-3.6) at weight 400
        self.assertAlmostEqual(sol.cost_odometry, 4 * 100 * (taken / 4) ** 2, places=9)
        self.assertAlmostEqual(sol.cost_loops, 400 * (0.4 - taken) ** 2, places=9)
        self.assertAlmostEqual(sol.cost, sol.cost_odometry + sol.cost_loops, places=12)
        self.assertEqual(len(sol.factor_costs), 5)

    def test_zero_residual_graph_stays_put(self):
        cap = chain_capture(tries={(0, 4, False): try_of(np.eye(4)), (1, 3, False): try_of(np.eye(4))})
        g = lg.LoopGraph(cap, [(0, 4, False), (1, 3, False)])
        sol = g.solve()
        np.testing.assert_allclose(sol.poses, g.initial, atol=1e-12)
        self.assertLess(sol.cost, 1e-18)
        np.testing.assert_allclose(g.node_poses(sol), cap.node_pose, atol=1e-12)

    def test_scale_zero_equals_leaving_the_edge_out(self):
        tries = {(0, 4, False): try_of(translation(-0.4), np.diag([4, 2, 1, 50, 50, 50.0])),
                 (1, 3, False): try_of(translation(0.1, 0.05), np.diag([3, 3, 1, 40, 40, 40.0]))}
        cap = chain_capture(tries=tries)
        scaled = lg.LoopGraph(cap, list(tries), loop_scale={"0-4": 0.0}).solve()
        only = lg.LoopGraph(cap, [(1, 3, False)])
        left_out = only.solve()
        np.testing.assert_allclose(scaled.poses, left_out.poses, atol=1e-10)
        self.assertAlmostEqual(scaled.cost, left_out.cost, places=10)
        # the other edge alone really moves something, so the comparison is not vacuous
        self.assertGreater(np.abs(left_out.poses[:, :3, 3] - only.initial[:, :3, 3]).max(), 1e-3)

    def test_loop_edge_information_normalisation(self):
        I = np.eye(4)
        H = np.diag([4, 2, 1, 10, 10, 10.0])
        om, ok = lg.loop_edge_information(I, I, I, H, False, 0.05)
        self.assertTrue(ok)
        np.testing.assert_allclose(om, H / 4 / 0.05 ** 2, rtol=1e-6)  # the best translation direction gets 1/sigma^2
        om, ok = lg.loop_edge_information(I, I, I, H, True, 0.05)  # cross floor: the height is a free nuisance
        np.testing.assert_allclose(om, np.diag([4, 2, 0, 10, 10, 10.0]) / 4 / 0.05 ** 2, rtol=1e-6, atol=1e-6)
        om, ok = lg.loop_edge_information(I, I, I, np.zeros((6, 6)), False, 0.05)
        self.assertFalse(ok)  # no information: dMapper keeps the fixed sigmas
        cap = chain_capture(tries={(0, 4, False): try_of(translation(-0.4), np.zeros((6, 6)))})
        g = lg.LoopGraph(cap, [(0, 4, False)])
        self.assertEqual(g.information_ok, [False])
        np.testing.assert_allclose(np.diag(g.omega[-1]), np.diag(W_LOOP))

    def test_odometry_sigma_hook(self):
        cap = chain_capture(tries={(0, 4, False): try_of(translation(-0.4))})
        loose = lg.LoopGraph(cap, [(0, 4, False)], odo_sigma=(10.0, 30.0), loop_omega={(0, 4, False): W_LOOP}).solve()
        self.assertAlmostEqual(loose.poses[4, 0, 3], 3.6, places=3)  # a very soft odometry lets the loop win
        oo = np.broadcast_to(np.diag([1e6] * 6), (4, 6, 6))
        stiff = lg.LoopGraph(cap, [(0, 4, False)], odo_omega=oo, loop_omega={(0, 4, False): W_LOOP}).solve()
        self.assertAlmostEqual(stiff.poses[4, 0, 3], 4.0 - 0.4 * 4e-6 / (4e-6 + 0.0025), places=7)

    def test_robust_kernel_downweights_a_distant_edge(self):
        tries = {(0, 4, False): try_of(translation(-0.4)), (1, 4, False): try_of(translation(-0.3)),
                 (0, 3, False): try_of(translation(-0.3)), (2, 4, False): try_of(translation(1.5))}
        cap = chain_capture(tries=tries)
        om = {e: W_LOOP for e in tries}
        plain = lg.LoopGraph(cap, list(tries), loop_omega=om).solve()
        robust = lg.LoopGraph(cap, list(tries), loop_omega=om, kernel_scale=3.0).solve()
        self.assertLess(abs(robust.poses[4, 0, 3] - 3.6), abs(plain.poses[4, 0, 3] - 3.6))

    def test_leave_one_out_consistent_edges_have_small_innovation(self):
        # truth: spacing 0.9, so the edge between segments a < b measures -0.1 (b - a) in x. Six edges agree with it.
        def tries(bad):
            return {e: try_of(translation(-0.1 * (e[1] - e[0]) + (bad if e == (2, 4, False) else 0.0)))
                    for e in [(0, 4, False), (1, 4, False), (0, 3, False), (2, 4, False), (1, 3, False), (0, 2, False)]}

        def table(bad):
            t = tries(bad)
            g = lg.LoopGraph(chain_capture(tries=t), list(t), loop_omega={e: W_LOOP for e in t})
            return g, {r["edge"]: r for r in lg.leave_one_out(g)}

        _, ok = table(0.0)
        self.assertLess(max(r["weighted_m"] for r in ok.values()), 0.05)  # each predicted from the rest to a fraction of a sigma-metre
        g, rows = table(1.0)  # one edge 1 m off
        self.assertEqual(max(rows, key=lambda k: rows[k]["weighted_m"]), "2-4")
        self.assertGreater(rows["2-4"]["weighted_m"], 0.9)
        self.assertGreater(rows["2-4"]["cost_change"], 0.0)
        # the innovation is the edge's residual in the graph solved without it, in weights x sigma
        idx = g.edge_index((2, 4, False))
        s = g.solve(mask=g.mask_without((2, 4, False)))
        self.assertAlmostEqual(rows["2-4"]["weighted_m"], np.sqrt(s.factor_costs[idx]) * 0.05, places=6)

    def test_correction_interpolated_onto_nodes_and_written(self):
        cap = chain_capture(n=3)
        g = lg.LoopGraph(cap, [])
        poses = g.initial.copy()
        poses[1] = translation(1.0 + 0.2)  # segment 1 moved 0.2 m, segment 2 moved 0.6 m, segment 0 fixed
        poses[2] = translation(2.0 + 0.6)
        sol = g.solution_of(poses)
        P = g.node_poses(sol)
        # segment 0's nodes (t 0, 0.3, 0.6) go from correction 0 towards 0.2 over its 1 s; segment 2 (last): its own 0.6
        shift = P[:, 0, 3] - cap.node_pose[:, 0, 3]
        np.testing.assert_allclose(shift[:3], [0.0, 0.06, 0.12], atol=1e-12)
        np.testing.assert_allclose(shift[3:6], [0.2, 0.2 + 0.4 * 0.3, 0.2 + 0.4 * 0.6], atol=1e-12)
        np.testing.assert_allclose(shift[6:], 0.6, atol=1e-12)
        with tempfile.TemporaryDirectory() as d:
            g.write_trajectory(sol, d)
            t, rot, pos = map_quality_lib.trajectory(d)
        np.testing.assert_allclose(t, cap.node_t, atol=1e-9)
        np.testing.assert_allclose(pos, P[:, :3, 3], atol=1e-9)
        np.testing.assert_allclose(rot, P[:, :3, :3], atol=1e-12)

    def test_edge_labels_and_lists(self):
        self.assertEqual(lg.parse_edge("24-63"), (24, 63, False))
        self.assertEqual(lg.parse_edge("51-78x"), (51, 78, True))
        self.assertEqual(lg.edge_label((51, 78, True)), "51-78x")
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "edges.txt"
            p.write_text("1 2\n3 4 x  # a comment\n")
            self.assertEqual(lg.load_edges(p), [(1, 2, False), (3, 4, True)])
            (Path(d) / "loop_closure_edges.json").write_text('{"edges_kept": [{"i": 5, "j": 9, "cross_floor": true}]}')
            self.assertEqual(lg.load_edges(d), [(5, 9, True)])

    def test_missing_try_is_an_error(self):
        with self.assertRaises(KeyError):
            lg.LoopGraph(chain_capture(), [(0, 4, False)])


    def test_capture_readings_format_2_and_none_in_format_1(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "c.bin"
            p.write_bytes(capture_bytes(1))
            self.assertIsNone(lg.read_capture(p).readings)
            p.write_bytes(capture_bytes(2, [None, (0.01, (0.0, 1.0, 0.0), 1, (0.1, 0.2, 0.3), 0.4)]))
            r = lg.read_capture(p).readings
            self.assertEqual(r["W"], 2)
            self.assertEqual(list(r["present"]), [False, True])
            self.assertTrue(np.isnan(r["ratio"][0]) and np.isnan(r["move"][0]).all() and not r["rest"][0])
            self.assertEqual(r["ratio"][1], 0.01)
            self.assertEqual(list(r["weak"][1]), [0.0, 1.0, 0.0])
            self.assertTrue(r["rest"][1])
            self.assertEqual(list(r["move"][1]), [0.1, 0.2, 0.3])
            self.assertEqual(r["dur"][1], 0.4)
            self.assertEqual(list(r["t"]), [0.0, 1.0])
            p.write_bytes(capture_bytes(5, [None, None]))  # a format not known
            with self.assertRaises(ValueError):
                lg.read_capture(p)

    def test_capture_format_3_per_window_information(self):
        H0 = np.array([[1.0, 2.5, 3.0], [2.0, 5.0, 6.0], [3.0, 6.0, 9.0]])  # not symmetric: a transposed read would show
        H1 = np.diag([7.0, 8.0, 9.0])
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "c.bin"
            readings = [None, (0.01, (0.0, 1.0, 0.0), 1, (0.1, 0.2, 0.3), 0.4)]
            p.write_bytes(capture_bytes(3, readings, blocks=[[H0, H1], []]))
            c = lg.read_capture(p)
            self.assertEqual(sorted(c.tries), [(0, 1, False), (0, 2, False)])
            Hw = c.tries[(0, 1, False)]["Hw"]
            self.assertEqual(Hw.shape, (2, 3, 3))
            np.testing.assert_array_equal(Hw[0], H0)
            np.testing.assert_array_equal(Hw[1], H1)
            self.assertFalse(c.tries[(0, 2, False)]["registered"])
            self.assertEqual(c.tries[(0, 2, False)]["Hw"].shape, (0, 3, 3))
            self.assertEqual(list(c.readings["present"]), [False, True])
            # format 2 with the same tries: no Hw
            p.write_bytes(capture_bytes(2, readings, blocks=[[H0, H1], []]))
            c = lg.read_capture(p)
            self.assertNotIn("Hw", c.tries[(0, 1, False)])
            # format 3 with a damaged count of blocks
            b = bytearray(capture_bytes(3, readings, blocks=[[H0, H1], []]))
            tail = 8 + 8 + 2 * 9 * 8 + 8  # the try count, the first try's count and blocks, the second's count
            b[-tail:-tail + 8] = struct.pack("<Q", 3)
            p.write_bytes(bytes(b))
            with self.assertRaises(ValueError):
                lg.read_capture(p)

    def test_capture_format_4_fine_time_curves(self):
        readings = [None, (0.01, (0.0, 1.0, 0.0), 0, (0.1, 0.2, 0.3), 0.4)]
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "c.bin"
            p.write_bytes(capture_bytes(4, readings, blocks=[[np.eye(3), np.eye(3)]], fine_time=True))
            c = lg.read_capture(p)
            f = c.fine_time
            self.assertEqual(f["segment_poses"].shape, (1, 4, 4))
            np.testing.assert_array_equal(f["segment_poses"][0], translation(0.25))
            e = f["eligible"][0]
            self.assertEqual((e["run"], e["first"], e["last"], e["support_first"], e["support_last"], e["overlap"]), (0, 1, 1, 0, 1, False))
            self.assertEqual((e["sigma"], e["pin_end"]), (1.5, 1.4))
            np.testing.assert_array_equal(e["axis"], [0, 1, 0])
            cu = f["curves"][0]
            self.assertEqual((cu["bound"], cu["reference_patches"]), (2.25, 42))
            np.testing.assert_array_equal(cu["grid"], [-0.1, 0.0, 0.1])
            w = cu["windows"][0]
            self.assertEqual((w["training_patches"], w["held_out_patches"]), (9, 3))
            np.testing.assert_array_equal(w["training"], [0.25, 0.01, 0.25])
            self.assertEqual(list(w["training_matched"]), [0, 9, 1])
            self.assertEqual(len(w["held_out"]), 0)
            self.assertIn("Hw", c.tries[(0, 1, False)])
            # format 4 without curves, and format 3: none
            p.write_bytes(capture_bytes(4, readings, blocks=[[np.eye(3), np.eye(3)]]))
            self.assertIsNone(lg.read_capture(p).fine_time)
            p.write_bytes(capture_bytes(3, readings, blocks=[[np.eye(3), np.eye(3)]]))
            self.assertIsNone(lg.read_capture(p).fine_time)


if __name__ == "__main__":
    unittest.main()
