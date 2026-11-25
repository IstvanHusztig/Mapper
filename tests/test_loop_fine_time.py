"""Tests of tools/loop_fine_time.py on constructed numbers and a synthetic corridor; no recorded data or mapper runs required."""
import itertools
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import loop_fine_time as ft
import loop_graph as lg
import wall_pairs as wp

X = np.array([1.0, 0.0, 0.0])


def brute_profile(E, grid, coupling):
    """Every combination of offsets: the lowest cost of rule 6 (ties not resolved here)."""
    best, arg = np.inf, None
    for path in itertools.product(range(len(grid)), repeat=len(E)):
        d = np.r_[0.0, grid[list(path)], 0.0]
        cost = sum(E[w, k] for w, k in enumerate(path)) + coupling * float(np.sum(np.diff(d) ** 2))
        if cost < best - 1e-12:
            best, arg = cost, path
    return best, np.array(arg)


def profile_cost(E, grid, path, coupling):
    d = np.r_[0.0, grid[path], 0.0]
    return sum(E[w, k] for w, k in enumerate(path)) + coupling * float(np.sum(np.diff(d) ** 2))


class Pieces(unittest.TestCase):
    def test_thin_keeps_the_first_point_of_a_voxel_in_input_order(self):
        x = np.array([[0.51, 0, 0], [0.01, 0, 0], [0.02, 0, 0], [0.55, 0, 0], [-0.01, 0, 0]])
        self.assertEqual(ft.thin(x, 0.1).tolist(), [0, 1, 4])

    def test_take_spreads_the_limit_over_all(self):
        self.assertEqual(ft.take(5, 256).tolist(), [0, 1, 2, 3, 4])
        t = ft.take(1000, 256)
        self.assertEqual((len(t), t[0], t[-1]), (256, 0, 999))
        self.assertTrue(np.all(np.diff(t) > 0))

    def test_held_out_is_a_fifth_and_fixed_by_position(self):
        rng = np.random.default_rng(1)
        q = rng.uniform(-20, 20, (5000, 3))
        h = ft.held_out(q)
        self.assertTrue(0.15 < h.mean() < 0.25)
        self.assertTrue(np.array_equal(h, ft.held_out(q + 1e-9)))
        self.assertEqual(ft.held_out(np.array([[-0.5, -0.5, -0.5]])).tolist(), ft.held_out(np.array([[-0.9, -0.1, -0.3]])).tolist())

    def test_patches_of_a_wall_across_the_axis_have_its_normal(self):
        g = np.arange(0.0, 2.0, 0.05)
        yy, zz = np.meshgrid(g, g)
        wall = np.column_stack([np.full(yy.size, 3.0), yy.ravel(), zz.ravel()])
        idx, n = ft.patches(wall, X, 0.07)
        self.assertGreater(len(idx), 100)
        self.assertTrue(np.all(np.abs(n @ X) > 0.999))
        along = np.column_stack([yy.ravel(), np.full(yy.size, 1.0), zz.ravel()])  # a wall along the axis: no patch
        self.assertEqual(len(ft.patches(along, X, 0.07)[0]), 0)
        self.assertEqual(len(ft.patches(wall[:10], X, 0.07)[0]), 0)  # fewer than 16 points

    def test_offset_grid(self):
        grid, bound = ft.offset_grid(5.04)
        self.assertAlmostEqual(bound, 7.56)
        self.assertEqual((len(grid), round(grid[0], 6), round(grid[-1], 6)), (151, -7.5, 7.5))
        self.assertEqual(len(ft.offset_grid(100.0)[0]), 161)

    def test_preference_order(self):
        grid = np.arange(-2, 3) * 0.1
        self.assertEqual(ft.preference(grid).tolist(), [2, 1, 3, 0, 4])


class Profile(unittest.TestCase):
    def test_equals_brute_force(self):
        rng = np.random.default_rng(7)
        grid = np.arange(-2, 3) * 0.1
        for _ in range(20):
            E = rng.uniform(0, 0.05, (5, len(grid)))
            E[rng.integers(0, 5)] = 0.0  # an unsupported window
            path = ft.profile(E, grid)
            best, _ = brute_profile(E, grid, 0.10)
            self.assertAlmostEqual(profile_cost(E, grid, path, 0.10), best, places=12)

    def test_no_support_gives_zero(self):
        grid = np.arange(-5, 6) * 0.1
        self.assertTrue(np.all(grid[ft.profile(np.zeros((7, len(grid))), grid)] == 0.0))

    def test_unsupported_windows_follow_their_neighbours_and_the_ends_are_pinned(self):
        grid = np.arange(-20, 21) * 0.1
        E = np.zeros((9, len(grid)))
        E[4] = 0.25 * (np.abs(grid - 1.0) > 1e-9)  # one window wants +1.0 m, strongly
        d = grid[ft.profile(E, grid)]
        self.assertAlmostEqual(d[4], 1.0)
        self.assertTrue(np.all(np.diff(d[:5]) > 0) and np.all(np.diff(d[4:]) < 0))  # a ramp up and down
        self.assertLess(abs(d[0]), abs(d[1]))

    def test_ties_take_the_smaller_then_the_negative_offset(self):
        grid = np.arange(-3, 4) * 0.1
        E = np.zeros((1, len(grid)))
        E[0] = 1.0
        E[0, [1, 5]] = 0.0  # -0.2 and +0.2 cost the same
        self.assertAlmostEqual(grid[ft.profile(E, grid)][0], -0.2)
        E[0, 4] = 0.10 * 2 * (0.2 ** 2 - 0.1 ** 2)  # +0.1 now costs exactly as much as +-0.2: the smaller |d| wins
        self.assertAlmostEqual(grid[ft.profile(E, grid)][0], 0.1)


class Edges(unittest.TestCase):
    def test_classes(self):
        self.assertEqual(ft.edge_class(0.3, 0.0, 5.0, 0.2), "unconfirmed")
        self.assertEqual(ft.edge_class(0.9, 0.0, 0.15, 0.2), "agrees")
        self.assertEqual(ft.edge_class(0.9, 1.2, -0.4, 0.2), "agrees")  # beyond the limit but no larger than before
        self.assertEqual(ft.edge_class(0.9, 0.0, 0.5, 0.2), "contradicted")

    def test_time_shares(self):
        H = np.zeros((3, 3, 3))
        H[0] = np.diag([1.0, 50.0, 0.0])
        H[2] = np.diag([3.0, 0.0, 0.0])
        self.assertTrue(np.allclose(ft.time_shares(H, X), [0.25, 0.0, 0.75]))
        self.assertIsNone(ft.time_shares(H, np.array([0.0, 0.0, 1.0])))

    def test_an_edge_with_late_information_is_read_late(self):
        data, _ = corridor(jambs=(), error=lambda t: 0.0)
        c = data.cap
        j, i = 3, 12  # source segment 3 (windows 18-23), an earlier-numbered target is not needed for the reading
        edge = (i, j, False)
        c.tries[edge] = dict(C=np.eye(4), H=np.eye(6))
        s = c.segments[j]
        wins = np.arange(s["first"], s["last"] + 1)
        shift = np.linspace(1.0, 0.0, 4)  # the first pose is 1 m off, from the fourth window on nothing
        for m, w in enumerate(wins):
            data.P1[data.first[w]:data.first[w + 1], 0, 3] += shift[m] if m < 4 else 0.0
        H = np.zeros((len(wins), 3, 3))
        H[4] = H[5] = np.eye(3)  # all the information in the last two windows
        run = dict(a0=X, a=X, first=18, last=23)
        first_pose, before, after, shares = ft.read_edge(data, edge, run, wins, H, data.P1[:, :3, 3], lambda n: data.P1[n] @ lg.inv(data.P0[n]))
        self.assertAlmostEqual(first_pose, 1.0)
        self.assertAlmostEqual(before, 0.0)
        self.assertAlmostEqual(after, 0.0)
        H[0] = np.eye(3) * 2  # half of it in the first window
        _, before, _, _ = ft.read_edge(data, edge, run, wins, H, data.P1[:, :3, 3], lambda n: data.P1[n] @ lg.inv(data.P0[n]))
        self.assertAlmostEqual(before, 0.5)


class Runs(unittest.TestCase):
    def test_weak_run_with_a_bridged_window(self):
        W = 20
        rd = dict(W=W, present=np.ones(W, bool), dur=np.full(W, 0.4), ratio=np.full(W, 0.5), rest=np.zeros(W, bool),
                  weak=np.tile(X, (W, 1)), move=np.tile(0.4 * X, (W, 1)))
        rd["ratio"][5:12] = 0.01
        rd["ratio"][8] = 0.5  # one seeing window inside: bridged
        rd["move"][5:12] = 0.2 * X
        rd["move"][8] = 0.4 * X
        runs = ft.weak_runs(rd)
        self.assertEqual(len(runs), 1)
        r = runs[0]
        self.assertEqual((r["first"], r["last"], r["weak_windows"]), (5, 11, 6))
        self.assertAlmostEqual(r["T"], 2.8)
        self.assertAlmostEqual(r["s1"], 0.5 * 0.05 * 2.8 ** 2)
        self.assertAlmostEqual(r["s2"], 0.5 * 0.4 * 6)  # six windows 0.5 m/s below the pace of 1 m/s
        rd["rest"][8] = True  # a window at rest is not bridged: two runs
        self.assertEqual([(r["first"], r["last"]) for r in ft.weak_runs(rd)], [(5, 7), (9, 11)])
        rd["rest"][5:12] = True
        self.assertEqual(ft.weak_runs(rd), [])


# ---- a synthetic corridor walked out and back ----------------------------------------------------------------------------
def corridor(jambs=(3.0, 6.5, 9.0, 13.0, 17.0), error=None, folder=None, run=(13, 27)):
    """A corridor along x (walls at y = +-1) with door jambs across it, walked out (0..20 m in 20 s) and back (40..60 s), 100
    windows of 4 nodes. The windows `run` are weak along x; `error(t)` [m] is added along x to the poses after the loop
    closure. -> (Data, truth positions per node)."""
    folder = Path(folder or tempfile.mkdtemp())
    g = np.arange(0.0, 2.4, 0.06)
    xs = np.arange(-2.0, 22.0, 0.06)
    xx, zz = np.meshgrid(xs, g)
    scene = [np.column_stack([xx.ravel(), np.full(xx.size, s), zz.ravel()]) for s in (-1.0, 1.0)]
    for xj in jambs:
        for y0 in (-1.0, 0.4):
            yy, z2 = np.meshgrid(np.arange(y0, y0 + 0.6001, 0.06), g)
            scene.append(np.column_stack([np.full(yy.size, xj), yy.ravel(), z2.ravel()]))
    scene = np.concatenate(scene)
    W, per = 100, 4
    node_t = np.array([(w * 0.4 if w < 50 else 40.0 + (w - 50) * 0.4) + m * 0.1 for w in range(W) for m in range(per)])
    truth = np.zeros((W * per, 3))
    truth[:, 0] = np.where(node_t <= 20.0, node_t, 20.0 - (node_t - 40.0))
    truth[:, 2] = 1.2
    first = np.arange(0, W * per + 1, per)
    P0 = np.tile(np.eye(4), (W * per, 1, 1))
    P0[:, :3, 3] = truth
    P1 = P0.copy()
    if error is not None:
        P1[:, 0, 3] += np.array([error(t) for t in node_t])
    for w in range(W):
        n = first[w] + 1
        vis = scene[np.abs(scene[:, 0] - truth[n, 0]) <= 4.0]
        arr = np.zeros(len(vis), ft.CACHE_POINT)
        arr["xyz"] = vis - truth[n]
        arr["node"] = 1
        arr["t"] = node_t[n]
        arr.tofile(folder / f"intermediate_points_{w}.cache")
    segs = []
    for k in range(0, W, 6):
        last = min(W, k + 6) - 1
        segs.append(dict(first=k, last=last, pose=P0[first[k]].copy(), center=truth[first[k]], path_begin=0.0, path_end=0.0,
                         t_begin=node_t[first[k]], t_end=node_t[first[last + 1] - 1]))
    rd = dict(W=W, present=np.ones(W, bool), dur=np.full(W, 0.4), ratio=np.full(W, 0.5), rest=np.zeros(W, bool),
              weak=np.tile(X, (W, 1)), move=np.tile(0.4 * X, (W, 1)), t=node_t[first[:-1]])
    rd["ratio"][run[0]:run[1] + 1] = 0.01
    rd["move"][run[0]:run[1] + 1] = 0.2 * X
    settings = dict(min_distance_lidar="0.1", max_distance_lidar="40", loop_closure_segment_windows="6")
    cap = lg.Capture(settings, segs, first, P0, node_t, node_t.copy(), {}, rd)
    tr = wp.Trajectory("after", np.rint(node_t * 1e9).astype(np.int64), P1[:, :3, :3], P1[:, :3, 3])
    return ft.Data(cap, tr, folder), truth


def bump(t):
    """An error along x that grows and falls inside the weak run (5.2 .. 11.2 s), 1.2 m at its top."""
    return 1.2 * max(0.0, 1.0 - abs(t - 8.2) / 3.0)


class Corridor(unittest.TestCase):
    def test_an_error_inside_the_blind_stretch_is_taken_out_where_jambs_are_seen(self):
        data, truth = corridor(error=bump)
        runs, table, P = ft.run_rule(data, [], log=lambda *_: None)
        el = [r for r in runs if r["eligible"]]
        self.assertEqual(len(el), 1)
        r = el[0]
        self.assertEqual((r["first"], r["last"], r["sup_first"], r["sup_last"], r["status"]), (13, 27, 7, 33, "applied"))
        self.assertTrue(r["supported"].all())
        self.assertLess(r["held_after"], r["held_before"])
        own = slice(data.first[13], data.first[28])
        self.assertGreater(np.abs(data.P1[own, 0, 3] - truth[own, 0]).max(), 1.0)
        self.assertLessEqual(np.abs(P[own, 0, 3] - truth[own, 0]).max(), 0.1 + 1e-9)  # to one grid step
        outside = np.r_[0:data.first[7], data.first[34]:len(truth)]
        self.assertTrue(np.array_equal(P[outside], data.P1[outside]))
        self.assertTrue(np.array_equal(P[:, :3, :3], data.P1[:, :3, :3]))
        self.assertTrue(np.array_equal(P[:, 1:3, 3], data.P1[:, 1:3, 3]))

    def test_a_right_trajectory_stays(self):
        data, truth = corridor(error=None)
        runs, _, P = ft.run_rule(data, [], log=lambda *_: None)
        r = [r for r in runs if r["eligible"]][0]
        self.assertEqual(r["status"], "unchanged")
        self.assertTrue(np.array_equal(P, data.P1))

    def test_nothing_across_the_passage_is_not_timed(self):
        data, _ = corridor(jambs=(), error=bump)
        runs, _, P = ft.run_rule(data, [], log=lambda *_: None)
        r = [r for r in runs if r["eligible"]][0]
        self.assertEqual((r["status"], r["cause"]), ("not timed", "no surface across the passage seen by another visit"))
        self.assertFalse(r["supported"].any())
        self.assertTrue(np.array_equal(P, data.P1))

    def test_equal_jambs_at_equal_spacing_do_not_move_a_right_trajectory(self):
        data, _ = corridor(jambs=tuple(np.arange(1.0, 20.0, 2.0)), error=None)
        runs, _, P = ft.run_rule(data, [], log=lambda *_: None)
        r = [r for r in runs if r["eligible"]][0]
        self.assertGreater(r["scores"]["bound"], 2.0)  # the search reaches beyond one spacing
        self.assertLessEqual(np.abs(r["offsets"]).max(), 0.1 + 1e-9)
        self.assertLessEqual(np.abs(P[:, :3, 3] - data.P1[:, :3, 3]).max(), 0.1 + 1e-9)


if __name__ == "__main__":
    unittest.main()
