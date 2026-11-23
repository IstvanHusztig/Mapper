"""Failure-oriented verification tests; no recorded data or mapper runs required."""
import json
import math
import struct
import subprocess
import sys
import tempfile
import unittest
from decimal import Decimal
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import trajectory_seams as seams
import snapshot_trajectory as snapshot
import run_health as health
import perturbation_sweep as sweep
import honesty_report as honesty
import wall_pairs as wp


def log_text(complete=True):
    return ("window diagnostics 1: t 0.00 s, dt 0.36 s, iterations 100, delta 0, start 0 0 0, guess move 0 0 0, info ratio 0.6\n"
            "window diagnostics 1: velocity start 0 0 0, end 0 0 0\n"
            "window diagnostics 1: heading 0 deg, yaw correction first 0 last 0 deg\n"
            "gyro bias [deg/s] window 1: 0 0 0\n"
            "finished at iteration 100/100 optimizing window 1/1 with acc_distance 0\n" + ("Processing time: 1 [s]\n" if complete else ""))


def snapshot_bytes(times=(1., 2.), empty_accel=False):
    # Rotation about Z, translation (4,5,6): catches row-major/transformed-world errors.
    pose = (0., 1., 0., 0., -1., 0., 0., 0., 0., 0., 1., 0., 4., 5., 6., 1.)
    def array(rows):
        return struct.pack('<Q', len(rows)) + b''.join(struct.pack('<'+'d'*len(r), *r) for r in rows)
    block = array([pose]*len(times))*2
    block += array([(t, t+1000) for t in times])
    block += array([(0.,0.,0.)]*len(times))
    block += array([] if empty_accel else [(0.,0.,0.)]*len(times)) + struct.pack('<27d', *([0.]*27))
    return struct.pack('<8sIQQ', b'DMSNAP01', 1, 123, 1) + block + struct.pack('<18d', *(pose+(0.,0.)))


class VerificationTests(unittest.TestCase):
    def test_sparse_shipped_evidence_options(self):
        import tomllib
        text=(ROOT/'settings/livox-vertical.toml').read_text(encoding='utf-8')
        for key in ('log_window_diagnostics','save_odometry_snapshot'):
            text=sweep.evidence_key(text,key)
        parsed=tomllib.loads(text)
        self.assertTrue(parsed['lidar_odometry_debug_info']['save_odometry_snapshot'])
        self.assertTrue(parsed['lidar_odometry_debug_info']['log_window_diagnostics'])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_strict_seams(self):
        t = Decimal('1794269921340994560')
        ref = [(t,0.,0.,0.), (t+1,1.,0.,0.)]
        for bad in (ref[:1], [(t+1,0.,0.,0.), (t+2,1.,0.,0.)], [ref[0],ref[0]], [(t,math.nan,0.,0.),ref[1]]):
            with self.assertRaises(ValueError):
                seams.compare(ref,bad,[.05])
        res = [(r[0],r[1]+.2,r[2],r[3]) for r in ref]
        result = seams.compare(ref,res,[.05])
        self.assertAlmostEqual(result['max_diff_m'],.2)
        self.assertAlmostEqual(result['seconds_above_005'],1e-9)

    def test_final_chunk_transform_and_exact_time(self):
        (self.root/'session_poses.mrp').write_text('1\nscan_lio_0.laz\n0 -1 0 10\n1 0 0 20\n0 0 1 30\n0 0 0 1\n')
        (self.root/'trajectory_lio_0.csv').write_text(' '.join(['h']*13)+'\n1794269921340994561 1 0 0 2 0 1 0 3 0 0 1 4\n')
        rows = seams.load(self.root)
        self.assertEqual(rows, [(Decimal('1794269921340994561'),7.,22.,34.)])

    def test_snapshot_world_frame_and_corruption(self):
        p=self.root/'odometry_snapshot.bin'; good=snapshot_bytes(); p.write_bytes(good)
        self.assertEqual(snapshot.load_snapshot(p),[(Decimal(1e9),4.,5.,6.),(Decimal(2e9),4.,5.,6.)])
        p.write_bytes(snapshot_bytes(empty_accel=True))
        self.assertEqual(len(snapshot.load_snapshot(p)),2)
        variants=[good[:n] for n in (0,8,12,28,40,len(good)-1)]
        variants += [good+b'x', b'BADMAGIC'+good[8:], good[:8]+struct.pack('<I',2)+good[12:],
                     good[:28]+struct.pack('<Q',2**63)+good[36:],snapshot_bytes((1.,1.)), snapshot_bytes((2.,1.)),snapshot_bytes((1.,math.nan))]
        for bad in variants:
            p.write_bytes(bad)
            with self.assertRaises((ValueError, ArithmeticError)):
                snapshot.load_snapshot(p)

    def test_missing_diagnostics_never_success(self):
        p=self.root/'run.log'
        p.write_text(log_text().splitlines()[0]+'\nProcessing time: 1 [s]\n')
        r=health.analyse(p)
        self.assertTrue({'bend','speed','bias'}.issubset(r['missing_required']))
        proc=subprocess.run([sys.executable,'-B',str(ROOT/'tools/run_health.py'),str(p)],capture_output=True,text=True)
        self.assertEqual(proc.returncode,2)
        self.assertIn('NOT MEASURED',proc.stdout)
        self.assertNotIn('result: PASS',proc.stdout)
        p.write_text(log_text(False))
        self.assertFalse(health.analyse(p)['completed'])
        p.write_text(log_text())
        self.assertEqual(health.analyse(p)['missing_required'],[])
        self.assertEqual(health.analyse(p)['coverage']['chi2'],0)
        p.write_text(log_text().replace('window 1/1','window 1/2'))
        self.assertIn('expected total window coverage NOT ESTABLISHED',health.analyse(p)['missing_required'])

    def test_duplicate_and_malformed_headers(self):
        p=self.root/'run.log'
        for text in (log_text()+log_text().splitlines()[0],log_text()+'window diagnostics 2: t nan s\n',log_text().replace('diagnostics 1:', 'diagnostics 2:')):
            p.write_text(text)
            with self.assertRaises(ValueError): health.analyse(p)

    def test_missing_conditional_uncertainty(self):
        p=self.root/'run.log';p.write_text(log_text().replace('info ratio 0.6','info ratio 0.2'))
        self.assertEqual(health.analyse(p)['uncertainty_missing'],[1])
        for suffix in ('window diagnostics 2: heading 0 deg, yaw correction first 0 last 0 deg\n',
                       'window diagnostics 1: weak axis malformed\n',
                       'gyro bias [deg/s] window 1: 0 0 0\n'):
            p.write_text(log_text()+suffix)
            with self.assertRaises(ValueError): health.analyse(p)

    def test_canceled_run_refused_even_with_exit_marker(self):
        p=self.root/'run.log';p.write_text(log_text()+'calculations canceled for TIMESTAMP\n')
        with self.assertRaisesRegex(ValueError,'incomplete run diagnostics|failure evidence'):
            sweep.measure(self.root,p,None,None)

    def test_provenance_refuses_mutations(self):
        recording=self.root/'input';recording.mkdir()
        (recording/'imu.csv').write_bytes(b'imu');(recording/'lidar.laz').write_bytes(b'points')
        exe=self.root/'mapper.exe';exe.write_bytes(b'exe')
        folder=self.root/'output';folder.mkdir();(folder/'cloud.laz').write_bytes(b'cloud')
        toml=self.root/'run.toml';toml.write_text('config')
        log=self.root/'run.log';log.write_text('log')
        identity=sweep.identity(recording,exe,'config')
        record={'identity':identity,'exit':0,'evidence_complete':True,'outputs':sweep.output_identity(folder,toml,log)}
        sweep.validate_resume(record,identity,folder,toml,log)
        with self.assertRaises(ValueError): sweep.validate_resume({},identity,folder,toml,log)
        with self.assertRaises(ValueError): sweep.validate_resume(record,sweep.identity(recording,exe,'changed'),folder,toml,log)
        exe.write_bytes(b'other exe')
        with self.assertRaises(ValueError): sweep.validate_resume(record,sweep.identity(recording,exe,'config'),folder,toml,log)
        (folder/'cloud.laz').write_bytes(b'changed cloud')
        with self.assertRaises(ValueError): sweep.validate_resume(record,identity,folder,toml,log)

    def test_sweep_failure_retains_files_and_exits_nonzero(self):
        recording=self.root/'input';recording.mkdir()
        (recording/'imu.csv').write_bytes(b'imu');(recording/'lidar.laz').write_bytes(b'points')
        exe=self.root/'mapper.exe';exe.write_bytes(b'exe')
        toml=self.root/'settings.toml';toml.write_text('log_window_diagnostics = false\nsave_odometry_snapshot = false\n')
        out=self.root/'runs'
        def fail(command, **kwargs):
            (Path(command[-1])/'cloud.laz').write_bytes(b'preserve me')
            return 7
        argv=['sweep',str(recording),str(toml),str(out),'--exe',str(exe),'--only','base','--profile','short']
        with patch.object(sys,'argv',argv),patch.object(sweep.subprocess,'call',side_effect=fail),patch.object(sweep.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'','')):
            self.assertEqual(sweep.main(),2)
        self.assertEqual((out/'base/cloud.laz').read_bytes(),b'preserve me')
        self.assertEqual(json.loads((out/'results.json').read_text())['base']['exit'],7)
        with patch.object(sys,'argv',argv),patch.object(sweep.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'','')):
            with self.assertRaises(ValueError): sweep.main()

    def test_seven_member_sweep_and_verified_resume(self):
        recording=self.root/'input';recording.mkdir()
        (recording/'imu.csv').write_bytes(b'imu');(recording/'lidar.laz').write_bytes(b'points')
        exe=self.root/'mapper.exe';exe.write_bytes(b'exe')
        toml=self.root/'settings.toml'
        toml.write_text('log_window_diagnostics = false\nsave_odometry_snapshot = false\nimu_acceleration_span_s = 0.01\ndecimation = 0.03\n'+''.join(f'{k} = 0.0001\n' for k in sweep.EDGE_KEYS))
        out=self.root/'runs'
        def produce(command, stdout, **kwargs):
            folder=Path(command[-1])
            (folder/'odometry_snapshot.bin').write_bytes(snapshot_bytes())
            (folder/'scan_lio_merged.laz').write_bytes(b'cloud')
            (folder/'session_poses.mrp').write_text('1\nscan_lio_0.laz\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n')
            (folder/'trajectory_lio_0.csv').write_text(' '.join(['h']*13)+'\n'+''.join(f'{t} 1 0 0 4 0 1 0 5 0 0 1 6\n' for t in (1000000000,2000000000)))
            stdout.write(log_text().replace('start 0 0 0, guess','start 4 5 6, guess'))
            return 0
        argv=['sweep',str(recording),str(toml),str(out),'--exe',str(exe),'--profile','full','--reference-route-m','20','--reference-id','frozen-fixture']
        with patch.object(sys,'argv',argv),patch.object(sweep.subprocess,'call',side_effect=produce) as launch,patch.object(sweep.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'','')):
            self.assertEqual(sweep.main(),0)
            self.assertEqual(launch.call_count,7)
            launch.reset_mock()
            self.assertEqual(sweep.main(),0)
            launch.assert_not_called()
        results=json.loads((out/'results.json').read_text())
        self.assertEqual(results['dec_plus']['stability']['odometry']['limit_m'],.06)
        self.assertEqual(results['dec_plus']['stability']['final']['max_diff_m'],0)
        self.assertEqual(len(list(out.glob('*/scan_lio_merged.laz'))),7)

    def test_build_refuses_a_release_folder_configured_as_debug(self):
        import kit_lib
        with tempfile.TemporaryDirectory() as d:
            tree = Path(d) / 'out/build/x64-Release'
            tree.mkdir(parents=True)
            cache = tree / 'CMakeCache.txt'
            with patch.object(kit_lib, 'ROOT', Path(d)):
                self.assertIsNone(kit_lib.build_type_mismatch('x64-Release'))  # not configured yet: cmake will do it
                cache.write_text('CMAKE_BUILD_TYPE:STRING=Debug\n')
                self.assertIn('needs Release', kit_lib.build_type_mismatch('x64-Release'))
                cache.write_text('CMAKE_BUILD_TYPE:STRING=Release\n')
                self.assertIsNone(kit_lib.build_type_mismatch('x64-Release'))


# window_features.csv columns before consistency-harness (no prepared_*, handed_*, rotation rate, start acceleration, moves)
OLD_FEATURE_COLUMNS = ("window,time_s,duration_s,iterations,delta,lm_factor,at_rest,turn_window,acc_term,along_x,along_y,"
                       "velocity_start_x,velocity_start_y,velocity_start_z,velocity_end_x,velocity_end_y,velocity_end_z,"
                       "information_ratio,velocity_chi2,stop").split(",")
NEW_FEATURE_COLUMNS = (["window", "at_rest", "acc_term", "information_ratio", "velocity_chi2", "handoff",
                        "rotation_rate_mean_deg_s"] + [f"velocity_start_{a}" for a in "xyz"] +
                       [f"prepared_velocity_prior_{a}_mps" for a in "xyz"] + [f"start_acceleration_{a}_mps2" for a in "xyz"] +
                       [f"{m}{s}" for m in ("move_start_velocity", "move_joint", "move_gyro_bias") for s in ("", "_reason", "_reduction")])


def write_features(path, columns, rows):
    import csv
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in columns})
    return path


def synthetic_features():
    """Nine windows with figures worked out by hand (see test_honesty_report_known_figures).
    Window 1: no accelerometer term; window 6: no prior (out of the move mask). Prior mean zero, so d = velocity_start,
    along x only: windows 2,3,4,5,7,8,9 -> d_x = 1,-1,2,-2,3,-3,4."""
    d_x = {2: 1, 3: -1, 4: 2, 5: -2, 6: 9, 7: 3, 8: -3, 9: 4}
    rate = {2: 10, 3: 20, 4: 30, 5: 40, 6: 1000, 7: 50, 8: 60, 9: 70}
    acc_x = {2: -7, 3: 6, 4: 5, 5: 4, 6: 1000, 7: 3, 8: 2, 9: 1}
    ratio = {2: 0.1, 3: 0.2, 4: 0.44, 5: 0.45, 6: 0.9, 7: 0.9, 8: 0.9, 9: 0.9}
    move = {2: 1, 3: 2, 4: 3, 5: 4, 7: 5, 8: 6, 9: 7}
    rows = [{"window": 1, "acc_term": 0, "at_rest": 1, "handoff": "done", "velocity_start_x": 0, "velocity_start_y": 0,
             "velocity_start_z": 0, "move_start_velocity_reason": "absent", "move_joint_reason": "absent",
             "move_gyro_bias_reason": "absent", "information_ratio": 0.1, "rotation_rate_mean_deg_s": 5}]
    for w in range(2, 10):
        r = {"window": w, "acc_term": 1, "at_rest": 1 if w in (2, 3) else 0, "handoff": "done" if w < 9 else "cancelled",
             "velocity_start_x": d_x[w], "velocity_start_y": 0, "velocity_start_z": 0,
             "rotation_rate_mean_deg_s": rate[w], "start_acceleration_x_mps2": acc_x[w], "start_acceleration_y_mps2": 0,
             "start_acceleration_z_mps2": 0, "information_ratio": ratio[w], "velocity_chi2": w - 1,
             "move_joint_reason": "absent", "move_gyro_bias_reason": "reduction below 0.5"}
        if w != 6:
            r.update({"prepared_velocity_prior_x_mps": 0, "prepared_velocity_prior_y_mps": 0, "prepared_velocity_prior_z_mps": 0,
                      "move_start_velocity": move[w], "move_start_velocity_reason": "measured",
                      "move_start_velocity_reduction": 3})
        else:
            r.update({"move_start_velocity_reason": "last step above 1 % of prior sigma"})
        if w == 9:
            r.update({"move_gyro_bias": 0.5, "move_gyro_bias_reason": "measured", "move_gyro_bias_reduction": 0.75})
        rows.append(r)
    return rows


class HonestyReportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.new = write_features(self.root / "new_window_features.csv", NEW_FEATURE_COLUMNS, synthetic_features())
        old_rows = [{"window": w, "acc_term": 1, "at_rest": 0, "velocity_start_x": 1, "velocity_start_y": 0, "velocity_start_z": 0,
                     "information_ratio": 0.3, "velocity_chi2": 2.0 * w} for w in (1, 2, 3)]
        self.old = write_features(self.root / "old_window_features.csv", OLD_FEATURE_COLUMNS, old_rows)

    def test_honesty_report_old_table_is_unavailable(self):
        cells = honesty.figure_cells(honesty.Table(self.old))
        available = {"windows", "all windows", "weak windows"}  # the only figures whose columns predate the change
        for label, cell in cells.items():
            if label in available:
                self.assertNotEqual(cell, honesty.UNAVAILABLE, label)
            else:
                self.assertEqual(cell, honesty.UNAVAILABLE, label)  # velocity_start alone is never turned into a move
        self.assertEqual(cells["windows"], "3")
        self.assertEqual(cells["all windows"], "n 3/3: median 4, p95 5.8, max 6")

    def test_honesty_report_known_figures(self):
        c = honesty.figure_cells(honesty.Table(self.new))
        self.assertEqual(c["hand-off"], "cancelled 1, done 8")
        self.assertEqual(c["coverage"], "n 7/9")
        # norms 1,1,2,2,3,3,4: median 2; p95 at rank 0.95*6 = 5.7 -> 3 + 0.7 * 1 = 3.7
        self.assertEqual(c["median |d|"], "2 (p95 3.7, max 4)")
        self.assertEqual((c["median |d_x|"], c["median |d_y|"], c["median |d_z|"]), ("2", "0", "0"))
        # rates of the mask 10..70 (window 6's 1000 is out): cuts 25, 40, 55
        self.assertEqual(c["|d| rotation-rate Q1"], "1 (n 2; 10..20)")
        self.assertEqual(c["|d| rotation-rate Q2"], "2 (n 2; 30..40)")
        self.assertEqual(c["|d| rotation-rate Q3"], "3 (n 1; 50..50)")
        self.assertEqual(c["|d| rotation-rate Q4"], "3.5 (n 2; 60..70)")
        # |start acceleration| 7,6,5,4,3,2,1 (window 2's -7 is a norm of 7): cuts 2.5, 4, 5.5
        self.assertEqual(c["|d| start-acc Q1"], "3.5 (n 2; 1..2)")
        self.assertEqual(c["|d| start-acc Q2"], "2.5 (n 2; 3..4)")
        self.assertEqual(c["|d| start-acc Q3"], "2 (n 1; 5..5)")
        self.assertEqual(c["|d| start-acc Q4"], "1 (n 2; 6..7)")
        # d_x: mean 4/7, deviations *7: 3,-11,10,-18 | 17,-25,24 (window 6 breaks the pairs 5-6, 6-7)
        # sum dev^2 = 2044/49, pairs 3*-11 + -11*10 + 10*-18 + 17*-25 + -25*24 = -1348 (/49): r1 = -1348/2044
        series = list(zip((2, 3, 4, 5, 7, 8, 9), (1, -1, 2, -2, 3, -3, 4)))
        r, pairs = honesty.lag1_autocorrelation(series)
        self.assertAlmostEqual(r, -1348 / 2044, places=12)
        self.assertEqual(pairs, 5)
        self.assertEqual(c["lag-1 autocorr d_x"], "-0.6595 (5 pairs)")
        self.assertEqual(c["lag-1 autocorr d_y"], "- (5 pairs)")  # zero variance: no value
        # |d|: mean 16/7, deviations *7: -9,-9,-2,-2 | 5,5,12; sum dev^2 364, pairs 81+18+4+25+60 = 188
        self.assertEqual(c["lag-1 autocorr |d|"], f"{188 / 364:.4g} (5 pairs)")
        self.assertEqual(c["median |d| at rest"], "1 (n 2)")
        self.assertEqual(c["median |d| moving"], "3 (n 5)")
        # q measured on 7 of 9: 1..7, median 4, p95 at rank 5.7 -> 6.7
        self.assertEqual(c["start velocity (n=3)"], "n 7/9: median 4, p95 6.7, max 7")
        self.assertEqual(c["start velocity: not measured"], "absent 1, last step above 1 % of prior sigma 1")
        self.assertEqual(c["start velocity: reduction n - tr"], "3 (n 7)")
        self.assertEqual(c["start velocity + offset (n=6)"], "n 0/9")
        self.assertEqual(c["start velocity + offset: not measured"], "absent 9")
        self.assertEqual(c["start velocity + offset: reduction n - tr"], "n 0")
        self.assertEqual(c["gyro bias (n=3)"], "n 1/9: median 0.5, p95 0.5, max 0.5")
        self.assertEqual(c["gyro bias: not measured"], "absent 1, reduction below 0.5 7")
        # chi2 1..8 on windows 2..9; weak = information ratio < 0.45: windows 2,3,4 (0.45 is not weak)
        self.assertEqual(c["all windows"], "n 8/9: median 4.5, p95 7.65, max 8")
        self.assertEqual(c["weak windows"], "n 3/9: median 2, p95 2.9, max 3")

    def test_honesty_report_side_by_side(self):
        import contextlib
        import io
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = honesty.main([str(self.new), str(self.old), "--label", "NEWRUN", "--label", "OLDRUN"])
        self.assertEqual(code, 0)
        lines = out.getvalue().splitlines()
        header = next(l for l in lines if "NEWRUN" in l and "OLDRUN" in l)
        self.assertLess(header.index("NEWRUN"), header.index("OLDRUN"))
        coverage = next(l for l in lines if l.strip().startswith("coverage"))
        self.assertLess(coverage.index("n 7/9"), coverage.index("unavailable"))
        chi2 = next(l for l in lines if l.strip().startswith("all windows"))
        self.assertIn("n 8/9", chi2)
        self.assertIn("n 3/3", chi2)
        self.assertTrue(any(l.startswith("Definitions") for l in lines))
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            honesty.main([str(self.new), str(self.old), "--label", "only one"])
        bad = self.root / "bad.csv"
        bad.write_text("a,b\n1,2\n", encoding="utf-8")
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(honesty.main([str(bad)]), 2)



def _yaw(a):
    c, s_ = math.cos(a), math.sin(a)
    return [[c, -s_, 0.], [s_, c, 0.], [0., 0., 1.]]


class WallPairsTests(unittest.TestCase):
    """A synthetic wall (the line y = 2, two visits 30 s apart, the second 0.05 m further out) seen by a sensor that
    walks along x with a turning heading; no cloud files, no recording."""

    def setUp(self):
        import numpy as np
        self.np = np
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.ns = np.arange(0, 60_000_000_000, 100_000_000, dtype=np.int64) + 50_000_000   # a node every 0.1 s
        tt = self.ns * 1e-9
        self.rot = np.array([_yaw(0.1 * t) for t in tt])
        self.pos = np.column_stack([tt, np.zeros_like(tt), np.zeros_like(tt)])
        self.origin = wp.Trajectory("origin", self.ns, self.rot, self.pos)
        rng = np.random.default_rng(3)
        t1, t2 = np.linspace(10.02, 14.02, 40), np.linspace(40.02, 44.02, 40)
        self.t = np.r_[t1, t2]
        self.world = np.column_stack([rng.uniform(0, 0.5, 80), np.r_[np.full(40, 2.0), np.full(40, 2.05)], np.full(80, 0.5)])
        self.table = self.root / "wall_precision_+0.50.csv"
        self.table.write_text("piece,x,y,dir_deg,length_m,points,pairs,visit_offsets_m,visit_times_s,plan_dist_m\n"
                              "7,0.25,2.0,0.0,0.5,80,1,+0.000;+0.050,10.0-14.0;40.0-44.0,0.01\n")

    def freeze(self, **kw):
        out = self.root / "set.npz"
        T = self.ns * 1e-9
        with patch.object(wp, "_read_slice", lambda *a: (self.world, self.t)), \
                patch("map_quality_lib.trajectory", lambda r: (T, self.rot, self.pos)):
            self.meta = wp.freeze(str(self.root), str(self.table), str(out), log=lambda *a: None, **kw)
        return wp.WallSet(out)

    def trajectory_patch(self, **trs):
        return patch.object(wp, "load_trajectory", lambda spec: trs[spec])

    def moved(self, name, shift_after=None, rigid=None, ns_offset=0):
        np = self.np
        rot, pos = self.rot.copy(), self.pos.copy()
        if rigid is not None:
            q, v = np.array(_yaw(rigid)), np.array([3., -2., 1.])
            rot, pos = q @ rot, pos @ q.T + v
        if shift_after is not None:
            pos[self.ns * 1e-9 >= 30., 1] += shift_after
        return wp.Trajectory(name, self.ns + ns_offset, rot, pos)

    def test_frozen_result_reproduces_its_cloud(self):
        ws = self.freeze()
        self.assertEqual((ws.meta["pieces"], ws.meta["visits"], ws.meta["pairs"], ws.meta["returns"]), (1, 2, 1, 80))
        self.assertEqual(ws.meta["selection"], "all pieces with visits")
        self.assertLess(self.np.abs(wp.reproduction_error(ws, self.origin)).max(), 1e-9)
        self.assertLess(ws.meta["max_round_trip_m"], 1e-9)
        self.assertAlmostEqual(float(wp.pair_gaps(ws, self.origin)[0]), 0.05, places=9)

    def test_rigid_move_changes_nothing_and_a_shifted_visit_shows_its_shift(self):
        ws = self.freeze()
        g0 = wp.pair_gaps(ws, self.origin)
        self.assertLess(abs(wp.pair_gaps(ws, self.moved("rigid", rigid=0.7))[0] - g0[0]), 1e-9)
        self.assertAlmostEqual(float(wp.pair_gaps(ws, self.moved("shift", shift_after=0.07))[0]), 0.12, places=9)
        self.assertAlmostEqual(float(wp.pair_gaps(ws, self.moved("shift", shift_after=-0.05))[0]), 0.0, places=9)

    def test_measure_table_crossings_and_refusal(self):
        self.freeze()
        out = []
        path = self.root / "set.npz"
        with self.trajectory_patch(a=self.origin, b=self.moved("b", shift_after=0.07), c=self.moved("c", ns_offset=1)):
            wp.measure(str(path), ["a", "b"], threshold=0.08, before=20., after=30., log=out.append)
            text = "\n".join(out)
            self.assertIn("not accuracy", text)
            self.assertIn("all pairs (1 pairs)", text)
            self.assertIn("before 20 s / after 30 s (1 pairs)", text)
            self.assertIn("0.050 / 0.050 / 0", text)
            self.assertIn("0.120 / 0.120 / 1", text)
            self.assertIn("origin -> b: 1 cross above, 0 fall below", text)
            with self.assertRaisesRegex(ValueError, "node times differ"):
                wp.measure(str(path), ["a", "c"], log=out.append)
        self.assertEqual(wp.main(["measure", str(path), str(self.root / "missing")]), 2)

    def test_plan_walls_selection_and_table_mismatch(self):
        self.freeze(plan_walls=True)
        self.assertIn("plan walls", self.meta["selection"])
        self.table.write_text(self.table.read_text().replace(",0.01\n", ",0.5\n"))
        with self.assertRaisesRegex(ValueError, "no pair survived"):
            self.freeze(plan_walls=True)
        # a table offset the cloud does not show: the pair is left out
        self.table.write_text(self.table.read_text().replace("+0.050", "+0.090"))
        with self.assertRaisesRegex(ValueError, "no pair survived"):
            self.freeze()

    def test_node_of_tie_goes_to_the_node_before(self):
        np = self.np
        nodes = np.array([1_000_000_000, 2_000_000_000, 3_000_000_000])
        self.assertEqual(wp.node_of(np.array([1.5, 2.0, 2.0000000004, 3.2]), nodes).tolist(), [0, 0, 0, 2])
        self.assertEqual(wp.node_of(np.array([0.5]), nodes).tolist(), [-1])

    def test_snapshot_trajectory_keeps_the_rotation(self):
        p = self.root / "odometry_snapshot.bin"
        p.write_bytes(snapshot_bytes())
        tr = wp.load_trajectory(str(p))
        self.assertEqual(tr.name, "odometry_snapshot")
        self.assertEqual(tr.ns.tolist(), [1_000_000_000, 2_000_000_000])
        self.assertEqual(self.np.round(tr.rot[0]).tolist(), [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.assertEqual(tr.pos[1].tolist(), [4., 5., 6.])


class RunHealthTimingTests(unittest.TestCase):
    STAGES = "stage durations: loading 12.5 s, odometry 400 s, loop closure 20.25 s, consistency n/a s, saving 3 s\n"

    def run_log(self, extra):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "run.log"
            p.write_text(log_text() + extra)
            return health.analyse(p)

    def rows(self, r):
        import argparse
        ap = argparse.ArgumentParser()
        health.add_limits(ap)
        return {row[0]: row for row in health.judge(r, ap.parse_args([]))}

    def test_zero_windows_at_the_limit_is_independent_of_load(self):
        r = self.run_log(self.STAGES + "windows stopped at the time limit: 0\n")
        self.assertEqual(r["time_limit_windows"], 0)
        self.assertEqual(r["stage_durations"]["odometry"], 400.0)
        self.assertIsNone(r["stage_durations"]["consistency"])
        rows = self.rows(r)
        self.assertIn("independent of the machine's load", rows["windows stopped at the time limit"][1])
        self.assertIn("loop closure 20.25 s", rows["stage durations"][1])
        self.assertIn("consistency n/a", rows["stage durations"][1])

    def test_a_count_above_zero_is_load_dependent(self):
        row = self.rows(self.run_log(self.STAGES + "windows stopped at the time limit: 3\n"))["windows stopped at the time limit"]
        self.assertIn("depends on the machine's load", row[1])
        self.assertNotIn("independent", row[1])
        self.assertNotEqual(row[3], "TRIGGER")

    def test_missing_lines_are_not_measured_and_do_not_change_the_exit_code(self):
        r = self.run_log("")
        rows = self.rows(r)
        self.assertEqual(rows["windows stopped at the time limit"][3], "NOT MEASURED")
        self.assertEqual(rows["stage durations"][3], "NOT MEASURED")
        self.assertEqual(r["missing_required"], [])

    def test_the_window_verdict_summary_gives_the_count(self):
        summary = ("window verdict summary: iterations stopped: converged 0, iteration limit 1016, time limit {}, forced 0; "
                   "rejected steps 3, steps not applied 0, big-jump nodes 0\n")
        self.assertEqual(self.run_log(summary.format(0))["time_limit_windows"], 0)
        self.assertEqual(self.run_log(summary.format(2))["time_limit_windows"], 2)
        # the dedicated line wins over the summary
        self.assertEqual(self.run_log(summary.format(2) + "windows stopped at the time limit: 0\n")["time_limit_windows"], 0)

    def test_malformed_or_duplicate_lines_are_errors(self):
        for extra in ("windows stopped at the time limit: many\n", "windows stopped at the time limit: 0\nwindows stopped at the time limit: 1\n",
                      "stage durations: loading 1 s, odometry 2 s\n", self.STAGES * 2):
            with self.assertRaises(ValueError):
                self.run_log(extra)


if __name__=='__main__':
    unittest.main()
