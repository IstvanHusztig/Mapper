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
import cloud_from_trajectory as cft
import contacts_to_wall_pairs as c2w
import landmark_check as lm


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


class CloudFromTrajectoryTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np = np
        self.T = np.array([0.0, 1.0, 2.0])
        rot = lambda a: np.array([[np.cos(a), -np.sin(a), 0.], [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])
        self.R = np.stack([rot(0.0), rot(0.3), rot(-0.2)])
        self.P = np.array([[0., 0., 0.], [1., 2., 0.], [3., -1., 0.5]])
        self.points = np.array([[1., 0., 0.], [2., 2., 1.], [4., -1., 0.5], [3.5, -0.5, 0.]])
        self.times = np.array([0.5, 1.2, 2.5, 1.99])

    def test_the_same_trajectory_leaves_every_point_where_it_is(self):
        tr = (self.T, self.R, self.P)
        self.assertTrue(self.np.allclose(cft.carry(self.points, self.times, tr, tr), self.points, atol=1e-12))

    def test_a_rigid_move_of_the_trajectory_moves_the_points_with_it(self):
        np = self.np
        a = 0.4
        M = np.array([[np.cos(a), -np.sin(a), 0.], [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])
        d = np.array([5., -2., 1.])
        moved = (self.T, np.einsum("ij,njk->nik", M, self.R), self.P @ M.T + d)
        y = cft.carry(self.points, self.times, (self.T, self.R, self.P), moved)
        self.assertTrue(np.allclose(y, self.points @ M.T + d, atol=1e-12))

    def test_a_point_belongs_to_the_last_node_at_or_before_its_time(self):
        np = self.np
        shifted = self.P.copy()
        shifted[1] += [0., 0., 1.]  # only node 1 moves
        y = cft.carry(self.points, self.times, (self.T, self.R, self.P), (self.T, self.R, shifted))
        self.assertEqual(np.round(y[:, 2] - self.points[:, 2], 12).tolist(), [0., 1., 0., 1.])


class _WalkMixin:
    """A sensor walking along x with a turning heading, a node every 0.1 s (as the wall pairs tests)."""

    def walk(self):
        import numpy as np
        self.np = np
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.ns = np.arange(0, 60_000_000_000, 100_000_000, dtype=np.int64) + 50_000_000
        tt = self.ns * 1e-9
        self.rot = np.array([_yaw(0.1 * t) for t in tt])
        self.pos = np.column_stack([tt, np.zeros_like(tt), np.zeros_like(tt)])
        self.origin = wp.Trajectory("origin", self.ns, self.rot, self.pos)

    def moved(self, name, shift_after=None, rigid=None, ns_offset=0, shift_all=None):
        np = self.np
        rot, pos = self.rot.copy(), self.pos.copy()
        if rigid is not None:
            q, v = np.array(_yaw(rigid)), np.array([3., -2., 1.])
            rot, pos = q @ rot, pos @ q.T + v
        if shift_after is not None:
            pos[self.ns * 1e-9 >= 30., 1] += shift_after
        if shift_all is not None:
            pos[:, 1] += shift_all
        return wp.Trajectory(name, self.ns + ns_offset, rot, pos)

    def trajectory_patch(self, **trs):
        return patch.object(wp, "load_trajectory", lambda spec: trs[spec])


class WallPairsUnionTests(_WalkMixin, unittest.TestCase):
    """Two maps of one walk. Map A shows the wall y = 2 (visits at 10-14 s and 40-44 s, 0.05 m apart) and a second wall
    (y = 5, visits at 20-24 s and 50-54 s) that A has doubled by 0.3 m, more than the 0.2 m admission across the line, so
    A's table cannot hold it. Map B shows both walls with 0.05 m between their visits."""

    def setUp(self):
        self.walk()
        np = self.np
        rng = np.random.default_rng(5)
        t = [np.linspace(10.02, 14.02, 40), np.linspace(40.02, 44.02, 40), np.linspace(20.02, 24.02, 40), np.linspace(50.02, 54.02, 40)]
        x = [rng.uniform(0, 0.5, 40) for _ in range(4)]
        col = lambda xs, y: np.column_stack([xs, np.full(40, y), np.full(40, 0.5)])
        self.cloud = {"A": (np.vstack([col(x[0], 2.0), col(x[1], 2.05), col(x[2], 5.0), col(x[3], 5.3)]), np.concatenate(t)),
                      "B": (np.vstack([col(x[0], 2.01), col(x[1], 2.06), col(x[2], 5.0), col(x[3], 5.05)]), np.concatenate(t))}
        head = "piece,x,y,dir_deg,length_m,points,pairs,visit_offsets_m,visit_times_s,plan_dist_m\n"
        tables = {"A": head + "7,0.25,2.0,0.0,0.5,80,1,+0.000;+0.050,10.0-14.0;40.0-44.0,0.01\n"
                              "8,0.25,5.0,0.0,0.5,80,1,+0.000;+0.050,20.0-24.0;50.0-54.0,0.01\n",
                  "B": head + "3,0.25,2.01,0.0,0.5,80,1,+0.000;+0.050,10.0-14.0;40.0-44.0,0.01\n"
                              "4,0.25,5.0,0.0,0.5,80,1,+0.000;+0.050,20.0-24.0;50.0-54.0,0.01\n"}
        for k, v in tables.items():
            (self.root / f"{k}.csv").write_text(v)
        self.out = self.root / "union.npz"

    def run_freeze(self, specs, out=None, ns_b=None, **kw):
        T = self.ns * 1e-9
        def traj(result):
            return ((ns_b * 1e-9) if (ns_b is not None and str(result).endswith("B")) else T), self.rot, self.pos
        specs = [s.replace("@", str(self.root) + "/") for s in specs]
        with patch.object(wp, "_read_slice", lambda r, h, ht: self.cloud[str(r)[-1]]), patch("map_quality_lib.trajectory", traj):
            return wp.freeze_union(specs, str(out or self.out), log=lambda *a: None, height=0.5, **kw)

    @staticmethod
    def specs(*keys):
        return [f"{k}=@{k}=@{k}.csv" for k in keys]

    def test_the_same_wall_in_two_maps_is_counted_once_and_a_wall_only_one_map_admits_is_kept(self):
        meta = self.run_freeze(self.specs("A", "B"))
        ws = wp.WallSet(self.out)
        self.assertEqual([o["mismatched_pieces"] for o in meta["origins"]], [1, 0])   # A cannot show its second wall
        self.assertEqual(meta["pairs"], 2)                        # wall 1 once (found in both), wall 2 from B only
        self.assertEqual(sorted(ws.pair_origin_mask.tolist()), [2, 3])
        self.assertEqual(ws.origin_labels.tolist(), ["A", "B"])
        self.assertEqual([o["pairs_found"] for o in meta["origins"]], [1, 2])
        self.assertIn("union of 2 results", meta["selection"])
        self.assertIn("A = ", meta["origin_result"])

    def test_a_gap_the_first_map_cannot_show_is_measured_on_another_trajectory(self):
        self.run_freeze(self.specs("A", "B"))
        out = []
        with self.trajectory_patch(a=self.origin, b=self.moved("b", shift_after=0.3)):
            wp.measure(str(self.out), ["a", "b"], threshold=0.08, severe=0.30, by_origin=True, log=out.append)
        text = "\n".join(out)
        self.assertIn("0.050 / 0.050 / 0 / 0 / 0.050", text)
        self.assertIn("0.350 / 0.350 / 2 / 2 / 0.350", text)      # both walls end 0.05 + 0.3 apart
        self.assertIn("found only in B (1 pairs)", text)
        self.assertIn("found in A (1 pairs)", text)
        # a set frozen from A alone holds the first wall only and cannot show the second
        self.run_freeze(self.specs("A"), out=self.root / "a_only.npz")
        self.assertEqual(wp.WallSet(self.root / "a_only.npz").meta["pairs"], 1)

    def test_origins_on_different_node_grids_equal_labels_or_a_bad_spec_are_refused(self):
        with self.assertRaisesRegex(ValueError, "node times differ"):
            self.run_freeze(self.specs("A", "B"), ns_b=self.ns + 1)
        with self.assertRaisesRegex(ValueError, "labels must differ"):
            self.run_freeze(["A=@A=@A.csv", "A=@B=@B.csv"])
        with self.assertRaisesRegex(ValueError, "expected"):
            self.run_freeze(["nonsense"])

    def test_overlap_is_measured_on_point_times(self):
        np = self.np
        t = np.linspace(10.0, 14.0, 40)
        self.assertEqual(wp._overlap(t, t + 1e-12), 1.0)
        self.assertEqual(wp._overlap(t, t + 100.0), 0.0)
        self.assertAlmostEqual(wp._overlap(t, np.r_[t[20:], t[-1] + 1.0 + np.arange(20) * 0.01]), 0.5)

    def test_measure_adds_the_severe_count_and_the_largest_gap(self):
        self.run_freeze(self.specs("B"))
        out = []
        with self.trajectory_patch(a=self.origin, b=self.moved("b", shift_after=0.07), c=self.moved("c", shift_after=0.3)):
            wp.measure(str(self.out), ["a", "b", "c"], threshold=0.08, severe=0.30, log=out.append)
        text = "\n".join(out)
        self.assertIn("0.050 / 0.050 / 0 / 0 / 0.050", text)
        self.assertIn("0.120 / 0.120 / 2 / 0 / 0.120", text)
        self.assertIn("0.350 / 0.350 / 2 / 2 / 0.350", text)
        self.assertIn("count at or above 300 mm / largest", text)


class ContactsToWallPairsTests(_WalkMixin, unittest.TestCase):
    """One contact with three shared cells on the wall y = 2 (the later visit 0.01, 0.05 and 0.20 m further out: the
    contact's gap is their median, 0.05) and a second contact with one cell 0.5 m apart."""

    def setUp(self):
        import pickle
        self.walk()
        np = self.np
        rng = np.random.default_rng(11)
        t, xyz, ia, ib, ga, gb = [], [], [], [], [], []
        cur = 0

        def add(tv, off, ids, cells, cell):
            nonlocal cur
            t.append(tv)
            xyz.append(np.column_stack([rng.uniform(0, 0.5, 20), np.full(20, 2.0 + off), np.full(20, 0.5)]))
            ids.extend(range(cur, cur + 20))
            cells.extend([cell] * 20)
            cur += 20

        for c, gap in enumerate([0.01, 0.05, 0.20]):
            add(np.linspace(10.02 + c, 12.02 + c, 20), 0.0, ia, ga, 100 + c)
            add(np.linspace(40.02 + c, 42.02 + c, 20), gap, ib, gb, 100 + c)
        add(np.linspace(20.02, 22.02, 20), 0.0, ia, ga, 7)
        add(np.linspace(55.02, 57.02, 20), 0.5, ib, gb, 7)
        arr = lambda v, lo, hi: np.array(v[lo:hi])
        q1 = dict(level=0, c=np.array([0.25, 2.0, 0.5]), n=np.array([0., 1., 0.]), shared=np.array([100, 101, 102]),
                  ia=arr(ia, 0, 60), ib=arr(ib, 0, 60), ga=arr(ga, 0, 60), gb=arr(gb, 0, 60))
        q2 = dict(level=2, c=np.array([0.25, 2.0, 0.5]), n=np.array([0., 1., 0.]), shared=np.array([7]),
                  ia=arr(ia, 60, 80), ib=arr(ib, 60, 80), ga=arr(ga, 60, 80), gb=arr(gb, 60, 80))
        self.contacts, self.sample, self.out = self.root / "c.pkl", self.root / "s.npz", self.root / "contacts.npz"
        with open(self.contacts, "wb") as fp:
            pickle.dump([q1, q2], fp)
        np.savez(self.sample, t=np.concatenate(t), xyz=np.vstack(xyz))     # the placed points: world positions on the origin trajectory

    def convert(self, out=None, **kw):
        with patch.object(wp, "load_trajectory", lambda spec: self.origin):
            return c2w.convert(str(self.contacts), str(self.sample), "origin", str(out or self.out), log=lambda *a: None, **kw)

    def test_contact_gap_is_the_median_of_its_cells_and_classes_follow_the_contact(self):
        self.convert()
        ws = wp.WallSet(self.out)
        self.assertEqual((ws.meta["pieces"], ws.meta["pairs"], ws.meta["visits"]), (2, 4, 8))
        self.assertEqual(ws.pair_contact.tolist(), [0, 0, 0, 1])
        self.assertEqual(self.np.round(wp.pair_gaps(ws, self.origin), 9).tolist(), [0.01, 0.05, 0.2, 0.5])
        self.assertEqual(self.np.round(wp.contact_gaps(ws, self.origin), 9).tolist(), [0.05, 0.5])
        out = []
        with self.trajectory_patch(o=self.origin, s=self.moved("s", shift_after=0.1)):
            wp.measure(str(self.out), ["o", "s"], threshold=0.08, severe=0.30, before=50., after=0.0, log=out.append,
                       csv_path=str(self.root / "g.csv"))
        text = "\n".join(out)
        self.assertIn("all pairs (2 pairs)", text)
        self.assertIn("2 contacts", text)
        self.assertIn("storey 0 (1 pairs)", text)
        self.assertIn("storey 2 (1 pairs)", text)
        self.assertIn("both visits before 50 s (1 pairs)", text)         # contact 1's later visit ends at 57 s
        self.assertIn("0.050 / 0.050 / 0 / 0 / 0.050", text)             # contact 0 on the origin trajectory
        self.assertIn("0.150 / 0.150 / 1 / 0 / 0.150", text)             # and with the later visit moved by 0.1
        self.assertEqual(len((self.root / "g.csv").read_text().strip().splitlines()), 3)    # header and two contacts

    def test_a_rigid_move_changes_nothing_and_a_shift_between_the_visits_shows(self):
        self.convert()
        ws = wp.WallSet(self.out)
        g0 = wp.contact_gaps(ws, self.origin)
        self.assertLess(self.np.abs(wp.contact_gaps(ws, self.moved("r", rigid=0.7)) - g0).max(), 1e-9)
        self.assertAlmostEqual(float(wp.contact_gaps(ws, self.moved("s", shift_after=0.07))[0]), 0.12, places=9)

    def test_local_shift_and_node_ties_are_applied_and_the_node_grid_is_checked(self):
        np = self.np
        self.convert()
        plain = wp.WallSet(self.out).local.copy()
        self.convert(local_shift=(0.0, 0.5, 0.0))
        ws = wp.WallSet(self.out)
        self.assertTrue(np.allclose(ws.local - plain, [0.0, 0.5, 0.0]))
        self.assertEqual(ws.meta["local_shift_m"], [0.0, 0.5, 0.0])
        self.assertNotIn(3, ws.node.tolist())
        ties = self.root / "ties.npz"
        np.savez(ties, ns=np.array([int(np.rint(10.02 * 1e9))]), idx=np.array([3]))     # the first return ties node 3
        self.convert(node_ties=str(ties))
        self.assertIn(3, wp.WallSet(self.out).node.tolist())
        with self.assertRaisesRegex(ValueError, "node times differ"):
            wp.contact_gaps(wp.WallSet(self.out), self.moved("m", ns_offset=1))
        self.assertEqual(c2w.main([str(self.contacts), str(self.sample), str(self.root / "missing"), str(self.out)]), 2)


class LandmarkCheckTests(_WalkMixin, unittest.TestCase):
    """The wall y = 2.03 in world seen at 10-14 s. The plan fit puts the wall at plan y = target_y: with rotation 0 the
    plan y is y + shift_y, so shift_y - target_y = -2 puts the expected line at y = 2."""

    def setUp(self):
        self.walk()
        np = self.np
        rng = np.random.default_rng(2)
        self.t = np.linspace(10.02, 14.02, 200)
        self.world = np.column_stack([rng.uniform(0, 0.5, 200), np.full(200, 2.03), np.full(200, 0.5)])
        self.node = wp.node_of(self.t, self.ns)
        self.local = wp.to_local(self.world, self.node, self.origin)

    def cohort(self, shift=(1.0, -1.0), target_y=1.0, rotation=0.0):
        p = self.root / "cohort.npz"
        self.np.savez(p, node=self.node, body=self.local, rotation_deg=rotation, shift_x=shift[0], shift_y=shift[1], target_y=target_y,
                      raw_ids=self.np.arange(200))
        return p

    def convert(self, name, **kw):
        out = self.root / name
        with patch.object(wp, "load_trajectory", lambda spec: self.origin):
            lm.from_cohort(str(self.cohort(**kw)), "origin", str(out), label="synthetic", log=lambda *a: None)
        return out

    def test_line_from_the_plan_fit_and_signed_median_of_a_cohort(self):
        np = self.np
        p0, n = lm.line_from_plan_fit(0.0, (1.0, -1.0), 1.0)
        self.assertEqual(n.tolist(), [0.0, 1.0, 0.0])
        self.assertAlmostEqual(float(p0[1]), 2.0)
        p0, n = lm.line_from_plan_fit(90.0, (0.0, 0.0), 3.0)     # plan y = x: the wall at world x = 3
        self.assertTrue(np.allclose(n, [1, 0, 0], atol=1e-12) and abs(p0[0] - 3.0) < 1e-12)
        out = self.convert("wall.npz")
        res = []
        with self.trajectory_patch(a=self.origin, b=self.moved("b", shift_all=0.2), r=self.moved("r", rigid=0.4)):
            rows = lm.measure(str(out), ["a", "b", "r"], log=res.append)["rows"]
        self.assertAlmostEqual(rows[0][1], 0.03, places=9)
        self.assertAlmostEqual(rows[1][1], 0.23, places=9)
        self.assertGreater(abs(rows[2][1] - 0.03), 0.1)           # the line is fixed in the world: a rigid move shows (the anchor)
        text = "\n".join(res)
        self.assertIn("caveat: the absolute value depends on the plan alignment", text)
        self.assertIn("rotation 0.0000 deg", text)
        self.assertIn("+0.200", text)                             # change of b against the first

    def test_the_change_between_trajectories_does_not_depend_on_the_anchor_but_the_absolute_value_does(self):
        out1 = self.convert("w1.npz", shift=(1.0, -1.0))
        out2 = self.convert("w2.npz", shift=(1.0, -0.4))          # another plan alignment
        with self.trajectory_patch(a=self.origin, b=self.moved("b", shift_all=0.2)):
            r1 = lm.measure(str(out1), ["a", "b"], log=lambda *a: None)["rows"]
            r2 = lm.measure(str(out2), ["a", "b"], log=lambda *a: None)["rows"]
        self.assertAlmostEqual(r1[0][1], 0.03, places=9)
        self.assertAlmostEqual(r2[0][1], 0.63, places=9)
        self.assertAlmostEqual(r1[1][3], 0.2, places=9)
        self.assertAlmostEqual(r2[1][3], 0.2, places=9)
        with self.trajectory_patch(c=self.moved("c", ns_offset=1)):
            with self.assertRaisesRegex(ValueError, "node times differ"):
                lm.measure(str(out1), ["c"], log=lambda *a: None)
        self.assertEqual(lm.main(["measure", str(out1), str(self.root / "missing")]), 2)

    def write_las(self, path, world, t_s):
        import laspy
        header = laspy.LasHeader(point_format=1, version="1.4")
        header.scales = self.np.array([1e-6, 1e-6, 1e-6])
        las = laspy.LasData(header)
        las.x, las.y, las.z = world[:, 0], world[:, 1], world[:, 2]
        las.gps_time = t_s * 1e9
        las.write(str(path))

    def test_freeze_takes_the_time_window_inside_the_band_and_counts_the_rest(self):
        np = self.np
        result = self.root / "ref"
        result.mkdir()
        far = self.world[:50] + [0.0, 3.0, 0.0]                   # 3 m behind the wall: outside a 1 m band
        t_all = np.r_[self.t, self.t[:50], np.linspace(30.02, 32.0, 50)]
        self.write_las(result / "scan_lio_merged.las", np.vstack([self.world, far, self.world[:50]]), t_all)
        (result / "scan_lio_merged.las").replace(result / "scan_lio_merged.laz")
        T = self.ns * 1e-9
        line = (np.array([0., 2., 0.]), np.array([0., 1., 0.]))
        out = self.root / "freeze.npz"
        with patch("map_quality_lib.trajectory", lambda r: (T, self.rot, self.pos)):
            meta = lm.freeze(str(result), str(out), (9.9, 20.0), line, band=1.0, log=lambda *a: None)
            self.assertEqual((meta["returns"], meta["returns_in_time_window"]), (200, 250))
            self.assertLess(meta["max_round_trip_m"], 1e-5)
            with self.assertRaisesRegex(ValueError, "no returns"):
                lm.freeze(str(result), str(out), (40.0, 50.0), line, log=lambda *a: None)
            fit = self.root / "plan_fit.json"
            fit.write_text(json.dumps({"rotation_deg": 0.0, "shift_m": [1.0, -1.0]}))
            out2 = self.root / "freeze2.npz"
            self.assertEqual(lm.main(["freeze", str(result), str(out2), "--time", "9.9", "20", "--plan-fit", str(fit), "--plan-y", "1.0"]), 0)
            self.assertEqual(lm.main(["freeze", str(result), str(out2), "--time", "0", "1"]), 2)      # no line given
        self.assertIn("plan_fit.json", lm.Landmark(out2).meta["plan_fit"])
        with patch.object(wp, "load_trajectory", lambda spec: self.origin):
            rows = lm.measure(str(out2), ["a"], log=lambda *a: None)["rows"]
        self.assertAlmostEqual(rows[0][1], 0.03, places=5)


class StepChecksRegisterLineTest(unittest.TestCase):
    """The decisions of step_checks.py's `check register` and 44_short lines (design D5 of check-register)."""

    def setUp(self):
        import step_checks
        self.sc = step_checks

    def test_identical_holds_everywhere(self):
        for main_tree in (False, True):
            self.assertEqual(self.sc.register_verdict(0, "IDENTICAL", main_tree), (True, "IDENTICAL"))

    def test_additions_hold_in_a_worktree_and_fail_in_the_main_tree(self):
        ok, text = self.sc.register_verdict(3, "to record: +1 case", False, "run/check_register.txt")
        self.assertTrue(ok)
        self.assertTrue(text.startswith("HOLDS, to record: +1 case"))
        self.assertIn("run/check_register.txt", text)
        ok, text = self.sc.register_verdict(3, "to record: +1 case", True, "run/check_register.txt")
        self.assertFalse(ok)
        self.assertTrue(text.startswith("FAILED, to record: +1 case"))

    def test_a_change_fails_everywhere_with_the_summary(self):
        for main_tree in (False, True):
            ok, text = self.sc.register_verdict(1, "1 assertion changed", main_tree, "l.txt")
            self.assertFalse(ok)
            self.assertTrue(text.startswith("FAILED: 1 assertion changed"))

    def test_unreadable_fails_with_the_tools_message(self):
        ok, text = self.sc.register_verdict(2, "", False, "l.txt", "tests/a.cpp:4: unmatched '('\n")
        self.assertFalse(ok)
        self.assertIn("unreadable: tests/a.cpp:4: unmatched '('", text)

    def test_an_unknown_exit_code_fails(self):
        self.assertFalse(self.sc.register_verdict(7, "x", False)[0])

    def test_instruments_summary_is_split_from_the_register_summary(self):
        out = "ADD case added: f :: c\ninstruments: review section s: its case differs ('c')\ninstruments: ok (3 instruments, 2 items not established or only stated)\nto record: +1 case\n"
        summary, inst = self.sc.parse_register_output(out)
        self.assertEqual(summary, "to record: +1 case")
        self.assertEqual(inst, "instruments: ok (3 instruments, 2 items not established or only stated)")
        self.assertEqual(self.sc.parse_register_output("IDENTICAL\n"), ("IDENTICAL", None))

    def test_the_main_tree_is_told_from_a_linked_worktree_by_the_git_dirs(self):
        with patch.object(self.sc.kit, "git", side_effect=lambda *a: {"--absolute-git-dir": "C:/r/.git", "--git-common-dir": ".git"}[a[1]]),                 patch.object(self.sc.kit, "ROOT", Path("C:/r")):
            self.assertTrue(self.sc.in_main_tree())
        with patch.object(self.sc.kit, "git", side_effect=lambda *a: {"--absolute-git-dir": "C:/r/.git/worktrees/w", "--git-common-dir": "C:/r/.git"}[a[1]]),                 patch.object(self.sc.kit, "ROOT", Path("C:/w")):
            self.assertFalse(self.sc.in_main_tree())

    def test_a_different_run_with_windows_at_the_time_limit_says_the_load_decided(self):
        text = self.sc.short_run_detail(True, False, "exit 0 | DIFFERENT", "5 min", 3)
        self.assertTrue(text.startswith("DIFFERENT (3 windows at the time limit: the result depended on the machine's load; run again)"))

    def test_a_different_run_without_windows_at_the_limit_is_plain(self):
        text = self.sc.short_run_detail(True, False, "exit 0 | DIFFERENT", "5 min", 0)
        self.assertNotIn("depended on the machine", text)
        self.assertIn("time-limit windows 0", text)

    def test_an_identical_run_reports_the_count_and_a_missing_one_says_so(self):
        self.assertIn("time-limit windows 0", self.sc.short_run_detail(True, True, "IDENTICAL", "5 min", 0))
        self.assertIn("time limit: not reported", self.sc.short_run_detail(True, True, "IDENTICAL", "5 min", None))


# ---- run_health.py on a log whose readings are known (check-register D10 item 2) ----

def known_log(n=200, bends=None, speeds=None, biases=None, chi2=None):
    """A run log of n windows in run_health's input format. Every window is bent 0.5 deg, walks at 1 m/s, has the gyro
    bias 0.10 / -0.05 / 0.02 deg/s and information ratio 0.6; the given dicts (window number -> value) plant a bend [deg],
    an end speed [m/s along x] or a bias x [deg/s]; chi2 (window number -> value) makes those windows weak (information
    ratio 0.3) with that chi-square of the start-velocity jump."""
    bends, speeds, biases, chi2 = bends or {}, speeds or {}, biases or {}, chi2 or {}
    lines = []
    for i in range(1, n + 1):
        info = 0.3 if i in chi2 else 0.6
        lines.append(f"window diagnostics {i}: t {i * 0.36:.2f} s, dt 0.36 s, iterations 100, delta 0, start {i * 0.4:.2f} 0 0, guess move 0 0 0, info ratio {info}")
        lines.append(f"window diagnostics {i}: velocity start 0 0 0, end {speeds.get(i, 1.0)} 0 0")
        lines.append(f"window diagnostics {i}: heading 0 deg, yaw correction first 0 last {bends.get(i, 0.5)} deg")
        if i in chi2:
            lines.append(f"window diagnostics {i}: weak axis 1 0 0, start-velocity jump 0.01 0 0 m/s, chi2 {chi2[i]}, prior sigma along it 0.02 m/s; "
                         "stiffness: ndt 3, velocity prior 1, motion model 1")
        lines.append(f"gyro bias [deg/s] window {i}: {biases.get(i, 0.10)} -0.05 0.02")
        lines.append(f"finished at iteration 100/100 optimizing window {i}/{n} with acc_distance 0")
    lines.append("Processing time: 1 [s]")
    return "\n".join(lines) + "\n"


def chi2_values(median, p95):
    """41 chi-square values on weak windows 10, 13, ... with exactly this median (sorted position 20 of 0..40) and 95th
    percentile (position 0.95 * 40 = 38); the values between the two sit halfway."""
    values = [median] * 21 + [(median + p95) / 2] * 17 + [p95] * 3
    return {10 + 3 * j: v for j, v in enumerate(values)}


class RunHealthKnownLogTests(unittest.TestCase):
    """run_health on logs written here: every reading and every row's verdict exact, with the plants above the limits
    (bent 4 deg, 2.6 m/s, bias 0.6 deg/s, chi2 median 8 and p95 30) and just below them (2.9 deg, 2.49 m/s, 0.49 deg/s,
    chi2 median 4.9 and p95 19.4)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "run.log"

    def read(self, text):
        import argparse
        self.path.write_text(text)
        ap = argparse.ArgumentParser()
        health.add_limits(ap)
        r = health.analyse(self.path)
        return r, {row[0]: row[1:] for row in health.judge(r, ap.parse_args([]))}

    def exit_code(self):
        proc = subprocess.run([sys.executable, "-B", str(ROOT / "tools/run_health.py"), str(self.path)], capture_output=True, text=True)
        return proc.returncode, proc.stdout

    ABOVE = dict(bends={77: -4.0}, speeds={120: 2.6}, biases={150: 0.6}, chi2=chi2_values(8.0, 30.0))
    BELOW = dict(bends={77: -2.9}, speeds={120: 2.49}, biases={150: 0.49}, chi2=chi2_values(4.9, 19.4))

    def test_the_planted_values_are_read_back_exactly_and_judged(self):
        r, rows = self.read(known_log(**self.ABOVE))
        self.assertEqual(r["windows"], 200)
        self.assertEqual((r["bend_gt3"], r["bend_gt10"], r["bend_max"]), (1, 0, 4.0))
        self.assertEqual(r["speed_max"], 2.6)
        self.assertEqual(r["bias_max"], [0.6, 0.05, 0.02])
        self.assertEqual((r["chi2_windows"], r["chi2_p50"], r["chi2_p95"]), (41, 8.0, 30.0))
        self.assertEqual(r["coverage"], {"bend": 200, "speed": 200, "bias": 200, "chi2": 41})
        self.assertEqual(r["missing_required"], [])
        # one window in 200 is 0.5 %, under the 1 % share limit: a single 4 deg bend does not trigger
        self.assertEqual(rows["windows bent > 3 deg against the gyro"], ("1 (0.5 %)", "<= 1 %", "CLEAR"))
        self.assertEqual(rows["windows bent > 10 deg"], ("0", "0", "CLEAR"))
        self.assertEqual(rows["largest bend"], ("4.0 deg", "", ""))
        self.assertEqual(rows["windows faster than 2.5 m/s"], ("1 (max 2.60 m/s)", "0", "TRIGGER"))
        self.assertEqual(rows["largest gyro bias estimate x / y / z"], ("0.60 / 0.05 / 0.02 deg/s", "<= 0.5", "TRIGGER"))
        self.assertEqual(rows["chi2 of the velocity jump, median / p95"], ("8 / 30 (41 windows)", "<= 5 / 20", "TRIGGER"))
        code, out = self.exit_code()
        self.assertEqual(code, 1)
        self.assertIn("result: TRIGGERS", out)

    def test_each_planted_value_just_below_its_limit_is_clear(self):
        r, rows = self.read(known_log(**self.BELOW))
        self.assertEqual((r["bend_gt3"], r["bend_max"], r["speed_max"], r["bias_max"][0]), (0, 2.9, 2.49, 0.49))
        self.assertEqual((r["chi2_p50"], r["chi2_p95"]), (4.9, 19.4))
        self.assertEqual(rows["windows bent > 3 deg against the gyro"], ("0 (0.0 %)", "<= 1 %", "CLEAR"))
        self.assertEqual(rows["largest bend"], ("2.9 deg", "", ""))
        self.assertEqual(rows["windows faster than 2.5 m/s"], ("0 (max 2.49 m/s)", "0", "CLEAR"))
        self.assertEqual(rows["largest gyro bias estimate x / y / z"], ("0.49 / 0.05 / 0.02 deg/s", "<= 0.5", "CLEAR"))
        self.assertEqual(rows["chi2 of the velocity jump, median / p95"], ("5 / 19 (41 windows)", "<= 5 / 20", "CLEAR"))
        self.assertEqual([k for k, v in rows.items() if v[2] == "TRIGGER"], [])
        code, out = self.exit_code()
        self.assertEqual(code, 0)
        self.assertIn("result: DIAGNOSTICS COMPLETE", out)

    def test_the_bend_share_triggers_from_the_third_window_in_200(self):
        # the share limit is 1 %: 2 windows bent 4 deg of 200 are exactly at it (CLEAR), 3 are over (TRIGGER)
        _, rows = self.read(known_log(bends={20: 4.0, 90: -4.0}))
        self.assertEqual(rows["windows bent > 3 deg against the gyro"], ("2 (1.0 %)", "<= 1 %", "CLEAR"))
        _, rows = self.read(known_log(bends={20: 4.0, 90: -4.0, 160: 4.0}))
        self.assertEqual(rows["windows bent > 3 deg against the gyro"], ("3 (1.5 %)", "<= 1 %", "TRIGGER"))
        self.assertEqual(self.exit_code()[0], 1)
        # one window bent over 10 deg triggers by itself
        _, rows = self.read(known_log(bends={20: 10.5}))
        self.assertEqual(rows["windows bent > 10 deg"], ("1", "0", "TRIGGER"))


# ---- compare_baseline.py on folders written here (check-register D10 item 3) ----

def write_result_folder(folder, out_path, in_path, stamp, elapsed, commit, data=b"lidar-points-0123456789"):
    """The smallest result folder that exercises compare_baseline's normalisation: the two dMapper_* files named with a
    date-time, session.mjs and the params TOML with the folders' own paths, run-dependent fields in the TOML and the
    JSON, a trajectory CSV and a LAZ stand-in (raw bytes), and a foreign .asc file the tool ignores."""
    folder.mkdir(parents=True)
    (folder / "session.mjs").write_text(json.dumps({"Session Settings": {"out_folder_name": str(out_path), "lidar_odometry_version": "0.1.0"}}, indent=2))
    (folder / f"dMapper_params_0.1.0_{stamp[:16]}.toml").write_text(
        f"# Created on: {stamp}\n[version_info]\nbuild_date = \"{stamp}\"\n[paths]\nworking_directory_preview = '{in_path}'\n"
        f"[lidar_odometry]\nthreshold = 0.5\n")
    (folder / f"dMapper_results_0.1.0_{stamp[:16]}.json").write_text(json.dumps(
        {"elapsed time seconds": elapsed, "processing_date": stamp, "git_commit": commit, "output": str(out_path) + "\\scan_lio_0.laz",
         "input": str(in_path) + "\\lidar0.laz", "windows": 12}, indent=2))
    (folder / "trajectory_lio_0.csv").write_text("timestamp x y z\n1 0.0 0.0 0.0\n2 1.0 0.5 0.0\n")
    (folder / "scan_lio_0.laz").write_bytes(data)
    (folder / "scan_lio_0.asc").write_text(f"foreign {stamp}\n")


class CompareBaselineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.a, self.b = self.root / "a" / "out", self.root / "b" / "out"
        write_result_folder(self.a, self.a, self.root / "inA" / "rec", "2026-11-25_10-00-31", 12.5, "abc1234")
        write_result_folder(self.b, self.b, self.root / "inB" / "rec", "2026-11-26_17-45-02", 99.0, "def5678")

    def run_tool(self, *args):
        proc = subprocess.run([sys.executable, "-B", str(ROOT / "tools/compare_baseline.py"), *map(str, args)], capture_output=True, text=True)
        return proc.returncode, proc.stdout

    def flip_a_byte(self, path):
        data = bytearray(path.read_bytes())
        data[len(data) // 2] ^= 0x01
        path.write_bytes(bytes(data))

    def test_folders_that_differ_only_in_run_dependent_fields_are_identical(self):
        code, out = self.run_tool(self.b, self.a)
        self.assertEqual((code, out.strip()), (0, "IDENTICAL: 5 files"))  # the .asc file is not one of them

    def test_a_changed_setting_is_not_normalised_away(self):
        toml = next(self.b.glob("dMapper_params_*.toml"))
        toml.write_text(toml.read_text().replace("threshold = 0.5", "threshold = 0.6"))
        code, out = self.run_tool(self.b, self.a)
        self.assertEqual(code, 1)
        self.assertIn("DIFFERENT: 1 of 5 files", out)

    def test_one_changed_byte_in_a_data_file_is_reported_by_name(self):
        for name in ("scan_lio_0.laz", "trajectory_lio_0.csv"):
            with self.subTest(name):
                changed = self.root / "c" / name
                write_result_folder(changed, changed, self.root / "inC" / "rec", "2026-11-27_08-00-00", 1.0, "0000000")
                self.flip_a_byte(changed / name)
                code, out = self.run_tool(changed, self.a)
                self.assertEqual(code, 1)
                self.assertIn(f"different  {name}", out.splitlines())
                self.assertEqual(sum(line.startswith("different") for line in out.splitlines()), 1)
                self.assertIn("DIFFERENT: 1 of 5 files", out)

    def test_a_manifest_written_from_a_folder_is_identical_to_it_and_to_its_twin(self):
        manifest = self.root / "manifest.json"
        code, out = self.run_tool("--write-manifest", self.a, manifest, "--dataset", "known", "--params", "p.toml")
        self.assertEqual((code, out.strip()), (0, f"wrote {manifest} (5 files)"))
        written = json.loads(manifest.read_text())
        self.assertEqual((written["dataset"], written["params"], len(written["files"])), ("known", "p.toml", 5))
        self.assertEqual(self.run_tool(self.a, manifest), (0, "IDENTICAL: 5 files\n"))
        self.assertEqual(self.run_tool(self.b, manifest), (0, "IDENTICAL: 5 files\n"))
        self.flip_a_byte(self.b / "scan_lio_0.laz")
        code, out = self.run_tool(self.b, manifest)
        self.assertEqual(code, 1)
        self.assertIn("different  scan_lio_0.laz", out.splitlines())
        self.assertIn("DIFFERENT: 1 of 5 files", out)


if __name__=='__main__':
    unittest.main()
