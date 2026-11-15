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


if __name__=='__main__':
    unittest.main()
