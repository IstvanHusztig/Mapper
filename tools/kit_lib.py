"""Shared helpers of the verification kit (step_checks.py, end_runs.py, run_replay.py): the workstation's paths
(tools/local_paths.toml), the build, the gate that queues replays and full runs, and running dMapper with its cost.

The kit works on the worktree its own file lies in: ROOT is that worktree, the build is its out/build/<preset>. The
captures, the reference files and the gate are shared by all worktrees (the `guards` folder of local_paths.toml).

Windows only (the build needs vcvars64.bat; memory and process readings use the Win32 API through ctypes)."""
import ctypes
import ctypes.wintypes as wt
import json
import os
import subprocess
import time
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / "out/build/x64-Release/bin"
WORKTREE = ROOT.name


def load_paths():
    with (ROOT / "tools/local_paths.toml").open("rb") as f:
        p = tomllib.load(f)
    p["guards"] = Path(p["guards"])
    p["recordings"] = Path(p["recordings"])
    return p


def run_folder(paths):
    """The kit's scratch folder of this worktree (logs, guard outputs, result folders)."""
    folder = paths["guards"] / "runs" / WORKTREE
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def clean_env():
    """The environment without DMAPPER_* variables (experiment switches of older builds)."""
    return {k: v for k, v in os.environ.items() if not k.startswith("DMAPPER_")}


def vs_run(paths, name, lines):
    """Runs command lines in this worktree with the MSVC environment (vcvars64.bat) loaded; returns the exit code."""
    bat = run_folder(paths) / f"{name}.bat"
    head = ["@echo off", f'call "{Path(paths["vcvars"])}" >nul 2>nul', f'cd /d "{ROOT}"']
    bat.write_text("\r\n".join(head + list(lines)) + "\r\n", newline="")
    return subprocess.run(["cmd", "/d", "/c", str(bat)], capture_output=True, text=True).returncode


def build(paths, preset="x64-Release", targets=("dMapper", "lidar_odometry_tests")):
    """Configure and build the preset in this worktree. Returns (ok, compiler error lines). A run or replay started from
    this worktree keeps its dMapper.exe open and the linker could not replace it: then nothing is built."""
    exe = ROOT / "out/build" / preset / "bin/dMapper.exe"
    if "dMapper" in targets and exe.exists():
        try:
            with open(exe, "ab"):
                pass
        except PermissionError:
            return False, [f"{exe} is in use (a run or replay started from this worktree is running): wait for it, then build"]
    log = run_folder(paths) / f"build_{preset}.log"
    code = vs_run(paths, f"build_{preset}", [f'cmake --preset {preset} > "{log}" 2>&1',
                                            "if errorlevel 1 exit /b 1",
                                            f'cmake --build --preset {preset} --target {" ".join(targets)} >> "{log}" 2>&1',
                                            "exit /b %ERRORLEVEL%"])
    errors = [l for l in log.read_text(errors="replace").splitlines() if "error C" in l or "error LNK" in l or "CMake Error" in l]
    return code == 0 and not errors, errors[:5]


def debug_ctest(paths):
    """ctest of this worktree's Debug build. Returns (ok, the summary lines)."""
    log = run_folder(paths) / "debug_ctest.log"
    code = vs_run(paths, "debug_ctest", [f'ctest --test-dir out/build/x64-Debug > "{log}" 2>&1', "exit /b %ERRORLEVEL%"])
    lines = [l.strip() for l in log.read_text(errors="replace").splitlines() if "tests passed" in l or "Total Test time" in l]
    return code == 0, lines


# --- Win32 readings -----------------------------------------------------------------------------------------------

class _MemoryStatus(ctypes.Structure):
    _fields_ = [("dwLength", wt.DWORD), ("dwMemoryLoad", wt.DWORD), ("ullTotalPhys", ctypes.c_ulonglong),
                ("ullAvailPhys", ctypes.c_ulonglong), ("ullTotalPageFile", ctypes.c_ulonglong),
                ("ullAvailPageFile", ctypes.c_ulonglong), ("ullTotalVirtual", ctypes.c_ulonglong),
                ("ullAvailVirtual", ctypes.c_ulonglong), ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]


class _MemoryCounters(ctypes.Structure):
    _fields_ = [("cb", wt.DWORD), ("PageFaultCount", wt.DWORD), ("PeakWorkingSetSize", ctypes.c_size_t),
                ("WorkingSetSize", ctypes.c_size_t), ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPagedPoolUsage", ctypes.c_size_t), ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                ("QuotaNonPagedPoolUsage", ctypes.c_size_t), ("PagefileUsage", ctypes.c_size_t),
                ("PeakPagefileUsage", ctypes.c_size_t)]


def free_memory_bytes():
    status = _MemoryStatus()
    status.dwLength = ctypes.sizeof(status)
    ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status))
    return status.ullAvailPhys


def process_alive(pid):
    k32 = ctypes.windll.kernel32
    k32.OpenProcess.restype = wt.HANDLE
    handle = k32.OpenProcess(0x1000, False, int(pid))  # PROCESS_QUERY_LIMITED_INFORMATION
    if not handle:
        return False
    code = wt.DWORD()
    k32.GetExitCodeProcess(wt.HANDLE(handle), ctypes.byref(code))
    k32.CloseHandle(wt.HANDLE(handle))
    return code.value == 259  # STILL_ACTIVE


def _process_cost(handle):
    """CPU seconds (kernel + user) and peak working set [bytes] of a process, also after it has ended."""
    k32 = ctypes.windll.kernel32
    created, exited, kernel, user = wt.FILETIME(), wt.FILETIME(), wt.FILETIME(), wt.FILETIME()
    k32.GetProcessTimes(wt.HANDLE(handle), ctypes.byref(created), ctypes.byref(exited), ctypes.byref(kernel), ctypes.byref(user))
    ticks = lambda f: (f.dwHighDateTime << 32) + f.dwLowDateTime
    counters = _MemoryCounters()
    counters.cb = ctypes.sizeof(counters)
    k32.K32GetProcessMemoryInfo(wt.HANDLE(handle), ctypes.byref(counters), counters.cb)
    return (ticks(kernel) + ticks(user)) / 1e7, counters.PeakWorkingSetSize


def dmapper_processes():
    out = subprocess.check_output(["tasklist", "/FI", "IMAGENAME eq dMapper.exe", "/FO", "CSV", "/NH"], text=True)
    return out.count('"dMapper.exe"')


def run_dmapper(args, log_path):
    """Runs this worktree's dMapper with its output in log_path. Returns exit code, wall and CPU seconds, peak memory."""
    start = time.monotonic()
    with open(log_path, "wb") as log:
        p = subprocess.Popen([str(BIN / "dMapper.exe"), *map(str, args)], cwd=BIN, env=clean_env(), stdout=log, stderr=subprocess.STDOUT)
        code = p.wait()
        cpu_s, peak = _process_cost(int(p._handle))
    return {"exit": code, "wall_s": time.monotonic() - start, "cpu_s": cpu_s, "peak_bytes": peak}


def cost_text(cost):
    cores = cost["cpu_s"] / cost["wall_s"] if cost["wall_s"] > 0 else 0.0
    return f'wall {cost["wall_s"]:.0f} s, CPU {cost["cpu_s"]:.0f} s ({cores:.1f} cores), peak memory {cost["peak_bytes"] / 2**30:.1f} GB'


# --- the gate -------------------------------------------------------------------------------------------------------

class Gate:
    """Queues the heavy jobs of all worktrees. A "replay" holder shares the gate with others (at most replay_slots, and
    only while enough memory is free for its estimated need); a "full" holder (a full run: timing-sensitive, and it uses
    its recording's cache folder) has the gate alone. A job waits before it starts; a running job is never paused (a pause
    frees no memory, and a paused window would run into the wall-clock time limit and change the result). A holder whose
    process has ended is ignored."""

    FRESH_S = 90.0  # a replay younger than this may not have reached its peak memory: its need still counts as taken

    def __init__(self, paths, say=print):
        self.folder = paths["guards"] / "gate"
        self.folder.mkdir(parents=True, exist_ok=True)
        self.slots = int(paths["gate"]["replay_slots"])
        self.factor = float(paths["gate"]["memory_factor"])
        self.reserve = float(paths["gate"]["memory_reserve_gb"]) * 2**30
        self.say = say

    def need_bytes(self, capture_file):
        return int(self.factor * Path(capture_file).stat().st_size)

    def holders(self):
        found = []
        for f in self.folder.glob("*.lock"):
            try:
                h = json.loads(f.read_text())
            except (OSError, ValueError):
                continue
            if process_alive(h["pid"]):
                found.append(h)
            else:
                f.unlink(missing_ok=True)  # its process ended without releasing
        return found

    def _mutex(self):
        m = self.folder / "mutex"
        while True:
            try:
                m.mkdir()
                return m
            except FileExistsError:
                try:
                    if time.time() - m.stat().st_mtime > 20.0:  # left by a process that died inside it
                        m.rmdir()
                except OSError:
                    pass
                time.sleep(0.05)

    def _reason(self, kind, need):
        holders = self.holders()
        full = [h for h in holders if h["kind"] == "full"]
        replays = [h for h in holders if h["kind"] == "replay"]
        if kind == "full":
            return f'{len(holders)} job(s) running ({", ".join(h["label"] for h in holders)})' if holders else None
        if full:
            return f'a full run ({full[0]["label"]})'
        if len(replays) >= self.slots:
            return f'a slot ({len(replays)} of {self.slots} replays running)'
        taken = sum(h.get("need", 0) for h in replays if time.time() - h["since"] < self.FRESH_S)
        free = free_memory_bytes() - taken
        if free < need + self.reserve:
            return f'memory ({free / 2**30:.1f} GB free, {(need + self.reserve) / 2**30:.1f} GB needed)'
        return None

    def acquire(self, kind, label, need=0):
        """Waits until the job may start; returns the lock file to pass to release()."""
        told = None
        while True:
            m = self._mutex()
            try:
                reason = self._reason(kind, need)
                if reason is None:
                    lock = self.folder / f"{kind}-{os.getpid()}-{time.time_ns()}.lock"
                    lock.write_text(json.dumps({"pid": os.getpid(), "kind": kind, "label": label, "since": time.time(), "need": need}))
                    return lock
            finally:
                m.rmdir()
            if reason != told:
                self.say(f"gate: {label} waits for {reason}")
                told = reason
            time.sleep(2.0)

    @staticmethod
    def release(lock):
        Path(lock).unlink(missing_ok=True)


def time_limit_windows(log_text):
    """How many windows the verdict summary says stopped at the time limit (None without a summary)."""
    for line in log_text.splitlines():
        if "window verdict summary: iterations stopped:" in line and "time limit " in line:
            return int(line.split("time limit ")[1].split(",")[0])
    return None


def gated_replay(paths, capture, settings, log_path, label, say=print):
    """A replay through the gate. Returns the cost of run_dmapper plus the replay's summary lines and time-limit count."""
    gate = Gate(paths, say)
    lock = gate.acquire("replay", label, gate.need_bytes(capture))
    try:
        cost = run_dmapper(["--replay", capture, settings], log_path)
    finally:
        gate.release(lock)
    text = Path(log_path).read_text(errors="replace")
    cost["summary"] = [l.split("replay: ", 1)[1] for l in text.splitlines() if "replay: " in l]
    cost["time_limit"] = time_limit_windows(text)
    return cost
