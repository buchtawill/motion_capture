#!/usr/bin/env python3

import subprocess
import psutil
import time
import statistics
import sys
from datetime import datetime
from pathlib import Path

SAMPLE_INTERVAL = 5  # seconds
VIVADO_CMD = ["vivado", "-mode", "batch", "-s", "export_bitstream.tcl"]
LOG_FILE = f"vivado_{datetime.now():%Y%m%d_%H%M%S}.log"


def bytes_to_gb(b):
    return b / (1024 ** 3)


def find_vivado_process(parent_pid):
    try:
        parent = psutil.Process(parent_pid)
        procs = [parent] + parent.children(recursive=True)
        return procs
    except psutil.NoSuchProcess:
        return []


def sample_ram(parent_pid):
    procs = find_vivado_process(parent_pid)

    process_ram = 0
    for p in procs:
        try:
            process_ram += p.memory_info().rss
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    system = psutil.virtual_memory()
    return {
        "process_ram_bytes":  process_ram,
        "system_used_bytes":  system.used,
        "system_total_bytes": system.total,
        "system_percent":     system.percent,
        "timestamp":          time.time(),
    }


def print_summary(samples, elapsed):
    if not samples:
        print("No samples collected.")
        return

    proc_samples = [s["process_ram_bytes"] for s in samples]
    sys_samples  = [s["system_used_bytes"]  for s in samples]
    sys_pct      = [s["system_percent"]     for s in samples]
    total_bytes  = samples[0]["system_total_bytes"]

    print("\n" + "=" * 60)
    print("  VIVADO RAM USAGE REPORT")
    print("=" * 60)

    print(f"\n  Build duration:        {elapsed:.1f}s  ({elapsed/60:.1f} min)")
    print(f"  Samples collected:     {len(samples)}")
    print(f"  Sample interval:       {SAMPLE_INTERVAL}s")
    print(f"  System total RAM:      {bytes_to_gb(total_bytes):.2f} GB")
    print(f"  Vivado log:            {LOG_FILE}")

    print(f"\n  --- Vivado Process RAM ---")
    print(f"  Peak:                  {bytes_to_gb(max(proc_samples)):.2f} GB")
    print(f"  Average:               {bytes_to_gb(statistics.mean(proc_samples)):.2f} GB")
    print(f"  Median:                {bytes_to_gb(statistics.median(proc_samples)):.2f} GB")
    print(f"  Min:                   {bytes_to_gb(min(proc_samples)):.2f} GB")
    if len(proc_samples) > 1:
        print(f"  Std dev:               {bytes_to_gb(statistics.stdev(proc_samples)):.2f} GB")

    print(f"\n  --- System RAM (total used) ---")
    print(f"  Peak:                  {bytes_to_gb(max(sys_samples)):.2f} GB  ({max(sys_pct):.1f}%)")
    print(f"  Average:               {bytes_to_gb(statistics.mean(sys_samples)):.2f} GB  ({statistics.mean(sys_pct):.1f}%)")
    print(f"  Min:                   {bytes_to_gb(min(sys_samples)):.2f} GB  ({min(sys_pct):.1f}%)")

    headroom = total_bytes - max(proc_samples)
    print(f"\n  --- Headroom at Peak Vivado Usage ---")
    print(f"  Free (approx):         {bytes_to_gb(headroom):.2f} GB")
    print(f"  Vivado % of total:     {(max(proc_samples)/total_bytes)*100:.1f}%")
    print("=" * 60)


def main():
    print(f"[{datetime.now():%H:%M:%S}] Starting Vivado...")
    print(f"  Command: {' '.join(VIVADO_CMD)}")
    print(f"  Vivado output → {LOG_FILE}")
    print(f"  Sampling every {SAMPLE_INTERVAL}s\n")

    start_time = time.time()
    samples = []

    try:
        log_fh = open(LOG_FILE, "w")
    except OSError as e:
        print(f"[ERROR] Could not open log file {LOG_FILE}: {e}")
        sys.exit(1)

    try:
        proc = subprocess.Popen(
            VIVADO_CMD,
            stdout=log_fh,
            stderr=log_fh,
        )
    except FileNotFoundError:
        print("[ERROR] 'vivado' not found. Make sure Vivado is on your PATH.")
        log_fh.close()
        sys.exit(1)

    print(f"[{datetime.now():%H:%M:%S}] Vivado PID: {proc.pid}\n")

    try:
        while proc.poll() is None:
            sample = sample_ram(proc.pid)
            samples.append(sample)

            elapsed = time.time() - start_time
            print(
                f"[{datetime.now():%H:%M:%S}]  "
                f"Vivado: {bytes_to_gb(sample['process_ram_bytes']):5.2f} GB  |  "
                f"System: {bytes_to_gb(sample['system_used_bytes']):5.2f} GB  "
                f"({sample['system_percent']:.1f}%)  |  "
                f"Elapsed: {elapsed/60:.1f} min"
            )

            time.sleep(SAMPLE_INTERVAL)

        sample = sample_ram(proc.pid)
        if sample["process_ram_bytes"] > 0:
            samples.append(sample)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Killing Vivado...")
        proc.terminate()
        proc.wait()

    finally:
        log_fh.close()

    elapsed = time.time() - start_time
    rc = proc.returncode
    status = "SUCCESS" if rc == 0 else f"FAILED (exit code {rc})"
    print(f"\n[{datetime.now():%H:%M:%S}] Vivado finished — {status}")

    print_summary(samples, elapsed)


if __name__ == "__main__":
    main()