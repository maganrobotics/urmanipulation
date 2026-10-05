"""Run every Community demo in a separate process; fail on any broken demo."""
import argparse
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

DEMOS = ('quick_start', 'ur3e_single_arm', 'grasping', 'motion_planning',
         'dual_arm', 'mobile_dual_arm', 'integrated_task')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir', type=Path, default=Path('community-results'))
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for demo in DEMOS:
        destination = output / (demo + '.json')
        # Never count an old successful result after a failed run.
        destination.unlink(missing_ok=True)
        cmd = [sys.executable, '-m', 'community.' + demo, '--headless', '--output', str(destination)]
        if demo == 'integrated_task':
            cmd += ['--plan-output', str(output / 'ros2-playback.json')]
        started = time.monotonic()
        try:
            run = subprocess.run(cmd, cwd=root, capture_output=True, text=True, timeout=120)
            (output / (demo + '.log')).write_text(run.stdout + run.stderr, encoding='utf-8')
            if run.returncode or not destination.exists():
                raise RuntimeError(f'exit={run.returncode}; see {demo}.log')
            result = json.loads(destination.read_text())
            if result.get('status') != 'passed':
                raise RuntimeError('Demo did not report success')
            result['elapsed_s'] = round(time.monotonic() - started, 2)
        except (RuntimeError, subprocess.TimeoutExpired, ValueError) as exc:
            result = dict(demo=demo, status='failed', error=str(exc))
        results.append(result)
        print(f"{demo}: {result['status']}", flush=True)
    summary = dict(python=sys.version.split()[0], platform=platform.platform(), results=results,
                   status='passed' if all(r['status'] == 'passed' for r in results) else 'failed',
                   ros2_transport='run community.ros2_verify separately on a sourced ROS2 host',
                   hardware='not tested')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    if summary['status'] != 'passed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
