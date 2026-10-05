"""Actual ROS2 action success/failure checks against isolated mock processes."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, default=Path('community-results/ros2'))
    p.add_argument('--shared-memory', action='store_true',
                   help='Use Fast DDS shared memory only, with no UDP/TCP transport')
    args = p.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    outcomes = []
    for i, (flag, expected) in enumerate([(None, 'passed'), ('--fail-navigation', 'failed'),
                                         ('--fail-manipulation', 'failed')]):
        env = dict(os.environ, ROS_DOMAIN_ID=str(81 + i), ROS_LOCALHOST_ONLY='1')
        if args.shared_memory:
            # Humble's localhost flag installs a UDP transport. The explicit
            # SHM-only profile instead enforces same-host communication by IPC.
            env.update(ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                       FASTRTPS_DEFAULT_PROFILES_FILE=str(Path(__file__).with_name('fastdds_shm.xml').resolve()))
        server_cmd = [sys.executable, '-m', 'community.ros2_bridge', 'mock-server']
        if flag:
            server_cmd.append(flag)
        with (args.output_dir / f'server-{i}.log').open('w') as log:
            server = subprocess.Popen(server_cmd, env=env, stdout=log, stderr=log)
            try:
                client = subprocess.run([sys.executable, '-m', 'community.ros2_bridge', 'client',
                                         '--plan', str(args.plan.resolve()), '--timeout', '15'],
                                        env=env, capture_output=True, text=True, timeout=70)
                (args.output_dir / f'client-{i}.log').write_text(client.stdout + client.stderr)
                records = [json.loads(line) for line in client.stdout.splitlines() if line.startswith('{')]
                if not records:
                    raise RuntimeError(f'No ROS2 result for case {i}; inspect logs')
                result = records[-1]
                if result['status'] != expected or ((client.returncode == 0) != (expected == 'passed')):
                    raise RuntimeError(f'Unexpected result for case {i}: {result}')
                history = result['task_history']
                if flag == '--fail-navigation' and ('manipulating' in history or history[-1] != 'failed'):
                    raise RuntimeError('Navigation failure did not block manipulation')
                if flag == '--fail-manipulation' and ('manipulating' not in history or history[-1] != 'failed'):
                    raise RuntimeError('Manipulation failure was not propagated')
                outcomes.append(dict(case=flag or 'success', result=result))
            finally:
                server.terminate()
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait()
    (args.output_dir / 'summary.json').write_text(json.dumps(outcomes, indent=2) + '\n')
    print('ROS2 transport: 3/3 cases passed')


if __name__ == '__main__':
    main()
