#!/usr/bin/env python3

from pathlib import Path


LAUNCH_FILES = [
    Path('launch/safe_scout.launch.py'),
    Path('launch/safe_bayesian_optimization.launch.py'),
]


def main():
    stale_package = 'safe_' + 'legged_scouting'
    bad_files = [str(path) for path in LAUNCH_FILES
                 if stale_package in path.read_text()]
    if bad_files:
        raise SystemExit('stale package reference: ' + ','.join(bad_files))


if __name__ == '__main__':
    main()
