#!/usr/bin/env python3
"""
Simple map saver — calls slam_toolbox's save map service,
then also saves via nav2 map_saver_cli for a standard .pgm/.yaml pair.
Usage:
  ros2 run amr_slam save_map
  ros2 run amr_slam save_map -- --map-name my_map
"""

import subprocess
import sys
import os
from datetime import datetime


def main():
    map_name = 'map'
    # Check for --map-name argument
    args = sys.argv[1:]
    for i, a in enumerate(args):
        if a == '--map-name' and i + 1 < len(args):
            map_name = args[i + 1]

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_name = f'{map_name}_{timestamp}'
    output_dir = os.path.expanduser('~/maps')
    os.makedirs(output_dir, exist_ok=True)

    output_path = os.path.join(output_dir, output_name)

    print(f'Saving map to: {output_path}')

    # Use nav2_map_server's map_saver_cli
    try:
        result = subprocess.run(
            [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', output_path,
                '--ros-args', '-p', 'save_map_timeout:=5.0'
            ],
            timeout=30,
            capture_output=True,
            text=True
        )
        print(result.stdout)
        if result.returncode == 0:
            print(f'Map saved successfully!')
            print(f'  PGM: {output_path}.pgm')
            print(f'  YAML: {output_path}.yaml')
        else:
            print(f'map_saver_cli returned non-zero: {result.stderr}')
    except subprocess.TimeoutExpired:
        print('Map save timed out')
    except FileNotFoundError:
        print('nav2_map_server not found. Install with:')
        print('  sudo apt install ros-<distro>-nav2-map-server')


if __name__ == '__main__':
    main()
