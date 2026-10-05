#!/usr/bin/env python3
"""Cache /map and write pgm+yaml when the node shuts down (Ctrl+C)."""

import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapAutosave(Node):

    def __init__(self):
        super().__init__('map_autosave')

        default_stem = os.path.join(
            os.path.expanduser('~'),
            'maps',
            'cartographer_map',
        )
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_filestem', default_stem)

        self.map_topic = self.get_parameter('map_topic').value
        self.map_filestem = os.path.expanduser(
            self.get_parameter('map_filestem').value
        )
        self._latest_map = None

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            10,
        )

        self.get_logger().info(
            f'Caching maps from {self.map_topic}; '
            f'will save to {self.map_filestem}.{{pgm,yaml}} on shutdown'
        )

    def _map_callback(self, msg: OccupancyGrid):
        self._latest_map = msg

    def save(self):
        if self._latest_map is None:
            self.get_logger().warn('No map received yet; nothing to save')
            return

        out_dir = os.path.dirname(self.map_filestem)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        # Avoid clobbering previous maps if the stem already exists
        stem = self.map_filestem
        if os.path.exists(stem + '.yaml') or os.path.exists(stem + '.pgm'):
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            stem = f'{self.map_filestem}_{stamp}'

        pgm_path = stem + '.pgm'
        yaml_path = stem + '.yaml'
        pgm_name = os.path.basename(pgm_path)

        grid = self._latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin = grid.info.origin

        # ROS map_saver pixel convention
        pixels = bytearray(width * height)
        for y in range(height):
            for x in range(width):
                value = grid.data[x + y * width]
                if value < 0:
                    pixel = 205
                elif value == 0:
                    pixel = 254
                elif value >= 100:
                    pixel = 0
                else:
                    pixel = int(254 - (value * 254 / 100))
                # PGM is top-down; OccupancyGrid is bottom-up
                pixels[x + (height - 1 - y) * width] = pixel

        with open(pgm_path, 'wb') as pgm:
            pgm.write(
                f'P5\n# CREATOR: amr_mapping map_autosave\n'
                f'{width} {height}\n255\n'.encode('ascii')
            )
            pgm.write(pixels)

        with open(yaml_path, 'w', encoding='utf-8') as yml:
            yml.write(f'image: {pgm_name}\n')
            yml.write('mode: trinary\n')
            yml.write(f'resolution: {resolution}\n')
            yml.write(
                'origin: '
                f'[{origin.position.x}, {origin.position.y}, 0.0]\n'
            )
            yml.write('negate: 0\n')
            yml.write('occupied_thresh: 0.65\n')
            yml.write('free_thresh: 0.25\n')

        self.get_logger().info(f'Saved map to {yaml_path} and {pgm_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MapAutosave()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f'Failed to save map: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
