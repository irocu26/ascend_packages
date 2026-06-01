"""
Main entry point - starts both FCUBridge and MissionExecutor nodes together.
"""

import logging
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .fcu_bridge import FCUBridge
from .mission_executor import MissionExecutor

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main(args=None):
    """Launch mission control system."""
    rclpy.init(args=args)

    try:
        # Create nodes
        fcu_bridge = FCUBridge()
        mission_executor = MissionExecutor(fcu_bridge)

        # Load mission from parameter (if provided)
        mission_file = mission_executor.get_parameter('mission_file').value if mission_executor.has_parameter('mission_file') else None
        if mission_file:
            mission_executor.load_mission_from_file(mission_file)
            logger.info(f"Mission loaded from {mission_file}")

        # Spin with multi-threaded executor
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(fcu_bridge)
        executor.add_node(mission_executor)

        logger.info("ASCEND Mission Control started")
        executor.spin()

    except KeyboardInterrupt:
        logger.info("Shutdown requested")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        return 1
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())