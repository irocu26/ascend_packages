"""
Mission Executor - ROS2 node orchestrating FSM and 10 Hz control loop.
Processes mission commands and evaluates transition thresholds.
"""

import logging
from typing import List, Optional
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from .states import DroneState, MissionEvent, MissionPlan, MissionWaypoint, GPSPoint
from .fsm import FlightFSM
from .fcu_bridge import FCUBridge
from .geo_utils import haversine_distance, is_within_radius

logger = logging.getLogger(__name__)


class MissionExecutor(Node):
    """
    Orchestrates autonomous missions via FSM.
    Runs 10 Hz control loop, evaluates thresholds, dispatches events.
    """

    def __init__(self, fcu_bridge: FCUBridge):
        super().__init__('mission_executor')

        self.fcu_bridge = fcu_bridge
        self.fsm = FlightFSM()
        
        # Mission state
        self.mission_plan: Optional[MissionPlan] = None
        self.current_waypoint_idx = 0
        self.mission_active = False
        self.waypoint_start_time = None

        # Parameters
        self.declare_parameter('control_loop_rate', 10.0)  # Hz
        self.declare_parameter('altitude_threshold', 0.5)  # m
        self.declare_parameter('gps_distance_threshold', 2.0)  # m
        self.declare_parameter('gps_accuracy_threshold', 5.0)  # m

        self.control_rate = self.get_parameter('control_loop_rate').value
        self.altitude_threshold = self.get_parameter('altitude_threshold').value
        self.distance_threshold = self.get_parameter('gps_distance_threshold').value
        self.gps_accuracy_threshold = self.get_parameter('gps_accuracy_threshold').value

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ======================== Publishers ========================
        self.mission_status_pub = self.create_publisher(
            String, '/ascend/mission/status', qos
        )
        self.position_pub = self.create_publisher(
            PoseStamped, '/ascend/mission/position', qos
        )

        # ======================== Subscribers ========================
        self.cmd_sub = self.create_subscription(
            String, '/ascend/mission/cmd',
            self._on_mission_command, qos
        )

        # ======================== Setup FSM Callbacks ========================
        self._setup_fsm_callbacks()

        # ======================== Control Loop Timer ========================
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_loop)

        logger.info(f"Mission Executor initialized ({self.control_rate} Hz)")

    def _setup_fsm_callbacks(self):
        """Register FSM state callbacks."""
        self.fsm.register_enter_callback(DroneState.ARMED, self._on_enter_armed)
        self.fsm.register_enter_callback(DroneState.TAKEOFF, self._on_enter_takeoff)
        self.fsm.register_enter_callback(DroneState.HOVER, self._on_enter_hover)
        self.fsm.register_enter_callback(DroneState.MISSION, self._on_enter_mission)
        self.fsm.register_enter_callback(DroneState.LAND, self._on_enter_land)

    def _on_enter_armed(self, context: dict):
        """Callback: entering ARMED state."""
        logger.info(">> ARMED")
        if not self.fcu_bridge.has_gps_fix():
            logger.error("No GPS fix!")
            self.fsm.process_event(MissionEvent.EV_ERROR)

    def _on_enter_takeoff(self, context: dict):
        """Callback: entering TAKEOFF state."""
        logger.info(f">> TAKEOFF to {self.mission_plan.takeoff_altitude}m")
        self.fcu_bridge.takeoff(self.mission_plan.takeoff_altitude)

    def _on_enter_hover(self, context: dict):
        """Callback: entering HOVER state."""
        logger.info(">> HOVER")
        self.fcu_bridge.hover()
        self.waypoint_start_time = self.get_clock().now()

    def _on_enter_mission(self, context: dict):
        """Callback: entering MISSION state."""
        logger.info(">> MISSION")
        self.mission_active = True

    def _on_enter_land(self, context: dict):
        """Callback: entering LAND state."""
        logger.info(">> LAND")
        self.fcu_bridge.land()

    def _on_mission_command(self, msg: String):
        """
        Handle mission commands from ROS topic.
        Commands: ARM, DISARM, TAKEOFF, LAND, EMERGENCY_LAND, START_MISSION
        """
        cmd = msg.data.strip().upper()
        logger.info(f"Mission command: {cmd}")

        try:
            if cmd == "ARM":
                if self.fsm.can_transition(MissionEvent.EV_ARM):
                    self.fcu_bridge.set_home()
                    self.fcu_bridge.arm()
                    self.fsm.process_event(MissionEvent.EV_ARM)
                else:
                    logger.warning(f"Cannot ARM from {self.fsm.get_state().value}")

            elif cmd == "DISARM":
                self.fcu_bridge.disarm()
                self.fsm.process_event(MissionEvent.EV_DISARM)

            elif cmd == "TAKEOFF":
                if self.fsm.can_transition(MissionEvent.EV_TAKEOFF_START):
                    self.fsm.process_event(MissionEvent.EV_TAKEOFF_START)
                else:
                    logger.warning(f"Cannot TAKEOFF from {self.fsm.get_state().value}")

            elif cmd == "LAND":
                if self.fsm.can_transition(MissionEvent.EV_LAND_START):
                    self.fsm.process_event(MissionEvent.EV_LAND_START)
                else:
                    logger.warning(f"Cannot LAND from {self.fsm.get_state().value}")

            elif cmd == "EMERGENCY_LAND":
                self.fsm.process_event(MissionEvent.EV_EMERGENCY_LAND)

            elif cmd == "START_MISSION":
                if self.mission_plan:
                    self.fsm.process_event(MissionEvent.EV_MISSION_START)
                else:
                    logger.error("No mission plan loaded")

            else:
                logger.warning(f"Unknown command: {cmd}")

        except Exception as e:
            logger.error(f"Command processing error: {e}")

    def _control_loop(self):
        """
        10 Hz control loop - evaluates thresholds and dispatches events.
        """
        if not self.mission_active:
            return

        try:
            telemetry = self.fcu_bridge.get_telemetry()
            current_state = self.fsm.get_state()

            # Publish telemetry
            self._publish_status(current_state, telemetry)

            # State-specific evaluations
            if current_state == DroneState.TAKEOFF:
                self._eval_takeoff(telemetry)
            elif current_state == DroneState.HOVER:
                self._eval_hover(telemetry)
            elif current_state == DroneState.MISSION:
                self._eval_mission(telemetry)
            elif current_state == DroneState.LAND:
                self._eval_land(telemetry)

        except Exception as e:
            logger.error(f"Control loop error: {e}")
            self.fsm.process_event(MissionEvent.EV_ERROR)

    def _eval_takeoff(self, telemetry: dict):
        """Evaluate takeoff state - trigger ALTITUDE_REACHED when target reached."""
        alt_agl = telemetry["altitude_agl"]
        target_alt = self.mission_plan.takeoff_altitude

        if alt_agl >= target_alt - self.altitude_threshold:
            logger.info(f"Altitude reached: {alt_agl:.1f}m >= {target_alt:.1f}m")
            self.fsm.process_event(MissionEvent.EV_ALTITUDE_REACHED)

    def _eval_hover(self, telemetry: dict):
        """Evaluate hover state - trigger HOVER_COMPLETE after duration."""
        if not self.waypoint_start_time:
            return

        elapsed = (self.get_clock().now() - self.waypoint_start_time).nanoseconds / 1e9
        hover_duration = (
            self.mission_plan.waypoints[self.current_waypoint_idx].hover_duration
            if self.current_waypoint_idx < len(self.mission_plan.waypoints)
            else 0.0
        )

        if elapsed >= hover_duration:
            logger.info(f"Hover complete ({elapsed:.1f}s)")
            self.fsm.process_event(MissionEvent.EV_HOVER_COMPLETE)

    def _eval_mission(self, telemetry: dict):
        """Evaluate mission state - navigate waypoints, trigger events."""
        if self.current_waypoint_idx >= len(self.mission_plan.waypoints):
            logger.info("All waypoints completed")
            self.fsm.process_event(MissionEvent.EV_MISSION_COMPLETE)
            return

        waypoint = self.mission_plan.waypoints[self.current_waypoint_idx]
        current_pos = telemetry["position"]

        # Check if reached waypoint
        distance = haversine_distance(
            current_pos.latitude, current_pos.longitude,
            waypoint.position.latitude, waypoint.position.longitude
        )

        if distance <= self.distance_threshold:
            logger.info(f"Waypoint {self.current_waypoint_idx} reached: {waypoint.name}")
            self.current_waypoint_idx += 1

            if self.current_waypoint_idx < len(self.mission_plan.waypoints):
                self.fsm.process_event(MissionEvent.EV_WAYPOINT_REACHED)
            else:
                self.fsm.process_event(MissionEvent.EV_MISSION_COMPLETE)
        else:
            # Command movement to waypoint
            self._move_to_waypoint(waypoint)

    def _eval_land(self, telemetry: dict):
        """Evaluate land state - trigger LANDED when altitude near zero."""
        alt_agl = telemetry["altitude_agl"]

        if alt_agl <= 0.5:
            logger.info("Landed")
            self.mission_active = False
            self.fsm.process_event(MissionEvent.EV_LANDED)
            self.fcu_bridge.disarm()

    def _move_to_waypoint(self, waypoint: MissionWaypoint):
        """Send movement command to waypoint."""
        current_pos = self.fcu_bridge.get_gps_position()
        
        # Simple approach: calculate NED offset and command it
        dlat = (waypoint.position.latitude - current_pos.latitude) * 111000
        dlon = (waypoint.position.longitude - current_pos.longitude) * 111000 * math.cos(
            math.radians(current_pos.latitude)
        )
        dalt = waypoint.position.altitude - current_pos.altitude

        self.fcu_bridge.move_to_local_offset(dlat, dlon, -dalt, speed=waypoint.speed)

    def _publish_status(self, state: DroneState, telemetry: dict):
        """Publish mission status."""
        status_msg = String()
        status_msg.data = (
            f"state={state.value},"
            f"waypoint={self.current_waypoint_idx}/{len(self.mission_plan.waypoints) if self.mission_plan else 0},"
            f"alt={telemetry['altitude_agl']:.1f}m,"
            f"gps_fix={telemetry['gps_fix']}"
        )
        self.mission_status_pub.publish(status_msg)

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pos = telemetry["position"]
        pose_msg.pose.position.x = pos.longitude
        pose_msg.pose.position.y = pos.latitude
        pose_msg.pose.position.z = telemetry["altitude_agl"]
        self.position_pub.publish(pose_msg)

    def load_mission_from_file(self, filepath: str) -> bool:
        """Load mission plan from YAML file."""
        try:
            import yaml
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            home = GPSPoint(
                data['home']['latitude'],
                data['home']['longitude'],
                data['home']['altitude']
            )
            
            mission = MissionPlan(
                name=data['name'],
                home=home,
                takeoff_altitude=data['takeoff_altitude'],
                mission_speed=data.get('mission_speed', 5.0),
                waypoint_reach_radius=data.get('waypoint_reach_radius', 2.0)
            )
            
            for wp_data in data['waypoints']:
                wp = MissionWaypoint(
                    name=wp_data['name'],
                    position=GPSPoint(
                        wp_data['position']['latitude'],
                        wp_data['position']['longitude'],
                        wp_data['position']['altitude']
                    ),
                    hover_duration=wp_data.get('hover_duration', 0.0),
                    speed=wp_data.get('speed', 5.0)
                )
                mission.add_waypoint(wp)
            
            self.mission_plan = mission
            logger.info(f"Mission loaded: {mission}")
            return True

        except Exception as e:
            logger.error(f"Failed to load mission: {e}")
            return False

    def get_mission_status(self) -> dict:
        """Get current mission status."""
        telemetry = self.fcu_bridge.get_telemetry()
        return {
            "state": self.fsm.get_state().value,
            "mission_active": self.mission_active,
            "waypoint": self.current_waypoint_idx,
            "total_waypoints": len(self.mission_plan.waypoints) if self.mission_plan else 0,
            "position": telemetry["position"],
            "altitude_agl": telemetry["altitude_agl"],
            "armed": telemetry["armed"],
            "gps_fix": telemetry["gps_fix"],
        }
    
    def main(args=None):

        rclpy.init(args=args)

        node = MissionExecutor()

        try:
            rclpy.spin(node)

        except KeyboardInterrupt:
            pass

        node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()


    