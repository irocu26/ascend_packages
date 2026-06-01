"""
Pure-Python FSM engine - zero ROS2 dependencies, fully unit-testable.
Evaluates state transitions and manages state callbacks.
"""

import logging
from typing import Callable, Dict, Optional, Tuple
from .states import DroneState, MissionEvent, TRANSITIONS

logger = logging.getLogger(__name__)


class FlightFSM:
    """Finite State Machine for autonomous flight control."""

    def __init__(self):
        self.state: DroneState = DroneState.IDLE
        self.previous_state: Optional[DroneState] = None
        
        # Callbacks
        self.on_enter_callbacks: Dict[DroneState, Callable] = {}
        self.on_exit_callbacks: Dict[DroneState, Callable] = {}
        self.on_transition_callbacks: Dict[Tuple[DroneState, DroneState], Callable] = {}
        
        # Event queue for batch processing
        self.event_queue = []

    def register_enter_callback(self, state: DroneState, callback: Callable):
        """Register callback when entering a state."""
        self.on_enter_callbacks[state] = callback
        logger.debug(f"Registered enter callback for {state.value}")

    def register_exit_callback(self, state: DroneState, callback: Callable):
        """Register callback when exiting a state."""
        self.on_exit_callbacks[state] = callback
        logger.debug(f"Registered exit callback for {state.value}")

    def register_transition_callback(
        self, from_state: DroneState, to_state: DroneState, callback: Callable
    ):
        """Register callback for specific state transition."""
        self.on_transition_callbacks[(from_state, to_state)] = callback
        logger.debug(f"Registered transition callback {from_state.value} -> {to_state.value}")

    def process_event(self, event: MissionEvent, context: Optional[Dict] = None) -> bool:
        """
        Process a mission event and attempt state transition.
        
        Args:
            event: The mission event to process
            context: Optional context data for callbacks
        
        Returns:
            True if transition occurred, False otherwise
        """
        context = context or {}
        transition_key = (self.state, event)
        
        if transition_key not in TRANSITIONS:
            logger.warning(f"Invalid transition: {self.state.value} + {event.value}")
            return False
        
        new_state = TRANSITIONS[transition_key]
        self._transition_to(new_state, context)
        return True

    def _transition_to(self, new_state: DroneState, context: Dict = None):
        """Internal transition handler."""
        context = context or {}
        
        if new_state == self.state:
            return
        
        old_state = self.state
        logger.info(f"FSM: {old_state.value} -> {new_state.value}")
        
        # Call exit callback for old state
        if old_state in self.on_exit_callbacks:
            try:
                self.on_exit_callbacks[old_state](context)
            except Exception as e:
                logger.error(f"Exit callback error: {e}")
        
        # Update state
        self.previous_state = old_state
        self.state = new_state
        
        # Call transition callback
        transition_key = (old_state, new_state)
        if transition_key in self.on_transition_callbacks:
            try:
                self.on_transition_callbacks[transition_key](context)
            except Exception as e:
                logger.error(f"Transition callback error: {e}")
        
        # Call enter callback for new state
        if new_state in self.on_enter_callbacks:
            try:
                self.on_enter_callbacks[new_state](context)
            except Exception as e:
                logger.error(f"Enter callback error: {e}")

    def get_state(self) -> DroneState:
        """Get current state."""
        return self.state

    def get_previous_state(self) -> Optional[DroneState]:
        """Get previous state."""
        return self.previous_state

    def is_in_state(self, state: DroneState) -> bool:
        """Check if in specific state."""
        return self.state == state

    def can_transition(self, event: MissionEvent) -> bool:
        """Check if an event can trigger a transition from current state."""
        return (self.state, event) in TRANSITIONS

    def __str__(self) -> str:
        return f"FSM(state={self.state.value})"

    def __repr__(self) -> str:
        return str(self)