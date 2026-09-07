"""ROS-independent Team 5 pit-cycle state machine."""


NORMAL_DRIVE = 'NORMAL_DRIVE'
WAIT_FOR_BATTERY = 'WAIT_FOR_BATTERY'
PRE_PIT_STOP = 'PRE_PIT_STOP'
ENTER_PIT = 'ENTER_PIT'
PIT_DRIVE = 'PIT_DRIVE'
PIT_DWELL = 'PIT_DWELL'
EXIT_PIT = 'EXIT_PIT'

NORMAL_MODE = 'NORMAL'
PIT_MODE = 'PIT'
EXIT_MODE = 'EXIT_PIT'

BATTERY_UNKNOWN = 'UNKNOWN'
BATTERY_HEALTHY = 'HEALTHY'
BATTERY_LOW = 'LOW'


class PitStateMachine:
    """Coordinate a battery-triggered, safely rearmed pit cycle."""

    def __init__(
        self,
        pre_pit_stop_duration=4.0,
        pit_drive_duration=10.0,
        pit_dwell_duration=5.0,
        sign_clear_time=1.0,
        acquisition_confirmation_frames=3,
        lane_status_timeout=0.5,
        transition_drive_timeout=5.0,
        battery_status_timeout=2.5,
        pit_lane_enabled=False,
    ):
        self.pre_pit_stop_duration = float(pre_pit_stop_duration)
        self.pit_drive_duration = float(pit_drive_duration)
        self.pit_dwell_duration = float(pit_dwell_duration)
        self.sign_clear_time = float(sign_clear_time)
        self.acquisition_confirmation_frames = int(
            acquisition_confirmation_frames
        )
        self.lane_status_timeout = float(lane_status_timeout)
        self.transition_drive_timeout = float(transition_drive_timeout)
        self.battery_status_timeout = float(battery_status_timeout)
        self.pit_lane_enabled = bool(pit_lane_enabled)

        self.state = NORMAL_DRIVE
        self.state_entered_at = 0.0
        self.battery_status_received = False
        self.battery_low = False
        self.last_battery_status_time = None
        self.stop_detected = False
        self.trigger_armed = True
        self.clear_started_at = None

        self.acquisition_count = 0
        self.last_lane_acquired = False
        self.last_lane_status_time = None
        self.last_lane_acquired_time = None
        self.pit_drive_elapsed = 0.0
        self.last_advance_time = None

    def update_battery(self, is_low, now):
        """Update battery condition and evaluate the pit trigger."""
        self.battery_low = bool(is_low)
        self.battery_status_received = True
        self.last_battery_status_time = float(now)
        if self.state == WAIT_FOR_BATTERY:
            if self.battery_low:
                self._transition(PRE_PIT_STOP, now)
            else:
                self._transition(NORMAL_DRIVE, now)
        self._maybe_start_cycle(float(now))

    def update_stop_detection(self, detected, now):
        """Update STOP detection, latching a qualifying rising event."""
        self.stop_detected = bool(detected)
        if self.stop_detected:
            self.clear_started_at = None
        self._maybe_start_cycle(float(now))

    def observe_lane_status(self, mode, acquired, now):
        """Record acquisition frames only when tagged for the active mode."""
        if str(mode).upper() != self.lane_mode:
            return False

        now = float(now)
        if (
            self.last_lane_status_time is not None
            and now - self.last_lane_status_time > self.lane_status_timeout
        ):
            self.acquisition_count = 0

        self.last_lane_acquired = bool(acquired)
        self.last_lane_status_time = now
        if self.last_lane_acquired:
            self.acquisition_count += 1
            self.last_lane_acquired_time = now
        else:
            self.acquisition_count = 0
        return True

    def advance(self, now):
        """Advance deterministic timer/acquisition transitions."""
        now = float(now)

        if self.state == NORMAL_DRIVE:
            self._update_rearm(now)
            self._maybe_start_cycle(now)

        elif self.state == WAIT_FOR_BATTERY:
            battery_status = self.battery_status(now)
            if battery_status == BATTERY_LOW:
                self._transition(PRE_PIT_STOP, now)
            elif battery_status == BATTERY_HEALTHY:
                self._transition(NORMAL_DRIVE, now)

        elif self.state == PRE_PIT_STOP:
            if now - self.state_entered_at >= self.pre_pit_stop_duration:
                self._transition(ENTER_PIT, now)

        elif self.state == ENTER_PIT:
            if (
                self.pit_lane_enabled
                and self.acquisition_count
                >= self.acquisition_confirmation_frames
                and self.lane_is_acquired(now)
            ):
                self._transition(PIT_DRIVE, now)

        elif self.state == PIT_DRIVE:
            if self.last_advance_time is None:
                self.last_advance_time = now
            elapsed = max(0.0, now - self.last_advance_time)
            self.last_advance_time = now
            if self.lane_is_acquired(now):
                self.pit_drive_elapsed += elapsed
            if self.pit_drive_elapsed >= self.pit_drive_duration:
                self._transition(PIT_DWELL, now)

        elif self.state == PIT_DWELL:
            if now - self.state_entered_at >= self.pit_dwell_duration:
                self._transition(EXIT_PIT, now)

        elif self.state == EXIT_PIT:
            enough_acquisition_frames = (
                self.acquisition_count
                >= self.acquisition_confirmation_frames
                and self.lane_is_acquired(now)
            )
            if enough_acquisition_frames:
                self._transition(NORMAL_DRIVE, now)
                self.trigger_armed = False
                self.clear_started_at = None

        return self.state

    def lane_is_acquired(self, now):
        """Return acquisition only while its mode-tagged status is fresh."""
        return (
            self.last_lane_acquired
            and self.last_lane_status_time is not None
            and float(now) - self.last_lane_status_time
            <= self.lane_status_timeout
        )

    def motion_allowed(self, now):
        """Allow motion only in moving states and bounded transitions."""
        if self.state in (WAIT_FOR_BATTERY, PRE_PIT_STOP, PIT_DWELL):
            return False
        if self.state == ENTER_PIT and not self.pit_lane_enabled:
            return False
        if self.state == PIT_DRIVE:
            return self.lane_was_recently_acquired(now)
        if self.state in (ENTER_PIT, EXIT_PIT):
            transition_age = float(now) - self.state_entered_at
            if (
                transition_age >= self.transition_drive_timeout
                and not self.lane_is_acquired(now)
            ):
                return False
        return True

    def lane_was_recently_acquired(self, now):
        """Provide the same short grace period after pit-lane loss."""
        return (
            self.last_lane_acquired_time is not None
            and float(now) - self.last_lane_acquired_time
            <= self.lane_status_timeout
        )

    @property
    def lane_mode(self):
        """Return the detector/guidance mode required by the current state."""
        if self.state in (ENTER_PIT, PIT_DRIVE, PIT_DWELL):
            return PIT_MODE
        if self.state == EXIT_PIT:
            return EXIT_MODE
        return NORMAL_MODE

    def battery_status(self, now):
        """Return LOW, HEALTHY, or UNKNOWN using telemetry freshness."""
        if (
            not self.battery_status_received
            or self.last_battery_status_time is None
            or float(now) - self.last_battery_status_time
            > self.battery_status_timeout
        ):
            return BATTERY_UNKNOWN
        return BATTERY_LOW if self.battery_low else BATTERY_HEALTHY

    def _maybe_start_cycle(self, now):
        if not (
            self.state == NORMAL_DRIVE
            and self.trigger_armed
            and self.stop_detected
        ):
            return

        battery_status = self.battery_status(now)
        if battery_status == BATTERY_HEALTHY:
            return

        if battery_status in (BATTERY_UNKNOWN, BATTERY_LOW):
            self.trigger_armed = False
            self.clear_started_at = None
            next_state = (
                WAIT_FOR_BATTERY
                if battery_status == BATTERY_UNKNOWN
                else PRE_PIT_STOP
            )
            self._transition(next_state, now)

    def _update_rearm(self, now):
        if self.trigger_armed:
            return
        if self.stop_detected:
            self.clear_started_at = None
            return
        if self.clear_started_at is None:
            self.clear_started_at = now
        elif now - self.clear_started_at >= self.sign_clear_time:
            self.trigger_armed = True
            self.clear_started_at = None

    def _transition(self, new_state, now):
        self.state = new_state
        self.state_entered_at = float(now)
        self.acquisition_count = 0
        self.last_lane_acquired = False
        self.last_lane_status_time = None
        self.last_lane_acquired_time = None
        self.last_advance_time = (
            float(now) if new_state == PIT_DRIVE else None
        )
        if new_state == ENTER_PIT:
            self.pit_drive_elapsed = 0.0
