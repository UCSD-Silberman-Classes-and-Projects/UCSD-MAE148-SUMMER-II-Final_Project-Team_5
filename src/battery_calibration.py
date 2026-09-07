"""Editable voltage-to-percentage calibration for the Team 5 4S LiPo."""


# Pack-voltage points must stay ordered by increasing voltage and percentage.
# Only 15.51 V through 16.80 V was measured against the charger. The 13.20 V
# zero-percent endpoint is retained from the previous code's 3.30 V/cell floor;
# interpolation below 15.51 V is therefore conservative and not experimentally
# calibrated for this battery.
VOLTAGE_PERCENTAGE_POINTS = (
    (13.20, 0.0),
    (15.51, 48.0),
    (15.55, 53.0),
    (15.61, 54.0),
    (15.71, 60.0),
    (16.00, 71.0),
    (16.27, 80.0),
    (16.45, 85.0),
    (16.74, 95.0),
    (16.80, 100.0),
)

LOW_ENTER_VOLTAGE = 15.50
LOW_RECOVERY_VOLTAGE = 15.60


def voltage_to_percentage(pack_voltage):
    """Convert 4S pack voltage to a clamped, piecewise-linear percentage."""
    voltage = float(pack_voltage)

    if voltage <= VOLTAGE_PERCENTAGE_POINTS[0][0]:
        return 0.0
    if voltage >= VOLTAGE_PERCENTAGE_POINTS[-1][0]:
        return 100.0

    for low_point, high_point in zip(
        VOLTAGE_PERCENTAGE_POINTS,
        VOLTAGE_PERCENTAGE_POINTS[1:],
    ):
        low_voltage, low_percentage = low_point
        high_voltage, high_percentage = high_point
        if low_voltage <= voltage <= high_voltage:
            fraction = (
                (voltage - low_voltage)
                / (high_voltage - low_voltage)
            )
            percentage = low_percentage + fraction * (
                high_percentage - low_percentage
            )
            return min(max(percentage, 0.0), 100.0)

    return 0.0


def update_low_battery_state(
    pack_voltage,
    was_low,
    low_enter_voltage=LOW_ENTER_VOLTAGE,
    low_recovery_voltage=LOW_RECOVERY_VOLTAGE,
):
    """Apply voltage hysteresis while preserving state inside the deadband."""
    voltage = float(pack_voltage)
    if voltage <= float(low_enter_voltage):
        return True
    if voltage >= float(low_recovery_voltage):
        return False
    return bool(was_low)
