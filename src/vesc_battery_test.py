from pyvesc import VESC
import os
import time


def find_vesc_port():
    possible_ports = [
        '/dev/ttyACM0',
        '/dev/ttyACM1'
    ]

    for port in possible_ports:
        if os.path.exists(port):
            return port

    return None


def main():
    port = find_vesc_port()

    if port is None:
        print("ERROR: No VESC device found.")
        print("Checked /dev/ttyACM0 and /dev/ttyACM1.")
        return

    print(f"Found possible VESC device: {port}")

    vesc = None

    # Retry connection because the VESC handshake may fail occasionally.
    for attempt in range(10):
        try:
            print(f"Connecting to VESC... attempt {attempt + 1}/10")

            vesc = VESC(
                serial_port=port,
                has_sensor=False,
                start_heartbeat=False,
                baudrate=115200,
                timeout=1.0
            )

            print("Connected to VESC.")
            break

        except Exception as error:
            print(f"Connection failed: {error}")
            time.sleep(1.0)

    if vesc is None:
        print("ERROR: Could not connect to VESC after 10 attempts.")
        return

    print("Reading battery voltage...")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            measurements = vesc.get_measurements()

            if measurements is None:
                print("No measurement received.")
                time.sleep(1.0)
                continue

            voltage = measurements.v_in

            print(f"Battery voltage: {voltage:.2f} V")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nBattery test stopped.")

    except Exception as error:
        print(f"VESC read error: {error}")


if __name__ == '__main__':
    main()
