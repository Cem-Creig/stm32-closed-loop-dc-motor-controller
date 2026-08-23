import csv
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import serial


PORT = "COM6"
BAUD_RATE = 115200

OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "motor_results"
RUN_NAME = datetime.now().strftime("project3_step_response_%Y%m%d_%H%M%S")
CSV_FILE = OUTPUT_DIRECTORY / f"{RUN_NAME}.csv"
GRAPH_FILE = OUTPUT_DIRECTORY / f"{RUN_NAME}.png"

CSV_HEADER = [
    "timestamp_ms",
    "target_rpm",
    "actual_rpm",
    "error_rpm",
    "i_term_percent",
    "pwm_duty",
]


def update_graph(
    speed_axis,
    pwm_axis,
    target_line,
    actual_line,
    pwm_line,
    times_s,
    targets_rpm,
    actuals_rpm,
    pwm_duties,
):
    target_line.set_data(times_s, targets_rpm)
    actual_line.set_data(times_s, actuals_rpm)
    pwm_line.set_data(times_s, pwm_duties)

    speed_axis.relim()
    speed_axis.autoscale_view()
    pwm_axis.relim()
    pwm_axis.autoscale_view()
    plt.pause(0.01)


def main():
    OUTPUT_DIRECTORY.mkdir(exist_ok=True)

    times_s = []
    targets_rpm = []
    actuals_rpm = []
    pwm_duties = []

    first_timestamp_ms = None
    motor_started = False

    plt.ion()
    figure, (speed_axis, pwm_axis) = plt.subplots(
        2,
        1,
        sharex=True,
        figsize=(10, 7),
    )

    target_line, = speed_axis.plot(
        [], [], label="Target RPM", color="tab:blue", linewidth=2
    )
    actual_line, = speed_axis.plot(
        [], [], label="Actual RPM", color="tab:red", linewidth=1.5
    )
    pwm_line, = pwm_axis.plot(
        [], [], label="PWM duty", color="tab:green", linewidth=1.5
    )

    speed_axis.set_title("STM32 Closed-Loop Motor Speed Controller")
    speed_axis.set_ylabel("Speed (RPM)")
    speed_axis.grid(True)
    speed_axis.legend()

    pwm_axis.set_xlabel("Time since motor start (s)")
    pwm_axis.set_ylabel("PWM duty (%)")
    pwm_axis.set_ylim(0, 100)
    pwm_axis.grid(True)
    pwm_axis.legend()

    figure.tight_layout()
    plt.show(block=False)

    print(f"Opening {PORT} at {BAUD_RATE} baud...")
    print("Close Tera Term before running this program.")

    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=1) as serial_port:
            print("Connected. Press RESET on the Nucleo to start a fresh run.")

            with CSV_FILE.open("w", newline="") as log_file:
                writer = csv.writer(log_file)
                writer.writerow(CSV_HEADER)

                while True:
                    line = (
                        serial_port.readline()
                        .decode("utf-8", errors="replace")
                        .strip()
                    )

                    if not line or line.startswith("timestamp_ms"):
                        continue

                    parts = line.split(",")
                    if len(parts) != 6:
                        print("Skipped malformed row:", line)
                        continue

                    try:
                        timestamp_ms = int(parts[0])
                        target_rpm = float(parts[1])
                        actual_rpm = float(parts[2])
                        error_rpm = float(parts[3])
                        i_term_percent = float(parts[4])
                        pwm_duty = int(parts[5])
                    except ValueError:
                        print("Skipped malformed row:", line)
                        continue

                    # Ignore old zero-speed rows until a fresh motor run begins.
                    if not motor_started:
                        if pwm_duty == 0:
                            continue

                        motor_started = True
                        first_timestamp_ms = timestamp_ms
                        print("Motor run detected; recording data...")

                    writer.writerow(
                        [
                            timestamp_ms,
                            target_rpm,
                            actual_rpm,
                            error_rpm,
                            i_term_percent,
                            pwm_duty,
                        ]
                    )
                    log_file.flush()

                    time_s = (timestamp_ms - first_timestamp_ms) / 1000.0
                    times_s.append(time_s)
                    targets_rpm.append(target_rpm)
                    actuals_rpm.append(actual_rpm)
                    pwm_duties.append(pwm_duty)

                    update_graph(
                        speed_axis,
                        pwm_axis,
                        target_line,
                        actual_line,
                        pwm_line,
                        times_s,
                        targets_rpm,
                        actuals_rpm,
                        pwm_duties,
                    )

                    # The firmware reports one coast-down row, then 0 RPM at 0% PWM.
                    if pwm_duty == 0 and abs(actual_rpm) <= 0.1:
                        print("Safe motor stop detected.")
                        break

    except serial.SerialException as error:
        print("Could not use the serial port:", error)
        print("Check that COM6 is correct and that Tera Term is closed.")
        return
    except KeyboardInterrupt:
        print("\nCapture stopped by user.")

    if not times_s:
        print("No motor data was captured, so no graph was saved.")
        return

    figure.savefig(GRAPH_FILE, dpi=200, bbox_inches="tight")
    print(f"CSV saved to: {CSV_FILE}")
    print(f"Graph saved to: {GRAPH_FILE}")

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
