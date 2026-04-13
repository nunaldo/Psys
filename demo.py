from __future__ import annotations

import argparse
import re
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Sequence, Tuple

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    plt = None
    MATPLOTLIB_IMPORT_ERROR = exc
else:
    MATPLOTLIB_IMPORT_ERROR = None

try:
    import serial
except ImportError as exc:
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None
    try:
        from serial.tools import list_ports
    except ImportError:
        list_ports = None


DEFAULT_PORT = "COM11"
BAUD_RATE = 115200
WINDOW_SIZE = 200
DEFAULT_REFS = [30.0, 30.0, 30.0]
DEFAULT_COSTS = [11.0, 0.5, 13.0]
DEFAULT_CAL_SECONDS = 9.0
DEFAULT_PID_SECONDS = 12.0
DEFAULT_ADMM_SECONDS = 18.0
DEFAULT_REPLY_WINDOW = 0.25
DEFAULT_COMMAND_DELAY = 0.10
DEFAULT_SAMPLE_PERIOD = 0.40

HUB_LINE_RE = re.compile(
    r"^([A-Za-z])\s+(\d+)\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)$"
)

NODE_COLORS = {
    1: "tab:blue",
    2: "tab:orange",
    3: "tab:green",
}


def parse_csv_floats(raw: str, expected_len: int, label: str) -> List[float]:
    items = [item.strip() for item in raw.split(",") if item.strip()]
    if len(items) != expected_len:
        raise ValueError(f"{label} must have exactly {expected_len} values")
    return [float(item) for item in items]


def parse_hub_line(line: str) -> Optional[Tuple[str, int, float]]:
    match = HUB_LINE_RE.match(line.strip())
    if match is None:
        return None
    return match.group(1), int(match.group(2)), float(match.group(3))


def read_lines(ser: serial.Serial, duration_s: float) -> List[str]:
    deadline = time.time() + duration_s
    lines: List[str] = []
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        else:
            time.sleep(0.01)
    return lines


def send_command(
    ser: serial.Serial,
    cmd: str,
    reply_window_s: float,
    *,
    echo: bool = True,
) -> List[str]:
    if echo:
        print(f">> {cmd}")
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()
    replies = read_lines(ser, reply_window_s)
    if echo:
        if replies:
            for line in replies:
                print(f"<< {line}")
        else:
            print("<< (no reply)")
    return replies


def discover_nodes(ser: serial.Serial, reply_window_s: float) -> List[int]:
    ser.reset_input_buffer()
    replies = send_command(ser, "g i 0", reply_window_s=max(reply_window_s, 1.0))
    found: List[int] = []
    for line in replies:
        parsed = parse_hub_line(line)
        if parsed is None:
            continue
        key, _, value = parsed
        if key != "i":
            continue
        node = int(round(value))
        if node not in found:
            found.append(node)
    return sorted(found)


def run_calibration(
    ser: serial.Serial,
    calibration_seconds: float,
    reply_window_s: float,
) -> None:
    print("\n== Calibration ==")
    send_command(ser, "cal", reply_window_s=max(reply_window_s, 0.5))
    print(f"Waiting {calibration_seconds:.1f}s for calibration to finish...")
    time.sleep(calibration_seconds)
    send_command(ser, "p 0 0", reply_window_s=max(reply_window_s, 1.0))


def run_setup_commands(
    ser: serial.Serial,
    commands: Sequence[str],
    reply_window_s: float,
    delay_s: float,
) -> None:
    for cmd in commands:
        send_command(ser, cmd, reply_window_s)
        time.sleep(delay_s)


def build_pid_commands(nodes: Sequence[int], refs: Sequence[float]) -> List[str]:
    commands: List[str] = []
    for idx, node in enumerate(nodes):
        commands.append(f"f {node} 0")
        commands.append(f"r {node} {refs[idx]}")
    for node in nodes:
        commands.append(f"m {node} 1")
    return commands


def build_admm_commands(
    nodes: Sequence[int],
    refs: Sequence[float],
    costs: Sequence[float],
) -> List[str]:
    commands: List[str] = []
    for idx, node in enumerate(nodes):
        commands.append(f"f {node} 0")
        commands.append(f"r {node} {refs[idx]}")
        commands.append(f"C {node} {costs[idx]}")
        commands.append(f"A {node} 1")
    for node in nodes:
        commands.append(f"m {node} 2")
    return commands


def init_plot(nodes: Sequence[int], refs: Sequence[float]):
    plt.style.use("ggplot")
    plt.ion()

    fig, (ax_lux, ax_duty) = plt.subplots(
        2,
        1,
        figsize=(12, 7),
        sharex=True,
        constrained_layout=True,
    )

    lines: Dict[str, Dict[int, plt.Line2D]] = {"lux": {}, "duty": {}}
    data_x: Dict[str, Dict[int, Deque[float]]] = {"lux": {}, "duty": {}}
    data_y: Dict[str, Dict[int, Deque[float]]] = {"lux": {}, "duty": {}}

    for idx, node in enumerate(nodes):
        color = NODE_COLORS.get(node, f"C{idx}")
        data_x["lux"][node] = deque(maxlen=WINDOW_SIZE)
        data_y["lux"][node] = deque(maxlen=WINDOW_SIZE)
        data_x["duty"][node] = deque(maxlen=WINDOW_SIZE)
        data_y["duty"][node] = deque(maxlen=WINDOW_SIZE)
        (lines["lux"][node],) = ax_lux.plot([], [], color=color, label=f"Node {node}")
        (lines["duty"][node],) = ax_duty.plot([], [], color=color, label=f"Node {node}")
        ax_lux.axhline(refs[idx], color=color, linestyle="--", linewidth=1.0, alpha=0.35)

    ax_lux.set_ylabel("Lux")
    ax_duty.set_ylabel("Duty")
    ax_duty.set_xlabel("Time [s]")
    ax_duty.set_ylim(0.0, 1.05)
    ax_lux.legend(loc="upper right")
    ax_duty.legend(loc="upper right")

    return fig, ax_lux, ax_duty, lines, data_x, data_y


def update_plot(
    fig,
    ax_lux,
    ax_duty,
    lines: Dict[str, Dict[int, plt.Line2D]],
    data_x: Dict[str, Dict[int, Deque[float]]],
    data_y: Dict[str, Dict[int, Deque[float]]],
    phase_name: str,
) -> None:
    for metric, axis in (("lux", ax_lux), ("duty", ax_duty)):
        for node_id, line in lines[metric].items():
            line.set_data(list(data_x[metric][node_id]), list(data_y[metric][node_id]))
        axis.relim()
        axis.autoscale_view()

    ax_lux.set_title(f"novo4 demo - {phase_name}")
    fig.canvas.draw_idle()
    plt.pause(0.01)


def process_replies(
    replies: Sequence[str],
    t_s: float,
    data_x: Dict[str, Dict[int, Deque[float]]],
    data_y: Dict[str, Dict[int, Deque[float]]],
) -> bool:
    changed = False
    for line in replies:
        parsed = parse_hub_line(line)
        if parsed is None:
            continue
        key, node, value = parsed
        if key == "y":
            data_x["lux"].setdefault(node, deque(maxlen=WINDOW_SIZE)).append(t_s)
            data_y["lux"].setdefault(node, deque(maxlen=WINDOW_SIZE)).append(value)
            changed = True
        elif key == "u":
            data_x["duty"].setdefault(node, deque(maxlen=WINDOW_SIZE)).append(t_s)
            data_y["duty"].setdefault(node, deque(maxlen=WINDOW_SIZE)).append(value)
            changed = True
    return changed


def run_phase(
    ser: serial.Serial,
    nodes: Sequence[int],
    duration_s: float,
    sample_period_s: float,
    reply_window_s: float,
    phase_name: str,
    time_offset_s: float,
    fig,
    ax_lux,
    ax_duty,
    lines: Dict[str, Dict[int, plt.Line2D]],
    data_x: Dict[str, Dict[int, Deque[float]]],
    data_y: Dict[str, Dict[int, Deque[float]]],
) -> None:
    print(f"\n== {phase_name} for {duration_s:.1f}s ==")
    phase_start = time.time()

    while True:
        cycle_start = time.time()
        elapsed = cycle_start - phase_start
        if elapsed >= duration_s:
            break

        got_new_samples = False
        for node in nodes:
            for cmd in (f"g y {node}", f"g u {node}"):
                replies = send_command(ser, cmd, reply_window_s, echo=False)
                got_new_samples |= process_replies(
                    replies,
                    time_offset_s + elapsed,
                    data_x,
                    data_y,
                )

        if got_new_samples:
            update_plot(fig, ax_lux, ax_duty, lines, data_x, data_y, phase_name)

        remaining = sample_period_s - (time.time() - cycle_start)
        if remaining > 0.0:
            time.sleep(remaining)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate the 3-node setup, run PID first, then ADMM, and plot lux/duty live."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port, e.g. COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--refs", default="30,30,30", help="Comma-separated refs for nodes 1..3")
    parser.add_argument("--costs", default="11,0.5,13", help="Comma-separated ADMM costs for nodes 1..3")
    parser.add_argument("--cal-seconds", type=float, default=DEFAULT_CAL_SECONDS)
    parser.add_argument("--pid-seconds", type=float, default=DEFAULT_PID_SECONDS)
    parser.add_argument("--admm-seconds", type=float, default=DEFAULT_ADMM_SECONDS)
    parser.add_argument("--reply-window", type=float, default=DEFAULT_REPLY_WINDOW)
    parser.add_argument("--command-delay", type=float, default=DEFAULT_COMMAND_DELAY)
    parser.add_argument("--sample-period", type=float, default=DEFAULT_SAMPLE_PERIOD)
    return parser


def available_ports_text() -> str:
    if list_ports is None:
        return "Could not enumerate serial ports on this machine."

    ports = list(list_ports.comports())
    if not ports:
        return "No serial ports were detected."

    labels = [port.device for port in ports]
    return "Available serial ports: " + ", ".join(labels)


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    if serial is None:
        raise SystemExit(
            "Missing dependency: pyserial. Install with `pip install pyserial`."
        ) from SERIAL_IMPORT_ERROR
    if plt is None:
        raise SystemExit(
            "Missing dependency: matplotlib. Install with `pip install matplotlib`."
        ) from MATPLOTLIB_IMPORT_ERROR

    refs = parse_csv_floats(args.refs, 3, "refs")
    costs = parse_csv_floats(args.costs, 3, "costs")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        raise SystemExit(
            f"Could not open serial port `{args.port}`. {available_ports_text()} "
            f"Use `--port COM11` or the correct port for your board."
        ) from exc

    with ser:
        print(f"Connected to {args.port} at {args.baud} baud.")
        time.sleep(1.0)

        nodes = discover_nodes(ser, args.reply_window)
        if len(nodes) != 3:
            raise SystemExit(f"Expected 3 nodes, got {nodes!r}")

        print(f"Discovered nodes: {nodes}")
        fig, ax_lux, ax_duty, lines, data_x, data_y = init_plot(nodes, refs)
        plt.show()

        run_calibration(ser, args.cal_seconds, args.reply_window)

        print("\n== Setup PID ==")
        run_setup_commands(
            ser,
            build_pid_commands(nodes, refs),
            args.reply_window,
            args.command_delay,
        )
        run_phase(
            ser,
            nodes,
            args.pid_seconds,
            args.sample_period,
            args.reply_window,
            "PID",
            0.0,
            fig,
            ax_lux,
            ax_duty,
            lines,
            data_x,
            data_y,
        )

        print("\n== Setup ADMM ==")
        run_setup_commands(
            ser,
            build_admm_commands(nodes, refs, costs),
            args.reply_window,
            args.command_delay,
        )
        ax_lux.axvline(args.pid_seconds, color="black", linestyle=":", linewidth=1.0, alpha=0.6)
        ax_duty.axvline(args.pid_seconds, color="black", linestyle=":", linewidth=1.0, alpha=0.6)
        run_phase(
            ser,
            nodes,
            args.admm_seconds,
            args.sample_period,
            args.reply_window,
            "ADMM",
            args.pid_seconds,
            fig,
            ax_lux,
            ax_duty,
            lines,
            data_x,
            data_y,
        )

        print("\nSequence finished. Close the plot window to exit.")
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
