#!/usr/bin/env python3
"""Run and analyze distributed-controller smoke tests over the hub serial port.

This script supports two workflows:
- live collection from the hub serial port
- offline analysis from an existing CSV via --plot-csv

Outputs:
- CSV with raw samples
- PNG plots when matplotlib is available
- otherwise a self-contained HTML report with inline SVG graphs
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from html import escape
import math
from pathlib import Path
import re
import statistics
import time
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import serial
except ImportError as exc:
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


HUB_LINE_RE = re.compile(
    r"^([A-Za-z])\s+(\d+)\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)$"
)

NODE_COLORS = {
    1: "tab:blue",
    2: "tab:orange",
    3: "tab:green",
}

ALGO_NAMES = {
    0: "primal_dual",
    1: "admm",
    2: "consensus",
}


@dataclass
class Sample:
    scenario: str
    t_s: float
    node: int
    metric: str
    value: float
    raw: str


@dataclass
class ScenarioConfig:
    name: str
    algo: int
    refs: List[float]
    costs: List[float]
    feedback: int
    start_mode: int = 2
    distributed: bool = True


def algorithm_name(algo: int) -> str:
    return ALGO_NAMES.get(algo, f"algo_{algo}")


def scenario_display_name(config: Optional[ScenarioConfig]) -> str:
    if config is None:
        return "unknown"
    if not config.distributed or config.start_mode == 1:
        return config.name
    return f"{config.name} [{algorithm_name(config.algo)}]"


def parse_csv_floats(raw: str, expected_len: int, label: str) -> List[float]:
    items = [item.strip() for item in raw.split(",") if item.strip()]
    if len(items) != expected_len:
        raise ValueError(f"{label} must have exactly {expected_len} comma-separated values")
    return [float(item) for item in items]


def parse_hub_line(line: str) -> Optional[Tuple[str, int, float]]:
    match = HUB_LINE_RE.match(line.strip())
    if not match:
        return None
    return match.group(1), int(match.group(2)), float(match.group(3))


def metric_from_key(key: str) -> Optional[str]:
    if key == "y":
        return "lux"
    if key == "u":
        return "duty"
    if key == "i":
        return "node_info"
    return None


def read_lines(ser: "serial.Serial", duration_s: float) -> List[str]:
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
    ser: "serial.Serial",
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


def discover_nodes(ser: "serial.Serial", reply_window_s: float) -> List[int]:
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


def build_setup_commands(nodes: Sequence[int], config: ScenarioConfig) -> List[str]:
    cmds: List[str] = []
    for idx, node in enumerate(nodes):
        if config.distributed:
            cmds.append(f"A {node} {config.algo}")
            cmds.append(f"C {node} {config.costs[idx]}")
        cmds.append(f"f {node} {config.feedback}")
        cmds.append(f"r {node} {config.refs[idx]}")
    for node in nodes:
        cmds.append(f"m {node} {config.start_mode}")
    return cmds


def run_setup(
    ser: "serial.Serial",
    nodes: Sequence[int],
    config: ScenarioConfig,
    delay_s: float,
    reply_window_s: float,
) -> None:
    print(f"\n== Setup: {scenario_display_name(config)} ==")
    for cmd in build_setup_commands(nodes, config):
        send_command(ser, cmd, reply_window_s)
        time.sleep(delay_s)


def run_calibration(
    ser: "serial.Serial",
    calibration_seconds: float,
    reply_window_s: float,
    delay_s: float,
) -> None:
    print("\n== Calibration ==")
    send_command(ser, "cal", reply_window_s=max(reply_window_s, 0.5))
    print(f"\n== Waiting {calibration_seconds:.1f}s for calibration FSM ==")
    time.sleep(calibration_seconds)
    send_command(ser, "p 0 0", reply_window_s=max(reply_window_s, 1.0))
    time.sleep(delay_s)


def collect_samples(
    ser: "serial.Serial",
    nodes: Sequence[int],
    config: ScenarioConfig,
    duration_s: float,
    sample_period_s: float,
    reply_window_s: float,
) -> List[Sample]:
    print(f"\n== Monitor: {scenario_display_name(config)} for {duration_s:.1f}s ==")
    start = time.time()
    records: List[Sample] = []

    while True:
        loop_t0 = time.time()
        elapsed = loop_t0 - start
        if elapsed >= duration_s:
            break

        for node in nodes:
            for cmd in (f"g y {node}", f"g u {node}"):
                replies = send_command(ser, cmd, reply_window_s)
                for line in replies:
                    parsed = parse_hub_line(line)
                    if parsed is None:
                        continue
                    key, src_node, value = parsed
                    metric = metric_from_key(key)
                    if metric not in ("lux", "duty"):
                        continue
                    records.append(
                        Sample(
                            scenario=config.name,
                            t_s=time.time() - start,
                            node=src_node,
                            metric=metric,
                            value=value,
                            raw=line,
                        )
                    )

        sleep_left = sample_period_s - (time.time() - loop_t0)
        if sleep_left > 0:
            time.sleep(sleep_left)

    return records


def samples_for(records: Sequence[Sample], node: int, metric: str) -> List[Sample]:
    return [record for record in records if record.node == node and record.metric == metric]


def tail_values(records: Sequence[Sample], fraction: float = 0.4) -> List[float]:
    if not records:
        return []
    start = max(0, int(len(records) * (1.0 - fraction)))
    return [record.value for record in records[start:]]


def pct(values: Sequence[bool]) -> float:
    if not values:
        return 0.0
    return 100.0 * (sum(1 for value in values if value) / len(values))


def summarize_node(
    records: Sequence[Sample],
    node: int,
    ref: float,
) -> Dict[str, float]:
    lux_records = samples_for(records, node, "lux")
    duty_records = samples_for(records, node, "duty")
    lux_tail = tail_values(lux_records)
    duty_tail = tail_values(duty_records)

    lux_values = [record.value for record in lux_records]
    duty_values = [record.value for record in duty_records]
    has_ref = not math.isnan(ref)

    summary: Dict[str, float] = {
        "ref": ref,
        "lux_avg": statistics.fmean(lux_tail) if lux_tail else float("nan"),
        "lux_min": min(lux_values) if lux_values else float("nan"),
        "lux_max": max(lux_values) if lux_values else float("nan"),
        "duty_avg": statistics.fmean(duty_tail) if duty_tail else float("nan"),
        "duty_min": min(duty_values) if duty_values else float("nan"),
        "duty_max": max(duty_values) if duty_values else float("nan"),
        "below_ref_pct": pct([value < ref for value in lux_values]) if has_ref else float("nan"),
        "duty_sat_hi_pct": pct([value >= 0.98 for value in duty_values]),
        "duty_sat_lo_pct": pct([value <= 0.02 for value in duty_values]),
    }

    if lux_tail and has_ref:
        deficits = [max(0.0, ref - value) for value in lux_tail]
        summary["tail_avg_deficit"] = statistics.fmean(deficits)
        summary["tail_max_deficit"] = max(deficits)
        summary["tail_lux_span"] = max(lux_tail) - min(lux_tail)
    else:
        summary["tail_avg_deficit"] = float("nan")
        summary["tail_max_deficit"] = float("nan")
        summary["tail_lux_span"] = float("nan")

    if duty_tail:
        summary["tail_duty_span"] = max(duty_tail) - min(duty_tail)
    else:
        summary["tail_duty_span"] = float("nan")

    return summary


def unique_scenarios(records: Sequence[Sample]) -> List[str]:
    seen = set()
    ordered: List[str] = []
    for record in records:
        if record.scenario in seen:
            continue
        seen.add(record.scenario)
        ordered.append(record.scenario)
    return ordered


def config_ref(config: Optional[ScenarioConfig], idx: int) -> float:
    if config is None or idx >= len(config.refs):
        return float("nan")
    return config.refs[idx]


def config_cost(config: Optional[ScenarioConfig], idx: int) -> float:
    if config is None or idx >= len(config.costs):
        return float("nan")
    return config.costs[idx]


def scenario_summary(
    records: Sequence[Sample],
    nodes: Sequence[int],
    config: Optional[ScenarioConfig],
) -> Dict[int, Dict[str, float]]:
    per_node: Dict[int, Dict[str, float]] = {}
    for idx, node in enumerate(nodes):
        per_node[node] = summarize_node(records, node, config_ref(config, idx))
    return per_node


def describe_node(node: int, cost: float, summary: Dict[str, float]) -> List[str]:
    lines: List[str] = []
    lines.append(
        f"Node {node}: ref={summary['ref']:.2f}, avg lux={summary['lux_avg']:.2f}, "
        f"avg duty={summary['duty_avg']:.3f}, below-ref={summary['below_ref_pct']:.1f}%, cost={cost:.2f}"
    )

    if summary["tail_avg_deficit"] <= 0.10:
        lines.append("  Visibility looks acceptable in steady state.")
    elif summary["duty_sat_hi_pct"] >= 40.0:
        lines.append("  Node spends a lot of time saturated high; target may be too ambitious or gains are off.")
    else:
        lines.append("  Node is missing the target by a visible margin in steady state.")

    if summary["tail_lux_span"] >= 0.50:
        lines.append("  Lux is oscillating noticeably; this may be controller instability or noisy sensing.")
    elif summary["tail_duty_span"] >= 0.20:
        lines.append("  Duty is moving around a lot even if lux looks calmer.")
    else:
        lines.append("  Steady-state variation looks small.")

    return lines


def analyze_scenario(
    records: Sequence[Sample],
    nodes: Sequence[int],
    config: ScenarioConfig,
) -> Dict[int, Dict[str, float]]:
    print(f"\n== Analysis: {scenario_display_name(config)} ==")
    per_node: Dict[int, Dict[str, float]] = {}
    for idx, node in enumerate(nodes):
        summary = summarize_node(records, node, config.refs[idx])
        per_node[node] = summary
        for line in describe_node(node, config.costs[idx], summary):
            print(line)
    total_duty = sum(per_node[node]["duty_avg"] for node in nodes)
    total_deficit = sum(per_node[node]["tail_avg_deficit"] for node in nodes)
    print(
        f"System view: total avg duty={total_duty:.3f}, "
        f"sum steady-state deficit={total_deficit:.3f}"
    )
    return per_node


def compare_scenarios(
    nodes: Sequence[int],
    base_config: ScenarioConfig,
    base_summary: Dict[int, Dict[str, float]],
    weighted_config: ScenarioConfig,
    weighted_summary: Dict[int, Dict[str, float]],
) -> None:
    print("\n== Cost Sensitivity ==")
    for idx, node in enumerate(nodes):
        base_duty = base_summary[node]["duty_avg"]
        weighted_duty = weighted_summary[node]["duty_avg"]
        delta = weighted_duty - base_duty
        cost_base = base_config.costs[idx]
        cost_weighted = weighted_config.costs[idx]
        print(
            f"Node {node}: duty {base_duty:.3f} -> {weighted_duty:.3f} "
            f"(delta {delta:+.3f}) | cost {cost_base:.2f} -> {cost_weighted:.2f}"
        )

    print("\nInterpretation:")
    print("- If a node cost went up and its average duty went down, that is a good sign.")
    print("- If all duties stay pinned near 1.0, the test is saturated and cost differences are not informative.")
    print("- With your gains, it is normal for node 2 to carry more load than node 3 because node 2 is much stronger.")


def compare_to_baseline(
    nodes: Sequence[int],
    baseline_config: ScenarioConfig,
    baseline_summary: Dict[int, Dict[str, float]],
    distributed_config: ScenarioConfig,
    distributed_summary: Dict[int, Dict[str, float]],
) -> None:
    print(f"\n== Baseline vs {distributed_config.name} ==")
    base_total_duty = 0.0
    dist_total_duty = 0.0
    base_total_deficit = 0.0
    dist_total_deficit = 0.0

    for node in nodes:
        base = baseline_summary[node]
        dist = distributed_summary[node]
        base_total_duty += base["duty_avg"]
        dist_total_duty += dist["duty_avg"]
        base_total_deficit += base["tail_avg_deficit"]
        dist_total_deficit += dist["tail_avg_deficit"]
        print(
            f"Node {node}: lux {base['lux_avg']:.2f} -> {dist['lux_avg']:.2f}, "
            f"duty {base['duty_avg']:.3f} -> {dist['duty_avg']:.3f}, "
            f"deficit {base['tail_avg_deficit']:.3f} -> {dist['tail_avg_deficit']:.3f}"
        )

    print(
        f"System: total avg duty {base_total_duty:.3f} -> {dist_total_duty:.3f}, "
        f"sum steady-state deficit {base_total_deficit:.3f} -> {dist_total_deficit:.3f}"
    )
    print("\nInterpretation:")
    print("- Lower total duty with similar or lower deficit means the distributed controller is helping.")
    print("- Lower deficit with similar duty means the distributed controller is improving comfort.")
    print("- Higher duty and higher deficit means the distributed tuning/model is worse than the local baseline.")


def try_load_matplotlib():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    return plt


def fmt_float(value: float, digits: int = 2) -> str:
    if math.isnan(value):
        return "n/a"
    return f"{value:.{digits}f}"


def metric_axis_bounds(
    values: Sequence[float],
    *,
    force_min: Optional[float] = None,
    force_max: Optional[float] = None,
) -> Tuple[float, float]:
    clean = [value for value in values if not math.isnan(value)]
    if clean:
        y_min = min(clean)
        y_max = max(clean)
    else:
        y_min, y_max = 0.0, 1.0

    if force_min is not None:
        y_min = force_min
    if force_max is not None:
        y_max = force_max

    if math.isclose(y_min, y_max):
        pad = 0.2 if y_max == 0.0 else abs(y_max) * 0.1
        y_min -= pad
        y_max += pad
    elif force_min is None or force_max is None:
        pad = 0.08 * (y_max - y_min)
        if force_min is None:
            y_min -= pad
        if force_max is None:
            y_max += pad

    return y_min, y_max


def render_line_chart_svg(
    records: Sequence[Sample],
    nodes: Sequence[int],
    config: Optional[ScenarioConfig],
    metric: str,
    title: str,
    *,
    width: int = 640,
    height: int = 300,
) -> str:
    margin_left = 56
    margin_right = 18
    margin_top = 28
    margin_bottom = 40
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    metric_records = [record for record in records if record.metric == metric]
    times = [record.t_s for record in metric_records]
    values = [record.value for record in metric_records]
    if metric == "lux":
        refs = [config_ref(config, idx) for idx, _ in enumerate(nodes)]
        values.extend([ref for ref in refs if not math.isnan(ref)])
        y_min, y_max = metric_axis_bounds(values)
    else:
        y_min, y_max = metric_axis_bounds(values, force_min=0.0, force_max=1.0)

    x_min = 0.0
    x_max = max(times) if times else 1.0
    if x_max <= x_min:
        x_max = x_min + 1.0

    def x_to_px(value: float) -> float:
        return margin_left + ((value - x_min) / (x_max - x_min)) * plot_width

    def y_to_px(value: float) -> float:
        return margin_top + plot_height - ((value - y_min) / (y_max - y_min)) * plot_height

    parts = [
        f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff" stroke="#d7dce5"/>',
        f'<text x="{margin_left}" y="20" font-size="15" font-family="sans-serif" fill="#1f2937">{escape(title)}</text>',
    ]

    for step in range(6):
        frac = step / 5.0
        y_value = y_min + frac * (y_max - y_min)
        y_px = y_to_px(y_value)
        parts.append(
            f'<line x1="{margin_left}" y1="{y_px:.1f}" x2="{width - margin_right}" y2="{y_px:.1f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{margin_left - 8}" y="{y_px + 4:.1f}" text-anchor="end" '
            f'font-size="11" font-family="monospace" fill="#4b5563">{fmt_float(y_value)}</text>'
        )

    for step in range(6):
        frac = step / 5.0
        x_value = x_min + frac * (x_max - x_min)
        x_px = x_to_px(x_value)
        parts.append(
            f'<line x1="{x_px:.1f}" y1="{margin_top}" x2="{x_px:.1f}" y2="{margin_top + plot_height}" '
            'stroke="#f1f5f9" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{x_px:.1f}" y="{height - 10}" text-anchor="middle" '
            f'font-size="11" font-family="monospace" fill="#4b5563">{fmt_float(x_value, 1)}</text>'
        )

    parts.append(
        f'<line x1="{margin_left}" y1="{margin_top + plot_height}" x2="{width - margin_right}" '
        f'y2="{margin_top + plot_height}" stroke="#111827" stroke-width="1.2"/>'
    )
    parts.append(
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_height}" '
        'stroke="#111827" stroke-width="1.2"/>'
    )

    if metric == "lux":
        for idx, node in enumerate(nodes):
            ref = config_ref(config, idx)
            if math.isnan(ref):
                continue
            ref_y = y_to_px(ref)
            color = NODE_COLORS.get(node, f"C{idx}")
            parts.append(
                f'<line x1="{margin_left}" y1="{ref_y:.1f}" x2="{width - margin_right}" y2="{ref_y:.1f}" '
                f'stroke="{color}" stroke-width="1.2" stroke-dasharray="6 4" opacity="0.45"/>'
            )

    for idx, node in enumerate(nodes):
        node_records = [record for record in metric_records if record.node == node]
        if not node_records:
            continue
        color = NODE_COLORS.get(node, f"C{idx}")
        points = " ".join(
            f"{x_to_px(record.t_s):.1f},{y_to_px(record.value):.1f}" for record in node_records
        )
        parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{points}"/>')
        for record in node_records:
            parts.append(
                f'<circle cx="{x_to_px(record.t_s):.1f}" cy="{y_to_px(record.value):.1f}" r="2.4" fill="{color}"/>'
            )

    legend_x = width - margin_right - 100
    legend_y = margin_top + 8
    for idx, node in enumerate(nodes):
        color = NODE_COLORS.get(node, f"C{idx}")
        y = legend_y + idx * 18
        parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 16}" y2="{y}" stroke="{color}" stroke-width="3"/>')
        parts.append(
            f'<text x="{legend_x + 22}" y="{y + 4}" font-size="11" font-family="sans-serif" fill="#1f2937">Node {node}</text>'
        )

    y_label = "Lux" if metric == "lux" else "Duty"
    parts.append(
        f'<text x="{margin_left + plot_width / 2:.1f}" y="{height - 4}" text-anchor="middle" '
        'font-size="12" font-family="sans-serif" fill="#374151">Time [s]</text>'
    )
    parts.append(
        f'<text x="16" y="{margin_top + plot_height / 2:.1f}" text-anchor="middle" '
        f'transform="rotate(-90 16 {margin_top + plot_height / 2:.1f})" '
        f'font-size="12" font-family="sans-serif" fill="#374151">{escape(y_label)}</text>'
    )

    parts.append("</svg>")
    return "".join(parts)


def render_grouped_bar_chart_svg(
    categories: Sequence[str],
    series: Sequence[Tuple[str, Sequence[float], str]],
    title: str,
    y_label: str,
    *,
    width: int = 640,
    height: int = 300,
    force_min: Optional[float] = None,
    force_max: Optional[float] = None,
    zero_line: bool = False,
) -> str:
    margin_left = 56
    margin_right = 18
    margin_top = 28
    margin_bottom = 44
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    all_values = [value for _, values, _ in series for value in values if not math.isnan(value)]
    y_min, y_max = metric_axis_bounds(all_values, force_min=force_min, force_max=force_max)
    if zero_line:
        y_min = min(y_min, 0.0)
        y_max = max(y_max, 0.0)

    def y_to_px(value: float) -> float:
        return margin_top + plot_height - ((value - y_min) / (y_max - y_min)) * plot_height

    parts = [
        f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff" stroke="#d7dce5"/>',
        f'<text x="{margin_left}" y="20" font-size="15" font-family="sans-serif" fill="#1f2937">{escape(title)}</text>',
    ]

    for step in range(6):
        frac = step / 5.0
        y_value = y_min + frac * (y_max - y_min)
        y_px = y_to_px(y_value)
        parts.append(
            f'<line x1="{margin_left}" y1="{y_px:.1f}" x2="{width - margin_right}" y2="{y_px:.1f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{margin_left - 8}" y="{y_px + 4:.1f}" text-anchor="end" '
            f'font-size="11" font-family="monospace" fill="#4b5563">{fmt_float(y_value)}</text>'
        )

    parts.append(
        f'<line x1="{margin_left}" y1="{margin_top + plot_height}" x2="{width - margin_right}" '
        f'y2="{margin_top + plot_height}" stroke="#111827" stroke-width="1.2"/>'
    )
    parts.append(
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_height}" '
        'stroke="#111827" stroke-width="1.2"/>'
    )

    if zero_line and y_min < 0.0 < y_max:
        zero_y = y_to_px(0.0)
        parts.append(
            f'<line x1="{margin_left}" y1="{zero_y:.1f}" x2="{width - margin_right}" y2="{zero_y:.1f}" '
            'stroke="#111827" stroke-width="1" stroke-dasharray="5 3" opacity="0.7"/>'
        )

    group_width = plot_width / max(1, len(categories))
    inner_width = group_width * 0.72
    bar_width = inner_width / max(1, len(series))

    for category_idx, category in enumerate(categories):
        group_left = margin_left + category_idx * group_width + (group_width - inner_width) / 2.0
        parts.append(
            f'<text x="{group_left + inner_width / 2:.1f}" y="{height - 10}" text-anchor="middle" '
            f'font-size="11" font-family="sans-serif" fill="#374151">{escape(category)}</text>'
        )
        for series_idx, (_, values, color) in enumerate(series):
            value = values[category_idx]
            if math.isnan(value):
                continue
            x = group_left + series_idx * bar_width
            y = y_to_px(max(value, 0.0))
            base_y = y_to_px(0.0) if zero_line or y_min < 0.0 else y_to_px(y_min)
            rect_y = min(y, base_y)
            rect_h = max(1.0, abs(base_y - y))
            parts.append(
                f'<rect x="{x:.1f}" y="{rect_y:.1f}" width="{max(1.0, bar_width - 4):.1f}" height="{rect_h:.1f}" '
                f'fill="{color}" opacity="0.88"/>'
            )
            value_y = rect_y - 4 if value >= 0.0 else rect_y + rect_h + 12
            parts.append(
                f'<text x="{x + (bar_width - 4) / 2:.1f}" y="{value_y:.1f}" text-anchor="middle" '
                f'font-size="10" font-family="monospace" fill="#374151">{fmt_float(value)}</text>'
            )

    legend_x = width - margin_right - 120
    legend_y = margin_top + 8
    for idx, (name, _, color) in enumerate(series):
        y = legend_y + idx * 18
        parts.append(f'<rect x="{legend_x}" y="{y - 8}" width="12" height="12" fill="{color}" opacity="0.88"/>')
        parts.append(
            f'<text x="{legend_x + 18}" y="{y + 2}" font-size="11" font-family="sans-serif" fill="#1f2937">{escape(name)}</text>'
        )

    parts.append(
        f'<text x="16" y="{margin_top + plot_height / 2:.1f}" text-anchor="middle" '
        f'transform="rotate(-90 16 {margin_top + plot_height / 2:.1f})" '
        f'font-size="12" font-family="sans-serif" fill="#374151">{escape(y_label)}</text>'
    )
    parts.append("</svg>")
    return "".join(parts)


def scenario_table_html(
    nodes: Sequence[int],
    config: Optional[ScenarioConfig],
    summary: Dict[int, Dict[str, float]],
) -> str:
    rows = [
        "<tr><th>Node</th><th>Ref</th><th>Cost</th><th>Steady lux</th><th>Steady duty</th>"
        "<th>Below ref</th><th>Tail deficit</th><th>Lux span</th></tr>"
    ]
    for idx, node in enumerate(nodes):
        node_summary = summary[node]
        rows.append(
            "<tr>"
            f"<td>{node}</td>"
            f"<td>{fmt_float(config_ref(config, idx))}</td>"
            f"<td>{fmt_float(config_cost(config, idx))}</td>"
            f"<td>{fmt_float(node_summary['lux_avg'])}</td>"
            f"<td>{fmt_float(node_summary['duty_avg'], 3)}</td>"
            f"<td>{fmt_float(node_summary['below_ref_pct'])}%</td>"
            f"<td>{fmt_float(node_summary['tail_avg_deficit'])}</td>"
            f"<td>{fmt_float(node_summary['tail_lux_span'])}</td>"
            "</tr>"
        )
    return "<table>" + "".join(rows) + "</table>"


def write_html_report(
    records: Sequence[Sample],
    nodes: Sequence[int],
    scenario_configs: Dict[str, ScenarioConfig],
    path: Path,
) -> None:
    sections: List[str] = []
    scenario_names = unique_scenarios(records)

    for scenario_name in scenario_names:
        scenario_records = [record for record in records if record.scenario == scenario_name]
        config = scenario_configs.get(scenario_name)
        display_name = scenario_display_name(config)
        summary = scenario_summary(scenario_records, nodes, config)
        lux_svg = render_line_chart_svg(scenario_records, nodes, config, "lux", f"{display_name}: lux tracking")
        duty_svg = render_line_chart_svg(scenario_records, nodes, config, "duty", f"{display_name}: duty command")
        steady_lux_svg = render_grouped_bar_chart_svg(
            [f"Node {node}" for node in nodes],
            [
                ("steady lux", [summary[node]["lux_avg"] for node in nodes], "#2563eb"),
                ("reference", [config_ref(config, idx) for idx, _ in enumerate(nodes)], "#111827"),
            ],
            f"{display_name}: steady-state lux",
            "Lux",
            force_min=0.0,
        )
        steady_duty_svg = render_grouped_bar_chart_svg(
            [f"Node {node}" for node in nodes],
            [("steady duty", [summary[node]["duty_avg"] for node in nodes], "#059669")],
            f"{display_name}: steady-state duty",
            "Duty",
            force_min=0.0,
            force_max=1.0,
        )

        sections.append(
            "<section>"
            f"<h2>{escape(display_name)}</h2>"
            f"{scenario_table_html(nodes, config, summary)}"
            '<div class="grid">'
            f'<div class="card">{lux_svg}</div>'
            f'<div class="card">{duty_svg}</div>'
            f'<div class="card">{steady_lux_svg}</div>'
            f'<div class="card">{steady_duty_svg}</div>'
            "</div>"
            "</section>"
        )

    if len(scenario_names) >= 2:
        summaries = {
            name: scenario_summary(
                [record for record in records if record.scenario == name],
                nodes,
                scenario_configs.get(name),
            )
            for name in scenario_names
        }
        total_duty_svg = render_grouped_bar_chart_svg(
            scenario_names,
            [("total duty", [sum(summaries[name][node]["duty_avg"] for node in nodes) for name in scenario_names], "#2563eb")],
            "Scenario comparison: total steady-state duty",
            "Duty sum",
            force_min=0.0,
        )
        total_deficit_svg = render_grouped_bar_chart_svg(
            scenario_names,
            [("total deficit", [sum(summaries[name][node]["tail_avg_deficit"] for node in nodes) for name in scenario_names], "#dc2626")],
            "Scenario comparison: total steady-state deficit",
            "Lux deficit sum",
            force_min=0.0,
        )
        node_duty_series = [
            (
                scenario_display_name(scenario_configs.get(name)),
                [summaries[name][node]["duty_avg"] for node in nodes],
                f"hsl({(idx * 110) % 360} 65% 45%)",
            )
            for idx, name in enumerate(scenario_names)
        ]
        node_error_series = [
            (
                scenario_display_name(scenario_configs.get(name)),
                [summaries[name][node]["lux_avg"] - summaries[name][node]["ref"] for node in nodes],
                f"hsl({(idx * 110) % 360} 65% 45%)",
            )
            for idx, name in enumerate(scenario_names)
        ]
        node_duty_svg = render_grouped_bar_chart_svg(
            [f"Node {node}" for node in nodes],
            node_duty_series,
            "Scenario comparison: steady-state duty by node",
            "Duty",
            force_min=0.0,
            force_max=1.0,
        )
        node_error_svg = render_grouped_bar_chart_svg(
            [f"Node {node}" for node in nodes],
            node_error_series,
            "Scenario comparison: steady-state lux error by node",
            "Lux avg - ref",
            zero_line=True,
        )
        sections.append(
            "<section>"
            "<h2>Scenario comparison</h2>"
            '<div class="grid">'
            f'<div class="card">{total_duty_svg}</div>'
            f'<div class="card">{total_deficit_svg}</div>'
            f'<div class="card">{node_duty_svg}</div>'
            f'<div class="card">{node_error_svg}</div>'
            "</div>"
            "</section>"
        )

    html = (
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Controller test report</title>"
        "<style>"
        "body{font-family:Segoe UI,Arial,sans-serif;background:#f3f6fb;color:#111827;margin:24px;}"
        "h1,h2{margin:0 0 14px 0;}"
        "section{margin:0 0 28px 0;padding:20px;background:#ffffff;border:1px solid #d7dce5;border-radius:12px;}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(640px,1fr));gap:16px;align-items:start;}"
        ".card{background:#fbfdff;border:1px solid #e5e7eb;border-radius:10px;padding:10px;overflow:auto;}"
        "table{border-collapse:collapse;width:100%;margin:0 0 16px 0;font-size:14px;}"
        "th,td{border:1px solid #d7dce5;padding:8px 10px;text-align:center;}"
        "th{background:#eef4ff;}"
        "p{margin:0 0 10px 0;}"
        "</style></head><body>"
        "<h1>Controller test report</h1>"
        "<p>This report was generated directly from the test CSV. Dashed lux lines are the references for each node.</p>"
        + "".join(sections)
        + "</body></html>"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")


def save_scenario_plot(
    plt,
    records: Sequence[Sample],
    nodes: Sequence[int],
    config: Optional[ScenarioConfig],
    path: Path,
) -> None:
    summary = scenario_summary(records, nodes, config)
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    ax_lux, ax_duty, ax_lux_bar, ax_duty_bar = axes.ravel()
    title = scenario_display_name(config) if config is not None else records[0].scenario
    fig.suptitle(f"Controller test: {title}", fontsize=14)

    for idx, node in enumerate(nodes):
        color = NODE_COLORS.get(node, f"C{idx}")
        ref = config_ref(config, idx)
        lux_records = samples_for(records, node, "lux")
        duty_records = samples_for(records, node, "duty")

        if lux_records:
            ax_lux.plot(
                [record.t_s for record in lux_records],
                [record.value for record in lux_records],
                marker="o",
                markersize=3,
                linewidth=1.5,
                color=color,
                label=f"Node {node}",
            )
        if not math.isnan(ref):
            ax_lux.axhline(ref, color=color, linestyle="--", linewidth=1.0, alpha=0.45)

        if duty_records:
            ax_duty.plot(
                [record.t_s for record in duty_records],
                [record.value for record in duty_records],
                marker="o",
                markersize=3,
                linewidth=1.5,
                color=color,
                label=f"Node {node}",
            )

    ax_lux.set_title("Lux tracking")
    ax_lux.set_xlabel("Time [s]")
    ax_lux.set_ylabel("Lux")
    ax_lux.grid(True, alpha=0.25)
    ax_lux.legend()

    ax_duty.set_title("Duty command")
    ax_duty.set_xlabel("Time [s]")
    ax_duty.set_ylabel("Duty")
    ax_duty.set_ylim(0.0, 1.05)
    ax_duty.grid(True, alpha=0.25)
    ax_duty.legend()

    x_positions = list(range(len(nodes)))
    lux_avg = [summary[node]["lux_avg"] for node in nodes]
    refs = [config_ref(config, idx) for idx, _ in enumerate(nodes)]
    duty_avg = [summary[node]["duty_avg"] for node in nodes]
    costs = [config_cost(config, idx) for idx, _ in enumerate(nodes)]
    bar_colors = [NODE_COLORS.get(node, f"C{idx}") for idx, node in enumerate(nodes)]

    ax_lux_bar.bar(x_positions, lux_avg, color=bar_colors, alpha=0.85, label="steady lux")
    if any(not math.isnan(value) for value in refs):
        ax_lux_bar.plot(x_positions, refs, color="black", marker="x", linestyle="None", label="reference")
    ax_lux_bar.set_title("Steady-state lux")
    ax_lux_bar.set_xlabel("Node")
    ax_lux_bar.set_ylabel("Lux")
    ax_lux_bar.set_xticks(x_positions, [str(node) for node in nodes])
    ax_lux_bar.grid(True, axis="y", alpha=0.25)
    ax_lux_bar.legend()
    for idx, node in enumerate(nodes):
        deficit = summary[node]["tail_avg_deficit"]
        if not math.isnan(deficit):
            ax_lux_bar.text(
                x_positions[idx],
                lux_avg[idx] if not math.isnan(lux_avg[idx]) else 0.0,
                f"def {deficit:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )

    ax_duty_bar.bar(x_positions, duty_avg, color=bar_colors, alpha=0.85)
    ax_duty_bar.set_title("Steady-state duty")
    ax_duty_bar.set_xlabel("Node")
    ax_duty_bar.set_ylabel("Duty")
    ax_duty_bar.set_xticks(x_positions, [str(node) for node in nodes])
    ax_duty_bar.set_ylim(0.0, 1.05)
    ax_duty_bar.grid(True, axis="y", alpha=0.25)
    for idx, node in enumerate(nodes):
        sat_hi = summary[node]["duty_sat_hi_pct"]
        cost = costs[idx]
        label = f"cost {cost:.2f}" if not math.isnan(cost) else "cost n/a"
        if not math.isnan(sat_hi):
            label += f"\nsat {sat_hi:.0f}%"
        ax_duty_bar.text(
            x_positions[idx],
            duty_avg[idx] if not math.isnan(duty_avg[idx]) else 0.0,
            label,
            ha="center",
            va="bottom",
            fontsize=8,
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_comparison_plot(
    plt,
    records: Sequence[Sample],
    nodes: Sequence[int],
    scenario_configs: Dict[str, ScenarioConfig],
    path: Path,
) -> None:
    scenario_names = unique_scenarios(records)
    summaries = {
        name: scenario_summary(
            [record for record in records if record.scenario == name],
            nodes,
            scenario_configs.get(name),
        )
        for name in scenario_names
    }

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    ax_total_duty, ax_total_deficit, ax_node_duty, ax_node_error = axes.ravel()
    fig.suptitle("Controller test: scenario comparison", fontsize=14)

    x_positions = list(range(len(scenario_names)))
    total_duty = [sum(summaries[name][node]["duty_avg"] for node in nodes) for name in scenario_names]
    total_deficit = [sum(summaries[name][node]["tail_avg_deficit"] for node in nodes) for name in scenario_names]

    ax_total_duty.bar(x_positions, total_duty, color="steelblue", alpha=0.85)
    ax_total_duty.set_title("Total steady-state duty")
    ax_total_duty.set_ylabel("Duty sum")
    ax_total_duty.set_xticks(x_positions, scenario_names, rotation=15)
    ax_total_duty.grid(True, axis="y", alpha=0.25)

    ax_total_deficit.bar(x_positions, total_deficit, color="indianred", alpha=0.85)
    ax_total_deficit.set_title("Total steady-state deficit")
    ax_total_deficit.set_ylabel("Lux deficit sum")
    ax_total_deficit.set_xticks(x_positions, scenario_names, rotation=15)
    ax_total_deficit.grid(True, axis="y", alpha=0.25)

    width = 0.8 / max(1, len(scenario_names))
    node_positions = list(range(len(nodes)))
    for scenario_idx, name in enumerate(scenario_names):
        offset = (scenario_idx - (len(scenario_names) - 1) / 2.0) * width
        duty_vals = [summaries[name][node]["duty_avg"] for node in nodes]
        error_vals = [summaries[name][node]["lux_avg"] - summaries[name][node]["ref"] for node in nodes]
        ax_node_duty.bar([x + offset for x in node_positions], duty_vals, width=width, label=name, alpha=0.85)
        ax_node_error.bar([x + offset for x in node_positions], error_vals, width=width, label=name, alpha=0.85)

    ax_node_duty.set_title("Steady-state duty by node")
    ax_node_duty.set_ylabel("Duty")
    ax_node_duty.set_xticks(node_positions, [str(node) for node in nodes])
    ax_node_duty.grid(True, axis="y", alpha=0.25)
    ax_node_duty.legend()

    ax_node_error.axhline(0.0, color="black", linewidth=1.0)
    ax_node_error.set_title("Steady-state lux error by node")
    ax_node_error.set_ylabel("Lux avg - ref")
    ax_node_error.set_xticks(node_positions, [str(node) for node in nodes])
    ax_node_error.grid(True, axis="y", alpha=0.25)
    ax_node_error.legend()

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_plots(
    records: Sequence[Sample],
    nodes: Sequence[int],
    scenario_configs: Dict[str, ScenarioConfig],
    out_dir: Path,
    prefix: str,
) -> List[Path]:
    if not records:
        return []

    plt = try_load_matplotlib()
    if plt is None:
        report_path = out_dir / f"{prefix}_report.html"
        write_html_report(records, nodes, scenario_configs, report_path)
        return [report_path]

    plot_paths: List[Path] = []
    for scenario_name in unique_scenarios(records):
        scenario_records = [record for record in records if record.scenario == scenario_name]
        if not scenario_records:
            continue
        path = out_dir / f"{prefix}_{scenario_name}.png"
        save_scenario_plot(plt, scenario_records, nodes, scenario_configs.get(scenario_name), path)
        plot_paths.append(path)

    if len(unique_scenarios(records)) >= 2:
        comparison_path = out_dir / f"{prefix}_comparison.png"
        save_comparison_plot(plt, records, nodes, scenario_configs, comparison_path)
        plot_paths.append(comparison_path)

    return plot_paths


def load_csv_records(path: Path) -> List[Sample]:
    records: List[Sample] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            records.append(
                Sample(
                    scenario=row["scenario"],
                    t_s=float(row["t_s"]),
                    node=int(row["node"]),
                    metric=row["metric"],
                    value=float(row["value"]),
                    raw=row["raw"],
                )
            )
    return records


def write_csv(records: Sequence[Sample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["scenario", "t_s", "node", "metric", "value", "raw"])
        for record in records:
            writer.writerow(
                [
                    record.scenario,
                    f"{record.t_s:.6f}",
                    record.node,
                    record.metric,
                    f"{record.value:.6f}",
                    record.raw,
                ]
            )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run distributed-controller tests and print a plain-language analysis."
    )
    parser.add_argument("--port", help="Serial port, e.g. COM11")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--algo", type=int, default=1, choices=[0, 1, 2], help="0=PRIMAL_DUAL, 1=ADMM, 2=CONSENSUS")
    parser.add_argument("--feedback", type=int, default=1, choices=[0, 1])
    parser.add_argument("--refs", default="20.0,20.0,20.0", help="Comma-separated refs for nodes 1..N")
    parser.add_argument("--equal-costs", default="1,1,1", help="Comma-separated costs for equal-cost scenario")
    parser.add_argument("--weighted-costs", default="1,3,8", help="Comma-separated costs for weighted scenario")
    parser.add_argument("--monitor-seconds", type=float, default=15.0)
    parser.add_argument("--sample-period", type=float, default=0.6)
    parser.add_argument("--reply-window", type=float, default=0.30)
    parser.add_argument("--delay", type=float, default=0.20)
    parser.add_argument("--settle-seconds", type=float, default=3.0, help="Wait after setup before sampling")
    parser.add_argument("--skip-calibration", action="store_true", help="Skip the automatic fresh calibration before testing")
    parser.add_argument("--calibration-seconds", type=float, default=9.0, help="How long to wait for the calibration FSM before starting tests")
    parser.add_argument("--skip-baseline", action="store_true", help="Skip the local PID baseline")
    parser.add_argument("--skip-weighted", action="store_true", help="Run only the equal-cost scenario")
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG/HTML graph generation")
    parser.add_argument("--plot-csv", help="Analyze an existing CSV instead of collecting live data")
    parser.add_argument("--out-dir", default="test_out")
    parser.add_argument("--prefix", default="")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    if not args.plot_csv and not args.port:
        parser.error("--port is required unless --plot-csv is used")

    refs = parse_csv_floats(args.refs, 3, "refs")
    equal_costs = parse_csv_floats(args.equal_costs, 3, "equal-costs")
    weighted_costs = parse_csv_floats(args.weighted_costs, 3, "weighted-costs")

    equal_cfg = ScenarioConfig(
        name="equal_cost",
        algo=args.algo,
        refs=refs,
        costs=equal_costs,
        feedback=args.feedback,
    )
    weighted_cfg = ScenarioConfig(
        name="weighted_cost",
        algo=args.algo,
        refs=refs,
        costs=weighted_costs,
        feedback=args.feedback,
    )
    baseline_cfg = ScenarioConfig(
        name="pid_baseline",
        algo=args.algo,
        refs=refs,
        costs=equal_costs,
        feedback=args.feedback,
        start_mode=1,
        distributed=False,
    )

    out_dir = Path(args.out_dir)
    prefix = args.prefix.strip()
    if args.plot_csv:
        prefix = prefix or Path(args.plot_csv).stem
    else:
        prefix = prefix or datetime.now().strftime(
            f"controller_test_%Y%m%d_%H%M%S_{algorithm_name(args.algo)}"
        )
    csv_path = out_dir / f"{prefix}.csv"

    scenario_configs = {
        baseline_cfg.name: baseline_cfg,
        equal_cfg.name: equal_cfg,
        weighted_cfg.name: weighted_cfg,
    }

    all_records: List[Sample] = []
    scenario_summaries: Dict[str, Dict[int, Dict[str, float]]] = {}

    if args.plot_csv:
        all_records = load_csv_records(Path(args.plot_csv))
        if not all_records:
            raise SystemExit(f"No samples found in {args.plot_csv}")
        nodes = sorted({record.node for record in all_records})
        print(f"Loaded {len(all_records)} samples from {args.plot_csv}")
        print(f"Nodes in CSV: {nodes}")
        for scenario_name in unique_scenarios(all_records):
            config = scenario_configs.get(scenario_name)
            if config is None:
                print(f"Skipping text analysis for unknown scenario `{scenario_name}`.")
                continue
            scenario_records = [record for record in all_records if record.scenario == scenario_name]
            scenario_summaries[scenario_name] = analyze_scenario(scenario_records, nodes, config)
    else:
        if serial is None:
            raise SystemExit(
                "Missing dependency: pyserial. Install with: pip install pyserial"
            ) from SERIAL_IMPORT_ERROR

        print(f"Opening {args.port} @ {args.baud}...")
        print(f"Distributed algorithm: {algorithm_name(args.algo)}")
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
            time.sleep(1.0)
            nodes = discover_nodes(ser, args.reply_window)
            if not nodes:
                raise SystemExit("No nodes replied to `g i 0`. Check the hub port and power.")
            print(f"Active nodes: {nodes}")
            if nodes != [1, 2, 3]:
                print("Warning: expected nodes [1, 2, 3]. Continuing with discovered nodes.")
            if len(nodes) != 3:
                raise SystemExit("This script expects exactly 3 nodes for the current firmware.")

            if not args.skip_calibration:
                run_calibration(ser, args.calibration_seconds, args.reply_window, args.delay)

            baseline_summary: Optional[Dict[int, Dict[str, float]]] = None
            if not args.skip_baseline:
                run_setup(ser, nodes, baseline_cfg, args.delay, args.reply_window)
                print(f"\n== Settling {args.settle_seconds:.1f}s ==")
                time.sleep(args.settle_seconds)
                baseline_records = collect_samples(
                    ser,
                    nodes,
                    baseline_cfg,
                    args.monitor_seconds,
                    args.sample_period,
                    args.reply_window,
                )
                all_records.extend(baseline_records)
                baseline_summary = analyze_scenario(baseline_records, nodes, baseline_cfg)
                scenario_summaries[baseline_cfg.name] = baseline_summary

            run_setup(ser, nodes, equal_cfg, args.delay, args.reply_window)
            print(f"\n== Settling {args.settle_seconds:.1f}s ==")
            time.sleep(args.settle_seconds)
            equal_records = collect_samples(
                ser,
                nodes,
                equal_cfg,
                args.monitor_seconds,
                args.sample_period,
                args.reply_window,
            )
            all_records.extend(equal_records)
            equal_summary = analyze_scenario(equal_records, nodes, equal_cfg)
            scenario_summaries[equal_cfg.name] = equal_summary
            if baseline_summary is not None:
                compare_to_baseline(nodes, baseline_cfg, baseline_summary, equal_cfg, equal_summary)

            if not args.skip_weighted:
                run_setup(ser, nodes, weighted_cfg, args.delay, args.reply_window)
                print(f"\n== Settling {args.settle_seconds:.1f}s ==")
                time.sleep(args.settle_seconds)
                weighted_records = collect_samples(
                    ser,
                    nodes,
                    weighted_cfg,
                    args.monitor_seconds,
                    args.sample_period,
                    args.reply_window,
                )
                all_records.extend(weighted_records)
                weighted_summary = analyze_scenario(weighted_records, nodes, weighted_cfg)
                scenario_summaries[weighted_cfg.name] = weighted_summary
                if baseline_summary is not None:
                    compare_to_baseline(nodes, baseline_cfg, baseline_summary, weighted_cfg, weighted_summary)
                compare_scenarios(nodes, equal_cfg, equal_summary, weighted_cfg, weighted_summary)

        write_csv(all_records, csv_path)
        print(f"\nSaved CSV: {csv_path}")

    if "pid_baseline" in scenario_summaries and "equal_cost" in scenario_summaries and args.plot_csv:
        compare_to_baseline(
            nodes,
            baseline_cfg,
            scenario_summaries["pid_baseline"],
            equal_cfg,
            scenario_summaries["equal_cost"],
        )
    if "pid_baseline" in scenario_summaries and "weighted_cost" in scenario_summaries and args.plot_csv:
        compare_to_baseline(
            nodes,
            baseline_cfg,
            scenario_summaries["pid_baseline"],
            weighted_cfg,
            scenario_summaries["weighted_cost"],
        )
    if "equal_cost" in scenario_summaries and "weighted_cost" in scenario_summaries and args.plot_csv:
        compare_scenarios(
            nodes,
            equal_cfg,
            scenario_summaries["equal_cost"],
            weighted_cfg,
            scenario_summaries["weighted_cost"],
        )

    if not args.no_plots:
        plot_paths = write_plots(all_records, nodes, scenario_configs, out_dir, prefix)
        for path in plot_paths:
            print(f"Saved plot: {path}")

    print("\nWhat you want to see:")
    if not args.plot_csv and not args.skip_calibration:
        print("- These results include a fresh calibration taken immediately before the test.")
    print("- Lux values close to each node reference, especially in the last 40% of samples.")
    print("- Duty not stuck at 1.0 all the time; otherwise the test is saturated.")
    if args.algo == 1:
        print("- With ADMM, the duties should settle instead of jumping violently between samples.")
        print("- With weighted costs, expensive nodes should tend to back off if the calibrated model is sane.")
    else:
        print("- With equal costs, stronger nodes may naturally carry more load.")
        print("- With weighted costs, expensive nodes should tend to reduce duty if the target is achievable.")


if __name__ == "__main__":
    main()
