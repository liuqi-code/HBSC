#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Summarize and plot the J138 ablation experiment results."""

from __future__ import annotations

import csv
import argparse
import math
import os
import statistics
from collections import defaultdict
from typing import Dict, Iterable, List, Tuple

from step4_config import METHOD_ORDER, PROJECT_ROOT, RESULTS_DIR, ensure_dirs


SUMMARY_FIELDS = [
    "run_type",
    "experiment",
    "scenario",
    "method",
    "method_label",
    "n",
    "MAE",
    "RMSE",
    "MaxDelay",
    "MaxAbsError",
    "MeanEstimatedDelay",
    "StdEstimatedDelay",
    "StdAbsError",
    "CI95_MAE",
]

PAIRED_FIELDS = [
    "experiment",
    "scenario",
    "baseline_method",
    "method",
    "n_pairs",
    "baseline_mean_abs_error",
    "method_mean_abs_error",
    "mean_diff_method_minus_baseline",
    "std_diff",
    "ci95_diff",
    "win_rate_method_better",
]


def _resolve_results_dir(results_dir: str | None) -> str:
    if not results_dir:
        return RESULTS_DIR
    if os.path.isabs(results_dir):
        return results_dir
    return os.path.join(PROJECT_ROOT, results_dir)


def _paths(results_dir: str) -> dict:
    analysis_dir = os.path.join(results_dir, "analysis")
    return {
        "results_dir": results_dir,
        "analysis_dir": analysis_dir,
        "method_estimates": os.path.join(results_dir, "method_estimates.csv"),
        "summary": os.path.join(analysis_dir, "ablation_summary.csv"),
        "paired": os.path.join(analysis_dir, "paired_method_comparison.csv"),
    }


def _find_sensitivity_csv(results_dir: str) -> str | None:
    if not os.path.isdir(results_dir):
        return None
    candidates = [
        os.path.join(results_dir, name)
        for name in os.listdir(results_dir)
        if name.startswith("method_estimates_sensitivity_") and name.endswith(".csv")
    ]
    return sorted(candidates)[0] if candidates else None


def _primary_run_type(summary_rows: List[dict]) -> str:
    run_types = sorted({row["run_type"] for row in summary_rows})
    if "main_no_cap" in run_types:
        return "main_no_cap"
    for run_type in run_types:
        if run_type.startswith("main_"):
            return run_type
    return run_types[0] if run_types else ""


def _read_rows(path: str) -> List[dict]:
    if not os.path.exists(path):
        return []
    with open(path, "r", encoding="utf-8-sig", newline="") as fh:
        return list(csv.DictReader(fh))


def _to_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _finite(values: Iterable[float]) -> List[float]:
    return [float(v) for v in values if math.isfinite(float(v))]


def _mean(values: List[float]) -> float:
    return statistics.mean(values) if values else float("nan")


def _std(values: List[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else 0.0


def _ci95(values: List[float]) -> float:
    if len(values) <= 1:
        return 0.0
    return 1.96 * _std(values) / math.sqrt(len(values))


def _fmt(value: float) -> str:
    if not math.isfinite(float(value)):
        return ""
    return f"{float(value):.6f}"


def summarize(rows: List[dict]) -> List[dict]:
    grouped: Dict[Tuple[str, str, str, str], List[dict]] = defaultdict(list)
    for row in rows:
        grouped[(row["run_type"], row["experiment"], row["scenario"], row["method"])].append(row)

    summary = []
    for key, group_rows in sorted(grouped.items()):
        run_type, experiment, scenario, method = key
        abs_errors = _finite(_to_float(row["abs_error"]) for row in group_rows)
        squared_errors = _finite(_to_float(row["squared_error"]) for row in group_rows)
        estimated = _finite(_to_float(row["estimated_delay"]) for row in group_rows)
        mae = _mean(abs_errors)
        rmse = math.sqrt(_mean(squared_errors)) if squared_errors else float("nan")

        summary.append(
            {
                "run_type": run_type,
                "experiment": experiment,
                "scenario": scenario,
                "method": method,
                "method_label": group_rows[0].get("method_label", ""),
                "n": len(abs_errors),
                "MAE": _fmt(mae),
                "RMSE": _fmt(rmse),
                "MaxDelay": _fmt(max(estimated) if estimated else float("nan")),
                "MaxAbsError": _fmt(max(abs_errors) if abs_errors else float("nan")),
                "MeanEstimatedDelay": _fmt(_mean(estimated)),
                "StdEstimatedDelay": _fmt(_std(estimated)),
                "StdAbsError": _fmt(_std(abs_errors)),
                "CI95_MAE": _fmt(_ci95(abs_errors)),
            }
        )
    return summary


def paired_comparison(rows: List[dict], baseline: str = "M1") -> List[dict]:
    per_seed: Dict[Tuple[str, str, str, str], List[float]] = defaultdict(list)
    labels: Dict[Tuple[str, str, str, str], str] = {}
    for row in rows:
        err = _to_float(row["abs_error"])
        if not math.isfinite(err):
            continue
        key = (row["experiment"], row["scenario"], row["seed"], row["method"])
        per_seed[key].append(err)
        labels[key] = row.get("method_label", "")

    mae_by_seed = {
        key: statistics.mean(values)
        for key, values in per_seed.items()
        if values
    }

    scenario_methods = defaultdict(set)
    for experiment, scenario, seed, method in mae_by_seed:
        scenario_methods[(experiment, scenario)].add(method)

    output = []
    for experiment, scenario in sorted(scenario_methods):
        methods = sorted(scenario_methods[(experiment, scenario)])
        seeds = sorted(
            {
                seed
                for exp, scen, seed, method in mae_by_seed
                if exp == experiment and scen == scenario
            }
        )
        for method in methods:
            if method == baseline:
                continue
            pairs = []
            baseline_values = []
            method_values = []
            for seed in seeds:
                b_key = (experiment, scenario, seed, baseline)
                m_key = (experiment, scenario, seed, method)
                if b_key not in mae_by_seed or m_key not in mae_by_seed:
                    continue
                b = mae_by_seed[b_key]
                m = mae_by_seed[m_key]
                baseline_values.append(b)
                method_values.append(m)
                pairs.append(m - b)
            if not pairs:
                continue
            output.append(
                {
                    "experiment": experiment,
                    "scenario": scenario,
                    "baseline_method": baseline,
                    "method": method,
                    "n_pairs": len(pairs),
                    "baseline_mean_abs_error": _fmt(_mean(baseline_values)),
                    "method_mean_abs_error": _fmt(_mean(method_values)),
                    "mean_diff_method_minus_baseline": _fmt(_mean(pairs)),
                    "std_diff": _fmt(_std(pairs)),
                    "ci95_diff": _fmt(_ci95(pairs)),
                    "win_rate_method_better": _fmt(
                        sum(1 for value in pairs if value < 0) / len(pairs)
                    ),
                }
            )
    return output


def _write_csv(path: str, fields: List[str], rows: List[dict]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot_grouped_bars(summary_rows: List[dict], experiment: str, out_path: str) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    primary_run_type = _primary_run_type(summary_rows)
    rows = [
        row
        for row in summary_rows
        if row["experiment"] == experiment and row["run_type"] == primary_run_type
    ]
    if not rows:
        return

    scenarios = sorted({row["scenario"] for row in rows})
    method_labels = {row["method"]: row.get("method_label", row["method"]) for row in rows}
    values = {
        (row["scenario"], row["method"]): (_to_float(row["MAE"]), _to_float(row["CI95_MAE"]))
        for row in rows
    }

    x = np.arange(len(scenarios))
    width = min(0.16, 0.78 / len(METHOD_ORDER))
    fig, ax = plt.subplots(figsize=(10, 4.8))
    colors = ["#2F6B9A", "#D07C40", "#3C8D5A", "#B84A62", "#6D5BA6"]

    for idx, method in enumerate(METHOD_ORDER):
        offset = (idx - (len(METHOD_ORDER) - 1) / 2) * width
        mae = [values.get((scenario, method), (float("nan"), 0.0))[0] for scenario in scenarios]
        ci = [values.get((scenario, method), (float("nan"), 0.0))[1] for scenario in scenarios]
        ax.bar(
            x + offset,
            mae,
            width,
            yerr=ci,
            capsize=2,
            label=f"{method}",
            color=colors[idx % len(colors)],
        )

    ax.set_ylabel("MAE (s)")
    ax.set_xlabel("Scenario")
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(ncol=5, fontsize=8)
    ax.set_title(f"{experiment} ({primary_run_type})")
    fig.tight_layout()
    fig.savefig(out_path, format="svg")
    plt.close(fig)


def _plot_sensitivity(main_rows: List[dict], sensitivity_rows: List[dict], out_path: str) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    if not sensitivity_rows:
        return
    main_summary = summarize(main_rows)
    sens_summary = summarize(sensitivity_rows)

    def overall(summary_rows: List[dict]) -> Dict[str, float]:
        by_method = defaultdict(list)
        primary_run_type = _primary_run_type(summary_rows)
        for row in summary_rows:
            if row["run_type"] == primary_run_type or row["run_type"].startswith("sensitivity_"):
                by_method[row["method"]].append(_to_float(row["MAE"]))
        return {method: _mean(_finite(values)) for method, values in by_method.items()}

    main = overall(main_summary)
    sens = overall(sens_summary)
    methods = METHOD_ORDER
    x = np.arange(len(methods))
    width = 0.35
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    ax.bar(x - width / 2, [main.get(m, float("nan")) for m in methods], width, label="Main")
    ax.bar(x + width / 2, [sens.get(m, float("nan")) for m in methods], width, label="Sensitivity")
    ax.set_ylabel("Overall MAE (s)")
    ax.set_xticks(x)
    ax.set_xticklabels(methods)
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    ax.set_title("Flow-cap sensitivity")
    fig.tight_layout()
    fig.savefig(out_path, format="svg")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-dir",
        default=None,
        help="Result directory. Relative paths are resolved under 4_IntersectionAblation.",
    )
    args = parser.parse_args()

    ensure_dirs()
    results_dir = _resolve_results_dir(args.results_dir)
    paths = _paths(results_dir)
    os.makedirs(paths["analysis_dir"], exist_ok=True)

    rows = _read_rows(paths["method_estimates"])
    if not rows:
        print(f"No method estimate file found: {paths['method_estimates']}")
        return

    summary_rows = summarize(rows)
    paired_rows = paired_comparison(rows)
    _write_csv(paths["summary"], SUMMARY_FIELDS, summary_rows)
    _write_csv(paths["paired"], PAIRED_FIELDS, paired_rows)

    _plot_grouped_bars(
        summary_rows,
        "ExpA_saturation",
        os.path.join(paths["analysis_dir"], "mae_by_saturation.svg"),
    )
    _plot_grouped_bars(
        summary_rows,
        "ExpB_uncertainty",
        os.path.join(paths["analysis_dir"], "mae_by_uncertainty.svg"),
    )

    sensitivity_path = _find_sensitivity_csv(results_dir)
    sensitivity_rows = _read_rows(sensitivity_path) if sensitivity_path else []
    _plot_sensitivity(
        rows,
        sensitivity_rows,
        os.path.join(paths["analysis_dir"], "sensitivity_main_vs_sensitivity.svg"),
    )

    print(f"Wrote {paths['summary']}")
    print(f"Wrote {paths['paired']}")
    print(f"Wrote plots under {paths['analysis_dir']}")


if __name__ == "__main__":
    main()
