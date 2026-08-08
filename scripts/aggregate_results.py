#!/usr/bin/env python3
"""聚合实验 episode，输出分组 JSON、CSV、Markdown 和可选 PNG。"""

from __future__ import annotations

import argparse
import collections
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Callable, Iterable

from experiment_utils import (
    FAILURE_TYPES,
    METRIC_FIELDS,
    atomic_write_json,
    metric_value,
    read_json,
    summarize_values,
    utc_now_iso,
    write_csv,
)

FAILURE_CSV_FIELDS = (
    "episode_id",
    "method",
    "scenario",
    "profile",
    "seed",
    "failure_reason",
    "failure_detail",
    "episode_directory",
    "bag_path",
)
SUMMARY_CSV_FIELDS = (
    "group",
    "metric",
    "count",
    "mean",
    "stddev",
    "median",
    "p90",
    "p95",
    "min",
    "max",
)
GROUP_CSV_FIELDS = (
    "group",
    "total_experiments",
    "success_count",
    "failure_count",
    "success_rate",
    "metric",
    "count",
    "mean",
    "stddev",
    "median",
    "p90",
    "p95",
    "min",
    "max",
)
ARCHIVED_EPISODE_PATTERN = re.compile(
    r"_(?:failed|superseded|interrupted)_attempt\d+$"
)

EPISODE_BASE_FIELDS = (
    "episode_id",
    "batch_id",
    "method",
    "scenario",
    "profile",
    "repetition",
    "seed",
    "success",
    "failure_reason",
    "duration_s",
    "git_commit",
    "dirty_worktree",
    "bag_path",
    "evaluation_path",
    "legacy_evaluation_path",
    "nav_land_count",
    "disarm_count",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Aggregate an experiment batch directory.")
    parser.add_argument("batch_directory", type=Path)
    parser.add_argument(
        "--no-plots", action="store_true", help="skip optional matplotlib plots"
    )
    return parser.parse_args()


def collect_records(batch_dir: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []
    if not batch_dir.is_dir():
        raise ValueError(f"batch directory does not exist: {batch_dir}")
    for episode_dir in sorted(path for path in batch_dir.iterdir() if path.is_dir()):
        if ARCHIVED_EPISODE_PATTERN.search(episode_dir.name):
            continue
        manifest_path = episode_dir / "manifest.json"
        if not manifest_path.is_file():
            continue
        try:
            manifest = read_json(manifest_path)
        except ValueError as error:
            issues.append(
                {
                    "episode_id": episode_dir.name,
                    "method": None,
                    "scenario": None,
                    "profile": None,
                    "seed": None,
                    "failure_reason": "EVALUATION_ERROR",
                    "failure_detail": str(error),
                    "episode_directory": str(episode_dir),
                    "bag_path": None,
                }
            )
            continue
        evaluation: dict[str, Any] | None = None
        evaluation_path = manifest.get("evaluation_path")
        if evaluation_path:
            try:
                evaluation = read_json(Path(str(evaluation_path)))
            except ValueError as error:
                issues.append(
                    {
                        "episode_id": manifest.get("episode_id", episode_dir.name),
                        "method": manifest.get("method", "B0"),
                        "scenario": manifest.get("scenario"),
                        "profile": manifest.get(
                            "profile", manifest.get("experiment_profile", "touchdown")
                        ),
                        "seed": manifest.get("seed"),
                        "failure_reason": "EVALUATION_ERROR",
                        "failure_detail": str(error),
                        "episode_directory": str(episode_dir),
                        "bag_path": manifest.get("bag_path"),
                    }
                )
        normalized_manifest = {
            **manifest,
            "method": manifest.get("method", "B0"),
            "profile": manifest.get(
                "profile", manifest.get("experiment_profile", "touchdown")
            ),
        }
        records.append(
            {
                "manifest": normalized_manifest,
                "evaluation": evaluation,
                "episode_directory": episode_dir,
            }
        )
    return records, issues


def _group_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    completed = [
        record for record in records if record["manifest"].get("completed") is True
    ]
    successful = [
        record for record in completed if record["manifest"].get("success") is True
    ]
    total = len(completed)
    success_count = len(successful)
    metrics: dict[str, dict[str, Any]] = {}
    for field in METRIC_FIELDS:
        values: list[float] = []
        for record in successful:
            evaluation = record["evaluation"]
            if evaluation is None:
                continue
            value = metric_value(evaluation, field)
            if value is not None:
                values.append(value)
        metrics[field] = summarize_values(values)
    return {
        "total_experiments": total,
        "success_count": success_count,
        "failure_count": total - success_count,
        "success_rate": success_count / total if total else 0.0,
        "metrics": metrics,
    }


def _group_by(
    records: list[dict[str, Any]], key_fn: Callable[[dict[str, Any]], str]
) -> dict[str, dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = collections.defaultdict(list)
    for record in records:
        groups[key_fn(record)].append(record)
    return {key: _group_summary(value) for key, value in sorted(groups.items())}


def _command_counts(evaluation: dict[str, Any] | None) -> tuple[int | None, int | None]:
    if evaluation is None:
        return None, None
    nav_candidates = (
        "nav_land_count",
        "nav_land_commands",
        "nav_land_command_count",
    )
    disarm_candidates = (
        "automatic_disarm_count",
        "disarm_commands",
        "disarm_command_count",
    )

    def first_int(keys: Iterable[str]) -> int | None:
        for key in keys:
            value = evaluation.get(key)
            if value is None:
                continue
            try:
                return int(value)
            except (TypeError, ValueError):
                continue
        return None

    return first_int(nav_candidates), first_int(disarm_candidates)


def _episode_rows(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for record in records:
        manifest = record["manifest"]
        evaluation = record["evaluation"]
        nav_count, disarm_count = _command_counts(evaluation)
        row: dict[str, Any] = {
            field: manifest.get(field) for field in EPISODE_BASE_FIELDS
        }
        row["profile"] = manifest.get(
            "profile", manifest.get("experiment_profile", "touchdown")
        )
        row["nav_land_count"] = nav_count
        row["disarm_count"] = disarm_count
        for field in METRIC_FIELDS:
            row[field] = metric_value(evaluation or {}, field)
        rows.append(row)
    return rows


def _group_csv_rows(groups: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for group_name, summary in groups.items():
        for metric, statistics in summary["metrics"].items():
            rows.append(
                {
                    "group": group_name,
                    "total_experiments": summary["total_experiments"],
                    "success_count": summary["success_count"],
                    "failure_count": summary["failure_count"],
                    "success_rate": summary["success_rate"],
                    "metric": metric,
                    **statistics,
                }
            )
    return rows


def _format_number(value: Any, digits: int = 4) -> str:
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_markdown(
    batch_dir: Path,
    summary: dict[str, Any],
    failure_rows: list[dict[str, Any]],
    plot_status: dict[str, Any],
) -> None:
    overall = summary["overall"]
    lines = [
        "# paper evaluation Results Summary",
        "",
        f"- Batch: `{batch_dir.name}`",
        f"- Generated: `{summary['generated_at']}`",
        f"- Executed: `{overall['total_experiments']}`",
        f"- Success: `{overall['success_count']}`",
        f"- Failure: `{overall['failure_count']}`",
        f"- Success rate: `{overall['success_rate']:.2%}`",
        "",
        "## By method",
        "",
        "| Method | N | Success | Rate | Horizontal RMSE mean (m) | Landing time mean (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for method, group in summary["by_method"].items():
        lines.append(
            "| {method} | {total} | {success} | {rate:.2%} | {rmse} | {time} |".format(
                method=method,
                total=group["total_experiments"],
                success=group["success_count"],
                rate=group["success_rate"],
                rmse=_format_number(
                    group["metrics"]["horizontal_error_rmse_m"]["mean"]
                ),
                time=_format_number(group["metrics"]["landing_time_s"]["mean"]),
            )
        )
    lines.extend(
        [
            "",
            "## By scenario",
            "",
            "| Scenario | N | Success | Rate | Horizontal RMSE P95 (m) |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for scenario, group in summary["by_scenario"].items():
        lines.append(
            "| {scenario} | {total} | {success} | {rate:.2%} | {rmse} |".format(
                scenario=scenario,
                total=group["total_experiments"],
                success=group["success_count"],
                rate=group["success_rate"],
                rmse=_format_number(
                    group["metrics"]["horizontal_error_rmse_m"]["p95"]
                ),
            )
        )
    lines.extend(["", "## Failure breakdown", ""])
    if summary["failure_breakdown"]:
        lines.extend(
            [
                "| Reason | Count | Ratio |",
                "| --- | ---: | ---: |",
            ]
        )
        for reason, detail in summary["failure_breakdown"].items():
            lines.append(
                f"| {reason} | {detail['count']} | {detail['ratio']:.2%} |"
            )
    else:
        lines.append("No executed failures.")
    lines.extend(
        [
            "",
            "## Safety counters",
            "",
            f"- NAV_LAND count: `{summary['safety']['nav_land_count']}`",
            f"- Automatic Disarm count: `{summary['safety']['disarm_count']}`",
            f"- Missing counter episodes: `{summary['safety']['missing_counter_episodes']}`",
            "",
            "## Result artifacts",
            "",
            "`summary.json`, `summary.csv`, `episodes.csv`, `failures.csv`, "
            "`by_method.csv`, `by_scenario.csv`, `by_method_scenario.csv`, "
            "`experiment_matrix.csv` and this Markdown file.",
            "",
            "## Plot generation",
            "",
            f"- Status: `{plot_status.get('status')}`",
            f"- Detail: `{plot_status.get('detail')}`",
        ]
    )
    if failure_rows:
        lines.extend(["", "## Failure evidence", ""])
        for row in failure_rows:
            lines.append(
                f"- `{row.get('episode_id')}`: `{row.get('failure_reason')}` — "
                f"`{row.get('episode_directory')}`"
            )
    (batch_dir / "RESULTS_SUMMARY.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def generate_plots(batch_dir: Path, summary: dict[str, Any]) -> dict[str, Any]:
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        return {"status": "SKIPPED", "detail": f"matplotlib unavailable: {error}"}

    plots_dir = batch_dir / "plots"
    plots_dir.mkdir(exist_ok=True)
    generated: list[str] = []

    def save_bar(filename: str, title: str, labels: list[str], values: list[float],
                 ylabel: str) -> None:
        if not labels:
            return
        figure = plt.figure(figsize=(8, 5))
        axis = figure.add_subplot(111)
        axis.bar(labels, values)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.tick_params(axis="x", rotation=30)
        figure.tight_layout()
        path = plots_dir / filename
        figure.savefig(path, dpi=160)
        plt.close(figure)
        generated.append(str(path))

    methods = list(summary["by_method"])
    save_bar(
        "success_rate_by_method.png",
        "Success rate by method",
        methods,
        [summary["by_method"][key]["success_rate"] for key in methods],
        "Success rate",
    )
    rmse_methods = [
        key for key in methods
        if summary["by_method"][key]["metrics"]["horizontal_error_rmse_m"]["mean"]
        is not None
    ]
    save_bar(
        "horizontal_rmse_by_method.png",
        "Horizontal RMSE by method",
        rmse_methods,
        [
            summary["by_method"][key]["metrics"]["horizontal_error_rmse_m"]["mean"]
            for key in rmse_methods
        ],
        "RMSE (m)",
    )
    scenarios = [
        key for key in summary["by_scenario"]
        if summary["by_scenario"][key]["metrics"]["landing_time_s"]["mean"]
        is not None
    ]
    save_bar(
        "landing_time_by_scenario.png",
        "Landing time by scenario",
        scenarios,
        [
            summary["by_scenario"][key]["metrics"]["landing_time_s"]["mean"]
            for key in scenarios
        ],
        "Time (s)",
    )
    failures = summary["failure_breakdown"]
    save_bar(
        "failure_breakdown.png",
        "Failure type distribution",
        list(failures),
        [detail["count"] for detail in failures.values()],
        "Count",
    )
    comparison = [key for key in ("B0", "B3") if key in summary["by_method"]]
    save_bar(
        "mpc_vs_rule_based_horizontal_rmse.png",
        "MPC vs adaptive rule-based tracking horizontal RMSE",
        comparison,
        [
            summary["by_method"][key]["metrics"]["horizontal_error_rmse_m"]["mean"]
            or 0.0
            for key in comparison
        ],
        "RMSE (m)",
    )
    t1_metrics = []
    if "B5" in summary["by_method"]:
        b5 = summary["by_method"]["B5"]["metrics"]
        for label, key in (
            ("Slip (m)", "touchdown_slip_m"),
            ("Attitude divergence (deg)", "attitude_divergence_increment_deg"),
        ):
            if b5[key]["mean"] is not None:
                t1_metrics.append((label, b5[key]["mean"]))
    save_bar(
        "t1_slip_attitude.png",
        "T1 contact stability",
        [item[0] for item in t1_metrics],
        [item[1] for item in t1_metrics],
        "Mean value",
    )
    return {"status": "PASS", "detail": generated}


def aggregate(batch_dir: Path, *, generate_plot_files: bool = True) -> dict[str, Any]:
    records, issues = collect_records(batch_dir)
    completed_records = [
        record for record in records if record["manifest"].get("completed") is True
    ]
    overall = _group_summary(completed_records)
    by_scenario = _group_by(
        completed_records, lambda record: str(record["manifest"].get("scenario"))
    )
    by_method = _group_by(
        completed_records, lambda record: str(record["manifest"].get("method", "B0"))
    )
    by_method_and_scenario = _group_by(
        completed_records,
        lambda record: (
            f"{record['manifest'].get('method', 'B0')}|"
            f"{record['manifest'].get('scenario')}"
        ),
    )

    failure_counts: collections.Counter[str] = collections.Counter()
    failure_rows: list[dict[str, Any]] = list(issues)
    for record in completed_records:
        manifest = record["manifest"]
        if manifest.get("success") is True:
            continue
        reason = str(manifest.get("failure_reason", "UNKNOWN"))
        if reason not in FAILURE_TYPES:
            reason = "UNKNOWN"
        failure_counts[reason] += 1
        failure_rows.append(
            {
                "episode_id": manifest.get("episode_id"),
                "method": manifest.get("method", "B0"),
                "scenario": manifest.get("scenario"),
                "profile": manifest.get(
                    "profile", manifest.get("experiment_profile", "touchdown")
                ),
                "seed": manifest.get("seed"),
                "failure_reason": reason,
                "failure_detail": manifest.get("failure_detail"),
                "episode_directory": str(record["episode_directory"]),
                "bag_path": manifest.get("bag_path"),
            }
        )
    if issues:
        failure_counts["EVALUATION_ERROR"] += len(issues)
    total = overall["total_experiments"]
    failure_breakdown = {
        reason: {"count": count, "ratio": count / total if total else 0.0}
        for reason, count in sorted(failure_counts.items())
    }

    episode_rows = _episode_rows(completed_records)
    nav_values = [row["nav_land_count"] for row in episode_rows if row["nav_land_count"] is not None]
    disarm_values = [row["disarm_count"] for row in episode_rows if row["disarm_count"] is not None]
    safety = {
        "nav_land_count": sum(nav_values),
        "disarm_count": sum(disarm_values),
        "missing_counter_episodes": sum(
            row["nav_land_count"] is None or row["disarm_count"] is None
            for row in episode_rows
        ),
    }
    summary = {
        "batch_directory": str(batch_dir),
        "generated_at": utc_now_iso(),
        "overall": overall,
        "by_scenario": by_scenario,
        "by_method": by_method,
        "by_method_and_scenario": by_method_and_scenario,
        "failure_breakdown": failure_breakdown,
        "corrupt_result_count": len(issues),
        "safety": safety,
        # Legacy top-level compatibility.
        "total_experiments": overall["total_experiments"],
        "success_count": overall["success_count"],
        "failure_count": overall["failure_count"] + len(issues),
        "success_rate": overall["success_rate"],
        "metrics": overall["metrics"],
    }
    atomic_write_json(batch_dir / "summary.json", summary)
    write_csv(
        batch_dir / "summary.csv",
        SUMMARY_CSV_FIELDS,
        (
            {"group": "overall", "metric": field, **statistics}
            for field, statistics in overall["metrics"].items()
        ),
    )
    write_csv(
        batch_dir / "episodes.csv",
        (*EPISODE_BASE_FIELDS, *METRIC_FIELDS),
        episode_rows,
    )
    write_csv(batch_dir / "failures.csv", FAILURE_CSV_FIELDS, failure_rows)
    write_csv(
        batch_dir / "by_method.csv", GROUP_CSV_FIELDS, _group_csv_rows(by_method)
    )
    write_csv(
        batch_dir / "by_scenario.csv", GROUP_CSV_FIELDS, _group_csv_rows(by_scenario)
    )
    write_csv(
        batch_dir / "by_method_scenario.csv",
        GROUP_CSV_FIELDS,
        _group_csv_rows(by_method_and_scenario),
    )
    plot_status = (
        generate_plots(batch_dir, summary)
        if generate_plot_files
        else {"status": "SKIPPED", "detail": "--no-plots"}
    )
    summary["plots"] = plot_status
    atomic_write_json(batch_dir / "summary.json", summary)
    write_markdown(batch_dir, summary, failure_rows, plot_status)
    return summary


def main() -> int:
    args = parse_arguments()
    try:
        summary = aggregate(
            args.batch_directory.expanduser().resolve(),
            generate_plot_files=not args.no_plots,
        )
    except (OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
