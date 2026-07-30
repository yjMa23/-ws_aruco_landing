#!/usr/bin/env python3
"""聚合 P7 episode 结果，输出 JSON 与 CSV。"""

from __future__ import annotations

import argparse
import collections
import json
import sys
from pathlib import Path
from typing import Any

from p7_experiment_utils import (
    FAILURE_TYPES,
    METRIC_FIELDS,
    atomic_write_json,
    metric_value,
    read_evaluation_json,
    read_json,
    summarize_values,
    utc_now_iso,
    write_csv,
)

FAILURE_CSV_FIELDS = (
    "episode_id",
    "scenario",
    "seed",
    "failure_reason",
    "failure_detail",
    "episode_directory",
)
SUMMARY_CSV_FIELDS = (
    "metric",
    "count",
    "mean",
    "stddev",
    "median",
    "p90",
    "p95",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Aggregate a P7 batch directory.")
    parser.add_argument("batch_directory", type=Path)
    return parser.parse_args()


def collect_records(batch_dir: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []
    if not batch_dir.is_dir():
        raise ValueError(f"batch directory does not exist: {batch_dir}")
    for episode_dir in sorted(path for path in batch_dir.iterdir() if path.is_dir()):
        manifest_path = episode_dir / "manifest.json"
        if not manifest_path.is_file():
            continue
        try:
            manifest = read_json(manifest_path)
        except ValueError as error:
            issues.append(
                {
                    "episode_id": episode_dir.name,
                    "scenario": None,
                    "seed": None,
                    "failure_reason": "EVALUATION_ERROR",
                    "failure_detail": str(error),
                    "episode_directory": str(episode_dir),
                }
            )
            continue
        evaluation: dict[str, Any] | None = None
        evaluation_path = manifest.get("evaluation_path")
        if evaluation_path:
            try:
                evaluation = read_evaluation_json(Path(str(evaluation_path)))
            except ValueError as error:
                issues.append(
                    {
                        "episode_id": manifest.get("episode_id", episode_dir.name),
                        "scenario": manifest.get("scenario"),
                        "seed": manifest.get("seed"),
                        "failure_reason": "EVALUATION_ERROR",
                        "failure_detail": str(error),
                        "episode_directory": str(episode_dir),
                    }
                )
        records.append(
            {
                "manifest": manifest,
                "evaluation": evaluation,
                "episode_directory": episode_dir,
            }
        )
    return records, issues


def aggregate(batch_dir: Path) -> dict[str, Any]:
    records, issues = collect_records(batch_dir)
    completed_records = [
        record for record in records if record["manifest"].get("completed") is True
    ]
    successful_records = [
        record for record in completed_records if record["manifest"].get("success") is True
    ]
    total = len(completed_records)
    success_count = len(successful_records)
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
                "scenario": manifest.get("scenario"),
                "seed": manifest.get("seed"),
                "failure_reason": reason,
                "failure_detail": manifest.get("failure_detail"),
                "episode_directory": str(record["episode_directory"]),
            }
        )
    if issues:
        failure_counts["EVALUATION_ERROR"] += len(issues)

    metrics: dict[str, dict[str, Any]] = {}
    for field in METRIC_FIELDS:
        values: list[float] = []
        for record in successful_records:
            evaluation = record["evaluation"]
            if evaluation is None:
                continue
            value = metric_value(evaluation, field)
            if value is not None:
                values.append(value)
        metrics[field] = summarize_values(values)

    failure_breakdown = {
        reason: {
            "count": count,
            "ratio": (count / total if total else 0.0),
        }
        for reason, count in sorted(failure_counts.items())
    }
    summary = {
        "batch_directory": str(batch_dir),
        "generated_at": utc_now_iso(),
        "total_experiments": total,
        "success_count": success_count,
        "success_rate": (success_count / total if total else 0.0),
        "failure_count": total - success_count + len(issues),
        "failure_breakdown": failure_breakdown,
        "corrupt_result_count": len(issues),
        "metrics": metrics,
    }
    atomic_write_json(batch_dir / "summary.json", summary)
    write_csv(
        batch_dir / "summary.csv",
        SUMMARY_CSV_FIELDS,
        ({"metric": field, **statistics} for field, statistics in metrics.items()),
    )
    write_csv(batch_dir / "failures.csv", FAILURE_CSV_FIELDS, failure_rows)
    return summary


def main() -> int:
    args = parse_arguments()
    try:
        summary = aggregate(args.batch_directory.expanduser().resolve())
    except (OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
