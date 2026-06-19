# Evaluation

The official PDF defines evaluation around mission completion, optical communication quality, formation control, CV robustness, code quality, and reproducibility.

## Public Scenarios

Recommended public scenarios:

| Scenario | Description | Purpose |
| --- | --- | --- |
| T1 | 3 drones, daytime, straight or simple route | Baseline MVP |
| T2 | 3 drones, route with turns | Follower-control validation |
| T3 | Temporary LED loss or short occlusion | Timeout and reacquisition |
| T4 | Lighting/distance variation | CV robustness |

## Hidden Scenarios

Recommended hidden scenarios:

| Scenario | Description | Purpose |
| --- | --- | --- |
| H1 | Changed route/checkpoints | Generalization |
| H2 | Different start offsets | Formation robustness |
| H3 | Short frame drops or signal gaps | Decoder stability |
| H4 | 5 drones | Optional scaling bonus |

Hidden tests should not make the task random. They should test generality without changing the official rules or allowed data.

## Scoring Rubric

| Criterion | Weight | What is evaluated |
| --- | ---: | --- |
| Mission completion | 25% | Target/checkpoint completion, no crashes, no collisions, completion time |
| Optical communication | 25% | Protocol, packet delivery ratio, latency, decoding, tolerance to missed frames |
| Formation and control | 15% | Formation error, distance to leader, movement stability |
| Computer vision | 15% | LED/marker detection across angle, distance, and lighting changes |
| Robustness | 10% | Signal reacquisition, safe behavior on loss, hidden-test performance |
| Code, docs, presentation | 10% | Readability, structure, README, reproducibility, architecture explanation |

## Metrics

| Metric | Definition | Interpretation |
| --- | --- | --- |
| `mission_success` | `0/1` or partial checkpoint score | Main completion metric |
| `pdr_percent` | `(received / sent) * 100` | Optical channel quality |
| `latency_ms_avg` | Time between signal emission and decode | Lower is better if accuracy is preserved |
| `formation_error_m_avg` | Average deviation from target formation/distance | Lower is better |
| `lost_signal_time_s` | Total time without valid signal | Lower is better |
| `collisions` | Number of drone/obstacle collisions | Critical penalty signal |
| `completion_time_s` | Mission completion time | Used after mission validity is checked |

## Suggested `results.json`

```json
{
  "team": "team_name",
  "scenario": "T1_day_follow",
  "mission_success": 1,
  "completed_checkpoints": 3,
  "completion_time_s": 128.4,
  "pdr_percent": 87.5,
  "latency_ms_avg": 94.2,
  "formation_error_m_avg": 1.35,
  "lost_signal_time_s": 4.8,
  "collisions": 0,
  "forbidden_topics_detected": false,
  "score": 82.0
}
```

## Penalties and Disqualification

| Event | Recommended action |
| --- | --- |
| Reading prohibited topics or ground truth | Zero for the scenario or disqualification after review |
| Critical collision or crash | Scenario not counted; partial protocol/code score may remain |
| Manual intervention during final test | Scenario not counted |
| Non-reproducible launch | Only code/presentation points if the issue is not organizer-side |
| LED-budget violation | Reduce optical-communication score or move result to bonus category |

## Current Repository Gap

The source PDF references `evaluate.py`, but this repository currently does not include it. Until it exists, teams should still produce the documented `results.json` shape manually or via their own evaluation script.

