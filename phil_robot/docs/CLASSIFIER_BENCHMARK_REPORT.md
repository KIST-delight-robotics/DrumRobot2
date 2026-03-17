# Phil Robot Classifier Benchmark Report

## 1. Objective

This report summarizes the classifier-model selection experiment for the Phil robot LLM control pipeline.

Target operating constraints:

- classifier latency target: `<= 1.5 s`
- planner model: fixed to `qwen3:30b-a3b-instruct-2507-q4_K_M`
- evaluation protocol: `smoke` suite in [cases_smoke.json](/home/shy/robot_project/phil_robot/eval/cases_smoke.json)

Decision criteria:

- preserve classifier correctness on the current control/task distribution
- reduce stage-1 intent-classification latency
- maintain end-to-end pipeline stability under the current multi-layer path:
  `STT -> classifier -> domain planner -> validator -> executor -> TTS`

## 2. Experimental Setup

### 2.1 Evaluation Dataset

- suite: `smoke`
- number of cases: `11`
- case categories:
  - greeting / chat
  - motion request
  - play request
  - stop request
  - status question
  - relative-motion edge case
  - safety lock / busy-state blocking
  - general knowledge chat
  - robot identity question

### 2.2 Metrics

- `classifier_checks`
  - exact-match pass count for classifier expectations defined in the suite
- `cases`
  - full end-to-end pass count across classifier, planner, validator, and speech checks
- `avg_classifier_sec`
  - mean stage-1 classifier latency
- `avg_planner_sec`
  - mean stage-2 planner latency
- `avg_total_sec`
  - mean combined LLM latency per turn

### 2.3 Fixed Pipeline Configuration

- planner model: `qwen3:30b-a3b-instruct-2507-q4_K_M`
- validator / executor: current repository implementation
- state input: reduced LLM state summary from [state_adapter.py](/home/shy/robot_project/phil_robot/pipeline/state_adapter.py)

## 3. Broad Classifier Screening

The first pass screened a mixed set of instruct-capable local models using classifier-only evaluation on the `smoke` suite.

| Model | Size | Classifier Checks | Avg (s) | Max (s) | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| `llama3.1:8b-instruct-q5_K_M` | 5.7 GB | 11/11 | 4.678 | 5.972 | Baseline, accurate but slow |
| `llama3.2:3b-instruct-q4_K_M` | 2.0 GB | 10/11 | 4.270 | 5.981 | Misclassified general-knowledge chat |
| `phi3:3.8b-mini-4k-instruct-q4_K_S` | 2.2 GB | 5/11 | 8.889 | 23.182 | Unstable for Korean intent classification |
| `gemma:2b-instruct` | 1.6 GB | 9/11 | 5.680 | 12.469 | Accuracy loss on greeting / knowledge chat |
| `qwen2.5:3b-instruct` | 1.9 GB | 10/11 | 3.742 | 9.774 | Strong early candidate |
| `qwen2.5:7b-instruct` | 4.7 GB | 11/11 | 5.567 | 14.588 | Accurate but too slow for stage 1 |

## 4. Quantized Qwen-Focused Screening

The second pass focused on explicit Qwen quantized candidates closer to the target operating point.

| Model | Size | Classifier Checks | Avg (s) | Median (s) | Max (s) | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `qwen2.5:0.5b-instruct-q4_K_M` | 397 MB | 4/11 | 2.020 | 1.696 | 5.052 | Too inaccurate |
| `qwen2.5:1.5b-instruct-q4_K_M` | 986 MB | 8/11 | 2.239 | 1.991 | 4.920 | Faster, but still unstable |
| `qwen2.5:3b-instruct-q4_K_M` | 1.9 GB | 10/11 | 2.672 | 2.368 | 5.659 | Good fallback candidate |
| `qwen3:4b-instruct-2507-q4_K_M` | 2.5 GB | 11/11 | 1.863 | 1.583 | 4.685 | Best overall classifier candidate |

## 5. Multi-Layer End-to-End Benchmark

These runs evaluate the full control stack, not just classifier output. Planner remained fixed to `qwen3:30b-a3b-instruct-2507-q4_K_M`.

| Classifier Model | Cases | Classifier Checks | Planner Checks | Validator Checks | E2E Checks | Avg Classifier (s) | Avg Planner (s) | Avg Total (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `llama3.1:8b-instruct-q5_K_M` | 11/11 | 11/11 | 15/15 | 11/11 | 10/10 | 6.481 | 6.121 | 12.602 |
| `llama3.2:3b-instruct-q4_K_M` | 9/11 | 9/11 | 13/15 | 11/11 | 10/10 | 4.491 | 7.800 | 12.291 |
| `qwen2.5:3b-instruct` | 10/11 | 10/11 | 14/15 | 11/11 | 10/10 | 2.733 | 4.802 | 7.535 |
| `qwen3:4b-instruct-2507-q4_K_M` | 11/11 | 11/11 | 15/15 | 11/11 | 10/10 | 1.627 | 4.717 | 6.344 |

Notes:

- The final `qwen3:4b` timing above was re-measured in a standalone run after model selection to reduce parallel-process interference.
- The `qwen3:4b` run is the current deployment candidate because it is the only tested classifier that simultaneously achieved:
  - `11/11` classifier correctness
  - `11/11` full-case pass rate
  - sub-`2 s` average classifier latency

## 6. Final Selection

Selected classifier:

- `qwen3:4b-instruct-2507-q4_K_M`

Selection rationale:

1. highest observed classifier accuracy among low-latency candidates
2. best end-to-end total latency among the accurate candidates
3. zero regressions on the current `smoke` suite
4. significantly lower stage-1 latency than the previous baseline `llama3.1:8b-instruct-q5_K_M`

Latency reduction versus the previous baseline:

- classifier latency: `6.481 s -> 1.627 s`
- absolute reduction: `4.854 s`
- relative reduction: `74.9%`

End-to-end LLM latency reduction versus the previous baseline:

- total LLM latency: `12.602 s -> 6.344 s`
- absolute reduction: `6.258 s`
- relative reduction: `49.7%`

## 7. Target Gap Analysis

The original target for stage-1 classification was `<= 1.5 s`.

Current selected model:

- `avg_classifier_sec = 1.627 s`
- `median_classifier_sec = 1.617 s`

Interpretation:

- The selected model is close to the target but does not strictly satisfy the mean-latency target yet.
- The remaining gap is small enough that the next optimization step should be architectural rather than purely model-size based.

Recommended next step:

- add a lightweight rule-based prefilter before the classifier for high-frequency, low-ambiguity utterances:
  - greeting
  - identity/name question
  - simple stop phrases
  - obvious general-knowledge chat

Expected benefit:

- reduce unnecessary classifier calls
- lower mean classifier latency below the `1.5 s` target
- preserve end-to-end correctness by keeping the current validator and planner stack unchanged

## 8. Artifacts

- evaluation suite: [cases_smoke.json](/home/shy/robot_project/phil_robot/eval/cases_smoke.json)
- latest smoke report for the selected classifier:
  [smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json](/home/shy/robot_project/phil_robot/eval/reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json)
- current classifier configuration:
  [intent_classifier.py](/home/shy/robot_project/phil_robot/pipeline/intent_classifier.py)
