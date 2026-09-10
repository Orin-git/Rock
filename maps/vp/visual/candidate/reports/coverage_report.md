# Visual Keyframe Coverage Report

- map: `vp`
- cell_size_m: **1.0**
- yaw_bins: **8**

## Totals

- Active frames: **37**
- Candidate frames: **11**
- Occupied spatial cells: **14**
- Active occupied cells: **9**
- Candidate-only new cells: **5**
- Active yaw coverage ratio (over Active cells): **0.292**
- Candidate new yaw bins (not in Active): **11**

## Capture / Dedup stats (this model session)

- captures: 0
- duplicate_skips: 0
- novelty_captures: 0
- quota_skips: 0
- covered_skips: 0

## Per-cell yaw masks

| cell | active | candidate | yaw_mask | active_bins | cand_bins |
|------|--------|-----------|----------|-------------|-----------|
| `cell_-10_0` | 7 | 0 | `00111100` | [2, 3, 4, 5] | [] |
| `cell_-9_3` | 6 | 0 | `11100000` | [0, 1, 2] | [] |
| `cell_-7_-2` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-6_-2` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-5_-2` | 3 | 0 | `00001000` | [4] | [] |
| `cell_-4_-2` | 3 | 0 | `00011000` | [3, 4] | [] |
| `cell_-4_-1` | 0 | 1 | `00001000` | [] | [4] |
| `cell_-3_-2` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-2_-2` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-1_-1` | 3 | 2 | `10101100` | [4, 5] | [0, 2] |
| `cell_-1_9` | 6 | 0 | `10000001` | [0, 7] | [] |
| `cell_0_-1` | 3 | 0 | `00011100` | [3, 4, 5] | [] |
| `cell_1_-1` | 3 | 0 | `01100000` | [1, 2] | [] |
| `cell_1_0` | 3 | 0 | `00110000` | [2, 3] | [] |

_Note: A3 reports where coverage exists; it does not require every free map cell to hold a keyframe._
