# Visual Keyframe Coverage Report

- map: `vp`
- cell_size_m: **1.0**
- yaw_bins: **8**

## Totals

- Active frames: **55**
- Candidate frames: **42**
- Occupied spatial cells: **32**
- Active occupied cells: **18**
- Candidate-only new cells: **14**
- Active yaw coverage ratio (over Active cells): **0.257**
- Candidate new yaw bins (not in Active): **23**

## Capture / Dedup stats (this model session)

- captures: 0
- duplicate_skips: 0
- novelty_captures: 0
- quota_skips: 0
- covered_skips: 0

## Per-cell yaw masks

| cell | active | candidate | yaw_mask | active_bins | cand_bins |
|------|--------|-----------|----------|-------------|-----------|
| `cell_-11_-1` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-10_-2` | 1 | 1 | `00100000` | [2] | [2] |
| `cell_-10_-1` | 1 | 1 | `00100000` | [2] | [2] |
| `cell_-10_0` | 7 | 0 | `00111100` | [2, 3, 4, 5] | [] |
| `cell_-10_3` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-9_-2` | 1 | 1 | `10000000` | [0] | [0] |
| `cell_-9_-1` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-9_3` | 6 | 0 | `11100000` | [0, 1, 2] | [] |
| `cell_-8_-2` | 3 | 3 | `10100000` | [0, 2] | [0, 2] |
| `cell_-8_-1` | 0 | 1 | `10000000` | [] | [0] |
| `cell_-8_3` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-8_4` | 0 | 1 | `10000000` | [] | [0] |
| `cell_-7_-2` | 2 | 2 | `10100000` | [0, 2] | [0, 2] |
| `cell_-6_-2` | 2 | 2 | `10100000` | [0, 2] | [0, 2] |
| `cell_-5_-2` | 3 | 0 | `00001000` | [4] | [] |
| `cell_-4_-2` | 3 | 0 | `00011000` | [3, 4] | [] |
| `cell_-4_-1` | 1 | 1 | `00001000` | [4] | [4] |
| `cell_-3_-2` | 2 | 2 | `10100000` | [0, 2] | [0, 2] |
| `cell_-3_-1` | 0 | 1 | `00100000` | [] | [2] |
| `cell_-2_-2` | 3 | 3 | `10100000` | [0, 2] | [0, 2] |
| `cell_-2_-1` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_-2_0` | 0 | 1 | `10000000` | [] | [0] |
| `cell_-1_-1` | 5 | 3 | `10101100` | [0, 2, 4, 5] | [0, 2] |
| `cell_-1_0` | 0 | 1 | `10000000` | [] | [0] |
| `cell_-1_9` | 6 | 0 | `10000001` | [0, 7] | [] |
| `cell_0_-1` | 3 | 0 | `00011100` | [3, 4, 5] | [] |
| `cell_0_0` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_0_1` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_1_-1` | 3 | 0 | `01100000` | [1, 2] | [] |
| `cell_1_0` | 3 | 0 | `00110000` | [2, 3] | [] |
| `cell_2_-1` | 0 | 2 | `10100000` | [] | [0, 2] |
| `cell_2_0` | 0 | 2 | `10100000` | [] | [0, 2] |

_Note: A3 reports where coverage exists; it does not require every free map cell to hold a keyframe._
