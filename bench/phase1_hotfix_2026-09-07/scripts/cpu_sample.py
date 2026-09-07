#!/usr/bin/env python3
import sys, time
out = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 65.0
samples = []
t0 = time.time()
while time.time() - t0 < secs:
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("cpu "):
                vals = list(map(int, line.split()[1:]))
                idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
                total = sum(vals)
                samples.append((idle, total))
                break
    time.sleep(0.5)
all_u = []
for i in range(1, len(samples)):
    ia, ta = samples[i - 1]
    ib, tb = samples[i]
    all_u.append(100.0 * (1.0 - (ib - ia) / max(1, (tb - ta))))

def pct(xs, p):
    xs = sorted(xs)
    k = (len(xs) - 1) * p / 100.0
    f = int(k)
    c = min(f + 1, len(xs) - 1)
    return xs[f] * (c - k) + xs[c] * (k - f) if f != c else xs[f]

la = open("/proc/loadavg").read().strip()
try:
    npu = open("/sys/class/devfreq/fdab0000.npu/load").read().strip()
except Exception:
    npu = "NA"
text = (
    f"all: avg={sum(all_u)/len(all_u):.1f} p50={pct(all_u,50):.1f} "
    f"p95={pct(all_u,95):.1f} max={max(all_u):.1f}\n"
    f"loadavg: {la}\nnpu={npu}\n"
)
open(out, "w").write(text)
print(text)
