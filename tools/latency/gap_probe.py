r"""Classify input-report gaps as dropped (radio) or delayed (buffered), using the DualSense's own data.

Each report carries a counter (byte 12) and the controller's sensor timestamp (bytes 28-31, 1/3 us units).
- Dropped: the counter jumps and controller time jumps as much as arrival time did.
- Buffered: the counter has no holes, the reports arrive late, then a burst catches up.
Extra latency = arrival time minus controller time, relative to the fastest reports (clock drift removed).

Usage: python gap_probe.py [seconds]   (needs: pip install hidapi)
"""
import statistics, sys, time
import hid

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 40

d = next((x for x in hid.enumerate(0x054C, 0x0CE6) if x['usage_page'] == 1 and x['usage'] == 5), None)
if d is None:
    sys.exit("No DualSense found (USB 054C:0CE6, via the Pico or paired over Bluetooth).")
print(f"Testing: {d['product_string']!r}")
h = hid.device(); h.open_path(d['path']); h.set_nonblocking(False)
# Over native Bluetooth, reading feature report 0x05 switches the controller to full 0x31 reports.
h.get_feature_report(0x05, 64)
while not h.read(78, 1000):
    pass
# Drain whatever Windows queued before we started.
h.set_nonblocking(True)
while h.read(64):
    pass
h.set_nonblocking(False)

rows = []  # (arrival s, counter, controller s)
wrap_ts = last_raw = 0
end = time.perf_counter() + SECONDS
while time.perf_counter() < end:
    r = h.read(78, 100)
    # USB (and the Pico): report 0x01. Native Bluetooth: report 0x31, same layout shifted by one byte.
    o = 0 if r and r[0] == 0x01 else 1 if r and r[0] == 0x31 else None
    if o is not None and len(r) >= 32 + o:
        now = time.perf_counter()
        raw = int.from_bytes(bytes(r[28 + o:32 + o]), 'little')
        if raw < last_raw:
            wrap_ts += 1 << 32
        last_raw = raw
        rows.append((now, r[12 + o], (wrap_ts + raw) / 3e6))
h.close()
if len(rows) < 2 or rows[-1][0] - rows[0][0] < 2:
    sys.exit("Too few reports to measure (need at least 2 seconds of data).")

# Extra latency: offset arrival-controller, minus a line through per-second minimum offsets.
off = [a - c for a, _, c in rows]
t0 = rows[0][0]
mins = {}
for (a, _, _), o in zip(rows, off):
    k = int(a - t0)
    mins[k] = min(mins.get(k, o), o)
xs = [k + 0.5 for k in mins]; ys = list(mins.values())
mx, my = statistics.mean(xs), statistics.mean(ys)
slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
extra = [(o - (my + slope * (a - t0 - mx))) * 1000 for (a, _, _), o in zip(rows, off)]
base = sorted(extra)[len(extra) // 100]  # 1st percentile as the floor
extra = [e - base for e in extra]

ctrl_gaps = [(c2 - c1) * 1000 for (_, _, c1), (_, _, c2) in zip(rows, rows[1:])]
lost = sum((n2 - n1 - 1) % 256 for (_, n1, _), (_, n2, _) in zip(rows, rows[1:]))
print(f"reports {len(rows)} ({len(rows)/SECONDS:.0f}/s), counter holes (reports never received): {lost}")
print(f"controller-side intervals ms: p50 {statistics.median(ctrl_gaps):.2f}  max {max(ctrl_gaps):.2f}  "
      f">10ms: {sum(g > 10 for g in ctrl_gaps)}")
se = sorted(extra)
print(f"extra latency ms: p50 {se[len(se)//2]:.2f}  p99 {se[int(len(se)*.99)]:.2f}  "
      f"p99.9 {se[int(len(se)*.999)]:.2f}  max {se[-1]:.2f}   (clock drift {slope*1e6:+.0f} ppm)")

print("\nSpikes (arrival gap > 10 ms):")
print("   at s  arrive  ctrl  missed  extra-lat  next 5 arrival gaps ms")
for i in range(1, len(rows)):
    ag = (rows[i][0] - rows[i-1][0]) * 1000
    if ag > 10:
        missed = (rows[i][1] - rows[i-1][1] - 1) % 256
        nxt = [round((rows[j][0] - rows[j-1][0]) * 1000, 1) for j in range(i + 1, min(i + 6, len(rows)))]
        print(f"  {rows[i][0]-t0:5.2f}  {ag:5.1f}  {ctrl_gaps[i-1]:5.1f}  {missed:5d}  {extra[i]:8.1f}   {nxt}")
