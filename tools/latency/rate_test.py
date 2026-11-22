r"""Measure the DS5Dongle's input-report rate and gaps.

Usage: python rate_test.py [seconds]   (needs: pip install hidapi)
Leave the controller idle on a table while it runs.
"""
import collections, statistics, sys, time
import hid

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20

devs = hid.enumerate(0x054C, 0x0CE6)
if not devs:
    sys.exit("Pico not found (no USB device 054C:0CE6). Replug it and try again.")
print("Matching HID interfaces:")
for x in devs:
    print(f"  usage_page={x['usage_page']:#06x} usage={x['usage']:#04x} iface={x['interface_number']} {x['product_string']!r}")
pads = [x for x in devs if x['usage_page'] == 1 and x['usage'] == 5]
if not pads:
    sys.exit("No gamepad interface (usage page 1, usage 5) found.")

h = hid.device(); h.open_path(pads[0]['path']); h.set_nonblocking(False)

# Wait for the controller to start sending, so a sleeping controller doesn't count as gaps.
print("Waiting for reports (press PS on the controller if it's off)...")
while not h.read(64, 1000):
    pass

ts, ids = [], collections.Counter()
end = time.perf_counter() + SECONDS
while time.perf_counter() < end:
    r = h.read(64, 100)
    if r:
        ids[r[0]] += 1
        if r[0] == 1:
            ts.append(time.perf_counter())
h.close()

print(f"report IDs seen: {dict(ids)}")
if len(ts) < 2:
    sys.exit("Too few type-1 reports to measure. Check the report IDs above.")
gaps = sorted((b - a) * 1000 for a, b in zip(ts, ts[1:]))
print(f"reports: {len(ts)} in {SECONDS:g}s -> {len(ts)/SECONDS:.0f} Hz")
print(f"interval ms: avg {statistics.mean(gaps):.2f}  p50 {gaps[len(gaps)//2]:.2f}  "
      f"p99 {gaps[int(len(gaps)*.99)]:.2f}  max {gaps[-1]:.2f}")
big = [round(g, 1) for g in gaps if g > 4]
print(f"gaps >4ms: {len(big)}  {big}")
