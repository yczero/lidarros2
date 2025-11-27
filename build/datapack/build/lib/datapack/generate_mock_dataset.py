import json
import math
import os
import random

ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
NUM_POINTS = 360
RANGE_MIN = 0.12
RANGE_MAX = 3.5

AVAILABLE_PATTERNS = ["front_wall", "left_wall", "right_wall"]

def create_empty_scan():
    ranges = [RANGE_MAX for _ in range(NUM_POINTS)]
    intensities = [100.0 for _ in range(NUM_POINTS)]  # int → float
    scan = {
        "angle_min": math.radians(ANGLE_MIN_DEG),
        "angle_max": math.radians(ANGLE_MAX_DEG),
        "angle_increment": math.radians(1),
        "range_min": RANGE_MIN,
        "range_max": RANGE_MAX,
        "ranges": ranges,
        "intensities": intensities
    }
    return scan

def make_wall(ranges, center_deg, width_deg):
    half_width = width_deg // 2
    for offset in range(-half_width, half_width + 1):
        idx = (center_deg + offset) % NUM_POINTS
        ranges[idx] = 0.4

def apply_pattern(scan, pattern):
    if pattern == "front_wall":
        make_wall(scan["ranges"], 0, 40)
    elif pattern == "left_wall":
        make_wall(scan["ranges"], 90, 30)
    elif pattern == "right_wall":
        make_wall(scan["ranges"], 270, 30)

def generate_dataset(num_scans=10, out_dir="lds02_dataset"):
    os.makedirs(out_dir, exist_ok=True)
    for i in range(num_scans):
        pattern = random.choice(AVAILABLE_PATTERNS)
        scan = create_empty_scan()
        apply_pattern(scan, pattern)
        scan["meta"] = {"pattern": pattern, "index": i}
        filename = os.path.join(out_dir, f"lds02_mock_{i:03d}.json")
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(scan, f, ensure_ascii=False, indent=2)
        print(f"Saved {filename} (pattern={pattern})")

if __name__ == "__main__":
    generate_dataset(num_scans=10)