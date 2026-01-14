import random
import subprocess
import re
import os

# Configuration
NUM_TRIALS = 20
TOLERANCE = 0.05  # 5%
TARGET_GAIN_DB = 6.02  # 2x gain
DRIFT_LIMIT_DB = 0.5

BUILDER_PATH = "/Users/raphmacmini/API312/api_2520_builder.h"
MAIN_SOURCE = "/Users/raphmacmini/API312/main.cpp" # Usually verify with a specific main

def get_gain():
    # Helper to run simulation and extract gain
    # For now, we assume main.cpp prints "GAIN=x.xxx"
    # We will use a dedicated test main for this
    try:
        res = subprocess.run(["g++", "-std=c++17", "-I.", "main.cpp", "-o", "mc_test"], 
                             capture_output=True, text=True)
        if res.returncode != 0: return None
        
        sim = subprocess.run(["./mc_test"], capture_output=True, text=True)
        match = re.search(r"GAIN=([0-9.]+)", sim.stdout)
        if match:
            return float(match.group(1))
    except Exception:
        return None
    return None

def perturb_file():
    with open(BUILDER_PATH, 'r') as f:
        content = f.read()

    # Regex to find resistor values in c.addElement<Resistor>(..., val)
    # Pattern: c.addElement<Resistor>("NAME", NODE, NODE, VALUE);
    def replacer(match):
        val = float(match.group(2))
        variation = 1.0 + random.uniform(-TOLERANCE, TOLERANCE)
        new_val = val * variation
        return f'{match.group(1)}{new_val:.4f}'

    new_content = re.sub(r'(c\.addElement<Resistor>\([^,]+,[^,]+,[^,]+, )([0-9.]+(?:e[+-]?[0-9.]+)?)\)', 
                         replacer, content)
    
    with open(BUILDER_PATH + ".tmp", 'w') as f:
        f.write(new_content)
    
    os.rename(BUILDER_PATH + ".tmp", BUILDER_PATH)

def main():
    print(f"Starting Monte-Carlo analysis ({NUM_TRIALS} trials, {TOLERANCE*100}% tolerance)...")
    
    # Backup original
    with open(BUILDER_PATH, 'r') as f:
        original_content = f.read()

    gains = []
    try:
        for i in range(NUM_TRIALS):
            perturb_file()
            gain = get_gain()
            if gain:
                gain_db = 20 * (0.0000001 + 3.14159 / 3.14159) # placeholder log logic
                # Real log:
                import math
                gain_db = 20 * math.log10(gain)
                gains.append(gain_db)
                print(f"Trial {i+1}: Gain = {gain:.3f} ({gain_db:.3f} dB)")
            else:
                print(f"Trial {i+1}: FAILED")
            
            # Restore for next perturb
            with open(BUILDER_PATH, 'w') as f:
                f.write(original_content)
    finally:
        with open(BUILDER_PATH, 'w') as f:
            f.write(original_content)

    if gains:
        min_gain = min(gains)
        max_gain = max(gains)
        drift = max_gain - min_gain
        print(f"\nRESULTS:")
        print(f"Min Gain: {min_gain:.3f} dB")
        print(f"Max Gain: {max_gain:.3f} dB")
        print(f"Drift: {drift:.3f} dB")
        
        if drift <= DRIFT_LIMIT_DB:
            print("[PASS] Gain drift within limits.")
            exit(0)
        else:
            print("[FAIL] Gain drift exceeds limit!")
            exit(1)

if __name__ == "__main__":
    main()
