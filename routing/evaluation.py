# routing/evaluation.py

import json
import os

def log_evaluation(
    ride_id,
    predicted_eta,
    actual_time,
    baseline_time,
    selected_route,
    baseline_route,
    best_possible_time=None,
    save_path="data/ride_logs.json"
):
    entry = {
        "ride_id": ride_id,
        "predicted_eta": predicted_eta,
        "actual_time": actual_time,
        "baseline_time": baseline_time,
        "eta_error": round(abs(predicted_eta - actual_time), 2),
        "delta_vs_baseline": round(baseline_time - actual_time, 2),
        "selected_route": selected_route,
        "baseline_route": baseline_route
    }

    if best_possible_time:
        entry["best_possible_time"] = best_possible_time
        entry["delta_vs_best"] = round(actual_time - best_possible_time, 2)

    # Load existing logs
    if os.path.exists(save_path):
        with open(save_path, "r") as f:
            logs = json.load(f)
    else:
        logs = []

    logs.append(entry)

    # Save updated log
    with open(save_path, "w") as f:
        json.dump(logs, f, indent=2)

    print(f"✅ Evaluation for ride {ride_id} logged.")

def print_summary(entry):
    print(f"Ride {entry['ride_id']}:")
    print(f"  ETA predicted: {entry['predicted_eta']}s")
    print(f"  Actual time:   {entry['actual_time']}s")
    print(f"  Baseline time: {entry['baseline_time']}s")
    print(f"  ETA error:     {entry['eta_error']}s")
    print(f"  Time saved vs baseline: {entry['delta_vs_baseline']}s")
    if "delta_vs_best" in entry:
        print(f"  Off from best possible: {entry['delta_vs_best']}s")
