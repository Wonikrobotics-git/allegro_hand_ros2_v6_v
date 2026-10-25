#!/usr/bin/env python3
"""
Kit Context Flatten Allegro Hand V6 USD Asset (v6_f / v6_v, Right / Left).
Flattening inlines referenced mesh layers and prim instances into a single composed USD asset file.
"""
import os
import argparse
from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Kit Context Flatten Allegro Hand V6 USD Asset")
parser.add_argument("--model", type=str, default="both", choices=["v6_f", "v6_v", "both"], help="Specify model variant: v6_f, v6_v, or both")
parser.add_argument("--hand", type=str, default="both", choices=["right", "left", "both"], help="Specify hand variant to flatten: right, left, or both")
args, unknown = parser.parse_known_args()

print("Starting SimulationApp for USD Flattening...")
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd

def flatten_single_hand(model_variant: str, hand_side: str, usd_dir: str):
    usd_subfolder = f"allegro_hand_{model_variant}_{hand_side}"
    hand_folder = os.path.join(usd_dir, usd_subfolder)
    input_usda = os.path.join(hand_folder, f"{usd_subfolder}.usda")
    output_usd = os.path.join(hand_folder, f"{usd_subfolder}_composed.usd")

    if not os.path.exists(input_usda):
        # Fallback to legacy folder if v6_f
        if model_variant == "v6_f":
            hand_folder = os.path.join(usd_dir, f"allegro_hand_v6_{hand_side}")
            input_usda = os.path.join(hand_folder, f"allegro_hand_v6_{hand_side}.usda")
            output_usd = os.path.join(hand_folder, f"allegro_hand_v6_{hand_side}_composed.usd")

    if not os.path.exists(input_usda):
        print(f"[USD Flattener] Skipping {model_variant.upper()} {hand_side.upper()} hand: USDA input file not found at {input_usda}")
        return False

    print(f"[USD Flattener] Opening stage in Kit context: {input_usda}")
    omni.usd.get_context().open_stage(input_usda)

    for _ in range(20):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    if not stage:
        print(f"[USD Flattener] Error: Failed to get stage for {input_usda}")
        return False

    uninstance_count = 0
    for prim in stage.Traverse():
        if prim.IsInstance():
            prim.SetInstanceable(False)
            uninstance_count += 1
    print(f"[USD Flattener] Un-instanced {uninstance_count} prims for {model_variant.upper()} {hand_side.upper()} hand.")

    for _ in range(10):
        simulation_app.update()

    print(f"[USD Flattener] Flattening fully composed USD stage for {model_variant.upper()} {hand_side.upper()} hand...")
    flattened_layer = stage.Flatten()

    print(f"[USD Flattener] Exporting flattened layer to: {output_usd}")
    flattened_layer.Export(output_usd)

    if os.path.exists(output_usd):
        print(f"[USD Flattener] SUCCESS: {output_usd} exported successfully!")
        return True
    else:
        print(f"[USD Flattener] FAILED: {output_usd} was not created!")
        return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    usd_dir = os.path.abspath(os.path.join(script_dir, "../usd"))

    models = ["v6_f", "v6_v"] if args.model == "both" else [args.model]
    hands = ["right", "left"] if args.hand == "both" else [args.hand]
    
    total_target = len(models) * len(hands)
    success_count = 0
    for m in models:
        for h in hands:
            if flatten_single_hand(m, h, usd_dir):
                success_count += 1

    print(f"[USD Flattener] Completed flattening of {success_count}/{total_target} hand variant(s).")
    simulation_app.close()

if __name__ == "__main__":
    main()
