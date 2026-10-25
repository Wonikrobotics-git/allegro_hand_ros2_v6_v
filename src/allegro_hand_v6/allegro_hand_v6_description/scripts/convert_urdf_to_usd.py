#!/usr/bin/env python3
"""
Convert Allegro Hand V6 URDF (v6_f / v6_v, Right / Left) to Omniverse USD asset using Isaac Lab's UrdfConverter.
"""
import os
import sys
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Convert Allegro Hand V6 URDF to Omniverse USD asset")
parser.add_argument("--model", type=str, default="both", choices=["v6_f", "v6_v", "both"], help="Specify model variant: v6_f, v6_v, or both")
parser.add_argument("--hand", type=str, default="both", choices=["right", "left", "both"], help="Specify hand variant to convert: right, left, or both")
parser.add_argument("--output-dir", type=str, default="", help="Output directory for USD asset files")
AppLauncher.add_app_launcher_args(parser)
args, unknown = parser.parse_known_args()

# Start Isaac Lab AppLauncher in headless mode
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

def convert_single_hand(model_variant: str, hand_side: str, pkg_dir: str, base_usd_dir: str):
    model_suffix = model_variant.replace("v6_", "") if model_variant.startswith("v6_") else model_variant
    urdf_file = os.path.join(pkg_dir, "urdf", f"allegro_hand_v6_{model_suffix}_{hand_side}.urdf")
    print(f"[USD Converter] Processing {model_variant.upper()} {hand_side.upper()} hand URDF: {urdf_file}")
    if not os.path.exists(urdf_file):
        print(f"[USD Converter] ERROR: URDF file not found at {urdf_file}")
        return False

    usd_subfolder = f"allegro_hand_{model_variant}_{hand_side}"
    target_dir = os.path.join(base_usd_dir, usd_subfolder)
    if os.path.exists(target_dir):
        import shutil
        shutil.rmtree(target_dir, ignore_errors=True)

    usd_file_name = f"{usd_subfolder}/{usd_subfolder}.usda"

    print(f"[USD Converter] Converting {urdf_file} to {base_usd_dir}/{usd_file_name}...")
    cfg = UrdfConverterCfg(
        asset_path=urdf_file,
        usd_dir=base_usd_dir,
        usd_file_name=usd_file_name,
        force_usd_conversion=True,
        make_instanceable=True,
        fix_base=True,
        merge_fixed_joints=False,
        self_collision=False,
    )

    converter = UrdfConverter(cfg)
    usd_path = converter.usd_path

    if os.path.exists(usd_path):
        print(f"[USD Converter] SUCCESS: {model_variant.upper()} {hand_side.upper()} hand USD saved to {usd_path}")
        # Backward compatibility alias for v6_f (legacy allegro_hand_v6_{hand_side})
        if model_variant == "v6_f":
            legacy_dir = os.path.join(base_usd_dir, f"allegro_hand_v6_{hand_side}")
            if os.path.exists(legacy_dir) and not os.path.islink(legacy_dir):
                import shutil
                shutil.rmtree(legacy_dir, ignore_errors=True)
            if not os.path.exists(legacy_dir):
                try:
                    os.symlink(usd_subfolder, legacy_dir)
                except OSError:
                    pass
        return True
    else:
        print(f"[USD Converter] FAILED: {model_variant.upper()} {hand_side.upper()} hand USD was not generated!")
        return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    base_usd_dir = args.output_dir if args.output_dir else os.path.join(pkg_dir, "usd")
    os.makedirs(base_usd_dir, exist_ok=True)

    models = ["v6_f", "v6_v"] if args.model == "both" else [args.model]
    hands = ["right", "left"] if args.hand == "both" else [args.hand]
    
    total_target = len(models) * len(hands)
    success_count = 0
    for m in models:
        for h in hands:
            if convert_single_hand(m, h, pkg_dir, base_usd_dir):
                success_count += 1

    print(f"[USD Converter] Completed conversion of {success_count}/{total_target} hand variant(s).")
    simulation_app.close()

if __name__ == "__main__":
    main()
