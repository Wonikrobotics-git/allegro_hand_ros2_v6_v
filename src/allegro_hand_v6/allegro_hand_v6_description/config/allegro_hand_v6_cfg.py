# Copyright (c) 2026, VoidRobot wr_humanoid. All rights reserved.
"""
Isaac Lab Articulation Configuration for Allegro Hand V6 (v6_f F / v6_v V, Right / Left).
Supports 5-finger 20-DOF architecture (Thumb, Index, Middle, Ring, Pinky).
"""
import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def _make_v6_cfg(model_variant: str, hand_side: str) -> ArticulationCfg:
    # Prefer specific model folder, fallback to legacy folder if not yet generated
    usd_path = os.path.join(_PKG_DIR, "usd", f"allegro_hand_v6_{model_variant}_{hand_side}", f"allegro_hand_v6_{model_variant}_{hand_side}_composed.usd")
    if not os.path.exists(usd_path):
        legacy_path = os.path.join(_PKG_DIR, "usd", f"allegro_hand_v6_{hand_side}", f"allegro_hand_v6_{hand_side}_composed.usd")
        if os.path.exists(legacy_path):
            usd_path = legacy_path

    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=10.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.3),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={
                "joint00": 0.0, "joint01": 0.0, "joint02": 0.0, "joint03": 0.0,
                "joint10": 0.0, "joint11": 0.0, "joint12": 0.0, "joint13": 0.0,
                "joint20": 0.0, "joint21": 0.0, "joint22": 0.0, "joint23": 0.0,
                "joint30": 0.0, "joint31": 0.0, "joint32": 0.0, "joint33": 0.0,
                "joint40": 0.0, "joint41": 0.0, "joint42": 0.0, "joint43": 0.0,
            },
        ),
        actuators={
            "hand_actuators": ImplicitActuatorCfg(
                joint_names_expr=["joint.*", "ah_joint.*"],
                stiffness=3.0,
                damping=0.1,
            ),
        },
    )

# F model (v6_f) configs
ALLEGRO_HAND_V6_F_RIGHT_CFG = _make_v6_cfg("v6_f", "right")
ALLEGRO_HAND_V6_F_LEFT_CFG = _make_v6_cfg("v6_f", "left")

# V model (v6_v) configs
ALLEGRO_HAND_V6_V_RIGHT_CFG = _make_v6_cfg("v6_v", "right")
ALLEGRO_HAND_V6_V_LEFT_CFG = _make_v6_cfg("v6_v", "left")

# Backward-compatible aliases (defaults to v6_f)
ALLEGRO_HAND_V6_RIGHT_CFG = ALLEGRO_HAND_V6_F_RIGHT_CFG
ALLEGRO_HAND_V6_LEFT_CFG = ALLEGRO_HAND_V6_F_LEFT_CFG
