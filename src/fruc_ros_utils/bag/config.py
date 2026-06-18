"""YAML config loading and merging helpers for fruc_ros_utils."""

import logging
import os
import pathlib

import yaml

logger = logging.getLogger(__name__)


def make_cfg_tree(cfg: dict) -> dict:
    """Rebuild a nested config tree from a flattened cfg dict."""
    from fruc_ros_utils.vision.illumination import IlluminationConfig

    illum_defaults = IlluminationConfig().__dict__.copy()

    raw_defaults = {
        "raw_enable_clahe": False,
        "raw_p_low": 1.0,
        "raw_p_high": 99.0,
        "raw_gamma_strength": 0.7,
        "raw_target_mean_under": 135.0,
        "raw_target_mean_over": 120.0,
        "raw_gamma_min": 0.75,
        "raw_gamma_max": 1.35,
    }
    deblur_defaults = {
        "mf_min_sharpness": illum_defaults.get("mf_min_sharpness", 50.0),
        "deblur_mode": illum_defaults.get("deblur_mode", "off"),
        "deblur_algorithm": illum_defaults.get("deblur_algorithm", "lucy"),
    }
    egomotion_defaults = {
        "ego_theta_thresh_rad": illum_defaults.get("ego_theta_thresh_rad", 0.0025),
    }
    retinex_defaults = {
        "retinex_scales": [15, 80, 250],
        "lime_guided_radius": 15,
        "lime_guided_eps": 0.001,
    }
    runtime_defaults = {
        "preserve_bayer": False,
        "force": False,
    }

    def section(fill_from: dict, defaults: dict) -> dict:
        return {k: fill_from.get(k, dv) for k, dv in defaults.items()}

    return {
        "illumination": section(cfg, illum_defaults),
        "raw":          section(cfg, raw_defaults),
        "deblur":       section(cfg, deblur_defaults),
        "egomotion":    section(cfg, egomotion_defaults),
        "retinex_lime": section(cfg, retinex_defaults),
        "runtime":      section(cfg, runtime_defaults),
    }


def load_yaml(path: str) -> dict:
    if not path or not os.path.exists(path):
        return {}
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logger.error("Failed to load YAML %s: %s", path, e)
        return {}


def load_configs(user_cfg_path: str = None, dev_cfg_path: str = None) -> dict:
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    default_dev = repo_root / "config" / "dev_defaults.yaml"

    cfg = load_yaml(str(default_dev))
    cfg.update(load_yaml(dev_cfg_path))
    cfg.update(load_yaml(user_cfg_path))
    logger.debug("Loaded config=%s", cfg)
    return cfg


def merge_configs(cli_args, user_cfg: dict, dev_cfg: dict) -> dict:
    cfg = {}
    cfg.update(dev_cfg or {})
    cfg.update(user_cfg or {})

    # These sections are named after one specific CLI subcommand and carry
    # keys (e.g. colorize_labels.out_path) that collide with other
    # subcommands' flattened keys. Only flatten one when it's the active
    # subcommand, so e.g. colorize_labels's out_path default can't leak into
    # remove_topic/change_frame_id/etc. cmd is unset for non-CLI callers
    # (e.g. analyze_metrics' direct merge_configs call), which keeps the old
    # flatten-everything behavior for them.
    command_scoped_sections = {"colorize_labels", "mapir_ndvi", "extract_metadata"}
    cmd = getattr(cli_args, "cmd", None)

    for section_name in (
        "bagutils",
        "extrinsics",
        "navsat",
        "imu",
        "illumination",
        "mapir_ndvi",
        "colorize_labels",
        "extract_metadata",
    ):
        if section_name in command_scoped_sections and cmd is not None and section_name != cmd:
            cfg.pop(section_name, None)
            continue
        if section_name in cfg and isinstance(cfg[section_name], dict):
            cfg.update(cfg.pop(section_name))

    for k, v in vars(cli_args).items():
        if v is not None:
            cfg[k] = v

    logger.debug("Final merged config=%s", cfg)
    return cfg
