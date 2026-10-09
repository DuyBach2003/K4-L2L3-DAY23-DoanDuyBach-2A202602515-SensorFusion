"""Bonus: perturb the FRONT camera extrinsic and measure innovation and RMSE.

Runs the same per-frame loop as ``fusion-run-lab --fusion fused`` (one predict,
lidar pass, camera pass) with the student's workspace, but rotates the camera
pose about the vehicle z axis by a yaw error before tracking. Detections and the
seeded camera pixels are computed once and shared by every level, so a zero
offset reproduces ``tracking.fused`` of the graded run exactly.

Usage (from the repo root)::

    python student/bonus/calibration_sweep.py --config student/config/paths.yaml \
        --yaw-deg 0 0.1 0.25 0.5 1 2 --out student/bonus/calibration_sweep.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from scipy.stats import chi2

from fusion_lab import tracking_params
from fusion_lab.evaluation import _partial_assignment, valid_ground_truth
from fusion_lab.evaluation import TRACK_GATE_METERS
from fusion_lab.scripts.run_lab import (
    _front_observations, _lidar_observations, _load_paths_config,
    _resolve_weights, _setup_import_paths,
)
from fusion_lab.tracking.filter import Filter
from fusion_lab.tracking.manager import TrackManager
from fusion_lab.tracking.sensors import Sensor
from fusion_lab.workspace_loader import load_workspace


class _PixelRecorder:
    """Stand-in sensor that keeps the seeded camera pixels instead of building measurements."""

    def generate_measurement(self, frame, z, observations):
        observations.append(np.asarray(z, dtype=float))
        return observations


def _yaw_rotation(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.eye(4)
    rotation[:2, :2] = [[c, -s], [s, c]]
    return rotation


def cache_frames(cfg, ws, seed):
    """Run the detector once; keep detections, valid labels and noisy FRONT pixels."""
    from simple_waymo_open_dataset_reader import WaymoDataFileReader, dataset_pb2, label_pb2
    from simple_waymo_open_dataset_reader import utils as waymo_utils
    from fusion_lab.lidar_pcl import pcl_from_range_image

    det_pipe, bev = ws["detection_pipeline"], ws["bev_mapping"]
    weights = _resolve_weights(cfg)
    det_cfg = det_pipe.load_fpn_resnet_config(str(weights))
    model = det_pipe.create_fpn_model(det_cfg, str(weights))
    rng = np.random.default_rng(seed)
    frame_start, frame_end = int(cfg.get("frame_start", 0)), int(cfg.get("frame_end", 20))
    reader = WaymoDataFileReader(str(Path(cfg["waymo_dir"]) / cfg["segment"]))
    frames, calibrations = [], None
    for cnt, frame in enumerate(reader):
        if cnt < frame_start:
            continue
        if cnt > frame_end:
            break
        if calibrations is None:
            calibrations = (
                waymo_utils.get(frame.context.laser_calibrations, dataset_pb2.LaserName.TOP),
                waymo_utils.get(frame.context.camera_calibrations, dataset_pb2.CameraName.FRONT),
            )
        points = pcl_from_range_image(frame, dataset_pb2.LaserName.TOP)
        tensor = torch.from_numpy(bev.bev_maps_from_pcl(points, det_cfg)).unsqueeze(0).float()
        detections = det_pipe.detect_objects_from_bev(tensor, model, det_cfg)
        labels = valid_ground_truth(frame.laser_labels, det_cfg, label_pb2.Label.Type.TYPE_VEHICLE)
        pixels = _front_observations(frame, cnt, _PixelRecorder(), rng,
                                     dataset_pb2.CameraName.FRONT,
                                     label_pb2.Label.Type.TYPE_VEHICLE)
        frames.append({"cnt": cnt, "detections": detections, "labels": labels, "pixels": pixels})
    return frames, calibrations, det_cfg


def run_level(frames, calibrations, det_cfg, ws, yaw_deg, use_camera=True):
    """Track all cached frames with the camera pose rotated by ``yaw_deg``; optionally lidar only."""
    kalman, assoc, cam = ws["kalman"], ws["association"], ws["camera_fusion"]
    lidar_sensor = Sensor("lidar", calibrations[0], cam)
    camera_sensor = Sensor("camera", calibrations[1], cam)
    camera_sensor.sens_to_veh = np.asmatrix(_yaw_rotation(np.radians(yaw_deg))) @ camera_sensor.sens_to_veh
    camera_sensor.veh_to_sens = np.asmatrix(np.linalg.inv(camera_sensor.sens_to_veh))

    filter_obj = Filter(kalman)
    camera_innovations, lidar_residuals = [], []
    original_update = filter_obj.update

    def recording_update(track, meas):
        # Innovation of every accepted pair, evaluated before the correction.
        H = meas.sensor.get_H(track.x)
        gamma = np.asarray(kalman.innovation(track.x, meas)).ravel()
        S = np.asarray(kalman.innovation_covariance(track.P, meas, H))
        if meas.sensor.name == "camera":
            nis = float(gamma @ np.linalg.inv(S) @ gamma)
            depth = float(np.asarray(track.x).ravel()[0])
            camera_innovations.append((gamma[0], gamma[1], nis, S[0, 0], depth))
        elif track.state == "confirmed":
            # Lateral lidar residual: a camera bias shows up as a pull-back here.
            lidar_residuals.append((gamma[1], S[0, 0]))
        original_update(track, meas)

    filter_obj.update = recording_update
    manager = TrackManager(ws["track_management"])
    errors, ghosts, misses, offered_camera = [], 0, 0, 0
    for item in frames:
        cnt = item["cnt"]
        for track in manager.track_list:
            filter_obj.predict(track)
            track.set_t(cnt * tracking_params.dt)
        observations = _lidar_observations(cnt, item["detections"], lidar_sensor, det_cfg)
        assoc.associate_and_update(manager, observations, filter_obj, lidar_sensor)
        if use_camera and item["pixels"] is not None:
            camera_meas = []
            for pixel in item["pixels"]:
                camera_sensor.generate_measurement(cnt, pixel, camera_meas)
            offered_camera += len(camera_meas)
            assoc.associate_and_update(manager, camera_meas, filter_obj, camera_sensor)
        confirmed = [t for t in manager.track_list if t.state == "confirmed"]
        positions = np.array([np.asarray(t.x).ravel()[:3] for t in confirmed]).reshape(-1, 3)
        centres = np.array([(l.box.center_x, l.box.center_y, l.box.center_z)
                            for l in item["labels"]]).reshape(-1, 3)
        delta = positions[:, None, :] - centres[None, :, :]
        distances = np.linalg.norm(delta[:, :, :2], axis=2)
        pairs = _partial_assignment(distances, np.isfinite(distances) & (distances <= TRACK_GATE_METERS))
        errors.extend(delta[i, j] for i, j in pairs)
        ghosts += len(confirmed) - len(pairs)
        misses += len(item["labels"]) - len(pairs)

    errors = np.array(errors).reshape(-1, 3)
    gammas = np.array(camera_innovations).reshape(-1, 5)
    residuals = np.array(lidar_residuals).reshape(-1, 2)
    rmse_axes = np.sqrt(np.mean(errors**2, axis=0)) if len(errors) else np.full(3, np.nan)
    return {
        "yaw_deg": yaw_deg if use_camera else None,
        "rmse": float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))) if len(errors) else None,
        "rmse_x": float(rmse_axes[0]), "rmse_y": float(rmse_axes[1]), "rmse_z": float(rmse_axes[2]),
        "matches": int(len(errors)), "ghost_track_frames": int(ghosts), "missed_gt_frames": int(misses),
        "camera_measurements": int(offered_camera),
        "camera_updates": int(len(gammas)),
        "mean_gamma_u_px": float(gammas[:, 0].mean()) if len(gammas) else None,
        "mean_gamma_v_px": float(gammas[:, 1].mean()) if len(gammas) else None,
        "rms_gamma_u_px": float(np.sqrt(np.mean(gammas[:, 0] ** 2))) if len(gammas) else None,
        "median_gamma_u_px": float(np.median(gammas[:, 0])) if len(gammas) else None,
        "mean_nis": float(gammas[:, 2].mean()) if len(gammas) else None,
        "gate_nis": float(chi2.ppf(tracking_params.gating_threshold, 2)),
        "camera_s_uu_px2_p10_p50_p90": (np.percentile(gammas[:, 3], [10, 50, 90]).tolist()
                                        if len(gammas) else None),
        "camera_update_depth_m_min_median_max": (
            [float(gammas[:, 4].min()), float(np.median(gammas[:, 4])), float(gammas[:, 4].max())]
            if len(gammas) else None),
        "lidar_confirmed_updates": int(len(residuals)),
        "lidar_mean_gamma_y_m": float(residuals[:, 0].mean()) if len(residuals) else None,
        "lidar_median_s_xx_m2": float(np.median(residuals[:, 1])) if len(residuals) else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--yaw-deg", type=float, nargs="+", default=[0.0, 0.1, 0.25, 0.5, 1.0, 2.0])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    _setup_import_paths()
    cfg = _load_paths_config(args.config)
    ws = load_workspace()
    frames, calibrations, det_cfg = cache_frames(cfg, ws, args.seed)
    results = [run_level(frames, calibrations, det_cfg, ws, yaw) for yaw in args.yaw_deg]
    camera = Sensor("camera", calibrations[1], ws["camera_fusion"])
    report = {"segment": cfg["segment"], "frames": [frames[0]["cnt"], frames[-1]["cnt"]],
              "seed": args.seed,
              "front_camera": {"f_i": float(camera.f_i), "f_j": float(camera.f_j),
                               "c_i": float(camera.c_i), "c_j": float(camera.c_j),
                               "width": camera.image_width,
                               "fov_deg": [float(np.degrees(a)) for a in camera.fov],
                               "height_m": float(np.asarray(camera.sens_to_veh)[2, 3])},
              "gt_centre_z_median_m": float(np.median(
                  [l.box.center_z for item in frames for l in item["labels"]])),
              "lidar_only": run_level(frames, calibrations, det_cfg, ws, 0.0, use_camera=False),
              "levels": results}
    print(json.dumps(report, indent=2))
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
