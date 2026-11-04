#!/usr/bin/env python3
"""把 Kalibr camchain.yaml 转成带模型名 + fisheye 外参的统一 JSON。"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
import yaml

from camera_model import KALIBR_CAMERA_MODEL, require_ds_camchain, require_model_list

_NUM_RE = re.compile(r"[-+]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:[eE][-+]?\d+)?")


def _parse_float_list(text: str) -> list[float]:
    return [float(x) for x in _NUM_RE.findall(text)]


def _camera_names_from_meta(meta: dict, count: int) -> list[str]:
    names = list(meta.get("camera_names") or [])
    if len(names) == count:
        return names
    mapping = meta.get("cam_topic_map") or {}
    inverse = {str(kalibr): f"fisheye{physical}" for physical, kalibr in mapping.items()}
    return [inverse.get(f"cam{i}", f"cam{i}") for i in range(count)]


def _save_named_camchain(camchain: Path, camera_names: list[str]) -> None:
    """将原生 camN 键替换为真实相机名；重复调用安全。"""
    raw = yaml.safe_load(camchain.read_text(encoding="utf-8"))
    old_keys = [f"cam{i}" for i in range(len(camera_names))]
    if not all(key in raw for key in old_keys):
        return
    named = {}
    for old_key, name in zip(old_keys, camera_names):
        camera = raw[old_key]
        camera["camera_name"] = name
        camera["rostopic"] = f"/{name}/image_raw"
        overlaps = camera.get("cam_overlaps", [])
        camera["camera_overlaps"] = [
            camera_names[int(i)] for i in overlaps if int(i) < len(camera_names)
        ]
        named[name] = camera
    camchain.write_text(
        yaml.safe_dump(named, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _invert_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _T_to_Rt(T: np.ndarray) -> dict:
    return {
        "R": T[:3, :3].tolist(),
        "t_m": T[:3, 3].reshape(3).tolist(),
        "T_4x4": T.tolist(),
        "description": "p_dst = R @ p_src + t",
    }


def _find_results_txt(export_dir: Path | None, camchain_path: Path) -> Path | None:
    cands: list[Path] = []
    if export_dir is not None:
        cands.extend(sorted(export_dir.glob("*results-cam*.txt")))
        cands.append(export_dir / "calib-results-cam.txt")
    cands.append(camchain_path.with_name(camchain_path.name.replace("-camchain.yaml", "-results-cam.txt")))
    cands.append(camchain_path.parent / "calib-results-cam.txt")
    seen = set()
    for p in cands:
        rp = p.resolve() if p.exists() else p
        key = str(rp)
        if key in seen:
            continue
        seen.add(key)
        if p.is_file():
            return p
    return None


def parse_kalibr_results_txt(path: Path) -> dict:
    """解析 calib-results-cam.txt：内参 std、重投影误差、链式外参 std。"""
    text = path.read_text(encoding="utf-8", errors="ignore")
    # 合并被换行打断的 projection 行
    text = re.sub(r"\]\s*\n\s*([-\d.eE+]+)", r"] \1", text)
    text = re.sub(
        r"(projection:\s*\[[^\]]*)\n\s*([^\]]+\])",
        lambda m: m.group(1) + " " + m.group(2),
        text,
    )

    cameras: dict[str, dict] = {}
    baselines: list[dict] = []

    cam_blocks = re.finditer(
        r"(cam\d+)\s*\(([^)]+)\):\s*\n"
        r"(.*?)"
        r"(?=cam\d+\s*\(|baseline T_|Target configuration|\Z)",
        text,
        flags=re.S,
    )
    for m in cam_blocks:
        cam_id = m.group(1)
        topic = m.group(2).strip()
        body = m.group(3)
        dist_m = re.search(r"distortion:\s*(\[[^\]]*\])\s*\+-\s*(\[[^\]]*\])", body)
        proj_m = re.search(
            r"projection:\s*(\[[^\]]*\])\s*\+-\s*(\[[^\]]*\])",
            body,
            flags=re.S,
        )
        # projection 可能跨行：用更宽松匹配
        if proj_m is None:
            proj_m = re.search(
                r"projection:\s*\[(.*?)\]\s*\+-\s*\[(.*?)\]",
                body,
                flags=re.S,
            )
            proj_vals = _parse_float_list(proj_m.group(1)) if proj_m else []
            proj_std = _parse_float_list(proj_m.group(2)) if proj_m else []
        else:
            proj_vals = _parse_float_list(proj_m.group(1))
            proj_std = _parse_float_list(proj_m.group(2))

        dist_vals = _parse_float_list(dist_m.group(1)) if dist_m else []
        dist_std = _parse_float_list(dist_m.group(2)) if dist_m else []

        reproj_m = re.search(
            r"reprojection error:\s*\[([^\]]*)\]\s*\+-\s*\[([^\]]*)\]",
            body,
        )
        reproj_mean = _parse_float_list(reproj_m.group(1)) if reproj_m else []
        reproj_std = _parse_float_list(reproj_m.group(2)) if reproj_m else []
        rms = None
        if len(reproj_std) >= 2:
            rms = float(np.sqrt(0.5 * (reproj_std[0] ** 2 + reproj_std[1] ** 2)))

        cameras[cam_id] = {
            "topic": topic,
            "intrinsics": proj_vals,
            "intrinsics_std": proj_std,
            "distortion_coeffs": dist_vals,
            "distortion_std": dist_std,
            "reprojection_error_px": {
                "mean_xy": reproj_mean,
                "std_xy": reproj_std,
                "rms_approx": rms,
                "description": "Kalibr residual: mean +- std of corner reprojection error (pixels)",
            },
        }

    for m in re.finditer(
        r"baseline T_(\d+)_(\d+):\s*\n"
        r"\s*q:\s*(\[[^\]]*\])\s*\+-\s*(\[[^\]]*\])\s*\n"
        r"\s*t:\s*(\[[^\]]*\])\s*\+-\s*(\[[^\]]*\])",
        text,
    ):
        dst_i = int(m.group(1))
        src_i = int(m.group(2))
        q = _parse_float_list(m.group(3))
        q_std = _parse_float_list(m.group(4))
        t = _parse_float_list(m.group(5))
        t_std = _parse_float_list(m.group(6))
        t_norm = float(np.linalg.norm(t)) if t else None
        t_std_norm = float(np.linalg.norm(t_std)) if t_std else None
        baselines.append(
            {
                "kalibr_name": f"T_{dst_i}_{src_i}",
                "from_cam_index": src_i,
                "to_cam_index": dst_i,
                "q": q,
                "q_std": q_std,
                "t_m": t,
                "t_std_m": t_std,
                "t_norm_m": t_norm,
                "t_std_norm_m": t_std_norm,
                "description": "Kalibr chain baseline uncertainty (quaternion xyz / translation meters)",
            }
        )

    return {"cameras": cameras, "baselines": baselines, "source": str(path)}


def _attach_errors(
    payload: dict,
    errors: dict,
    camera_names: list[str],
) -> None:
    """把 results-txt 误差挂到 cameras / camera_to_camera。"""
    cam_errs = errors.get("cameras") or {}
    baselines = errors.get("baselines") or []

    # 按 cam0/cam1... 顺序对齐真实名
    ordered_names = list(camera_names)
    if not ordered_names:
        ordered_names = list(payload.get("camera_names") or payload.get("cameras", {}).keys())

    for i, name in enumerate(ordered_names):
        src = cam_errs.get(f"cam{i}")
        if not src or name not in payload["cameras"]:
            continue
        cam = payload["cameras"][name]
        cam["intrinsics_std"] = src.get("intrinsics_std") or []
        cam["distortion_std"] = src.get("distortion_std") or []
        cam["reprojection_error_px"] = src.get("reprojection_error_px")
        # ds 参数展开 std
        if cam.get("kalibr_model") == KALIBR_CAMERA_MODEL and len(cam.get("intrinsics_std") or []) >= 6:
            s = cam["intrinsics_std"]
            cam.setdefault("ds_params", {})
            cam["ds_params_std"] = {
                "xi": s[0],
                "alpha": s[1],
                "fx": s[2],
                "fy": s[3],
                "cx": s[4],
                "cy": s[5],
            }

    chain_errs: dict[str, dict] = {}
    for b in baselines:
        src_i = int(b["from_cam_index"])
        dst_i = int(b["to_cam_index"])
        if src_i >= len(ordered_names) or dst_i >= len(ordered_names):
            continue
        src_name = ordered_names[src_i]
        dst_name = ordered_names[dst_i]
        key = f"{dst_name}_from_{src_name}"
        entry = {
            "q": b.get("q"),
            "q_std": b.get("q_std"),
            "t_m": b.get("t_m"),
            "t_std_m": b.get("t_std_m"),
            "t_norm_m": b.get("t_norm_m"),
            "t_std_norm_m": b.get("t_std_norm_m"),
            "source": "kalibr_chain_baseline",
            "description": (
                "相邻相机外参不确定度（Kalibr recoverCovariance）；"
                "q_std 为四元数虚部 xyz，t_std_m 单位米"
            ),
        }
        chain_errs[key] = entry
        # 挂到目标相机的 T_from_ref 旁（仅当目标相对参考且链相邻于参考时完整对应；
        # 对相邻对一律写入 transforms 的 uncertainty）
        if src_i == 0 and dst_name in payload["cameras"]:
            payload["cameras"][dst_name]["T_from_ref"]["uncertainty"] = {
                "q_std": b.get("q_std"),
                "t_std_m": b.get("t_std_m"),
                "t_std_norm_m": b.get("t_std_norm_m"),
                "note": "Kalibr baseline uncertainty of T_cn_cnm1 when src is reference",
            }

    c2c = payload.setdefault("camera_to_camera", {})
    transforms = c2c.setdefault("transforms", {})
    for key, err in chain_errs.items():
        if key in transforms:
            transforms[key]["uncertainty"] = err
        else:
            transforms[key] = {"uncertainty": err}
    c2c["chain_baseline_errors"] = list(chain_errs.values())

    # 汇总联合重投影
    reproj_summary = {}
    all_rms = []
    for name in ordered_names:
        cam = payload["cameras"].get(name) or {}
        r = cam.get("reprojection_error_px") or {}
        if r.get("rms_approx") is not None:
            reproj_summary[name] = r
            all_rms.append(float(r["rms_approx"]))
    payload["errors"] = {
        "source_results_txt": errors.get("source"),
        "reprojection_error_px": {
            "per_camera": reproj_summary,
            "mean_rms_approx": float(np.mean(all_rms)) if all_rms else None,
            "description": "联合标定后各相机角点重投影误差（像素）",
        },
        "extrinsics": {
            "chain_baselines": chain_errs,
            "description": "相邻相机外参协方差（Kalibr）；非相邻外参为链式复合，此处仅给相邻对 std",
        },
    }
    payload.setdefault("notes", [])
    payload["notes"].extend(
        [
            "intrinsics_std / distortion_std：Kalibr 参数标准差（与 intrinsics 同维）",
            "reprojection_error_px：联合优化后角点重投影 mean/std（像素）",
            "camera_to_camera.chain_baseline_errors / transforms[*].uncertainty：相邻外参不确定度",
        ]
    )


def _intrinsics_payload(cam: dict) -> dict:
    intr = list(cam.get("intrinsics") or [])
    dist = list(cam.get("distortion_coeffs") or [])
    return {
        "kalibr_model": KALIBR_CAMERA_MODEL,
        "camera_model": cam.get("camera_model"),
        "distortion_model": cam.get("distortion_model"),
        "resolution_wh": list(cam.get("resolution") or []),
        "intrinsics": intr,
        "distortion_coeffs": dist,
        "rostopic": cam.get("rostopic"),
        "ds_params": {
            "xi": intr[0],
            "alpha": intr[1],
            "fx": intr[2],
            "fy": intr[3],
            "cx": intr[4],
            "cy": intr[5],
        },
    }


def convert(
    camchain_path: Path,
    *,
    cam_topic_map: dict[str, str],
    camera_names: list[str] | None = None,
    models: list[str] | None,
    session_dir: str | None,
    selected_frames: int | None,
    results_txt: Path | None = None,
) -> dict:
    import re

    raw = require_ds_camchain(camchain_path)
    # cam_topic_map: {"0":"fisheye0","3":"fisheye3"} 或旧 {"0":"cam0","3":"cam1"}
    kalibr_to_fish = {v: f"fisheye{k}" for k, v in cam_topic_map.items()}
    for k, v in list(cam_topic_map.items()):
        if str(v).startswith("fisheye"):
            kalibr_to_fish[str(v)] = str(v)
        # 物理编号本身也可能是 key
        kalibr_to_fish[f"fisheye{k}"] = f"fisheye{k}"

    camera_names = list(camera_names or [])
    if camera_names and all(name in raw for name in camera_names):
        cam_keys = camera_names
        kalibr_to_fish.update({name: name for name in camera_names})
    else:
        # 优先真实 fisheyeN 键，否则 camN
        fish_keys = [k for k in raw.keys() if str(k).startswith("fisheye")]
        cam_keys_raw = [k for k in raw.keys() if str(k).startswith("cam")]
        if fish_keys:
            cam_keys = sorted(
                fish_keys,
                key=lambda n: int(re.search(r"\d+", str(n)).group()) if re.search(r"\d+", str(n)) else 0,
            )
            kalibr_to_fish.update({k: k for k in cam_keys})
        else:
            cam_keys = sorted(
                cam_keys_raw,
                key=lambda n: int(re.search(r"\d+", str(n)).group()) if re.search(r"\d+", str(n)) else 0,
            )
    if not cam_keys:
        raise RuntimeError(f"camchain 无相机: {camchain_path}")
    if models is not None:
        require_model_list(models, len(cam_keys))

    # 链式外参：第一路为参考，cam_i 相对 cam_{i-1} 的 T_cn_cnm1
    T_from_ref: dict[str, np.ndarray] = {}
    T_acc = np.eye(4)
    for i, ck in enumerate(cam_keys):
        cam = raw[ck]
        if i == 0:
            T_from_ref[ck] = np.eye(4)
            continue
        T_rel = np.asarray(cam["T_cn_cnm1"], dtype=np.float64)
        T_acc = T_rel @ T_acc
        T_from_ref[ck] = T_acc.copy()

    cameras: dict = {}
    for i, ck in enumerate(cam_keys):
        cam = raw[ck]
        fish = kalibr_to_fish.get(ck, ck)
        entry = _intrinsics_payload(cam)
        entry["kalibr_name"] = ck
        entry["camera_name"] = fish
        entry["T_from_ref"] = _T_to_Rt(T_from_ref[ck])
        if i == 0:
            entry["T_from_ref"]["description"] = "reference camera; identity"
        else:
            entry["T_from_ref"]["description"] = f"p_{fish} = R @ p_ref + t"
        cameras[fish] = entry

    # 两两外参表（任意数量）
    names = list(cameras.keys())
    pairwise: dict = {}
    T_map = {n: np.asarray(cameras[n]["T_from_ref"]["T_4x4"], dtype=np.float64) for n in names}
    for dst in names:
        for src in names:
            key = f"{dst}_from_{src}"
            if dst == src:
                pairwise[key] = _T_to_Rt(np.eye(4))
                continue
            T_ds = T_map[dst] @ _invert_T(T_map[src])
            pairwise[key] = _T_to_Rt(T_ds)

    ref_fish = kalibr_to_fish.get(cam_keys[0], cam_keys[0])
    payload = {
        "description": "Kalibr multi-camera intrinsics + extrinsics",
        "calib_backend": "ros1_tartancalib",
        "session_dir": session_dir,
        "source_camchain": str(camchain_path),
        "reference_camera": ref_fish,
        "num_cameras": len(names),
        "camera_names": names,
        "kalibr_models": {n: cameras[n]["kalibr_model"] for n in names},
        "coordinate_convention": "p_dst = R @ p_src + t   (T_dst_from_src), units meters",
        "selected_frames": selected_frames,
        "cameras": cameras,
        "camera_to_camera": {
            "reference": ref_fish,
            "transforms": pairwise,
        },
        "notes": [
            "kalibr_model 固定为 ds-none；转换前已校验六维 DS 内参",
            "支持可变数量 fisheye；参考相机为链中第一路",
            "T_from_ref：相对参考相机的外参",
            "camera_to_camera.transforms['fisheyeA_from_fisheyeB'] 为任意两路相对位姿",
        ],
    }

    results_path = results_txt or _find_results_txt(camchain_path.parent, camchain_path)
    if results_path is not None and results_path.is_file():
        try:
            err = parse_kalibr_results_txt(results_path)
            _attach_errors(payload, err, names)
        except Exception as exc:  # noqa: BLE001
            payload.setdefault("notes", []).append(f"解析 results-txt 失败: {exc}")
    return payload


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-dir", type=Path, required=True, help="含 calib-camchain.yaml / kalibr_meta.json 的目录")
    ap.add_argument("--camchain", type=Path, default=None)
    ap.add_argument("--results-txt", type=Path, default=None, help="Kalibr calib-results-cam.txt（默认自动查找）")
    ap.add_argument("--out-json", type=Path, default=None)
    args = ap.parse_args()

    export_dir = args.export_dir.resolve()
    meta_path = export_dir / "kalibr_meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
    camchain = args.camchain or export_dir / "calib-camchain.yaml"
    if not camchain.is_file():
        # Kalibr 有时写成 bag名-camchain.yaml
        cands = sorted(export_dir.glob("*camchain*.yaml"))
        if not cands:
            raise FileNotFoundError(f"未找到 camchain: {export_dir}")
        camchain = cands[0]

    results_txt = args.results_txt or _find_results_txt(export_dir, camchain)
    payload = convert(
        camchain,
        cam_topic_map=meta.get("cam_topic_map") or {},
        camera_names=meta.get("camera_names"),
        models=meta.get("models"),
        session_dir=meta.get("session_dir"),
        selected_frames=meta.get("num_selected_frames"),
        results_txt=results_txt,
    )
    camera_names = _camera_names_from_meta(meta, len(payload["cameras"]))
    _save_named_camchain(camchain, camera_names)
    out_json = args.out_json or export_dir / "kalibr_calibration.json"
    out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    # 同步一份到 session 根目录（若可推断）
    session = meta.get("session_dir")
    if session:
        side = Path(session) / "kalibr_calibration.json"
        side.write_text(out_json.read_text(encoding="utf-8"), encoding="utf-8")
        print(f"also → {side}")

    # 简要打印外参 + 误差
    ref = payload["reference_camera"]
    print(f"wrote → {out_json}")
    print(f"models: {payload['kalibr_models']}")
    print(f"reference: {ref}")
    if results_txt:
        print(f"errors from: {results_txt}")
    for name, cam in payload["cameras"].items():
        r = cam.get("reprojection_error_px") or {}
        rms = r.get("rms_approx")
        std = r.get("std_xy")
        if rms is not None:
            print(f"  {name} reproj_rms≈{rms:.3f}px  std_xy={std}")
        if name == ref:
            continue
        t = cam["T_from_ref"]["t_m"]
        unc = (cam.get("T_from_ref") or {}).get("uncertainty") or {}
        t_std = unc.get("t_std_m")
        extra = f"  t_std_m={t_std}" if t_std else ""
        print(
            f"  {name}_from_{ref}: t_m={[round(x, 6) for x in t]}  "
            f"|t|={np.linalg.norm(t)*1000:.1f}mm{extra}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
