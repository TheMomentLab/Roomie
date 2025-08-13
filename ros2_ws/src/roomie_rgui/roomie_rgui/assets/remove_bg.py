#!/usr/bin/env python3
"""
PNG 배경 제거 스크립트 (단색 키 제거)

- 기본: 이미지 네 모서리 평균색을 배경으로 자동 추정(auto)
- 수동: --bg #RRGGBB 로 배경색 지정 가능
- 허용 오차: --tol 30 (0~255) 가까울수록 엄격
- 출력: 기본은 *_transparent.png, --inplace 지정 시 원본 덮어쓰기
- 일괄 처리: --src-dir 와 --pattern(glob)

예)
  python remove_bg.py --src-dir . --pattern "rgui_*.png" --bg auto --tol 28 --feather 1
  python remove_bg.py --input rgui_guide_1.png --inplace
"""
from __future__ import annotations
import argparse
import os
import sys
import glob
import numpy as np
import cv2


def parse_color(s: str) -> np.ndarray:
    s = s.strip().lower()
    if s == "auto":
        return None
    if s.startswith("#"):
        s = s[1:]
    if len(s) != 6:
        raise ValueError("색상은 #RRGGBB 형식이어야 합니다")
    r = int(s[0:2], 16)
    g = int(s[2:4], 16)
    b = int(s[4:6], 16)
    return np.array([b, g, r], dtype=np.float32)  # OpenCV BGR


def detect_bg_bgr(img_bgr: np.ndarray, corner: int = 10) -> np.ndarray:
    h, w = img_bgr.shape[:2]
    c = corner
    patches = [
        img_bgr[0:c, 0:c],
        img_bgr[0:c, w - c:w],
        img_bgr[h - c:h, 0:c],
        img_bgr[h - c:h, w - c:w],
    ]
    mean = np.mean([p.reshape(-1, 3).mean(axis=0) for p in patches], axis=0)
    return mean.astype(np.float32)


def remove_bg_one(input_path: str, output_path: str, bg: str, tol: int, feather: int) -> bool:
    img = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"[WARN] 읽기 실패: {input_path}")
        return False

    if img.shape[2] == 3:
        bgr = img.astype(np.float32)
        alpha_init = np.full((img.shape[0], img.shape[1]), 255, dtype=np.uint8)
    else:
        bgr = img[:, :, :3].astype(np.float32)
        alpha_init = img[:, :, 3].copy()

    bg_bgr = detect_bg_bgr(bgr) if bg == "auto" else parse_color(bg)
    diff = np.linalg.norm(bgr - bg_bgr, axis=2)
    mask_bg = (diff <= float(tol))

    alpha = alpha_init.copy()
    alpha[mask_bg] = 0

    if feather > 0:
        k = max(1, int(feather))
        k = k + (1 - k % 2)  # 홀수
        alpha = cv2.GaussianBlur(alpha, (k, k), 0)

    out = np.dstack([bgr.astype(np.uint8), alpha])
    ok = cv2.imwrite(output_path, out)
    if not ok:
        print(f"[WARN] 저장 실패: {output_path}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=str, help="단일 파일 경로")
    ap.add_argument("--src-dir", type=str, default=".", help="일괄 처리 폴더")
    ap.add_argument("--pattern", type=str, default="*.png", help="glob 패턴")
    ap.add_argument("--bg", type=str, default="auto", help="배경색(#RRGGBB 또는 auto)")
    ap.add_argument("--tol", type=int, default=28, help="허용 오차(0~255)")
    ap.add_argument("--feather", type=int, default=1, help="가장자리 페더(픽셀)")
    ap.add_argument("--inplace", action="store_true", help="원본 덮어쓰기")
    args = ap.parse_args()

    targets: list[str] = []
    if args.input:
        targets = [args.input]
    else:
        targets = sorted(glob.glob(os.path.join(args.src_dir, args.pattern)))

    if not targets:
        print("[INFO] 처리할 파일이 없습니다")
        return 0

    processed = 0
    for p in targets:
        root, ext = os.path.splitext(p)
        out = p if args.inplace else f"{root}_transparent.png"
        if remove_bg_one(p, out, args.bg, args.tol, args.feather):
            processed += 1
            print(f"[OK] {os.path.basename(p)} -> {os.path.basename(out)}")
    print(f"[DONE] {processed}/{len(targets)} 파일 처리")
    return 0


if __name__ == "__main__":
    sys.exit(main()) 