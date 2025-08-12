#!/usr/bin/env python3
"""
PNG 투명 여백 자동 크롭 스크립트

- 원리: 알파 채널(투명도) 기준으로 유효 픽셀(alpha > threshold)의 최소 바운딩 박스를 잘라냄
- 옵션:
  --alpha-th    : 유효 픽셀 알파 임계값 (기본 5)
  --pad         : 바운딩 박스 주변 패딩 픽셀 (기본 2)
  --inplace     : 원본 덮어쓰기 (기본은 *_trim.png 생성)
  --input       : 단일 파일 처리
  --src-dir / --pattern : 일괄 처리

예)
  python trim_png.py --src-dir . --pattern "rgui_*.png" --alpha-th 5 --pad 2 --inplace
  python trim_png.py --input rgui_guide_1.png --pad 4
"""
from __future__ import annotations
import argparse
import os
import sys
import glob
import cv2
import numpy as np


def trim_one(input_path: str, output_path: str, alpha_th: int, pad: int) -> bool:
    img = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"[WARN] 읽기 실패: {input_path}")
        return False

    if img.shape[2] < 4:
        # 알파가 없으면 크롭 불가 → 그대로 저장
        return cv2.imwrite(output_path, img)

    bgr = img[:, :, :3]
    alpha = img[:, :, 3]

    mask = (alpha > alpha_th).astype(np.uint8)
    if mask.max() == 0:
        # 전부 투명 → 그대로 저장
        return cv2.imwrite(output_path, img)

    ys, xs = np.where(mask > 0)
    y1, y2 = int(np.min(ys)), int(np.max(ys))
    x1, x2 = int(np.min(xs)), int(np.max(xs))

    # 패딩 적용
    h, w = alpha.shape
    x1 = max(0, x1 - pad)
    y1 = max(0, y1 - pad)
    x2 = min(w - 1, x2 + pad)
    y2 = min(h - 1, y2 + pad)

    cropped = img[y1:y2 + 1, x1:x2 + 1]
    ok = cv2.imwrite(output_path, cropped)
    if not ok:
        print(f"[WARN] 저장 실패: {output_path}")
    return ok


essential = [
    "rgui_guide_1.png",
    "rgui_card.png",
    "rgui_touch.png",
    "rgui_scan.png",
    "rgui_guide_out.png",
    "rgui_eye_2.png",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=str, help="단일 파일 경로")
    ap.add_argument("--src-dir", type=str, default=".", help="일괄 처리 폴더")
    ap.add_argument("--pattern", type=str, default="*.png", help="glob 패턴")
    ap.add_argument("--alpha-th", type=int, default=5, help="알파 임계값")
    ap.add_argument("--pad", type=int, default=2, help="패딩 픽셀")
    ap.add_argument("--inplace", action="store_true", help="원본 덮어쓰기")
    args = ap.parse_args()

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
        out = p if args.inplace else f"{root}_trim.png"
        if trim_one(p, out, args.alpha_th, args.pad):
            processed += 1
            print(f"[OK] {os.path.basename(p)} -> {os.path.basename(out)}")
    print(f"[DONE] {processed}/{len(targets)} 파일 크롭")
    return 0


if __name__ == "__main__":
    sys.exit(main()) 