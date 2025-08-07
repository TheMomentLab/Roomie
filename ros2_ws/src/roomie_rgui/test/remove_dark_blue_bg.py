#!/usr/bin/env python3
"""
남색 배경 제거 스크립트
rgui_box_2.png 파일의 남색 배경을 투명하게 만듭니다.
"""

from PIL import Image
import numpy as np
import os

def remove_dark_blue_background(input_path, output_path, tolerance=30):
    """
    남색 배경을 제거하고 투명하게 만듭니다.
    
    Args:
        input_path (str): 입력 이미지 경로
        output_path (str): 출력 이미지 경로
        tolerance (int): 색상 허용 오차 (0-255)
    """
    
    # 이미지 로드
    img = Image.open(input_path)
    
    # RGBA 모드로 변환 (투명도 지원)
    if img.mode != 'RGBA':
        img = img.convert('RGBA')
    
    # 이미지를 numpy 배열로 변환
    data = np.array(img)
    
    # 남색 배경 색상 정의 (RGB)
    # 어두운 남색: 약 (23, 30, 38) 또는 (17, 30, 38)
    dark_blue = np.array([23, 30, 38])
    
    # 배경 마스크 생성
    # 각 픽셀이 남색과 얼마나 다른지 계산
    diff = np.sqrt(np.sum((data[:, :, :3] - dark_blue) ** 2, axis=2))
    
    # 허용 오차 내의 픽셀을 배경으로 간주
    background_mask = diff <= tolerance
    
    # 배경 픽셀의 알파값을 0으로 설정 (투명하게)
    data[background_mask, 3] = 0
    
    # numpy 배열을 다시 이미지로 변환
    result_img = Image.fromarray(data)
    
    # 결과 저장
    result_img.save(output_path, 'PNG')
    
    print(f"배경 제거 완료: {output_path}")
    print(f"원본 파일: {input_path}")
    print(f"허용 오차: {tolerance}")

def main():
    # 파일 경로 설정
    input_file = "rgui_elevator.png"
    output_file = "rgui_elevator.png"  # 같은 파일명으로 덮어쓰기
    
    # 현재 디렉토리에서 실행
    if os.path.exists(input_file):
        remove_dark_blue_background(input_file, output_file)
    else:
        print(f"파일을 찾을 수 없습니다: {input_file}")

if __name__ == "__main__":
    main() 