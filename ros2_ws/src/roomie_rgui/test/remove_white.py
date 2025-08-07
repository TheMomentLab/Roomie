#!/usr/bin/env python3
from PIL import Image
import os

def remove_white_background(input_path, output_path, tolerance=30):
    """
    이미지의 흰색 배경을 투명하게 만듭니다.
    
    Args:
        input_path: 입력 이미지 경로
        output_path: 출력 이미지 경로
        tolerance: 흰색으로 간주할 색상 허용 오차 (0-255)
    """
    try:
        # 이미지 로드
        img = Image.open(input_path)
        
        # RGBA 모드로 변환 (투명도 지원)
        if img.mode != 'RGBA':
            img = img.convert('RGBA')
        
        # 이미지 데이터 가져오기
        data = img.getdata()
        
        # 새로운 데이터 생성 (흰색을 투명하게)
        new_data = []
        for item in data:
            # RGB 값이 모두 높으면 (흰색에 가까우면) 투명하게
            if item[0] > 255 - tolerance and item[1] > 255 - tolerance and item[2] > 255 - tolerance:
                new_data.append((255, 255, 255, 0))  # 투명
            else:
                new_data.append(item)  # 원본 색상 유지
        
        # 새 이미지 생성
        new_img = Image.new('RGBA', img.size)
        new_img.putdata(new_data)
        
        # 저장
        new_img.save(output_path, 'PNG')
        print(f"✅ 성공: {input_path} -> {output_path}")
        print(f"   크기: {img.size}")
        print(f"   모드: {img.mode} -> RGBA")
        
    except Exception as e:
        print(f"❌ 오류: {e}")

if __name__ == "__main__":
    # 파일 경로 설정
    input_file = "rgui_caution.png"
    output_file = "rgui_caution.png"  # 같은 이름으로 저장
    
    # 현재 디렉토리에서 실행
    if os.path.exists(input_file):
        remove_white_background(input_file, output_file, tolerance=30)
    else:
        print(f"❌ 파일을 찾을 수 없습니다: {input_file}") 