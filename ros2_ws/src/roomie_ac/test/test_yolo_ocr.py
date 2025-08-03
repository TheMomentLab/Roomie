import cv2
from ultralytics import YOLO
import easyocr

# --- 설정 ---
# 사용자의 환경에 맞게 이 부분의 값을 수정하세요.
MODEL_PATH = '/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/best.pt'
CAMERA_INDEX = 4
CONFIDENCE_THRESHOLD = 0.25 # 테스트를 위해 신뢰도 기준을 조금 낮게 설정

# 특정 클래스만 감지하고 싶을 때 이 변수를 수정합니다. (예: 'button_')
# None이면 감지되는 모든 클래스를 화면에 표시합니다.
TARGET_CLASS_PREFIX = None 
# --- 설정 끝 ---


def main():
    """
    YOLO 모델과 OCR을 사용하여 실시간으로 객체를 감지하고 테스트하는 메인 함수
    """
    # 1. 모델 및 리더기 초기화
    print("YOLO 모델을 로드하는 중...")
    model = YOLO(MODEL_PATH)
    model.conf = CONFIDENCE_THRESHOLD
    print("EasyOCR 리더기를 로드하는 중... (첫 실행 시 시간이 걸릴 수 있습니다)")
    # GPU 사용이 어려울 경우 gpu=False로 변경
    reader = easyocr.Reader(['en'], gpu=True) 
    print("모델 및 리더기 로드 완료.")

    # 2. 카메라 열기
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"오류: 카메라 인덱스 {CAMERA_INDEX}를 열 수 없습니다.")
        return
    print(f"카메라 {CAMERA_INDEX} 실행 중... 'q' 키를 누르면 종료됩니다.")

    # 3. 메인 루프: 실시간 감지 및 표시
    while True:
        ret, frame = cap.read()
        if not ret:
            print("오류: 카메라에서 프레임을 읽을 수 없습니다.")
            break

        # YOLO 모델로 객체 감지 수행
        results = model(frame)

        # 감지된 각 객체에 대해 반복
        for result in results:
            for box in result.boxes:
                # 클래스 이름 가져오기
                class_id = int(box.cls[0])
                class_name = model.names[class_id]

                # [핵심] 특정 클래스 이름 필터링 로직
                # TARGET_CLASS_PREFIX가 설정된 경우, 해당 이름으로 시작하지 않으면 건너뜀
                if TARGET_CLASS_PREFIX is not None and not class_name.startswith(TARGET_CLASS_PREFIX):
                    continue

                # 경계 상자 좌표 가져오기
                xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                
                # 감지된 객체 영역(ROI) 자르기
                roi = frame[ymin:ymax, xmin:xmax]

                # OCR 수행
                ocr_text = "N/A"
                if roi.size > 0:
                    ocr_results = reader.readtext(roi, detail=1)
                    # OCR 결과에서 텍스트만 추출하여 한 줄로 합침
                    ocr_text = ' '.join([res[1] for res in ocr_results])

                # 화면에 결과 그리기
                # 1. 경계 상자
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                # 2. 정보 텍스트 (클래스, 신뢰도, OCR)
                label_class = f"Class: {class_name}"
                label_conf = f"Conf: {box.conf[0]:.2f}"
                label_ocr = f"OCR: {ocr_text}"
                
                # 텍스트 위치를 보기 좋게 조정
                cv2.putText(frame, label_class, (xmin, ymin - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, label_conf, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, label_ocr, (xmin, ymax + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 결과 영상 보여주기
        cv2.imshow("YOLO + OCR Test | Press 'q' to quit", frame)

        # 'q' 키를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 4. 자원 해제
    cap.release()
    cv2.destroyAllWindows()
    print("프로그램을 종료합니다.")


if __name__ == '__main__':
    main()