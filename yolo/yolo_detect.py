import cv2
from ultralytics import YOLO

# 모델 로드
model = YOLO('best.pt')

# 클래스 이름 확인 (디버깅용)
print("Model Classes:", model.names)

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    
    results = model(frame, stream=True, verbose=False)

    # 현재 프레임에서 감지된 신호 상태 변수
    detected_status = None 

    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf[0]
            cls = int(box.cls[0])
            name = model.names[cls] # 모델이 인식한 클래스 이름

            # 기본 박스 색상 (보라색)
            box_color = (127, 0, 127) 
            
            # 대소문자 구분 없이 이름에 'red'나 'green'이 들어가는지 확인
            lower_name = name.lower()

            if "red" in lower_name:
                detected_status = "RED"
                box_color = (0, 0, 255) # 빨간색 (BGR)
            elif "green" in lower_name:
                detected_status = "GREEN"
                box_color = (0, 255, 0) # 초록색 (BGR)

            # 물체에 박스 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(frame, f"{name} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

    # 화면 중앙에 큰 글씨로 상태 출력
    height, width, channels = frame.shape
    
    if detected_status == "RED":
        # 빨간색 텍스트 출력
        cv2.putText(frame, "RED LIGHT!", (50, height//2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
    elif detected_status == "GREEN":
        # 초록색 텍스트 출력
        cv2.putText(frame, "GREEN LIGHT!", (50, height//2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)

    cv2.imshow('frame', frame)

    if cv2.waitKey(50) == 27: # ESC키 누르면 종료
        break

cap.release()
cv2.destroyAllWindows()