1️⃣ 전체 시스템 구성 개요
📌 주요 구성 요소

주행 노드 (IntegratedNavigation)

비전 인식 노드

사람 인식 (YOLO)

신호등 인식 (색상 기반)

GUI 관제 프로그램 (PyQt5)

로그 시스템

MinIO (이미지 저장)

MySQL (상황 로그 DB)

2️⃣ 주행 / 제어 시스템
🔁 주행 우선순위 구조 (핵심 설계)
비상정지 > 사람 추종 > 신호등 정지 > 일반 자율 주행

✅ 제어 흐름

비상정지

어떤 상황이든 즉시 정지

버튼 토글 방식 (누르면 정지 / 다시 누르면 해제)

사람 감지

감지되면 경로 주행 중단

사람 중심 좌표 기반 추종 주행

신호등 감지

RED → 정지

GREEN → 주행 재개

일반 자율 주행

A* 기반 경로 생성

Waypoint 순찰

3️⃣ 비상정지 (Emergency Stop)
🔴 기능 요약

/emergency_stop 토픽 수신

토글 방식

1회: 즉시 정지 + 로그 발생

2회: 정지 해제 (자동 주행 가능)

🔒 특징

사람 추종 중이든, 신호등이든 무조건 최우선

GUI 버튼과 실제 주행 노드 완전 연동

4️⃣ 사람 인식 & 추종 기능
👤 사람 인식

YOLO 기반 사람 감지

감지 시:

/human_state → "PERSON"

/person → 사람 중심 x좌표 발행

🚶 사람 추종 로직
angular.z = -0.003 * (person_x - image_center)
linear.x  = 0.12

📌 설계 포인트

사람 감지 시 1회만 로그 발생

손, 부분 객체 오인식 감소를 위해:

confidence threshold 조절

조건 단순화 (하나의 조건만 사용)

5️⃣ 신호등 인식 기능
🚦 인식 방식

카메라 이미지 기반 색상 인식

GREEN / RED 상태를 문자열로 발행

🟢🟥 주행 제어
신호	동작
RED	정지
GREEN	주행
6️⃣ GUI 관제 시스템 (PyQt5)
🖥️ 실시간 화면

카메라 영상

지도(Map) + 경로(Path)

로봇 위치 표시

🎛️ 제어 버튼

DRIVING

STOP

AUTO / MANUAL

EMERGENCY STOP

7️⃣ 이벤트 로그 시스템 (핵심 기능)
📣 로그 발생 조건

비상정지 발생

사람 감지 (HUMAN DETECT)

기타 주요 이벤트

8️⃣ MinIO 이미지 저장 시스템
📦 역할

이벤트 발생 시 카메라 이미지 저장

MinIO(S3 호환)에 이미지 업로드

🔗 저장 방식
http://<minio-ip>:9000/patrolimage/UUID.png

✅ 특징

이벤트 로그와 이미지 URL 연동

DB에는 실제 이미지 파일이 아닌 URL만 저장

9️⃣ MySQL 로그 DB 구조
🗄️ 테이블: situation_log
컬럼명	설명
situation	상황 (HUMAN DETECT, EMERGENCY STOP 등)
status	상태 설명
x	로봇 X 좌표
y	로봇 Y 좌표
image_path	MinIO 이미지 URL
created_at	발생 시간
🔟 GUI 로그 히스토리 기능
📋 Log History 창

DB에서 최근 로그 조회

테이블 형태로 표시

🎨 시각적 강조
상황	색상
HUMAN DETECT	연한 빨강 (RGB 255,200,200)
EMERGENCY	연한 노랑 (RGB 255,245,200)
🔍 이미지 보기

"보기" 버튼 클릭 시

웹브라우저로 MinIO 이미지 열기

1️⃣1️⃣ 이벤트 중복 방지 로직
🧠 설계 의도

같은 이벤트가 연속으로 쌓이는 것 방지

✅ 구현

uploaded_once 플래그 사용

HUMAN 이벤트는 예외 처리 (연속 표시 허용)
