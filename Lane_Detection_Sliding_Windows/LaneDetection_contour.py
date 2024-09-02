import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32

# ROS 노드 초기화
rospy.init_node('lane_detection_and_steering')

# 비디오 파일 캡쳐
vidcap = cv2.VideoCapture("solidWhiteRight.mp4")

# 콜백함수, 큰 의미는 없지만 함수 작동에 필요함
def nothing(x):
    pass

# 트랙바 윈도우 창 만들기
cv2.namedWindow("Trackbars")

# 트랙바 구성요소 만들기
cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

# 초기 베이스값 및 곡률값 설정
previous_left_base = None
previous_right_base = None
previous_offset = None
EXPECTED_LANE_DISTANCE = 450  # 두 차선 간의 예상 거리 (픽셀 단위)
OFFSET_THRESHOLD = 70  # 허용되는 센터 오프셋 변화의 임계값
WHEELBASE = 1.2  # 차량 휠베이스 (단위: 미터)

# 조향 각도 토픽 발행자
steering_pub = rospy.Publisher('/steering_angle', Float32, queue_size=10)

# 오프셋 값을 저장할 리스트 초기화
offset_values = []

# 곡률 계산 함수
def calculate_curvature(poly_coefficients, y_eval):
    A = poly_coefficients[0]
    B = poly_coefficients[1]
    curvature_radius = ((1 + (2 * A * y_eval + B) ** 2) ** 1.5) / np.abs(2 * A)
    return curvature_radius

# 곡률 시각화 함수
def draw_curvature_line(img, fit, color=(255, 0, 0)):
    ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
    curve_x = fit[0] * ploty ** 2 + fit[1] * ploty + fit[2]
    curve_points = np.array([np.transpose(np.vstack([curve_x, ploty]))])
    cv2.polylines(img, np.int32(curve_points), isClosed=False, color=color, thickness=5)

# 조향 각도 계산 함수
def calculate_steering_angle(curvature_radius, wheelbase=2.5):
    if curvature_radius != 0:
        steering_angle_rad = np.arctan(wheelbase / curvature_radius)  # 라디안 단위
        steering_angle_deg = np.degrees(steering_angle_rad)  # 도(degree) 단위로 변환
        return steering_angle_deg
    else:
        return 0.0 

# 메인 루프
while not rospy.is_shutdown():
    success = True
    while success:
        success, image = vidcap.read()
        
        if not success or image is None:  # 이미지가 없는 경우 루프 종료
            break

        # 영상 크기 조정 (640x480)
        frame = cv2.resize(image, (640, 480))

        # 가우시안 블러 적용
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # 버드뷰(관점 변환)를 위한 포인트 설정
        tl = (260, 309)
        bl = (71, 448)
        tr = (385, 301)
        br = (600, 445)

        # 버드뷰 변환을 위한 좌표 설정
        pts1 = np.float32([tl, bl, tr, br])
        pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

        # 변환 행렬 계산
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        # 버드뷰로 변환
        transformed_frame = cv2.warpPerspective(blurred_frame, matrix, (640, 480))

        # Object Detection (차선 검출)
        # HSV 색공간으로 변환
        hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)

        # 트랙바 값을 읽어와서 HSV 범위 설정
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        # 하한값과 상한값을 배열로 저장
        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])

        # 설정된 HSV 범위로 이진 마스크 생성
        mask = cv2.inRange(hsv_transformed_frame, lower, upper)

        # 전처리 1단계: 형태학적 닫기 연산 적용
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 컬러로 변환 (컨투어가 보이도록)
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # 히스토그램 생성 (하단 절반에서의 가로 축의 픽셀 값 합계)
        histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
        # 히스토그램의 중앙점 계산
        midpoint = np.int(histogram.shape[0] / 2)
        # 히스토그램에서 가장 높은 값(차선의 시작점)을 왼쪽과 오른쪽으로 구분
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        # 이전 프레임의 베이스 값을 저장할 변수 초기화
        if previous_left_base is None:
            previous_left_base = left_base
        if previous_right_base is None:
            previous_right_base = right_base

        y = 472
        lx = []
        ly = []
        rx = []
        ry = []

        # 첫 번째 윈도우에서 차선 중심 계산
        initial_lane_center = (left_base + right_base) // 2
        image_center = mask_colored.shape[1] // 2
        initial_offset = initial_lane_center - image_center

        # 슬라이딩 윈도우를 사용하여 차선 검출
        while y > 0:
            # 왼쪽 차선 검출
            img = mask[y - 40:y, left_base - 50:left_base + 50]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # 전처리 2단계: 컨투어 필터링: 너무 작은 노이즈 컨투어 제거
            contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]

            if len(contours) > 0:
                cv2.drawContours(mask_colored[y - 40:y, left_base - 50:left_base + 50], contours, -1, (0, 255, 0), 3)

                for contour in contours:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        left_base = left_base - 50 + cx
                        previous_left_base = left_base  # 이전 베이스값 갱신
            else:
                # 노이즈 방지 1단계:  왼쪽 컨투어가 없을 경우, 오른쪽 컨투어로부터 예상 위치 추정
                left_base = previous_right_base - EXPECTED_LANE_DISTANCE

            lx.append(left_base)
            ly.append(y)
            cv2.circle(mask_colored, (left_base, y - 20), 5, (0, 0, 255), -1)  # 점으로 표시

            # 오른쪽 차선 검출
            img = mask[y - 40:y, right_base - 50:right_base + 50]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # 컨투어 필터링: 너무 작은 노이즈 컨투어 제거
            contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]

            if len(contours) > 0:
                cv2.drawContours(mask_colored[y - 40:y, right_base - 50:right_base + 50], contours, -1, (0, 255, 0), 3)

                for contour in contours:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        right_base = right_base - 50 + cx
                        previous_right_base = right_base  # 이전 베이스값 갱신
            else:
                # 오른쪽 컨투어가 없을 경우, 왼쪽 컨투어로부터 예상 위치 추정
                right_base = previous_left_base + EXPECTED_LANE_DISTANCE

            rx.append(right_base)
            ry.append(y)
            cv2.circle(mask_colored, (right_base, y - 20), 5, (0, 0, 255), -1)  # 점으로 표시

            # 두 베이스의 간격 확인 (안정성 추가)
            lane_distance = abs(left_base - right_base)
            
            # 왼쪽 차선만 보일 경우 오른쪽 차선 추정
            if lane_distance < EXPECTED_LANE_DISTANCE - 30 or lane_distance > EXPECTED_LANE_DISTANCE + 30:
                if len(contours) == 0:  # 오른쪽이 사라진 경우
                    right_base = left_base + EXPECTED_LANE_DISTANCE
                elif len(contours) == 0:  # 왼쪽이 사라진 경우
                    left_base = right_base - EXPECTED_LANE_DISTANCE

            # 슬라이딩 윈도우 시각화
            cv2.rectangle(mask_colored, (left_base - 50, y), (left_base + 50, y - 40), (255, 0, 0), 2)
            cv2.rectangle(mask_colored, (right_base - 50, y), (right_base + 50, y - 40), (255, 0, 0), 2)

            y -= 40

        # 곡률 계산 및 시각화
        y_eval = np.max(ly)  # 곡률 계산을 위한 y 값 평가
        left_fit = np.polyfit(ly, lx, 2)
        right_fit = np.polyfit(ry, rx, 2)

        left_curvature = calculate_curvature(left_fit, y_eval)
        right_curvature = calculate_curvature(right_fit, y_eval)
        avg_curvature = (left_curvature + right_curvature) / 2

        # 곡률 시각화 ("Lane Detection - Sliding Windows with Contours" 창에 그리기)
        avg_fit = (left_fit + right_fit) / 2
        draw_curvature_line(mask_colored, avg_fit, color=(0, 255, 0))

        # 이미지의 첫 번째 윈도우에서 중앙 픽셀과 차선 중심 사이의 거리 계산
        lane_center = (lx[0] + rx[0]) // 2  # 첫 번째 슬라이딩 윈도우의 차선 중심
        offset = lane_center - image_center

        # 노이즈 방지 2단계: 센터 오프셋 변화 확인 및 노이즈 제거
        if previous_offset is not None:
            if abs(offset - previous_offset) > OFFSET_THRESHOLD:
                offset = previous_offset

        # 오프셋값 업데이트
        previous_offset = offset

        # 오프셋을 리스트에 저장
        offset_values.append(offset)

        # 조향 각도 계산 및 토픽 발행
        steering_angle = calculate_steering_angle(avg_curvature)
        print(f"Steering angle: {steering_angle:.2f} degrees")
        steering_pub.publish(steering_angle)

        
        # 곡률 정보와 중앙 오프셋을 화면에 표시
        cv2.putText(mask_colored, f"Average curvature: {avg_curvature:.2f} px", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(mask_colored, f"Center offset: {offset:.2f} px", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(mask_colored, f"Steering_angle_deg: {steering_angle:.4f}", (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 결과 이미지 출력
        cv2.imshow("Original", frame)
        cv2.imshow("Bird's Eye View", transformed_frame)
        cv2.imshow("Lane Detection - Image Thresholding", mask)
        cv2.imshow("Lane Detection - Sliding Windows with Contours", mask_colored)

        # 0.5배속 재생
        if cv2.waitKey(100) == 27:
            break

    # 영상이 끝나면 다시 시작점으로 돌아가도록 함
    vidcap.set(cv2.CAP_PROP_POS_FRAMES, 0)

cv2.destroyAllWindows()