import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 전역 변수 선언
previous_left_base = None
previous_right_base = None
EXPECTED_LANE_DISTANCE =  500  # 두 차선 간의 예상 거리 (픽셀 단위)

# ROS 노드 초기화
rospy.init_node('lane_detection')

# CvBridge 생성
bridge = CvBridge()

# 트랙바 콜백 함수 (큰 의미는 없지만 필요)
def nothing(x):
    pass

# 트랙바 생성 함수
def create_trackbars():
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
    # 가우시안 블러용 트랙바 추가
    cv2.createTrackbar("Blur Kernel Size", "Trackbars", 1, 20, nothing)  # 커널 크기 (1~20)
    cv2.createTrackbar("Blur SigmaX", "Trackbars", 0, 100, nothing)  # SigmaX (0~100)

# 카메라 이미지 콜백 함수
def image_callback(msg):
    global previous_left_base, previous_right_base  # 전역 변수를 사용하도록 설정

    # ROS Image 메시지를 OpenCV 이미지로 변환
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # 영상 크기 조정
    frame = cv2.resize(frame, (640, 480))

    # 관점 변환(버드뷰)
    tl = (111,335)
    bl = (25,469)
    tr = (547,330)
    br = (615,473)

    pts1 = np.float32([tl, bl, tr, br])
    pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    transformed_frame = cv2.warpPerspective(frame, matrix, (640, 480))

    # Object Detection
    hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)

    # 트랙바에서 가우시안 블러의 커널 크기와 SigmaX 가져오기
    kernel_size = cv2.getTrackbarPos("Blur Kernel Size", "Trackbars") * 2 + 1  # 홀수만 허용
    sigma_x = cv2.getTrackbarPos("Blur SigmaX", "Trackbars") / 10.0  # 적절한 범위로 스케일링

    # 가우시안 블러 적용
    blurred_frame = cv2.GaussianBlur(hsv_transformed_frame, (kernel_size, kernel_size), sigma_x)

    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")

    lower = np.array([l_h, l_s, l_v])
    upper = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(blurred_frame, lower, upper)

    # 컬러로 변환 (컨투어가 보이도록)
    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # 히스토그램 만들기 
    histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
    midpoint = np.int(histogram.shape[0] / 2)
    left_base = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    # 이전 프레임의 베이스 값을 저장할 변수
    if previous_left_base is None:
        previous_left_base = left_base
    if previous_right_base is None:
        previous_right_base = right_base

    y = 472
    lx = []
    rx = []

    while y > 0:
        ## 왼쪽 내에서 검출하기
        img = mask[y - 40:y, left_base - 50:left_base + 50]
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            cv2.drawContours(mask_colored[y - 40:y, left_base - 50:left_base + 50], contours, -1, (0, 255, 0), 3)

            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    left_base = left_base - 50 + cx
                    previous_left_base = left_base  # 이전 베이스값 갱신
        else:
            # 왼쪽 컨투어가 없을 경우, 오른쪽 컨투어로부터 예상 위치 추정
            left_base = previous_right_base - EXPECTED_LANE_DISTANCE

        lx.append(left_base)
        cv2.circle(mask_colored, (left_base, y - 20), 5, (0, 0, 255), -1)  # 점으로 표시

        ## 오른쪽 내에서 검출하기
        img = mask[y - 40:y, right_base - 50:right_base + 50]
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
        cv2.circle(mask_colored, (right_base, y - 20), 5, (0, 0, 255), -1)  # 점으로 표시

        # 두 베이스의 간격 확인 (안정성 추가)
        lane_distance = abs(left_base - right_base)
        
        # 차선 간의 거리 조건 확인 후 조정
        if lane_distance < EXPECTED_LANE_DISTANCE - 30 or lane_distance > EXPECTED_LANE_DISTANCE + 30:
            if len(contours) == 0:  # 오른쪽이 사라진 경우
                right_base = left_base + EXPECTED_LANE_DISTANCE
            elif len(contours) == 0:  # 왼쪽이 사라진 경우
                left_base = right_base - EXPECTED_LANE_DISTANCE

        cv2.rectangle(mask_colored, (left_base - 50, y), (left_base + 50, y - 40), (255, 0, 0), 2)
        cv2.rectangle(mask_colored, (right_base - 50, y), (right_base + 50, y - 40), (255, 0, 0), 2)

        y -= 40

    # 각 프레임마다 이미지를 갱신
    cv2.imshow("Original", frame)
    cv2.imshow("Bird's Eye View", transformed_frame)
    cv2.imshow("Blurred Frame", blurred_frame)  # 가우시안 블러가 적용된 화면 출력
    cv2.imshow("Lane Detection - Image Thresholding", mask)
    cv2.imshow("Lane Detection - Sliding Windows with Contours", mask_colored)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('Quit')

# 트랙바 생성
create_trackbars()

# ROS 이미지 토픽 구독
rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

# ROS 루프 실행
rospy.spin()

# 모든 창 닫기
cv2.destroyAllWindows()
