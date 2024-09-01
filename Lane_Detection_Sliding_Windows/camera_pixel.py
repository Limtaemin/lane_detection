import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 클릭한 좌표를 저장할 리스트
points = []

# ROS 초기화
rospy.init_node('camera_point_selector')

# CvBridge 생성
bridge = CvBridge()

# 클릭한 좌표를 삭제하거나 추가하는 함수
def get_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # 마우스 왼쪽 버튼 클릭 시
        for i, point in enumerate(points):
            if abs(point[0] - x) < 10 and abs(point[1] - y) < 10:  # 점 근처를 클릭한 경우
                print(f"점 삭제: ({point[0]}, {point[1]})")
                points.pop(i)
                return
        points.append((x, y))  # 새로운 점 추가
        print(f"좌표: ({x}, {y})")

# 프레임에 선택된 좌표에 점을 그리는 함수
def draw_points_on_frame(frame):
    for point in points:
        cv2.circle(frame, point, 5, (0, 0, 255), -1)  # 빨간색(BGR: 0, 0, 255) 점 그리기

# 카메라 이미지 콜백 함수
def image_callback(msg):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # 프레임 크기 조정
    frame = cv2.resize(frame, (640, 480))

    # 프레임에 점 그리기
    draw_points_on_frame(frame)

    cv2.imshow('frame', frame)
    cv2.setMouseCallback('frame', get_mouse_click)  # 마우스 콜백 설정

    key = cv2.waitKey(1)
    if key == ord('q'):  # 'q'를 누르면 종료
        rospy.signal_shutdown('Quit')
        cap.release()
        cv2.destroyAllWindows()
    elif key == ord('p'):  # 'p'를 누르면 프레임 정지
        while True:  # 정지 상태 유지
            frame_copy = frame.copy()
            draw_points_on_frame(frame_copy)
            cv2.imshow('frame', frame_copy)  # 업데이트된 프레임 표시
            
            key = cv2.waitKey(1)
            if key == ord('p'):  # 다시 'p'를 누르면 재생
                break
            elif key == ord('q'):  # 'q'를 누르면 종료
                rospy.signal_shutdown('Quit')
                cv2.destroyAllWindows()
                exit()
            elif key == ord('d'):  # 'd'를 누르면 마지막 점 삭제
                if points:
                    deleted_point = points.pop()
                    print(f"최근 점 삭제: ({deleted_point[0]}, {deleted_point[1]})")

# ROS 이미지 토픽 구독
rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

# ROS 루프 실행
rospy.spin()

# 모든 창 닫기
cv2.destroyAllWindows()
