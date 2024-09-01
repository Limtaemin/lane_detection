import cv2

cap = cv2.VideoCapture('solidWhiteRight.mp4', cv2.CAP_ANY)

# 클릭한 좌표를 저장할 리스트
points = []

def get_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # 마우스 왼쪽 버튼 클릭 시
        # 이미 찍은 점을 클릭하면 해당 점을 삭제
        for i, point in enumerate(points):
            if abs(point[0] - x) < 10 and abs(point[1] - y) < 10:  # 점 근처를 클릭한 경우
                print(f"점 삭제: ({point[0]}, {point[1]})")
                points.pop(i)
                return
        # 새로운 점 추가
        points.append((x, y))
        print(f"좌표: ({x}, {y})")

def draw_points_on_frame(frame):
    # 클릭한 모든 좌표에 빨간색 점 그리기
    for point in points:
        cv2.circle(frame, point, 5, (0, 0, 255), -1)  # 빨간색(BGR: 0, 0, 255) 점 그리기

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # 프레임 크기 조정 (예: 640x480)
    frame = cv2.resize(frame, (640, 480))

    # 프레임에 점 그리기
    draw_points_on_frame(frame)

    cv2.imshow('frame', frame)
    cv2.setMouseCallback('frame', get_mouse_click)  # 마우스 콜백 설정

    key = cv2.waitKey(1)
    if key == ord('q'):  # 'q'를 누르면 종료
        break
    elif key == ord('p'):  # 'p'를 누르면 프레임 정지
        while True:  # 정지 상태 유지
            # 정지된 프레임에 점을 다시 그리기
            frame_copy = frame.copy()
            draw_points_on_frame(frame_copy)
            cv2.imshow('frame', frame_copy)  # 업데이트된 프레임 표시
            
            key = cv2.waitKey(1)
            if key == ord('p'):  # 다시 'p'를 누르면 재생
                break
            elif key == ord('q'):  # 'q'를 누르면 종료
                cap.release()
                cv2.destroyAllWindows()
                exit()
            elif key == ord('d'):  # 'd'를 누르면 마지막 점 삭제
                if points:
                    deleted_point = points.pop()
                    print(f"최근 점 삭제: ({deleted_point[0]}, {deleted_point[1]})")
            elif key != 255:  # 다른 키 입력 처리
                cv2.setMouseCallback('frame', get_mouse_click)

cap.release()
cv2.destroyAllWindows()
