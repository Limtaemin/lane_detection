import cv2
import numpy as np


######################### Bird View & 트랙바 설정 #########################

# 비디오 파일 캡쳐
vidcap = cv2.VideoCapture("LaneVideo.mp4")
success, image = vidcap.read()
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

while success:
    success, image = vidcap.read()
    # 영상 사이즈 제한두기 -> 제멋대로 영상들 통일하는 느낌
    frame = cv2.resize(image, (640,480))

    # 관점 변환(버드뷰) 하기 위해 각 포인트를 저장 -> 픽셀 일치하는 영상, 카메라인지 확인해야함
    tl = (239,385)
    bl = (18,475)
    tr = (383,381)
    br = (579,478)
    # 위에서 찾은 점 보이는 점 찍기
    cv2.circle(frame, tl, 5, (0,0,255), -1)
    cv2.circle(frame, bl, 5, (0,0,255), -1)
    cv2.circle(frame, tr, 5, (0,0,255), -1)
    cv2.circle(frame, br, 5, (0,0,255), -1)

    # 관전 변환 진행 -> 간단하게 pts1을 pts2랑 일치시키는 느낌
    pts1 = np.float32([tl, bl, tr, br]) 
    pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]]) 
    
    # 변환행렬 구하기 -> 변환에 필요한 변환 정보를 담는 느낌
    matrix = cv2.getPerspectiveTransform(pts1, pts2) 
    # 기존 프레임크기로 변환행렬을 통해 버드뷰를 진행
    transformed_frame = cv2.warpPerspective(frame, matrix, (640,480))

######################### Object Detection #########################

    # Image Thresholding -> Thresholding이란 문턱이라는 뜻인데, 픽셀의 값이 급격하게 바뀌는 지점을 캐치해내는 것을 잡는 느낌
    # 색상은 대표적으로 RGB와 HSV가 있는데, HSV가 좀 더 색상 관리하기에 편하기에 HSV 로 바꾸는 코드
    # cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 이게 흑백으로 바꾸는 코드
    hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)
    
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    # 트랙바의 저장된 값을 각각 변수(lower, upper)에 저장
    lower = np.array([l_h,l_s,l_v])
    upper = np.array([u_h,u_s,u_v])
    # inrange 함수는 사이값에 해당하는 픽셀이 있을때마다 1로 저장, 없는건 0으로 저장하는 함수
    mask = cv2.inRange(hsv_transformed_frame, lower, upper)

    # 히스토그램 만들기 
    # histogram = np.sum(mask[mask.shape[0]//2 밑에서부터 위까지 절반을 나타냄 : 이건 제일 밑 나타내는거, 가로를 전부 본다는 뜻:], 열 행렬의 값을 더한다는 뜻 axis=0)
    histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
    # 히스토그램의 가로 중앙점 게산하기 histogram.shape[0] -> 이게 배열의 길이를 나타냄
    midpoint = np.int(histogram.shape[0]/2)
    # 미드 포인트 기준으로 왼쪽영역에서 가장 큰 값
    left_base = np.argmax(histogram[:midpoint])
    # 미드 포인트 기준으로 오른쪽 영역에서 가장 큰 값인데, 영역시작점이 중앙부터라 차잇값을 더해줌
    right_base = np.argmax(histogram[midpoint:]) + midpoint

##################################################################
# 중간 요약: 히스토그램을 통해 윈도우가 생성될 초기 위치를 결정 
# -> 차선이 있을 확률이 가장 높은 곳을 결정하여 거기서부터 차선인식을 시작함
##################################################################


######################### Sliding Window #########################

    y = 472
    lx = []
    rx = []

    msk = mask.copy()

    while y>0:
        ## 왼쪽 내에서 검출하기
        # 마스크(이진화된 이미지) 안에서 left 베이스 좌우 및 위 아래 의 가상의 이미지를 형성
        img = mask[y-40:y, left_base-50:left_base+50]
        # 왼쪽 영역에서 컨투어가 검출되면 저장함
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 찾은 컨투어가 웬만하면 하나밖에 안생기지만 for문 안에서 반복하는 이유는, 노이즈 등에 의해 컨투어가 2개 이상 생겼을 시 반복문 안에서 처리함으로서 에러를 막는 안전장치 역할을 수행하기 위함
        for contour in contours:
            M = cv2.moments(contour)
            # 전체 면적이 0이 아니라면
            if M["m00"] != 0:
                # 전체 면적에서 x와 y의 값을 센터로 잡고
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                # 이 값을 저장, x값만 활용 / 저장하는 이유는 추후 조향값에 대한 값들로 활용하기 위함
                lx.append(left_base-50 + cx)
                left_base = left_base-50 + cx
        
        ## 오른쪽 내에서 검출하기(왼쪽과 동일하지만 시작 위치만 조금 다름)
        img = mask[y-40:y, right_base-50:right_base+50]
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                lx.append(right_base-50 + cx)
                right_base = right_base-50 + cx
        
        # 왼쪽과 오른쪽에 대해 정사각형 그리기(윈도우 그리기)
        cv2.rectangle(msk, (left_base-50,y), (left_base+50,y-40), (255,255,255), 2)
        cv2.rectangle(msk, (right_base-50,y), (right_base+50,y-40), (255,255,255), 2)
        # 40픽셀씩 올리면서 진행
        y -= 40
        
    cv2.imshow("Original", frame)
    cv2.imshow("Bird's Eye View", transformed_frame)
    cv2.imshow("Lane Detection - Image Thresholding", mask)
    cv2.imshow("Lane Detection - Sliding Windows", msk)

    if cv2.waitKey(10) == 27:
        break
