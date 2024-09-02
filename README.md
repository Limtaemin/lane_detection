- [ ]  고국원 교수님 20.04 Ntrip 파이썬 코드 다운받은 뒤 실행해보기(0901 확인하고 다시 올려주신다함)
- [ ]  깃허브 활용 공부하기
- [ ]  같은 지점에서 Gps Fix 되었을 때 원반경 내의 값 정확하게 유지되는지 체크하기
- [ ]  새로운 뎁스 카메라로 라인 트래킹 및 기존 카메라와 화각 비교
→ 기존 카메라보다 화각이 넓다면 기존 USB_Cam으로 Yolo 진행
- [ ]  YOLO 공부하기 → 재형이형 노션 흔적 따라가기
- [ ]  카메라마다 버드뷰하는거 정확히 정리해두기
- [ ]  기존 카메라 및 새로운 카메라 적용방법 및 특징, 토픽 정보 등 관련사항 정리하기
- [ ]  차 재조립해서 웨이포인트 따라가는지 확인하기(헤딩값을 MBC가아니라 2-way 헤딩으로)
---

<img src="https://capsule-render.vercel.app/api?type=slice&color=gradient&height=300&section=header&text=파일구조및코드설명&fontSize=20" />
<img src="https://capsule-render.vercel.app/api?type=slice&color=gradient&height=100&section=footer&text=Lane_Detection_Sliding_Windows&fontSize=16" />


파일 구조 및 코드 설명
* Lane_Detection_Sliding_Windows
  ** lane_detect.txt
    - 사용된 cv2 / numpy 함수 기본적인 설명 
  ** LaneDetection.py
    - 상세한 주석이 달려있는 슬라이딩윈도우 기본 코드(여기서부터 시작)
    - 버드뷰, 히스토그램, 슬라이딩 윈도우
  ** LaneDetection_contour.py
    - 최종 수정본 코드(노이즈제거 포함)
    - 버드뷰, 히스토그램, 곡률계산(정확한 곡률인지는 모름), 휠베이스와 곡률을 통한 조향각도결정, 슬라이딩윈도우, 가우시안블러, 첫번째 윈도우 중심계산, 인식안될시 이전값유지, 한쪽만 인식되면 오프셋으로 결정, 컨투어 필터링, 곡률 및 각도/중앙과의 오프셋 시각화, 조향각도 토픽발행, 영상이 끝나면 영상 다시 재생
  ** LaneDetection_ros.py
    - 잘 작동안함
  ** Video_pixel.py
    - 비디오파일에서 정지한 뒤, 원하는 픽셀을 찍어서 버드뷰 좌표를 얻는 코드 / 정지: p, 직전 점 삭제: d
  ** camera_pixel.py
    - 카메라 캠에서도 작동하는 코드
[MP4_Files]
