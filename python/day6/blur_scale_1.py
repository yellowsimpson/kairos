import cv2
def callback1(value):
    pass
def callback2(value):
    pass
# 트랙바 설정
cv2.namedWindow('style', cv2.WINDOW_NORMAL)
cv2.createTrackbar('sigma_s', 'style', 10, 300, callback1)
cv2.createTrackbar('sigma_r', 'style', 0, 100, callback2)
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    # 트랙바에서 sigma_s와 sigma_r 값 가져오기
    sigma_s = cv2.getTrackbarPos('sigma_s', 'style')
    sigma_r = cv2.getTrackbarPos('sigma_r', 'style') / 100.0
    # 영상에 스타일 적용
    frame_style = cv2.stylization(frame, sigma_s=sigma_s, sigma_r=sigma_r)
    # 원본 영상과 스타일 적용된 영상 보여주기
    cv2.imshow("Original Video", frame)
    cv2.imshow("Styled Video", frame_style)
    # 'q' 키를 누르면 종료
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
# 카메라와 모든 윈도우 해제
cap.release()
cv2.destroyAllWindows()






