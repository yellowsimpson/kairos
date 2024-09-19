import cv2
import numpy as np

cap = cv2.VideoCapture(0)
ret, prev_frame = cap.read()
prev_gray_frame = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    _, thresh_binary = cv2.threshold(gray_frame, 127, 255, cv2.THRESH_BINARY)
    
    # 이전 프레임과 현재 프레임 간의 차이 계산
    diff = cv2.absdiff(prev_gray_frame, gray_frame)
    
    # 차이가 30 이상인 경우 255(흰색), 그렇지 않은 경우 0(검정색)
    _, diff_thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
    
    # 레이블링 및 바운딩 박스 표시
    num_labels, labels, stats, center = cv2.connectedComponentsWithStats(diff_thresh)
    
    for i in range(1, num_labels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        
        # 면적이 특정 값 이상인 경우에만 바운딩 박스를 그림
        if area > 1000:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
    
    cv2.imshow("Frame", frame)
    cv2.imshow("Binary", diff_thresh)
    
    key = cv2.waitKey(10) & 0xff
    if key == ord('q'):
        break
    
    # 현재 프레임을 이전 프레임으로 저장
    prev_gray_frame = gray_frame

cap.release()
cv2.destroyAllWindows()

