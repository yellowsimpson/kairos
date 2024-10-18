import cv2

# 외부 USB 카메라 열기 (인덱스 1 사용)
cap = cv2.VideoCapture(1)

# 카메라가 정상적으로 열렸는지 확인
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

photo_count = 1  # 사진 저장을 위한 카운터 변수

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break
    
    # 프레임을 화면에 표시
    cv2.imshow("USB Camera", frame)

    # 키 입력 대기 (1ms)
    key = cv2.waitKey(1) & 0xFF
    
    # 'q' 키를 누르면 종료
    if key == ord('q'):
        break
    # 's' 키를 누르면 사진 저장
    elif key == ord('s'):
        filename = f"C:\\Users\\shims\\Desktop\\github\\kairos\\photo\\photo_{photo_count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"사진 저장: {filename}")
        photo_count += 1

# 카메라와 윈도우 해제
cap.release()
cv2.destroyAllWindows()
