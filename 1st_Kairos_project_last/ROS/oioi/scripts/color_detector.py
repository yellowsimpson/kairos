import cv2
import numpy as np

class ColorObjectDetector:
    def __init__(self):
        pass

    def find_color_in_image(self, image, *colors):
        # 이미지를 HSV 형식으로 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 색상 범위 정의
        color_ranges = {
            # 'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
            # 'blue': (np.array([100, 100, 50]), np.array([130, 255, 255])),
            'red': (np.array([170, 100, 100]), np.array([179, 255, 255])),
            # 'green':(np.array([40, 100, 100]), np.array([80, 255, 255])),
            # # 'purple': (np.array([130, 50, 50]), np.array([170, 255, 255])),
        }

        max_area = 0
        max_contour = None
        max_color = None

        # 'random'이 colors에 포함되어 있으면 해당 인수를 제거
        colors = [color for color in colors if color != 'random']

        # 인자로 전달된 색상이 있으면 해당 색상만 검사
        if colors:
            for color_name in colors:
                color_range = color_ranges.get(color_name)
                if color_range is not None:
                    lower, upper = color_range
                    mask = cv2.inRange(hsv, lower, upper)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        contour = max(contours, key=cv2.contourArea)
                        contour_area = cv2.contourArea(contour)
                        if contour_area > max_area:
                            max_area = contour_area
                            max_contour = contour
                            max_color = color_name
        else:
            # 인자로 전달된 색상이 없으면 모든 색상을 검사
            for color_name, color_range in color_ranges.items():
                lower, upper = color_range
                mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    contour = max(contours, key=cv2.contourArea)
                    contour_area = cv2.contourArea(contour)
                    if contour_area > max_area:
                        max_area = contour_area
                        max_contour = contour
                        max_color = color_name

        # 가장 큰 컨투어가 존재하는 경우
        if max_contour is not None:
            # 외곽선을 감싸는 최소 사각형 구하기
            rect = cv2.minAreaRect(max_contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)

            # 회전된 네모 박스 그리기
            cv2.drawContours(image, [box], 0, (255, 0, 255), 2)

            # 중심 좌표 계산
            center_x = int(rect[0][0])
            center_y = int(rect[0][1])

            length = np.linalg.norm(box[0] - box[1])

            # 카메라 픽셀 좌표의 중심
            camera_pixel_center_x = 640 // 2
            camera_pixel_center_y = 480 // 2

            # 중심 좌표와 카메라 픽셀 좌표의 중심 간 거리 계산
            distance_x = -(center_x - camera_pixel_center_x) * 0.05 / length
            distance_y = -(center_y - camera_pixel_center_y) * 0.05 / length

            # 물체가 돌아간 각도 계산
            angle_msg = int(rect[2])
            cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)

            return distance_x, distance_y, angle_msg, image, max_color

        # 색상을 찾지 못한 경우
        return 0, 0, 0, image, "no found color"

def color_detector(*colors, img_print=False):
    try:
        detector = ColorObjectDetector()

        # 'random'이 colors에 포함되어 있으면 해당 인수를 제거
        colors = [color for color in colors if color != 'random']

        # 웹캠에서 실시간으로 캡처
        cap = cv2.VideoCapture(0)
        # 해상도 설정
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        ret, frame = cap.read()
        if not ret:
            print("비디오 스트림에서 프레임을 읽을 수 없습니다.")
            return

        result = detector.find_color_in_image(frame, *colors)
        if result is not None:
            distance_x, distance_y, angle, image, found_color = result
            print("Color found! Distance X:", distance_x, "Distance Y:", distance_y, "Angle:", angle, "Color:", found_color)
            # 이미지를 보여줌
            if img_print:
                cv2.imshow('Detected Image', image)
                cv2.waitKey(0)  # 키 입력을 기다림
                cv2.destroyAllWindows()  # 창을 닫음
            return distance_x, distance_y, angle, found_color

    except KeyboardInterrupt:
        pass

# 코드 실행
if __name__ == "__main__":
    color1 = 'random'
    color_detector(color1, img_print=True)
