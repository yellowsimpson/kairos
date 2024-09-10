#사진 한장넣으면 500장으로 만들어주는 코드
#####################################################################
from PIL import Image, ImageEnhance, ImageOps
import random
import os

# 이미지 증강 함수
def augment_image(img, save_path, index):
    # 1. 회전 (Random Rotation) 및 배경을 흰색으로 설정
    angle = random.randint(-30, 30)
    img = img.convert("RGBA")  # 투명한 배경을 허용하기 위해 RGBA로 변환
    rotated_img = img.rotate(angle, expand=True)

    # 회전 후 투명한 부분을 흰색으로 채우기
    background = Image.new("RGBA", rotated_img.size, (255, 255, 255, 255))  # 흰색 배경
    img = Image.alpha_composite(background, rotated_img)  # 투명 배경을 흰색으로 합성
    img = img.convert("RGB")  # JPEG 형식 저장을 위해 다시 RGB로 변환

    # 2. 좌우 및 상하 뒤집기 (Random Flip)
    if random.choice([True, False]):
        img = ImageOps.mirror(img)  # 좌우 뒤집기
    if random.choice([True, False]):
        img = ImageOps.flip(img)  # 상하 뒤집기

    # 3. 크기 조정 및 자르기 (Random Crop & Rescale)
    width, height = img.size
    crop_size = random.uniform(0.8, 1.0)
    new_width, new_height = int(width * crop_size), int(height * crop_size)
    img = img.resize((new_width, new_height), Image.ANTIALIAS)
    
    # 중앙으로 다시 자르기
    img = ImageOps.fit(img, (width, height), Image.ANTIALIAS)
    
    # 4. 밝기 및 대비 변화 (Brightness & Contrast)
    enhancer = ImageEnhance.Brightness(img)
    img = enhancer.enhance(random.uniform(0.7, 1.3))  # 밝기 조절

    enhancer = ImageEnhance.Contrast(img)
    img = enhancer.enhance(random.uniform(0.7, 1.3))  # 대비 조절

    # 5. 색상 변화 (Color)
    enhancer = ImageEnhance.Color(img)
    img = enhancer.enhance(random.uniform(0.7, 1.3))  # 색상 조절

    # 6. 노이즈 추가 (Simple noise using effect_spread)
    img = img.effect_spread(random.randint(1, 5))  # 노이즈 추가

    # 저장
    img.save(os.path.join(save_path, f"red_{index}.jpg"))  

# 이미지 증강을 위한 메인 함수
def generate_augmented_images(image_path, save_path, num_images):
    # 폴더 생성
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    # 원본 이미지 로드
    img = Image.open(image_path)

    # num_images 만큼 이미지 생성
    for i in range(1, num_images + 1):
        augment_image(img, save_path, i)
        print(f"Image {i} saved.")

# 사용 예시
image_path = "red_original.jpg"  # 원본 이미지 경로
save_path = "red"  # 증강된 이미지 저장 경로
num_images = 1000  # 생성할 이미지 수

generate_augmented_images(image_path, save_path, num_images)