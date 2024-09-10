import pygame
import random

WIDTH, HEIGHT = 600, 600

window = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Conner Jeong")

player = pygame.image.load('./assets/doodugi.png')
e1 = pygame.image.load('./assets/ddong.png')
e1 = pygame.transform.scale(e1, (100, 100)) 
e2 = pygame.image.load('./assets/ddong.png')
e2 = pygame.transform.scale(e2, (100, 100)) 
e3 = pygame.image.load('./assets/ddong.png')
e3 = pygame.transform.scale(e3, (100, 100)) 
e4 = pygame.image.load('./assets/ddong.png')
e4 = pygame.transform.scale(e4, (100, 100)) 
background = pygame.image.load('./assets/background-black.png')
player = pygame.transform.scale(player, (100, 150)) 

run = True  # while True:
y = HEIGHT-200
x = WIDTH // 2
e1_y = HEIGHT - 100 #random.randint(-400, 0)
e1_x = 200 #random.randint(0, HEIGHT)
e2_y = random.randint(-400, 0)
e2_x = random.randint(0, HEIGHT)
e3_y = random.randint(-400, 0)
e3_x = random.randint(0, HEIGHT)
e4_y = random.randint(-400, 0) 
e4_x = random.randint(0, HEIGHT)

flag = True

clock = pygame.time.Clock()

while run:
    
    dt = clock.tick(60)
    # 파이게임 종료 여부 체크
    for event in pygame.event.get():  # 화면 x 클릭하면 종료
        if event.type == pygame.QUIT:
            run = False
    # 키보드 입력 체크 
    key = pygame.key.get_pressed()
    if key[pygame.K_LEFT]:
        x -= 5
        if x < 0:
            x = 0
    elif key[pygame.K_RIGHT]:
        x += 5
        if x > WIDTH-100:
            x = WIDTH-100 
    # 적군 움직이기
    e1_y += 5
    e2_y += 5
    e3_y += 5
    e4_y += 5
    if e1_y > HEIGHT:
        e1_x = random.randint(0, HEIGHT)
        e1_y = random.randint(-400, 0)
    if e2_y > HEIGHT:
        e2_x = random.randint(0, HEIGHT)
        e2_y = random.randint(-400, 0)
    if e3_y > HEIGHT:
        e3_x = random.randint(0, HEIGHT)
        e3_y = random.randint(-400, 0)
    if e4_y > HEIGHT:
        e4_x = random.randint(0, HEIGHT)
        e4_y = random.randint(-400, 0)
            
    #window.blit(background_scaled, (0, 0))
    window.fill((255, 255, 255))
    window.blit(player, (x, y))
    
    '''rect1 = player.get_rect(center=(x,y))
    rect2 = e1.get_rect(center=(e1_x,e1_y))
    if rect1.colliderect(rect2):
        print('collide')
    else:
        print('not')
        window.blit(e1, (e1_x, e1_y)) 
    '''
    if e1_x - x > 100:
        print('NOT')
    else:
        print('collide')
    window.blit(e1, (e1_x, e1_y)) 
    window.blit(e2, (e2_x, e2_y))
    window.blit(e3, (e3_x, e3_y))
    window.blit(e4, (e4_x, e4_y))
    pygame.display.update()
pygame.quit()