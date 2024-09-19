#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <cstdlib>  // rand()와 srand()를 사용하기 위한 헤더 파일
#include <ctime>    // 시간 기반 시드를 사용하기 위한 헤더 파일
#include <iostream>

int main() {
    // 창 크기 설정
    const int windowWidth = 640;
    const int windowHeight = 480;

    // SFML 창 생성
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Dodge the Poop!");

    // 플레이어 캐릭터 이미지 로드
    sf::Texture playerTexture;
    if (!playerTexture.loadFromFile("doodugi.png")) 
    {
        std::cout << " 파일 x" << std::endl;
        return -1; // 이미지 로드 실패 시 종료
    }

    // 플레이어 스프라이트 생성
    sf::Sprite playerSprite;
    playerSprite.setTexture(playerTexture);
    playerSprite.setScale(0.3f, 0.3f);  // 이미지 크기 조정
    playerSprite.setPosition(windowWidth / 2 - playerSprite.getGlobalBounds().width / 2, windowHeight  - playerSprite.getGlobalBounds().height - 10);
    //처음 캐릭터 위치 설정
    //windowWidth / 2 - playerSprite.getGlobalBounds().width / 1 -> x축 설정
    //windowHeight - playerSprite.getGlobalBounds().height - 10 -> y축 설정


    // 똥 이미지 로드
    sf::Texture poopTexture;
    if (!poopTexture.loadFromFile("ddong.png")) 
    {

        std::cout << " 파일2 x" << std::endl;
        return -1; // 이미지 로드 실패 시 종료
    }

    // 똥 스프라이트 생성
    sf::Sprite poopSprite;
    poopSprite.setTexture(poopTexture);
    poopSprite.setScale(0.3f, 0.3f);  // 이미지 크기 조정
    poopSprite.setPosition(rand() % (windowWidth - static_cast<int>(poopSprite.getGlobalBounds().width)), 0);
    //(windowWidth -> x축의 전체 길이 0 -> 처음 y축 위치

    // 똥의 속도
    float poopSpeed = 1000.0f;

    // 플레이어의 속도
    float playerSpeed = 300.0f;

    // 시간 측정을 위한 시계
    sf::Clock clock;

    // 난수 초기화
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // 방귀 소리 로드
    sf::SoundBuffer fartBuffer;
    if (!fartBuffer.loadFromFile("water.mp3")) 
    {

        std::cout << " 소리 x" << std::endl;
        return -1; // 소리 로드 실패 시 종료
    }
    sf::Sound fartSound;
    fartSound.setBuffer(fartBuffer);

    // 게임 루프
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // 경과 시간 계산
        float deltaTime = clock.restart().asSeconds();

        // 플레이어 움직임
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left) && playerSprite.getPosition().x > 0) {
            playerSprite.move(-playerSpeed * deltaTime, 0); // 왼쪽으로 이동
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right) && playerSprite.getPosition().x + playerSprite.getGlobalBounds().width < windowWidth) {
            playerSprite.move(playerSpeed * deltaTime, 0);  // 오른쪽으로 이동
        }

        // 똥 이미지 이동
        poopSprite.move(0, poopSpeed * deltaTime);

        // 똥이 화면 아래로 사라지면 새 위치에서 다시 떨어지기
        if (poopSprite.getPosition().y > windowHeight) {
            poopSprite.setPosition(rand() % (windowWidth - static_cast<int>(poopSprite.getGlobalBounds().width)), 0);
            // 똥이 다시 떨어질 때마다 속도를 증가시킴
            poopSpeed += 20.0f; // 속도 증가 값
        }

        // 충돌 감지 (똥과 플레이어 스프라이트 간의 충돌 체크)
        if (poopSprite.getGlobalBounds().intersects(playerSprite.getGlobalBounds())) {
            // 충돌 시 게임 종료
            fartSound.play(); // 방귀 소리 재생
            //window.close();
        }

        // 창을 검은색으로 지우기
        window.clear(sf::Color::Black);

        // 도형 및 스프라이트 그리기
        window.draw(playerSprite);
        window.draw(poopSprite);

        // 그린 내용을 화면에 표시
        window.display();
    }

    return 0;
}
