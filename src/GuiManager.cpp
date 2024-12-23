#include "../include/managers/GuiManager.hpp"

GuiManager::GuiManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef)
{

    clientLen = sizeof(clientAddr);
}

GuiManager::~GuiManager()
{
    close(sockfd);
}

void GuiManager::initializeServer()
{
    // 소켓 생성
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 서버 주소 설정
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(8888);

    // 소켓에 주소 바인딩
    if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << PORT << std::endl;
}

bool GuiManager::send()
{
    // 클라이언트에게 데이터 전송
    ssize_t sendBytes = sendto(sockfd, &sendMessage, sizeof(sendMessage), 0, (struct sockaddr *)&clientAddr, clientLen);

    if (sendBytes == -1)
    {
        perror("sendto failed");
        return false;
    }
    return true;
}

bool GuiManager::receive()
{
    // 클라이언트로부터 데이터 수신
    int n = recvfrom(sockfd, &receiveMessage, sizeof(receiveMessage), 0, (struct sockaddr *)&clientAddr, &clientLen);
    if (n < 0)
    {
        std::cerr << "Error receiving data from client" << std::endl;
        return false;
    }
    else
    {
        std::cout << "Received message from client:" << std::endl;
        std::cout << "stateDemand: " << receiveMessage.stateDemand << std::endl;
        std::cout << "shutdown: " << receiveMessage.shutdown << std::endl;
        std::cout << "allHome: " << receiveMessage.allHome << std::endl;
        std::cout << "axisHome: " << receiveMessage.axisHome << std::endl;
        std::cout << "musicPath: " << receiveMessage.musicPath << std::endl;
    }

    // 받은 메시지를 출력

    return true;
}

void GuiManager::guiThread()
{
    bool testInit = false;

    while (state.main != Main::Shutdown)
    {
        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            usleep(50000);
            break;
        }
        case Main::Ideal:
        {
            if (!testInit)
            {
                initializeServer();
                testInit = true;
            }
            if (!guiConnected)
            { // 연결이 안된 상태일 때
                if (receive())
                { // 연결 시도
                    guiConnected = true;
                }
                else
                {
                    usleep(100000); // 10ms
                }
            }
            else // 연결이 된 상태
            {
                if (receive())
                {
                    // send();
                    usleep(3000000); // 300ms
                }
                else
                {
                    guiConnected = false;
                }
            }
            usleep(100000);
            break;
        }
        // case Main::Perform:
        // {
        //     break;
        // }
        case Main::Play:
        {
            break;
        }
        case Main::AddStance:
        {
            break;
        }
        case Main::Test:
        {
            break;
        }
        case Main::Pause:
        {
            break;
        }
        case Main::Error:
        {
            break;
        }
        case Main::Shutdown:
            break;
        }
    }
}
