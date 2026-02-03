#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string>

struct ClientToServerMessage {
    int stateDemand;
    int shutdown;
    int allHome;
    int axisHome;
    char musicPath[256]; 
};

struct ServerToClientMessage {
    int mainState;
    bool can1Status;
    bool can2Status;
    bool can3Status;

    struct LimbStatus {
        float posDes;
        float vel;
        float cur;
        bool status;
        bool homeDone;
    };

    LimbStatus L1;
    LimbStatus L2;
    LimbStatus L3;
    LimbStatus LW;
    LimbStatus R1;
    LimbStatus R2;
    LimbStatus R3;
    LimbStatus RW;
    LimbStatus Waist;
};

#endif // PROTOCOL_H
