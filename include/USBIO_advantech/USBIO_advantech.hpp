#pragma once

#include <stdlib.h>
#include <iostream>
#include "../USBIO_advantech/compatibility.h"
#include "../USBIO_advantech/bdaqctrl.h"

using namespace Automation::BDaq;

typedef unsigned char byte;

#define  deviceDescription  L"USB-4761,BID#0"

class USBIO
{
public:
    USBIO();

    ~USBIO();

    void USBIO_4761_init();
    bool USBIO_4761_output();
    void USBIO_4761_set(int num, bool state);
    void USBIO_4761_exit();

    bool useUSBIO = true;

private:
    const wchar_t* profilePath = L"../../../profile/DemoDevice.xml";

    ErrorCode ret;// = Success;
    InstantDoCtrl *instantDoCtrl = InstantDoCtrl::Create();

    uint8 bufferForWriting[1] = {0x00};
};

