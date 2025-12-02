#pragma once

#include <stdlib.h>
#include <iostream>
#include "../USBIO_advantech/compatibility.h"
#include "../USBIO_advantech/bdaqctrl.h"

typedef unsigned char byte;

#define  deviceDescription  L"USB-4761,BID#0"

class USBIO
{
public:
    USBIO();

    ~USBIO();

    void initUSBIO4761();
    bool outputUSBIO4761();
    void setUSBIO4761(int num, bool state);
    void exitUSBIO4761();

    bool useUSBIO = true;

private:
    const wchar_t* profilePath = L"../../../profile/DemoDevice.xml";

    Automation::BDaq::ErrorCode ret;// = Success;
    Automation::BDaq::InstantDoCtrl *instantDoCtrl = Automation::BDaq::InstantDoCtrl::Create();

    Automation::BDaq::uint8 bufferForWriting[1] = {0x00};
};

