#include "../include/USBIO_advantech/USBIO_advantech.hpp"

USBIO::USBIO()
{
}

USBIO::~USBIO()
{
}

void USBIO::USBIO_4761_init()
{
    useUSBIO = true;

    DeviceInformation devInfo(deviceDescription);

    do
    {
        ret = instantDoCtrl->setSelectedDevice(devInfo);
        CHK_RESULT(ret);
        ret = instantDoCtrl->LoadProfile(profilePath);//Loads a profile to initialize the device.
        CHK_RESULT(ret);
    } while (false);

    if(BioFailed(ret))
    {
        wchar_t enumString[256];
        AdxEnumToString(L"ErrorCode", (int32)ret, 256, enumString);
        std::cout << "Some error occurred. And the last error code is 0x" << ret << ". [" << enumString << "]\n";

        useUSBIO = false;
    }
    else
    {
        std::cout << "USBIO-4761 init\n";
    }
}

void USBIO::USBIO_4761_set(int num, bool state)
{
    uint8 current_output = bufferForWriting[0];

    if (state)
    {
        current_output |= 1<<num;
    }
    else
    {
        current_output &= ~(1<<num);
    }

    bufferForWriting[0] = current_output;
}

bool USBIO::USBIO_4761_output()
{
    do
    {
        ret = instantDoCtrl->Write(0, 1, bufferForWriting);
        CHK_RESULT(ret);
    } while (false);
    

    if(BioFailed(ret))
    {
        wchar_t enumString[256];
        AdxEnumToString(L"ErrorCode", (int32)ret, 256, enumString);
        std::cout << "Some error occurred. And the last error code is 0x" << ret << ". [" << enumString << "]\n";

        return false;
    }
    else
    {
        return true;
    }
}

void USBIO::USBIO_4761_exit()
{
    instantDoCtrl->Dispose();
}

