
#include "../include/USBIO_advantech/USBIO_advantech.hpp"

#include <iostream>

USBIO::USBIO() { 

    std::cout << "[WARN] USBIO Dummy Mode." << std::endl; 

}

USBIO::~USBIO() {}


void USBIO::setUSBIO4761(int num, bool state) {}


// 에러 났던 함수 강제 생성

bool USBIO::outputUSBIO4761() {
    return true;
}

void USBIO::initUSBIO4761() {
}

void USBIO::exitUSBIO4761() {
}

