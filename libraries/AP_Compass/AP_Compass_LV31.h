#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend_Serial.h"

#define LV31_ID 21048

class AP_Compass_LV31 : public AP_Compass_Backend_Serial
{

public:

    // using AP_Compass_Backend_Serial::AP_Compass_Backend_Serial;
    
    //constructor
    AP_Compass_LV31(uint8_t serial_instance);
private:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }
    // the value 0 is special to the UARTDriver - it's "use default"
    uint16_t rx_bufsize() const override{ return 0; }
    uint16_t tx_bufsize() const override{ return 0; }
    // read
    void read(void) override;

    // find signature byte in buffer starting at start, moving that
    // byte and following bytes to start of buffer.
    bool find_signature_in_buffer(uint8_t start);
    void crc16(uint8_t & ck_a,uint8_t & ck_b);

    uint8_t buffer[14];
    uint8_t buffer_used;
    uint8_t _compass_instance;
};
