/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Compass_LV31.h"
#include <ctype.h>

extern const AP_HAL::HAL& hal;

AP_Compass_LV31::AP_Compass_LV31(uint8_t serial_instance):AP_Compass_Backend_Serial(serial_instance)
{
    if(!register_compass(LV31_ID,_compass_instance)){
        hal.console->printf("Can't register compass\n");
    }else
    {
        set_dev_id(_compass_instance,LV31_ID);
        //
        // save_dev_id(_compass_instance);
        //
        set_rotation(_compass_instance,ROTATION_NONE);
        set_external(_compass_instance,true);
    }
    
}
bool AP_Compass_LV31::find_signature_in_buffer(uint8_t start)
{
    for (uint8_t i=start; i<buffer_used; i++) {
        if (buffer[i] == 0xB5) {
            memmove(&buffer[0], &buffer[i], buffer_used-i);
            buffer_used -= i;
            return true;
        }
    }
    // header byte not in buffer
    buffer_used = 0;
    return false;
}
void AP_Compass_LV31::crc16(uint8_t & ck_a,uint8_t & ck_b)
{
    ck_a = 0; ck_b = 0;
    for (uint8_t i = 2; i < 12; i++)
    {
        ck_a += buffer[i];
        ck_b += ck_a;
    }
    ck_a &= 0xFF;
    ck_b &= 0xFF;
}
// get_reading - read a value from the sensor
void AP_Compass_LV31::read(void)
{
    if (uart == nullptr) {
        return;
    }
    // hal.console->printf("Bo vao duoc roi nhe \n");
    // int16_t nbytes = uart->available();
    // hal.console->printf("number byte available:%d\n",nbytes);
    const uint8_t num_read = uart->read(&buffer[buffer_used], ARRAY_SIZE(buffer)-buffer_used);
    buffer_used += num_read;

    if (buffer_used == 0) {
        return;
    }

    if (buffer[0] != 0xB5 && buffer[1] != 0x62 &&
        !find_signature_in_buffer(1)) {
        return;
    }

    if (buffer_used < ARRAY_SIZE(buffer)) {
        return;
    }
    // hal.console->printf("So phan tu nhan duoc %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n"
    //                     ,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]
    //                     ,buffer[7],buffer[8],buffer[9],buffer[10],buffer[11],buffer[12],buffer[13]);
    uint8_t ck_a = 0, ck_b = 0;
    crc16(ck_a,ck_b);
    if (buffer[12] != ck_a || buffer[13] != ck_b) {
        find_signature_in_buffer(1);
        return;
    }
    int16_t _mag_x,_mag_y,_mag_z;
    _mag_x = buffer[6]<<8 | buffer[7];
    _mag_y = buffer[8]<<8 | buffer[9];
    _mag_z = buffer[10]<<8 | buffer[11];
    Vector3f raw_field = Vector3f(_mag_x, _mag_y, _mag_z);

    // hal.console->printf("mX:%d mY:%d mZ:%d\n",_mag_x,_mag_y,_mag_z);
    //
    accumulate_sample(raw_field,_compass_instance);
    //
    drain_accumulated_samples(_compass_instance);
    //
    buffer_used = 0;
    uart->discard_input();
}
