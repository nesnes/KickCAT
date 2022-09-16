#ifndef CAN_ELMO_ERROR_H
#define CAN_ELMO_ERROR_H

#include "kickcat/protocol.h"


struct EmergencyError
{
    uint16_t code;
    char const* desc{""};
};

namespace can
{
    namespace emergency
    {
        EmergencyError constexpr EMERGENCY_0000    {0x0000,  "Error Reset or No Error"};
        EmergencyError constexpr EMERGENCY_1000    {0x1000,  "Generic Error"};
        EmergencyError constexpr EMERGENCY_2000    {0x2000,  "Current"};
        EmergencyError constexpr EMERGENCY_2100    {0x2100,  "Current device input side"};
        EmergencyError constexpr EMERGENCY_2200    {0x2200,  "Current inside the device"};
        EmergencyError constexpr EMERGENCY_2300    {0x2300,  "Current device output side"};
        EmergencyError constexpr EMERGENCY_3000    {0x3000,  "Voltage"};
        EmergencyError constexpr EMERGENCY_3100    {0x3100,  "Mains Voltage"};
        EmergencyError constexpr EMERGENCY_3200    {0x3200, 	"Voltage inside the device"};
        EmergencyError constexpr EMERGENCY_3300    {0x3300, 	"Output Voltage"};
        EmergencyError constexpr EMERGENCY_4000    {0x4000, 	"Temperature"};
        EmergencyError constexpr EMERGENCY_4100    {0x4100, 	"Ambient Temperature"};
        EmergencyError constexpr EMERGENCY_4200    {0x4200, 	"Device Temperature"};
        EmergencyError constexpr EMERGENCY_5000    {0x5000, 	"Device Hardware"};
        EmergencyError constexpr EMERGENCY_6000    {0x6000, 	"Device Software"};
        EmergencyError constexpr EMERGENCY_6100    {0x6100, 	"Internal Software"};
        EmergencyError constexpr EMERGENCY_6200    {0x6200, 	"User Software"};
        EmergencyError constexpr EMERGENCY_6300    {0x6300, 	"Data Set"};
        EmergencyError constexpr EMERGENCY_7000    {0x7000, 	"Additional Modules"};
        EmergencyError constexpr EMERGENCY_8000    {0x8000, 	"Monitoring"};
        EmergencyError constexpr EMERGENCY_8100    {0x8100, 	"Communication"};
        EmergencyError constexpr EMERGENCY_8110    {0x8110, 	"CAN Overrun (Objects lost)"};
        EmergencyError constexpr EMERGENCY_8120    {0x8120, 	"CAN in Error Passive Mode"};
        EmergencyError constexpr EMERGENCY_8130    {0x8130, 	"Life Guard Error or Heartbeat Error"};
        EmergencyError constexpr EMERGENCY_8140    {0x8140, 	"recovered from bus off"};
        EmergencyError constexpr EMERGENCY_8150    {0x8150, 	"CAN-ID collision"};
        EmergencyError constexpr EMERGENCY_8200    {0x8200, 	"Protocol Error"};
        EmergencyError constexpr EMERGENCY_8210    {0x8210, 	"PDO not processed due to length error"};
        EmergencyError constexpr EMERGENCY_8220    {0x8220, 	"PDO length exceeded"};
        EmergencyError constexpr EMERGENCY_8230    {0x8230, 	"DAM MPDO not processed destination object not available"};
        EmergencyError constexpr EMERGENCY_8240    {0x8240, 	"Unexpected SYNC data length"};
        EmergencyError constexpr EMERGENCY_8250    {0x8250, 	"RPDO timeout"};
        EmergencyError constexpr EMERGENCY_9000    {0x9000, 	"External Error"};
        EmergencyError constexpr EMERGENCY_F000    {0xF000, 	"Additional Functions"};
        EmergencyError constexpr EMERGENCY_FF00    {0xFF00, 	"Device specific"};

        EmergencyError codeToError(uint16_t const& code)
        {
            uint8_t head = code/0x0100;
            uint8_t tail = code%0x0100;

            switch (head)
            {
                case 0x00: {return EMERGENCY_0000;};
                case 0x10: {return EMERGENCY_1000;};
                case 0x20: {return EMERGENCY_2000;};
                case 0x21: {return EMERGENCY_2100;};
                case 0x22: {return EMERGENCY_2200;};
                case 0x23: {return EMERGENCY_2300;};
                case 0x30: {return EMERGENCY_3000;};
                case 0x31: {return EMERGENCY_3100;};
                case 0x32: {return EMERGENCY_3200;};
                case 0x33: {return EMERGENCY_3300;};
                case 0x40: {return EMERGENCY_4000;};
                case 0x41: {return EMERGENCY_4100;};
                case 0x42: {return EMERGENCY_4200;};
                case 0x50: {return EMERGENCY_5000;};
                case 0x60: {return EMERGENCY_6000;};
                case 0x61: {return EMERGENCY_6100;};
                case 0x62: {return EMERGENCY_6200;};
                case 0x63: {return EMERGENCY_6300;};
                case 0x70: {return EMERGENCY_7000;};
                case 0x80: {return EMERGENCY_8000;};
                case 0x81: 
                {
                    switch (tail)
                    {
                        case 0x10: {return EMERGENCY_8110;};
                        case 0x20: {return EMERGENCY_8120;};
                        case 0x30: {return EMERGENCY_8130;};
                        case 0x40: {return EMERGENCY_8140;};
                        case 0x50: {return EMERGENCY_8150;};
                        default: {return EMERGENCY_8100;}
                    };
                    break;
                };
                case 0x82:
                {
                    switch (tail)
                    {
                        case 0x10: {return EMERGENCY_8210;};
                        case 0x20: {return EMERGENCY_8220;};
                        case 0x30: {return EMERGENCY_8230;};
                        case 0x40: {return EMERGENCY_8240;};
                        case 0x50: {return EMERGENCY_8250;};
                        default: {return EMERGENCY_8200;}
                    };
                    break;
                };
                case 0x90: {return EMERGENCY_9000;};
                case 0xF0: {return EMERGENCY_F000;};
                case 0xFF: {return EMERGENCY_FF00;};
                default: {return {code, "Unknown error"};};
            }
        }
    }
}







#endif
