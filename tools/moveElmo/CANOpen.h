///////////////////////////////////////////////////////////////////////////////
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////
#ifndef WDC_CANOPEN_H
#define WDC_CANOPEN_H

#include <cstdint>
#include <string>
#include <bitset>

#define U16_REVERSE(v) ((uint8_t const)v<<8 | v>>8) /* 0x1234 to 0x3412 */
namespace CANOpen
{
    enum class State
    {
        OFF,
        SAFE_RESET,
        PREPARE_TO_SWITCH_ON,
        SWITCH_ON,
        ON,
        FAULT
    };

    enum class Command
    {
        NONE,
        ENABLE,
        DISABLE,
        SET_POSITION
    };

    namespace masks
    {
        // Status bits
        uint16_t const STATUS_MASK           = 0x7F;
        uint16_t const READY_TO_SWITCH_ON    = 1U << 0;
        uint16_t const SWITCHED_ON           = 1U << 1;
        uint16_t const OPERATION_ENABLE      = 1U << 2;
        uint16_t const FAULT_MODE            = 1U << 3;
        uint16_t const VOLTAGE_ENABLED       = 1U << 4;
        uint16_t const QUICK_STOP            = 1U << 5;
        uint16_t const SWITCH_ON_DISABLED    = 1U << 6;
        uint16_t const WARNING               = 1U << 7;
        uint16_t const REMOTE                = 1U << 9;
        uint16_t const TARGET_REACHED        = 1U << 10;
        uint16_t const INTERNAL_LIMIT_ACTIVE = 1U << 11;
        uint16_t const SETPOINT_ACKNOWLEDGE  = 1U << 12;
    }

    namespace status
    {
        /// \brief Some status word values corresponding to CANOpen states.
        uint16_t const OFF_STATE =  masks::SWITCH_ON_DISABLED;
        uint16_t const READY_TO_SWITCH_ON_STATE = masks::VOLTAGE_ENABLED | masks::READY_TO_SWITCH_ON;
        uint16_t const ON_STATE = masks::QUICK_STOP | masks::VOLTAGE_ENABLED | masks::OPERATION_ENABLE | masks::SWITCHED_ON | masks::READY_TO_SWITCH_ON;
        uint16_t const POSITION_STATE = masks::QUICK_STOP | masks::VOLTAGE_ENABLED | masks::OPERATION_ENABLE | masks::SWITCHED_ON | masks::READY_TO_SWITCH_ON | masks::SETPOINT_ACKNOWLEDGE;
        uint16_t const DISABLED_STATE = masks::QUICK_STOP | masks::VOLTAGE_ENABLED | masks::SWITCHED_ON | masks::READY_TO_SWITCH_ON;
        uint16_t const FAULT_STATE = masks::FAULT_MODE;
    }

    namespace control
    {
        namespace word
        {
            uint16_t const SHUTDOWN                       = 0x0006U;
            uint16_t const SWITCH_ON_OR_DISABLE_OPERATION = 0x0007U;
            uint16_t const ENABLE_OPERATION               = 0x000FU;
            uint16_t const FAULT_RESET                    = 0x0080U;
            uint16_t const DISABLE_VOLTAGE                = 0x0000U;
            uint16_t const QUICK_STOP                     = 0x0002U;
            uint16_t const SET_ABS_POINT_NOBLEND          = 0x001FU;
            uint16_t const SET_POINT_RESET                = 0x000FU;
        }

        // controlmode stores the available CAN control mode (also called "Mode of Operation")
        // For more infos, see "Docs/CANopen DS402.pdf", page 37
        enum Mode
        {
            NO_MODE             = -1, // Default value to mean "error occurred"
            POSITION_PROFILE    = 1,
            VELOCITY_PROFILE    = 3,
            TORQUE_PROFILE      = 4,
            HOMING              = 6,
            POSITION_CYCLIC     = 8,
            VELOCITY_CYCLIC     = 9,
            TORQUE_CYCLIC       = 10,
            POSITION_DIRECT     = 102,
            VELOCITY_DIRECT     = 101,
            TORQUE_DIRECT       = 100
        };

        bool isTorque(Mode m);
        bool isVelocity(Mode m);
        bool isPosition(Mode m);

    }

    struct PDOMappingStruct {
        uint8_t bitLength;
        uint8_t subIndex;
        uint16_t index;
    }__attribute__((packed));
    
    namespace pdo
    {
        namespace motor
        {
            namespace measures
            {
                //                          Name                            Size Subindex Index
                constexpr PDOMappingStruct TPDO_STATUS_WORD =                {16, 0, 0x6041  };
                constexpr PDOMappingStruct TPDO_MODE_OF_OPERATION_DISPLAY =  {8, 0, 0x6061  };
                constexpr PDOMappingStruct TPDO_POSITION_DEMAND_VALUE =      {32, 0, 0x6062  };
                constexpr PDOMappingStruct TPDO_POSITION_ACTUAL_VALUE =      {32, 0, 0x6064  };
                constexpr PDOMappingStruct TPDO_VELOCITY_ACTUAL_VALUE =      {32, 0, 0x606C  };
                constexpr PDOMappingStruct TPDO_TORQUE_DEMAND_VALUE =        {16, 0, 0x6074  };
                constexpr PDOMappingStruct TPDO_TORQUE_ACTUAL_VALUE =        {16, 0, 0x6077  };
                constexpr PDOMappingStruct TPDO_DC_LINK_CIRCUIT_VOLTAGE =    {32, 0, 0x6079  };
                constexpr PDOMappingStruct TPDO_POSITION_FOLLOWING_ERROR =   {32, 0, 0x60F4  };
                constexpr PDOMappingStruct TPDO_DIGITAL_INPUTS =             {32, 0, 0x60FD  };
                constexpr PDOMappingStruct TPDO_ANALOG_INPUT =               {16, 0, 0x2205  };
                // See the "EtherCAT Application Manual" in the "Transmit PDO Mapping (Inputs)" section
                /*constexpr uint16_t TPDO_STATUS_WORD=0x6041;                            // 16bit
                constexpr uint16_t TPDO_MODE_OF_OPERATION_DISPLAY=0x6061;              // 8bit
                constexpr uint16_t TPDO_POSITION_DEMAND_VALUE=0x6062;                  // 32bit
                constexpr uint16_t TPDO_POSITION_ACTUAL_VALUE=0x6064;                  // 32bit
                constexpr uint16_t TPDO_VELOCITY_ACTUAL_VALUE=0x606C;                  // 32bit
                constexpr uint16_t TPDO_TORQUE_DEMAND_VALUE=0x6074;                    // 16bit
                constexpr uint16_t TPDO_TORQUE_ACTUAL_VALUE=0x6077;                    // 16bit
                constexpr uint16_t TPDO_DC_LINK_CIRCUIT_VOLTAGE=0x6079;                // 32bit
                constexpr uint16_t TPDO_POSITION_FOLLOWING_ERROR=0x60F4;               // 32bit
                constexpr uint16_t TPDO_DIGITAL_INPUTS=0x60FD;                         // 32bit
                constexpr uint16_t TPDO_ANALOG_INPUT=0x2205;                           // 16bit*/

                /*constexpr uint16_t TPDO_STATUS_WORD=0x1A0A;                            // 16bit
                constexpr uint16_t TPDO_MODE_OF_OPERATION_DISPLAY=0x1A0B;              // 8bit
                constexpr uint16_t TPDO_POSITION_DEMAND_VALUE=0x1A0C;                  // 32bit
                constexpr uint16_t TPDO_POSITION_ACTUAL_VALUE=0x1A0E;                  // 32bit
                constexpr uint16_t TPDO_VELOCITY_ACTUAL_VALUE=0x1A11;                  // 32bit
                constexpr uint16_t TPDO_TORQUE_DEMAND_VALUE=0x1A12;                    // 16bit
                constexpr uint16_t TPDO_TORQUE_ACTUAL_VALUE=0x1A13;                    // 16bit
                constexpr uint16_t TPDO_DC_LINK_CIRCUIT_VOLTAGE=0x1A18;                // 32bit
                constexpr uint16_t TPDO_POSITION_FOLLOWING_ERROR=0x1A19;               // 32bit
                constexpr uint16_t TPDO_DIGITAL_INPUTS=0x1A1C;                         // 32bit
                constexpr uint16_t TPDO_ANALOG_INPUT=0x1A1D;                           // 16bit*/
                
                /// \details See CANOpen DS402 document for description of fields.
                ///          Here fields are name "Torque" but actually refer to current in amps.
                struct Data // Max 20 entry or 32Byte
                {
                    uint16_t statusWord;
                    uint8_t   modeOfOperationDisplay;
                    int32_t  actualPosition;
                    int32_t  actualVelocity;
                    int16_t  actualTorque;  ///< Actual torque in RTU.
                    //int8_t   padding;       // Why is this padding required?
                    /*uint32_t dcVoltage;
                    int16_t  demandTorque;
                    int32_t  digitalInput;
                    int16_t  analogInput;*/
                    uint32_t userMiso;
                    uint32_t timestamp;
                    int32_t positionDemandInternal;
                    int32_t velocityDemand;
                    int16_t torqueDemand;
                } __attribute__((packed));

                struct Input
                {
                    uint16_t statusWord;
                    int8_t   modeOfOperationDisplay;
                    int8_t   padding;
                    int32_t  actualPosition;
                    int32_t  actualVelocity;
                    int16_t  demandTorque;
                    int16_t  actualTorque;  ///< Actual torque in RTU.
                    uint32_t dcVoltage;
                    int32_t  digitalInput;
                    int16_t  analogInput;
                    int32_t  demandPosition;
                } __attribute__((packed));
                constexpr uint8_t TxMapping[] = { 0x0A, 0x00, 0x0A, 0x1A, 0x0B, 0x1A, 0x0E, 0x1A, 0x11, 0x1A, 0x12, 0x1A, 0x13, 0x1A, 0x18, 0x1A, 0x1C, 0x1A, 0x1D, 0x1A, 0x1B, 0x1A };

                struct Output
                {
                    uint16_t controlWord;
                    int8_t modeOfOperation;
                    uint8_t padding;
                    int16_t  targetTorque;   ///< Target current in RTU, 1 RTU = Motor Rate Current (amps) / 1000.
                    uint16_t maxTorque;      ///< Maximum current in mAmps.
                    int32_t  targetPosition;
                    uint32_t velocityOffset;
                    int32_t  digitalOutput;
                } __attribute__((packed));
                constexpr uint8_t RxMapping[] = { 0x07, 0x00, 0x0A, 0x16, 0x0B, 0x16, 0x0C, 0x16, 0x0D, 0x16, 0x0F, 0x16, 0x17, 0x16, 0x1D, 0x16 };

                constexpr uint8_t mappingCount = 0x09;
                constexpr PDOMappingStruct mapping[mappingCount] = {
                    TPDO_STATUS_WORD,
                    TPDO_MODE_OF_OPERATION_DISPLAY,
                    TPDO_POSITION_ACTUAL_VALUE,
                    TPDO_VELOCITY_ACTUAL_VALUE,
                    TPDO_TORQUE_ACTUAL_VALUE,
                    TPDO_DC_LINK_CIRCUIT_VOLTAGE,
                    TPDO_TORQUE_DEMAND_VALUE,
                    TPDO_DIGITAL_INPUTS,
                    TPDO_ANALOG_INPUT
                };
            }

            namespace control
            {
                // See the "EtherCAT Application Manual" in the "Receive PDO Mapping (Outputs)" section
                //                          Name                            Size Subindex Index
                constexpr PDOMappingStruct RPDO_CONTROL_WORD =               {16, 0, 0x6040  };
                constexpr PDOMappingStruct RPDO_MODE_OF_OPERATION =          {8 , 0, 0x6060  };
                constexpr PDOMappingStruct RPDO_TARGET_TORQUE =              {16, 0, 0x6071  };
                constexpr PDOMappingStruct RPDO_MAX_TORQUE =                 {16, 0, 0x6072  };
                constexpr PDOMappingStruct RPDO_MAX_CURRENT =                {16, 0, 0x6073  };
                constexpr PDOMappingStruct RPDO_TARGET_POSITION =            {32, 0, 0x607A  };
                constexpr PDOMappingStruct RPDO_POLARITY =                   {8 , 0, 0x607E  };
                constexpr PDOMappingStruct RPDO_PROFILE_VELOCITY =           {32, 0, 0x6081  };
                constexpr PDOMappingStruct RPDO_PROFILE_ACCELERATION =       {32, 0, 0x6083  };
                constexpr PDOMappingStruct RPDO_PROFILE_DECELERATION =       {32, 0, 0x6084  };
                constexpr PDOMappingStruct RPDO_TORQUE_SLOPE =               {16, 0, 0x6087  };
                constexpr PDOMappingStruct RPDO_VELOCITY_OFFSET =            {32, 0, 0x60B1  };
                constexpr PDOMappingStruct RPDO_TORQUE_OFFSET =              {16, 0, 0x60B2  };
                constexpr PDOMappingStruct RPDO_DIGITAL_OUTPUT =             {32, 0, 0x60FE  };
                constexpr PDOMappingStruct RPDO_TARGET_VELOCITY =            {32, 0, 0x60FF  };
                /*constexpr uint16_t RPDO_CONTROL_WORD=0x6040;               // 16bit
                constexpr uint16_t RPDO_MODE_OF_OPERATION=0x6060;          // 8bit
                constexpr uint16_t RPDO_TARGET_TORQUE=0x6071;              // 16bit
                constexpr uint16_t RPDO_MAX_TORQUE=0x6072;                 // 16bit
                constexpr uint16_t RPDO_MAX_CURRENT=0x6073;                // 16bit
                constexpr uint16_t RPDO_TARGET_POSITION=0x607A;            // 32bit
                constexpr uint16_t RPDO_POLARITY=0x607E;                   // 8bit
                constexpr uint16_t RPDO_PROFILE_VELOCITY=0x6081;           // 32bit
                constexpr uint16_t RPDO_PROFILE_ACCELERATION=0x6083;       // 32bit
                constexpr uint16_t RPDO_PROFILE_DECELERATION=0x6084;       // 32bit
                constexpr uint16_t RPDO_TORQUE_SLOPE=0x6087;               // 16bit
                constexpr uint16_t RPDO_VELOCITY_OFFSET=0x60B1;            // 32bit
                constexpr uint16_t RPDO_TORQUE_OFFSET=0x60B2;              // 16bit
                constexpr uint16_t RPDO_TARGET_VELOCITY=0x60FF;            // 32bit
                constexpr uint16_t RPDO_DIGITAL_OUTPUT=0x60FE;             // 32bit*/

                /*constexpr uint16_t RPDO_CONTROL_WORD=0x160A;               // 16bit
                constexpr uint16_t RPDO_MODE_OF_OPERATION=0x160B;          // 8bit
                constexpr uint16_t RPDO_TARGET_TORQUE=0x160C;              // 16bit
                constexpr uint16_t RPDO_MAX_TORQUE=0x160D;                 // 16bit
                constexpr uint16_t RPDO_MAX_CURRENT=0x160E;                // 16bit
                constexpr uint16_t RPDO_TARGET_POSITION=0x160F;            // 32bit
                constexpr uint16_t RPDO_POLARITY=0x161E;                   // 8bit
                constexpr uint16_t RPDO_PROFILE_VELOCITY=0x1611;           // 32bit
                constexpr uint16_t RPDO_PROFILE_ACCELERATION=0x1613;       // 32bit
                constexpr uint16_t RPDO_PROFILE_DECELERATION=0x1614;       // 32bit
                constexpr uint16_t RPDO_TORQUE_SLOPE=0x1615;               // 16bit
                constexpr uint16_t RPDO_VELOCITY_OFFSET=0x1617;            // 32bit
                constexpr uint16_t RPDO_TORQUE_OFFSET=0x1618;              // 16bit
                constexpr uint16_t RPDO_TARGET_VELOCITY=0x161C;            // 32bit
                constexpr uint16_t RPDO_DIGITAL_OUTPUT=0x161D;             // 32bit*/

                struct Data
                {
                    uint16_t controlWord;
                    uint8_t modeOfOperation;
                    int16_t targetTorque;   ///< Target current in RTU, 1 RTU = Motor Rate Current (amps) / 1000.
                    int32_t targetPosition;
                    int32_t targetVelocity;
                    uint16_t torqueOffset;
                    uint32_t tunningCommand;
                    //uint8_t padding;         // Why is this padding required?
                    /*uint16_t maxTorque;      ///< Maximum current in mAmps.
                    uint32_t velocityOffset;
                    int32_t  digitalOutput;*/
                } __attribute__((packed));

                constexpr uint16_t mappingCount = 0x0008;
                constexpr PDOMappingStruct mapping[mappingCount] = {
                    RPDO_CONTROL_WORD,
                    RPDO_MODE_OF_OPERATION,
                    RPDO_TARGET_TORQUE,
                    RPDO_TARGET_POSITION,
                    RPDO_TARGET_VELOCITY,
                    RPDO_MAX_TORQUE,
                    RPDO_VELOCITY_OFFSET,
                    RPDO_DIGITAL_OUTPUT
                };
            }
        }

    }

    namespace sdo
    {
        enum Access {
            READ,
            READ_WRITE
        };
        //template<typename T>
        struct Object
        {
            uint16_t index;         ///< Main index in the SDO dict
            uint8_t  subindex;      ///< Subindex in the SDO dict
            char const* name{""};   ///< Name of SDO, only for print purpose
            Access access;
            //T data;
        };
        namespace motor
        {
            constexpr uint32_t MAX_PROFILED_VELOCITY = 1000000U;  ///< Value of the sdo on the motors.

            constexpr uint32_t STOP_DECELERATION_VALUE = 2147483647;

            /// \brief Min profile velocity for torque mode.
            constexpr int32_t MIN_POSITIVE_PROFILED_VELOCITY = 1;

            constexpr int32_t ENABLE_STOPS_PARAM = 1;
            constexpr int32_t ENABLE_WATCHDOG_PARAM = 1;
            constexpr int32_t SYNC_STOPS_PARAM = 1;

            // On elmo boards, the command to save the values to flash is to send the word
            // "save" written in ascii hex values, in little-endian order.
            constexpr uint32_t const SAVE_DATA_PARAM = 0x65766173U;

            namespace Objects
            {
                static Object NOT_AN_OBJECT       { 0x0000, 0, "Not an Object"                 , Access::READ };
                static Object PRE_DEFINED_ERROR_FIELD_COUNT         { 0x1003, 0, "Pre_defined error field count"      , Access::READ_WRITE };
                static Object PRE_DEFINED_ERROR_FIELD_1         { 0x1003, 1, "Pre_defined error field 1"      , Access::READ };
                static Object PRE_DEFINED_ERROR_FIELD_2         { 0x1003, 2, "Pre_defined error field 2"      , Access::READ };
                static Object PRE_DEFINED_ERROR_FIELD_3         { 0x1003, 3, "Pre_defined error field 3"      , Access::READ };
                static Object PRE_DEFINED_ERROR_FIELD_4         { 0x1003, 4, "Pre_defined error field 4"      , Access::READ };
                static Object DEVICE_NAME         { 0x1008, 0, "Manufacturer device name"      , Access::READ };
                static Object HARDWARE_VERSION    { 0x1009, 0, "Manufacturer hardware version" , Access::READ };
                static Object SOFTWARE_VERSION    { 0x100A, 0, "Manufacturer software version" , Access::READ };
                static Object VENDOR_ID           { 0x1018, 1, "Vendor ID"                     , Access::READ };
                static Object PRODUCT_ID          { 0x1018, 2, "Product ID"                    , Access::READ };
                static Object REVISION_NUMBER     { 0x1018, 3, "Revision number"               , Access::READ };
                static Object SERIAL_NUMBER       { 0x1018, 4, "Serial number"                 , Access::READ };
                static Object FOLLOWING_ERROR_WINDOW      { 0x6065, 0, "Following error window"       , Access::READ_WRITE };
                static Object FOLLOWING_ERROR_TIMEOUT     { 0x6066, 0, "Following error timeout"      , Access::READ_WRITE };
                static Object VELOCITY_ERROR_WINDOW             { 0x606D, 0, "Velocity error window"       , Access::READ_WRITE };
                static Object VELOCITY_ERROR_WINDOW_TIMEOUT     { 0x606E, 0, "Velocity error timeout"      , Access::READ_WRITE };
                static Object NOMINAL_CURRENT     { 0x6075, 0, "Nominal current"               , Access::READ_WRITE }; // mA
                static Object RATED_TORQUE_FACTOR { 0x6076, 0, "Rated torque factor"           , Access::READ_WRITE };
                static Object MAX_MOTOR_SPEED     { 0x6080, 0, "Max motor speed"               , Access::READ_WRITE };
                static Object MAX_TORQUE          { 0x6072, 0, "Max torque"                    , Access::READ_WRITE };
                static Object PROFILE_VELOCITY    { 0x6081, 0, "Profile velocity"              , Access::READ_WRITE };
                static Object PROFILE_END_VELOCITY    { 0x6082, 0, "Profile end velocity"      , Access::READ_WRITE };
                static Object PROFILE_ACCELERATION   { 0x6083, 0, "Profile acceleration"       , Access::READ_WRITE };
                static Object PROFILE_DECELERATION   { 0x6084, 0, "Profile deceleration"       , Access::READ_WRITE };
                static Object PROFILE_MAX_VELOCITY   { 0x607F, 0, "Profile max velocity"       , Access::READ_WRITE };
                static Object PROFILE_RANGE_LIMIT_MIN   { 0x607B, 1, "Profile range limit min" , Access::READ_WRITE };
                static Object PROFILE_RANGE_LIMIT_MAX   { 0x607B, 2, "Profile range limit max" , Access::READ_WRITE };
                static Object SOFTWARE_POSITION_LIMIT_MIN   { 0x607D, 1, "Software position limit min" , Access::READ_WRITE };
                static Object SOFTWARE_POSITION_LIMIT_MAX   { 0x607D, 2, "Software position limit max" , Access::READ_WRITE };
                static Object QUICK_STOP_DECELERATION   { 0x6085, 0, "Motor quick stop deceleration"   , Access::READ_WRITE };
                static Object MOTION_PROFILE_TYPE { 0x6086, 0, "Motion profile type"           , Access::READ_WRITE };
                static Object TORQUE_SLOPE        { 0x6087, 0, "Torque slope"                  , Access::READ_WRITE };
                static Object MAX_ACCELERATION    { 0x60C5, 0, "Maximum acceleration"          , Access::READ_WRITE };
                static Object MAX_DECELERATION    { 0x60C6, 0, "Maximum deceleration"          , Access::READ_WRITE };
                static Object STORE_PARAMETERS    { 0x1010, 1, "Store parameter"               , Access::READ_WRITE };
                static Object RPDO_PARAM_1        { 0x1400, 1, "Receive PDO1 Params"           , Access::READ_WRITE };
                static Object RPDO_PARAM_2        { 0x1401, 1, "Receive PDO2 Params"           , Access::READ_WRITE };
                static Object RPDO_PARAM_3        { 0x1402, 1, "Receive PDO3 Params"           , Access::READ_WRITE };
                static Object RPDO_PARAM_4        { 0x1403, 1, "Receive PDO4 Params"           , Access::READ_WRITE };
                static Object RPDO_MAP_1          { 0x1600, 1, "Receive PDO1 Mapping"          , Access::READ_WRITE };
                static Object RPDO_MAP_2          { 0x1601, 1, "Receive PDO2 Mapping"          , Access::READ_WRITE };
                static Object RPDO_MAP_3          { 0x1602, 1, "Receive PDO3 Mapping"          , Access::READ_WRITE };
                static Object RPDO_MAP_4          { 0x1603, 1, "Receive PDO4 Mapping"          , Access::READ_WRITE };
                static Object TPDO_PARAM_1        { 0x1800, 1, "Transmit PDO1 Params"          , Access::READ_WRITE };
                static Object TPDO_PARAM_2        { 0x1801, 1, "Transmit PDO2 Params"          , Access::READ_WRITE };
                static Object TPDO_PARAM_3        { 0x1802, 1, "Transmit PDO3 Params"          , Access::READ_WRITE };
                static Object TPDO_PARAM_4        { 0x1803, 1, "Transmit PDO4 Params"          , Access::READ_WRITE };
                static Object TPDO_MAP_1          { 0x1A00, 1, "Transmit PDO1 Mapping"         , Access::READ_WRITE };
                static Object TPDO_MAP_2          { 0x1A01, 1, "Transmit PDO2 Mapping"         , Access::READ_WRITE };
                static Object TPDO_MAP_3          { 0x1A02, 1, "Transmit PDO3 Mapping"         , Access::READ_WRITE };
                static Object TPDO_MAP_4          { 0x1A03, 1, "Transmit PDO4 Mapping"         , Access::READ_WRITE };
                static Object ERROR               { 0x1001, 0, "Error"         , Access::READ };
                static Object ERROR_CODE          { 0x603F, 0, "Error code"         , Access::READ };
                static Object EXTENDED_ERROR_FEEDBACK { 0x2081, 1, "Extended Error Feedback"         , Access::READ };
                static Object EXTENDED_ERROR_PROFILER { 0x2081, 2, "Extended Error Profiler"         , Access::READ };
                static Object EXTENDED_ERROR_SDO { 0x2081, 3, "Extended Error SDO"         , Access::READ };
                static Object EXTENDED_ERROR_MOTOR { 0x2081, 4, "Extended Error Motor fault reason"         , Access::READ };
                static Object EXTENDED_ERROR_ECAM { 0x2081, 5, "Extended Error ECAM"         , Access::READ };
                static Object EMCY_ERROR_CODE     { 0x2F21, 0, "EMC Error code"         , Access::READ };
                static Object CONTROL_RPDO        { 0x1C12, 0, "RxPDO" };
                static Object MEASURES_TPDO       { 0x1C13, 0, "TxPDO" };
            }

        }
    }

    namespace motor 
    {
        double NmToAmps(double const& torque, double const& kT);
        double AmpsToNm(double const& amps, double const& kT);
        int16_t ampsToRTU(double const& amp, double const& nominalCurrent);
        int16_t NmToRTU(double const& torque, double const& kT, double const& nominalCurrent);
        double RTUToAmps(int16_t const& RTU, double const& nominalCurrent);
        double RTUToNm(int16_t const& RTU, double const& kT, double const& nominalCurrent);
        double radpsToRPM(double const& speed);
        double RPMToRadps(double const& speed);
        std::string const errorCodeToString(uint16_t code);
    }

    class StateMachine
    {
    public:
        StateMachine();

        /// \brief Set the desired state.
        void setCommand(Command command);
        State getState();

        /// \brief Update state machine with a fresh statusword, to be called on each ethercat update.
        void update(uint16_t statusWord);

        /// \brief Get the new control word that must be sent the the slave.
        uint16_t getControlWord();

        void reset();

        bool isReadyToSwitchOn()        { return std::bitset<16>(statusWord_)[0]; }
        bool isSwitchedOn()             { return std::bitset<16>(statusWord_)[1]; }
        bool isOperationEnabled()       { return std::bitset<16>(statusWord_)[2]; }
        bool isFaulty()                 { return std::bitset<16>(statusWord_)[3]; }
        bool isVoltageEnabled()         { return std::bitset<16>(statusWord_)[4]; }
        bool isQuickStop()              { return std::bitset<16>(statusWord_)[5]; }
        bool isSwitchOnDisabled()       { return std::bitset<16>(statusWord_)[6]; }
        bool isWarning()                { return std::bitset<16>(statusWord_)[7]; }
        bool isTargetReached()          { return std::bitset<16>(statusWord_)[10]; }
        bool isInternalLimitTriggered() { return std::bitset<16>(statusWord_)[11]; }

        bool isSetpointACK(uint16_t const modeOfOperation) {
            return std::bitset<16>(statusWord_)[12] && modeOfOperation == CANOpen::control::Mode::POSITION_PROFILE;
        }
        bool isFollowingError(uint16_t const modeOfOperation) {
            return std::bitset<16>(statusWord_)[13] && (modeOfOperation == CANOpen::control::Mode::POSITION_PROFILE or modeOfOperation == CANOpen::control::Mode::POSITION_CYCLIC);
        }
        bool isSpeed(uint16_t const modeOfOperation) {
            return std::bitset<16>(statusWord_)[12] && modeOfOperation == CANOpen::control::Mode::VELOCITY_PROFILE;
        }
        bool isMaxSlippageError(uint16_t const modeOfOperation) {
            return std::bitset<16>(statusWord_)[13] && modeOfOperation == CANOpen::control::Mode::VELOCITY_PROFILE;
        }
        bool isPositionInterpolationActive(uint16_t const modeOfOperation) {
            return std::bitset<16>(statusWord_)[12] && modeOfOperation == CANOpen::control::Mode::POSITION_CYCLIC;
        }
        
    private:
        void enable();
        void disable();
        void setPosition();
        int64_t getCurrentTimeUs();

        Command command_;
        State state_;

        int64_t startTimestamp_;
        uint16_t controlWord_;
        uint16_t statusWord_;
    };

}
#endif
