#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>
#include "kickcat/Bus.h"
#include "CANOpen.h"
#include "kickcat/Prints.h"

#ifdef __linux__
    #include "kickcat/OS/Linux/Socket.h"
#elif __PikeOS__
    #include "kickcat/OS/PikeOS/Socket.h"
#else
    #error "Unknown platform"
#endif


using namespace kickcat;

auto callback_error = [](DatagramState const&){THROW_ERROR("Something bad happened");};
uint8_t io_buffer[2048];

/******* TEST SETTINGS ********/
int slaveID = 1; // index of the slave on the bus, starts at 0.
int printErrorCode(kickcat::ErrorCode const& e)
{
    std::cerr << e.what() << ": " << ALStatus_to_string(e.code()) << std::endl;
    return 1;
};

int printException(std::exception const& e)
{
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
};

void printSingleConfig32(Bus& bus, Slave& slave, CANOpen::sdo::Object& object)
{
    try {
        uint32_t data = 0;
        uint32_t size=sizeof(data);
        bus.readSDO(slave, object.index, object.subindex, Bus::Access::PARTIAL, (void*)&data, &size);
        printf("\t %s: %04X = %i\n", object.name, data, data);
    } catch (...) {}
};

void printSingleConfig16(Bus& bus, Slave& slave, CANOpen::sdo::Object& object)
{
    try {
        uint16_t data = 0;
        uint32_t size=sizeof(data);
        bus.readSDO(slave, object.index, object.subindex, Bus::Access::PARTIAL, (void*)&data, &size);
        printf("\t %s: %02X = %i\n", object.name, data, data);
    } catch (...) {}
};

// Utils
void initMapping(Bus& bus, Slave& slave)
{
    auto mapPDO = [&](CANOpen::sdo::Object PDO_param, CANOpen::sdo::Object PDO_map, uint32_t* data, uint32_t dataSize,  CANOpen::sdo::Object SM_map){
        // Driver should be in PRE-OP state
        // Unmap previous registers, setting 0 in 0x1A00:00 (PDO's register list)
        uint8_t zeroU8=0;
        uint32_t zeroU8_size=sizeof(zeroU8);
        uint16_t zeroU16 = 0;
        uint32_t zeroU16_size = sizeof(zeroU16);
        uint32_t zeroU32=0;
        uint32_t zeroU32_size=sizeof(zeroU32);
        bus.writeSDO(slave, PDO_map.index, 0, false, (void*)&zeroU8, zeroU8_size); // on elmo, sub0 is u8 while sub1-8 is u32
        // Modify mapping, setting register address in PDO's subindexes from 0x1A00:01
        for (int i=0;i<dataSize;i++)
        {
            bus.writeSDO(slave, PDO_map.index, i+1, false, (void*)&(data[i]), sizeof(data[i]));
        }
        // Enable mapping by setting number of registers in 0x1A00:00
        uint8_t tpdoCount=dataSize;
        uint32_t tpdoCount_size=sizeof(tpdoCount);
        bus.writeSDO(slave, PDO_map.index, 0, false, (void*)&tpdoCount, tpdoCount_size);
        // Set PDO mapping to SM
        bus.writeSDO(slave, SM_map.index, 0, false, (void*)(&zeroU8), zeroU8_size); // size to 0, elmo doc says u16 but only u8 works.
        uint16_t pdoMapAddr = PDO_map.index;
        uint32_t pdoMapAddr_size = sizeof(pdoMapAddr);
        bus.writeSDO(slave, SM_map.index, 1, false, (void*)(&pdoMapAddr), pdoMapAddr_size); // first PDO map at subindex 1
        uint8_t pdoMapSize = 1;
        uint32_t pdoMapSize_size = sizeof(pdoMapSize);
        bus.writeSDO(slave, SM_map.index, 0, false, (void*)(&pdoMapSize), pdoMapSize_size); // size to 1
    };
    uint32_t mapTPDO[] = {0x60410010/*, 0x60610008*/, 0x60640020, 0x606C0020/*, 0x60740010*/, 0x60770010/*, 0x60790020*/};
    uint32_t mapTPDOCount = sizeof(mapTPDO)/sizeof(uint32_t);
    mapPDO(CANOpen::sdo::motor::Objects::TPDO_PARAM_1, CANOpen::sdo::motor::Objects::TPDO_MAP_1, mapTPDO, mapTPDOCount, CANOpen::sdo::motor::Objects::MEASURES_TPDO);
    
    uint32_t mapRPDO[] = {0x60400010, 0x60600008, 0x60710010/*, 0x60720010, 0x607A0020, 0x60B10020, 0x60FF0020, 0x60B20010*/};
    uint32_t mapRPDOCount = sizeof(mapRPDO)/sizeof(uint32_t);
    mapPDO(CANOpen::sdo::motor::Objects::RPDO_PARAM_1, CANOpen::sdo::motor::Objects::RPDO_MAP_1, mapRPDO, mapRPDOCount, CANOpen::sdo::motor::Objects::CONTROL_RPDO);
};

int main(int argc, char* argv[])
{
    auto socket = std::make_shared<Socket>();
    Bus bus(socket);

    auto print_current_state = [&]()
    {
        for (auto& slave : bus.slaves())
        {
            State state = bus.getCurrentState(slave);
            printf("Slave %d state is %s\n", slave.address, toString(state));
        }
    };

    uint8_t io_buffer[2048];
    try
    {
        socket->open(argv[1], 2ms);
        bus.init();

        Slave& elmo = bus.slaves().at(1);

        printInfo(elmo);
        initMapping(bus, elmo);


        
        Slave& pelvis = bus.slaves().at(0);
        pelvis.is_static_mapping = true;
        pelvis.input.bsize = 356;
        pelvis.input.sync_manager = 3;
        pelvis.output.bsize = 8;
        pelvis.output.sync_manager = 2;
        
        bus.createMapping(io_buffer);

        bus.requestState(State::SAFE_OP);
        bus.waitForState(State::SAFE_OP, 1s);
        print_current_state();
    }
    catch (ErrorCode const& e)
    {
        std::cerr << e.what() << ": " << ALStatus_to_string(e.code()) << std::endl;
        return 1;
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    Slave& slave = bus.slaves().at(1);
    // Set default values in PDO
    struct OutPDO {
        uint16_t controlWorld;
        //int8_t padding;//?
        int8_t modeOfOperation;
        int8_t padding;//?
        int16_t targetTorque;
    };
    OutPDO* outPDO = reinterpret_cast<OutPDO*>(slave.output.data);
    outPDO->controlWorld = 0;
    outPDO->modeOfOperation = 10; // Cyclic synchronous torque
    outPDO->targetTorque = 0;

    CANOpen::sdo::Object SDO_STATUS_WORD{ 0x6041, 0, "Status word", CANOpen::sdo::Access::READ };
    printSingleConfig16(bus, bus.slaves().at(slaveID), SDO_STATUS_WORD);

    // OPERATIONAL
    printf("Slave %d state is %s\n", slave.address, toString(bus.getCurrentState(slave)));
    try { bus.processDataReadWrite(callback_error); } catch (...) {}
    //try { bus.processDataRead(callback_error); } catch (...) {}
    //try { bus.processDataWrite([](){}); } catch (...) {}
    
    printf("Switch to OPERATIONAL\n");
    bus.requestState(State::OPERATIONAL);
    while(true){
        try { bus.processDataReadWrite(callback_error); } catch (...) {}
        //try { bus.processDataRead(callback_error); } catch (...) {}
        //try { bus.processDataWrite([](){}); } catch (...) {}
        try
        {
            bus.waitForState(State::OPERATIONAL, 100ms);
            break;
        }
        catch (ErrorCode const& e){ printErrorCode(e);}
        catch (std::exception const& e){ printException(e); }
    }
    printf("Slave %d state is %s\n", slave.address, toString(bus.getCurrentState(slave)));

    printSingleConfig32(bus, bus.slaves().at(slaveID), CANOpen::sdo::motor::Objects::ERROR);
    printSingleConfig16(bus, bus.slaves().at(slaveID), CANOpen::sdo::motor::Objects::ERROR_CODE);
    printSingleConfig16(bus, bus.slaves().at(slaveID), SDO_STATUS_WORD);

    struct InPDO {
        uint16_t statusWord;
        int32_t actualPosition;
        int32_t actualVelocity;
        int16_t actualTorque;
    };

    return 1;
    while(true){
        try { bus.processDataReadWrite(callback_error); } catch (...) {}
        //try { bus.processDataWrite([](){}); } catch (...) {}
        try { 
            //bus.processDataRead(callback_error); 
            InPDO* in = reinterpret_cast<InPDO*>(slave.input.data);
            std::cout << "PDO \t" << in->statusWord << "\t" << in->actualPosition << "\t" << in->actualVelocity << "\t" << in->actualTorque << "                 \r";
            // Set output
            OutPDO* outPDO = reinterpret_cast<OutPDO*>(slave.output.data);
            outPDO->controlWorld = 0;
            outPDO->modeOfOperation = 10; // Cyclic synchronous torque
            outPDO->targetTorque = 100;
        } catch (...) {}
        sleep(milliseconds(5));
    }


    return 0;
}