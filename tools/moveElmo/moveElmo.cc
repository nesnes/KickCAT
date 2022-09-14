#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>
#include "kickcat/Bus.h"
#include "kickcat/Prints.h"
#include "ElmoProtocol.h"

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
        wdc::hal::CANOpenStateMachine elmo_CAN;

        printInfo(elmo);

        //Read CANOpen Object
        uint32_t data;
        uint32_t length = sizeof(data);
        bus.readSDO(elmo, wdc::hal::sdo::motor::elmo::BOARD_TEMPERATURE.index, wdc::hal::sdo::motor::elmo::BOARD_TEMPERATURE.subindex, Bus::Access::PARTIAL, &data, &length);
        printf("Data received %i \n", data);

        //Configure Mapping
        uint32_t Rx_length = sizeof(wdc::hal::pdo::motor::elmo::RxMapping);
        bus.writeSDO(elmo, wdc::hal::sdo::motor::elmo::RxPDO.index, wdc::hal::sdo::motor::elmo::RxPDO.subindex, Bus::Access::COMPLETE, (void*) wdc::hal::pdo::motor::elmo::RxMapping, Rx_length);
        uint32_t Tx_length = sizeof(wdc::hal::pdo::motor::elmo::TxMapping);
        bus.writeSDO(elmo, wdc::hal::sdo::motor::elmo::TxPDO.index, wdc::hal::sdo::motor::elmo::TxPDO.subindex, Bus::Access::COMPLETE, (void*) wdc::hal::pdo::motor::elmo::TxMapping, Tx_length);
        
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

        auto callback_error = [](DatagramState const&){ THROW_ERROR("something bad happened"); };
        bus.processDataRead(callback_error);
        auto itWillBeWrong = [](DatagramState const&){printf("(Previous line was a False alarm) \n");};
        bus.processDataWrite(itWillBeWrong);

        bus.requestState(State::OPERATIONAL);
        bus.waitForState(State::OPERATIONAL, 100ms);
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





    Slave& elmo = bus.slaves().at(1);
    wdc::hal::CANOpenStateMachine elmo_CAN;

    elmo_CAN.update((uint16_t) wdc::hal::CANOpenCommand::ENABLE);
    printf("%i\n", elmo_CAN.getMotorState());


    return 0;
}