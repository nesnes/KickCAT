#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>

#include "kickcat/Bus.h"
#include "kickcat/Prints.h"
#include "kickcat/Diagnostics.h"
#include "ElmoProtocol.h"
#include "Error.h"

#ifdef __linux__
    #include "kickcat/OS/Linux/Socket.h"
#elif __PikeOS__
    #include "kickcat/OS/PikeOS/Socket.h"
#else
    #error "Unknown platform"
#endif


using namespace kickcat;

auto callback_error = [](DatagramState const&){THROW_ERROR("Something bad happened");};

void printInputPDO(hal::pdo::elmo::Input* inputPDO)
{
    printf("statusWord :        %04x            \n", inputPDO->statusWord);
    printf("modeOfOperation :   %i              \n", inputPDO->modeOfOperationDisplay);
    printf("actualPosition:     %i              \n", inputPDO->actualPosition);
    printf("actualVelocity :    %i              \n", inputPDO->actualVelocity);
    printf("demandTorque :      %i              \n", inputPDO->demandTorque);
    printf("actualTorque :      %i              \n", inputPDO->actualTorque);  ///< Actual torque in RTU.
    printf("dcVoltage:          %i              \n", inputPDO->dcVoltage);
    printf("digitalInput :      %i              \n",  inputPDO->digitalInput);
    printf("analogInput :       %i              \n",  inputPDO->analogInput);
    printf("demandPosition :    %i              \n", inputPDO->demandPosition);
    printf("\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F");
}

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


        Slave& elmo = bus.slaves().at(0);

        printInfo(elmo);

        //Read CANOpen Object
        uint32_t data;
        uint32_t length = sizeof(data);
        bus.readSDO(elmo, hal::sdo::elmo::BOARD_TEMPERATURE.index, hal::sdo::elmo::BOARD_TEMPERATURE.subindex, Bus::Access::PARTIAL, &data, &length);
        printf("Data received %i \n", data);

        //Configure Mapping
        uint32_t Rx_length = sizeof(hal::pdo::elmo::RxMapping);
        bus.writeSDO(elmo, hal::sdo::elmo::RxPDO.index, hal::sdo::elmo::RxPDO.subindex, Bus::Access::COMPLETE, (void*) hal::pdo::elmo::RxMapping, Rx_length);
        uint32_t Tx_length = sizeof(hal::pdo::elmo::TxMapping);
        bus.writeSDO(elmo, hal::sdo::elmo::TxPDO.index, hal::sdo::elmo::TxPDO.subindex, Bus::Access::COMPLETE, (void*) hal::pdo::elmo::TxMapping, Tx_length);
        
        /*Slave& pelvis = bus.slaves().at(0);
        pelvis.is_static_mapping = true;
        pelvis.input.bsize = 356;
        pelvis.input.sync_manager = 3;
        pelvis.output.bsize = 8;
        pelvis.output.sync_manager = 2;

        Slave& ankle = bus.slaves().at(2);
        ankle.is_static_mapping = true;
        ankle.input.bsize = 114;
        ankle.input.sync_manager = 3;
        ankle.output.bsize = 4;
        ankle.output.sync_manager = 2; */
        
        bus.createMapping(io_buffer);

        bus.requestState(State::SAFE_OP);
        bus.waitForState(State::SAFE_OP, 1s);
        print_current_state();

        auto callback_error = [](DatagramState const&){ THROW_ERROR("something bad happened"); };
        bus.processDataRead(callback_error);
        auto itWillBeWrong = [](DatagramState const&){DEBUG_PRINT("Previous error was a false alarm\n");};
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



    Slave& elmo = bus.slaves().at(0);
    hal::pdo::elmo::Input* inputPDO {nullptr};
    hal::pdo::elmo::Output* outputPDO {nullptr};
    outputPDO = reinterpret_cast<hal::pdo::elmo::Output*>(elmo.output.data);
    inputPDO = reinterpret_cast<hal::pdo::elmo::Input*>(elmo.input.data);
    can::CANOpenStateMachine stateMachine;


    //Turn on
    stateMachine.setCommand(can::CANOpenCommand::ENABLE);
    while(not stateMachine.isON())
    {
        sleep(2ms);
        bus.sendLogicalRead(callback_error);
        bus.finalizeDatagrams();
        bus.processAwaitingFrames();

        stateMachine.statusWord_ = inputPDO->statusWord;
        stateMachine.update();

        outputPDO->controlWord = stateMachine.controlWord_;
        bus.sendLogicalWrite(callback_error);

    }
    stateMachine.printState();

    // Main Loop
    int64_t last_error = 0;
    for (int64_t i = 0; i < 8000; ++i)
    {
        sleep(1ms);

        try
        {
            bus.sendLogicalRead(callback_error);
            bus.sendLogicalWrite(callback_error);

            stateMachine.statusWord_ = inputPDO->statusWord;
            stateMachine.update();

            outputPDO->controlWord = stateMachine.controlWord_;
            outputPDO->modeOfOperation = 4;
            outputPDO->targetTorque = 50;
            outputPDO->maxTorque = 3990;
            outputPDO->targetPosition = 0;
            outputPDO->velocityOffset = 0;
            outputPDO->digitalOutput = 0;

            bus.processAwaitingFrames();

            printInputPDO(inputPDO);
        }
        catch (std::exception const& e)
        {
            int64_t delta = i - last_error;
            last_error = i;
            std::cerr << e.what() << " at " << i << " delta: " << delta << std::endl;
        }
    }
    //Turn off
    printf("\n\n\n\n\n\n\n\n\n\n\n\n");
    stateMachine.setCommand(can::CANOpenCommand::DISABLE);
    while(stateMachine.isON())
    {
        bus.sendLogicalRead(callback_error);
        bus.finalizeDatagrams();

        stateMachine.statusWord_ = inputPDO->statusWord;
        stateMachine.update();

        outputPDO->controlWord = stateMachine.controlWord_;
        bus.sendLogicalWrite(callback_error);
        bus.processAwaitingFrames();
    }
    stateMachine.printState();


    return 0;
}