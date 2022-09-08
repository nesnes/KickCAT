#ifndef KICKCAT_DEBUGHELPERS_H
#define KICKCAT_DEBUGHELPERS_H

#include "Link.h"
#include "Bus.h"

namespace kickcat
{

    template<typename T>
    void sendGetRegister(Bus bus, uint16_t slave_address, uint16_t reg_address, T& value_read)
    {
        auto process = [&value_read](DatagramHeader const*, uint8_t const* data, uint16_t wkc)
        {
            if (wkc != 1)
            {
                return DatagramState::INVALID_WKC;
            }

            value_read = *reinterpret_cast<T const*>(data);
            return DatagramState::OK;
        };

        auto error = [](DatagramState const&)
        {
            THROW_ERROR("Error while trying to get slave register.");
        };

        bus.link_.addDatagram(Command::FPRD, createAddress(slave_address, reg_address), nullptr, sizeof(value_read), process, error);
        bus.link_.processDatagrams();
    }

    void sendGetRegister(Bus bus, uint16_t slave_address, uint16_t reg_address, uint16_t value_read)
    {
        sendGetRegister<uint16_t>(bus, slave_address, reg_address, value_read)
    }

    template<typename T>
    void sendWriteRegister(Bus bus, uint16_t slave_address, uint16_t reg_address, T& value_write)
    {
        auto process = [&value_read](DatagramHeader const*, uint8_t const* data, uint16_t wkc)
        {
            if (wkc != 1)
            {
                return DatagramState::INVALID_WKC;
            }
            return DatagramState::OK;
        };

        auto error = [](DatagramState const&)
        {
            THROW_ERROR("Error while trying to set slave register.");
        };

        bus.link_.addDatagram(Command::FPWR, createAddress(slave_address, reg_address), value_write, sizeof(value_write), process, error);
        bus.link_.processDatagrams();
    }
}

#endif
