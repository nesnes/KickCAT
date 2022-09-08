#include <gtest/gtest.h>
#include <cstring>

#include "kickcat/Link.h"
#include "kickcat/DebugHelpers.h"
#include "Mocks.h"

using ::testing::Return;
using ::testing::_;
using ::testing::Invoke;
using ::testing::InSequence;

using namespace kickcat;

class LinkTest : public testing::Test
{
public:
    void checkSendFrame(int32_t datagrams_number)
    {
        EXPECT_CALL(*io, write(_,_))
        .WillOnce(Invoke([this, datagrams_number](uint8_t const* data, int32_t data_size)
        {
            int32_t available_datagrams = 0;
            Frame frame(data, data_size);
            while (frame.isDatagramAvailable())
            {
                (void)frame.nextDatagram();
                available_datagrams++;
            }
            EXPECT_EQ(datagrams_number, available_datagrams);
            return data_size;
        }));
    }

    template<typename T>
    void addDatagram(enum Command command, uint32_t address, void const* data, uint16_t data_size,
                           std::function<DatagramState(DatagramHeader const*, uint8_t const* data, uint16_t wkc)> const& process,
                           std::function<void(DatagramState const& state)> const& error)
    {
        link.addDatagram(command, address, &data, sizeof(data), process, error);
    }

protected:
    std::shared_ptr<MockSocket> io{ std::make_shared<MockSocket>() };
    Link link{ io };
};

TEST_F(LinkTest, send_get_register)
{
    uint16_t value_read;

    sendGetRegister(link, 0x00, 0x110, value_read);
    printf("%08x", value_read);
}