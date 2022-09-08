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
    void handleReply(std::vector<T> answers, uint16_t replied_wkc = 1)
    {
        EXPECT_CALL(*io, read(_,_))
        .WillOnce(Invoke([this, replied_wkc, answers](uint8_t* data, int32_t)
        {
            auto it = answers.begin();
            uint16_t* wkc = reinterpret_cast<uint16_t*>(payload + header->len);
            DatagramHeader* current_header = header;                     // current heade rto check loop condition

            do
            {
                std::memcpy(payload, &(*it), sizeof(T));
                *wkc = replied_wkc;

                current_header = header;                                    // save current header
                ++it;                                                       // next payload
                datagram = reinterpret_cast<uint8_t*>(wkc) + 2;             // next datagram
                header = reinterpret_cast<DatagramHeader*>(datagram);       // next header
                payload = datagram + sizeof(DatagramHeader);                // next payload
                wkc = reinterpret_cast<uint16_t*>(payload + header->len);   // next wkc
            } while (current_header->multiple == 1);

            int32_t answer_size = inflight.finalize();
            std::memcpy(data, inflight.data(), answer_size);
            return answer_size;
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

    handleReply<uint8_t>({0x})

    sendGetRegister(link, 0x00, 0x110, value_read);
    printf("%08x", value_read);
}