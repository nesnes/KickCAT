#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/if_packet.h>

#include <cstring>

#include "LinuxSocket.h"
#include "protocol.h"

namespace kickcat
{
    void LinuxSocket::open(std::string const& interface)
    {
        // RAW socket with EtherCAT type
        fd_ = socket(PF_PACKET, SOCK_RAW, ETH_ETHERCAT_TYPE);
        if (fd_ < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }

        // Non-blocking mode
        struct timeval timeout{0, 1};

        int rc = setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }

        rc = setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }

        // EtherCAT frames shall not be routed to another network, but only on the dedicated interface
        int const dont_route = 1;
        rc = setsockopt(fd_, SOL_SOCKET, SO_DONTROUTE, &dont_route, sizeof(dont_route));
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }

        // Connect socket to interface and configure interface for EtherCAT use (promiscious, broadcast)
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface.c_str(), sizeof(ifr.ifr_name)-1);
        rc = ioctl(fd_, SIOCGIFINDEX, &ifr);
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }
        int interface_index = ifr.ifr_ifindex;

        ifr.ifr_flags = 0;
        rc = ioctl(fd_, SIOCGIFFLAGS, &ifr);
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }

        ifr.ifr_flags = ifr.ifr_flags | IFF_PROMISC | IFF_BROADCAST;
        rc = ioctl(fd_, SIOCSIFFLAGS, &ifr);
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }

        struct sockaddr_ll link_layer;
        link_layer.sll_family = AF_PACKET;
        link_layer.sll_ifindex = interface_index;
        link_layer.sll_protocol = ETH_ETHERCAT_TYPE;
        rc = bind(fd_, (struct sockaddr *)&link_layer, sizeof(link_layer));
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }
    }

    void LinuxSocket::close()
    {
        int rc = ::close(fd_);
        if (rc < 0)
        {
            throw std::system_error(errno, std::generic_category());
        }
    }

    int32_t LinuxSocket::read(uint8_t* frame, int32_t frame_size)
    {
        return ::read(fd_, frame, frame_size);
    }

    int32_t LinuxSocket::write(uint8_t const* frame, int32_t frame_size)
    {
        return ::write(fd_, frame, frame_size);
    }
}
