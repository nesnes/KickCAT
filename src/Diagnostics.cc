#include "Diagnostics.h"
#include "Error.h"

#include <unordered_map>
#include <cstdio>
#include <stack>


namespace kickcat
{
    std::unordered_map<uint16_t, uint16_t> getTopology(std::vector<Slave>& slaves)
    {
        std::unordered_map<uint16_t, uint16_t> topology;

        uint16_t lastSeen = slaves.at(0).address;
        std::stack<uint16_t> branches;
        for (auto& slave : slaves) 
        {   
            int openPorts = slave.countOpenPorts();

            switch (openPorts)
            {   
                case 0:
                {
                    THROW_ERROR("No open port on a slave - it should not exist in the bus");
                    break;
                }
                case 1:
                {
                    topology[slave.address] = lastSeen;
                    lastSeen = slave.address;
                    if (not branches.empty())
                    {
                        lastSeen = branches.top();
                        branches.pop();
                    }
                    break;
                }
                case 2:
                {
                    topology[slave.address] = lastSeen;
                    lastSeen = slave.address;
                    break;
                }
                default:
                {
                    topology[slave.address] = lastSeen;
                    lastSeen = slave.address;
                    for (int i = 2; i < openPorts; ++i)
                    {
                        branches.push(slave.address);
                    }
                }
            }
        }
        return topology;
    }

    void CRCAnalysis(std::vector<Slave>& slaves)
    {   
        std::unordered_map<uint16_t, uint16_t> topology = getTopology(slaves);
        std::vector<std::vector<uint16_t> > mat(slaves.size());
        for (size_t i = 0; i < slaves.size(); ++i) { mat[i].resize(4); }

        auto callback_error = [](DatagramState const&){ THROW_ERROR("something bad happened"); };

        uint16_t first_error = 0;
        uint16_t error_port = 0xFF;
        bool check = true;

        for (auto port = 0; port < 4; ++port)
        {
            for (size_t slave_id = 0; slave_id < slaves.size(); ++slave_id)
            {
                mat[slave_id][port] = slaves.at(slave_id).error_counters.rx[port].physical_layer;
                if (mat[slave_id][port] > 0 & check)
                {
                    first_error = slave_id;
                    error_port = port;
                    check = false;
                    break;
                }
            }
        }
        if (error_port != 0xFF) {printf("Error occured at : %04x - port %i\n", first_error, error_port);};
    }
}
