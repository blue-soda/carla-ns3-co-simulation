#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VanetCamSimulation");

void SendCamMessage(Ptr<Socket> socket, Ipv4Address broadcastAddress, uint16_t port, std::string vehicleId) {
    Ptr<Packet> packet = Create<Packet>((uint8_t*)vehicleId.c_str(), vehicleId.size());
    socket->SendTo(packet, 0, InetSocketAddress(broadcastAddress, port));
}

int main(int argc, char *argv[]) {
    Time::SetResolution(Time::NS);

    uint32_t numVehicles = 3;
    double simulationTime = 10.0;  
    double interval = 1.0;         

    NodeContainer vehicles;
    vehicles.Create(numVehicles);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);

    YansWifiPhyHelper wifiPhy; 
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer wifiDevices = wifi.Install(wifiPhy, wifiMac, vehicles);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(vehicles);

    for (uint32_t i = 0; i < numVehicles; i++) {
        Ptr<Node> node = vehicles.Get(i);
        Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
        mob->SetPosition(Vector(i * 10.0, 0.0, 0.0));
        mob->SetVelocity(Vector(5.0, 0.0, 0.0));   
    }

    InternetStackHelper internet;
    internet.Install(vehicles);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(wifiDevices);
    Ipv4Address broadcastAddress = Ipv4Address("10.0.0.255");

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    std::vector<Ptr<Socket>> sockets(numVehicles);

    for (uint32_t i = 0; i < numVehicles; i++) {
        sockets[i] = Socket::CreateSocket(vehicles.Get(i), tid);
        sockets[i]->SetAllowBroadcast(true);
    }

    for (uint32_t i = 0; i < numVehicles; i++) {
        std::string vehicleId = "Vehicle-" + std::to_string(i+1);
        Simulator::ScheduleWithContext(
            vehicles.Get(i)->GetId(), Seconds(1.0),
            [&sockets, i, broadcastAddress, vehicleId, interval]() {
                Simulator::Schedule(Seconds(0.0), &SendCamMessage, sockets[i], broadcastAddress, 8080, vehicleId);
                Simulator::Schedule(Seconds(interval), [&sockets, i, broadcastAddress, vehicleId, interval]() {
                    SendCamMessage(sockets[i], broadcastAddress, 8080, vehicleId);
                });
            }
        );
    }

    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11);
    wifiPhy.EnablePcap("vanet-simulation", wifiDevices);

    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
