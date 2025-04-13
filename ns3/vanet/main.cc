#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/yans-wifi-helper.h"

#include "cam-application.h"
#include "geo-networking.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CamBroadcastExample");

int main(int argc, char *argv[]) {
  LogComponentEnable("CamBroadcastExample", LOG_LEVEL_INFO);
  LogComponentEnable("CamApplication", LOG_LEVEL_INFO);

  Simulator::SetImplementation(CreateObject<RealtimeSimulatorImpl>());

  uint32_t nVehicles = 2;
  double simTime = 10.0;
  double camInterval = 0.1;
  bool enablePcap = true;
  std::string pcapFilePrefix = "cam-broadcast";
  uint16_t geoNetRadius = 1000;

  CommandLine cmd;
  cmd.AddValue("nVehicles", "Number of vehicles", nVehicles);
  cmd.AddValue("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue("camInterval", "CAM broadcast interval in seconds", camInterval);
  cmd.AddValue("enablePcap", "Enable PCAP file generation", enablePcap);
  cmd.AddValue("pcapFilePrefix", "Prefix for PCAP files", pcapFilePrefix);
  cmd.AddValue("geoNetRadius", "GeoNetworking broadcast radius in meters", geoNetRadius);
  cmd.Parse(argc, argv);

  NodeContainer vehicles;
  vehicles.Create(nVehicles);

  PacketSocketHelper packetSocketHelper;
  packetSocketHelper.Install(vehicles);

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211a);  // 5 GHz
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                               StringValue("OfdmRate6Mbps"), "ControlMode",
                               StringValue("OfdmRate6Mbps"));

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(wifiChannel.Create());

  WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");  // ad-hoc mode

  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, vehicles);

  if (enablePcap) {
    wifiPhy.EnablePcap(pcapFilePrefix, devices);
  }

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc =
      CreateObject<ListPositionAllocator>();

  for (uint32_t i = 0; i < nVehicles; i++) {
    positionAlloc->Add(Vector(i * 50.0, 0.0, 0.0));
  }

  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(vehicles);

  for (uint32_t i = 0; i < nVehicles; i++) {
    Ptr<ConstantVelocityMobilityModel> mobility =
        vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    mobility->SetVelocity(Vector((i % 5 + 1) * 5.0, 0, 0));
  }

  for (uint32_t i = 0; i < nVehicles; i++) {
    Ptr<CamSender> sender = CreateObject<CamSender>();
    sender->SetVehicleId(i + 1);
    sender->SetInterval(Seconds(camInterval));
    sender->SetBroadcastRadius(geoNetRadius);
    vehicles.Get(i)->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0 + 0.01 * i));
    sender->SetStopTime(Seconds(simTime));

    Ptr<CamReceiver> receiver = CreateObject<CamReceiver>();
    vehicles.Get(i)->AddApplication(receiver);
    receiver->SetStartTime(Seconds(0.0));
    receiver->SetStopTime(Seconds(simTime));
  }

  NS_LOG_INFO("Running simulation...");
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  NS_LOG_INFO("Simulation completed.");

  return 0;
}