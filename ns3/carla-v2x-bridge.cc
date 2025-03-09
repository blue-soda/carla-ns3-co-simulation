/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wave-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <string>
#include <cstring>
#include <atomic>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <atomic>

using json = nlohmann::json;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CarlaV2XBridge");

struct VehicleData {
  uint32_t id;
  double x, y, z, yaw, speed;
};

std::vector<VehicleData> g_vehicleData;
std::mutex g_mutex;
std::atomic<int> g_client_fd(-1);
std::queue<std::string> g_message_queue;
std::mutex g_queue_mutex;

void SendToCarla(const std::string& message) {
  int client_fd = g_client_fd.load();
  if (client_fd < 0) return;

  std::lock_guard<std::mutex> lock(g_queue_mutex);
  g_message_queue.push(message);
}

void ProcessMessageQueue() {
  int client_fd = g_client_fd.load();
  if (client_fd < 0) return;

  std::lock_guard<std::mutex> lock(g_queue_mutex);
  while (!g_message_queue.empty()) {
    std::string msg = g_message_queue.front();
    g_message_queue.pop();
    send(client_fd, msg.c_str(), msg.size(), 0);
  }
}

void ReceiveFromCarla() {
  int server_fd, client_fd;
  sockaddr_in address{};
  int opt = 1;
  char buffer[4096];

  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) return;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(5556);

  if (bind(server_fd, (sockaddr*)&address, sizeof(address)) < 0) return;
  if (listen(server_fd, 3) < 0) return;

  while (true) {
    if ((client_fd = accept(server_fd, nullptr, nullptr)) < 0) continue;
    g_client_fd.store(client_fd);

    while (true) {
      ProcessMessageQueue();
      ssize_t valread = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
      if (valread <= 0) break;
      buffer[valread] = '\0';

      try {
        auto j = json::parse(buffer);
        std::lock_guard<std::mutex> lock(g_mutex);
        g_vehicleData.clear();
        for (const auto& v : j) {
          g_vehicleData.push_back({v["id"], v["position"]["x"], v["position"]["y"], v["position"]["z"], v["rotation"]["yaw"], std::sqrt(v["velocity"]["x"].get<double>() * v["velocity"]["x"].get<double>() + v["velocity"]["y"].get<double>() * v["velocity"]["y"].get<double>())});
        }
      } catch (...) {}
    }
    close(client_fd);
    g_client_fd.store(-1);
  }
  close(server_fd);
}

void UpdateNodePositions(NodeContainer& nodes) {
  std::vector<VehicleData> vehicleData;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    vehicleData = g_vehicleData;
  }

  for (uint32_t i = 0; i < std::min(nodes.GetN(), (uint32_t)vehicleData.size()); ++i) {
    nodes.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(vehicleData[i].x, vehicleData[i].y, vehicleData[i].z));
  }
}

void V2XMessageReceived(std::string context, Ptr<const Packet> packet, double txPower) {
  NS_LOG_INFO("V2X Message Received: " << packet->GetSize() << " bytes at power " << txPower);
}

int main(int argc, char *argv[]) {
    LogComponentEnable("CarlaV2XBridge", LOG_LEVEL_INFO);
    CommandLine cmd;
    uint32_t numVehicles = 10;
    double simTime = 100.0;
    cmd.AddValue("numVehicles", "Number of vehicles", numVehicles);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.Parse(argc, argv);
  
    std::thread receiveThread(ReceiveFromCarla);
    NodeContainer nodes;
    nodes.Create(numVehicles);
  
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
  
    YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
    YansWavePhyHelper wavePhy = YansWavePhyHelper::Default();
    wavePhy.SetChannel(waveChannel.Create());
  
    QosWaveMacHelper waveMac = QosWaveMacHelper::Default();
    WaveHelper waveHelper = WaveHelper::Default();
    NetDeviceContainer devices = waveHelper.Install(wavePhy, waveMac, nodes);
  
    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
  
    Config::ConnectWithoutContext(
        "/NodeList/0/DeviceList/0/$ns3::WaveNetDevice/PhyEntities/0/$ns3::WifiPhy/PhyTxBegin",
        MakeCallback(&V2XMessageReceived));
    
    AnimationInterface anim("carla-v2x-animation.xml");
  
    for (double t = 0.0; t < simTime; t += 0.05) {
      Simulator::Schedule(Seconds(t), &UpdateNodePositions, nodes);
    }
  
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
  
    receiveThread.join();
    return 0;
  }
  