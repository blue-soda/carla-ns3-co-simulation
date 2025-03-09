/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wave-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"

#include <zmq.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CarlaV2XBridge");

// Structure to hold vehicle data
struct VehicleData {
  uint32_t id;
  double x;
  double y;
  double z;
  double yaw;
  double speed;
};

// Global variables
std::vector<VehicleData> g_vehicleData;
std::mutex g_mutex;
bool g_running = true;

// Function to receive data from CARLA
void ReceiveFromCarla() {
  zmq::context_t context(1);
  zmq::socket_t subscriber(context, ZMQ_SUB);
  
  subscriber.connect("tcp://localhost:5556");
  // Replace deprecated setsockopt with set
  subscriber.set(zmq::sockopt::subscribe, "");
  
  NS_LOG_INFO("ZMQ subscriber started, waiting for CARLA data...");
  
  while (g_running) {
    try {
      zmq::message_t message;
      // Store the result of recv to avoid the -Werror=unused-result error
      static_cast<void>(subscriber.recv(message, zmq::recv_flags::none));
      
      std::string data(static_cast<char*>(message.data()), message.size());
      auto j = json::parse(data);
      
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_vehicleData.clear();
        
        for (const auto& vehicle : j) {
          VehicleData vd;
          vd.id = vehicle["id"];
          vd.x = vehicle["position"]["x"];
          vd.y = vehicle["position"]["y"];
          vd.z = vehicle["position"]["z"];
          vd.yaw = vehicle["rotation"]["yaw"];
          
          // Calculate speed from velocity components
          double vx = vehicle["velocity"]["x"];
          double vy = vehicle["velocity"]["y"];
          double vz = vehicle["velocity"]["z"];
          vd.speed = std::sqrt(vx*vx + vy*vy + vz*vz);
          
          g_vehicleData.push_back(vd);
        }
      }
    }
    catch (const std::exception& e) {
      NS_LOG_ERROR("Error parsing CARLA data: " << e.what());
    }
  }
}

// Function to update node positions based on CARLA data
void UpdateNodePositions(NodeContainer& nodes) {
  std::vector<VehicleData> vehicleData;
  
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    vehicleData = g_vehicleData;
  }
  
  if (vehicleData.empty()) {
    return;
  }
  
  // Update node positions based on vehicle data
  for (uint32_t i = 0; i < std::min(nodes.GetN(), (uint32_t)vehicleData.size()); ++i) {
    Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
    mobility->SetPosition(Vector(vehicleData[i].x, vehicleData[i].y, vehicleData[i].z));
  }
}

int main(int argc, char *argv[]) {
  // Enable logging
  LogComponentEnable("CarlaV2XBridge", LOG_LEVEL_INFO);
  
  // Command line arguments
  CommandLine cmd;
  uint32_t numVehicles = 10;
  double simTime = 100.0;
  
  cmd.AddValue("numVehicles", "Number of vehicles to simulate", numVehicles);
  cmd.AddValue("simTime", "Simulation time in seconds", simTime);
  cmd.Parse(argc, argv);
  
  // Start ZMQ thread to receive data from CARLA
  std::thread zmqThread(ReceiveFromCarla);
  
  // Create nodes
  NodeContainer nodes;
  nodes.Create(numVehicles);
  
  // Configure mobility model
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);
  
  // Setup WAVE/802.11p
  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
  YansWavePhyHelper wavePhy = YansWavePhyHelper::Default();
  wavePhy.SetChannel(waveChannel.Create());
  
  // MAC layer
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default();
  WaveHelper waveHelper = WaveHelper::Default();
  
  NetDeviceContainer devices = waveHelper.Install(wavePhy, waveMac, nodes);
  
  // Configure Internet stack
  InternetStackHelper internet;
  internet.Install(nodes);
  
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
  
  // Setup simple application to broadcast messages
  uint16_t wavePort = 9999;
  
  ApplicationContainer apps;
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    // Create UDP socket for broadcasting
    Ptr<Socket> socket = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
    socket->SetAllowBroadcast(true);
    
    // Create simple application
    Ptr<UdpEchoClient> app = CreateObject<UdpEchoClient>();
    app->SetRemote(Ipv4Address("255.255.255.255"), wavePort);
    app->SetAttribute("MaxPackets", UintegerValue(10000));
    app->SetAttribute("Interval", TimeValue(Seconds(0.1)));
    app->SetAttribute("PacketSize", UintegerValue(200));
    
    nodes.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    app->SetStopTime(Seconds(simTime));
    
    apps.Add(app);
  }
  
  // Setup animation (optional)
  AnimationInterface anim("carla-v2x-animation.xml");
  
  // Schedule position updates
  for (double t = 0.0; t < simTime; t += 0.05) {
    Simulator::Schedule(Seconds(t), &UpdateNodePositions, nodes);
  }
  
  // Run simulation
  NS_LOG_INFO("Running simulation for " << simTime << " seconds");
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  
  // Clean up ZMQ thread
  g_running = false;
  zmqThread.join();
  
  NS_LOG_INFO("Simulation completed");
  
  return 0;
}