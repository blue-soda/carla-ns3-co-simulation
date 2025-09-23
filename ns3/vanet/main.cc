#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/yans-wifi-helper.h"

#include "cam-application.h"
#include "carla_vanet.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CarlaFixedIdVanet");

NodeContainer vehicles;
uint32_t nVehicles = 3;
double simTime = 10.0;
double camInterval = 0.1;

std::map<int, int> indexToCarlaId;
std::map<int, int> carlaIdToIndex;
std::atomic indexBindToCarlaId(false);

std::map<int, Vector> latestPositions;
std::map<int, Vector> latestVelocities;
std::map<int, std::vector<int>> latestRequests;
std::mutex dataMutex;
std::atomic firstDataReceived(false);
bool running = true;

void ProcessData_VehiclePosition(const json& vehicleArray) {
  for (const auto &vehicle : vehicleArray) {
    int id = vehicle["carla_id"];
    if(!indexBindToCarlaId){
      int index = vehicle["id"];
      indexToCarlaId[index] = id;
      carlaIdToIndex[id] = index;
    }
    // if (static_cast<uint32_t>(id) >= vehicles.GetN()) continue;
    if(!carlaIdToIndex.count(id)) {
      std::cerr << "[WARN] " << id << " skipped during ProcessData_VehiclePosition\n";
      continue;
    }
    const Vector pos(vehicle["position"]["x"],
                      vehicle["position"]["y"],
                      vehicle["position"]["z"]);
    const Vector vel(vehicle["velocity"]["x"], vehicle["velocity"]["y"], vehicle["velocity"]["z"]);

    latestPositions[id] = pos;
    latestVelocities[id] = vel;

    const double heading = vehicle.value("heading", 0.0);
    const double speed   = vehicle.value("speed", 0.0);
    
    std::cout << "[INFO] Vehicle " << id
              << " @ (" << pos.x << "," << pos.y << ")"
              << " heading: " << heading << "°"
              << " speed: " << speed << " m/s\n";
  }
  if(!indexBindToCarlaId){
    indexBindToCarlaId = true;
    std::cout << "[INFO] indexBindToCarlaId\n";
  }
}

void ProcessData_TransferRequests(const json &requests){
  for (const auto &req : requests) {
    if (!req.contains("source") || !req.contains("target") || !req.contains("size")) {
      std::cerr << "[WARN] transfer request missing fields, skipping\n";
      continue;
    }
    int source = req["source"].get<int>();
    int target = req["target"].get<int>();
    int size = req["size"].get<int>();
    latestRequests[source] = {size, target};

    // if (source < 0 || target < 0 || static_cast<uint32_t>(source) >= vehicles.GetN() ||
    //     static_cast<uint32_t>(target) >= vehicles.GetN()) {
    //   std::cerr << "[WARN] transfer request with invalid node ids: "
    //             << source << "->" << target << ", skipping\n";
    //   continue;
    // }
    if(!carlaIdToIndex.count(source) || !carlaIdToIndex.count(target)) {
      std::cerr << "[WARN] (" << source << ", " << target << ") skipped during ProcessData_TransferRequests\n";
      continue;
    }

    std::cout << "[INFO] Transfer request: " << source << " -> " << target
              << ", size = " << size << " bytes\n";
  }
}

void ProcessData_VehiclesNum(const int &num){
  uint32_t unum = (uint32_t)num;
  if(nVehicles != unum){
    nVehicles = unum;
    InitializeVehicles(unum);
    indexBindToCarlaId = false;
  }
  std::cout << "[INFO] ProcessData_VehiclesNum: " << nVehicles << "\n";
}


void ProcessJsonData(const std::string &data) {
  try {
    json msg = json::parse(data);
    std::lock_guard<std::mutex> lock(dataMutex);

    if (msg.contains("type")) {
      std::string type = msg["type"].get<std::string>();

      if (type == "transfer_requests") {
        if (!msg.contains("transfer_requests") || !msg["transfer_requests"].is_array()) {
          std::cerr << "[ERR] transfer_requests message missing 'transfer_requests' array\n";
          return;
        }
        ProcessData_TransferRequests(msg["transfer_requests"]);
      } 

      else if (type == "vehicles_position"){
        if (!msg.contains("vehicles_position") || !msg["vehicles_position"].is_array()) {
          std::cerr << "[ERR] vehicles_position message missing 'vehicles_position' array\n";
          return;
        }
        ProcessData_VehiclePosition(msg["vehicles_position"]);
      } 

      else if (type == "vehicles_num"){
        if (!msg.contains("vehicles_num")) {
          std::cerr << "[ERR] vehicles_num message missing 'vehicles_num'\n";
          return;
        }
        ProcessData_VehiclesNum(msg["vehicles_num"].get<int>());
      } 

      else{
        std::cerr << "[ERR] Wrong type\n";
      }
    } 

    else {
      std::cerr << "[ERR] Message does not contain 'type'\n";
    }
  }
  catch (json::exception &e) {
    std::cerr << "[ERR] JSON parse error: " << e.what() << "\n";
  }

  if (!firstDataReceived) {
    firstDataReceived = true;
    std::cout << "[INFO] First data received from Carla!\n";
  }
}


void SocketServerThread() {
  sockaddr_in address{};
  int addrlen = sizeof(address);
  char buffer[8192];

  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    perror("socket failed");
    return;
  }

  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(5556);

  if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    perror("bind failed");
    close(server_fd);
    return;
  }

  if (listen(server_fd, 1) < 0) {
    perror("listen failed");
    close(server_fd);
    return;
  }

  std::cout << "[INFO] Waiting for Carla on port 5556...\n";
  const int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&address), reinterpret_cast<socklen_t*>(&addrlen));
  if (client_fd < 0) {
    perror("accept failed");
    close(server_fd);
    return;
  }

  std::cout << "[INFO] Carla connected.\n";

  while (running) {
    memset(buffer, 0, sizeof(buffer));
    const ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);

    if (bytes <= 0) {
      std::cout << "[INFO] Carla disconnected or error.\n";
      break;
    }

    buffer[bytes] = '\0';

    ProcessJsonData(std::string(buffer));
  }

  close(client_fd);
  close(server_fd);
}

void UpdateVehiclePositions() {
  try{
    std::lock_guard lock(dataMutex);
    for (const auto &[id, pos] : latestPositions) {
      if(!carlaIdToIndex.count(id)) {
        std::cerr << "[WARN] " << id << " skipped during UpdateVehiclePositions\n";
        continue;
      }
      int index = carlaIdToIndex[id];
      // if (static_cast<uint32_t>(id) >= vehicles.GetN()) continue;
      Ptr<ConstantVelocityMobilityModel> mobility =
          vehicles.Get(index)->GetObject<ConstantVelocityMobilityModel>();
      if (mobility) {
        mobility->SetPosition(pos);
        mobility->SetVelocity(latestVelocities[id]);
      }
    }
    if (running) {
      Simulator::Schedule(Seconds(0.1), &UpdateVehiclePositions);
    }
  } 
  catch(std::exception &e){ std::cerr << "[ERR] UpdateVehiclePositions error: " << e.what() << "\n"; }
  catch (...) { std::cerr << "[ERR] UpdateVehiclePositions: unknown exception caught\n"; }

}

void SendSimulationEndSignal() {
  int client_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (client_fd < 0) {
    perror("socket failed");
    return;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = inet_addr("127.0.0.1");
  address.sin_port = htons(5557);

  if (connect(client_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    perror("connect failed");
    close(client_fd);
    return;
  }

  const char* end_signal = "{\"type\":\"simulation_end\"}\n";
  if (send(client_fd, end_signal, strlen(end_signal), 0) < 0) {
    perror("send failed");
  }

  close(client_fd);
}

void InitializeVehicles(uint32_t nVehicles = 3){
  vehicles.Create(nVehicles);

  PacketSocketHelper packetSocketHelper;
  packetSocketHelper.Install(vehicles);

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211p);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                               StringValue("OfdmRate6Mbps"), "ControlMode",
                               StringValue("OfdmRate6Mbps"));

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(wifiChannel.Create());

  WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, vehicles);
  wifiPhy.EnablePcap("../../temp/carla-vanet", devices);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  for (uint32_t i = 0; i < nVehicles; i++) {
    positionAlloc->Add(Vector(0, 0, 0));
  }

  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(vehicles);

  for (uint32_t i = 0; i < nVehicles; i++) {
    Ptr<ConstantVelocityMobilityModel> mob =
        vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(0, 0, 0));
  }

  for (uint32_t i = 0; i < nVehicles; i++) {
    Ptr<CamSender> sender = CreateObject<CamSender>();
    sender->SetVehicleId(i + 1);
    sender->SetInterval(Seconds(camInterval));
    sender->SetBroadcastRadius(1000);
    vehicles.Get(i)->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0 + 0.01 * i));
    sender->SetStopTime(Seconds(simTime));

    Ptr<CamReceiver> receiver = CreateObject<CamReceiver>();
    vehicles.Get(i)->AddApplication(receiver);
    receiver->SetStartTime(Seconds(0.0));
    receiver->SetStopTime(Seconds(simTime));
  }
}
int main(int argc, char *argv[]) {

  LogComponentEnable("CamApplication", LOG_LEVEL_INFO);

  CommandLine cmd;
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("camInterval", "CAM interval (s)", camInterval);
  cmd.Parse(argc, argv);

  Simulator::SetImplementation(CreateObject<RealtimeSimulatorImpl>());
  std::thread serverThread(SocketServerThread);
  
  std::cout << "[INFO] Waiting for first Carla data...\n";
  while (!firstDataReceived) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "[INFO] Starting simulation!\n";
  Simulator::Schedule(Seconds(0.1), &UpdateVehiclePositions);
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  running = false;
  serverThread.join();
  SendSimulationEndSignal();
  Simulator::Destroy();

  std::cout << "[INFO] Simulation finished.\n";
  return 0;
}
