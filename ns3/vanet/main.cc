#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/internet-module.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"

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
#include <csignal>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CarlaFixedIdVanet");

NodeContainer vehicles;
uint32_t nVehicles = 0;
double simTime = 10.0;
double camInterval = 0.1;
Time slBearersActivationTime = Seconds(1.0);
Time finalSlBearersActivationTime = slBearersActivationTime + Seconds(0.01);

std::vector<Ptr<CamSender>> senders;
std::vector<Ptr<CamReceiver>> receivers;
std::vector<Ipv4Address> vehicleIps;
std::vector<uint32_t> vehicleL2Ids;

std::map<int, int> indexToCarlaId;
std::map<int, int> carlaIdToIndex;
std::atomic indexBindToCarlaId(false);

std::map<int, Vector> latestPositions;
std::map<int, Vector> latestVelocities;
std::map<int, std::vector<int>> latestRequests;
std::unordered_map<int, TransferRequestSubChannel> latestRequestsSubChannel;

std::mutex msgMutex;
std::mutex dataMutex;
std::atomic firstDataReceived(false);

Ptr<NrSlHelper> NazonoNrSlHelper; //deletion of this var will cause Signals.SIGABRT: 6
bool running = true;
int send_to_carla_fd = -1;
int totalSubChannel = 0;

void ProcessData_VehiclePosition(const json& vehicleArray) {
  for (const auto &vehicle : vehicleArray) {
    int id = vehicle["carla_id"];
    if(!indexBindToCarlaId) {
      int index = vehicle["id"];
      indexToCarlaId[index] = id;
      carlaIdToIndex[id] = index;
      senders[index]->SetVehicleId(id);
      receivers[index]->SetVehicleId(id);
    }
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

    // if(!indexBindToCarlaId) {
    //   const double heading = vehicle.value("heading", 0.0);
    //   const double speed   = vehicle.value("speed", 0.0);

    //   std::cout << "[INFO] Vehicle " << id << " index(" << carlaIdToIndex[id] << ") position"
    //             << " @ (" << pos.x << "," << pos.y << ")"
    //             << " heading: " << heading << "°"
    //             << " speed: " << speed << " m/s\n";
    // }
  }
  indexBindToCarlaId = true;
  std::cout << "[INFO] Received Vehicle Position Msg at " << std::to_string(Simulator::Now().GetMilliSeconds()) << std::endl;
}

void ProcessData_TransferRequests(const json &requests) {
  for (const auto &req : requests) {
    if (!req.contains("source") || !req.contains("target") || !req.contains("size")) {
      std::cerr << "[WARN] transfer request missing fields, skipping\n";
      continue;
    }
    int source = req["source"].get<int>();
    int target = req["target"].get<int>();
    int size = req["size"].get<int>();

    bool contains_rb = req.contains("sc_start") && req.contains("sc_num");
    if(contains_rb) {
      uint8_t sc_start = req["sc_start"].get<uint8_t>();
      uint8_t sc_num = req["sc_num"].get<uint8_t>();
      double tx_power = req.contains("tx_power") ? req["tx_power"].get<double>() : 0.1; // 默认 0.1W=20dBm
      latestRequestsSubChannel[source] = {(uint32_t)size, target, sc_start, sc_num, tx_power};
    } else {
      latestRequests[source] = {size, target};
    }

    if(!carlaIdToIndex.count(source) || !carlaIdToIndex.count(target)) {
      std::cerr << "[WARN] (" << source << ", " << target << ") skipped during ProcessData_TransferRequests\n";
      continue;
    }

    std::cout << "[INFO] Transfer request: " << source << " -> " << target
              << ", size = " << size << " bytes\n";
    
    if(indexBindToCarlaId) {
      int source_index = carlaIdToIndex[source];
      int target_index = carlaIdToIndex[target];
      if(source_index >= (int)senders.size() || target_index >= (int)senders.size()) {
        std::cerr << "[ERR] index out range during ProcessData_TransferRequests, source_index = " 
          << source_index << " id = " << source << ", or target_index = " << target_index << " id = " << target << "\n";
        continue;
      }
      if(senders[source_index]->IsRunning()) {
        if(contains_rb) {
          TransferRequestSubChannel sc_req = latestRequestsSubChannel[source];
          std::cout << "[INFO] sender id: " << source << " sending " << sc_req.size << " bytes to id: " << target << " subChannel_start: " << (uint32_t)sc_req.start << " num: " << (uint32_t)sc_req.num << " tx_power: " << sc_req.tx_power << " W\n";
          CamSenderNR *sender_nr = GetPointer(DynamicCast<CamSenderNR>(senders[source_index]));
          sender_nr->ScheduleCam((uint32_t)sc_req.size, vehicleIps[target_index], sc_req.start, sc_req.num, sc_req.tx_power, vehicleL2Ids[source_index] , vehicleL2Ids[target_index]);
        } else {
          std::cout << "[INFO] sender id: " << source << " sending " << size << " bytes\n";
          // 对于没有指定子信道的情况，仍然使用原有接口
          senders[source_index]->ScheduleCam((uint32_t)size, vehicleIps[target_index]);
        }
      }
    }
  }
  std::cout << "[INFO] Received Transfer Request Msg at " << std::to_string(Simulator::Now().GetMilliSeconds()) << std::endl;
}

void ProcessData_VehiclesNum(const int &num) {
  uint32_t unum = (uint32_t)num;
  if(nVehicles < unum) {
    std::cout << "[INFO] Vehicles number changed: " << nVehicles << " -> " << unum << "\n";
    InitializeVehicles(unum);
    indexBindToCarlaId = false;
  }
  // std::cout << "[INFO] ProcessData_VehiclesNum: " << nVehicles << "\n";
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

      else if (type == "vehicles_position") {
        if (!msg.contains("vehicles_position") || !msg["vehicles_position"].is_array()) {
          std::cerr << "[ERR] vehicles_position message missing 'vehicles_position' array\n";
          return;
        }
        ProcessData_VehiclePosition(msg["vehicles_position"]);
      } 

      else if (type == "vehicles_num") {
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
    std::cerr << "[ERR] JSON parse error: " << e.what() << "; input: " << data << "\n"; 
  }

  if (!firstDataReceived) {
    firstDataReceived = true;
    std::cout << "[INFO] First data received from Carla!\n";
  }
}

void ProcessReceivedData(std::string &receive_buffer) {
  size_t pos;
  while ((pos = receive_buffer.find("\n\r")) != std::string::npos) {
    std::string complete_message = receive_buffer.substr(0, pos);
    receive_buffer.erase(0, pos + 1);
    if (!complete_message.empty()) {
      try {
          ProcessJsonData(complete_message);
      } catch (const std::exception& e) {
          std::cerr << "[ERR] Failed to process message: " << e.what() << std::endl;
          std::cerr << "Raw message: " << complete_message << std::endl;
      }
    }
  }
}

void SocketReceiverServerThread() {
  sockaddr_in address{};
  int addrlen = sizeof(address);
  char buffer[8192];
  std::string receive_buffer_string;

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

  std::cout << "[INFO] Carla connected on port 5556.\n";

  while (running) {
    memset(buffer, 0, sizeof(buffer));
    const ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);

    if (bytes <= 0) {
      std::cout << "[INFO] Carla disconnected or error on port 5556.\n";
      break;
    }

    buffer[bytes] = '\0';

    receive_buffer_string = std::string(buffer);
    ProcessReceivedData(receive_buffer_string);
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
      uint32_t index = (uint32_t)carlaIdToIndex[id];
      if (index >= vehicles.GetN()) { 
        std::cerr << "[ERR] (" << id << "-> "<< index << ") skipped during UpdateVehiclePositions\n";
        continue;
      }
      Ptr<ConstantVelocityMobilityModel> mobility =
          vehicles.Get(index)->GetObject<ConstantVelocityMobilityModel>();
      if (mobility) {
        mobility->SetPosition(pos);
        mobility->SetVelocity(latestVelocities[id]);
      }
    }
    if (running) {
      Simulator::Schedule(Seconds(0.05), &UpdateVehiclePositions);
    }
  } 
  catch(std::exception &e) { std::cerr << "[ERR] UpdateVehiclePositions error: " << e.what() << "\n"; }
  catch (...) { std::cerr << "[ERR] UpdateVehiclePositions: unknown exception caught\n"; }

}

void SendSimulationEndSignal() {
  std::string msg = R"({"type": "simulation_end"})";
  SendMsgToCarla(msg, false);
}

void SendMsgToCarla(const std::string &msg, bool try_reconnect = true) {
  std::lock_guard<std::mutex> lock(msgMutex);
  std::string msg_with_delimiter = msg + "\r\n";
  std::cout << "[INFO] SendMsgToCarla: " << msg << ", send_to_carla_fd: " << send_to_carla_fd << "\n";
  if(send_to_carla_fd < 0 && try_reconnect) { //if not connected, try to connect for once
    SocketSenderServerConnect();
    if(send_to_carla_fd < 0) {
      std::cerr << "[ERR] send_to_carla_fd is not connected\n";
      return;
    }
  }
  if (send(send_to_carla_fd, msg_with_delimiter.c_str(), msg_with_delimiter.size(), 0) < 0 && try_reconnect) {
    std::cerr << "[ERR] send failed, send_to_carla_fd = " << send_to_carla_fd << "\n";
    perror("[ERR] send failed");
    SocketSenderServerConnect();
  }
}

void SocketSenderServerConnect() {
  if (send_to_carla_fd >= 0) {
      close(send_to_carla_fd);
      send_to_carla_fd = -1;
      std::cout << "[INFO] Previous connection to Carla closed.\n";
  }

  std::cout << "[INFO] Connecting to Carla on port 5557...\n";
  send_to_carla_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (send_to_carla_fd < 0) {
    perror("[ERR] socket failed");
    return;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = inet_addr("127.0.0.1");
  address.sin_port = htons(5557);

  if (connect(send_to_carla_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    perror("[ERR] connect failed");
    close(send_to_carla_fd);
    send_to_carla_fd = -1;
    return;
  }
  std::cout << "[INFO] Connected to Carla on port 5557.\n";
}

void SocketSenderServerDisconnect() {
  if (send_to_carla_fd != -1) {
    close(send_to_carla_fd);
    send_to_carla_fd = -1;
    std::cout << "[INFO] Disconnected from Carla on port 5557.\n";
  }
}

void HandleSigInt(int signum) {
    std::cout << "\n[INFO] Received SIGINT (Ctrl+C), exiting gracefully...\n";
    Simulator::Stop();
    running = false;
    SocketSenderServerDisconnect();
}

int main(int argc, char *argv[]) {

  // LogComponentEnable("NrSlUeMacSchedulerFixedMcs", LOG_ALL);
  // LogComponentEnable("CamApplication", LOG_ALL);
  // LogComponentEnable("NrSlUeMac", LOG_LEVEL_INFO);
  // LogComponentEnable("NrUeNetDevice", LOG_LEVEL_INFO);
  signal(SIGPIPE, SIG_IGN);
  signal(SIGINT, HandleSigInt);

  CommandLine cmd;
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("camInterval", "CAM interval (s)", camInterval);
  cmd.Parse(argc, argv);

  Simulator::SetImplementation(CreateObject<RealtimeSimulatorImpl>());
  std::thread serverReceiverThread(SocketReceiverServerThread);
  
  std::cout << "[INFO] Waiting for first Carla data...\n";
  while (!firstDataReceived) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  SocketSenderServerConnect();

  std::cout << "[INFO] Starting simulation!\n";
  Simulator::Schedule(Seconds(0.1), &UpdateVehiclePositions);
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  running = false;
  serverReceiverThread.join();
  SendSimulationEndSignal();
  SocketSenderServerDisconnect();
  Simulator::Destroy();
  return 0;
}


void InitializeVehicles_DSRC(uint32_t n_vehicles = 3){
  if(nVehicles >= n_vehicles) return;

  nVehicles = n_vehicles;
  vehicles = NodeContainer();
  vehicles.Create(n_vehicles);
  senders.clear();
  receivers.clear();
  vehicleIps.clear();

  std::cout << "[INFO] Installing Wifi\n";
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211p);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("OfdmRate54Mbps"),
                                 "ControlMode",
                                 StringValue("OfdmRate54Mbps"));

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(wifiChannel.Create());

  WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, vehicles);
  // wifiPhy.EnablePcap("../../temp/carla-vanet", devices);

  // std::cout << "[INFO] Installing PacketSocket\n";
  // PacketSocketHelper packetSocketHelper;
  // packetSocketHelper.Install(vehicles);
  std::cout << "[INFO] Installing internet\n";
  InternetStackHelper internet;
  internet.Install(vehicles);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.0.0", "255.255.255.0");
  Ipv4InterfaceContainer staIfs = ipv4.Assign(devices);
  for(uint32_t i = 0; i < vehicles.GetN(); i++) {
    Ptr<Ipv4> ipv4 = vehicles.Get(i)->GetObject<Ipv4>();
    Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal();
    std::cout << "[INFO] Vehicle " << i << " IP address: " << addr << "\n";
  }

  std::cout << "[INFO] Installing MobilityModel\n";
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  for (uint32_t i = 0; i < vehicles.GetN(); i++) {
    positionAlloc->Add(Vector(i * 10.0, 0, 0));
  }

  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(vehicles);

  for (uint32_t i = 0; i < vehicles.GetN(); i++) {
    Ptr<ConstantVelocityMobilityModel> mob =
        vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(0, 0, 0));
  }

  std::cout << "[INFO] Installing Cam applications\n";
  for (uint32_t i = 0; i < vehicles.GetN(); i++) {
    Ptr<Ipv4> ipv4 = vehicles.Get(i)->GetObject<Ipv4>();
    Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal();
    vehicleIps.push_back(addr);

    Ptr<CamSenderDSRC> sender = CreateObject<CamSenderDSRC>();
    sender->SetVehicleId(i + 1);
    sender->SetIp(addr);
    sender->SetInterval(Seconds(camInterval));
    sender->SetBroadcastRadius(1000);
    vehicles.Get(i)->AddApplication(sender);
    sender->SetStartTime(Seconds(0.0));
    sender->SetStopTime(Seconds(simTime));
    senders.push_back(sender);

    Ptr<CamReceiverDSRC> receiver = CreateObject<CamReceiverDSRC>();
    receiver->SetVehicleId(i + 1);
    receiver->SetIp(addr);
    vehicles.Get(i)->AddApplication(receiver);
    receiver->SetStartTime(Seconds(0.0));
    receiver->SetStopTime(Seconds(simTime));
    receiver->SetReplyFunction([](const std::string& msg) {
      return SendMsgToCarla(msg);
    });
    receivers.push_back(receiver);
  }

  for (uint32_t i = 0; i < vehicles.GetN(); ++i)
  {
      PrintRoutingTable(vehicles.Get(i));
  }
  std::cout << "[INFO] DSRC vehiclesInitialized with " << vehicles.GetN() << " nodes.\n";
}

void PrintRoutingTable (Ptr<Node> node)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    Ipv4StaticRoutingHelper helper;
    Ptr<Ipv4StaticRouting> routing = helper.GetStaticRouting(ipv4);
    uint32_t nRoutes = routing->GetNRoutes();
    std::cout << "Routing table of node " << node->GetId() << ":\n";
    for (uint32_t i = 0; i < nRoutes; i++)
    {
        Ipv4RoutingTableEntry route = routing->GetRoute(i);
        std::cout << route.GetDest() << "\t"
                  << route.GetGateway() << "\t"
                  << route.GetDestNetworkMask() << "\t"
                  << route.GetInterface() << "\n";
    }
}

void InitializeVehicles_NR_V2X_Mode2(uint32_t n_vehicles = 3)
{
    if(nVehicles >= n_vehicles) return;

    nVehicles = n_vehicles;
    vehicles = NodeContainer();
    vehicles.Create(n_vehicles);
    senders.clear();
    receivers.clear();
    vehicleIps.clear();
    vehicleL2Ids = std::vector<uint32_t>(n_vehicles, 0);

    /*
     * Assign mobility to the UEs.
     *  1. Set mobility model type.
     *  2. Assign position to the UEss
     *  3. Install mobility model
     */
    std::cout << "[INFO] Installing MobilityModel\n";
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < vehicles.GetN(); i++) {
        positionAlloc->Add(Vector(0, 0, 0)); // 初始位置
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(vehicles);

    for (uint32_t i = 0; i < vehicles.GetN(); i++) {
        vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, 0, 0));
    }
    
    // 提前安装网络栈，确保TrafficControlLayer在NR设备安装前正确初始化
    std::cout << "[INFO] Installing Internet Stack\n";
    InternetStackHelper internet;
    internet.Install(vehicles);

    // NR parameters. We will take the input from the command line, and then we
    // will pass them inside the NR module.
    uint16_t numerologyBwpSl = 1;
    double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
    // uint16_t bandwidthBandSl = 400;         // Multiple of 100 KHz; 400 = 40 MHz
    // uint16_t bandwidthBandSl = 720 + 40;
    uint16_t bandwidthBandSl = 400;
    double txPower = 23;                    // dBm
    uint16_t SlSubchannelSize = 10;
    totalSubChannel = floor( (bandwidthBandSl * 100) / (15 * pow(2, numerologyBwpSl) * 12) / SlSubchannelSize );
    std::cout << "[INFO] NR V2X Mode 2: totalSubChannel = " << totalSubChannel << "\n";
    /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - EpcHelper, which will setup the core network
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
     */
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetEpcHelper(epcHelper);

    /*
     * Spectrum division. We create one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by the input parameters.
     * We will use the StreetCanyon channel modeling.
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
     * creates a single BWP per CC
     */
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     BandwidthPartInfo::V2V_Highway);

    // By using the configuration created, it is time to make the operation bands
    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);

    /*
     * The configured spectrum division is:
     * ------------Band1--------------
     * ------------CC1----------------
     * ------------BwpSl--------------
     */

    /*
     * Attributes of ThreeGppChannelModel still cannot be set in our way.
     * TODO: Coordinate with Tommaso
     */
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    /*
     * Initialize channel and pathloss, plus other things inside bandSl. If needed,
     * the band configuration can be done manually, but we leave it for more
     * sophisticated examples. For the moment, this method will take care
     * of all the spectrum initialization needs.
     */
    nrHelper->InitializeOperationBand(&bandSl);
    allBwps = CcBwpCreator::GetAllBwps({bandSl});

    std::cout << "实际带宽配置：" << bandSl.m_channelBandwidth << " Hz" << std::endl;
    std::cout << "OperationBand频率：" << bandSl.m_centralFrequency << " Hz, (" <<  bandSl.m_lowerFrequency << " - " << bandSl.m_higherFrequency << ")" << std::endl;
      for (const auto& bwp : allBwps) {
        std::unique_ptr<ns3::BandwidthPartInfo>& bwpPtr = bwp.get();
        std::cout << "频率" << bwpPtr->m_centralFrequency << " Hz, (" <<  bwpPtr->m_lowerFrequency << " - " << bwpPtr->m_higherFrequency << ")" << std::endl;
      } 
    /*
     * allBwps contains all the spectrum configuration needed for the nrHelper.
     *
     * Now, we can setup the attributes. We can have three kind of attributes:
     * (i) parameters that are valid for all the bandwidth parts and applies to
     * all nodes, (ii) parameters that are valid for all the bandwidth parts
     * and applies to some node only, and (iii) parameters that are different for
     * every bandwidth parts. The approach is:
     *
     * - for (i): Configure the attribute through the helper, and then install;
     * - for (ii): Configure the attribute through the helper, and then install
     * for the first set of nodes. Then, change the attribute through the helper,
     * and install again;
     * - for (iii): Install, and then configure the attributes by retrieving
     * the pointer needed, and calling "SetAttribute" on top of such pointer.
     *
     */

    // Packet::EnableChecking();
    // Packet::EnablePrinting();

    /*
     *  Case (i): Attributes valid for all the nodes
     */
    // Core latency
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    /*
     * Antennas for all the UEs
     * We are not using beamforming in SL, rather we are using
     * quasi-omnidirectional transmission and reception, which is the default
     * configuration of the beams.
     */
    // nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    // nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPower));

    // NR Sidelink attribute of UE MAC, which are would be common for all the UEs
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    // nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(true));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(1));
    // nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(1));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));
    
    uint8_t bwpIdForGbrMcptt = 0;

    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    // following parameter has no impact at the moment because:
    // 1. No support for PQI based mapping between the application and the LCs
    // 2. No scheduler to consider PQI
    // However, till such time all the NR SL examples should use GBR_MC_PUSH_TO_TALK
    // because we hard coded the PQI 65 in UE RRC.
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    /*
     * We miss many other parameters. By default, not configuring them is equivalent
     * to use the default values. Please, have a look at the documentation to see
     * what are the default values for all the attributes you are not seeing here.
     */

    /*
     * Case (ii): Attributes valid for a subset of the nodes
     */

    // NOT PRESENT IN THIS SIMPLE EXAMPLE

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */
    NetDeviceContainer ueVoiceNetDev = nrHelper->InstallUeDevice(vehicles, allBwps);

    /*
     * Case (iii): Go node for node and change the attributes we have to setup
     * per-node.
     */

    // When all the configuration is done, explicitly call UpdateConfig ()
    for (auto it = ueVoiceNetDev.Begin(); it != ueVoiceNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    /*
     * Configure Sidelink. We create the following helpers needed for the
     * NR Sidelink, i.e., V2X simulation:
     * - NrSlHelper, which will configure the UEs protocol stack to be ready to
     *   perform Sidelink related procedures.
     * - EpcHelper, which takes care of triggering the call to EpcUeNas class
     *   to establish the NR Sidelink bearer (s). We note that, at this stage
     *   just communicate the pointer of already instantiated EpcHelper object,
     *   which is the same pointer communicated to the NrHelper above.
     */
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    // Put the pointers inside NrSlHelper
    nrSlHelper->SetEpcHelper(epcHelper);

    /*
     * Set the SL error model and AMC
     * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
     *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
     * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
     */
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    /*
     * Set the SL scheduler attributes
     * In this example we use NrSlUeMacSchedulerFixedMcs scheduler, which uses
     * a fixed MCS value
     */
    // nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetNrSlSchedulerTypeId(ns3::NrSlUeMacSchedulerManual::GetTypeId());

    // nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(20));

    /*
     * Very important method to configure UE protocol stack, i.e., it would
     * configure all the SAPs among the layers, setup callbacks, configure
     * error model, configure AMC, and configure ChunkProcessor in Interference
     * API.
     */
    nrSlHelper->PrepareUeForSidelink(ueVoiceNetDev, bwpIdContainer);

    /*
     * Start preparing for all the sub Structs/RRC Information Element (IEs)
     * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
     * hold all the pre-configuration related to Sidelink.
     */

    // SlResourcePoolNr IE
    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    // get it from pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    /*
     * Above pool factory is created to help the users of the simulator to create
     * a pool with valid default configuration. Please have a look at the
     * constructor of NrSlCommResourcePoolFactory class.
     *
     * In the following, we show how one could change those default pool parameter
     * values as per the need.
     */
    // std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    std::vector<std::bitset<1>> slBitmap = {1,1,1,1,1,1,1,1,1,1,1,1}; // 12个时隙全可用
    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100); // T0 in ms
    ptrFactory->SetSlSelectionWindow(5);
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(SlSubchannelSize);
    // ptrFactory->SetSlMaxNumPerReserve(3);
    ptrFactory->SetSlMaxNumPerReserve(1);
    // std::list<uint16_t> resourceReservePeriodList = {0, 100}; // in ms
    std::list<uint16_t> resourceReservePeriodList = {0}; //, 10, 20, 50, 100}; 
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, we can create the pool
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    // Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    // Pool id, ranges from 0 to 15
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    // Array for pools, we insert the pool in the array as per its poolId
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = numerologyBwpSl;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg = 1;
    bwp.bandwidth = bandwidthBandSl;

    // Configure the SlBwpGeneric IE
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Configure the SlBwpConfigCommonNr IE
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Configure the SlFreqConfigCommonNr IE, which hold the array to store
    // the configuration of all Sidelink BWP (s).
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    // Array for BWPs. Here we will iterate over the BWPs, which
    // we want to use for SL.
    for (const auto& it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    // tddUlDlConfigCommon.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";
    tddUlDlConfigCommon.tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
    // tddUlDlConfigCommon.tddPattern = "F|F|F|F|F|F|F|F|F|F|F|F|";

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    // slUeSelectedPreConfig.slProbResourceKeep = 0;
    slUeSelectedPreConfig.slProbResourceKeep = 0.8;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 1;
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    /*
     * Finally, configure the SidelinkPreconfigNr This is the main structure
     * that needs to be communicated to NrSlUeRrc class
     */
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Communicate the above pre-configuration to the NrSlHelper
    nrSlHelper->InstallNrSlPreConfiguration(ueVoiceNetDev, slPreConfigNr);

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(100 * 1024 * 1024));

    /****************************** End SL Configuration ***********************/

    /*
     * Fix the random streams
     */
    int64_t stream = 1;
    stream += nrHelper->AssignStreams(ueVoiceNetDev, stream);
    stream += nrSlHelper->AssignStreams(ueVoiceNetDev, stream);
    
    // 网络栈已提前安装，这里只需要分配流
    stream += internet.AssignStreams(vehicles, stream);
    
    /* 
     * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
     * configured time.
     *
     * This example supports IPV4 and IPV6
     */
    // Ipv4Address groupAddress4("225.0.0.0"); // use multicast address as destination

    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(ueVoiceNetDev);

    // set the default gateway for the UE
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t u = 0; u < vehicles.GetN(); ++u)
    {
        Ptr<Node> ueNode = vehicles.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    Ipv4Address groupAddress4("225.0.0.0"); // use multicast address as destination
    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Unicast;
    slInfo.m_dstL2Id = 255;
    slInfo.m_rri = MilliSeconds(5);
    // slInfo.m_rri = MilliSeconds(1);
    slInfo.m_pdb = Seconds(0);
    slInfo.m_harqEnabled = false;
    slInfo.m_dynamic = true;

    for(uint32_t i = 0; i < ueIpIface.GetN(); ++i)
    {
      NetDeviceContainer receiver;
      receiver.Add(ueVoiceNetDev.Get(i));
      Ipv4Address destIp = ueIpIface.GetAddress(i);
      uint32_t dstL2Id = DynamicCast<NrUeNetDevice>(ueVoiceNetDev.Get(i))->GetMac(0)->GetObject<NrSlUeMac>()->GetSrcL2Id();
      slInfo.m_dstL2Id = dstL2Id;
      vehicleL2Ids[i] = dstL2Id;
      Ptr<LteSlTft> tftUnicastReceiver = Create<LteSlTft>(
        LteSlTft::Direction::RECEIVE,
        destIp, 
        slInfo
      );
      std::cout << "SLINFO.dstL2Id: " << slInfo.m_dstL2Id << std::endl;
      nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, receiver, tftUnicastReceiver);
      for (uint32_t j = 0; j < ueIpIface.GetN(); ++j)
      {
        if (i == j)
          continue;
        NetDeviceContainer sender;
        sender.Add(ueVoiceNetDev.Get(j));
        Ptr<LteSlTft> tftUnicastSender = Create<LteSlTft>(
            LteSlTft::Direction::TRANSMIT,
            destIp, 
            slInfo
        );
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, sender, tftUnicastSender);
        // nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, ueVoiceNetDev, tftUnicast);
      }
    }

    NazonoNrSlHelper = nrSlHelper;
    // // Install Application
    std::cout << "[INFO] Installing Application\n";    
    for (uint32_t i = 0; i < vehicles.GetN(); i++) {
        Ipv4Address ip = ueIpIface.GetAddress(i);
        std::cout << "[INFO] Vehicle " << i << " IP address: " << ip << "\n";
        vehicleIps.push_back(ip);

        Ptr<CamSenderNR> sender = CreateObject<CamSenderNR>();
        sender->SetVehicleId(i+1);
        sender->SetInterval(Seconds(camInterval));
        sender->SetIp(ip);
        vehicles.Get(i)->AddApplication(sender);
        sender->SetStartTime(slBearersActivationTime);
        sender->SetStopTime(Seconds(simTime));
        senders.push_back(sender);

        Ptr<CamReceiverNR> receiver = CreateObject<CamReceiverNR>();
        receiver->SetVehicleId(i+1);
        receiver->SetIp(ip);

        vehicles.Get(i)->AddApplication(receiver);
        receiver->SetStartTime(slBearersActivationTime);
        receiver->SetStopTime(Seconds(simTime));
        receiver->SetReplyFunction([i](const std::string& msg) {
          // Simulator::Schedule(MilliSeconds(i), [msg] { SendMsgToCarla(msg); });
          return SendMsgToCarla(msg);
        });
        receivers.push_back(receiver);
    }

    Ptr<Node> node = vehicles.Get(0);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

    Ptr<Ipv4ListRouting> listRouting = DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
    if (listRouting)
    {
        for (uint32_t j = 0; j < listRouting->GetNRoutingProtocols(); ++j)
        {
            int16_t priority;
            Ptr<Ipv4RoutingProtocol> subRouting = listRouting->GetRoutingProtocol(j, priority);
            Ptr<Ipv4StaticRouting> staticRouting = DynamicCast<Ipv4StaticRouting>(subRouting);
            if (staticRouting)
            {
                Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(&std::cout);
                std::cout << "==== Routing table for Node " << node->GetId() << " ====" << std::endl;
                staticRouting->PrintRoutingTable(stream);
            }
        }
    }

    std::cout << "[INFO] NR-V2X Mode2 vehicles initialized: " << vehicles.GetN() << " UE nodes.\n";
}
