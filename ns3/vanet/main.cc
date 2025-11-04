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
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-sl-ue-mac-scheduler-lcg.h"

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
uint32_t nVehicles = 0;
double simTime = 10.0;
double camInterval = 0.1;

std::vector<Ptr<CamSender>> senders;
std::vector<Ptr<CamReceiver>> receivers;
std::vector<Ipv4Address> vehicleIps;

std::map<int, int> indexToCarlaId;
std::map<int, int> carlaIdToIndex;
std::atomic indexBindToCarlaId(false);

std::map<int, Vector> latestPositions;
std::map<int, Vector> latestVelocities;
std::map<int, std::vector<int>> latestRequests;
std::mutex dataMutex;
std::atomic firstDataReceived(false);
bool running = true;

int send_to_carla_fd = -1;

void ProcessData_VehiclePosition(const json& vehicleArray) {
  for (const auto &vehicle : vehicleArray) {
    int id = vehicle["carla_id"];
    if(!indexBindToCarlaId){
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

    // if(!indexBindToCarlaId){
    //   const double heading = vehicle.value("heading", 0.0);
    //   const double speed   = vehicle.value("speed", 0.0);

    //   std::cout << "[INFO] Vehicle " << id << " index(" << carlaIdToIndex[id] << ") position"
    //             << " @ (" << pos.x << "," << pos.y << ")"
    //             << " heading: " << heading << "°"
    //             << " speed: " << speed << " m/s\n";
    // }
  }
  indexBindToCarlaId = true;
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

    if(!carlaIdToIndex.count(source) || !carlaIdToIndex.count(target)) {
      std::cerr << "[WARN] (" << source << ", " << target << ") skipped during ProcessData_TransferRequests\n";
      continue;
    }

    std::cout << "[INFO] Transfer request: " << source << " -> " << target
              << ", size = " << size << " bytes\n";
    
    if(indexBindToCarlaId){
      int source_index = carlaIdToIndex[source];
      int target_index = carlaIdToIndex[target];
      if(source_index >= (int)senders.size() || target_index >= (int)senders.size()){
        std::cerr << "[ERR] index out range during ProcessData_TransferRequests, source_index = " 
          << source_index << " id = " << source << ", or target_index = " << target_index << " id = " << target << "\n";
        continue;
      }
      if(senders[source_index]->IsRunning()){
        std::cout << "[INFO] sender index: " << source_index << " id: " << source << " sending " << size << " bytes\n";
        senders[source_index]->ScheduleCam((uint32_t)size, vehicleIps[target_index]);
      }
    }
  }
}

void ProcessData_VehiclesNum(const int &num){
  uint32_t unum = (uint32_t)num;
  if(nVehicles < unum){
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

    // ProcessJsonData(std::string(buffer));
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
      if (index >= vehicles.GetN()){ 
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
  catch(std::exception &e){ std::cerr << "[ERR] UpdateVehiclePositions error: " << e.what() << "\n"; }
  catch (...) { std::cerr << "[ERR] UpdateVehiclePositions: unknown exception caught\n"; }

}

void SendSimulationEndSignal() {
  std::string msg = R"({"type": "simulation_end"})";
  SendMsgToCarla(msg);
}

void SendMsgToCarla(const std::string &msg) {
  std::cout << "[INFO] SendMsgToCarla: " << msg << ", send_to_carla_fd: " << send_to_carla_fd << "\n";
  if(send_to_carla_fd < 0){ //if not connected, try to connect for once
    SocketSenderServerConnect();
    if(send_to_carla_fd < 0){
      std::cerr << "[ERR] send_to_carla_fd is not connected\n";
      return;
    }
  }
  if (send(send_to_carla_fd, msg.c_str(), msg.size(), 0) < 0) {
    std::cerr << "[ERR] send failed, send_to_carla_fd = " << send_to_carla_fd << "\n";
    perror("[ERR] send failed");
  }
}


void SocketSenderServerConnect() {
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
  wifiPhy.EnablePcap("../../temp/carla-vanet", devices);

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
    positionAlloc->Add(Vector(0, 0, 0));
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


/*
 * Global variables to count TX/RX packets and bytes.
 */

uint32_t rxByteCounter = 0; //!< Global variable to count RX bytes
uint32_t txByteCounter = 0; //!< Global variable to count TX bytes
uint32_t rxPktCounter = 0;  //!< Global variable to count RX packets
uint32_t txPktCounter = 0;  //!< Global variable to count TX packets

/**
 * \brief Method to listen the packet sink application trace Rx.
 * \param packet The packet
 */
void
ReceivePacket(Ptr<const Packet> packet, const Address&)
{
    rxByteCounter += packet->GetSize();
    rxPktCounter++;
}

/**
 * \brief Method to listen the transmitting application trace Tx.
 * \param packet The packet
 */
void
TransmitPacket(Ptr<const Packet> packet)
{
    txByteCounter += packet->GetSize();
    txPktCounter++;
}


/*
 * Global variable used to compute PIR
 */
uint64_t pirCounter =
    0; //!< counter to count how many time we computed the PIR. It is used to compute average PIR
Time lastPktRxTime; //!< Global variable to store the RX time of a packet
Time pir;           //!< Global variable to store PIR value

/**
 * \brief This method listens to the packet sink application trace Rx.
 * \param packet The packet
 * \param from The address of the transmitter
 */
void
ComputePir(Ptr<const Packet> packet, const Address&)
{
    if (pirCounter == 0 && lastPktRxTime.GetSeconds() == 0.0)
    {
        // this the first packet, just store the time and get out
        lastPktRxTime = Simulator::Now();
        return;
    }
    pir = pir + (Simulator::Now() - lastPktRxTime);
    lastPktRxTime = Simulator::Now();
    pirCounter++;
}

Time slBearersActivationTime = Seconds(1.0);
Time finalSlBearersActivationTime = slBearersActivationTime + Seconds(0.01);
Time finalSimTime = Seconds(simTime) + finalSlBearersActivationTime;
double dataRateBe = 108 * 1024;  //108Mbps
uint32_t udpPacketSizeBe = 200;
std::string dataRateBeString = std::to_string(dataRateBe) + "kb/s";
double realAppStart =
    finalSlBearersActivationTime.GetSeconds() +
    ((double)udpPacketSizeBe * 8.0 / (DataRate(dataRateBeString).GetBitRate()));
double appStopTime = (finalSimTime).GetSeconds();

void InitializeVehicles_NR_V2X_Mode2(uint32_t n_vehicles = 3)
{
    if(nVehicles >= n_vehicles) return;

    nVehicles = n_vehicles;
    vehicles = NodeContainer();
    vehicles.Create(n_vehicles);
    senders.clear();
    receivers.clear();
    vehicleIps.clear();

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

    // NR parameters. We will take the input from the command line, and then we
    // will pass them inside the NR module.
    uint16_t numerologyBwpSl = 2;
    double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
    uint16_t bandwidthBandSl = 400;         // Multiple of 100 KHz; 400 = 40 MHz
    double txPower = 23;                    // dBm

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
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
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
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    // nrSlHelper->SetNrSlSchedulerTypeId(ns3::NrSlUeMacSchedulerLCG::GetTypeId());

    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));

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
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100); // T0 in ms
    ptrFactory->SetSlSelectionWindow(5);
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(50);
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> resourceReservePeriodList = {0, 100}; // in ms
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
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    slUeSelectedPreConfig.slProbResourceKeep = 0;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
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

    /****************************** End SL Configuration ***********************/

    /*
     * Fix the random streams
     */
    int64_t stream = 1;
    stream += nrHelper->AssignStreams(ueVoiceNetDev, stream);
    stream += nrSlHelper->AssignStreams(ueVoiceNetDev, stream);

    /* 
     * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
     * configured time.
     *
     * This example supports IPV4 and IPV6
     */

    InternetStackHelper internet;
    internet.Install(vehicles);
    stream += internet.AssignStreams(vehicles, stream);
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
    slInfo.m_rri = MilliSeconds(100);
    // slInfo.m_rri = MicroSeconds(10);
    slInfo.m_pdb = Seconds(0);
    slInfo.m_harqEnabled = false;
    slInfo.m_dynamic = false;

    for(uint32_t i = 0; i < ueIpIface.GetN(); ++i)
    {
      NetDeviceContainer receiver;
      receiver.Add(ueVoiceNetDev.Get(i));
      Ipv4Address destIp = ueIpIface.GetAddress(i);
      slInfo.m_dstL2Id = DynamicCast<NrUeNetDevice>(ueVoiceNetDev.Get(i))->GetMac(0)->GetObject<NrSlUeMac>()->GetSrcL2Id();
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
    // slInfo.m_dstL2Id = 255;
    // Ptr<LteSlTft> tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, slInfo);
    // // Set Sidelink bearers
    // nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, ueVoiceNetDev, tft);

    // // OnOffHelper sidelinkClient = OnOffHelper("ns3::UdpSocketFactory", remoteAddress);
    // UdpClientHelper sidelinkClient(remoteAddress, port);
    // sidelinkClient.SetAttribute("MaxPackets", UintegerValue(5)); 
    // sidelinkClient.SetAttribute("Interval", TimeValue(Seconds(2.0))); // 立即发送 (启动后)
    // sidelinkClient.SetAttribute("PacketSize", UintegerValue(5000)); // 设置包大小
    // // sidelinkClient.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    // // std::cout << "Data rate " << DataRate(dataRateBeString) << std::endl;
    // // sidelinkClient.SetConstantRate(DataRate(dataRateBeString), udpPacketSizeBe);

    // PacketSinkHelper sidelinkSink("ns3::UdpSocketFactory", localAddress);
    // sidelinkSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    // // Install Application
    std::cout << "[INFO] Installing Application\n";    
    for (uint32_t i = 0; i < vehicles.GetN(); i++) {
        Ipv4Address ip = ueIpIface.GetAddress(i);
        std::cout << "[INFO] Vehicle " << i << " IP address: " << ip << "\n";
        vehicleIps.push_back(ip);

        // ApplicationContainer clientApps = sidelinkClient.Install(vehicles.Get(i));
        // clientApps.Start(finalSlBearersActivationTime);
        // clientApps.Stop(finalSimTime);

        // ApplicationContainer serverApps = sidelinkSink.Install(vehicles.Get(i));;
        // serverApps.Start(finalSlBearersActivationTime);

        Ptr<CamSenderNR> sender = CreateObject<CamSenderNR>();
        sender->SetVehicleId(i+1);
        sender->SetInterval(Seconds(camInterval));
        sender->SetIp(ip);

        // sender->SetClientApp(clientApps);
            // Output app start, stop and duration

        std::cout << "App start time " << realAppStart << " sec" << std::endl;
        std::cout << "App stop time " << appStopTime << " sec" << std::endl;

        sender->SetNrSlHelper(nrSlHelper);
        // sender->SetBroadcastAddress(groupAddress4, port);
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
        // receiver->SetReplyFunction([](const std::string& msg) {
        //   return SendMsgToCarla(msg);
        // });
        receiver->SetReplyFunction([i](const std::string& msg) {
          Simulator::Schedule(MilliSeconds(i), [msg] { SendMsgToCarla(msg); });
        });
        receivers.push_back(receiver);

        // // Trace receptions; use the following to be robust to node ID changes
        // std::ostringstream path;
        // path << "/NodeList/" << vehicles.Get(i)->GetId()
        //     << "/ApplicationList/1/$ns3::PacketSink/Rx";
        // Config::ConnectWithoutContext(path.str(), MakeCallback(&ReceivePacket));
        // path.str("");

        // path << "/NodeList/" << vehicles.Get(i)->GetId()
        //     << "/ApplicationList/1/$ns3::PacketSink/Rx";
        // Config::ConnectWithoutContext(path.str(), MakeCallback(&ComputePir));
        // path.str("");

        // path << "/NodeList/" << vehicles.Get(i)->GetId()
        //     << "/ApplicationList/0/$ns3::UdpClient/Tx";
        // Config::ConnectWithoutContext(path.str(), MakeCallback(&TransmitPacket));
        // path.str("");
    }

    std::cout << "[INFO] NR-V2X Mode2 vehicles initialized: " << vehicles.GetN() << " UE nodes.\n";
}


int main(int argc, char *argv[]) {

  LogComponentEnable("CamApplication", LOG_ALL);

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

  std::cout << "[INFO] Simulation finished.\n";

  std::cout << "Total Tx bits = " << txByteCounter * 8 << std::endl;
  std::cout << "Total Tx packets = " << txPktCounter << std::endl;

  std::cout << "Total Rx bits = " << rxByteCounter * 8 << std::endl;
  std::cout << "Total Rx packets = " << rxPktCounter << std::endl;

  std::cout << "Avrg thput = "
            << (rxByteCounter * 8) / (finalSimTime - Seconds(realAppStart)).GetSeconds() / 1000.0
            << " kbps" << std::endl;

  std::cout << "Average Packet Inter-Reception (PIR) " << pir.GetSeconds() / pirCounter << " sec"
            << std::endl;
  return 0;
}
