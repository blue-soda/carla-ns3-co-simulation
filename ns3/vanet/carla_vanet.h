#ifndef CARLA_VANET_H
#define CARLA_VANET_H
#include <nlohmann/json.hpp>
using json = nlohmann::json;

void ProcessData_VehiclePosition(const json& vehicleArray);
void ProcessData_TransferRequests(const json &requests);
void ProcessData_VehiclesNum(const int &num);
void ProcessJsonData(const std::string &data);
void ProcessReceivedData(std::string &receive_buffer);

void SocketServerThread();
void UpdateVehiclePositions();
void SendSimulationEndSignal();
void SendMsgToCarla(const std::string &msg, bool try_reconnect);
void SocketSenderServerConnect();
void SocketSenderServerDisconnect();

void InitializeVehicles_DSRC(uint32_t nVehicles);
void InitializeVehicles_NR_V2X_Mode2(uint32_t nVehicles);
void InitializeVehicles(uint32_t nVehicles = 3){
  // InitializeVehicles_DSRC(nVehicles);
  InitializeVehicles_NR_V2X_Mode2(nVehicles);
}
void PrintRoutingTable (ns3::Ptr<ns3::Node> node);

struct TransferRequestSubChannel {
  uint32_t size;         // 数据大小
  int target;            // 目标 CARLA ID
  uint8_t start;      // CARLA 指定的子信道起始索引
  uint8_t num;        // CARLA 指定的子信道数量
  double tx_power;       // CARLA 指定的发射功率(W)
};

#endif