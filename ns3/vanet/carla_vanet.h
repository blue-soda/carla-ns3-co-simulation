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
void SendMsgToCarla(const std::string &msg);
void SocketSenderServerConnect();
void SocketSenderServerDisconnect();

void InitializeVehicles_DSRC(uint32_t nVehicles);
void InitializeVehicles_NR_V2X_Mode2(uint32_t nVehicles);
void InitializeVehicles(uint32_t nVehicles = 3){
  // InitializeVehicles_DSRC(nVehicles);
  InitializeVehicles_NR_V2X_Mode2(nVehicles);
}
#endif