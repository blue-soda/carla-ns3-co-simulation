#ifndef CARLA_VANET_H
#define CARLA_VANET_H
#include <nlohmann/json.hpp>
using json = nlohmann::json;

void ProcessData_VehiclePosition(const json& vehicleArray);
void ProcessData_TransferRequests(const json &requests);
void ProcessData_VehiclesNum(const int &num);
void ProcessJsonData(const std::string &data);

void SocketServerThread();
void UpdateVehiclePositions();
void SendSimulationEndSignal();

void InitializeVehicles(uint32_t nVehicles);

#endif