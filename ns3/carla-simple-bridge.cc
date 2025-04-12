#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

#include "ns3/core-module.h"

using namespace ns3;
using json = nlohmann::json;

void ProcessJsonData(const std::string& data) {
  try {
    json vehicles = json::parse(data);

    for (const auto& vehicle : vehicles) {
      int id = vehicle["id"];
      auto position = vehicle["position"];
      auto rotation = vehicle["rotation"];
      auto velocity = vehicle["velocity"];
      std::cout << "Vehicle ID: " << id << "\n";
      std::cout << "Position - x: " << position["x"] << ", y: " << position["y"]
                << ", z: " << position["z"] << "\n";
      std::cout << "Rotation - Pitch: " << rotation["pitch"]
                << ", Yaw: " << rotation["yaw"]
                << ", Roll: " << rotation["roll"] << "\n";
      std::cout << "Velocity - x: " << velocity["x"] << ", y: " << velocity["y"]
                << ", z: " << velocity["z"] << "\n";
      std::cout << "---------------------------\n";
    }
  } catch (json::exception& e) {
    std::cerr << "JSON parsing error: " << e.what() << "\n";
  }
}

void SocketServerThread() {
  int server_fd, client_fd;
  sockaddr_in address{};
  int addrlen = sizeof(address);
  char buffer[4096];

  server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    perror("socket failed");
    return;
  }

  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(5556);

  if (bind(server_fd, (sockaddr*)&address, sizeof(address)) < 0) {
    perror("bind failed");
    close(server_fd);
    return;
  }

  if (listen(server_fd, 1) < 0) {
    perror("listen failed");
    close(server_fd);
    return;
  }

  std::cout << "Server ready, waiting for connection on port 5556...\n";

  client_fd = accept(server_fd, (sockaddr*)&address, (socklen_t*)&addrlen);
  if (client_fd < 0) {
    perror("accept failed");
    close(server_fd);
    return;
  }

  std::cout << "Client connected: " << inet_ntoa(address.sin_addr) << ":"
            << ntohs(address.sin_port) << "\n";

  while (true) {
    memset(buffer, 0, sizeof(buffer));
    ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);

    if (bytes < 0) {
      perror("recv error");
      break;
    } else if (bytes == 0) {
      std::cout << "Client disconnected gracefully.\n";
      break;
    }

    buffer[bytes] = '\0';
    std::cout << "Received (" << bytes << " bytes): " << buffer << "\n";

    ProcessJsonData(std::string(buffer));
  }

  close(client_fd);
  close(server_fd);
}

void KeepAlive() { Simulator::Schedule(Seconds(1.0), &KeepAlive); }

int main(int argc, char** argv) {
  CommandLine cmd;
  cmd.Parse(argc, argv);

  std::thread server_thread(SocketServerThread);

  Simulator::Schedule(Seconds(1.0), &KeepAlive);
  Simulator::Stop(Seconds(3600));
  Simulator::Run();

  server_thread.join();
  Simulator::Destroy();

  std::cout << "Simulation finished.\n";
  return 0;
}
