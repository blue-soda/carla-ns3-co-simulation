#include "ns3/core-module.h"
#include <iostream>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>

using namespace ns3;

void SocketServerThread()
{
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

  std::cout << "Client connected.\n" << client_fd << "\n";

  if (client_fd < 0) {
    perror("accept failed");
    close(server_fd);
    return;
  }

  std::cout << "Connected: " << inet_ntoa(address.sin_addr)
            << ":" << ntohs(address.sin_port) << "\n";

  // Send initial handshake message
  const char* init_msg = "NS-3 bridge connected\n";
  send(client_fd, init_msg, strlen(init_msg), 0);


  while (true) {
    memset(buffer, 0, sizeof(buffer));
    ssize_t bytes = recv(client_fd, buffer, sizeof(buffer)-1, 0);

    if (bytes < 0) {
      perror("recv error");
      break;
    } else if (bytes == 0) {
      std::cout << "Client disconnected gracefully.\n";
      break;
    }

    buffer[bytes] = '\0';
    std::cout << "Received (" << bytes << " bytes): " << buffer << "\n";

    // Simple echo for testing purposes
    send(client_fd, buffer, bytes, 0);
  }

  close(client_fd);
  close(server_fd);
}

void KeepAlive()
{
  Simulator::Schedule(Seconds(1.0), &KeepAlive);
}

int main(int argc, char** argv)
{
  CommandLine cmd;
  cmd.Parse(argc, argv);

  std::thread server_thread(SocketServerThread);

  // Schedule periodic keep-alive to ensure simulator doesn't stop prematurely
  Simulator::Schedule(Seconds(1.0), &KeepAlive);

  Simulator::Stop(Seconds(3600));
  Simulator::Run();

  server_thread.join();

  Simulator::Destroy();

  std::cout << "Simulation finished.\n";
  return 0;
}
