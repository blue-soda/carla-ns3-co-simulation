#include "cam-application.h"
#include "geo-networking.h"
#include "ns3/llc-snap-header.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/internet-module.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE("CamApplication");

NS_OBJECT_ENSURE_REGISTERED(CamSender);
TypeId CamSender::GetTypeId() {
  static TypeId tid = TypeId("ns3::CamSender")
                          .SetParent<Application>()
                          .SetGroupName("Applications")
                          .AddConstructor<CamSender>();
  return tid;
}
CamSender::CamSender()
    : m_socket(nullptr),
      m_vehicleId(0),
      m_interval(Seconds(1.0)),
      m_radius(1000),
      m_running(false),
      m_packetsSent(0) {
  m_jitterRng = CreateObject<UniformRandomVariable>();
}
CamSender::~CamSender() { m_socket = nullptr; }
void CamSender::SetVehicleId(const uint32_t id) { m_vehicleId = id; }
void CamSender::SetIp(const Ipv4Address& addr) { m_addr = addr; }
void CamSender::SetPort(const uint16_t port) { m_port = port; }
void CamSender::SetBroadcastAddress(Ipv4Address addr, uint16_t port) {
    m_broadcastAddr = addr;
    m_port = port;
}
void CamSender::SetInterval(const Time& interval) { m_interval = interval; }
void CamSender::SetBroadcastRadius(const uint16_t radius) { m_radius = radius; }
bool CamSender::IsRunning() { return m_running; };
void CamSender::ScheduleCam(uint32_t bytes, Ipv4Address dest_addr) { 
  Simulator::Schedule(MilliSeconds(0), [this, bytes, dest_addr] { SendCam(bytes, dest_addr); });
}
void CamSender::SendCam(uint32_t bytes, Ipv4Address dest_addr) { std::cout << "[WARN] CamSender::SendCam should be overridden\n"; }
void CamSender::StartApplication() { m_running = true; }
void CamSender::StopApplication() {
  m_running = false;
  if (m_sendEvent.IsPending()) {
    Simulator::Cancel(m_sendEvent);
  }
  if (m_socket) {
    m_socket->Close();
  }
}
void CamSender::ScheduleNextCam() {
  if (m_running) {
    // Time jitter = MicroSeconds(m_jitterRng->GetInteger(0, 100000));
    // Time nextTime = m_interval + jitter;
    // m_sendEvent = Simulator::Schedule(nextTime, [this] { SendCam(); });
  }
}

NS_OBJECT_ENSURE_REGISTERED(CamReceiver);
TypeId CamReceiver::GetTypeId() {
  static TypeId tid = TypeId("ns3::CamReceiver")
                          .SetParent<Application>()
                          .SetGroupName("Applications")
                          .AddConstructor<CamReceiver>();
  return tid;
}
CamReceiver::CamReceiver() : m_socket(nullptr), m_vehicleId(0), m_packetsReceived(0) {}
CamReceiver::~CamReceiver() { m_socket = nullptr; }
void CamReceiver::SetVehicleId(const uint32_t id) { m_vehicleId = id; }
void CamReceiver::SetIp(const Ipv4Address& addr) { m_addr = addr; }
void CamReceiver::SetPort(const uint16_t port) { m_port = port; }
void CamReceiver::SetReplyFunction(std::function<void(const std::string&)> replyFunction) {
  m_replyFunction = replyFunction;
}
void CamReceiver::StartApplication() {}
void CamReceiver::StopApplication() {}
void CamReceiver::HandleRead(Ptr<Socket> socket) { std::cout << "[WARN] CamReceiver::HandleRead should be overridden\n"; }

// ==================== DSRC derived classes ====================
NS_OBJECT_ENSURE_REGISTERED(CamSenderDSRC);
void CamSenderDSRC::StartApplication() {
  m_running = true;

  if (!m_socket) {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  }
}

void CamSenderDSRC::StopApplication() {
  CamSender::StopApplication();
}

void CamSenderDSRC::SendCam(uint32_t bytes, Ipv4Address dest_addr) {

  NS_ASSERT(m_running);
  NS_ASSERT(m_socket);

  Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
  NS_ASSERT(mobility);

  Vector pos = mobility->GetPosition();
  Vector vel = mobility->GetVelocity();
  double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
  double heading = std::atan2(vel.y, vel.x) * 180.0 / M_PI;

  Ptr<Packet> packet = Create<Packet>(bytes);

  CamHeader camHeader;
  camHeader.SetVehicleId(m_vehicleId);
  camHeader.SetPositionX(pos.x);
  camHeader.SetPositionY(pos.y);
  camHeader.SetSpeed(speed);
  camHeader.SetHeading(heading);
  camHeader.SetTimestamp(Simulator::Now().GetMilliSeconds());
  packet->AddHeader(camHeader);

  GeoNetHeader geoHeader;
  geoHeader.SetVersion(1);
  geoHeader.SetNextHeader(PROT_NUM_CAM);
  geoHeader.SetMessageType(GeoNetHeader::GEOBROADCAST);
  geoHeader.SetSourcePosition(pos.x, pos.y);
  geoHeader.SetSourceId(m_vehicleId);
  geoHeader.SetRadius(m_radius);
  geoHeader.SetLifetime(10);
  packet->AddHeader(geoHeader);

  LlcSnapHeader llc;
  llc.SetType(PROT_NUM_GEONETWORKING);
  packet->AddHeader(llc);

  InetSocketAddress destination = InetSocketAddress(dest_addr, m_port); // dest port
  // m_socket->SetAllowBroadcast(true);

  m_socket->SendTo(packet, 0, destination);

  m_packetsSent++;
  NS_LOG_INFO("Vehicle " << m_vehicleId << "(ip = " << m_addr << ") sent CAM at "
                         << Simulator::Now().GetSeconds() << "s"
                         << " Position: (" << pos.x << "," << pos.y << ")"
                         << " Speed: " << speed << " Heading: " << heading << " size: " << packet->GetSize() << " bytes"
                         << " Dest: " << dest_addr << ":" << m_port);
}


// ==================== CamReceiverDSRC ====================
NS_OBJECT_ENSURE_REGISTERED(CamReceiverDSRC);
TypeId CamReceiverDSRC::GetTypeId() {
  static TypeId tid = TypeId("ns3::CamReceiverDSRC")
                          .SetParent<CamReceiver>()
                          .AddConstructor<CamReceiverDSRC>();
  return tid;
}
TypeId CamSenderDSRC::GetTypeId() {
  static TypeId tid = TypeId("ns3::CamSenderDSRC")
                          .SetParent<CamSender>()
                          .AddConstructor<CamSenderDSRC>();
  return tid;
}
void CamReceiverDSRC::StartApplication() {
  if (!m_socket) {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    m_socket->Bind(local);
  }

  m_socket->SetRecvCallback(MakeCallback(&CamReceiverDSRC::HandleRead, this));
}

void CamReceiverDSRC::StopApplication() {
  if (m_socket) {
    m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    m_socket->Close();
  }
}

void CamReceiverDSRC::HandleRead(Ptr<Socket> socket) {
  std::cout << "[DEBUG] CamReceiverDSRC::HandleRead called\n";
  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from))) {
    InetSocketAddress inetAddr = InetSocketAddress::ConvertFrom(from);
    Ipv4Address src = inetAddr.GetIpv4();
    uint16_t srcPort = inetAddr.GetPort();
    std::cout << "[DEBUG] CamReceiverDSRC::HandleRead received packet from " << src << ":" << srcPort << ", size: " << packet->GetSize() << " bytes\n";
    LlcSnapHeader llc;
    packet->RemoveHeader(llc);

    if (llc.GetType() == PROT_NUM_GEONETWORKING) {
      GeoNetHeader geoHeader;
      packet->RemoveHeader(geoHeader);

      Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
      NS_ASSERT(mobility);
      Vector myPos = mobility->GetPosition();

      double dx = myPos.x - geoHeader.GetSourcePositionX();
      double dy = myPos.y - geoHeader.GetSourcePositionY();
      double distance = std::sqrt(dx * dx + dy * dy);
      std::cout << "[DEBUG] CamReceiverDSRC::HandleRead distance: " << distance << ", radius: " << geoHeader.GetRadius() << "\n";
      if (distance <= geoHeader.GetRadius()) {
        if (geoHeader.GetNextHeader() == PROT_NUM_CAM) {
          CamHeader camHeader;
          packet->RemoveHeader(camHeader);
          auto packetSize = packet->GetSize();

          NS_LOG_INFO("Node " << GetNode()->GetId() << " (Vehicle " << m_vehicleId << ")" 
                      << " received CAM from Vehicle "
                      << camHeader.GetVehicleId() << " at "
                      << Simulator::Now().GetSeconds() << "s" << " Position: ("
                      << camHeader.GetPositionX() << ","
                      << camHeader.GetPositionY() << ")"
                      << " Speed: " << camHeader.GetSpeed()
                      << " Heading: " << camHeader.GetHeading()
                      << " Timestamp: " << camHeader.GetTimestamp() << " ms"
                      << " Distance: " << distance << "m"
                      << " GeoNet sourceId: " << geoHeader.GetSourceId()
                      << " Packet size: " << packetSize << " bytes"
                    );

          m_packetsReceived++;

          try{
            if(m_replyFunction) {
              std::string msg = R"({"type":"cam_received",)"
                                R"("sender_id":)" + std::to_string(camHeader.GetVehicleId()) +
                                R"(,"receiver_id":)" + std::to_string(m_vehicleId) + 
                                R"(,"receive_timestamp":)" + std::to_string(Simulator::Now().GetMilliSeconds()) +
                                R"(,"send_timestamp":)" + std::to_string(camHeader.GetTimestamp()) +
                                R"(,"packet_size":)" + std::to_string(packetSize) +
                                R"(,"is_last_packet":)" + std::to_string(true) + 
                                R"(})";
              // m_replyFunction(msg);
                Simulator::ScheduleNow([this, msg]() {
                    m_replyFunction(msg);
                });
            }
          } catch(std::exception &e){ 
            NS_LOG_ERROR("CamReceiver::HandleRead m_replyFunction error: " << e.what());
          } catch (...) { 
            NS_LOG_ERROR("CamReceiver::HandleRead m_replyFunction: unknown exception caught");
          }
        }
      } else {
        NS_LOG_INFO("Node " << GetNode()->GetId() << " (Vehicle " << m_vehicleId << ")" 
                    << " discarded packet from Vehicle "
                    << geoHeader.GetSourceId()
                    << " - outside broadcast radius (distance=" << distance
                    << "m, radius=" << geoHeader.GetRadius() << "m)");
      }
    }
  }
}

// ==================== NR-V2X derived classes ====================
NS_OBJECT_ENSURE_REGISTERED(CamSenderNR);

void CamSenderNR::StartApplication() {
  m_running = true;
  if (!m_socket) {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  }
  if (!m_scheduler) {
    if(! (m_scheduler = GetScheduler())) {
      std::cerr << "CamSenderNR: failed to get cluster scheduler" << std::endl;
      return;
    }
  }
}

void CamSenderNR::SendCam(uint32_t bytes, Ipv4Address dest_addr)
{
    // bytes = 1000;
    // std::cout << "CamSenderNR::SendCam(uint32_t bytes, Ipv4Address dest_addr)\n";
    NS_ASSERT(m_running);
    NS_ASSERT(m_socket);
    Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
    NS_ASSERT(mobility);

    InetSocketAddress destination = InetSocketAddress(dest_addr, m_port); // dest port
    // InetSocketAddress destination = InetSocketAddress(Ipv4Address("225.0.0.0"), m_port); // dest port
    Ptr<Packet> packet = Create<Packet>(bytes);

    Vector pos = mobility->GetPosition();
    Vector vel = mobility->GetVelocity();
    double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
    double heading = std::atan2(vel.y, vel.x) * 180.0 / M_PI;
    CamHeader camHeader;
    camHeader.SetVehicleId(m_vehicleId);
    camHeader.SetPositionX(pos.x);
    camHeader.SetPositionY(pos.y);
    camHeader.SetSpeed(speed);
    camHeader.SetHeading(heading);
    camHeader.SetTimestamp(Simulator::Now().GetMilliSeconds());
    packet->AddHeader(camHeader);

    // std::cout << "Addheader\n";

    m_socket->SendTo(packet, 0, destination);

    m_packetsSent++;
    NS_LOG_INFO("Vehicle " << m_vehicleId << "(ip = " << m_addr << ") sent CAM at "
                  << Simulator::Now().GetSeconds() << "s"
                  << " Position: (" << pos.x << "," << pos.y << ")"
                  << " Speed: " << speed << " Heading: " << heading << " size: " << packet->GetSize() << " bytes"
                  << " Dest: " << dest_addr << ":" << m_port
              );
}

TypeId CamSenderNR::GetTypeId() {
    static TypeId tid = TypeId("ns3::CamSenderNR")
                            .SetParent<CamSender>()
                            .AddConstructor<CamSenderNR>();
    return tid;
}

void CamSenderNR::StopApplication() {
    CamSender::StopApplication();
}

void CamSenderNR::ScheduleCam(uint32_t bytes, Ipv4Address dest_addr, 
                               uint8_t sc_start, uint8_t sc_num, double tx_power, uint32_t src_L2Id, uint32_t dest_L2Id) { 
  Simulator::Schedule(MilliSeconds(0), [this, bytes, dest_addr, sc_start, sc_num, tx_power, src_L2Id, dest_L2Id] { 
    SendCam(bytes, dest_addr, sc_start, sc_num, tx_power, src_L2Id, dest_L2Id); 
  });
}

void CamSenderNR::SendCam(uint32_t bytes, Ipv4Address dest_addr, 
                               uint8_t sc_start, uint8_t sc_num, double tx_power, uint32_t src_L2Id, uint32_t dest_L2Id)
{
  NS_LOG_FUNCTION(this << bytes << dest_addr << (uint32_t)sc_start << (uint32_t)sc_num << tx_power << dest_L2Id);
  
  Ptr<Packet> packet = Create<Packet>(bytes);

  Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
  Vector pos = mobility->GetPosition();
  Vector vel = mobility->GetVelocity();
  double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
  double heading = std::atan2(vel.y, vel.x) * 180.0 / M_PI;
  CamHeader camHeader;
  camHeader.SetVehicleId(m_vehicleId);
  camHeader.SetPositionX(pos.x);
  camHeader.SetPositionY(pos.y);
  camHeader.SetSpeed(speed);
  camHeader.SetHeading(heading);
  camHeader.SetTimestamp(Simulator::Now().GetMilliSeconds());
  packet->AddHeader(camHeader);

  // std::cout << "packet created\n";

    // 获取发送方和接收方的 L2 ID
  uint32_t srcL2Id = src_L2Id; // 使用传入的发送方 L2 ID
  uint32_t dstL2Id = dest_L2Id; // 使用传入的接收方 L2 ID

  // 构建 CARLA 传输指令
  CarlaTxCommand cmd;
  cmd.srcL2Id = srcL2Id;
  cmd.dstL2Id = dstL2Id;
  cmd.maxDataSize = (int)packet->GetSize() + 35; // 包含头部的总数据大小
  cmd.slSubchannelStart = sc_start;
  cmd.slSubchannelSize = sc_num;
  // cmd.txPower = tx_power;

  // 调用调度器接口，下发指令
  m_scheduler->AddCarlaTxCommand(cmd);
  
  // std::cout << "CarlaTxCommand added to scheduler\n";

  InetSocketAddress destination = InetSocketAddress(dest_addr, m_port);
  m_socket->SendTo(packet, 0, destination);

  std::cout << "CamSenderNR: scheduled CAM, srcL2Id=" << srcL2Id << ", dstL2Id=" << dstL2Id
              << ", Subchannel=[" << (uint32_t)sc_start << ", " << (uint32_t)(sc_start+sc_num-1) << "]"
              << ", size=" << packet->GetSize() << " bytes\n";
}

Ptr<NrSlUeMacSchedulerManual> CamSenderNR::GetScheduler()
{
  Ptr<NrUeNetDevice> ueNetDev = nullptr;
  for (uint32_t i = 0; i < m_node->GetNDevices(); ++i)
  {
    if(!ueNetDev)
      ueNetDev = DynamicCast<NrUeNetDevice>(m_node->GetDevice(i));
  }
  if (!ueNetDev) { 
    std::cerr << "CamSenderNR: failed to get NrUeNetDevice from Node" << std::endl;
    return nullptr;
  }
  Ptr<NrSlUeMac> slMac = ueNetDev->GetMac(0)->GetObject<NrSlUeMac>();
  if (!slMac) {
    std::cerr << "CamSenderNR: failed to get NrSlUeMac from NrUeNetDevice" << std::endl;
    return nullptr;
  }
  return slMac->GetScheduler()->GetObject<NrSlUeMacSchedulerManual>();
}

// ==================== CamReceiverNR ====================
NS_OBJECT_ENSURE_REGISTERED(CamReceiverNR);

TypeId CamReceiverNR::GetTypeId() {
    static TypeId tid = TypeId("ns3::CamReceiverNR")
                            .SetParent<CamReceiver>()
                            .AddConstructor<CamReceiverNR>();
    return tid;
}

void CamReceiverNR::StartApplication() {
  if (!m_socket) {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    m_socket->Bind(local);
  }
  m_socket->SetRecvCallback(MakeCallback(&CamReceiverNR::HandleRead, this));
}

void CamReceiverNR::StopApplication() {
    if (m_socket) {
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        m_socket->Close();
    }
}

void CamReceiverNR::HandleRead(Ptr<Socket> socket) {
    std::cout << "CamReceiverNR::HandleRead\n";
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from))) {
        InetSocketAddress inetAddr = InetSocketAddress::ConvertFrom(from);
        Ipv4Address src = inetAddr.GetIpv4();
        uint16_t srcPort = inetAddr.GetPort();
        CamHeader camHeader;
        packet->RemoveHeader(camHeader);
        auto packetSize = packet->GetSize();
        NS_LOG_INFO("NR-V2X Node " << GetNode()->GetId() << " (Vehicle " << m_vehicleId << ")"
                    << " received CAM from Vehicle "
                    << camHeader.GetVehicleId() << " at "
                    << Simulator::Now().GetSeconds() << "s"
                    << " Position: (" << camHeader.GetPositionX() << ","
                    << camHeader.GetPositionY() << ")"
                    << " Speed: " << camHeader.GetSpeed()
                    << " Heading: " << camHeader.GetHeading()
                    << " Timestamp: " << camHeader.GetTimestamp() << " ms"
                    << " Packet size: " << packetSize << " bytes"
                    << " Src IP: " << src << ":" << srcPort);
        m_packetsReceived++;
        try {
            if (m_replyFunction) {
                std::string msg = R"({"type":"cam_received",)"
                                  R"("sender_id":)" + std::to_string(camHeader.GetVehicleId()) +
                                  R"(,"receiver_id":)" + std::to_string(m_vehicleId) +
                                  R"(,"receive_timestamp":)" + std::to_string(Simulator::Now().GetMilliSeconds()) +
                                  R"(,"send_timestamp":)" + std::to_string(camHeader.GetTimestamp()) +
                                  R"(,"packet_size":)" + std::to_string(packetSize) +
                                  R"(,"is_last_packet":)" + std::to_string(true) +
                                  R"(})";
                // m_replyFunction(msg);
                // 异步调度发送，避免阻塞HandleRead
                Simulator::ScheduleNow([this, msg]() {
                    m_replyFunction(msg);
                });
            }
        } catch (std::exception& e) {
            NS_LOG_ERROR("CamReceiverNR::HandleRead m_replyFunction error: " << e.what());
        } catch (...) {
            NS_LOG_ERROR("CamReceiverNR::HandleRead unknown exception");
        }
    }
}

}