#ifndef CAM_APPLICATION_H
#define CAM_APPLICATION_H

#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/lte-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/onoff-application.h"

namespace ns3 {

class CamSender : public Application {
public:
    static TypeId GetTypeId();
    CamSender();
    ~CamSender() override;
    virtual void SetVehicleId(uint32_t id);
    virtual void SetIp(const Ipv4Address& addr);
    virtual void SetPort(uint16_t port);
    virtual void SetBroadcastAddress(Ipv4Address addr, uint16_t port);
    virtual void SetInterval(const Time& interval);
    virtual void SetBroadcastRadius(uint16_t radius);
    virtual void ScheduleCam(uint32_t bytes, Ipv4Address dest_addr);
    virtual void SendCam(uint32_t bytes, Ipv4Address dest_addr);
    bool IsRunning();
    struct TxMeta {
        uint8_t rb_start;
        uint8_t rb_num;
        double tx_power;
    };
    std::unordered_map<Ptr<Packet>, TxMeta> m_packetTxMeta; // Packet → 传输元信息
    // 获取调度器实例（从 Node 的 NetDevice 中提取）
    Ptr<NrSlUeMacSchedulerCluster> GetScheduler();

protected:
    // Application lifecycle hooks (to be overridden)
    void StartApplication() override;
    void StopApplication() override;
    virtual void ScheduleNextCam();

    Ptr<Socket> m_socket;
    Ipv4Address m_addr;
    Ipv4Address m_broadcastAddr;
    uint16_t m_port{5000};
    uint32_t m_vehicleId;
    Time m_interval;
    uint16_t m_radius;
    EventId m_sendEvent;
    bool m_running;
    uint32_t m_packetsSent;
    Ptr<UniformRandomVariable> m_jitterRng;
};

class CamReceiver : public Application {
public:
    static TypeId GetTypeId();
    CamReceiver();
    ~CamReceiver() override;
    virtual void SetVehicleId(uint32_t id);
    virtual void SetReplyFunction(std::function<void(const std::string&)> replyFunction);
    virtual void SetIp(const Ipv4Address& addr);
    virtual void SetPort(uint16_t port);
protected:
    void StartApplication() override;
    void StopApplication() override;
    virtual void HandleRead(Ptr<Socket> socket);
    Ptr<Socket> m_socket;
    Ipv4Address m_addr;
    uint16_t m_port{5000};
    uint32_t m_vehicleId;
    uint32_t m_packetsReceived;
    std::function<void(const std::string&)> m_replyFunction;
};



// ==================== DSRC derived classes ====================
class CamSenderDSRC : public CamSender {
public:
  static TypeId GetTypeId();
  void StartApplication() override;
  void StopApplication() override;
  void SendCam(uint32_t bytes, Ipv4Address dest_addr) override;
};
class CamReceiverDSRC : public CamReceiver {
public:
  static TypeId GetTypeId();
  void StartApplication() override;
  void StopApplication() override;
  void HandleRead(Ptr<Socket> socket) override;
};


// ==================== NR-V2X derived classes ====================
class CamSenderNR : public CamSender {
public:
    static TypeId GetTypeId();
    void StartApplication() override;
    void StopApplication() override;
    void SendCam(uint32_t bytes, Ipv4Address dest_addr) override;
    void ScheduleCam(uint32_t bytes, Ipv4Address dest_addr, 
                               uint8_t rb_start, uint8_t rb_num, double tx_power,  uint32_t dest_L2Id);
    void SendCam(uint32_t bytes, Ipv4Address dest_addr, 
                                uint8_t rb_start, uint8_t rb_num, double tx_power, uint32_t dest_L2Id);
    Ptr<NrSlUeMacSchedulerCluster> GetScheduler();
};
class CamReceiverNR : public CamReceiver {
public:
    static TypeId GetTypeId();
    void StartApplication() override;
    void StopApplication() override;
    void HandleRead(Ptr<Socket> socket) override;
};

} // namespace ns3

#endif