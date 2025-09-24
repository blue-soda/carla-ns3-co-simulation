#ifndef CAM_APPLICATION_H
#define CAM_APPLICATION_H

#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"

namespace ns3 {

class CamSender : public Application {
public:
    static TypeId GetTypeId();
    CamSender();
    ~CamSender() override;
    virtual void SetVehicleId(uint32_t id);
    virtual void SetInterval(const Time& interval);
    virtual void SetBroadcastRadius(uint16_t radius);
    virtual void SendCam(uint32_t bytes = 0, bool addCamHeader = true, bool addGeoNetHeader = true);
    bool IsRunning();
protected:
    // Application lifecycle hooks (to be overridden)
    void StartApplication() override;
    void StopApplication() override;
    virtual void ScheduleNextCam();

    Ptr<Socket> m_socket;
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
protected:
    void StartApplication() override;
    void StopApplication() override;
    virtual void HandleRead(Ptr<Socket> socket);
    Ptr<Socket> m_socket;
    uint32_t m_vehicleId;
    uint32_t m_packetsReceived;
    std::function<void(const std::string&)> m_replyFunction;
};



// ==================== DSRC derived classes ====================
class CamSenderDSRC : public CamSender {
public:
  static TypeId GetTypeId();
//   CamSenderDSRC();
//   ~CamSenderDSRC() override;
  void StartApplication() override;
  void StopApplication() override;
  void SendCam(uint32_t bytes = 0, bool addCamHeader = true, bool addGeoNetHeader = true) override;
};
class CamReceiverDSRC : public CamReceiver {
public:
  static TypeId GetTypeId();
//   CamReceiverDSRC();
//   ~CamReceiverDSRC() override;
  void StartApplication() override;
  void StopApplication() override;
  void HandleRead(Ptr<Socket> socket) override;
};


} // namespace ns3

#endif