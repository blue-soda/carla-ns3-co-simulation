#ifndef CAM_APPLICATION_H
#define CAM_APPLICATION_H

#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"

namespace ns3 {

class CamSender : public Application {
public:
    static TypeId GetTypeId(void);
    CamSender();
    virtual ~CamSender();

    void SetVehicleId(uint32_t id);
    void SetInterval(Time interval);
    void SetBroadcastRadius(uint16_t radius);

private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void SendCam(void);
    void ScheduleNextCam(void);

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
    static TypeId GetTypeId(void);
    CamReceiver();
    virtual ~CamReceiver();

private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void HandleRead(Ptr<Socket> socket);

    Ptr<Socket> m_socket;
    uint32_t m_packetsReceived;
};

}

#endif