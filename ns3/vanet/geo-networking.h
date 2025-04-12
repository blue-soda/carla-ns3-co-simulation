#ifndef GEO_NETWORKING_H
#define GEO_NETWORKING_H

#include "ns3/header.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

#define PROT_NUM_GEONETWORKING 0x8947  // EtherType for GeoNetworking
#define PROT_NUM_CAM 0x02

namespace ns3 {

class GeoNetHeader : public Header {
 public:
  enum GeoNetMessageType {
    BEACON = 1,
    GEOBROADCAST = 2,
    GEOUNICAST = 3,
    GEOANYCAST = 4,
    TOPOLOGICALLY_SCOPED_BROADCAST = 5
  };

  GeoNetHeader();
  virtual ~GeoNetHeader();

  static TypeId GetTypeId(void);
  virtual TypeId GetInstanceTypeId(void) const;
  virtual uint32_t GetSerializedSize(void) const;
  virtual void Serialize(Buffer::Iterator start) const;
  virtual uint32_t Deserialize(Buffer::Iterator start);
  virtual void Print(std::ostream &os) const;

  void SetVersion(uint8_t version);
  uint8_t GetVersion(void) const;

  void SetNextHeader(uint8_t nextHeader);
  uint8_t GetNextHeader(void) const;

  void SetMessageType(GeoNetMessageType type);
  GeoNetMessageType GetMessageType(void) const;

  void SetSourcePosition(double x, double y);
  double GetSourcePositionX(void) const;
  double GetSourcePositionY(void) const;

  void SetSourceId(uint32_t id);
  uint32_t GetSourceId(void) const;

  void SetRadius(uint16_t radius);
  uint16_t GetRadius(void) const;

  void SetLifetime(uint16_t seconds);
  uint16_t GetLifetime(void) const;

 private:
  uint8_t m_version;
  uint8_t m_nextHeader;
  GeoNetMessageType m_messageType;
  double m_sourcePositionX;
  double m_sourcePositionY;
  uint32_t m_sourceId;
  uint16_t m_radius;
  uint16_t m_lifetime;
};

class CamHeader : public Header {
 public:
  CamHeader();
  virtual ~CamHeader();

  static TypeId GetTypeId(void);
  virtual TypeId GetInstanceTypeId(void) const;
  virtual uint32_t GetSerializedSize(void) const;
  virtual void Serialize(Buffer::Iterator start) const;
  virtual uint32_t Deserialize(Buffer::Iterator start);
  virtual void Print(std::ostream &os) const;

  void SetVehicleId(uint32_t id);
  uint32_t GetVehicleId(void) const;

  void SetPositionX(double x);
  double GetPositionX(void) const;

  void SetPositionY(double y);
  double GetPositionY(void) const;

  void SetSpeed(double speed);
  double GetSpeed(void) const;

  void SetHeading(double heading);
  double GetHeading(void) const;

  void SetTimestamp(uint64_t timestamp);
  uint64_t GetTimestamp(void) const;

 private:
  uint32_t m_vehicleId;
  double m_positionX;
  double m_positionY;
  double m_speed;
  double m_heading;
  uint64_t m_timestamp;
};

}

#endif