#ifndef GEO_NETWORKING_H
#define GEO_NETWORKING_H

#include "ns3/header.h"

#define PROT_NUM_GEONETWORKING 0x8947  // EtherType for GeoNetworking
#define PROT_NUM_CAM 0x02

namespace ns3 {

class GeoNetHeader final : public Header {
 public:
  enum GeoNetMessageType {
    BEACON = 1,
    GEOBROADCAST = 2,
    GEOUNICAST = 3,
    GEOANYCAST = 4,
    TOPOLOGICALLY_SCOPED_BROADCAST = 5
  };

  GeoNetHeader();
  ~GeoNetHeader() override;

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  void SetVersion(uint8_t version);
  uint8_t GetVersion() const;

  void SetNextHeader(uint8_t nextHeader);
  uint8_t GetNextHeader() const;

  void SetMessageType(GeoNetMessageType type);
  GeoNetMessageType GetMessageType() const;

  void SetSourcePosition(double x, double y);
  double GetSourcePositionX() const;
  double GetSourcePositionY() const;

  void SetSourceId(uint32_t id);
  uint32_t GetSourceId() const;

  void SetRadius(uint16_t radius);
  uint16_t GetRadius() const;

  void SetLifetime(uint16_t seconds);
  uint16_t GetLifetime() const;

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

class CamHeader final : public Header {
 public:
  CamHeader();
  ~CamHeader() override;

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  void SetVehicleId(uint32_t id);
  uint32_t GetVehicleId() const;

  void SetPositionX(double x);
  double GetPositionX() const;

  void SetPositionY(double y);
  double GetPositionY() const;

  void SetSpeed(double speed);
  double GetSpeed() const;

  void SetHeading(double heading);
  double GetHeading() const;

  void SetTimestamp(uint64_t timestamp);
  uint64_t GetTimestamp() const;

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