#include "geo-networking.h"
#include <cstring>

namespace ns3 {

GeoNetHeader::GeoNetHeader()
    : m_version(1),
      m_nextHeader(PROT_NUM_CAM),
      m_messageType(GEOBROADCAST),
      m_sourcePositionX(0.0),
      m_sourcePositionY(0.0),
      m_sourceId(0),
      m_radius(1000),
      m_lifetime(60) {}

GeoNetHeader::~GeoNetHeader() {}

TypeId GeoNetHeader::GetTypeId() {
  static TypeId tid = TypeId("ns3::GeoNetHeader")
                          .SetParent<Header>()
                          .AddConstructor<GeoNetHeader>();
  return tid;
}

TypeId GeoNetHeader::GetInstanceTypeId() const { return GetTypeId(); }

uint32_t GeoNetHeader::GetSerializedSize() const {
  // version + nextHeader + messageType + posX + posY + sourceId + radius + lifetime
  return 1 + 1 + 1 + 8 + 8 + 4 + 2 + 2;
}

void GeoNetHeader::Serialize(Buffer::Iterator start) const {
  start.WriteU8(m_version);
  start.WriteU8(m_nextHeader);
  start.WriteU8(static_cast<uint8_t>(m_messageType));

  uint64_t posX;
  uint64_t posY;
  std::memcpy(&posX, &m_sourcePositionX, sizeof(double));
  std::memcpy(&posY, &m_sourcePositionY, sizeof(double));

  start.WriteHtonU64(posX);
  start.WriteHtonU64(posY);

  start.WriteHtonU32(m_sourceId);
  start.WriteHtonU16(m_radius);
  start.WriteHtonU16(m_lifetime);
}

uint32_t GeoNetHeader::Deserialize(Buffer::Iterator start) {
  m_version = start.ReadU8();
  m_nextHeader = start.ReadU8();
  m_messageType = static_cast<GeoNetMessageType>(start.ReadU8());

  const uint64_t posX = start.ReadNtohU64();
  const uint64_t posY = start.ReadNtohU64();

  std::memcpy(&m_sourcePositionX, &posX, sizeof(double));
  std::memcpy(&m_sourcePositionY, &posY, sizeof(double));

  m_sourceId = start.ReadNtohU32();
  m_radius = start.ReadNtohU16();
  m_lifetime = start.ReadNtohU16();

  return GetSerializedSize();
}

void GeoNetHeader::Print(std::ostream &os) const {
  os << "GeoNetHeader: Version=" << static_cast<uint32_t>(m_version)
     << " NextHeader=" << static_cast<uint32_t>(m_nextHeader)
     << " MessageType=" << static_cast<uint32_t>(m_messageType) << " SourcePosition=("
     << m_sourcePositionX << "," << m_sourcePositionY << ")"
     << " SourceId=" << m_sourceId << " Radius=" << m_radius
     << " Lifetime=" << m_lifetime;
}

void GeoNetHeader::SetVersion(const uint8_t version) { m_version = version; }

uint8_t GeoNetHeader::GetVersion() const { return m_version; }

void GeoNetHeader::SetNextHeader(const uint8_t nextHeader) {
  m_nextHeader = nextHeader;
}

uint8_t GeoNetHeader::GetNextHeader() const { return m_nextHeader; }

void GeoNetHeader::SetMessageType(const GeoNetMessageType type) {
  m_messageType = type;
}

GeoNetHeader::GeoNetMessageType GeoNetHeader::GetMessageType() const {
  return m_messageType;
}

void GeoNetHeader::SetSourcePosition(const double x, const double y) {
  m_sourcePositionX = x;
  m_sourcePositionY = y;
}

double GeoNetHeader::GetSourcePositionX() const {
  return m_sourcePositionX;
}

double GeoNetHeader::GetSourcePositionY() const {
  return m_sourcePositionY;
}

void GeoNetHeader::SetSourceId(const uint32_t id) { m_sourceId = id; }

uint32_t GeoNetHeader::GetSourceId() const { return m_sourceId; }

void GeoNetHeader::SetRadius(const uint16_t radius) { m_radius = radius; }

uint16_t GeoNetHeader::GetRadius() const { return m_radius; }

void GeoNetHeader::SetLifetime(const uint16_t seconds) { m_lifetime = seconds; }

uint16_t GeoNetHeader::GetLifetime() const { return m_lifetime; }

CamHeader::CamHeader()
    : m_vehicleId(0),
      m_positionX(0.0),
      m_positionY(0.0),
      m_speed(0.0),
      m_heading(0.0),
      m_timestamp(0) {}

CamHeader::~CamHeader() {}

TypeId CamHeader::GetTypeId() {
  static TypeId tid =
      TypeId("ns3::CamHeader").SetParent<Header>().AddConstructor<CamHeader>();
  return tid;
}

TypeId CamHeader::GetInstanceTypeId() const { return GetTypeId(); }

uint32_t CamHeader::GetSerializedSize() const {
  return sizeof(m_vehicleId) + sizeof(m_positionX) + sizeof(m_positionY) +
         sizeof(m_speed) + sizeof(m_heading) + sizeof(m_timestamp);
}

void CamHeader::Serialize(Buffer::Iterator start) const {
  start.WriteHtonU32(m_vehicleId);

  uint64_t posX;
  uint64_t posY;
  uint64_t speed;
  uint64_t heading;

  std::memcpy(&posX, &m_positionX, sizeof(double));
  std::memcpy(&posY, &m_positionY, sizeof(double));
  std::memcpy(&speed, &m_speed, sizeof(double));
  std::memcpy(&heading, &m_heading, sizeof(double));

  start.WriteHtonU64(posX);
  start.WriteHtonU64(posY);
  start.WriteHtonU64(speed);
  start.WriteHtonU64(heading);

  start.WriteHtonU64(m_timestamp);
}

uint32_t CamHeader::Deserialize(Buffer::Iterator start) {
  m_vehicleId = start.ReadNtohU32();

  const uint64_t posX = start.ReadNtohU64();
  const uint64_t posY = start.ReadNtohU64();
  const uint64_t speed = start.ReadNtohU64();
  const uint64_t heading = start.ReadNtohU64();

  std::memcpy(&m_positionX, &posX, sizeof(double));
  std::memcpy(&m_positionY, &posY, sizeof(double));
  std::memcpy(&m_speed, &speed, sizeof(double));
  std::memcpy(&m_heading, &heading, sizeof(double));

  m_timestamp = start.ReadNtohU64();

  return GetSerializedSize();
}

void CamHeader::Print(std::ostream &os) const {
  os << "Vehicle ID: " << m_vehicleId << " Position: (" << m_positionX << ","
     << m_positionY << ")" << " Speed: " << m_speed << " Heading: " << m_heading
     << " Timestamp: " << m_timestamp;
}

void CamHeader::SetVehicleId(const uint32_t id) { m_vehicleId = id; }

uint32_t CamHeader::GetVehicleId() const { return m_vehicleId; }

void CamHeader::SetPositionX(const double x) { m_positionX = x; }

double CamHeader::GetPositionX() const { return m_positionX; }

void CamHeader::SetPositionY(const double y) { m_positionY = y; }

double CamHeader::GetPositionY() const { return m_positionY; }

void CamHeader::SetSpeed(const double speed) { m_speed = speed; }

double CamHeader::GetSpeed() const { return m_speed; }

void CamHeader::SetHeading(const double heading) { m_heading = heading; }

double CamHeader::GetHeading() const { return m_heading; }

void CamHeader::SetTimestamp(const uint64_t timestamp) { m_timestamp = timestamp; }

uint64_t CamHeader::GetTimestamp() const { return m_timestamp; }

}