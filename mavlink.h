// 
// Mavlink frame detection: 
// Used to monitor the mavlink stream 
// 

//Field name			Index (Bytes)	Purpose
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Start-of-frame		0				Denotes the start of frame transmission (v1.0: 0xFE)
// Payload length		1				Length of the following payload
// Packet sequence		2				Each component counts up his send sequence. Allows to detect packet loss
// System ID			3				Identification of the SENDING system. Allows to differentiate different systems on the same network.
// Component ID			4				Identification of the SENDING component. Allows to differentiate different components of the same system, e.g. the IMU and the autopilot.
// Message ID			5				Identification of the message - the id defines what the payload �means� and how it should be correctly decoded.
// Payload				6 to (n+6)		The data into the message, depends on the message id.
// CRC					(n+7) to (n+8)	Check-sum of the entire packet, excluding the packet start sign (LSB to MSB)
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define MAVLINK_MAX_SIZE 64
#define MAVLINK_PACKET_START 0xFE
#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#define MAVLINK_MANUAL_CONTROL_CHANNELS 4
#define MAVLINK_MANUAL_CONTROL_BUTTONS 4

class MavlinkFrameDetector
{
public:
  int16_t channel[MAVLINK_MANUAL_CONTROL_CHANNELS];
  int16_t button[MAVLINK_MANUAL_CONTROL_BUTTONS];
  uint16_t buttons_mask;
  MavlinkFrameDetector() {
    Reset();
  }

  // //0-pending; 1-discard; 2-passthrough; 3-manual control frame complete 
  uint8_t Parse(uint8_t ch, uint8_t crc_check) {
    if (buf_c==MAVLINK_MAX_SIZE) {
        buf_c = 0;
	m_state = 0;
        return  1;
    }

    if (crc_check) {
      buf[buf_c++] = ch;
    }
    switch (m_state) {
    case MavParse_Idle:
      if (ch == MAVLINK_PACKET_START) {
        Reset();
        m_state = MavParse_PayloadLength;
      }
      break;
    case MavParse_PayloadLength:
      m_payloadLength = ch;
      m_state = MavParse_PacketSequence;
      break;
    case MavParse_Payload: //this arrives only after messageID
      if (m_id==MAVLINK_MSG_ID_MANUAL_CONTROL) {
        if (m_payloadByteParsedCount<(2*MAVLINK_MANUAL_CONTROL_CHANNELS)) //first 8 bytes (4 channels)
          if (m_payloadByteParsedCount%2==0) channel[m_payloadByteParsedCount/2] = 0; //reset channel value
          channel[m_payloadByteParsedCount/2] += ch<<(m_payloadByteParsedCount%2);
        } else if (m_payloadByteParsedCount<(2*MAVLINK_MANUAL_CONTROL_CHANNELS+2)) { //following 2 bytes (uint16) for buttons (up to 16 of them)
          if (m_payloadByteParsedCount%2==0) buttons_mask = 0; //reset button value
          buttons_mask += ch<<(m_payloadByteParsedCount%2);
        } 
      if (++m_payloadByteParsedCount >= m_payloadLength) {
        ++m_state;
      }
      break;
    case MavParse_PacketSequence:
      ++m_state;
      break;
    case MavParse_SystemID:
      ++m_state;
      break;
    case MavParse_ComponentID:
      ++m_state;
      break;
    case MavParse_MessageID:
      ++m_state;
      m_id = ch;
      break;
    case MavParse_CRC1:
      ++m_state;
      break;
    case MavParse_CRC2:
      m_state = MavParse_Idle;
      if (crc_check) {
        uint16_t crc = mavlink_crc(buf);
        uint16_t recv_crc = ((uint16_t)buf[buf_c-1]<<8) + buf[buf_c-2];
        buf_c = 0;
        if (crc!=recv_crc) {
	  m_state = 9;
          return 1;
        }
      }
      if (m_id==MAVLINK_MSG_ID_MANUAL_CONTROL) {
        m_id = 0;
        buf_c = 0;
	m_state = 0;
        return 3;
      }
      else {
        m_id = 0;
        buf_c = 0;
	m_state = 0;
        return 2;
      }
      break;
    }

    if (m_state<MavParse_MessageID) return 0; //parsing unknown message
    else if (crc_check) return 0; //waiting for entire message to check CRC
    else if (m_id==MAVLINK_MSG_ID_MANUAL_CONTROL) return 0; //parsing MANUAL_CONTROL

    buf_c=0;
    m_state = 0;
    return 2;
  }


  void Reset() {
    uint8_t i;
    for (i=0;i<MAVLINK_MANUAL_CONTROL_CHANNELS;i++)
	channel[i] = 1500;
    for (i=0;i<MAVLINK_MANUAL_CONTROL_BUTTONS;i++)
	button[i] = 1500;
    buf_c = 0;
    m_payloadLength = 0;
    m_payloadByteParsedCount = 0; // clear helper
    m_state = MavParse_Idle;
    
  }

  bool IsIdle() {
    return m_state == MavParse_Idle;
  }


#define MAV_HEADER_SIZE 6
/*
 * Calculates the MAVLink checksum on a packet in parameter buffer
 * and append it after the data
 */
static uint16_t mavlink_crc(uint8_t* buf)
{
  uint16_t ret = 0;
  register uint8_t length = buf[0];
  uint16_t sum = 0xFFFF;
  uint8_t i, stoplen;


  stoplen = length + MAV_HEADER_SIZE;

  i = 0;
  while (i<stoplen) {
    register uint8_t tmp;
    tmp = buf[i] ^ (uint8_t)(sum&0xff);
    tmp ^= (tmp<<4);
    sum = (sum>>8) ^ (tmp<<8) ^ (tmp<<3) ^ (tmp>>4);
    i++;
  }

  ret =  ((uint16_t)sum&0xFF << 8) + (sum>>8);

  return ret;
  //buf[length+MAV_HEADER_SIZE] = sum&0xFF;
  //buf[length+MAV_HEADER_SIZE+1] = sum>>8;
}

uint8_t getMID() { return m_id; }
uint8_t getMPayloadLength() { return m_payloadLength; }
uint8_t getMState() { return m_state; }

private:
  enum MavlinkParseState {
    MavParse_Idle, //0
    MavParse_PayloadLength, //1
    MavParse_PacketSequence, //2
    MavParse_SystemID, //3
    MavParse_ComponentID, //4
    MavParse_MessageID, //5
    MavParse_Payload, //6
    MavParse_CRC1, //7
    MavParse_CRC2 //8
  };

  uint8_t buf[MAVLINK_MAX_SIZE+1];
  uint8_t buf_c = 0;
  uint8_t m_payloadLength;
  uint8_t m_payloadByteParsedCount;
  uint8_t m_state;
  uint8_t m_id;
};



