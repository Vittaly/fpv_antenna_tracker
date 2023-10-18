#include <Arduino.h>

enum ltmStates
{
  IDLE,
  HEADER_START1,
  HEADER_START2,
  HEADER_MSGTYPE,
  HEADER_DATA
};

/*
 * LTM based on https://github.com/KipK/Ghettostation/blob/master/GhettoStation/LightTelemetry.cpp implementation
   https://github.com/DzikuVx/ltm_telemetry_reader/blob/master/ltm_telemetry_reader.ino
 */

#define LONGEST_FRAME_LENGTH 14

#define G_FRAMELENGTH LONGEST_FRAME_LENGTH
#define A_FRAMELENGTH 6
#define S_FRAMELENGTH 7
#define O_FRAMELENGTH LONGEST_FRAME_LENGTH
#define N_FRAMELENGTH 6
#define X_FRAMELENGTH 6

enum flightModes
{
  Manual,
  Rate,
  Angle,
  Horizon,
  Acro,
  Stabilized1,
  Stabilized2,
  Stabilized3,
  Altitude_Hold,
  GPS_Hold,
  Waypoints,
  Head_free,
  Circle,
  RTH,
  FollowMe,
  Land,
  FlyByWireA,
  FlyByWireB,
  Cruise,
  Unknown
};

enum FrameCode
{
  UNKNOWN = 0,
  G = 'G',
  A = 'A',
  S = 'S',
  O = 'O',
  N = 'N',
  X = 'X'
};

typedef struct
{
  FrameCode code;
  uint8_t length;
} FrameProperties;


#define FRAME_TYPES_COUNT 6
const FrameProperties FrameTypeList[FRAME_TYPES_COUNT + 1] =
    {
        (FrameProperties){FrameCode::G, G_FRAMELENGTH},
        (FrameProperties){FrameCode::A, A_FRAMELENGTH},
        (FrameProperties){FrameCode::S, S_FRAMELENGTH},
        (FrameProperties){FrameCode::O, O_FRAMELENGTH},
        (FrameProperties){FrameCode::N, N_FRAMELENGTH},
        (FrameProperties){FrameCode::X, X_FRAMELENGTH},
        (FrameProperties){FrameCode::UNKNOWN, 0}
        };

#define FRAME_TYPE_UNKNOWN FrameTypeList[FRAME_TYPES_COUNT]    


enum GPSFix: uint8_t {
 NO_FIX = 0,
 FIX_2D = 1,
 FIX_3D = 2
};

struct GPSFrameData_s {
  int32_t latitude {0};
  int32_t longitude {0};
  int32_t altitude {0};
  uint8_t groundSpeed {0};
  GPSFix gpsFix {GPSFix::NO_FIX};
  uint8_t gpsSats {0};

};

struct AttitudeFrameData_s{
int pitch{0};
  int roll{0};
  int heading{0};

};




struct StatusFrameData_s{
  uint16_t vbat;
  uint16_t current;
  u_char rssi;
  u_char airSpeed;
  bool armed {false};
  bool failSafe {false};
  byte flightMode {flightModes::Unknown};
};


struct OriginFrameData_s{
  int32_t latitude{0};
  int32_t longitude{0};
  int32_t altitude{0};
  u_char osdOn {0};
  u_char fix {0};
};

struct NavigationFrameData_s{
  u_char gpsMode;
  u_char navMode;
  u_char navAction;
  u_char waypointN;
  u_char nanErr;
  u_char flags;
};

struct ExtendedFrameData_s{
  uint16_t hhop;
  byte sensorStatus;
};



struct RemoteData_s
{
  GPSFrameData_s gpsInfo;
  AttitudeFrameData_s attitudeInfo;
  StatusFrameData_s statusInfo;
  OriginFrameData_s originInfo;
  NavigationFrameData_s navigationInfo;
  ExtendedFrameData_s extendedInfo;


  

  
  uint16_t hdop {UINT16_MAX};


  int32_t homeLatitude{0};
  int32_t homeLongitude{0};

  uint8_t sensorStatus{0};
};

class LTMParser
{
private:
  uint8_t serialBuffer[LONGEST_FRAME_LENGTH];
  uint8_t state = IDLE;
  FrameProperties frameProperties = FRAME_TYPE_UNKNOWN;

  byte receiverIndex;
  RemoteData_s telemetryData;

  byte readByte(uint8_t offset);
  u_char checkSumCalc = 0;
  int readInt(uint8_t offset);
  int32_t readInt32(uint8_t offset);

  bool attitudeDataUpdated = false;
  bool extendDataUpdated = false;
  bool gpsDataUpdated = false;
  bool statusDataUpdated = false;

  bool lastFrameCheckSumErr = false;

public:
  bool parseChar(char data);
  bool isGpsDataUpdated();

  RemoteData_s getTelemetryData();
  GPSFrameData_s& getGPSData();
};
