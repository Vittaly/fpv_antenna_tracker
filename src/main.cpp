#include <Arduino.h>

// #include <AccelStepper.h>

#include <SpeedyStepper.h>
#include <AltAzimuttRange.h>
#include <ltmParser.h>

/*#include <minimal/mavlink.h>
#include <common/mavlink_msg_global_position_int.h>
#include <common/mavlink_msg_command_int.h>
#include <common/mavlink_msg_param_set.h>
#include <common/mavlink_msg_param_value.h>
#include <common/mavlink_msg_param_request_list.h>
#include <common/common.h>
#include <minimal/minimal.h>*/

// compass

// first motor (azimuth) pins
#define EN_PIN PA0   // Enable - PURPLE
#define DIR_PIN PA1  // Direction - WHITE
#define STEP_PIN PA2 // Step - ORANGE

#define DIR_PIN2 PA3  // Direction - WHITE
#define STEP_PIN2 PA4 // Step - ORANGE

// first motor (azimuth) gear ratio and driver microsteps
#define FIRST_MOTOR_GEAR_RATIO 7.571428571428571 // 106:14
#define FIRST_MOTOR_STEPS 200                    // steps per Revolution
#define FIRST_MOTOR_MICROSTEPS 16                // microsteps per step

#define SECOND_MOTOR_GEAR_RATIO 12.42857142857143 // 174:14
#define SECOND_MOTOR_STEPS 200                    // steps per Revolution
#define SECOND_MOTOR_MICROSTEPS 16                // microsteps per step

#define MIN_ELEVATION_DEGREE -18 // gear limit. look at construction

SpeedyStepper stepper1;
SpeedyStepper stepper2;

HardwareTimer tim4(TIM4);

volatile bool LEDOn13 = true;

volatile bool invert_direction = false;
volatile bool IsMotorMotionProcessing = false;

bool isHomeSetted = false;

AltAzimuthRange altAzimuthRange;
GpsData gp;
LTMParser ltpParser;
AzimuthInfo ai, ai_cur;
int16_t fullRevolutions = 0;

/* const u_int8_t MAV_SYS_ID = 200;
const MAV_COMPONENT MAV_COMP = MAV_COMP_ID_SYSTEM_CONTROL; */

void motorMotionProcessing()
{
  /*if (IsMotorMotionProcessing)
    return;                       // if processing already started just exit
  IsMotorMotionProcessing = true;*/
  // set blocking flag
  stepper1.processMovement();
  stepper2.processMovement();
  /* IsMotorMotionProcessing = false; */ // reset blocking flag
}

void firstMotorDriverSetup()
{

  pinMode(EN_PIN, OUTPUT);   // Set pinmodes
  digitalWrite(EN_PIN, LOW); // Enable board

  stepper1.connectToPins(STEP_PIN, DIR_PIN);
  stepper1.setStepsPerRevolution((FIRST_MOTOR_STEPS * FIRST_MOTOR_MICROSTEPS * FIRST_MOTOR_GEAR_RATIO));

  stepper1.setSpeedInRevolutionsPerSecond(0.8l);
  stepper1.setAccelerationInRevolutionsPerSecondPerSecond(0.4l);
}

void secondMotorDriverSetup()
{
  /* #if SECOND_MOTOR_DRIVER == TMC220x
    Stepper1Serial.begin(11520);

    TMCdriver.begin();          // UART: Init SW UART (if selected) with default 115200 baudrate
    TMCdriver.toff(5);          // Enables driver in software
    TMCdriver.rms_current(500); // Set motor RMS current
    TMCdriver.microsteps(256);  // Set microsteps

    TMCdriver.en_spreadCycle(false);
    TMCdriver.pwm_autoscale(true); // Needed for stealthChop
  #endif */

  stepper2.connectToPins(STEP_PIN2, DIR_PIN2);
  stepper2.setStepsPerRevolution((SECOND_MOTOR_STEPS * SECOND_MOTOR_MICROSTEPS * SECOND_MOTOR_GEAR_RATIO));

  stepper2.setSpeedInRevolutionsPerSecond(0.8l);
  stepper2.setAccelerationInRevolutionsPerSecondPerSecond(.4l);
}

void func_tim_4() // обработчик прерывания
{
  motorMotionProcessing();
}

void tim4_setup()
{

  tim4.pause();
  tim4.setOverflow(2000, HERTZ_FORMAT);
  // tim4.setPrescaleFactor(512);
  tim4.attachInterrupt(func_tim_4);
  tim4.resume();
}

#define COMPASS_ENABLE
#define GPS_ENABLE

// #define BRUSH

#ifdef COMPASS_ENABLE
#define COMPASS_CALIBRATE_ROTATION_STEP 0.005
#define COMPASS_READ_SMOOTHY_COUNT 5

#define COMPASS_ADR 0x0D
// #define CMP_SLC PB6
// #define CMP_CDA PB7
#define COMPASS_ORIENTATION_CORRECT -90

#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setupCompass()
{

  compass.init();
  compass.setMagneticDeclination(COMPASS_ORIENTATION_CORRECT + 15, 25); // -90 compass orientation compensate
  compass.setSmoothing(COMPASS_READ_SMOOTHY_COUNT, false);
}

void compassReadSmoothy()
{
  digitalWrite(EN_PIN, HIGH); // Disable motors
  delay(100);
  for (int i = 0; i < COMPASS_READ_SMOOTHY_COUNT; i++)
  {
    compass.read();
  }
  digitalWrite(EN_PIN, LOW); // Enable motors
}

void setNorth()
{
  int comp_err;

  SerialUSB.println("Start compass calibrating...");

  int xMin = 0, xMax = 0, yMin = 0, yMax = 0, zMin = 0, zMax = 0, az, stepForNorth = 0;
  for (float i = 0; i <= 1; i += COMPASS_CALIBRATE_ROTATION_STEP)
  {
    stepper1.setupMoveInRevolutions(i);
    while (!stepper1.motionComplete())
      delay(2);
    compassReadSmoothy();

    // comp_err = compass.read();

    if (compass.getX() > xMax)
      xMax = compass.getX();
    if (compass.getX() < xMin)
      xMin = compass.getX();
    if (compass.getY() > yMax)
      yMax = compass.getY();
    if (compass.getY() < yMin)
      yMin = compass.getY();
    if (compass.getZ() > zMax)
      zMax = compass.getZ();
    if (compass.getZ() < zMax)
      zMax = compass.getZ();
#ifdef DEBUG_INFO
    SerialUSB.println();
    SerialUSB.println("-------");
    SerialUSB.print("Step:");
    SerialUSB.println(i);
    // SerialUSB.print("Cmp_err:");
    // SerialUSB.println(comp_err);
    SerialUSB.print("xMin:");
    SerialUSB.println(xMin);
    SerialUSB.print("xMax:");
    SerialUSB.println(xMax);
    SerialUSB.print("yMin:");
    SerialUSB.println(yMin);
    SerialUSB.print("yMax:");
    SerialUSB.println(yMax);
    SerialUSB.print("zMin:");
    SerialUSB.println(zMin);
    SerialUSB.print("zMax:");
    SerialUSB.println(zMax);
#endif
  }
  compass.setCalibration(xMin, xMax, yMin, yMax, zMin, zMax);
  compassReadSmoothy();
  az = compass.getAzimuth();
  if (az < 0)
    az += 360;
#ifdef DEBUG_INFO
  SerialUSB.print("Azimuth:");
  SerialUSB.println(az);
#endif
  for (float i = 0; i <= 1; i += COMPASS_CALIBRATE_ROTATION_STEP)
  {
    stepper1.moveToPositionInRevolutions(i);

    compassReadSmoothy();
    if ((compass.getAzimuth() < 0 ? compass.getAzimuth() + 360 : compass.getAzimuth()) < az)
    {
      az = compass.getAzimuth();
      if (az < 0)
        az += 360;
      stepForNorth = stepper1.getCurrentPositionInSteps();
#ifdef DEBUG_INFO
      SerialUSB.println("Mim azimuth refreshed");
#endif
    }
#ifdef DEBUG_INFO
    SerialUSB.print("Min azimuth:");
    SerialUSB.println(az);
    SerialUSB.print("Cur azimuth:");
    SerialUSB.println(compass.getAzimuth());
#endif
  }
#ifdef DEBUG_INFO
  SerialUSB.print("stepForNorth:");
  SerialUSB.println(stepForNorth);
#endif

  stepper1.moveToPositionInSteps(stepForNorth);
  stepper1.setCurrentPositionInSteps(0);

  for (int i = 0; i < 3; i++)
  {
    stepper1.moveToPositionInRevolutions(0.025);
    stepper1.moveToPositionInRevolutions(-0.025);
  }
  stepper1.moveToPositionInRevolutions(0);
}
#endif

#ifdef GPS_ENABLE
// #include <Adafruit_GPS.h>
// #include <iarduino_GPS_NMEA.h>
#include <TinyGPS++.h>
#define MAX_WAIT_GPS_SEC 300

TinyGPSPlus gps;

void updateGpsDate()
{

  byte c;

  // read and parse gps date
  while (!gps.location.isUpdated() || !gps.altitude.isUpdated())
    if (Serial1.available() > 0)
    {
      c = Serial1.read();
      if (c >= 0)
      {
#ifdef DEBUG_GPS_INFO
        SerialUSB.write(c);
#endif
        gps.encode(c);
      }
    }
}
#define GPS_AVG_COUNT 20
#define max( a, b) (a > b ? a :b)
#define min(a, b) (a < b ? a :b)
void setLocation()
{
  SerialUSB.println("Start GPS reading...");
  Serial1.begin(9600);

  double avgLat, avgLon, avgAlt, latMax, latMin, lonMin, lonMax, altMin, altMAx;
  avgLat = 0;
  avgLon = 0;
  avgAlt = 0;
  double latlist[GPS_AVG_COUNT];
  double lonlist[GPS_AVG_COUNT];
  double altlist[GPS_AVG_COUNT];

  for (uint16_t i = 0; i < MAX_WAIT_GPS_SEC * 2; i++)
  {

    updateGpsDate();

#ifdef DEBUG_INFO
    SerialUSB.print("hhop:");
    SerialUSB.print(gps.hdop.hdop());
    SerialUSB.print(" sats:");
    SerialUSB.println(gps.satellites.value());
#endif
    if (gps.satellites.value() > 5 and gps.hdop.hdop() <= 10)
      break;
    delay(500);
  }
  uint8_t i = 0;
  do
  {

    updateGpsDate();
    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    { 
      latlist[i]= gps.location.lat();
      lonlist[i] = gps.location.lng();
      altlist[i] = gps.altitude.meters();

      if (i ==0) { 
        latMin = latMax = latlist[i];
        lonMin = lonMax = lonlist[i];
        altMin = altMAx = altlist[i];
      
      } else {
        latMax = max (latMax, latlist[i]);
        latMin = min (latMax, latlist[i]);
        lonMax = max (lonMax, lonlist[i]);
        lonMin = min (lonMin, lonlist[i]);
        altMin = min (altMin, altlist[i]);
        altMAx = max (altMAx, altlist[i]);
      }


      ++i;
    }
  }   
    while (i < GPS_AVG_COUNT);
  

   for (i = 0; i < GPS_AVG_COUNT; i++) {

    if (latMin != latlist[i] && latMax != latlist[i]) avgLat += latlist[i];
    if (lonMax != lonlist[i] && lonMin != lonlist[i]) avgLon += lonlist[i];
     if (altMAx != altlist[i] && altMin != altlist[i]) avgAlt += altlist[i];
   }
    avgLat /= GPS_AVG_COUNT - 2;
    avgLon /= GPS_AVG_COUNT - 2;
    avgAlt /= GPS_AVG_COUNT - 2;

    altAzimuthRange.setObserverLocation({avgLat, avgLon, avgAlt});

#ifdef DEBUG_INFO
    SerialUSB.println("set home at:");
    SerialUSB.print("Lat:");
    SerialUSB.println(avgLat, 6);

    SerialUSB.print("Lon:");
    SerialUSB.println(avgLon, 6);

    SerialUSB.print(" Alt:");
    SerialUSB.print(avgAlt, 3);
#endif

    Serial1.end();
    isHomeSetted = true;
  }

#endif

  void setup()
  {
    // delay(10000);
    firstMotorDriverSetup();
    secondMotorDriverSetup();

    pinMode(LED_BUILTIN, OUTPUT);

    // tim4_setup();

    SerialUSB.begin();

#ifdef COMPASS_ENABLE
    setupCompass();
    setNorth();
#endif
#ifdef GPS_ENABLE
    setLocation();
#endif
#ifdef BRUSH
    stepper1.setupMoveInRevolutions(10);
#endif
  }

  /* mavlink_status_t status;
  mavlink_message_t msg;
  uint8_t chan = MAVLINK_COMM_0;
  mavlink_global_position_int_t gp;
  mavlink_command_int_t mci;
  mavlink_param_request_list_t prl; */

  void loop()
  {
#ifndef BRUSH
    if (SerialUSB.available())
    {
      uint8_t byte = SerialUSB.read();
      if (ltpParser.parseChar(byte) && ltpParser.isGpsDataUpdated())
      {
#ifdef DEBUG_INFO
        SerialUSB.println("received LTP frame");
#endif

        GPSFrameData_s telemetry = ltpParser.getGPSData();
        if (isHomeSetted)
        {
          ai_cur = ai;
          ai = altAzimuthRange.calculate({static_cast<FPD_TYPE>(telemetry.latitude / 1E7),
                                          static_cast<FPD_TYPE>(telemetry.longitude / 1E7),
                                          static_cast<FPD_TYPE>(telemetry.altitude / 1E2)});

          ai.az+= 360 * fullRevolutions;                             

          if (ai.az - 180 > ai_cur.az)
          {
            ai.az -= 360; // short way through zero
            fullRevolutions--;
          }

          else if (ai_cur.az - 180 > ai.az)
          {
            ai.az += 360; // short way through zero
            fullRevolutions++;
          }

          if (ai.ele < MIN_ELEVATION_DEGREE)
            ai.ele = MIN_ELEVATION_DEGREE;

          // if motors motion still incomplete skip new data
          if (stepper1.motionComplete() && stepper2.motionComplete())
          {
            stepper1.setupMoveInRevolutions(ai.az / 360);
            stepper2.setupMoveInRevolutions(ai.ele / 360);
          }
          else
          {
#ifdef DEBUG_INFO
            SerialUSB.println("motion incomplite. skeep moving");
#endif
          }

#ifdef DEBUG_INFO
          SerialUSB.println("Look at position:");
          SerialUSB.print("lat:");
          SerialUSB.println(static_cast<FPD_TYPE>(telemetry.latitude / 1E7), 8);
          SerialUSB.print("lon:");
          SerialUSB.println(static_cast<FPD_TYPE>(telemetry.longitude / 1E7), 8);
          SerialUSB.print("alt:");
          SerialUSB.println(static_cast<FPD_TYPE>(telemetry.altitude / 1E2), 8);
          SerialUSB.println("-----");
          SerialUSB.print("az:");
          SerialUSB.println(ai.az, 8);
          SerialUSB.print("ele:");
          SerialUSB.println(ai.ele, 8);
          SerialUSB.print("dist:");
          SerialUSB.println(ai.dist, 8);

#endif
        }
        else
        {
          altAzimuthRange.setObserverLocation({static_cast<FPD_TYPE>(telemetry.latitude / 1E7),
                                               static_cast<FPD_TYPE>(telemetry.longitude / 1E7),
                                               static_cast<FPD_TYPE>(telemetry.altitude / 1E2)});
#ifdef DEBUG_INFO
          SerialUSB.println("Set home position:");
          SerialUSB.print("lat:");
          SerialUSB.println(telemetry.latitude / 1E7);
          SerialUSB.print("lon:");
          SerialUSB.println(telemetry.longitude / 1E7);
          SerialUSB.print("alt:");
          SerialUSB.println(telemetry.altitude / 1E2);
#endif

          isHomeSetted = true;
        }
      }
    }
#else
  if (stepper1.motionComplete())
    stepper1.setupMoveInRevolutions(-stepper1.getCurrentPositionInRevolutions());
#endif

    motorMotionProcessing();
  }

  void yield(void)
  {
    motorMotionProcessing();
  }