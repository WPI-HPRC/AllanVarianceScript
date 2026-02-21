#include <Arduino.h>
#include <Wire.h>

#if defined(USBCON)
#define DBG SerialUSB
#else
#define DBG Serial
#endif

//I2C pins
static constexpr uint8_t SDA_PIN = PB7;
static constexpr uint8_t SCL_PIN = PB6;

//ICM20948 libs
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20948 icm;

//ICM ic2 address
static constexpr uint8_t ICM_ADDR = 0x68;

//ASM libs
#include <ASM330LHHSensor.h>
static ASM330LHHSensor asmimu(&Wire);

#include <Adafruit_LPS2X.h>
Adafruit_LPS22 lps;

#include <SparkFun_u-blox_GNSS_v3.h>
SFE_UBLOX_GNSS GPS;

//time constants
static constexpr uint32_t HZ = 40;
static constexpr uint32_t DT = 1000000UL / HZ;
static constexpr uint32_t TOTAL_TIME = 7UL * 3600UL;
static constexpr uint32_t TOTAL_SAMPLES = HZ * TOTAL_TIME;

struct MAX10SData {
    float lat;
    float lon;
    float ecefX;
    float ecefY;
    float ecefZ;
    float altMSL;
    float altEllipsoid;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t epochTime;
    uint8_t satellites;
    uint8_t gpsLockType;
};

//Sample struct
struct Sample {
  float icm_ax, icm_ay, icm_az;
  float icm_gx, icm_gy, icm_gz;
  float icm_mx, icm_my, icm_mz;
  float asm_ax, asm_ay, asm_az;
  float asm_gx, asm_gy, asm_gz;
  float lps_p, lps_t;
  MAX10SData max10s;
};

static inline void zeroSample(Sample &s) {
  s.icm_ax = s.icm_ay = s.icm_az = 0.0f;
  s.icm_gx = s.icm_gy = s.icm_gz = 0.0f;
  s.icm_mx = s.icm_my = s.icm_mz = 0.0f;
  s.asm_ax = s.asm_ay = s.asm_az = 0.0f;
  s.asm_gx = s.asm_gy = s.asm_gz = 0.0f;
  s.lps_p = s.lps_t = 0.0f;
  s.max10s = {0};
}

//Read sensor data
static bool readSensorData(Sample &s) {
  zeroSample(s);

  //grab gps data
  s.max10s.lat = (float)GPS.getLatitude();
  s.max10s.lon = (float)GPS.getLongitude();
  s.max10s.velN = GPS.getNedNorthVel();
  s.max10s.velE = GPS.getNedEastVel();
  s.max10s.velD = GPS.getNedDownVel();
    /** 
  s.max10s.ecefX = (float)GPS.getHighResECEFX() * 0.01f;
  s.max10s.ecefY = (float)GPS.getHighResECEFY() * 0.01f;
  s.max10s.ecefZ = (float)GPS.getHighResECEFZ() * 0.01f;
  s.max10s.altMSL = (float)GPS.getAltitudeMSL() / 1000.0f;
  s.max10s.altEllipsoid = (float)GPS.getAltitude() / 1000.0f;
  s.max10s.epochTime = GPS.getUnixEpoch();
  s.max10s.satellites = GPS.getSIV();
  s.max10s.gpsLockType = GPS.getFixType();  
  */



  //grab lps data
  sensors_event_t pressure, lps_temp;
  lps.getEvent(&pressure, &lps_temp);
  s.lps_p = pressure.pressure;
  s.lps_t = lps_temp.temperature;

  //grab icm data
  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  s.icm_ax = accel.acceleration.x * 0.980665f;
  s.icm_ay = accel.acceleration.y * 0.980665f;
  s.icm_az = accel.acceleration.z * 0.980665f;

  s.icm_gx = gyro.gyro.x;
  s.icm_gy = gyro.gyro.y;
  s.icm_gz = gyro.gyro.z;

  s.icm_mx = mag.magnetic.x;
  s.icm_my = mag.magnetic.y;
  s.icm_mz = mag.magnetic.z;

  //gram asm data
  int32_t acc[3] = {0,0,0};
  int32_t gyr[3] = {0,0,0};

  asmimu.Get_X_Axes(acc);
  asmimu.Get_G_Axes(gyr);

  s.asm_ax = acc[0] * 0.00980665f;
  s.asm_ay = acc[1] * 0.00980665f;
  s.asm_az = acc[2] * 0.00980665f;
  s.asm_gx = gyr[0] / 1000.0f * (PI / 180.0f);
  s.asm_gy = gyr[1] / 1000.0f * (PI / 180.0f);
  s.asm_gz = gyr[2] / 1000.0f * (PI / 180.0f);

  return true;
}


void setup() {
  DBG.begin(115200);
  delay(6000);

  //I2C
  Wire.setSDA(PB_7);
  Wire.setSCL(PB_6);
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  //init gps
  if (GPS.begin()) {
    GPS.setNavigationRate(1);
    GPS.setAutoPVT(true);
    Serial.println("GPS init OK");
    } 
    else {
      Serial.println("GPS init FAILED");
    }

  //init baro
  if(!lps.begin_I2C(0x5C)) {
    DBG.println("LPS init failed");
  } else {
    DBG.println("LPS init OK");
  }

  lps.setDataRate(LPS22_RATE_50_HZ);

  //ASM init
  int asmStatus = asmimu.begin();
  if (asmStatus != 0) {
    DBG.print("ASM330 init failed: ");
    DBG.println(asmStatus);
  } else {
    DBG.println("ASM330 init OK");
    asmimu.Enable_X();
    asmimu.Enable_G();
  }

  //ICM init
  if (!icm.begin_I2C(0x68, &Wire)) {
    DBG.println("ICM init failed");
  } else {
    DBG.println("ICM init OK");
  }

  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  //Configure ICM
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

  icm.setAccelRateDivisor(0);  
  icm.setGyroRateDivisor(0);    

  DBG.println("READY");
  DBG.println("index,t_us,icm_ax,icm_ay,icm_az,icm_gx,icm_gy,icm_gz,icm_mx,icm_my,icm_mz,asm_ax,asm_ay,asm_az,asm_gx,asm_gy,asm_gz,lsm_p,lsm_t,max10s.lat,max10s.lon,max10s.velN,max10s.velE,max10s.velD");
}

void loop() {
  static uint32_t start_us = micros();
  static uint32_t next_us  = start_us;
  static uint32_t index = 0;
  static bool done = false;

  if (done) {
    delay(1000);
    return;
  }

  uint32_t now = micros();
  if ((int32_t)(now - next_us) < 0) return;
  next_us += DT;

  if (index >= TOTAL_SAMPLES) {
    done = true;
    return;
  }

  Sample s;
  readSensorData(s);

  uint32_t t_us = now - start_us;

  DBG.print(index); DBG.print(',');
  DBG.print(t_us); DBG.print(',');
  DBG.print(s.icm_ax, 5); DBG.print(',');
  DBG.print(s.icm_ay, 5); DBG.print(',');
  DBG.print(s.icm_az, 5); DBG.print(',');
  DBG.print(s.icm_gx, 5); DBG.print(',');
  DBG.print(s.icm_gy, 5); DBG.print(',');
  DBG.print(s.icm_gz, 5); DBG.print(',');
  DBG.print(s.icm_mx); DBG.print(',');
  DBG.print(s.icm_my); DBG.print(',');
  DBG.print(s.icm_mz); DBG.print(',');
  DBG.print(s.asm_ax, 5); DBG.print(',');
  DBG.print(s.asm_ay, 5); DBG.print(',');
  DBG.print(s.asm_az, 5); DBG.print(',');
  DBG.print(s.asm_gx, 5); DBG.print(',');
  DBG.print(s.asm_gy, 5); DBG.print(',');
  DBG.print(s.asm_gz, 5); DBG.print(',');
  DBG.print(s.lps_p); DBG.print(',');
  DBG.print(s.lps_t); DBG.print(',');
  DBG.print(s.max10s.lat); DBG.print(',');
  DBG.print(s.max10s.lon); DBG.print(',');
  DBG.print(s.max10s.velN); DBG.print(',');
  DBG.print(s.max10s.velE); DBG.print(',');
  DBG.println(s.max10s.velD);

  index++;
}