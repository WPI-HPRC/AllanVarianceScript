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

//time constants
static constexpr uint32_t HZ = 40;
static constexpr uint32_t DT = 1000000UL / HZ;
static constexpr uint32_t TOTAL_TIME = 7UL * 3600UL;
static constexpr uint32_t TOTAL_SAMPLES = HZ * TOTAL_TIME;

//Sample struct
struct Sample {
  float icm_ax, icm_ay, icm_az;
  float icm_gx, icm_gy, icm_gz;
  int32_t asm_ax, asm_ay, asm_az;
  int32_t asm_gx, asm_gy, asm_gz;
};

static inline void zeroSample(Sample &s) {
  s.icm_ax = s.icm_ay = s.icm_az = 0;
  s.icm_gx = s.icm_gy = s.icm_gz = 0;
  s.asm_ax = s.asm_ay = s.asm_az = 0;
  s.asm_gx = s.asm_gy = s.asm_gz = 0;
}

//Read sensor data
static bool readSensorData(Sample &s) {
  zeroSample(s);


  //grab icm data
  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  s.icm_ax = accel.acceleration.x;
  s.icm_ay = accel.acceleration.y;
  s.icm_az = accel.acceleration.z;

  s.icm_gx = gyro.gyro.x;
  s.icm_gy = gyro.gyro.y;
  s.icm_gz = gyro.gyro.z;

  //gram asm data
  int32_t acc[3] = {0,0,0};
  int32_t gyr[3] = {0,0,0};

  asmimu.Get_X_Axes(acc);
  asmimu.Get_G_Axes(gyr);

  s.asm_ax = acc[0];
  s.asm_ay = acc[1];
  s.asm_az = acc[2];
  s.asm_gx = gyr[0];
  s.asm_gy = gyr[1];
  s.asm_gz = gyr[2];

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

  // ---- Configure ICM20948 (REQUIRED) ----
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

  icm.setAccelRateDivisor(0);  
  icm.setGyroRateDivisor(0);    

  DBG.println("READY");
  DBG.println("index,t_us,icm_ax,icm_ay,icm_az,icm_gx,icm_gy,icm_gz,asm_ax,asm_ay,asm_az,asm_gx,asm_gy,asm_gz");
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
  DBG.print(s.icm_ax); DBG.print(',');
  DBG.print(s.icm_ay); DBG.print(',');
  DBG.print(s.icm_az); DBG.print(',');
  DBG.print(s.icm_gx); DBG.print(',');
  DBG.print(s.icm_gy); DBG.print(',');
  DBG.print(s.icm_gz); DBG.print(',');
  DBG.print(s.asm_ax); DBG.print(',');
  DBG.print(s.asm_ay); DBG.print(',');
  DBG.print(s.asm_az); DBG.print(',');
  DBG.print(s.asm_gx); DBG.print(',');
  DBG.print(s.asm_gy); DBG.print(',');
  DBG.println(s.asm_gz);

  index++;
}