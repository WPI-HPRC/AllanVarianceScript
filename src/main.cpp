#include <Arduino.h>

static constexpr uint32_t HZ = 40;
static constexpr uint32_t DT = 1000000UL / HZ;
static constexpr uint32_t TOTAL_TIME = 7UL * 3600UL;
static constexpr uint32_t TOTAL_SAMPLES = HZ * TOTAL_TIME;

struct Sample {
    int32_t ax, ay, az;
    int32_t gx, gy, gz;
};

bool readSensorData(Sample &s) {
    //idk how to read sensor data lol

    s = {0, 0, 0, 0, 0, 0};
    return true;
}

void setup() {
    //Connect serial
    Serial.begin(921600);
    while(!Serial){
        delay(10);
    }

    //print csv header
    Serial.println("index,t_us,ax,ay,az,gx,gy,gz");
}

void loop(){
    static uint32_t start_us = micros();
    static uint32_t next_us = start_us;
    static uint32_t index = 0;
    static bool done = false;

    if(done){
        delay(1000);
        return;
    }

    uint32_t now_us = micros();

    if(((int32_t)(now_us - next_us)) < 0){
        //not sample time yet
        return;
    }

    while(((int32_t)(now_us - next_us)) >= 0){
        next_us += DT;
    }

    if (index >= TOTAL_SAMPLES) {
        done = true;
        Serial.println("DONE");
        return;
    }

    Sample s;
    bool success = readSensorData(s);

    uint32_t t_us = (uint32_t)(now_us - start_us);

    if(success){
        Serial.print(index); Serial.print(',');
        Serial.print(t_us); Serial.print(',');
        Serial.print(s.ax); Serial.print(',');
        Serial.print(s.ay); Serial.print(',');
        Serial.print(s.az); Serial.print(',');
        Serial.print(s.gx); Serial.print(',');
        Serial.print(s.gy); Serial.print(',');
        Serial.println(s.gz);
    }
    else {
        Serial.print(index); Serial.print(',');
        Serial.print(t_us); Serial.println(",ERROR,,,,,");

    }
    index++;

}