/* To Do
 *  Detect taps
 *  Disable accel/gyro from fifo - speeds up i2c transactions
 *  The dmp_read_fifo function returns number of packets left
 *  Use "more" to represent data ready?

Tap is working! Dir = 6 count = number before max time
 Tap settings: 
        dmp_set_tap_thresh(TAP_XYZ, 250);
        dmp_set_tap_axes(TAP_XYZ);
        dmp_set_tap_count(1);
        dmp_set_tap_time(100);
        dmp_set_tap_time_multi(500);

        dmp_set_shake_reject_thresh(GYRO_SF, 200);
        dmp_set_shake_reject_time(40);
        dmp_set_shake_reject_timeout(10);
*/
#include <Wire.h>
#include "I2Cdev.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "quaternionMath.h"

#define MPU_HZ 50

const uint8_t mpuInterruptPin = 0;
volatile bool mpuDataReady = false;
Quaternion currentQuat, initPos;
bool initSet = false;
float YPRdev[] = {0.0, 0.0, 0.0};
unsigned long timestamp = 0, lastTimestamp = 0;

short sensors;
uint8_t more;

void setup()
{
    Wire.begin();

    Serial.begin(115200);
    while(!Serial);

    Serial.println("Starting; LED turns off when init complete (has quat lib).");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

//     initMPUwDebug();
    initMPU();

    mpuDataReady = false;

    digitalWrite(LED_BUILTIN, LOW);
}

void loop(){
    if(millis() > 30000 && !initSet){
        initPos = currentQuat;
        initSet = true;

        Serial.print("Initial position set: ");
        Serial.print(initPos.w,9);
        Serial.print("\t");
        Serial.print(initPos.x,9);
        Serial.print("\t");
        Serial.print(initPos.y,9);
        Serial.print("\t");
        Serial.print(initPos.z,9);
        Serial.println();
    }
    if(initSet && timestamp != lastTimestamp){
        lastTimestamp = timestamp;
        
        getYPRDev(&initPos,&currentQuat,YPRdev);
        
        Serial.print("Deviation from init position: ");
        Serial.print(YPRdev[0]);
        Serial.print(", ");
        Serial.print(YPRdev[1]);
        Serial.print(", ");
        Serial.print(YPRdev[2]);
        Serial.print(", sensors = 0b");
        Serial.print(sensors, BIN);
        Serial.print(", more = 0b");
        Serial.print(more, BIN);
        Serial.println();
    }
    
    if (mpuDataReady){
        short gyro[3], accel[3];
        int ret;

        long quat[4];

        if (0 == (ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))){
            if (!more) mpuDataReady = false;

            longArrayToQuat(quat, &currentQuat);
            if(!initSet){
                Serial.print(currentQuat.w,9);
                Serial.print("\t");
                Serial.print(currentQuat.x,9);
                Serial.print("\t");
                Serial.print(currentQuat.y,9);
                Serial.print("\t");
                Serial.print(currentQuat.z,9);
                Serial.println();
            }
        }
        else{
            Serial.print("Error: ");
            Serial.println(ret);
        }
    }
}

static void mpuInterrupt(){
    mpuDataReady = true;
}

void longArrayToQuat(long *input, Quaternion *quat){
    quat->w = float(input[0])/0x40000000;
    quat->x = float(input[1])/0x40000000;
    quat->y = float(input[2])/0x40000000;
    quat->z = float(input[3])/0x40000000;
}



void initMPU(){
    struct int_param_s int_param;
    int_param.cb = mpuInterrupt;
    int_param.pin = mpuInterruptPin;

    mpu_init(&int_param);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(MPU_HZ);
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(0x88); // {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} orientation matrix
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
                                     | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL 
                                     | DMP_FEATURE_TAP);
    dmp_set_fifo_rate(MPU_HZ);
    mpu_set_dmp_state(1);
    dmp_register_tap_cb(tap_cb);
}

void tap_cb(unsigned char dir, unsigned char count){
    Serial.print("Tap detected! Direction: ");
    Serial.print(dir);
    Serial.print(" count: ");
    Serial.print(count);
    Serial.println();
}

/*
void initMPUwDebug(){
    int ret;

    struct int_param_s int_param;
    int_param.cb = mpuInterrupt;
    int_param.pin = mpuInterruptPin;

    Serial.println("Init MPU");

    if (0 != (ret = mpu_init(&int_param))){
        Serial.print("Failed to init mpu: ");
        Serial.println(ret);
    }

    if (0 != (ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))){
        Serial.print("Failed to set sensor: ");
        Serial.println(ret);
    }

    if (0 != (ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))){
        Serial.print("Failed to set sensor: ");
        Serial.println(ret);
    }

    if (0 != (ret = mpu_set_sample_rate(MPU_HZ))){
        Serial.print("Failed to set sensor: ");
        Serial.println(ret);
    }

    if (0 != (ret = dmp_load_motion_driver_firmware())){
        Serial.print("Failed to load dmp firmware: ");
        Serial.println(ret);
    }

    if (0 != (ret = dmp_set_orientation(0X88))){ // {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} orientation matrix
        Serial.print("Failed to set orientation: ");
        Serial.println(ret);
    }

    if (0 != (ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
                                     | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL 
                                     | DMP_FEATURE_TAP))){
        Serial.print("Failed to enable feature: ");
        Serial.println(ret);
    }

    if (0 != (ret = dmp_set_fifo_rate(MPU_HZ))){
        Serial.print("Failed to set fifo rate: ");
        Serial.println(ret);
    }

    if (0 != (ret = mpu_set_dmp_state(1))){
        Serial.print("Failed to set DMP state: ");
        Serial.println(ret);
    }

    printMPUValues();

    Serial.println("DONE!!!");
}


void printMPUValues(){
    unsigned short ushortValue;
    unsigned char ucharValue;
    float floatValue;

    mpu_get_accel_fsr(&ucharValue);
    Serial.print("Accel full-scale range rate: ");
    Serial.println(ucharValue);

    mpu_get_accel_sens(&ushortValue);
    Serial.print("Accel sensitivity: ");
    Serial.println(ushortValue);

    mpu_get_gyro_fsr(&ushortValue);
    Serial.print("Gyro full-scale range rate: ");
    Serial.println(ushortValue);

    mpu_get_gyro_sens(&floatValue);
    Serial.print("Gyro sensitivity: ");
    Serial.println(floatValue);

    mpu_get_lpf(&ushortValue);
    Serial.print("DLPF: ");
    Serial.println(ushortValue);

    dmp_get_fifo_rate(&ushortValue);
    Serial.print("FIFO rate: ");
    Serial.println(ushortValue);

    mpu_get_sample_rate(&ushortValue);
    Serial.print("Sample rate: ");
    Serial.println(ushortValue);
}

*/
