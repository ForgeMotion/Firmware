/*====================================================================
The MIT License (MIT)

Copyright (c) 2014 Johan Gummesson

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
====================================================================*/

#include <Wire.h>
#include "I2Cdev.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define MPU_HZ 50

const uint8_t mpuInterruptPin = 0;
volatile bool mpuDataReady = false;

void setup()
{
    Wire.begin();

    Serial.begin(115200);
    while(!Serial);

    Serial.println("Starting; LED turns off when init complete.");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

//     initMPUwDebug();
    initMPU();

    mpuDataReady = false;

    digitalWrite(LED_BUILTIN, LOW);
}

void loop(){
    if (mpuDataReady){
        short gyro[3], accel[3];
        unsigned long timestamp;
        unsigned char more;
        int ret;

        long quat[4];
        float quaternion[4];
        short sensors;

        if (0 == (ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))){
            if (!more)
                mpuDataReady = false;
                
            for(int i=0; i<4; i++)
                quaternion[i] = float(quat[i]);

            quatLongToFloat(quat, quaternion);
            
            for(int i=0; i<4; i++){
                Serial.print(quaternion[i],9);
                if(i<3) Serial.print("\t");
            }
            Serial.println();
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

void quatLongToFloat(long *quatIn, float *quatOut){
    for(int i=0; i<4; i++)
      quatOut[i] = float(quatIn[i])/0x40000000;
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
}

/*

void normalizeQuat(float *quat){    
    float length = sqrt(quat[0] * quat[0] + quat[1] * quat[1] +  
        quat[1] * quat[1] + quat[1] * quat[1]);

    if (length == 0)
        return;

    for(int i=0; i<4; i++)
        quat[i] /= length;
}

 char _orientation[9] = { 1, 0, 0,
                          0, 1, 0,
                          0, 0, 1 };
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

    if (0 != (ret = dmp_set_orientation(matrixToScalar(_orientation)))){
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


unsigned short rowToScale(const char *row){
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;

    return b;
}

unsigned short matrixToScalar(const char *mtx){
    unsigned short scalar;

    scalar = rowToScale(mtx);
    scalar |= rowToScale(mtx + 3) << 3;
    scalar |= rowToScale(mtx + 6) << 6;

    return scalar;
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
