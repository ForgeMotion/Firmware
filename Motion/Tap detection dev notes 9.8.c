Tap Detection
/*	It looks like we got access to the library from Invensense (MotionDriver 6.1) which has commands for all of the features, and there's Arm 32 bit example code. 
	I can implement this library by defining the appropriate I2C commands.. This should all be tested on a normal teensy and mpu-6050 breakout.
	Good news: you can detect taps in only particular directions (e.g. Z backwards), you can set it to only trigger when 2 (or 1 or 3 or 4) taps are detected, and you can adjust the threshold and timing.
	Adding the library to the code folder (MPU6050_MotionDriver_6_1).

	Definitely should be using the latest MPU-6050 library from the "develop" (testing) branch. It has a lot of fixes. Going to merge in my Euler angle and other fixes locally. 

*/
Starting with 
	The example code
		enumerates packet_type_e to sort quickly through messages from fifo, one type is PACKET_TYPE_TAP

		from forum thread: "The way to get interrupt on orientation is very similar to the way to tap interrupt. The two lines of code provided here dmp_enable_feature(DMP_FEATURE_TAP) and dmp_register_tap_cb(tap_cb) are sufficient."

		instructions from example code:
		     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in inv_mpu_dmp_motion_driver.h into the MPU memory.
		     * 2. Push the gyro and accel orientation matrix to the DMP.
		     * 3. Register gesture callbacks. Don't worry, these callbacks won't be executed unless the corresponding feature is enabled.
		     * 4. Call dmp_enable_feature(mask) to enable different features.
		     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
		     * 6. Call any feature-specific control functions.

		     1, 2, 5 and 6 are handled my i2cdev lib
		     3, dmp_register_tap_cb(tap_cb) as mentioned in forum comment above
		     4, dmp_enable_feature(DMP_FEATURE_TAP) as mentioned in forum comment above

		     the tap callback function, tap_cb, sends a packet (i2c?) of char buf = {$, PACKET_TYPE_TAP, direction, count}

/**
*  @brief      Register a function to be executed on a tap event.
*  The tap direction is represented by one of the following:
*  \n TAP_X_UP
*  \n TAP_X_DOWN
*  \n TAP_Y_UP
*  \n TAP_Y_DOWN
*  \n TAP_Z_UP
*  \n TAP_Z_DOWN
*  @param[in]  func    Callback function.
*  @return     0 if successful.
*/
int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char))
{
    dmp.tap_cb = func;
    return 0;
}
dmp_enable_feature(DMP_FEATURE_TAP)
int dmp_enable_feature(unsigned short mask){
	// ... bunch of stuff gets ignored

    /* Send gesture data to the FIFO. */
    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        tmp[0] = DINA20; // #defined as 0x20
    else
        tmp[0] = 0xD8;
    mpu_write_mem(CFG_27,1,tmp);

    // ... other stuff

    if (mask & DMP_FEATURE_TAP) {
        /* Enable tap. */
        tmp[0] = 0xF8;
        mpu_write_mem(CFG_20, 1, tmp);
        dmp_set_tap_thresh(TAP_XYZ, 250);
        dmp_set_tap_axes(TAP_XYZ);
        dmp_set_tap_count(1);
        dmp_set_tap_time(100);
        dmp_set_tap_time_multi(500);

        dmp_set_shake_reject_thresh(GYRO_SF, 200);
        dmp_set_shake_reject_time(40);
        dmp_set_shake_reject_timeout(10);
    } else {
        tmp[0] = 0xD8;
        mpu_write_mem(CFG_20, 1, tmp);
    }

    // ... more stuff


    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        dmp.packet_length += 4;

    return 0;
}

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)

#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)

from dmp_read_fifo(...){
	// gets the packet, extracts data, and figures out offset within packet (unsigned char ii) depending on what features are enabled
	// For nothing but normal quat enabled, ii = 16

	/* Gesture data is at the end of the DMP packet. Parse it and call
     * the gesture callbacks (if registered).
     */
    if (dmp.feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        decode_gesture(fifo_data + ii); // unsigned char fifo_data[] is an array, so it's being passed as a pointer with offset
}

/**
 *  @brief      Decode the four-byte gesture data and execute any callbacks.
 *  @param[in]  gesture Gesture data from DMP packet.
 *  @return     0 if successful.
 */
static int decode_gesture(unsigned char *gesture)
{
    unsigned char tap, android_orient;

    android_orient = gesture[3] & 0xC0; // 0b11000000 << 6
    tap = 0x3F & gesture[3]; // 0b00111111

    if (gesture[1] & INT_SRC_TAP) {  // if lower bit of second byte is 1, we have a tap
        unsigned char direction, count;
        direction = tap >> 3;
        count = (tap % 8) + 1;
        if (dmp.tap_cb)
            dmp.tap_cb(direction, count);
    }

    if (gesture[1] & INT_SRC_ANDROID_ORIENT) {
        if (dmp.android_orient_cb)
            dmp.android_orient_cb(android_orient >> 6);
    }

    return 0;
}


// ======================== My understanding of getting the gesture data ============================
ok: so we get the bytes from the packet (an extra 4 bytes, according to end of dmp_enable_feature())
decode_gesture() then sends the information to our callback function:

if(packet[1] & 0x01) // then we got a tap message
direction = (packet[3] & 0b00111111) >> 3;
count = ((packet[3] & 0b00111111) % 8) + 1; // % 8 is same as & (8-1) or & 0b111

could simply rewrite as:
direction = (packet[3] >> 3) & 0b111;
count = (packet[3] & 0b111) + 1;

dmp.tap_cb(direction, count);

which was registered beforehand - in example code, it just prints it over usb.

count = number of taps
direction =
#define TAP_X_UP            (0x01) 0b001
#define TAP_X_DOWN          (0x02) 0b010
#define TAP_Y_UP            (0x03) 0b011
#define TAP_Y_DOWN          (0x04) 0b100
#define TAP_Z_UP            (0x05) 0b101
#define TAP_Z_DOWN          (0x06) 0b110


// ======================== My understanding of enabling tap detection ============================
#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

 /* Send gesture data to the FIFO. */
when dmp_enable_feature(DMP_FEATURE_TAP) gets called, it does the following (skipping some probably-unneeded messages)
    mpu_write_mem(CFG_27,1,0x20);
    mpu_write_mem(CFG_20, 1, 0xF8);
    dmp_set_tap_thresh(TAP_XYZ, 250); 
    	/* this scales the threshold according to sample rate and full scale range of accelerometer, first 4 bytes of tmp
			then it uses mpu_write_mem to different registers for each axis if they're enabled
    	*/
    dmp_set_tap_axes(TAP_XYZ); // mpu_write_mem(D_1_72,1,tmp) where temp is 6 bits that determine proper axes
    dmp_set_tap_count(1); 		// similar
    dmp_set_tap_time(100); // min time
    dmp_set_tap_time_multi(500); // max time

    dmp_set_shake_reject_thresh(GYRO_SF, 200);
    dmp_set_shake_reject_time(40);
    dmp_set_shake_reject_timeout(10);


// Ok, so I'll need to rewrite all of these functions in the i2cdev style - which is what?

The Invensense example code sets up the raw sensors, then sends all of the dmp settings to it, then finally enables it and enables interrupts
The i2cdev library just has a huge initialization function... that'll all be kind of a pain in the ass

To get it working quickly: 
att the top of inv_mpu_dmp_motion_driver.c there are defines for the i2c reads and writes and delays
Just adapt it to the i2cdev/wire lib'


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ok. next day. adding in #if defined __MK20DX256__ for teensy
Well. It seems that I've found a MPU6050_MotionDriver_6_0 and MPU6050_MotionDriver_6_1_2. This'll be interesting. 
The "driver" code is identical across all versions - that is all that the STM32 ARM example uses.
It seems like the precompiled libraries might have some really advanced features?
No, according to the porting library, the driver layer does the IMU interface, and the Motion Processing Libraries (MPL) 