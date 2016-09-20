
#ifndef SwingForgeMotors_h
#define SwingForgeMotors_h

#define PROC_PIN_1  31
#define PROC_PIN_2  26
#define PROC_PIN_9  A10
#define PROC_PIN_10 A11
#define PROC_PIN_11 A12
#define PROC_PIN_12 A13
#define PROC_PIN_18 A14
#define PROC_PIN_26 33
#define PROC_PIN_27 24
#define PROC_PIN_28 3
#define PROC_PIN_29 4
#define PROC_PIN_35 16
#define PROC_PIN_36 17
#define PROC_PIN_37 19
#define PROC_PIN_38 18
#define PROC_PIN_39 0
#define PROC_PIN_40 1
#define PROC_PIN_41 32
#define PROC_PIN_42 25
#define PROC_PIN_43 15
#define PROC_PIN_44 22
#define PROC_PIN_45 23
#define PROC_PIN_46 9
#define PROC_PIN_49 10
#define PROC_PIN_50 13
#define PROC_PIN_51 11
#define PROC_PIN_52 12
#define PROC_PIN_53 28
#define PROC_PIN_54 27
#define PROC_PIN_55 29
#define PROC_PIN_56 30
#define PROC_PIN_57 2
#define PROC_PIN_58 14
#define PROC_PIN_59 7
#define PROC_PIN_60 8
#define PROC_PIN_61 6
#define PROC_PIN_62 20
#define PROC_PIN_63 21
#define PROC_PIN_64 5


enum motor_position_e {
    MOTOR_POS_FRONT=0,
    MOTOR_POS_BACK_TOP,
    MOTOR_POS_BACK_BOTTOM,
    MOTOR_POS_SHOULDER_L,
    MOTOR_POS_SHOULDER_R,
    MOTOR_POS_SIDE_L,
    MOTOR_POS_SIDE_R
};


class sfMotor {
	public:
		sfMotor(int16_t pin, int16_t pos);
		void init();
		void disable();
		void enable(int16_t speed);
		void setSpeed(int16_t speed);
		int16_t getSpeed();
		int16_t getPin();
	    motor_position_e position;
	private:
	    bool _enabled = true;
	    int16_t _pin;
	    int16_t _speed = 0;
};

#endif