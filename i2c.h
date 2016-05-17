#include <Arduino.h>
#define I2C_PULLUPS_DISABLE PORTC &= ~(1<<4); PORTC &= ~(1<<5);
#define ACC_1G 265
#define MAG 1


static void Device_Mag_getADC();
static void Baro_init();
static void Mag_init();
static void ACC_init();


int16_t  i2c_errors_count = 0;
static uint8_t rawADC[6];
uint16_t calibratingG;
typedef struct {
	int16_t  accSmooth[3];
	int16_t  gyroData[3];
	int16_t  magADC[3];
	int16_t  gyroADC[3];
	int16_t  accADC[3];
} imu_t;
imu_t imu;
int16_t gyroZero[3] = { 0,0,0 };
uint8_t axis, tilt = 0;
uint16_t calibratingA = 0;
typedef struct {
	uint8_t currentSet;
	int16_t accZero[3];
	int16_t magZero[3];
	uint16_t flashsum;
	uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
} global_conf_t;
global_conf_t global_conf;
enum rc {
	ROLL,
	PITCH,
	YAW,
	THROTTLE,
	AUX1,
	AUX2,
	AUX3,
	AUX4,
	AUX5,
	AUX6,
	AUX7,
	AUX8
};

struct pid_ {
	uint8_t P8;
	uint8_t I8;
	uint8_t D8;
};
enum pid {
	PIDROLL,
	PIDPITCH,
	PIDYAW,
	PIDALT,
	PIDPOS,
	PIDPOSR,
	PIDNAVR,
	PIDLEVEL,
	PIDMAG,
	PIDVEL,     // not used currently
	PIDITEMS
};
typedef struct {
	pid_    pid[PIDITEMS];
	uint8_t rcRate8;
	uint8_t rcExpo8;
	uint8_t rollPitchRate;
	uint8_t yawRate;
	uint8_t dynThrPID;
	uint8_t thrMid8;
	uint8_t thrExpo8;
	int16_t angleTrim[2];
#if defined(EXTENDED_AUX_STATES)
	uint32_t activate[CHECKBOXITEMS];  //Extended aux states define six different aux state for each aux channel
#else
	uint16_t activate[5];
#endif 
	uint8_t powerTrigger1;
#if MAG
	int16_t mag_declination;
#endif
	
#if defined(GYRO_SMOOTHING)
	uint8_t Smoothing[3];
#endif
#if defined (FAILSAFE)
	int16_t failsafe_throttle;
#endif
#ifdef VBAT
	uint8_t vbatscale;
	uint8_t vbatlevel_warn1;
	uint8_t vbatlevel_warn2;
	uint8_t vbatlevel_crit;
#endif
#ifdef POWERMETER
	uint8_t pint2ma;
#endif
#ifdef POWERMETER_HARD
	uint16_t psensornull;
#endif
#ifdef MMGYRO
	uint8_t mmgyro;
#endif
#ifdef ARMEDTIMEWARNING
	uint16_t armedtimewarning;
#endif
	int16_t minthrottle;
#ifdef GOVERNOR_P
	int16_t governorP;
	int16_t governorD;
#endif
#ifdef YAW_COLL_PRECOMP
	uint8_t yawCollPrecomp;
	uint16_t yawCollPrecompDeadband;
#endif
	uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;
conf_t conf;
uint32_t currentTime = 0;
typedef struct {
	uint8_t OK_TO_ARM : 1;
	uint8_t ARMED : 1;
	uint8_t ACC_CALIBRATED : 1;
	uint8_t ANGLE_MODE : 1;
	uint8_t HORIZON_MODE : 1;
	uint8_t MAG_MODE : 1;
	uint8_t BARO_MODE : 1;
#ifdef HEADFREE
	uint8_t HEADFREE_MODE : 1;
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
	uint8_t PASSTHRU_MODE : 1;
#endif
	uint8_t SMALL_ANGLES_25 : 1;
#if MAG
	uint8_t CALIBRATE_MAG : 1;
#endif
#ifdef VARIOMETER
	uint8_t VARIO_MODE : 1;
#endif
	uint8_t GPS_mode : 2;               // 0-3 NONE,HOLD, HOME, NAV (see GPS_MODE_* defines
#if BARO || GPS
	uint8_t THROTTLE_IGNORED : 1;      // If it is 1 then ignore throttle stick movements in baro mode;
#endif
#if GPS
	uint8_t GPS_FIX : 1;
	uint8_t GPS_FIX_HOME : 1;
	uint8_t GPS_BARO_MODE : 1;         // This flag is used when GPS controls baro mode instead of user (it will replace rcOptions[BARO]
	uint8_t GPS_head_set : 1;           // it is 1 if the navigation engine got commands to control heading (SET_POI or SET_HEAD) CLEAR_HEAD will zero it
	uint8_t LAND_COMPLETED : 1;
	uint8_t LAND_IN_PROGRESS : 1;
#endif
} flags_struct_t;
flags_struct_t f;



// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************
void i2c_init(void) {
#if defined(INTERNAL_I2C_PULLUPS)
	I2C_PULLUPS_ENABLE
#else
	I2C_PULLUPS_DISABLE
#endif
		TWSR = 0;                                    // no prescaler => prescaler = 1
	TWBR = ((F_CPU / 400000) - 16) / 2;          // set the I2C clock rate to 400kHz, SCL frequency=400000, TWBR=12
	TWCR = 1 << TWEN;                              // enable twi module, no interrupt
	i2c_errors_count = 0;
}

void __attribute__((noinline)) waitTransmissionI2C(uint8_t twcr) {
	TWCR = twcr;
	uint8_t count = 255;
	while (!(TWCR & (1 << TWINT))) {
		count--;
		if (count == 0) {              //we are in a blocking state => we don't insist
			TWCR = 0;                  //and we force a reset on TWINT register
#if defined(WMP)
			neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
#endif
			i2c_errors_count++;
			break;
		}
	}
}

void i2c_rep_start(uint8_t address) {
	waitTransmissionI2C((1 << TWINT) | (1 << TWSTA) | (1 << TWEN)); // send REPEAT START condition and wait until transmission completed
	TWDR = address;                                           // send device address
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN));              // wail until transmission completed
}

void i2c_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	//  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data) {
	TWDR = data;                                 // send data to the previously addressed device
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN));
}

uint8_t i2c_readAck() {
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
	return TWDR;
}

uint8_t i2c_readNak() {
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN));
	uint8_t r = TWDR;
	i2c_stop();
	return r;
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size) {
	i2c_rep_start(add << 1); // I2C write direction
	i2c_write(reg);        // register selection
	i2c_rep_start((add << 1) | 1);  // I2C read direction
	uint8_t *b = buf;
	while (--size) *b++ = i2c_readAck(); // acknowledge all but the final byte
	*b = i2c_readNak();
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
	i2c_read_reg_to_buf(add, reg, rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
	i2c_rep_start(add << 1); // I2C write direction
	i2c_write(reg);        // register selection
	i2c_write(val);        // value to write in register
	i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
	uint8_t val;
	i2c_read_reg_to_buf(add, reg, &val, 1);
	return val;
}



// ****************
// GYRO common part
// ****************
void GYRO_Common() {
	static int16_t previousGyroADC[3] = { 0,0,0 };
	static int32_t g[3];
	uint8_t axis, tilt = 0;

	if (calibratingG>0) {
		for (axis = 0; axis < 3; axis++) {
			if (calibratingG == 512) { // Reset g[axis] at start of calibration
				g[axis] = 0;
			}
			g[axis] += imu.gyroADC[axis]; // Sum up 512 readings
			gyroZero[axis] = g[axis] >> 9;			
		}
		calibratingG--;
	}


	for (axis = 0; axis < 3; axis++) {
		imu.gyroADC[axis] -= gyroZero[axis];
		//anti gyro glitch, limit the variation between two consecutive readings
		imu.gyroADC[axis] = constrain(imu.gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
   
		previousGyroADC[axis] = imu.gyroADC[axis];
	}

	}


// ****************
// ACC common part
// ****************
void ACC_Common() {
	static int32_t a[3];
	if (calibratingA>0) {
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++) {
			if (calibratingA == 511) a[axis] = 0;   // Reset a[axis] at start of calibration
			a[axis] += imu.accADC[axis];           // Sum up 512 readings
			global_conf.accZero[axis] = a[axis] >> 9; // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingA == 0) {
			global_conf.accZero[YAW] -= ACC_1G;   // shift Z down by ACC_1G and store values in EEPROM at end of calibration
			conf.angleTrim[ROLL] = 0;
			conf.angleTrim[PITCH] = 0;
			
		}
	}

	imu.accADC[ROLL] -= global_conf.accZero[ROLL];
	imu.accADC[PITCH] -= global_conf.accZero[PITCH];
	imu.accADC[YAW] -= global_conf.accZero[YAW];

}

// ************************************************************************************************************
// I2C Accelerometer ADXL345 
// ************************************************************************************************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
// ************************************************************************************************************


#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}

#define ADXL345

#if defined(ADXL345)
#if !defined(ADXL345_ADDRESS) 
#define ADXL345_ADDRESS 0x53
//#define ADXL345_ADDRESS 0x53   //WARNING: Conflicts with a Wii Motion plus!
#endif

void ACC_init() {
	delay(10);
	i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1 << 3); //  register: Power CTRL  -- value: Set measure bit 3 on
	i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
	i2c_writeReg(ADXL345_ADDRESS, 0x2C, 0x09); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
}

void ACC_getADC() {
	i2c_getSixRawADC(ADXL345_ADDRESS, 0x32);

	ACC_ORIENTATION(((rawADC[1] << 8) | rawADC[0]),
		((rawADC[3] << 8) | rawADC[2]),
		((rawADC[5] << 8) | rawADC[4]));
	ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Gyroscope L3G4200D 
// ************************************************************************************************************
#define L3G4200D
#if defined(L3G4200D)
#define L3G4200D_ADDRESS 0x69
void Gyro_init() {
	delay(100);
	i2c_writeReg(L3G4200D_ADDRESS, 0x20, 0x8F); // CTRL_REG1   400Hz ODR, 20hz filter, run!
	delay(5);
	i2c_writeReg(L3G4200D_ADDRESS, 0x24, 0x02); // CTRL_REG5   low pass filter enable
	delay(5);
	i2c_writeReg(L3G4200D_ADDRESS, 0x23, 0x30); // CTRL_REG4 Select 2000dps
}

void Gyro_getADC() {
	i2c_getSixRawADC(L3G4200D_ADDRESS, 0x80 | 0x28);

	GYRO_ORIENTATION(((rawADC[1] << 8) | rawADC[0]) >> 2,
		((rawADC[3] << 8) | rawADC[2]) >> 2,
		((rawADC[5] << 8) | rawADC[4]) >> 2);
	GYRO_Common();
}
#endif

// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************

#if MAG
static float magGain[3] = { 1.0,1.0,1.0 };  // gain for each axis, populated at sensor init

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
	static uint32_t t, tCal = 0;
	static int16_t magZeroTempMin[3], magZeroTempMax[3];
	uint8_t axis;

	if (currentTime < t) return 0; //each read is spaced by 100ms
	t = currentTime + 100000;
	Device_Mag_getADC();

	for (axis = 0; axis<3; axis++) {
		imu.magADC[axis] = imu.magADC[axis] * magGain[axis];
		if (!f.CALIBRATE_MAG) imu.magADC[axis] -= global_conf.magZero[axis];
	}

	if (f.CALIBRATE_MAG) {
		if (tCal == 0) // init mag calibration
			tCal = t;
		if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
			
			for (axis = 0; axis<3; axis++) {
				if (tCal == t) { // it happens only in the first step, initialize the zero
					magZeroTempMin[axis] = imu.magADC[axis];
					magZeroTempMax[axis] = imu.magADC[axis];
				}
				if (imu.magADC[axis] < magZeroTempMin[axis]) { magZeroTempMin[axis] = imu.magADC[axis];  }
				if (imu.magADC[axis] > magZeroTempMax[axis]) { magZeroTempMax[axis] = imu.magADC[axis];  }
				global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
			}
		}
		else {
			f.CALIBRATE_MAG = 0;
			tCal = 0;
			
		}
	}

#if defined(SENSORS_TILT_45DEG_LEFT)
	int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL]) * 7) / 10;
	imu.magADC[ROLL] = ((imu.magADC[ROLL] + imu.magADC[PITCH]) * 7) / 10;
	imu.magADC[PITCH] = temp;
#endif
#if defined(SENSORS_TILT_45DEG_RIGHT)
	int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL]) * 7) / 10;
	imu.magADC[ROLL] = ((imu.magADC[ROLL] - imu.magADC[PITCH]) * 7) / 10;
	imu.magADC[PITCH] = temp;
#endif

	return 1;
}
#endif

#define HMC5883

#if defined(HMC5883)

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

static int32_t xyz_total[3] = { 0,0,0 };  // 32 bit totals so they won't overflow.

static void getADC() {
	i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
	MAG_ORIENTATION(((rawADC[0] << 8) | rawADC[1]),
		((rawADC[4] << 8) | rawADC[5]),
		((rawADC[2] << 8) | rawADC[3]));
}

static uint8_t bias_collect(uint8_t bias) {
	int16_t abs_magADC;

	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, bias);            // Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias
	for (uint8_t i = 0; i<10; i++) {                               // Collect 10 samples
		i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
		delay(100);
		getADC();                                                  // Get the raw values in case the scales have already been changed.
		for (uint8_t axis = 0; axis<3; axis++) {
			abs_magADC = abs(imu.magADC[axis]);
			xyz_total[axis] += abs_magADC;                            // Since the measurements are noisy, they should be averaged rather than taking the max.
			if ((int16_t)(1 << 12) < abs_magADC) return false;         // Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
		}
	}
	return true;
}

static void Mag_init() {
	bool bret = true;                // Error indicator

									 // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
									 // The new gain setting is effective from the second measurement and on.
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
	delay(100);
	getADC();  //Get one sample, and discard it

	if (!bias_collect(0x010 + HMC_POS_BIAS)) bret = false;
	if (!bias_collect(0x010 + HMC_NEG_BIAS)) bret = false;

	if (bret) // only if no saturation detected, compute the gain. otherwise, the default 1.0 is used
		for (uint8_t axis = 0; axis<3; axis++)
			magGain[axis] = 820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0 / xyz_total[axis];  // note: xyz_total[axis] is always positive

																						 // leave test mode
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x70); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 0x00); //Mode register             -- 000000 00    continuous Conversion Mode
	delay(100);
}

#if !defined(MPU6050_I2C_AUX_MASTER)
static void Device_Mag_getADC() {
	getADC();
}
#endif
#endif


void initS() {
	i2c_init();
  Gyro_init();
	  Mag_init();
	   ACC_init();
	
}

void initSensors() {
	uint8_t c = 5;

	while (c) { // We try several times to init all sensors without any i2c errors. An I2C error at this stage might results in a wrong sensor settings
		c--;
		initS();
		if (i2c_errors_count == 0) break; // no error during init => init ok
	}
}

double pangle=0, rangle=0;

double getpitchangle(int16_t ac_x,int16_t ac_z, int16_t gy_y){
	double deg1 = atan2(abs(ac_x), abs(ac_z)) * 180 / PI;
	/*double dgy_x = gy_y / 131;*/
	pangle = (0.95*(pangle + (gy_y*0.001))) + (0.05*deg1);
	return pangle;
}

double getrollangle(int16_t ac_y, int16_t ac_z, int16_t gy_x){
	double deg2 = atan2(abs(ac_y), abs(ac_z)) * 180 / PI;
	rangle = (0.95*(rangle + (gy_x*0.001))) + (0.05*deg2);
	return rangle;
}