﻿#include <Arduino.h>
#include "Adafruit_BMP085.h"
#define I2C_PULLUPS_DISABLE PORTC &= ~(1<<4); PORTC &= ~(1<<5);
#define ACC_1G 265
#define MAG 1
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)
//hello hi good night

static void Device_Mag_getADC();
static void Baro_init();
static void Mag_init();
static void ACC_init();

#if defined(Adafruit)
extern Adafruit_BMP085 bmp;
float altitudes[10];
float totalAltitude = 0.0;
#endif

int16_t  i2c_errors_count = 0;
static uint8_t rawADC[6];

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
uint16_t calibratingA = 512;
uint16_t calibratingG = 512;
uint16_t calibratingB = 200;
int16_t axisPID[3];
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
	uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;
conf_t conf;
uint16_t cycleTime = 0;

// ************************************************************************************************************
// PID value PID_CONTROLLER == 2
// ************************************************************************************************************
void LoadDefaults() {
	conf.pid[ROLL].P8 = 28; conf.pid[ROLL].I8 = 10; conf.pid[ROLL].D8 = 7;
	conf.pid[PITCH].P8 = 28; conf.pid[PITCH].I8 = 10; conf.pid[PITCH].D8 = 7;
	conf.pid[PIDLEVEL].P8 = 30; conf.pid[PIDLEVEL].I8 = 32; conf.pid[PIDLEVEL].D8 = 0;
	conf.pid[YAW].P8 = 68;  conf.pid[YAW].I8 = 45;  conf.pid[YAW].D8 = 0;
	conf.pid[PIDALT].P8 = 64; conf.pid[PIDALT].I8 = 25; conf.pid[PIDALT].D8 = 24;
#if defined (GPS)
	conf.pid[PIDPOS].P8 = POSHOLD_P * 100;     conf.pid[PIDPOS].I8 = POSHOLD_I * 100;       conf.pid[PIDPOS].D8 = 0;
	conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10; conf.pid[PIDPOSR].I8 = POSHOLD_RATE_I * 100;  conf.pid[PIDPOSR].D8 = POSHOLD_RATE_D * 1000;
	conf.pid[PIDNAVR].P8 = NAV_P * 10;          conf.pid[PIDNAVR].I8 = NAV_I * 100;           conf.pid[PIDNAVR].D8 = NAV_D * 1000;
#endif
	conf.pid[PIDMAG].P8 = 40;

	conf.pid[PIDVEL].P8 = 0;      conf.pid[PIDVEL].I8 = 0;    conf.pid[PIDVEL].D8 = 0;
}
uint32_t currentTime = 0;
uint16_t previousTime = 0;
typedef struct {
	uint8_t OK_TO_ARM : 1;
	uint8_t ARMED : 1;
	uint8_t ACC_CALIBRATED : 1;
	uint8_t ANGLE_MODE : 1;
	uint8_t HORIZON_MODE : 1;
	uint8_t MAG_MODE : 1;
	uint8_t BARO_MODE : 1;

	uint8_t SMALL_ANGLES_25 : 1;
#if MAG
	uint8_t CALIBRATE_MAG : 1;
#endif

	uint8_t GPS_mode : 2;               // 0-3 NONE,HOLD, HOME, NAV (see GPS_MODE_* defines


} flags_struct_t;
flags_struct_t f;

typedef struct {
	int32_t  EstAlt;             // in cm
	int16_t  vario;              // variometer in cm/s
} alt_t;

alt_t alt;
int32_t  AltHold; // in cm
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;

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
#if defined (Baro1)
  Baro_init();
#endif
	  Mag_init();
	   ACC_init();
	   
#if defined (Adafruit)
	   bmp.begin(BMP085_ULTRAHIGHRES);
#endif
}

#if defined (Adafruit)
float getalt()
{
	static int index=0;
	float alt;
	totalAltitude = totalAltitude - altitudes[index];

	altitudes[index] = bmp.readAltitude(101800);

	totalAltitude += altitudes[index];

	if (index >= 10)
		index = 0;
	alt = totalAltitude / 10;
	return alt;

}
#endif

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
	double deg1 = atan2(ac_x, ac_z) * 180 / PI;
	/*double dgy_x = gy_y / 131;*/
	pangle = (0.95*(pangle + (gy_y*0.001))) + (0.05*deg1);
	return pangle;
}

double getrollangle(int16_t ac_y, int16_t ac_z, int16_t gy_x){
	double deg2 = atan2(ac_y, ac_z) * 180 / PI;
	rangle = (0.95*(rangle + (gy_x*0.001))) + (0.05*deg2);
	return rangle;
}

float getheadingangle(int16_t xaxis, int16_t yaxis){
	float heading = atan2(yaxis, xaxis);
	// If you cannot find your Declination, comment out these two lines, your compass will be slightly off. 
	  float declinationAngle = 0.0404;
   heading += declinationAngle;


		   // Correct for when signs are reversed. 
		  if (heading < 0)
		     heading += 2 * PI;
	

		   // Check for wrap due to addition of declination. 
		   if (heading > 2 * PI)
		     heading -= 2 * PI;
		   float headingDegrees = heading * 180 / M_PI;
		   return headingDegrees;

}

typedef struct  {
	int32_t X, Y, Z;
} t_int32_t_vector_def;

typedef struct  {
	uint16_t XL; int16_t X;
	uint16_t YL; int16_t Y;
	uint16_t ZL; int16_t Z;
} t_int16_t_vector_def;

static int16_t accZ = 0;
typedef union {
	int32_t A32[3];
	t_int32_t_vector_def V32;
	int16_t A16[6];
	t_int16_t_vector_def V16;
} t_int32_t_vector;

#define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f)) // 70 milli deg/s /digit => 1 deg/s = 1000/70 LSB
#define ACC_LPF_FACTOR 4 // that means a LPF of 16
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
/* Set the Gyro Weight for Gyro/Acc complementary filter
Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
#define GYR_CMPF_FACTOR 10 //  that means a CMP_FACTOR of 1024 (2^10)
#endif

typedef struct {
	int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
	int16_t heading;             // variometer in cm/s
} att_t;

att_t att;

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 8 // that means a CMP_FACTOR of 256 (2^8)

// signed16 * signed16
// 22 cycles
// http://mekonik.wordpress.com/2009/03/18/arduino-avr-gcc-multiplication/
#define MultiS16X16to32(longRes, intIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %A2 \n\t" \
"movw %A0, r0 \n\t" \
"muls %B1, %B2 \n\t" \
"movw %C0, r0 \n\t" \
"mulsu %B2, %A1 \n\t" \
"sbc %D0, r26 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r26 \n\t" \
"mulsu %B1, %A2 \n\t" \
"sbc %D0, r26 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (longRes) \
: \
"a" (intIn1), \
"a" (intIn2) \
: \
"r26" \
)
int32_t  __attribute__((noinline)) mul(int16_t a, int16_t b) {
	int32_t r;
	MultiS16X16to32(r, a, b);
	//r = (int32_t)a*b; without asm requirement
	return r;
}

//return angle , unit: 1/10 degree
int16_t _atan2(int32_t y, int32_t x){
	float z = y;
	int16_t a;
	uint8_t c;
	c = abs(y) < abs(x);
	if (c) { z = z / x; }
	else { z = x / z; }
	a = 2046.43 * (z / (3.5714 + z * z));
	if (c){
		if (x<0) {
			if (y<0) a -= 1800;
			else a += 1800;
		}
	}
	else {
		a = 900 - a;
		if (y<0) a -= 1800;
	}
	return a;
}

float InvSqrt(float x){
	union{
		int32_t i;
		float   f;
	} conv;
	conv.f = x;
	conv.i = 0x5f1ffff9 - (conv.i >> 1);
	return conv.f * (1.68191409f - 0.703952253f * x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV32(t_int32_t_vector *v, int16_t* delta) {
	int16_t X = v->V16.X;
	int16_t Y = v->V16.Y;
	int16_t Z = v->V16.Z;

	v->V32.Z -= mul(delta[ROLL], X) + mul(delta[PITCH], Y);
	v->V32.X += mul(delta[ROLL], Z) - mul(delta[YAW], Y);
	v->V32.Y += mul(delta[PITCH], Z) + mul(delta[YAW], X);
}


void getEstimatedAttitude(){
	uint8_t axis;
	int32_t accMag = 0;
	float scale;
	int16_t deltaGyroAngle16[3];
	static t_int32_t_vector EstG = { 0, 0, (int32_t)ACC_1G << 16 };
#if MAG
	static t_int32_t_vector EstM;
#endif
	static uint32_t LPFAcc[3];
	float invG; // 1/|G|
	static int16_t accZoffset = 0;
	int32_t accZ_tmp = 0;
	static uint16_t previousT;
	uint16_t currentT = micros();

	// unit: radian per bit, scaled by 2^16 for further multiplication
	// with a delta time of 3000 us, and GYRO scale of most gyros, scale = a little bit less than 1
	scale = (currentT - previousT) * (GYRO_SCALE * 65536);
	previousT = currentT;

	// Initialization
	for (axis = 0; axis < 3; axis++) {
		// valid as long as LPF_FACTOR is less than 15
		imu.accSmooth[axis] = LPFAcc[axis] >> ACC_LPF_FACTOR;
		LPFAcc[axis] += imu.accADC[axis] - imu.accSmooth[axis];
		// used to calculate later the magnitude of acc vector
		accMag += mul(imu.accSmooth[axis], imu.accSmooth[axis]);
		// unit: radian scaled by 2^16
		// imu.gyroADC[axis] is 14 bit long, the scale factor ensure deltaGyroAngle16[axis] is still 14 bit long
		deltaGyroAngle16[axis] = imu.gyroADC[axis] * scale;
	}

	// we rotate the intermediate 32 bit vector with the radian vector (deltaGyroAngle16), scaled by 2^16
	// however, only the first 16 MSB of the 32 bit vector is used to compute the result
	// it is ok to use this approximation as the 16 LSB are used only for the complementary filter part
	rotateV32(&EstG, deltaGyroAngle16);
	rotateV32(&EstM, deltaGyroAngle16);

	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation
	// To do that, we just skip filter, as EstV already rotated by Gyro
	for (axis = 0; axis < 3; axis++) {
		if ((int16_t)(0.85*ACC_1G*ACC_1G / 256) < (int16_t)(accMag >> 8) && (int16_t)(accMag >> 8) < (int16_t)(1.15*ACC_1G*ACC_1G / 256))
			EstG.A32[axis] += (int32_t)(imu.accSmooth[axis] - EstG.A16[2 * axis + 1]) << (16 - GYR_CMPF_FACTOR);
		accZ_tmp += mul(imu.accSmooth[axis], EstG.A16[2 * axis + 1]);
#if MAG
		EstM.A32[axis] += (int32_t)(imu.magADC[axis] - EstM.A16[2 * axis + 1]) << (16 - GYR_CMPFM_FACTOR);
#endif
	}

	if (EstG.V16.Z > ACCZ_25deg)
		f.SMALL_ANGLES_25 = 1;
	else
		f.SMALL_ANGLES_25 = 0;

	// Attitude of the estimated vector
	int32_t sqGX_sqGZ = mul(EstG.V16.X, EstG.V16.X) + mul(EstG.V16.Z, EstG.V16.Z);
	invG = InvSqrt(sqGX_sqGZ + mul(EstG.V16.Y, EstG.V16.Y));
	att.angle[ROLL] = _atan2(EstG.V16.X, EstG.V16.Z);
	att.angle[PITCH] = _atan2(EstG.V16.Y, InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);

	//note on the second term: mathematically there is a risk of overflow (16*16*16=48 bits). assumed to be null with real values
	att.heading = _atan2(
		mul(EstM.V16.Z, EstG.V16.X) - mul(EstM.V16.X, EstG.V16.Z),
		(EstM.V16.Y * sqGX_sqGZ - (mul(EstM.V16.X, EstG.V16.X) + mul(EstM.V16.Z, EstG.V16.Z)) * EstG.V16.Y)*invG);
#if MAG
	att.heading += conf.mag_declination; // Set from GUI
#endif
	att.heading /= 10;

#if defined(THROTTLE_ANGLE_CORRECTION)
	cosZ = mul(EstG.V16.Z, 100) / ACC_1G;                                                   // cos(angleZ) * 100 
	throttleAngleCorrection = THROTTLE_ANGLE_CORRECTION * constrain(100 - cosZ, 0, 100) >> 3;  // 16 bit ok: 200*150 = 30000  
#endif

	// projection of ACC vector to global Z, with 1G subtructed
	// Math: accZ = A * G / |G| - 1G
	accZ = accZ_tmp *  invG;
	if (!f.ARMED) {
		accZoffset -= accZoffset >> 3;
		accZoffset += accZ;
	}
	accZ -= accZoffset >> 3;
}

#if defined (Baro1)
// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

int32_t baroPressure;
int16_t baroTemperature;
int32_t baroPressureSum;

#define BMP085_ADDRESS 0x77

static struct {
	// sensor registers from the BOSCH BMP085 datasheet
	int16_t  ac1, ac2, ac3;
	uint16_t ac4, ac5, ac6;
	int16_t  b1, b2, mb, mc, md;
	union { uint16_t val; uint8_t raw[2]; } ut; //uncompensated T
	union { uint32_t val; uint8_t raw[4]; } up; //uncompensated P
	uint8_t  state;
	uint32_t deadline;
} bmp085_ctx;
#define OSS 3

/* transform a series of bytes from big endian to little
endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
	/* we swap in-place, so we only have to
	* place _one_ element on a temporary tray
	*/
	uint8_t tray;
	uint8_t *from;
	uint8_t *to;
	/* keep swapping until the pointers have assed each other */
	for (from = (uint8_t*)buf, to = &from[size - 1]; from < to; from++, to--) {
		tray = *from;
		*from = *to;
		*to = tray;
	}
}



void i2c_BMP085_readCalibration(){
	delay(10);
	//read calibration data in one go
	size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
	i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, (uint8_t*)&bmp085_ctx.ac1, s_bytes);
	// now fix endianness
	int16_t *p;
	for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
		swap_endianness(p, sizeof(*p));
	}
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start(void) {
	i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x2e);
	i2c_rep_start(BMP085_ADDRESS << 1);
	i2c_write(0xF6);
	i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start() {
	i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x34 + (OSS << 6)); // control register value for oversampling setting 3
	i2c_rep_start(BMP085_ADDRESS << 1); //I2C write direction => 0
	i2c_write(0xF6);
	i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read() {
	i2c_rep_start((BMP085_ADDRESS << 1) | 1);//I2C read direction => 1
	bmp085_ctx.up.raw[2] = i2c_readAck();
	bmp085_ctx.up.raw[1] = i2c_readAck();
	bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
	i2c_rep_start((BMP085_ADDRESS << 1) | 1);//I2C read direction => 1
	bmp085_ctx.ut.raw[1] = i2c_readAck();
	bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate() {
	int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
	uint32_t b4, b7;
	// Temperature calculations
	x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
	x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
	b5 = x1 + x2;
	baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
	// Pressure calculations
	b6 = b5 - 4000;
	x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11;
	x2 = bmp085_ctx.ac2 * b6 >> 11;
	x3 = x1 + x2;
	tmp = bmp085_ctx.ac1;
	tmp = (tmp * 4 + x3) << OSS;
	b3 = (tmp + 2) / 4;
	x1 = bmp085_ctx.ac3 * b6 >> 13;
	x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = ((uint32_t)(bmp085_ctx.up.val >> (8 - OSS)) - b3) * (50000 >> OSS);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	baroPressure = p + ((x1 + x2 + 3791) >> 4);
}

void  Baro_init() {
	delay(10);
	i2c_BMP085_readCalibration();
	delay(5);
	i2c_BMP085_UT_Start();
	bmp085_ctx.deadline = currentTime + 5000;
}


#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define BARO_TAB_SIZE   21

#define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now

static void Baro_Common() {
	static int32_t baroHistTab[BARO_TAB_SIZE];
	static uint8_t baroHistIdx;

	uint8_t indexplus1 = (baroHistIdx + 1);
	if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
	baroHistTab[baroHistIdx] = baroPressure;
	baroPressureSum += baroHistTab[baroHistIdx];
	baroPressureSum -= baroHistTab[indexplus1];
	baroHistIdx = indexplus1;
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                   // first UT conversion is started in init procedure
	if (currentTime < bmp085_ctx.deadline) return 0;
	bmp085_ctx.deadline = currentTime + 6000; // 1.5ms margin according to the spec (4.5ms T convetion time)
	if (bmp085_ctx.state == 0) {
		i2c_BMP085_UT_Read();
		i2c_BMP085_UP_Start();
		bmp085_ctx.state = 1;
		Baro_Common();
		bmp085_ctx.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
		return 1;
	}
	else {
		i2c_BMP085_UP_Read();
		i2c_BMP085_UT_Start();
		i2c_BMP085_Calculate();
		bmp085_ctx.state = 0;
		return 2;
	}
}


#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
    } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }


uint8_t getEstimatedAltitude(){
	int32_t  BaroAlt;
	static float baroGroundTemperatureScale, logBaroGroundPressureSum;
	static float vel = 0.0f;
	static uint16_t previousT;
	uint16_t currentT = micros();
	uint16_t dTime;

	dTime = currentT - previousT;
	if (dTime < UPDATE_INTERVAL) return 0;
	previousT = currentT;

	if (calibratingB > 0) {
		logBaroGroundPressureSum = log(baroPressureSum);
		baroGroundTemperatureScale = ((int32_t)baroTemperature + 27315) * (2 * 29.271267f); // 2 *  is included here => no need for * 2  on BaroAlt in additional LPF
		calibratingB--;
	}

	// baroGroundPressureSum is not supposed to be 0 here
	// see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
	BaroAlt = (logBaroGroundPressureSum - log(baroPressureSum)) * baroGroundTemperatureScale;

	alt.EstAlt = (alt.EstAlt * 6 + BaroAlt) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
#if (defined(VARIOMETER) && (VARIOMETER != 2)) || !defined(SUPPRESS_BARO_ALTHOLD)
	//P
	int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300);
	applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
	BaroPID = constrain((conf.pid[PIDALT].P8 * error16 >> 7), -150, +150);

	//I
	errorAltitudeI += conf.pid[PIDALT].I8 * error16 >> 6;
	errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
	BaroPID += errorAltitudeI >> 9; //I in range +/-60

	applyDeadband(accZ, ACC_Z_DEADBAND);

	static int32_t lastBaroAlt;
	// could only overflow with a difference of 320m, which is highly improbable here
	int16_t baroVel = mul((alt.EstAlt - lastBaroAlt), (1000000 / UPDATE_INTERVAL));

	lastBaroAlt = alt.EstAlt;

	baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
	applyDeadband(baroVel, 10); // to reduce noise near zero

	// Integrator - velocity, cm/sec
	vel += accZ * ACC_VelScale * dTime;

	// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
	// By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
	vel = vel * 0.985f + baroVel * 0.015f;

	//D
	alt.vario = vel;
	applyDeadband(alt.vario, 5);
	BaroPID -= constrain(conf.pid[PIDALT].D8 * alt.vario >> 4, -150, 150);
#endif
	return 1;
}

#endif


void computeIMU() {
	uint8_t axis;
	static int16_t gyroADCprevious[3] = { 0, 0, 0 };
	static int16_t gyroADCinter[3];

	uint16_t timeInterleave = 0;

	for (axis = 0; axis < 3; axis++)
		gyroADCinter[axis] = imu.gyroADC[axis];
	timeInterleave = micros();
	uint8_t t = 0;
	while ((int16_t)(micros() - timeInterleave)<650) t = 1; //empirical, interleaving delay between 2 consecutive reads

	Gyro_getADC();

	for (axis = 0; axis < 3; axis++) {
		gyroADCinter[axis] = imu.gyroADC[axis] + gyroADCinter[axis];
		// empirical, we take a weighted value of the current and the previous values
		imu.gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
		gyroADCprevious[axis] = gyroADCinter[axis] >> 1;
	}
#if defined(GYRO_SMOOTHING)
	static int16_t gyroSmooth[3] = { 0, 0, 0 };
	for (axis = 0; axis < 3; axis++) {
		imu.gyroData[axis] = (int16_t)(((int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis] - 1)) + imu.gyroData[axis] + 1) / conf.Smoothing[axis]);
		gyroSmooth[axis] = imu.gyroData[axis];
	}
#elif defined(TRI)
	static int16_t gyroYawSmooth = 0;
	imu.gyroData[YAW] = (gyroYawSmooth * 2 + imu.gyroData[YAW]) / 3;
	gyroYawSmooth = imu.gyroData[YAW];
#endif
}

// ************************************************************************************************************
// **** PITCH & ROLL & YAW PID ****
// ************************************************************************************************************

void getPID(){

#define GYRO_I_MAX 256
#define ACC_I_MAX 256
	int16_t error, errorAngle;
	int16_t AngleRateTmp, RateError;
	int16_t PTerm = 0, ITerm = 0, DTerm, PTermACC, ITermACC;
	static int32_t errorGyroI[3] = { 0, 0, 0 };
	static int16_t lastError[3] = { 0, 0, 0 };
	static int16_t delta1[3], delta2[3];
	int16_t delta;
	int16_t deltaSum;
	//int32_t prop = 0;
	//prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]

	//----------PID controller----------
	for (axis = 0; axis < 3; axis++) {
		//-----Get the desired angle rate depending on flight mode
		if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2) { // MODE relying on ACC
			// calculate error and limit the angle to 50 degrees max inclination
			errorAngle = att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
		}
		if (axis == 2) {//YAW is always gyro-controlled (MAG correction is applied to rcCommand)
			AngleRateTmp = (((int32_t)(conf.yawRate + 27)) >> 5);
		}
		else {
				//it's the ANGLE mode - control is angle based, so control loop is needed
				AngleRateTmp = ((int32_t)errorAngle * conf.pid[PIDLEVEL].P8) >> 4;
			 }

		//--------low-level gyro-based PID. ----------
		//Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
		//-----calculate scaled error.AngleRates
		//multiplication of rcCommand corresponds to changing the sticks scaling here
		RateError = AngleRateTmp - imu.gyroData[axis];

		//-----calculate P component
		PTerm = ((int32_t)RateError * conf.pid[axis].P8) >> 7;

		//-----calculate I component
		//there should be no division before accumulating the error to integrator, because the precision would be reduced.
		//Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
		//Time correction (to avoid different I scaling for different builds based on average cycle time)
		//is normalized to cycle time = 2048.
		errorGyroI[axis] += (((int32_t)RateError * cycleTime) >> 11) * conf.pid[axis].I8;
		//limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
		//I coefficient (I8) moved before integration to make limiting independent from PID settings
		errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t)-GYRO_I_MAX << 13, (int32_t)+GYRO_I_MAX << 13);
		ITerm = errorGyroI[axis] >> 13;

		//-----calculate D-term
		delta = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
		lastError[axis] = RateError;

		//Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
		// would be scaled by different dt each time. Division by dT fixes that.
		delta = ((int32_t)delta * ((uint16_t)0xFFFF / (cycleTime >> 4))) >> 6;
		//add moving average here to reduce noise
		deltaSum = delta1[axis] + delta2[axis] + delta;
		delta2[axis] = delta1[axis];
		delta1[axis] = delta;

		//DTerm = (deltaSum*conf.pid[axis].D8)>>8;
		//Solve overflow in calculation above...
		DTerm = ((int32_t)deltaSum*conf.pid[axis].D8) >> 8;
		//-----calculate total PID output
		axisPID[axis] = PTerm + ITerm + DTerm;
	}

}

void getPID2() {

	int16_t error, error_previous;
	int16_t AngleRateTmp, RateError;
	int16_t PTerm = 0, ITerm = 0, DTerm, PTermACC, ITermACC;
	static int32_t errorGyroI[3] = { 0, 0, 0 };
	static int16_t lastError[3] = { 0, 0, 0 };
	static int16_t delta1[3], delta2[3];
	int16_t delta;
	int16_t deltaSum;
	int16_t Kp, Ki, Kd;

	for (axis = 0; axis < 3; axis++) {
		error = att.angle[axis] + conf.angleTrim[axis];

		PTerm = Kp * error;
		ITerm += Ki * error * cycleTime;
		DTerm = Kd * (error - error_previous) / cycleTime;

		axisPID[axis] = PTerm + ITerm + DTerm;
		axisPID[axis] = constrain(axisPID[axis], -255, 255);
		
		error_previous = error;
	}
}
