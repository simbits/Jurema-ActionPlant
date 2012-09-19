#include <Arduino.h>
#include <CapSense.h>
#include <Wire.h>

#undef DBG
#undef DBG_CAPSENSE 
#undef TEST_MOTOR
#define ENABLE_PUMP

#define RELAY1_PIN					A0
#define RELAY2_PIN					A1
#define DIST1_PIN					A3
#define DIST2_PIN					A2	
#define DIST_FRONT_PIN				DIST2_PIN
#define DIST_BACK_PIN				DIST1_PIN

#define DIST_FRONT					0
#define DIST_BACK					1
#define DIST_FRONT_MASK				1
#define DIST_BACK_MASK				2
#define DIST_CAL_SAMPLES			30
#define DIST_DETECT_THRESHOLD		80

#define CAPSENSE_SENSORS			1
#define CAPSENSE_TOUCH_THRESHOLD	700
#define CAPSENSE_DRV_PIN			37
#define CAPSENSE_IN1_PIN			43
#define CAPSENSE_IN2_PIN			45
#define CAPSENSE_IN3_PIN			47
#define CAPSENSE_LED1_PIN			50
#define CAPSENSE_LED2_PIN			52
#define CAPSENSE_LED3_PIN			A15

#define LED_STAT1_PIN				A14
#define LED_STAT2_PIN				A13
#define LED_STAT3_PIN				A12
#define LED_STAT4_PIN				A11
#define LED_STAT5_PIN				A10

#define DAC_CMD_UPDATE				0x30
#define DAC_UD_ADDR					0x10
#define DAC_LR_ADDR 				0x11
#define DAC_REF_ADDR				0x12

#define DAC_UD_VHIGH				0xd0
#define DAC_UD_VMID					0xb0
#define DAC_UD_VLOW					0x90
#define DAC_LR_VHIGH				0xc5
#define DAC_LR_VMID					0xb0
#define DAC_LR_VLOW					0xa6
#define DAC_VREF					0xb0


typedef struct {
	uint8_t dac_vud;
	uint8_t dac_vlr;
	uint8_t dac_vref;
	uint16_t dist_cal_vals[2];
	uint8_t dist_an_pin[2];
	uint8_t dist_led_pin[2];
	CapSense *cs[CAPSENSE_SENSORS];
	uint8_t cs_led_pin[3];
	uint8_t funkyMove;
} juremaStc;

static juremaStc jurema;

static void updateDAC()
{
	Wire.beginTransmission(DAC_UD_ADDR); 
	Wire.write(DAC_CMD_UPDATE);       
	Wire.write(jurema.dac_vud);
	Wire.write((uint8_t)0x00);
	Wire.endTransmission();

	Wire.beginTransmission(DAC_LR_ADDR);
	Wire.write(DAC_CMD_UPDATE); 
	Wire.write(jurema.dac_vlr);
	Wire.write((uint8_t)0x00);
	Wire.endTransmission();

	Wire.beginTransmission(DAC_REF_ADDR);
	Wire.write(DAC_CMD_UPDATE);
	Wire.write(jurema.dac_vref);
	Wire.write((uint8_t)0x10);
	Wire.endTransmission();
}

static void resetDAC()
{
	jurema.dac_vud = DAC_UD_VMID;
	jurema.dac_vlr = DAC_LR_VMID;
	jurema.dac_vref = DAC_VREF;
	updateDAC();
}

static void calibrateDistanceSensors()
{
	int i, state, err = 0;
	uint16_t val_front = 0;
	uint16_t val_back = 0;

	jurema.dist_an_pin[DIST_FRONT] = DIST_FRONT_PIN;
	jurema.dist_an_pin[DIST_BACK] = DIST_BACK_PIN;
	jurema.dist_led_pin[DIST_FRONT] = LED_STAT1_PIN;
	jurema.dist_led_pin[DIST_BACK] = LED_STAT2_PIN;

	for (i=0; i<DIST_CAL_SAMPLES; i++) {
		val_front += analogRead(DIST_FRONT_PIN);
		val_back += analogRead(DIST_BACK_PIN);
	}

	val_front /= DIST_CAL_SAMPLES;
	val_back /= DIST_CAL_SAMPLES;

#if defined(DBG)	
	Serial.print("avg -> front: ");
	Serial.print(val_front);
	Serial.print(" - back: " );
	Serial.println(val_back);
#endif

	if (val_front > DIST_DETECT_THRESHOLD) {
		val_front = 0;
		err |= (DIST_FRONT_MASK);
	}

	if (val_back > DIST_DETECT_THRESHOLD) {
		val_back = 0;
		err |= (DIST_BACK_MASK);
	}

	jurema.dist_cal_vals[DIST_FRONT] = val_front;
	jurema.dist_cal_vals[DIST_BACK] = val_back;

	for (i=0,state=1; i<50; i++, state = !state) {
		if (err & DIST_FRONT_MASK) digitalWrite(LED_STAT1_PIN, state);
		if (err & DIST_BACK_MASK) digitalWrite(LED_STAT2_PIN, state);
		delay(50);
	}
}

static inline uint16_t distGetReading(int side)
{
	return analogRead(jurema.dist_an_pin[side]);
}

static inline uint8_t distDetectObject()
{
	uint8_t retmask = 0;
	long reading_front = distGetReading(DIST_FRONT);
	long reading_back = distGetReading(DIST_BACK);

#if defined(DBG)	
		Serial.print("front: ");
		Serial.print(reading_front);
		Serial.print(" - back: " );
		Serial.println(reading_back);
#endif

	if (reading_front > DIST_DETECT_THRESHOLD) {
		digitalWrite(jurema.dist_led_pin[DIST_FRONT], HIGH);
		retmask |= DIST_FRONT_MASK;
	} else {
		digitalWrite(jurema.dist_led_pin[DIST_FRONT], LOW);
	}

	if (reading_back > DIST_DETECT_THRESHOLD) {
		digitalWrite(jurema.dist_led_pin[DIST_BACK], HIGH);
		retmask |= DIST_BACK_MASK;
	} else {
		digitalWrite(jurema.dist_led_pin[DIST_BACK], LOW);
	}

	return retmask;
}

static int initCapSense()
{
	int i, state;

	for (i=0; i<CAPSENSE_SENSORS; i++) {
		jurema.cs[i] = new CapSense(CAPSENSE_DRV_PIN, CAPSENSE_IN1_PIN + i*2);
		jurema.cs[i]->set_CS_AutocaL_Millis(10000);
	}

	jurema.cs_led_pin[0] = CAPSENSE_LED1_PIN;
	jurema.cs_led_pin[1] = CAPSENSE_LED2_PIN;
	jurema.cs_led_pin[2] = CAPSENSE_LED3_PIN;
	
	for (i=0, state=1; i<4; i++,state=!state) {
		digitalWrite(CAPSENSE_LED1_PIN, state);
		digitalWrite(CAPSENSE_LED2_PIN, state);
		digitalWrite(CAPSENSE_LED3_PIN, state);
		delay(100);
	}

}

static inline long getCapSenseReading(int n, int samples)
{
	return jurema.cs[n]->capSense(samples);
}

static inline long isTouched(int n)
{
	long reading = getCapSenseReading(n, 30); 
#if defined(DBG_CAPSENSE)
	Serial.print(n);
	Serial.print(": ");
	Serial.println(reading);
#endif
	return (reading > CAPSENSE_TOUCH_THRESHOLD) ? reading : 0;
}

#if defined(ENABLE_PUMP)
static void waterPlant(int duration)
{
	digitalWrite(RELAY1_PIN, HIGH);
	delay(duration);
	digitalWrite(RELAY1_PIN, LOW);
}
#endif

static void funkyInitScene()
{
	int i;
	int state = 0;

	for (i=0; i<9; i++,state=!state) {
		digitalWrite(CAPSENSE_LED1_PIN, state);
		digitalWrite(CAPSENSE_LED2_PIN, state);
		digitalWrite(CAPSENSE_LED3_PIN, state);
		digitalWrite(LED_STAT1_PIN, state);
		digitalWrite(LED_STAT2_PIN, state);
		digitalWrite(LED_STAT3_PIN, state);
		digitalWrite(LED_STAT4_PIN, state);
		digitalWrite(LED_STAT5_PIN, state);
		delay(50);
	}

	digitalWrite(CAPSENSE_LED1_PIN, HIGH);
	delay(100);
	digitalWrite(CAPSENSE_LED2_PIN, HIGH);
	delay(100);
	digitalWrite(CAPSENSE_LED1_PIN, LOW);
	delay(100);
	digitalWrite(CAPSENSE_LED3_PIN, HIGH);
	delay(100);
	digitalWrite(CAPSENSE_LED2_PIN, LOW);
	delay(100);
	digitalWrite(LED_STAT1_PIN, HIGH);
	delay(100);
	digitalWrite(CAPSENSE_LED3_PIN, LOW);
	delay(100);
	digitalWrite(LED_STAT2_PIN, HIGH);
	delay(100);
	digitalWrite(LED_STAT1_PIN, LOW);
	delay(100);
	digitalWrite(LED_STAT3_PIN, HIGH);
	delay(100);
	digitalWrite(LED_STAT2_PIN, LOW);
	delay(100);
	digitalWrite(LED_STAT4_PIN, HIGH);
	delay(100);
	digitalWrite(LED_STAT3_PIN, LOW);
	delay(100);
	digitalWrite(LED_STAT5_PIN, HIGH);
	delay(100);
	digitalWrite(LED_STAT4_PIN, LOW);
	delay(100);
	digitalWrite(LED_STAT5_PIN, LOW);
}

void setup()
{
	Serial.begin(9600);

	Serial.println("Welcome to Jurema ActionPlant Development Edition");

	Wire.begin(); 
 	delay(100);

	Serial.println("Setting up DAC voltages");
	resetDAC();
  
	Serial.println("Seeding the random");
  	randomSeed(analogRead(A5));

	Serial.println("Initializing GPIO");
	pinMode(RELAY1_PIN, OUTPUT);
	pinMode(RELAY2_PIN, OUTPUT);
	pinMode(CAPSENSE_LED1_PIN, OUTPUT);
	pinMode(CAPSENSE_LED2_PIN, OUTPUT);
	pinMode(CAPSENSE_LED3_PIN, OUTPUT);
	pinMode(LED_STAT1_PIN, OUTPUT);
	pinMode(LED_STAT2_PIN, OUTPUT);
	pinMode(LED_STAT3_PIN, OUTPUT);
	pinMode(LED_STAT4_PIN, OUTPUT);
	pinMode(LED_STAT5_PIN, OUTPUT);

	Serial.println("Do some useless fancy blinking");
	funkyInitScene();

#if defined(ENABLE_PUMP)
	Serial.println("Water the plants");
	waterPlant(2000);
#endif

	Serial.println("Calibrate distance sensors");
	calibrateDistanceSensors();

	Serial.println("Initialize plant sensors");
	initCapSense();

	jurema.funkyMove = 0;

	Serial.println("");
	Serial.println("Ready for action");
}

#define MOVE_NONE		0
#define MOVE_FORWARD	1
#define MOVE_BACK		2
#define MOVE_LEFT		4
#define MOVE_RIGHT		8

static void _moveForward(int duration)
{
	uint8_t state = 1;

	if (distDetectObject() & DIST_FRONT_MASK) 
		return;

	jurema.dac_vud = DAC_UD_VHIGH;
	updateDAC();

	do {
		uint8_t detect_mask;
		delay(10);
		digitalWrite(LED_STAT4_PIN, state);

		detect_mask= distDetectObject();
		if (detect_mask & DIST_FRONT_MASK) {
			Serial.println("Object detected in the front! aborting move");
			break;
		}

		state = !state;
	} while (--duration);

	digitalWrite(LED_STAT4_PIN, LOW);
	resetDAC();
}

static void _moveBackward(int duration)
{
	uint8_t state = 1;

	if (distDetectObject() & DIST_BACK_MASK) 
		return;

	jurema.dac_vud = DAC_UD_VLOW;
	updateDAC();

	do {
		delay(10);
		digitalWrite(LED_STAT4_PIN, state);

		if (distDetectObject() & DIST_BACK_MASK) {
			Serial.println("Object detected in the back! aborting move");
			break;
		}

		state = !state;
	} while (--duration);

	digitalWrite(LED_STAT4_PIN, LOW);
	resetDAC();
}

static void _moveLeft(int duration)
{
	uint8_t state = 1;

	jurema.dac_vlr = DAC_LR_VLOW;
	updateDAC();

	do {
		delay(10);
		digitalWrite(LED_STAT5_PIN, state);
		state = !state;
/*
		if (distDetectObject()) {
			Serial.println("Object detected! aborting move");
			break;
		}
*/
	} while (--duration);

	digitalWrite(LED_STAT5_PIN, LOW);
	resetDAC();
}

static void _moveRight(int duration)
{
	uint8_t state = 1;
	jurema.dac_vlr = DAC_LR_VHIGH;
	updateDAC();

	do {
		delay(10);
		digitalWrite(LED_STAT5_PIN, state);
		state = !state;
/*
		if (distDetectObject()) {
			Serial.println("Object detected! aborting move");
			break;
		}
*/
	} while (--duration);

	digitalWrite(LED_STAT5_PIN, LOW);
	resetDAC();
}

static void doMove(uint8_t move, int duration)
{
	switch (move)
	{
		case MOVE_FORWARD:
			Serial.println("Move forward");
			_moveForward(duration);
			break;
		case MOVE_BACK:
			Serial.println("Move backward");
			_moveBackward(duration);
			break;
		case MOVE_LEFT:
			Serial.println("Move left");
			_moveLeft(duration);
			break;
		case MOVE_RIGHT:
			Serial.println("Move right");
			_moveRight(duration);
			break;
		default:
			break;
	}
}

static void funkyMove(uint8_t move)
{
	Serial.print("FunkyMove nr: ");
	Serial.println(move);

	switch (move) {
		case 0:
			doMove(MOVE_BACK, 175);
			doMove(MOVE_LEFT, 150);
			doMove(MOVE_BACK, 100);
			break;

		case 1:
			doMove(MOVE_RIGHT, 150);
			doMove(MOVE_BACK, 100);
			delay(1000);
			doMove(MOVE_RIGHT, 150);
			doMove(MOVE_FORWARD, 175);
			break;

		case 2:	
			doMove(MOVE_RIGHT, 100);
			delay(1000);
			doMove(MOVE_RIGHT, 150);
			doMove(MOVE_FORWARD, 175);
			doMove(MOVE_LEFT, 150);
			doMove(MOVE_BACK, 175);
			break;

		case 3:	
			doMove(MOVE_BACK, 175);
			doMove(MOVE_LEFT, 100);
			doMove(MOVE_BACK, 175);
			doMove(MOVE_LEFT, 100);
			doMove(MOVE_BACK, 100);
			doMove(MOVE_FORWARD, 175);
			break;
		
		case 4:
			doMove(MOVE_LEFT, 100);
			doMove(MOVE_BACK, 175);
			delay(1000);
			doMove(MOVE_RIGHT, 150);
			doMove(MOVE_BACK, 150);
			doMove(MOVE_LEFT, 150);
			doMove(MOVE_BACK, 100);
			break;
			
		case 5:
			doMove(MOVE_LEFT, 100);
			doMove(MOVE_BACK, 175);
			delay(1000);
			doMove(MOVE_RIGHT, 100);
			doMove(MOVE_BACK, 100);
			break;

		case 6:
			doMove(MOVE_RIGHT, 100);
			doMove(MOVE_BACK, 175);
			delay(1000);
			doMove(MOVE_RIGHT, 100);
			doMove(MOVE_BACK, 100);
			doMove(MOVE_FORWARD, 175);
			doMove(MOVE_RIGHT, 100);
			doMove(MOVE_BACK, 100);
			break;
	
		case 7:	
			doMove(MOVE_RIGHT, 100);
			doMove(MOVE_BACK, 100);
			doMove(MOVE_FORWARD, 175);
			doMove(MOVE_RIGHT, 100);
			doMove(MOVE_BACK, 100);
			break;

		default:
			break;
	}
}

static uint8_t selectMove()
{
	uint8_t detect_mask = distDetectObject();

	Serial.println("Selecting move");
	if ((detect_mask & DIST_FRONT_MASK) & (detect_mask & DIST_BACK_MASK)) {
		Serial.println("Objects detected on both sides");
		Serial.print("Action: ");
		doMove(MOVE_RIGHT, 175);
	} else if (detect_mask & DIST_FRONT_MASK) {
		Serial.println("Objects detected in front");
		Serial.print("Action: ");
		doMove(MOVE_BACK, 175);
		doMove(MOVE_RIGHT, 100);
	} else if (detect_mask & DIST_BACK_MASK) {
		Serial.println("Objects detected in the back");
		Serial.print("Action: ");
		doMove(MOVE_FORWARD, 175);
		doMove(MOVE_LEFT, 100);
		doMove(MOVE_FORWARD, 175);
	} else {
		//long r = random(7);
		Serial.println("No objects detected, free to move");
		Serial.print("Action: ");
		funkyMove(jurema.funkyMove++);
		if (jurema.funkyMove > 7)
			jurema.funkyMove = 0;
	}

	Serial.println("Move finished");
	resetDAC();
	delay(1000);
}

#if defined(TEST_MOTOR)
void testMotor()
{
	Serial.println("Test forward");
	doMove(MOVE_FORWARD, 20);
	delay(2000);
	Serial.println("Test backward");
	doMove(MOVE_BACK, 20);
	delay(2000);
	Serial.println("Test left");
	doMove(MOVE_LEFT, 20);
	delay(2000);
	Serial.println("Test right");
	doMove(MOVE_RIGHT, 20);
	delay(2000);
}
#endif

void loop()
{
	int i;

#if defined(TEST_MOTOR)
	testMotor();
#endif

	for (i=0; i<CAPSENSE_SENSORS; i++) {
		if (isTouched(i)) {
			Serial.println("Have touch");
			digitalWrite(jurema.cs_led_pin[i], HIGH);		
			selectMove();
			delay(2000);
			Serial.println("Recalibrate touch");
			jurema.cs[i]->reset_CS_AutoCal();
			Serial.println("Done");
			Serial.println("");
			break;
		} else {
			digitalWrite(jurema.cs_led_pin[i], LOW);		
		}
	}

	distDetectObject();
}
