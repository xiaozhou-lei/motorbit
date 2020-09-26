/*
ken@emakefun.com
modified from pxt-servo/servodriver.ts
load dependency
"motorbit": "file:../pxt-motorbit"
*/

enum RgbColors {
    //% block=red
    Red = 1,
    //% block=orange
    Orange = 2,
    //% block=yellow
    Yellow = 3,
    //% block=green
    Green = 4,
    //% block=blue
    Blue = 5,
    //% block=indigo
    Indigo = 6,
    //% block=violet
    Violet = 7,
    //% block=purple
    Purple = 8,
    //% block=white
    White = 9
}

enum RgbUltrasonics {
//% block=left
Left = 0x00,
//% block=right
Right = 0x01,
//% block=all
All = 0x02
}

enum ColorEffect {
//% block=none
None = 0x00,
//% block=breathing
Breathing = 0x01,
//% block=rotate
Rotate = 0x02,
//% block=flash
Flash = 0x03
}

enum rotation_direction {
    //% block="none"
    none = 0,
    //% block="clockwise"
    clockwise = 1,
    //% block="counter-clockwise"
    counterclockwise = 2,
    //% block="180-degree"
    one_eighty_degree = 3,
}

//% color="#EE6A50" weight=10 icon="\uf085"
namespace motorbit {
	const PCA9685_ADDRESS = 0x40
	const MODE1 = 0x00
	const MODE2 = 0x01
	const SUBADR1 = 0x02
	const SUBADR2 = 0x03
	const SUBADR3 = 0x04
	const PRESCALE = 0xFE
	const LED0_ON_L = 0x06
	const LED0_ON_H = 0x07
	const LED0_OFF_L = 0x08
	const LED0_OFF_H = 0x09
	const ALL_LED_ON_L = 0xFA
	const ALL_LED_ON_H = 0xFB
	const ALL_LED_OFF_L = 0xFC
	const ALL_LED_OFF_H = 0xFD

	const STP_CHA_L = 2047
	const STP_CHA_H = 4095

	const STP_CHB_L = 1
	const STP_CHB_H = 2047

	const STP_CHC_L = 1023
	const STP_CHC_H = 3071

	const STP_CHD_L = 3071
	const STP_CHD_H = 1023

	const _NOOP = 0 // no-op (do nothing, doesn't change current status)
	const _DIGIT = [1, 2, 3, 4, 5, 6, 7, 8] // digit (LED column)
	const _DECODEMODE = 9 // decode mode (1=on, 0-off; for 7-segment display on MAX7219, no usage here)
	const _INTENSITY = 10 // intensity (LED brightness level, 0-15)
	const _SCANLIMIT = 11 // scan limit (number of scanned digits)
	const _SHUTDOWN = 12 // turn on (1) or off (0)
	const _DISPLAYTEST = 15 // force all LEDs light up, no usage here

	let _pinCS = DigitalPin.P16 // LOAD pin, 0=ready to receive command, 1=command take effect
	let _matrixNum = 1 // number of MAX7219 matrix linked in the chain
	let _displayArray: number[] = [] // display array to show accross all matrixs
	let _rotation = 0 // rotate matrixs display for 4-in-1 modules
	let _reversed = false // reverse matrixs display order for 4-in-1 modules

	export enum Servos {
		S1 = 0x01,
		S2 = 0x02,
		S3 = 0x03,
		S4 = 0x04,
		S5 = 0x05,
		S6 = 0x06,
		S7 = 0x07,
		S8 = 0x08
	}

	export enum Motors {
		A01A02 = 0x3,
		B01B02 = 0x4,
		A03A04 = 0x1,
		B03B04 = 0x2
	}

	export enum Steppers {
		STPM1_2 = 0x2,
		STPM3_4 = 0x1
	}

	export enum SonarVersion {
		V1 = 0x1,
		V2 = 0x2
	}

	export enum Turns {
		//% blockId="T1B4" block="1/4"
		T1B4 = 90,
		//% blockId="T1B2" block="1/2"
		T1B2 = 180,
		//% blockId="T1B0" block="1"
		T1B0 = 360,
		//% blockId="T2B0" block="2"
		T2B0 = 720,
		//% blockId="T3B0" block="3"
		T3B0 = 1080,
		//% blockId="T4B0" block="4"
		T4B0 = 1440,
		//% blockId="T5B0" block="5"
		T5B0 = 1800
	}

	let initialized = false
//	let neoStrip: neopixel.Strip;
	let emRGBLight: EMRGBLight.EmakefunRGBLight;
	
	
	let matBuf = pins.createBuffer(17);
	let distanceBuf = 0;

	function i2cwrite(addr: number, reg: number, value: number) {
		let buf = pins.createBuffer(2)
		buf[0] = reg
		buf[1] = value
		pins.i2cWriteBuffer(addr, buf)
	}

	function i2ccmd(addr: number, value: number) {
		let buf = pins.createBuffer(1)
		buf[0] = value
		pins.i2cWriteBuffer(addr, buf)
	}

	function i2cread(addr: number, reg: number) {
		pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
		let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
		return val;
	}

	function initPCA9685(): void {
		i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
		setFreq(50);
		for (let idx = 0; idx < 16; idx++) {
			setPwm(idx, 0, 0);
		}
		initialized = true
	}

	function setFreq(freq: number): void {
		// Constrain the frequency
		let prescaleval = 25000000;
		prescaleval /= 4096;
		prescaleval /= freq;
		prescaleval -= 1;
		let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
		let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
		let newmode = (oldmode & 0x7F) | 0x10; // sleep
		i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
		i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
		i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
		control.waitMicros(5000);
		i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
	}

	function setPwm(channel: number, on: number, off: number): void {
		if (channel < 0 || channel > 15)
			return;
		//serial.writeValue("ch", channel)
		//serial.writeValue("on", on)
		//serial.writeValue("off", off)

		let buf = pins.createBuffer(5);
		buf[0] = LED0_ON_L + 4 * channel;
		buf[1] = on & 0xff;
		buf[2] = (on >> 8) & 0xff;
		buf[3] = off & 0xff;
		buf[4] = (off >> 8) & 0xff;
		pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
	}


	function setStepper(index: number, dir: boolean): void {
		if (index == 1) {
			if (dir) {
				setPwm(0, STP_CHA_L, STP_CHA_H);
				setPwm(2, STP_CHB_L, STP_CHB_H);
				setPwm(1, STP_CHC_L, STP_CHC_H);
				setPwm(3, STP_CHD_L, STP_CHD_H);
			} else {
				setPwm(3, STP_CHA_L, STP_CHA_H);
				setPwm(1, STP_CHB_L, STP_CHB_H);
				setPwm(2, STP_CHC_L, STP_CHC_H);
				setPwm(0, STP_CHD_L, STP_CHD_H);
			}
		} else {
			if (dir) {
				setPwm(4, STP_CHA_L, STP_CHA_H);
				setPwm(6, STP_CHB_L, STP_CHB_H);
				setPwm(5, STP_CHC_L, STP_CHC_H);
				setPwm(7, STP_CHD_L, STP_CHD_H);
			} else {
				setPwm(7, STP_CHA_L, STP_CHA_H);
				setPwm(5, STP_CHB_L, STP_CHB_H);
				setPwm(6, STP_CHC_L, STP_CHC_H);
				setPwm(4, STP_CHD_L, STP_CHD_H);
			}
		}
	}

	function stopMotor(index: number) {
		setPwm((index - 1) * 2, 0, 0);
		setPwm((index - 1) * 2 + 1, 0, 0);
	}

	/**
	 * Servo Execute
	 * @param index Servo Channel; eg: S1
	 * @param degree [0-180] degree of servo; eg: 0, 90, 180
	*/
	//% blockId=motorbit_servo block="Servo|%index|degree %degree=protractorPicker"
	//% weight=100
	//% degree.defl=90
	//% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
	export function Servo(index: Servos, degree: number): void {
		if (!initialized) {
			initPCA9685()
		}
		// 50hz: 20,000 us
		let v_us = (degree * 1800 / 180 + 600) // 0.6 ~ 2.4
		let value = v_us * 4096 / 20000
		setPwm(index + 7, 0, value)
	}

	/**
	 * Geek Servo
	 * @param index Servo Channel; eg: S1
	 * @param degree [-45-225] degree of servo; eg: -45, 90, 225
	*/
	//% blockId=motorbit_gservo block="Geek Servo|%index|degree %degree=protractorPicker"
	//% weight=98
	//% blockGap=50
	//% degree.defl=90
	//% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
	export function GeekServo(index: Servos, degree: number): void {
		if (!initialized) {
			initPCA9685()
		}
		// 50hz: 20,000 us
		let v_us = ((degree - 90) * 20 / 3 + 1500) // 0.6 ~ 2.4
		let value = v_us * 4096 / 20000
		setPwm(index + 7, 0, value)
	}

	/**
	 * Servo Execute
	 * @param index Servo Channel; eg: S1
	 * @param degree1 [0-180] degree of servo; eg: 0, 90, 180
	 * @param degree2 [0-180] degree of servo; eg: 0, 90, 180
	 * @param speed [1-10] speed of servo; eg: 1, 10
	*/
	//% blockId=motorbit_servospeed block="Servo|%index|degree start %degree1|end %degree2|speed %speed"
	//% weight=96
	//% degree1.min=0 degree1.max=180
	//% degree2.min=0 degree2.max=180
	//% speed.min=1 speed.max=10
	//% inlineInputMode=inline
	//% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
	export function Servospeed(index: Servos, degree1: number, degree2: number, speed: number): void {
		if (!initialized) {
			initPCA9685()
		}
		// 50hz: 20,000 us
		if(degree1 > degree2){
			for(let i=degree1;i>degree2;i--){
				let v_us = (i * 1800 / 180 + 600) // 0.6 ~ 2.4
				let value = v_us * 4096 / 20000
				basic.pause(4 * (10 - speed));
				setPwm(index + 7, 0, value)
			}
		}
		else{
			for(let i=degree1;i<degree2;i++){
				let v_us = (i * 1800 / 180 + 600) // 0.6 ~ 2.4
				let value = v_us * 4096 / 20000
				basic.pause(4 * (10 - speed));
				setPwm(index + 7, 0, value)
			}
		}
	}


	//% blockId=motorbit_stepper_degree block="Stepper 28BYJ-48|%index|degree %degree"
	//% weight=91
	export function StepperDegree(index: Steppers, degree: number): void {
		if (!initialized) {
			initPCA9685()
		}
		setStepper(index, degree > 0);
		degree = Math.abs(degree);
		basic.pause(10240 * degree / 360);
		MotorStopAll()
	}


	//% blockId=motorbit_stepper_turn block="Stepper 28BYJ-48|%index|turn %turn"
	//% weight=90
	export function StepperTurn(index: Steppers, turn: Turns): void {
		let degree = turn;
		StepperDegree(index, degree);
	}

	//% blockId=motorbit_stepper_dual block="Dual Stepper(Degree) |STPM1_2 %degree1| STPM3_4 %degree2"
	//% weight=89
	export function StepperDual(degree1: number, degree2: number): void {
		if (!initialized) {
			initPCA9685()
		}
		setStepper(1, degree1 > 0);
		setStepper(2, degree2 > 0);
		degree1 = Math.abs(degree1);
		degree2 = Math.abs(degree2);
		basic.pause(10240 * Math.min(degree1, degree2) / 360);
		if (degree1 > degree2) {
			stopMotor(3); stopMotor(4);
			basic.pause(10240 * (degree1 - degree2) / 360);
		} else {
			stopMotor(1); stopMotor(2);
			basic.pause(10240 * (degree2 - degree1) / 360);
		}

		MotorStopAll()
	}

	/**
	 * Stepper Car move forward
	 * @param distance Distance to move in cm; eg: 10, 20
	 * @param diameter diameter of wheel in mm; eg: 48
	*/
	//% blockId=motorbit_stpcar_move block="Car Forward|Distance(cm) %distance|Wheel Diameter(mm) %diameter"
	//% weight=88
	export function StpCarMove(distance: number, diameter: number): void {
		if (!initialized) {
			initPCA9685()
		}
		let delay = 10240 * 10 * distance / 3 / diameter; // use 3 instead of pi
		setStepper(1, delay > 0);
		setStepper(2, delay > 0);
		delay = Math.abs(delay);
		basic.pause(delay);
		MotorStopAll()
	}

	/**
	 * Stepper Car turn by degree
	 * @param turn Degree to turn; eg: 90, 180, 360
	 * @param diameter diameter of wheel in mm; eg: 48
	 * @param track track width of car; eg: 125
	*/
	//% blockId=motorbit_stpcar_turn block="Car Turn|Degree %turn|Wheel Diameter(mm) %diameter|Track(mm) %track"
	//% weight=87
	//% blockGap=50
	export function StpCarTurn(turn: number, diameter: number, track: number): void {
		if (!initialized) {
			initPCA9685()
		}
		let delay = 10240 * turn * track / 360 / diameter;
		setStepper(1, delay < 0);
		setStepper(2, delay > 0);
		delay = Math.abs(delay);
		basic.pause(delay);
		MotorStopAll()
	}



	//% blockId=motorbit_stop_all block="Motor Stop All"
	//% weight=85
	//% blockGap=50
	export function MotorStopAll(): void {
		if (!initialized) {
			initPCA9685()
		}
		for (let idx = 1; idx <= 4; idx++) {
			stopMotor(idx);
		}
	}

	//% blockId=motorbit_stop block="Motor Stop|%index|"
	//% weight=84
	export function MotorStop(index: Motors): void {
		MotorRun(index, 0);
	}

	//% blockId=motorbit_motor_run block="Motor|%index|speed %speed"
	//% weight=82
	//% speed.min=-255 speed.max=255
	//% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
	export function MotorRun(index: Motors, speed: number): void {
		if (!initialized) {
			initPCA9685()
		}
		speed = speed * 16; // map 255 to 4096
		if (speed >= 4096) {
			speed = 4095
		}
		if (speed <= -4096) {
			speed = -4095
		}
		if (index > 4 || index <= 0)
			return
		let pp = (index - 1) * 2
		let pn = (index - 1) * 2 + 1
		if (speed >= 0) {
			setPwm(pp, 0, speed)
			setPwm(pn, 0, 0)
		} else {
			setPwm(pp, 0, 0)
			setPwm(pn, 0, -speed)
		}
	}

	/**
	 * Execute single motors with delay
	 * @param index Motor Index; eg: A01A02, B01B02, A03A04, B03B04
	 * @param speed [-255-255] speed of motor; eg: 150, -150
	 * @param delay seconde delay to stop; eg: 1
	*/
	//% blockId=motorbit_motor_rundelay block="Motor|%index|speed %speed|delay %delay|s"
	//% weight=81
	//% speed.min=-255 speed.max=255
	//% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
	export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
		MotorRun(index, speed);
		basic.pause(delay * 1000);
		MotorRun(index, 0);
	}



	/**
	 * Execute two motors at the same time
	 * @param motor1 First Motor; eg: A01A02, B01B02
	 * @param speed1 [-255-255] speed of motor; eg: 150, -150
	 * @param motor2 Second Motor; eg: A03A04, B03B04
	 * @param speed2 [-255-255] speed of motor; eg: 150, -150
	*/
	//% blockId=motorbit_motor_dual block="Motor|%motor1|speed %speed1|%motor2|speed %speed2"
	//% weight=80
	//% inlineInputMode=inline
	//% speed1.min=-255 speed1.max=255
	//% speed2.min=-255 speed2.max=255
	//% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
	export function MotorRunDual(motor1: Motors, speed1: number, motor2: Motors, speed2: number): void {
		MotorRun(motor1, speed1);
		MotorRun(motor2, speed2);
	}


	/**
	 * Init RGB pixels mounted on motorbit
	 */
	//% blockId="motorbit_rgb" block="RGB"
	//% weight=78
	export function rgb(): EMRGBLight.EmakefunRGBLight {
		if (!emRGBLight) {
			emRGBLight = EMRGBLight.create(DigitalPin.P2, 10, EMRGBPixelMode.RGB)
		}
		return emRGBLight;
	}

	/**
	 * Get RUS04 distance
	 * @param pin Microbit ultrasonic pin; eg: P2
	*/
	//% blockId=motorbit_ultrasonic block="Read RgbUltrasonic Distance|pin %pin|cm"
	//% weight=76
	export function Ultrasonic(pin: DigitalPin): number {
		return UltrasonicVer(pin, SonarVersion.V1);
	}

	function UltrasonicVer(pin: DigitalPin, v: SonarVersion): number {

		// send pulse
		if (v == SonarVersion.V1) {
			pins.setPull(pin, PinPullMode.PullNone);
		}
		else { pins.setPull(pin, PinPullMode.PullDown); }
		pins.digitalWritePin(pin, 0);
		control.waitMicros(2);
		pins.digitalWritePin(pin, 1);
		control.waitMicros(50);
		pins.digitalWritePin(pin, 0);

		// read pulse
		let d = pins.pulseIn(pin, PulseValue.High, 25000);
		let ret = d;
		// filter timeout spikes
		if (ret == 0 && distanceBuf != 0) {
			ret = distanceBuf;
		}
		distanceBuf = d;
		if (v == SonarVersion.V1) {
			return Math.floor(ret * 9 / 6 / 58);
		}
		return Math.floor(ret / 40 + (ret / 800));
		// Correction
	}

	function RgbDisplay(indexstart: number, indexend: number, rgb: RgbColors): void {
		for (let i = indexstart; i <= indexend; i++) {
			emRGBLight.setPixelColor(i, rgb);
		}
		emRGBLight.show();
	}

	//% blockId="motorbit_rus04" block="RgbUltrasonic|%RgbUltrasonics|show color %rgb|effect %ColorEffect|rgbpin %pin"
	//% weight=75
	export function RUS_04(index: RgbUltrasonics, rgb: RgbColors, effect: ColorEffect,pin:DigitalPin): void {
		let start, end;
		if (!emRGBLight) {
			emRGBLight = EMRGBLight.create(DigitalPin.P2, 6, EMRGBPixelMode.RGB)
		}
		if (index == RgbUltrasonics.Left) {
			start = 0;
			end = 2;
		} else if (index == RgbUltrasonics.Right) {
			start = 3;
			end = 5;
		} else if (index == RgbUltrasonics.All) {
			start = 0;
			end = 5;
		}
		switch(effect) {
			case ColorEffect.None:
				RgbDisplay(start, end, rgb);
				break;
			case ColorEffect.Breathing:
			for (let i = 0; i < 255; i+=2) {
				emRGBLight.setBrightness(i);
				RgbDisplay(start, end, rgb);
				//basic.pause((255 - i)/2);
				basic.pause((i < 20)? 80 :(255/i));
			}
			for (let i = 255; i > 0; i-=2) {
				emRGBLight.setBrightness(i);
				RgbDisplay(start, end, rgb);
				basic.pause((i < 20)? 80 :(255/i));
			}
			break;
			case ColorEffect.Rotate:
				for (let i = 0; i < 4; i++) {
					emRGBLight.setPixelColor(start, rgb);
					emRGBLight.setPixelColor(start+1, 0);
					emRGBLight.setPixelColor(start+2, 0);
					if (index == RgbUltrasonics.All) {
						emRGBLight.setPixelColor(end-2, rgb);
						emRGBLight.setPixelColor(end-1, 0);
						emRGBLight.setPixelColor(end, 0);
					}
					emRGBLight.show();
					basic.pause(150);
					emRGBLight.setPixelColor(start, 0);
					emRGBLight.setPixelColor(start+1, rgb);
					emRGBLight.setPixelColor(start+2, 0);
					if (index == RgbUltrasonics.All) {
						emRGBLight.setPixelColor(end-2, 0);
						emRGBLight.setPixelColor(end-1, rgb);
						emRGBLight.setPixelColor(end, 0);
					}
					emRGBLight.show();
					basic.pause(150);
					emRGBLight.setPixelColor(start, 0);
					emRGBLight.setPixelColor(start+1, 0);
					emRGBLight.setPixelColor(start+2, rgb);
					if (index == RgbUltrasonics.All) {
						emRGBLight.setPixelColor(end-2, 0);
						emRGBLight.setPixelColor(end-1, 0);
						emRGBLight.setPixelColor(end, rgb);
					}
					emRGBLight.show();
					basic.pause(150);
				}
				RgbDisplay(4, 9, 0);
				break;
			case ColorEffect.Flash:
			for (let i = 0; i < 6; i++) {
				RgbDisplay(start, end, rgb);
				basic.pause(150);
				RgbDisplay(start, end, 0);
				basic.pause(150);
			}
			break;
		}
	}
	//% blockId=motorbit_GetKeyCode
	//% block="keyboard|SODPIN %SDO|SCLPIN %SCL"
	export function GetKeyCode(SDO: DigitalPin, SCL: DigitalPin): string
	{

	  let DATA = 0;

	  pins.digitalWritePin(SDO, 1);
	  control.waitMicros(93);

	  pins.digitalWritePin(SDO, 0);
	  control.waitMicros(10);

	  for (let i = 0; i < 16; i++)
	  {

		   pins.digitalWritePin(SCL, 1);
		   pins.digitalWritePin(SCL, 0);

		   DATA |= pins.digitalReadPin(SDO) << i;   
	  }

	  control.waitMicros(2 * 1000);

	  switch (DATA&0xFFFF)
	  {     
		   case 0xFFFE: return "1"; 
		   case 0xFFFD: return "2"; 
		   case 0xFFFB: return "3"; 
		   case 0xFFEF: return "4";  
		   case 0xFFDF: return "5"; 
		   case 0xFFBF: return "6"; 
		   case 0xFEFF: return "7"; 
		   case 0xFDFF: return "8"; 
		   case 0xFBFF: return "9"; 
		   case 0xDFFF: return "0"; 
		   case 0xFFF7: return "A"; 
		   case 0xFF7F: return "B"; 
		   case 0xF7FF: return "C"; 
		   case 0x7FFF: return "D"; 
		   case 0xEFFF: return "*"; 
		   case 0xBFFF: return "#";      

		   default: return " ";
	  }          

	}
    /**
    * Setup/reset MAX7219s. If you are using 4-in-1 module you'll need to set rotation as true. If your chain are consisted of single modules set it as false (default).
    */
    //% block="Setup MAX7219:|Number of matrixs $num|CS(LOAD) = $cs|MOSI(DIN) = $mosi|MISO(not used) = $miso|SCK(CLK) = $sck"
    //% num.min=1 num.defl=1 cs.defl=DigitalPin.P16 mosi.defl=DigitalPin.P15 miso.defl=DigitalPin.P14 sck.defl=DigitalPin.P13 rotate.defl=false group="1. Setup"
    export function setup(num: number, cs: DigitalPin, mosi: DigitalPin, miso: DigitalPin, sck: DigitalPin) {
        // set internal variables        
        _pinCS = cs
        _matrixNum = num
        // prepare display array (for displaying texts; add extra 8 columns at each side as buffers)
        for (let i = 0; i < (num + 2) * 8; i++)  _displayArray.push(0)
        // set micro:bit SPI
        pins.spiPins(mosi, miso, sck)
        pins.spiFormat(8, 3)
        pins.spiFrequency(1000000)
        // initialize MAX7219s
        _registerAll(_SHUTDOWN, 0) // turn off
        _registerAll(_DISPLAYTEST, 0) // test mode off
        _registerAll(_DECODEMODE, 0) // decode mode off
        _registerAll(_SCANLIMIT, 7) // set scan limit to 7 (column 0-7)
        _registerAll(_INTENSITY, 15) // set brightness to 15
        _registerAll(_SHUTDOWN, 1) // turn on
        clearAll() // clear screen on all MAX7219s
    }

    /**
    * Rotation/reverse order options for 4-in-1 MAX7219 modules
    */
    //% block="Rotate matrix display $rotation|Reverse printing order $reversed" rotation.defl=rotation_direction.none group="1. Setup" blockExternalInputs=true advanced=true
    export function for_4_in_1_modules(rotation: rotation_direction, reversed: boolean) {
        _rotation = rotation
        _reversed = reversed
    }

    /**
    * (internal function) write command and data to all MAX7219s
    */
    function _registerAll(addressCode: number, data: number) {
        pins.digitalWritePin(_pinCS, 0) // LOAD=LOW, start to receive commands
        for (let i = 0; i < _matrixNum; i++) {
            // when a MAX7219 received a new command/data set
            // the previous one would be pushed to the next matrix along the chain via DOUT
            pins.spiWrite(addressCode) // command (8 bits)
            pins.spiWrite(data) //data (8 bits)
        }
        pins.digitalWritePin(_pinCS, 1) // LOAD=HIGH, commands take effect
    }

    /**
    * (internal function) write command and data to a specific MAX7219 (index 0=farthest on the chain)
    */
    function _registerForOne(addressCode: number, data: number, matrixIndex: number) {
        if (matrixIndex <= _matrixNum - 1) {
            pins.digitalWritePin(_pinCS, 0) // LOAD=LOW, start to receive commands
            for (let i = 0; i < _matrixNum; i++) {
                // when a MAX7219 received a new command/data set
                // the previous one would be pushed to the next matrix along the chain via DOUT
                if (i == matrixIndex) { // send change to target
                    pins.spiWrite(addressCode) // command (8 bits)
                    pins.spiWrite(data) //data (8 bits)
                } else { // do nothing to non-targets
                    pins.spiWrite(_NOOP)
                    pins.spiWrite(0)
                }
            }
            pins.digitalWritePin(_pinCS, 1) // LOAD=HIGH, commands take effect
        }
    }

    /**
    * (internal function) rotate matrix
    */
    function _rotateMatrix(matrix: number[][]): number[][] {
        let tmp = 0
        for (let i = 0; i < 4; i++) {
            for (let j = i; j < 7 - i; j++) {
                tmp = matrix[i][j]
                if (_rotation == rotation_direction.clockwise) { // clockwise
                    matrix[i][j] = matrix[j][7 - i]
                    matrix[j][7 - i] = matrix[7 - i][7 - j]
                    matrix[7 - i][7 - j] = matrix[7 - j][i]
                    matrix[7 - j][i] = tmp
                } else if (_rotation == rotation_direction.counterclockwise) { // counter-clockwise
                    matrix[i][j] = matrix[7 - j][i]
                    matrix[7 - j][i] = matrix[7 - i][7 - j]
                    matrix[7 - i][7 - j] = matrix[j][7 - i]
                    matrix[j][7 - i] = tmp
                } else if (_rotation == rotation_direction.one_eighty_degree) { // 180 degree
                    matrix[i][j] = matrix[7 - i][7 - j]
                    matrix[7 - i][7 - j] = tmp
                    tmp = matrix[7 - j][i]
                    matrix[7 - j][i] = matrix[j][7 - i]
                    matrix[j][7 - i] = tmp
                }
            }
        }
        return matrix
    }

    /**
    * (internal function) get 8x8 matrix from a column array
    */
    function _getMatrixFromColumns(columns: number[]): number[][] {
        let matrix: number[][] = getEmptyMatrix()
        for (let i = 0; i < 8; i++) {
            for (let j = 7; j >= 0; j--) {
                if (columns[i] >= 2 ** j) {
                    columns[i] -= 2 ** j
                    matrix[i][j] = 1
                } else if (columns[i] == 0) {
                    break
                }
            }
        }
        return matrix
    }

    /**
    * Scroll a text accross all MAX7219 matrixs for once
    */
    //% block="Scroll text $text|delay (ms) $delay|at the end wait (ms) $endDelay" text.defl="Hello world!" delay.min=0 delay.defl=75 endDelay.min=0 endDelay.defl=500 group="2. Display text on matrixs" blockExternalInputs=true
    export function scrollText(text: string, delay: number, endDelay: number) {
        let printPosition = _displayArray.length - 8
        let characters_index: number[] = []
        let currentChrIndex = 0
        let currentFontArray: number[] = []
        let nextChrCountdown = 1
        let chrCountdown: number[] = []
        let totalScrollTime = 0
        // clear screen and array
        for (let i = 0; i < _displayArray.length; i++) _displayArray[i] = 0
        clearAll()
        // get font index of every characters and total scroll time needed
        for (let i = 0; i < text.length; i++) {
            let index = font.indexOf(text.substr(i, 1))
            if (index >= 0) {
                characters_index.push(index)
                chrCountdown.push(font_matrix[index].length)
                totalScrollTime += font_matrix[index].length
            }
        }
        totalScrollTime += _matrixNum * 8
        // print characters into array and scroll the array
        for (let i = 0; i < totalScrollTime; i++) {
            nextChrCountdown -= 1
            if (currentChrIndex < characters_index.length && nextChrCountdown == 0) {
                // print a character just "outside" visible area
                currentFontArray = font_matrix[characters_index[currentChrIndex]]
                if (currentFontArray != null)
                    for (let j = 0; j < currentFontArray.length; j++)
                        _displayArray[printPosition + j] = currentFontArray[j]
                // wait until current character scrolled into visible area
                nextChrCountdown = chrCountdown[currentChrIndex]
                currentChrIndex += 1
            }
            // scroll array (copy all columns to the one before it)
            for (let j = 0; j < _displayArray.length - 1; j++) {
                _displayArray[j] = _displayArray[j + 1]
            }
            _displayArray[_displayArray.length - 1] = 0
            // write every 8 columns of display array (visible area) to each MAX7219s
            let matrixCountdown = _matrixNum - 1
            let actualMatrixIndex = 0
            for (let j = 8; j < _displayArray.length - 8; j += 8) {
                if (matrixCountdown < 0) break
                if (!_reversed) actualMatrixIndex = matrixCountdown
                else actualMatrixIndex = _matrixNum - 1 - matrixCountdown
                if (_rotation == rotation_direction.none) {
                    for (let k = j; k < j + 8; k++)
                        _registerForOne(_DIGIT[k - j], _displayArray[k], actualMatrixIndex)
                } else { // rotate matrix if needed
                    let tmpColumns = [0, 0, 0, 0, 0, 0, 0, 0]
                    let l = 0
                    for (let k = j; k < j + 8; k++) tmpColumns[l++] = _displayArray[k]
                    displayLEDsForOne(_getMatrixFromColumns(tmpColumns), actualMatrixIndex)
                }
                matrixCountdown--
            }
            basic.pause(delay)
        }
        basic.pause(endDelay)
    }

    /**
    * Print a text accross the chain of MAX7219 matrixs at a specific spot. Offset value -8 ~ last column of matrixs. You can choose to clear the screen or not (if not it can be used to print multiple string on the MAX7219 chain).
    */
    //% block="Display text $text|offset $offset|clear screen first $clear" text.defl="Hi!" offset.min=-8 clear.defl=true group="2. Display text on matrixs" blockExternalInputs=true
    export function displayText(text: string, offset: number, clear: boolean) {
        // clear screen and array if needed
        if (clear) {
            for (let i = 0; i < _displayArray.length; i++) _displayArray[i] = 0
            clearAll()
        }
        let printPosition = Math.constrain(offset, -8, _displayArray.length - 9) + 8
        let currentPosition = printPosition
        let characters_index: number[] = []
        let currentChrIndex = 0
        let currentFontArray: number[] = []
        // get font index of every characters
        for (let i = 0; i < text.length; i++) {
            let index = font.indexOf(text.substr(i, 1))
            if (index >= 0) characters_index.push(index)
        }
        // print characters into array from offset position
        while (currentPosition < _displayArray.length - 8) {
            currentFontArray = font_matrix[characters_index[currentChrIndex]]
            if (currentFontArray != null)
                for (let j = 0; j < currentFontArray.length; j++)
                    _displayArray[printPosition++] = currentFontArray[j]
            currentChrIndex += 1
            if (currentChrIndex == characters_index.length) break
        }
        // write every 8 columns of display array (visible area) to each MAX7219s
        let matrixCountdown = _matrixNum - 1
        let actualMatrixIndex = 0
        for (let i = 8; i < _displayArray.length - 8; i += 8) {
            if (matrixCountdown < 0) break
            if (!_reversed) actualMatrixIndex = matrixCountdown
            else actualMatrixIndex = _matrixNum - 1 - matrixCountdown
            if (_rotation == rotation_direction.none) {
                for (let j = i; j < i + 8; j++)
                    _registerForOne(_DIGIT[j - i], _displayArray[j], actualMatrixIndex)
            } else { // rotate matrix and reverse order if needed
                let tmpColumns = [0, 0, 0, 0, 0, 0, 0, 0]
                let l = 0
                for (let j = i; j < i + 8; j++)  tmpColumns[l++] = _displayArray[j]
                displayLEDsForOne(_getMatrixFromColumns(tmpColumns), actualMatrixIndex)
            }
            matrixCountdown--
        }
    }

    /**
    * Print a custom character from a number array on the chain of MAX7219 matrixs at a specific spot. Each number in the array is 0-255, the decimal version of column's byte number. Offset value -8 ~ last column of matrixs. You can choose to clear the screen or not (if not it can be used to print multiple string on the MAX7219 chain).
    */
    //% block="Display custom character from|number array $customCharArray|offset $offset|clear screen first $clear" offset.min=-8 clear.defl=true group="2. Display text on matrixs" blockExternalInputs=true advanced=true
    export function displayCustomCharacter(customCharArray: number[], offset: number, clear: boolean) {
        // clear screen and array if needed
        if (clear) {
            for (let i = 0; i < _displayArray.length; i++) _displayArray[i] = 0
            clearAll()
        }
        let printPosition: number = Math.constrain(offset, -8, _displayArray.length - 9) + 8
        if (customCharArray != null) {
            // print column data to display array
            for (let i = 0; i < customCharArray.length; i++)
                _displayArray[printPosition + i] = customCharArray[i]
            // write every 8 columns of display array (visible area) to each MAX7219s
            let matrixCountdown = _matrixNum - 1
            let actualMatrixIndex = 0
            for (let i = 8; i < _displayArray.length - 8; i += 8) {
                if (matrixCountdown < 0) break
                if (!_reversed) actualMatrixIndex = matrixCountdown
                else actualMatrixIndex = _matrixNum - 1 - matrixCountdown
                if (_rotation == rotation_direction.none) {
                    for (let j = i; j < i + 8; j++)
                        _registerForOne(_DIGIT[j - i], _displayArray[j], actualMatrixIndex)
                } else { // rotate matrix and reverse order if needed
                    let tmpColumns = [0, 0, 0, 0, 0, 0, 0, 0]
                    let l = 0
                    for (let j = i; j < i + 8; j++) tmpColumns[l++] = _displayArray[j]
                    displayLEDsForOne(_getMatrixFromColumns(tmpColumns), actualMatrixIndex)
                }
                matrixCountdown--
            }
        }
    }

    /**
    * Return a number array calculated from a 8x8 LED byte array (example: B00100000,B01000000,B10000110,B10000000,B10000000,B10000110,B01000000,B00100000)
    */
    //% block="Get custom character number array|from byte-array string $text" text.defl="B00100000,B01000000,B10000110,B10000000,B10000000,B10000110,B01000000,B00100000" group="2. Display text on matrixs" blockExternalInputs=true advanced=true
    export function getCustomCharacterArray(text: string) {
        let tempTextArray: string[] = []
        let resultNumberArray: number[] = []
        let currentIndex = 0
        let currentChr = ""
        let currentNum = 0
        let columnNum = 0
        if (text != null && text.length >= 0) {
            // seperate each byte number to a string
            while (currentIndex < text.length) {
                tempTextArray.push(text.substr(currentIndex + 1, 8))
                currentIndex += 10
            }
            for (let i = 0; i < tempTextArray.length; i++) {
                columnNum = 0
                // read each bit and calculate the decimal sum
                for (let j = tempTextArray[i].length - 1; j >= 0; j--) {
                    currentChr = tempTextArray[i].substr(j, 1)
                    if (currentChr == "1" || currentChr == "0")
                        currentNum = parseInt(currentChr)
                    else
                        currentNum = 0
                    columnNum += (2 ** (tempTextArray[i].length - j - 1)) * currentNum
                }
                // generate new decimal array
                resultNumberArray.push(columnNum)
            }
            return resultNumberArray
        } else {
            return null
        }
    }

    /**
    * Add a custom character from a number array at the end of the extension's font library.
    * Each number in the array is 0-255, the decimal version of column's byte number.
    */
    //% block="Add custom character $chr|number array $customCharArray|to the extension font library"
    //% chr.defl=""
    //% blockExternalInputs=true
    //% group="2. Display text on matrixs"
    //% advanced=true
    export function addCustomChr(chr: string, customCharArray: number[]) {
        if (chr != null && chr.length == 1 && customCharArray != null) {
            // add new character
            font.push(chr)
            font_matrix.push(customCharArray)
        }
    }

    /**
    * Display all fonts in the extension font library
    */
    //% block="Display all fonts at delay $delay" delay.min=0 delay.defl=200 group="2. Display text on matrixs" advanced=true
    export function fontDemo(delay: number) {
        let offsetIndex = 0
        clearAll()
        // print all characters on all matrixs
        for (let i = 1; i < font_matrix.length; i++) {
            // print two blank spaces to "reset" a matrix
            displayCustomCharacter(font_matrix[0], offsetIndex * 8, false)
            displayCustomCharacter(font_matrix[0], offsetIndex * 8 + 4, false)
            // print a character
            displayCustomCharacter(font_matrix[i], offsetIndex * 8, false)
            if (offsetIndex == _matrixNum - 1) offsetIndex = 0
            else offsetIndex += 1
            basic.pause(delay)
        }
        basic.pause(delay)
        clearAll()
    }

    /**
    * Turn on or off all MAX7219s
    */
    //% block="Turn on all matrixs $status" status.defl=true group="3. Basic light control" advanced=true
    export function togglePower(status: boolean) {
        if (status) _registerAll(_SHUTDOWN, 1)
        else _registerAll(_SHUTDOWN, 0)
    }

    /**
    * Set brightness level of LEDs on all MAX7219s
    */
    //% block="Set all brightness level $level" level.min=0 level.max=15 level.defl=15 group="3. Basic light control"
    export function brightnessAll(level: number) {
        _registerAll(_INTENSITY, level)
    }

    /**
    * Set brightness level of LEDs on a specific MAX7219s (index 0=farthest on the chain)
    */
    //% block="Set brightness level $level on matrix index = $index" level.min=0 level.max=15 level.defl=15 index.min=0 group="3. Basic light control" advanced=true
    export function brightnessForOne(level: number, index: number) {
        _registerForOne(_INTENSITY, level, index)
    }

    /**
    * Turn on all LEDs on all MAX7219s
    */
    //% block="Fill all LEDs" group="3. Basic light control"
    export function fillAll() {
        for (let i = 0; i < 8; i++) _registerAll(_DIGIT[i], 255)
    }

    /**
    * Turn on LEDs on a specific MAX7219
    */
    //% block="Fill LEDs on matrix index = $index" index.min=0 group="3. Basic light control" advanced=true
    export function fillForOne(index: number) {
        for (let i = 0; i < 8; i++) _registerForOne(_DIGIT[i], 255, index)
    }

    /**
    * Turn off LEDs on all MAX7219s
    */
    //% block="Clear all LEDs" group="3. Basic light control"
    export function clearAll() {
        for (let i = 0; i < 8; i++) _registerAll(_DIGIT[i], 0)
    }

    /**
    * Turn off LEDs on a specific MAX7219 (index 0=farthest on the chain)
    */
    //% block="Clear LEDs on matrix index = $index" index.min=0 group="3. Basic light control" advanced=true
    export function clearForOne(index: number) {
        for (let i = 0; i < 8; i++) _registerForOne(_DIGIT[i], 0, index)
    }

    /**
    * Turn on LEDs randomly on all MAX7219s
    */
    //% block="Randomize all LEDs" index.min=0 group="3. Basic light control"
    export function randomizeAll() {
        for (let i = 0; i < 8; i++) _registerAll(_DIGIT[i], Math.randomRange(0, 255))
    }

    /**
    * Turn on LEDs randomly on a specific MAX7219 (index 0=farthest on the chain)
    */
    //% block="Randomize LEDs on matrix index = $index" index.min=0 group="3. Basic light control" advanced=true
    export function randomizeForOne(index: number) {
        for (let i = 0; i < 8; i++) _registerForOne(_DIGIT[i], Math.randomRange(0, 255), index)
    }

    /**
    * Set LEDs of all MAX7219s to a pattern from a 8x8 matrix variable (index 0=farthest on the chain)
    */
    //% block="Display 8x8 pattern $newMatrix on all matrixs" group="4. Set custom LED pattern on matrixs" advanced=true
    export function displayLEDsToAll(newMatrix: number[][]) {
        let columnValue = 0
        if (newMatrix != null) {
            if (_rotation != rotation_direction.none) newMatrix = _rotateMatrix(newMatrix) // rotate matrix if needed
            for (let i = 0; i < 8; i++) {
                if (newMatrix[i] != null) {
                    columnValue = 0
                    for (let j = 0; j < 8; j++) {
                        if (newMatrix[i][j]) {
                            // combine row 0-7 status into a byte number (0-255)
                            columnValue += 2 ** j
                        }
                    }
                    _registerAll(_DIGIT[i], columnValue)
                }
            }
        }
    }

    /**
    * Set LEDs of a specific MAX7219s to a pattern from a 8x8 number matrix variable (index 0=farthest on the chain)
    */
    //% block="Display 8x8 pattern $newMatrix|on matrix index = $index" index.min=0 blockExternalInputs=true group="4. Set custom LED pattern on matrixs"
    export function displayLEDsForOne(newMatrix: number[][], index: number) {
        let columnValue = 0
        if (newMatrix != null) {
            if (_rotation != rotation_direction.none) newMatrix = _rotateMatrix(newMatrix) // rotate matrix if needed
            for (let i = 0; i < 8; i++) {
                if (newMatrix[i] != null) {
                    columnValue = 0
                    for (let j = 0; j < 8; j++) {
                        if (newMatrix[i][j]) {
                            // combine row 0-7 status into a byte number (0-255)
                            columnValue += 2 ** j
                        }
                    }
                    _registerForOne(_DIGIT[i], columnValue, index)
                }
            }
        }
    }

    /**
    * Return a empty 8x8 number matrix variable
    */
    //% block="Empty 8x8 pattern" group="4. Set custom LED pattern on matrixs"
    export function getEmptyMatrix() {
        return [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
        ]
    }

    /**
    * Return a full 8x8 number matrix variable
    */
    //% block="Full 8x8 pattern" group="4. Set custom LED pattern on matrixs" advanced=true
    export function getFullMatrix() {
        return [
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
        ]
    }

    /**
    * Return a specific value from a 8x8 number matrix variable
    */
    //% block="Get value from 8x8 pattern %matrix|x = $x y = $y" x.min=0 x.max=7 y.min=0 y.max=7 group="4. Set custom LED pattern on matrixs" blockExternalInputs=true advanced=true
    export function getValueFromMatrix(matrix: number[][], x: number, y: number) {
        return matrix[x][y]
    }

    /**
    * Set a specific value in a 8x8 number matrix variable
    */
    //% block="Set 8x8 pattern %matrix|x = $x y = $y value to $value" value.min=0 value.max=1 x.min=0 x.max=7 y.min=0 y.max=7 group="4. Set custom LED pattern on matrixs" blockExternalInputs=true
    export function setValueInMatrix(matrix: number[][], x: number, y: number, value: number) {
        matrix[x][y] = value
    }

    /**
    * Toggle (between 0/1) a specific value in a 8x8 number matrix variable
    */
    //% block="Toogle value in 8x8 pattern %matrix|x = $x y = $y" x.min=0 x.max=7 y.min=0 y.max=7 group="4. Set custom LED pattern on matrixs" blockExternalInputs=true advanced=true
    export function toogleValueInMatrix(matrix: number[][], x: number, y: number) {
        if (matrix[x][y] == 1) matrix[x][y] = 0
        else if (matrix[x][y] == 0) matrix[x][y] = 1
    }


    let font = [" ", "!", "\"", "#", "$", "%", "&", "\'", "(", ")",
        "*", "+", ",", "-", ".", "/",
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
        ":", ";", "<", "=", ">", "?", "@",
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L",
        "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
        "[", "\\", "]", "_", "`",
        "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l",
        "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z",
        "{", "|", "}", "~", "^"]

    let font_matrix = [
        [0b00000000,
            0b00000000,
            0b00000000,
            0b00000000],
        [0b01011111,
            0b00000000],
        [0b00000011,
            0b00000000,
            0b00000011,
            0b00000000],
        [0b00010100,
            0b00111110,
            0b00010100,
            0b00111110,
            0b00010100,
            0b00000000],
        [0b00100100,
            0b01101010,
            0b00101011,
            0b00010010,
            0b00000000],
        [0b01100011,
            0b00010011,
            0b00001000,
            0b01100100,
            0b01100011,
            0b00000000],
        [0b00110110,
            0b01001001,
            0b01010110,
            0b00100000,
            0b01010000,
            0b00000000],
        [0b00000011,
            0b00000000],
        [0b00011100,
            0b00100010,
            0b01000001,
            0b00000000],
        [0b01000001,
            0b00100010,
            0b00011100,
            0b00000000],
        [0b00101000,
            0b00011000,
            0b00001110,
            0b00011000,
            0b00101000,
            0b00000000],
        [0b00001000,
            0b00001000,
            0b00111110,
            0b00001000,
            0b00001000,
            0b00000000],
        [0b10110000,
            0b01110000,
            0b00000000],
        [0b00001000,
            0b00001000,
            0b00001000],
        [0b01100000,
            0b01100000,
            0b00000000],
        [0b01100000,
            0b00011000,
            0b00000110,
            0b00000001,
            0b00000000],
        [0b00111110,
            0b01000001,
            0b01000001,
            0b00111110,
            0b00000000],
        [0b01000010,
            0b01111111,
            0b01000000,
            0b00000000],
        [0b01100010,
            0b01010001,
            0b01001001,
            0b01000110,
            0b00000000],
        [0b00100010,
            0b01000001,
            0b01001001,
            0b00110110,
            0b00000000],
        [0b00011000,
            0b00010100,
            0b00010010,
            0b01111111,
            0b00000000],
        [0b00100111,
            0b01000101,
            0b01000101,
            0b00111001,
            0b00000000],
        [0b00111110,
            0b01001001,
            0b01001001,
            0b00110000,
            0b00000000],
        [0b01100001,
            0b00010001,
            0b00001001,
            0b00000111,
            0b00000000],
        [0b00110110,
            0b01001001,
            0b01001001,
            0b00110110,
            0b00000000],
        [0b00000110,
            0b01001001,
            0b01001001,
            0b00111110,
            0b00000000],
        [0b00010100,
            0b00000000],
        [0b00100000,
            0b00010100,
            0b00000000],
        [0b00001000,
            0b00010100,
            0b00100010,
            0b00000000],
        [0b00010100,
            0b00010100,
            0b00010100,
            0b00000000],
        [0b00100010,
            0b00010100,
            0b00001000,
            0b00000000],
        [0b00000010,
            0b01011001,
            0b00001001,
            0b00000110,
            0b00000000],
        [0b00111110,
            0b01001001,
            0b01010101,
            0b01011101,
            0b00001110,
            0b00000000],
        [0b01111110,
            0b00010001,
            0b00010001,
            0b01111110,
            0b00000000],
        [0b01111111,
            0b01001001,
            0b01001001,
            0b00110110,
            0b00000000],
        [0b00111110,
            0b01000001,
            0b01000001,
            0b00100010,
            0b00000000],
        [0b01111111,
            0b01000001,
            0b01000001,
            0b00111110,
            0b00000000],
        [0b01111111,
            0b01001001,
            0b01001001,
            0b01000001,
            0b00000000],
        [0b01111111,
            0b00001001,
            0b00001001,
            0b00000001,
            0b00000000],
        [0b00111110,
            0b01000001,
            0b01001001,
            0b01111010,
            0b00000000],
        [0b01111111,
            0b00001000,
            0b00001000,
            0b01111111,
            0b00000000],
        [0b01000001,
            0b01111111,
            0b01000001,
            0b00000000],
        [0b00110000,
            0b01000000,
            0b01000001,
            0b00111111,
            0b00000000],
        [0b01111111,
            0b00001000,
            0b00010100,
            0b01100011,
            0b00000000],
        [0b01111111,
            0b01000000,
            0b01000000,
            0b01000000,
            0b00000000],
        [0b01111111,
            0b00000010,
            0b00001100,
            0b00000010,
            0b01111111,
            0b00000000],
        [0b01111111,
            0b00000100,
            0b00001000,
            0b00010000,
            0b01111111,
            0b00000000],
        [0b00111110,
            0b01000001,
            0b01000001,
            0b00111110,
            0b00000000],
        [0b01111111,
            0b00001001,
            0b00001001,
            0b00000110,
            0b00000000],
        [0b00111110,
            0b01000001,
            0b01000001,
            0b10111110,
            0b00000000],
        [0b01111111,
            0b00001001,
            0b00001001,
            0b01110110,
            0b00000000],
        [0b01000110,
            0b01001001,
            0b01001001,
            0b00110010,
            0b00000000],
        [0b00000001,
            0b00000001,
            0b01111111,
            0b00000001,
            0b00000001,
            0b00000000],
        [0b00111111,
            0b01000000,
            0b01000000,
            0b00111111,
            0b00000000],
        [0b00001111,
            0b00110000,
            0b01000000,
            0b00110000,
            0b00001111,
            0b00000000],
        [0b00111111,
            0b01000000,
            0b00111000,
            0b01000000,
            0b00111111,
            0b00000000],
        [0b01100011,
            0b00010100,
            0b00001000,
            0b00010100,
            0b01100011,
            0b00000000],
        [0b00000111,
            0b00001000,
            0b01110000,
            0b00001000,
            0b00000111,
            0b00000000],
        [0b01100001,
            0b01010001,
            0b01001001,
            0b01000111,
            0b00000000],
        [0b01111111,
            0b01000001,
            0b00000000],
        [0b00000001,
            0b00000110,
            0b00011000,
            0b01100000,
            0b00000000],
        [0b01000001,
            0b01111111,
            0b00000000],
        [0b01000000,
            0b01000000,
            0b01000000,
            0b01000000,
            0b00000000],
        [0b00000001,
            0b00000010,
            0b00000000],
        [0b00100000,
            0b01010100,
            0b01010100,
            0b01111000,
            0b00000000],
        [0b01111111,
            0b01000100,
            0b01000100,
            0b00111000,
            0b00000000],
        [0b00111000,
            0b01000100,
            0b01000100,
            0b00101000,
            0b00000000],
        [0b00111000,
            0b01000100,
            0b01000100,
            0b01111111,
            0b00000000],
        [0b00111000,
            0b01010100,
            0b01010100,
            0b00011000,
            0b00000000],
        [0b00000100,
            0b01111110,
            0b00000101,
            0b00000000],
        [0b10011000,
            0b10100100,
            0b10100100,
            0b01111000,
            0b00000000],
        [0b01111111,
            0b00000100,
            0b00000100,
            0b01111000,
            0b00000000],
        [0b01000100,
            0b01111101,
            0b01000000,
            0b00000000],
        [0b01000000,
            0b10000000,
            0b10000100,
            0b01111101,
            0b00000000],
        [0b01111111,
            0b00010000,
            0b00101000,
            0b01000100,
            0b00000000],
        [0b01000001,
            0b01111111,
            0b01000000,
            0b00000000],
        [0b01111100,
            0b00000100,
            0b01111100,
            0b00000100,
            0b01111000,
            0b00000000],
        [0b01111100,
            0b00000100,
            0b00000100,
            0b01111000,
            0b00000000],
        [0b00111000,
            0b01000100,
            0b01000100,
            0b00111000,
            0b00000000],
        [0b11111100,
            0b00100100,
            0b00100100,
            0b00011000,
            0b00000000],
        [0b00011000,
            0b00100100,
            0b00100100,
            0b11111100,
            0b00000000],
        [0b01111100,
            0b00001000,
            0b00000100,
            0b00000100,
            0b00000000],
        [0b01001000,
            0b01010100,
            0b01010100,
            0b00100100,
            0b00000000],
        [0b00000100,
            0b00111111,
            0b01000100,
            0b00000000],
        [0b00111100,
            0b01000000,
            0b01000000,
            0b01111100,
            0b00000000],
        [0b00011100,
            0b00100000,
            0b01000000,
            0b00100000,
            0b00011100,
            0b00000000],
        [0b00111100,
            0b01000000,
            0b00111100,
            0b01000000,
            0b00111100,
            0b00000000],
        [0b01000100,
            0b00101000,
            0b00010000,
            0b00101000,
            0b01000100,
            0b00000000],
        [0b10011100,
            0b10100000,
            0b10100000,
            0b01111100,
            0b00000000],
        [0b01100100,
            0b01010100,
            0b01001100,
            0b00000000],
        [0b00001000,
            0b00110110,
            0b01000001,
            0b00000000],
        [0b01111111,
            0b00000000],
        [0b01000001,
            0b00110110,
            0b00001000,
            0b00000000],
        [0b00001000,
            0b00000100,
            0b00001000,
            0b00000100,
            0b00000000],
        [0b00000010,
            0b00000001,
            0b00000010,
            0b00000000]]


}
