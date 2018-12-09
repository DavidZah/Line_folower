#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <QTRSensors.h>


#define Kp 0.1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.1 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       4  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   10     // emitter is controlled by digital pin 2

#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 2
#define leftMotor1 5
#define leftMotor2 6
#define leftMotorPWM 7
#define motorPower 8

QTRSensorsAnalog qtrrc((unsigned char[]) { A0, A1, A2, A3, A4, A5,A6,A7} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19


// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
	// initialize the LCD
	lcd.begin();
  for(int counter=0;counter<100;counter++){
     qtrrc.calibrate();
     lcd.setCursor(0, 0);    
     lcd.print("calibration"); 
     delay(50);
    }
}

void loop()
{
	bool blinking = true;
	lcd.cursor();

	while (1) {
		 unsigned int sensors[8];
     int position = qtrrc.readLine(sensors); 
		 lcd.setCursor(0, 1); 
     lcd.print(position); 
     delay(100);
	}
}
