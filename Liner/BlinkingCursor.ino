#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <QTRSensors.h>


#define Kp 0.1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.1 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 510 // max speed of the robot
#define leftMaxSpeed 510 // max speed of the robot
#define rightBaseSpeed 500 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 500  // this is the speed at which the motors should spin when the robot is perfectly on the line

#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       4  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   10     // emitter is controlled by digital pin 2

QTRSensorsAnalog qtrrc((unsigned char[]) { A0, A1, A2, A3, A4, A5,A6,A7} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19


int position; 

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void print_display(){
   lcd.setCursor(0, 1); 
   lcd.print(position);
  }
void motor_setup(){
  for(int i=2;i<8;i++)
    pinMode(i,OUTPUT);
}
void motor1_left(int pwm){
    digitalWrite(3, 1);
    digitalWrite(4, 0);
    analogWrite(2, pwm);
}
void motor1_right(int pwm){
    digitalWrite(3, 0);
    digitalWrite(4, 1);
    analogWrite(2, pwm);
}
void motor2_right(int pwm){
    digitalWrite(5, 1);
    digitalWrite(6, 0);
    analogWrite(7, pwm);
}
void motor2_left(int pwm){
    digitalWrite(5, 0);
    digitalWrite(6, 1);
    analogWrite(7, pwm);
}
void motor(int pwm1, int pwm2 ){
  // motor 1 (0-254) backward, (255) stop, (256-510) forward
  if(pwm1 < 255 && pwm1 >=0){
    motor1_left(pwm1);
  }else if(pwm1>255 && pwm1 <= 510){
    pwm1 -= 255;
    motor1_right(pwm1);
  }else if(pwm1 == 255){
    motor1_left(0);  
  }
  // motor 2 (0-254) backward, (255) stop, (256-510) forward
  if(pwm2 < 255 && pwm2 >=0){
    motor2_right(pwm2);
  }else if(pwm2>255 && pwm2 <= 510){
    pwm2 -= 255;
    motor2_left(pwm2);
  }else if(pwm1 == 255){
    motor2_left(0);  
  }
}

void setup()
{
  motor_setup();
  Serial.begin(9600);
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
  int lastError =0; 
	while (1) {
		 unsigned int sensors[8];
     position = qtrrc.readLine(sensors);
      
		 int error = position - 3500;
    
		 int motorSpeed = Kp * error + Kd * (error - lastError);
     lastError = error;  

     int rightMotorSpeed = rightBaseSpeed + motorSpeed;
     int leftMotorSpeed = leftBaseSpeed - motorSpeed;

     if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
     if (leftMotorSpeed  > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
     if (rightMotorSpeed  < 0) rightMotorSpeed = 0; // keep the motor speed positive
     if (leftMotorSpeed   < 0) leftMotorSpeed = 0; // keep the motor speed positive

     Serial.println(rightMotorSpeed);
     Serial.println(leftMotorSpeed);
     motor(rightMotorSpeed,leftMotorSpeed);
  
     print_display(); 
	}
}
