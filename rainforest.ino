/*************************************************** 
 * This is an example for our Adafruit 16-channel PWM & Servo driver
 * PWM test - this will drive 16 PWMs in a 'wave'
 * 
 * Pick one up today in the adafruit shop!
 * ------> http://www.adafruit.com/products/815
 * 
 * These displays use I2C to communicate, 2 pins are required to  
 * interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4
 * 
 * Adafruit invests time and resources providing this open source code, 
 * please support Adafruit and open-source hardware by purchasing 
 * products from Adafruit!
 * 
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 * BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define NUM_LED   15
#define VOLT_PIN  A0

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

uint8_t currentLED = 0;
int blinkTime = 1000;

void allLEDOff(){
  for (uint8_t pwmnum=0; pwmnum < NUM_LED; pwmnum++) {
    pwm.setPWM(pwmnum, 4095,4095 );
  }
}


void allLEDOn(){
  for (uint8_t pwmnum=0; pwmnum < NUM_LED; pwmnum++) {
    pwm.setPWM(pwmnum, 0,4095 );
  }
}



void setup() {
  
  Serial.begin(9600);
  //Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  allLEDOff();


}

uint8_t randomLED(){
  uint8_t led = random(NUM_LED);
  while(currentLED==led){
    led = random(NUM_LED);
  }
  //Serial.println(led);
  currentLED = led;
  return led;
}  


int getVoltage(){
  int voltage = analogRead(VOLT_PIN);
  return voltage;
}


bool voltageRaised(){
  if(getVoltage() > 100)
    return true;
  else
    return false;
}

void turnOnLED(uint8_t led, int sleep){
  for(uint16_t i=0; i < 4096; i+=8){

    pwm.setPWM(led, 0,i );
    if(voltageRaised())
      break;
    delay(sleep);
  }  
}



void turnOffLED(uint8_t led, int sleep){
  for(uint16_t i=0; i < 4096; i+=8){
    pwm.setPWM(led, i,4095);
    if(voltageRaised())
      break;
  
    delay(sleep);
  }  
}


void loop() {
  int voltage = getVoltage();
  Serial.println(voltage);
  if(voltage < 100){
    // Random fairy light mode
    uint8_t led = randomLED(); 
    turnOnLED(led,1);
    delay(1000);
    turnOffLED(led,10);

  }else{
    if(voltage < 512){
      blinkTime = map(voltage, 0, 512, 700,300);

    }else{
      blinkTime = map(voltage, 512, 1024, 300,10);

    }
    
    allLEDOn();
    delay(blinkTime);
    allLEDOff();
    delay(blinkTime);
    
  }
  
/*  uint8_t led = randomLED();
  turnOnLED(led,1);
  delay(1000);
  turnOffLED(led,10);

  allLEDOn();
  delay(1000);
  allLEDOff();
*/  
//  delay(500);

}

