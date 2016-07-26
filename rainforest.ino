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
#define NUM_LED 15

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

uint8_t currentLED = 0;

void allLEDOff(){
  for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) {
    pwm.setPWM(pwmnum, 4095,4095 );
  }
}


void setup() {

  Serial.begin(9600);
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  allLEDOff();
  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!


}

uint8_t randomLED(){
  uint8_t led = random(NUM_LED);
  while(currentLED==led){
    led = random(NUM_LED);
    Serial.println(led);
  }
  Serial.println(led);
  currentLED = led;
  return led;
}  

void turnOnLED(uint8_t led, int sleep){
  for(uint16_t i=0; i < 4096; i+=8){

    pwm.setPWM(led, 0,i );
    delay(sleep);
  }  
}



void turnOffLED(uint8_t led, int sleep){
  for(uint16_t i=0; i < 4096; i+=8){
    pwm.setPWM(led, i,4095 );
    delay(sleep);
  }  
}



void loop() {
  uint8_t led = randomLED();
  turnOnLED(led,1);
  delay(1000);
  turnOffLED(led,10);
}

