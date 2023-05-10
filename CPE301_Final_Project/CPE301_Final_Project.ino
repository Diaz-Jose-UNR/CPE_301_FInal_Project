#include <LiquidCrystal.h>
#include <Stepper.h> 
#include <RTClib.h>
#include "DHT.h"

// Water Level Sensor
#define RDA 0x80
#define TBE 0x20
#define WATER_SENSOR_PIN 1
const int WATER_THRESHOLD = 150;

// RTC Clock
RTC_DS1307 rtc;

unsigned long lastUpdateTime = 0;

// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
// Timer Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// GPIO Pointers
volatile unsigned char *portB     = (unsigned char*) 0x25;
volatile unsigned char *portDDRB  = (unsigned char*) 0x24;

// Button Pointers
volatile uint8_t *ddrB = (uint8_t *)0x24;
volatile uint8_t *pinB = (uint8_t *)0x23; 

// LCD and Temperature/Humidity Sensor
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

volatile unsigned int* portH = (unsigned int*)0x102;
volatile unsigned int* ddrH = (unsigned int*)0x101;

// Motor Pins
#define ENA 5
#define IN1 4
#define IN2 3

// Stepper Motor
#define IN1 22 
#define IN2 24
#define IN3 26
#define IN4 28
const int stepsPerRevolution = 2038;
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

#define POTENTIOMETER_PIN A5

// LED Pins
#define BLUE_LED_PIN 0
#define GREEN_LED_PIN 1
#define YELLOW_LED_PIN 2
#define RED_LED_PIN 3

volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c = (unsigned char*) 0x27;

// States and Helpers
const int DISABLED = 0;
const int IDLE = 1;
const int ERROR = 2;
const int RUNNING = 3;
int currentState = IDLE;
bool resetButton;

void setup() {
  *ddr_c |= (1 << RED_LED_PIN) | (1 << YELLOW_LED_PIN) | (1 << GREEN_LED_PIN) | (1 << BLUE_LED_PIN);

  *port_c |= (1 << RED_LED_PIN) | (1 << YELLOW_LED_PIN) | (1 << GREEN_LED_PIN) | (1 << BLUE_LED_PIN);
  U0init(9600);
  *ddrH &= ~(1 << WATER_SENSOR_PIN);
  
  volatile uint8_t *ddrD = (uint8_t *)0x2A;
  *ddrD |= (1 << ENA) | (1 << IN1) | (1 << IN2);

  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  motorStop();
  adc_init();
  dht.begin();
  lcd.begin(16, 2);
  myStepper.setSpeed(10);

  // Set Reset Button as Input
  resetButton = false;
  *ddrB &= ~(1 << 1); 
}

void loop() {
  readAndDisplayTempAndHumidity();

  unsigned long currentTime = millis();
  float waterLevel;

  // Check reset button status
  resetButton = (*pinB & (1 << 1)) != 0;
  if (resetButton) {
    currentState = IDLE;
    reportEvent("Button pressed. Current State: IDLE");
        turnOffLED(BLUE_LED_PIN);
        turnOffLED(RED_LED_PIN);
        turnOffLED(YELLOW_LED_PIN);
        turnOffLED(GREEN_LED_PIN);
  }

  switch (currentState) {
    case DISABLED:
      turnOnLED(YELLOW_LED_PIN);
      reportEvent("Current State: DISABLED");
      if(currentState != DISABLED){
        turnOffLED(YELLOW_LED_PIN);
      }
      break;
    case IDLE:    
    turnOnLED(GREEN_LED_PIN);
      reportEvent("Current State: IDLE");
      if (currentTime - lastUpdateTime >= 60000) {
          lastUpdateTime = currentTime;
          readAndDisplayTempAndHumidity();
      }
      waterLevel = checkWaterLevel();
      if(waterLevel < WATER_THRESHOLD){
        currentState = ERROR;
        turnOffLED(GREEN_LED_PIN);
      }
      if(dht.readTemperature() > 25){
        currentState = RUNNING;
        turnOffLED(GREEN_LED_PIN);
      }
      break;

    case ERROR:
      turnOnLED(RED_LED_PIN);
      reportEvent("Current State: ERROR");
      if (currentTime - lastUpdateTime >= 60000) {
          lastUpdateTime = currentTime;
          readAndDisplayTempAndHumidity();
      }
      break;

    case RUNNING:
      turnOnLED(BLUE_LED_PIN);
      reportEvent("Current State: RUNNING");
      if (currentTime - lastUpdateTime >= 60000) {
          lastUpdateTime = currentTime;
          readAndDisplayTempAndHumidity();
      }
      enableFanMotor();
      enableVentMotor();

      float temperature = dht.readTemperature();
        if (temperature < 25) {
          currentState = IDLE;
          turnOffLED(BLUE_LED_PIN);
        }
      waterLevel = checkWaterLevel();
      if(waterLevel < WATER_THRESHOLD){
        currentState = ERROR;
        turnOffLED(BLUE_LED_PIN);
      }
      break;
  }
  // delay(100);
}

void turnOnLED(int LED) {
  *port_c &= ~(1 << LED); 
}

void turnOffLED(int LED) {
  *port_c |= (1 << LED);
}

void printMessage(const char* message) {
  while(*message) {
    U0putchar(*message++);
  }
  U0putchar('\n');
}

float checkWaterLevel() {
  int waterLevel = adc_read(WATER_SENSOR_PIN);
  if (waterLevel < 150) { 
    printMessage("Water level too low!");
  }
  return waterLevel;
}

void readAndDisplayTempAndHumidity() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print((char)223); // Degree symbol
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
}

void enableFanMotor() {
  float temperature = dht.readTemperature();
  if (temperature > 25) {
    motorControl(255, true);
  }
  else {
    motorStop();
  }
}

void enableVentMotor() {
  int potentiometerReading = analogRead(POTENTIOMETER_PIN);
  Serial.println(analogRead(POTENTIOMETER_PIN));
  if (potentiometerReading > 250){
    myStepper.step(-stepsPerRevolution/20);
  }
  else if (potentiometerReading <= 50){
    myStepper.step(stepsPerRevolution/20);
  }
}


void reportEvent(const char *eventMessage) {
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" - ");
  Serial.println(eventMessage);
}

void motorControl(int speed, bool direction) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, direction);
  digitalWrite(IN2, !direction);
}

void motorStop() {
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void adc_init(){
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
