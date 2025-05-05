// CPE 301 Final Project || Swamp Cooler
// Austin Landis & Jacob Bledsoe

#include <LiquidCrystal.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <Stepper.h>

// ====== LED CONFIGURATION ======
#define GREEN_LED_PIN  PORTB7  // Pin 13
#define RED_LED_BIT    PH5     // Pin 8
#define YELLOW_LED_BIT PB4     // Pin 10

#define GREEN_LED_DDR  DDRB
#define GREEN_LED_PORT PORTB
#define RED_LED_DDR    DDRH
#define RED_LED_PORT   PORTH
#define YELLOW_LED_DDR DDRB
#define YELLOW_LED_PORT PORTB

#define GREEN_LED_ON()     (GREEN_LED_PORT |= (1 << GREEN_LED_PIN))
#define GREEN_LED_OFF()    (GREEN_LED_PORT &= ~(1 << GREEN_LED_PIN))
#define GREEN_LED_INIT()   do { GREEN_LED_DDR |= (1 << DDB7); GREEN_LED_OFF(); } while(0)

#define RED_LED_ON()       (RED_LED_PORT |= (1 << RED_LED_BIT))
#define RED_LED_OFF()      (RED_LED_PORT &= ~(1 << RED_LED_BIT))
#define RED_LED_INIT()     do { RED_LED_DDR |= (1 << RED_LED_BIT); RED_LED_OFF(); } while(0)

#define YELLOW_LED_ON()    (YELLOW_LED_PORT |= (1 << YELLOW_LED_BIT))
#define YELLOW_LED_OFF()   (YELLOW_LED_PORT &= ~(1 << YELLOW_LED_BIT))
#define YELLOW_LED_INIT()  do { YELLOW_LED_DDR |= (1 << YELLOW_LED_BIT); YELLOW_LED_OFF(); } while(0)

// ====== FAN CONFIGURATION ======
#define FAN_PIN     PORTH3   // Pin 6
#define FAN_PORT    PORTH
#define FAN_DDR     DDRH
#define FAN_TEMP_THRESHOLD 25

// ====== LCD CONFIGURATION ======
#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ====== DHT SENSOR CONFIGURATION ======
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ====== RTC CONFIGURATION ======
RTC_DS3231 rtc;

// ====== WATER SENSOR (ADC) CONFIGURATION ======
volatile unsigned char* my_ADMUX    = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int*  my_ADC_DATA = (unsigned int*) 0x78;

// ====== BUTTON CONFIGURATION ======
#define RESET_BUTTON_PINH PINH
#define RESET_BUTTON_BIT  PH6  // Pin 9

#define START_BUTTON_PINA PIND
#define START_BUTTON_BIT  PIND3  // Pin 18

#define STOP_BUTTON_PINA  PINA
#define STOP_BUTTON_BIT   PINA1  // Pin 23

#define VENT_RIGHT_BUTTON_PIND  PINA
#define VENT_RIGHT_BUTTON_BIT   PINA2  // Pin 24

#define VENT_LEFT_BUTTON_PIND   PINA
#define VENT_LEFT_BUTTON_BIT    PINA3  // Pin 25

// ====== UART CONFIGURATION ======
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

void U0init(unsigned long baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit() { return (*myUCSR0A & RDA) ? 1 : 0; }
unsigned char U0getchar() { while (!U0kbhit()); return *myUDR0; }
void U0putchar(unsigned char data) { while (!(*myUCSR0A & TBE)); *myUDR0 = data; }

void uart_print(const char* str) { while (*str) U0putchar(*str++); }

void uart_print_int(int val) {
  char buf[10];
  itoa(val, buf, 10);
  uart_print(buf);
}

void uart_print_float(float val) {
  char buf[10];
  dtostrf(val, 5, 1, buf);
  uart_print(buf);
}

// ====== STEPPER MOTOR (VENT) CONFIGURATION ======
#define STEPS_PER_REV 2048
#define IN1 51
#define IN2 50
#define IN3 49
#define IN4 48
Stepper stepper(STEPS_PER_REV, IN1, IN3, IN2, IN4);

// ====== SYSTEM STATE ======
enum SystemState { DISABLED, ERROR, IDLE, RUNNING };
SystemState currentState = DISABLED;
SystemState lastDisplayedState = DISABLED;

// ====== GLOBAL VARIABLES ======
float temperature = 0.0;
float humidity = 40.0;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 60000;

// ====== INTERRUPT SETUP ======
#define START_BUTTON_PIN 18
volatile bool startButtonPressed = false;

void startButtonISR() {
  startButtonPressed = true;
}

// ====== SETUP FUNCTION ======
void setup() {
  stepper.step(1);  // Nudge stepper on power-up
  U0init(9600);

  GREEN_LED_INIT();
  RED_LED_INIT();
  YELLOW_LED_INIT();

  FAN_DDR |= (1 << PH3);  // Fan pin as output
  FAN_PORT &= ~(1 << FAN_PIN);  // Fan off

  DDRH &= ~(1 << RESET_BUTTON_BIT);  // Reset button input
  PORTH |= (1 << RESET_BUTTON_BIT);  // Enable pull-up

  DDRA &= ~((1 << STOP_BUTTON_BIT) | (1 << VENT_RIGHT_BUTTON_BIT) | (1 << VENT_LEFT_BUTTON_BIT));
  PORTA |= (1 << STOP_BUTTON_BIT) | (1 << VENT_RIGHT_BUTTON_BIT ) | (1 << VENT_LEFT_BUTTON_BIT);

  DDRD &= ~(1 << START_BUTTON_BIT);
  PORTD |= (1 << START_BUTTON_BIT);
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonISR, FALLING);

  lcd.begin(16, 2);
  dht.begin();
  adc_init();
  stepper.setSpeed(10);

  if (!rtc.begin()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RTC not found!");
    while (1);
  }

  enterDisabledState();
  updateDisplay();
}

// ====== MAIN LOOP ======
void loop() {
  handleButtons();

  if (startButtonPressed && currentState == DISABLED) {
    enterIdleState();
    startButtonPressed = false;
  }

  unsigned long now = millis();
  if (now - lastDisplayUpdate > displayInterval) {
    updateDisplay();
    lastDisplayUpdate = now;
  }

  switch (currentState) {
    case DISABLED:
    case ERROR:
      FAN_PORT &= ~(1 << FAN_PIN);
      break;

    case IDLE:
    case RUNNING:
      temperature = dht.readTemperature();
      humidity = dht.readHumidity();

      if (adc_read(0) < 50) {
        enterErrorState();
        return;
      }

      if (!isnan(temperature) && temperature >= FAN_TEMP_THRESHOLD) {
        FAN_PORT |= (1 << FAN_PIN);
        if (currentState != RUNNING) logFanStatus("ON");
        currentState = RUNNING;
        GREEN_LED_OFF();
      } else {
        FAN_PORT &= ~(1 << FAN_PIN);
        if (currentState != IDLE) logFanStatus("OFF");
        currentState = IDLE;
        GREEN_LED_ON();
      }
      break;
  }
}

// ====== BUTTON HANDLING ======
void handleButtons() {
  bool startPressed     = !(START_BUTTON_PINA & (1 << START_BUTTON_BIT));
  bool stopPressed      = !(STOP_BUTTON_PINA & (1 << STOP_BUTTON_BIT));
  bool resetPressed     = !(RESET_BUTTON_PINH & (1 << RESET_BUTTON_BIT));
  bool ventRightPressed = !(VENT_RIGHT_BUTTON_PIND & (1 << VENT_RIGHT_BUTTON_BIT));
  bool ventLeftPressed  = !(VENT_LEFT_BUTTON_PIND & (1 << VENT_LEFT_BUTTON_BIT));

  if (startPressed && currentState == DISABLED) enterIdleState();
  else if (stopPressed && currentState != DISABLED) enterDisabledState();
  else if (resetPressed && currentState == ERROR) {
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("System Resetting");
    delay(1500); enterIdleState();
  } else if (ventRightPressed && currentState != DISABLED) {
    logVentAction("Right");
    stepper.step(STEPS_PER_REV / 4);  // 90° CW
  } else if (ventLeftPressed && currentState != DISABLED) {
    logVentAction("Left");
    stepper.step(-STEPS_PER_REV / 4);  // 90° CCW
  }
}

// ====== DISPLAY UPDATE ======
void updateDisplay() {
  if (currentState != lastDisplayedState) {
    lcd.clear();
    lastDisplayedState = currentState;
  }

  switch (currentState) {
    case DISABLED:
      lcd.setCursor(0, 0); lcd.print("System Disabled ");
      lcd.setCursor(0, 1); lcd.print("Press START Btn ");
      break;

    case ERROR:
      lcd.setCursor(0, 0); lcd.print("WATER LEVEL LOW!");
      lcd.setCursor(0, 1); lcd.print("Press RESET Btn ");
      break;

    case IDLE:
    case RUNNING: {
      float temp = isnan(temperature) ? -999 : temperature;
      float hum  = isnan(humidity) ? -1 : humidity;
      unsigned int waterADC = adc_read(0);

      lcd.setCursor(0, 0);
      lcd.print("T:"); lcd.print(temp == -999 ? "--" : String(temp, 1));
      lcd.print("C H:"); lcd.print(hum == -1 ? "--" : String(hum, 0)); lcd.print("%");

      lcd.setCursor(0, 1);
      lcd.print("W: "); lcd.print(waterADC);
      lcd.print(" ("); lcd.print(currentState == RUNNING ? "RUNNING" : "IDLE   "); lcd.print(")");
      break;
    }
  }
}

// ====== STATE TRANSITIONS ======
void enterDisabledState() {
  currentState = DISABLED;
  logStateChange("DISABLED");
  YELLOW_LED_ON(); RED_LED_OFF(); GREEN_LED_OFF();
  FAN_PORT &= ~(1 << FAN_PIN);
  lastDisplayedState = (SystemState)(-1);
  updateDisplay();
}

void enterErrorState() {
  currentState = ERROR;
  logStateChange("ERROR");
  YELLOW_LED_OFF(); GREEN_LED_OFF(); RED_LED_ON();
  FAN_PORT &= ~(1 << FAN_PIN);
  lastDisplayedState = (SystemState)(-1);
  updateDisplay();
}

void enterIdleState() {
  currentState = IDLE;
  logStateChange("IDLE");
  YELLOW_LED_OFF(); RED_LED_OFF(); GREEN_LED_ON();
  FAN_PORT &= ~(1 << FAN_PIN);
  lastDisplayedState = (SystemState)(-1);
  updateDisplay();
}

// ====== ADC FUNCTIONS ======
void adc_init() {
  *my_ADCSRA = 0b10000000;
  *my_ADMUX  = 0b01000000;
  *my_ADCSRB = 0b00000000;
}

unsigned int adc_read(unsigned char channel) {
  *my_ADMUX &= 0b11100000;
  *my_ADMUX |= (channel & 0x1F);
  *my_ADCSRA |= 0b01000000;
  while (*my_ADCSRA & 0b01000000);
  return *my_ADC_DATA;
}

// ====== LOGGING ======
void logFanStatus(const char* status) {
  DateTime now = rtc.now();
  uart_print("Fan "); uart_print(status); uart_print(" at ");
  uart_print_int(now.year()); U0putchar('/');
  uart_print_int(now.month()); U0putchar('/');
  uart_print_int(now.day()); U0putchar(' ');
  uart_print_int(now.hour()); U0putchar(':');
  uart_print_int(now.minute()); U0putchar(':');
  uart_print_int(now.second()); U0putchar('\n');
}

void logStateChange(const char* state) {
  DateTime now = rtc.now();
  uart_print(state); uart_print(" State at ");
  uart_print_int(now.year()); U0putchar('/');
  uart_print_int(now.month()); U0putchar('/');
  uart_print_int(now.day()); U0putchar(' ');
  uart_print_int(now.hour()); U0putchar(':');
  uart_print_int(now.minute()); U0putchar(':');
  uart_print_int(now.second()); U0putchar('\n');
}

void logVentAction(const char* direction) {
  DateTime now = rtc.now();
  uart_print("Vent "); uart_print(direction); uart_print(" at ");
  uart_print_int(now.year()); U0putchar('/');
  uart_print_int(now.month()); U0putchar('/');
  uart_print_int(now.day()); U0putchar(' ');
  uart_print_int(now.hour()); U0putchar(':');
  uart_print_int(now.minute()); U0putchar(':');
  uart_print_int(now.second()); U0putchar('\n');
}
