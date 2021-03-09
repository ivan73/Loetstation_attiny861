#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Bounce2.h>


//#define LCD
#define LCD 0
#define MAX7219 1
#define MAX6958 2
#define ANZEIGE MAX6958 //LCD //MAX6958 //MAX7219
#define Version "FW 2.5 by DFI"
#define VersionSeg "F2.5"

#include "Loetstation_FW2.4_7Seg.h"

uint16_t getTemperature();
void notAus(int code);
void btnSW_UP_Press();
void btnSW_DOWN_Press();

void setup()
{
  wdt_disable();
  pinMode(PIN_LED, OUTPUT);  // Arduino-Pin als Ausgang festlegen
  TCCR1A = 0b11110011; // enable fastPWM on A and B output compare units
  TCCR1C = 0b11110000;  // D output compare unit disabled
  TCCR1B = 0b00001000;  // 128 prescaler

  btnSW_UP.attach(PIN_SW_UP, INPUT_PULLUP);  btnSW_UP.interval(5); // interval in ms
  btnSW_DOWN.attach(PIN_SW_DOWN, INPUT_PULLUP);  btnSW_DOWN.interval(5); // interval in ms
  btnSTDBY.attach(PIN_STDBY, INPUT_PULLUP);  btnSTDBY.interval(10); // interval in ms

  digitalWrite(PIN_LED, HIGH); // Pin LED auf 1-Signal
  delay(500);
  digitalWrite(PIN_LED, LOW);  // Pin LED auf 0-Signal
  delay(500);

  target_temperature = EEPROM.read(EE_Target) | (((int) EEPROM.read(EE_Target + 1)) << 8);
  if (target_temperature < 0)
  {
    target_temperature = TARGET_TEMPERATURE_DEFAULT;
    EEPROM.write(EE_Target, target_temperature & 0xff);
    EEPROM.write(EE_Target + 1, (target_temperature & 0xff00) >> 8);
  }
  target_temperature_store = target_temperature;

  initDisplay();

  pinMode(PIN_STDBY, INPUT_PULLUP);
  
  //pinMode (PIN_PWM_OUT,OUTPUT);
  
  millisSetup = millis();
  millisSTDBY = millis() + (STDBY_SEC * 1000);

  cli();
  WDTCR = 0xD8 | WDTO_250MS;
  sei();

  wdt_reset();

  last_temperature = getTemperature();              // Letzten Temperaturwert auf aktuelle Temperatur setzen
  //last_temperature = last_temperature_1/16;
  //actual_temperature_anz = last_temperature;        // Startwert fuer die laufende Mittelwertbildung

  digitalWrite(PIN_LED, HIGH); // Pin LED auf 1-Signal

  
}

void loop()
{
  wdt_reset();
  if (millis() > millis_pid)
  {
    millis_pid = millis() + REFRESH_MILLIS_PID;
    actual_temperature = getTemperature();

    // Wenn wiederholt erratische Temperaturwerte empfangen werden -> NOT - AUS
    if (abs(actual_temperature - last_temperature) > NOTAUSDIFF)  {
      actual_temperature = last_temperature;
      unguelt++;
      if (unguelt > NOTAUSANZ) {
        notAus(3);
      }
    }
    else {
      unguelt = 0;
    }
    // Diffferenz aus Soll- und Ist-Temperatur berechnen
    diff = target_temperature_tmp - actual_temperature;

    //apply P controller:
    //pwm_value = diff * CNTRL_GAIN;
    
    if(sw_stdby){                                         // Wenn im Standby-Modus,
      pwm_value = P_KOEFF_P * diff;
      }                                                  //   berechne den neuen PWM-Wert mit P-Regler gegenueber Standby-Temperatur,
    else{                                                // sonst,
      esum+=diff;
      if (esum < (255/I_KOEFF_PID*-1)) {esum = 255/I_KOEFF_PID*-1;} //Begrenzung I-Anteil
      if (esum > (255/I_KOEFF_PID)) {esum = 255/I_KOEFF_PID;}
      
      pwm_value = P_KOEFF_PID * diff + I_KOEFF_PID * esum;
      pwm_value = pwm_value - D_KOEFF_PID * (actual_temperature-last_temperature);

      }       
      // PID-Regler
      //pwm = P_KOEFF_PID * (a_tsoll - a_tist) + I_KOEFF_PID * pwm;
      //pwm = pwm - D_KOEFF_PID * (a_tist - lr_tist);}  
      //#define P_KOEFF_PID  2    // 9 
      //#define I_KOEFF_PID  0.05   // 0.2
      //#define D_KOEFF_PID  4.9   // 19.5

    //limit pwm_value value to 0...255:
    pwm_value = pwm_value > 255 ? pwm_value = 255 : pwm_value < 0 ? pwm_value = 0 : pwm_value;

    //set PWM:
    OCR1B = 255 - pwm_value;

    //set heat LED
    if (pwm_value > 0)
      ledOn = true;
    else
      ledOn = false;
    digitalWrite(PIN_LED, ledOn);

    last_temperature = actual_temperature;
    millis_pid = millis() + REFRESH_MILLIS_PID;
  }

  if (sw_stdby)
    target_temperature_tmp = STDBY_TEMP_IN_DEGREES; //override target temperature
  else
    target_temperature_tmp = target_temperature;
  
  

  if (millis() > millis_lcd)
  {

    millis_lcd = millis() + REFRESH_MILLIS_LCD;
    lcd_display();
        
    if (!btnSW_UP.read())
      btnSW_UP_Press();
    if (!btnSW_DOWN.read())
      btnSW_DOWN_Press();

    //limit target_temperature value to 0...MAX:
    target_temperature = target_temperature > MAX_TARGET_TEMP_IN_DEGREES ? target_temperature = MAX_TARGET_TEMP_IN_DEGREES : target_temperature < 0 ? target_temperature = 0 : target_temperature;
  }

  btnSW_UP.update();
  btnSW_DOWN.update();
  btnSTDBY.update();

  if (btnSTDBY.rose())
    {
      sw_stdby = false;
      millisSTDBY = millis() + (STDBY_SEC * 1000);
    }
    if (abs(diff) < STDBY_DIFF || !btnSTDBY.read()) {
      if (millis() > millisSTDBY) {
        sw_stdby = true;
      }
    }
    else {
      millisSTDBY = millis() + (STDBY_SEC * 1000);
    }
    if (sw_stdby){
      millisSTDBY = millis() + (STDBY_SEC * 1000);
      if (diff >= STDBY_NORMAL_DIFF)
        sw_stdby = false;
      
    }

      

  if (!btnSW_UP.read() && !btnSW_DOWN.read())
    sw_stdby = true;

  if (target_temperature != target_temperature_store)
      targed_changed = true;
  
  if (targed_changed && millis() > millisStore)
    {
      EEPROM.write(EE_Target, target_temperature & 0xff);
      EEPROM.write(EE_Target + 1, (target_temperature & 0xff00) >> 8);
      target_temperature_store = target_temperature;
      targed_changed = false;
      
#if ANZEIGE == LCD
      OLED_pos(2, 13);
      OLED_str("Stored..");
#else

#endif
    }
  if (!targed_changed)
    millisStore = millis()+DELAY_BEFORE_STORE;

   if (sw_stdby) {
    if (millis() > millisOFF) {
      notAus(4);
    }
   }
   else {
    millisOFF = millis()+(STDBY_OFF_SEC*1000);
   }
}

void btnSW_UP_Click() {
  if (!sw_stdby)
    target_temperature++;
  else
    sw_stdby = false;
  millisStore = millis()+DELAY_BEFORE_STORE; 
}

void btnSW_DOWN_Click() {
  if (!sw_stdby) 
    target_temperature--;
  else
    sw_stdby = false;
  millisStore = millis()+DELAY_BEFORE_STORE; 
}

void btnSW_UP_Press() {
  if (!sw_stdby) {
    if (btnSW_UP.duration() > 1000)
      target_temperature += 5;
    else
      target_temperature++;
    
  }
  else
    sw_stdby = false;
  millisStore = millis()+DELAY_BEFORE_STORE; 
}

void btnSW_DOWN_Press() {
  if (!sw_stdby) {
    if (btnSW_DOWN.duration() > 1000)
      target_temperature -= 5;
    else
      target_temperature--;
  }
  else
    sw_stdby = false;
  millisStore = millis()+DELAY_BEFORE_STORE; 
}

// E01 - Bei extrem hoher Temperatur oder ohne Sensor -> NOT-AUS
// E02 - Wenn bei laufendem Betrieb immer noch keine Temperatur gemeldet wird -> NOT-AUS
// E03 - // Wenn wiederholt erratische Temperaturwerte empfangen werden -> NOT - AUS
// Not-Aus Routine, wird bei ungewoehnlichem Verhalten aufgerufen und schaltet Loetspitze aus
void notAus(int code) {                                        
  pinMode(PIN_PWM_OUT, INPUT);
  OCR1B = 255 - 0;  // OCR1B = 255 - pwm_value;
  digitalWrite(PIN_LED, LOW);
  displayNotAus(code);
  wdt_reset();
  delay(100);
  while (code>0)
  {
    wdt_reset();
    btnSW_UP.update();
    btnSW_DOWN.update();
     if (!btnSW_UP.read() && !btnSW_DOWN.read()) {
        digitalWrite(PIN_LED, HIGH);
        while (true) {} // Endlosschleife bewirkt Watchdog-Reset und damit einen Controller-Neustart
     }
     digitalWrite(PIN_LED, LOW);
  }
}


/*
  Lese die Temperatur vom AD-Wandler
*/
uint16_t getTemperature()
{
  int i = 0;
  uint32_t adcValue = 0;

  pinMode(PIN_PWM_OUT, INPUT);
  OCR1B = 255 - pwm_value;
  delay(DELAY_BEFORE_MEASURE); //wait for some time (to get low pass filter in steady state)
  
  while (i<16) { // sample 16 times to get a virtual 16 bit reading..
		adcValue +=  analogRead(PIN_Temp); // read the input on analog pin
		i++;
	}
  pinMode(PIN_PWM_OUT, OUTPUT);
  OCR1B = 255 - pwm_value;

  if (adcValue > (1000*16)) {
    notAus(1); // Bei extrem hoher Temperatur oder ohne Sensor -> NOT-AUS
  }
  if (millis() > (millisSetup + NOTAUSABW) && adcValue < (70*16)) {
    notAus(2); // Wenn bei laufendem Betrieb immer noch keine Temperatur gemeldet wird -> NOT-AUS
  }
  else {
    millisSetup = millis();
  }
  

  //return A_ADC*adcValue*adcValue + B_ADC*adcValue + C_ADC + 0.5;
  return round((((float) adcValue) * ADC_TO_TEMP_GAIN )/16+ADC_TO_TEMP_OFFSET); //apply linear conversion to actual temperature
}
