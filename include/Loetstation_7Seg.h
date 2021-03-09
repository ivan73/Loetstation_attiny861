#if ANZEIGE == LCD
  #include "LedControl.h"

#elif ANZEIGE == MAX7219

#elif ANZEIGE == MAX6958
  #include <TinyWireS.h>

#endif


#define PIN_LED   PIN_A6
#define PIN_PWM_OUT PIN_B3
#define PIN_Temp A0 // IC-Pin20

#define PIN_SW_UP PIN_B5
#define PIN_SW_DOWN PIN_B4

#define PIN_STDBY PIN_A5

  // Definition of Portpins for Software SPI
#define SS_Pin    PIN_A7
#define MOSI_Pin  PIN_B0
#define CLK_Pin   PIN_B2

Bounce btnSW_UP = Bounce();
Bounce btnSW_DOWN = Bounce();
Bounce btnSTDBY = Bounce();

//R2=68K:
#define ADC_TO_TEMP_GAIN 0.436378866  //0.42278496 V.1    //0.436378866 V.2
#define ADC_TO_TEMP_OFFSET 22.5         //23.9

#define A_ADC        -0.000155                        // Koeffizient des quadratrischen Terms im Zusammenhang zwischen ADC-Wert und IST-Temperatur -0.000151
#define B_ADC        0.533                            // Koeffizient des linearen Terms im Zusammenhang zwischen ADC-Wert und IST-Temperatur       0.533
#define C_ADC        37.40                            // Offset des quadratischen Zusammenhangs zwischen ADC-Wert und IST-Temperatur               37.40
#define W_AKT        0.7                              // Gewicht des aktuellen Temperaturwertes bei der laufenden Mittelwertbildung     

#define CNTRL_GAIN 10

#define P_KOEFF_P    7          // Koeffizient des P-Reglers; VORSICHT: Darf maximal einen Wert von 30 annehmen, sonst Int-Problem im Regler
#define P_KOEFF_PID  4          // P-Anteil alt: 2 
#define I_KOEFF_PID  0.005       // I-Anteil alt: 0.03
#define D_KOEFF_PID  10        // D-Anteil alt: 5.5

#define MAX_TARGET_TEMP_IN_DEGREES 400
#define TARGET_TEMPERATURE_DEFAULT 280
#define STDBY_TEMP_IN_DEGREES 100

#define NOTAUSDIFF   50         // Abweichung in °C vom letzten (sinnvollen) Wert, ab welcher ein Wert als ungueltig betrachtet wird
#define NOTAUSANZ    20         // Anzahl der direkt aufeinanderfolgenden, ungueltigen Werte, bevor NOTAUS erfolgt
#define NOTAUSABW    4000       // Zeit ms NotAus wenn bei laufendem Betrieb keine Temperatur (AD=0) gemeldet wird

#define STDBY_DIFF      8          // Wenn Abweichung kleiner als dieser Wert ist beginnt die StandbyZeit
#define STDBY_SEC       5UL*60     // Bei einer geringen TempAbweichung, nach dieser StandbyZeit (sec) schaltet die Station in den StandbyModus
#define STDBY_OFF_SEC   10UL*60     // nach dieser Zeit (sec) schaltet die Station vom StandbyModus aus
#define STDBY_NORMAL_DIFF  5       // Wenn Temp im StandbyModus um diesen Wert sinkt, wieder auf normalen Modus wechseln

#define DELAY_BEFORE_MEASURE 10 // in ms
#define DELAY_MAIN_LOOP 10
#define DELAY_MILLIS_SCHNELL 5000
#define DELAY_BEFORE_STORE 2000

#define REFRESH_MILLIS_LCD 200
#define REFRESH_MILLIS_PID 100

#define EE_Target 0
byte count = 0;
int diff;
int pwm_value = 0;
double esum = 0;
int target_temperature = TARGET_TEMPERATURE_DEFAULT;
int target_temperature_tmp;
int target_temperature_store;
bool targed_changed = true;
int actual_temperature = 20;                //
//uint32_t last_temperature_1 = 0;
//uint32_t last_temperature_2 = 0;
int  last_temperature = 20;
//uint16_t actual_temperature_anz;
bool ledOn = false;
bool sw_stdby = false;
byte unguelt =  0;                           // Anzahl der aufeinanderfolgenden, ungueltigen Temperaturwerte

unsigned long millis_lcd = 0;               // Zeitpunkt für nächste Display-Aktualisierunf
unsigned long millis_pid = 0;               // Zeitpunkt für nächste Aktualisierung des Reglers
unsigned long millisSetup = 0;              // Zeitpunkt Beginn Loop-Schleife
unsigned long millisSTDBY = 0;              // Zeitpunkt für nächste Standy-By-Modus
unsigned long millisOFF = 0;                // Zeitpunkt bis zum Ausschalten nach Standy-By-Modus
unsigned long millisStore = 0;              // Zeitpunkt bis zum Speichern des Sollwertes (wenn keine Änderung mehr)


#if ANZEIGE == LCD
  char cstr[21];  // wird fuer LCD-Ausgabe benoetigt
  
  void OLED_init(void);                         //Init OLED (2 LINES)
  void OLED_str(char *str);                     //Output string at current cursor position
  void OLED_pos (uint8_t line, uint8_t offset); //set cursor, home position = 1,1
  void OLED_clear(void);                        //clear Display an return Home

  void SpiTransfer(uint16_t shOut);
  void OLED_cmd(uint8_t cmd);
  void OLED_dat(uint8_t data);
  void OLED_init(void);
  void OLED_str(char *str);
  void OLED_pos (uint8_t line, uint8_t offset);
  void OLED_clear(void);
  
#elif ANZEIGE == MAX7219
  unsigned long delaytime=250;
  LedControl lc=LedControl(MOSI_Pin, CLK_Pin, SS_Pin, 4); //int dataPin, int clkPin, int csPin, int numDevices)
  void printStr(char *str);
  void printNumber(int v);
  void writeArduinoOn7Segment(void);

#elif ANZEIGE == MAX6958
#define I2C_SLAVE_ADDRESS 0x38 // the 7-bit address
byte d[] = { 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0x77, 0x1F, 0x4E, 0x3D, 0x4F, 0x47 };
byte dp[] ={0b00010000, 0b00100000, 0b01000000};  // bit4-Digit0 bit5-Digit1 bit6-Digit2 bit7-Digit3

void set_digit(byte digit, byte value);
void set_dp(byte value);
void set_intensity(byte value);
void printNumber(int v);

#endif

void initDisplay()
{
    
#if ANZEIGE == LCD
  OLED_init();
  OLED_clear();
  OLED_str(" Hello");
  OLED_pos(2, 2);
  OLED_str("OLED");
  delay(200);
  digitalWrite(PIN_LED, LOW);  // Pin LED auf 0-Signal
  OLED_clear();
  delay(200);

  OLED_pos(1, 4);
  OLED_str("EA W204-XLG");
  OLED_pos(2, 1);
  OLED_str("4x20 characters");
  OLED_pos(3, 1);
  OLED_str("   L\357tstation");
  OLED_pos(3, 1);
  OLED_str(Version);
  delay(1500);
  OLED_clear();

#elif ANZEIGE == MAX7219
/*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,10);
  /* and clear the display */
  lc.clearDisplay(0);
  printStr(VersionSeg);
  delay(1000);
  printNumber(target_temperature);
  lc.setChar(0, 3, ' ' , true);
  delay(1000);
  lc.setIntensity(0,5); // 0-15
#elif ANZEIGE == MAX6958
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.send(0x01); // register decode mode
  TinyWireS.send(0x00); // disable decode mode for all digits
  TinyWireS.send(0x3f); // intensity max
  TinyWireS.send(0x02); // scan limit
  TinyWireS.send(0x03); // normal operation
#endif
}

void displayNotAus(int code)
{
#if ANZEIGE == LCD
  OLED_clear(); 
  if (code == 4) {
    OLED_pos(1, 1);
    OLED_str("STDBY - OFF");
  }
  else {
    // LCD-Anzeige loeschen
    OLED_pos(1, 1);
    OLED_str("NOT-AUS");
    OLED_pos(2, 1);
    OLED_str("Problem E:0");
    sprintf(cstr, "%d", code); OLED_str(cstr);
    OLED_pos(3, 1);
    OLED_str("mit Loetspitze");
  }
#elif ANZEIGE == MAX7219
  lc.clearDisplay(0);
  if (code == 4) {
    lc.setDigit(0,2,0,false);
    lc.setChar(0,1,'F',false);
    lc.setChar(0,0,'F',false);
  }
  else {
    lc.setChar(0,3,'E',true);
    lc.setDigit(0,2,0,false);
    lc.setDigit(0,1,code,false);
  }
#elif ANZEIGE == MAX6958
if (code == 4) {

    set_digit(0, 0); 
    set_digit(1, 0xF); 
    set_digit(2, 0xF);
  }
  else {
    set_digit(0, 0xE); 
    set_digit(1, 0); 
    set_digit(2, code);
    set_dp(dp[0]); 
  }
#endif
}

void lcd_display()
{
#if ANZEIGE == LCD
  //OLED_clear();
  count++;
  OLED_pos(1, 1);
  OLED_str("Ist :    ");
  OLED_dat(0xD2);
  OLED_str("C");
  OLED_pos(1, 7);
  sprintf(cstr, "%d", actual_temperature); OLED_str(cstr);

  OLED_pos(1, 13);
  OLED_str("u:    ");
  OLED_pos(1, 16);
  sprintf(cstr, "%d", unguelt); OLED_str(cstr);

//  OLED_pos(1, 13);
//  sprintf(cstr, "%d", count); OLED_str(cstr);
//  OLED_str("   ");

  OLED_pos(2, 1);
  OLED_str("Soll:    ");
  OLED_dat(0xD2);
  OLED_str("C             ");
  OLED_pos(2, 7);
  sprintf(cstr, "%d", target_temperature_tmp); OLED_str(cstr);

  OLED_pos(2, 13);
  if (sw_stdby)
    OLED_str("Standby");
  else
    OLED_str("       ");

  OLED_pos(3, 1);
  OLED_str("Diff:    ");
  OLED_dat(0xD2);
  OLED_str("C");
  OLED_pos(3, 7);
  sprintf(cstr, "%d", diff); OLED_str(cstr);

  OLED_pos(3, 13);
  OLED_str("pwm:    ");
  OLED_pos(3, 17);
  sprintf(cstr, "%d", pwm_value); OLED_str(cstr);
#elif ANZEIGE == MAX7219
  if (targed_changed) {
    printNumber(target_temperature);
    lc.setChar(0, 3, ' ' , true);
    lc.setIntensity(0,15); // 0-15
  }
  else {
    printNumber(actual_temperature);
    lc.setIntensity(0,5);
  }
#elif ANZEIGE == MAX6958
 
      
  if (targed_changed) {
    printNumber(target_temperature);
    set_intensity(63); 
  }
  else {
    printNumber(actual_temperature);
    set_intensity(20);
  }
  if (ledOn)
    set_dp(dp[2]);
  else
    set_dp(0);
   
#endif
}

#if ANZEIGE == LCD
/********************
   void SpiTransfer(uint16_t shOut)
   Software SPI
   Transfer 10 bit with Bit banging
 ********************/
void SpiTransfer(uint16_t shOut)
{
  uint8_t dat = 0;

  digitalWrite (SS_Pin, LOW);
  for (uint8_t i = 0; i < 10; i++)
  {
    digitalWrite (MOSI_Pin, ( (shOut & 0x200) ? 1 : 0)); // condition? Expression1 : Expression2 // if(condition) Expression1 else Expression2

    delayMicroseconds(1);   //warten damit Datensignale korrekt anliegen
    digitalWrite (CLK_Pin, 1);
    delayMicroseconds(1);  //steigen Flanke einlesen
    digitalWrite (CLK_Pin, 0);
    delayMicroseconds(1); //warten fÃ¼r nÃ¤chsten Clockpuls

    shOut <<= 1;
  }
  digitalWrite (SS_Pin, HIGH);
}

/********************
   void OLED_cmd(uint8_t cmd)
   Command to display
 ********************/
void OLED_cmd(uint8_t cmd)
{
  SpiTransfer((uint16_t)cmd);
}

/********************
   void OLED_dat(uint8_t data)
   Data to display
 ********************/
void OLED_dat(uint8_t data)
{
  SpiTransfer(0x0200 + (uint16_t)data);
}

/********************
   void OLED_init(void)
   Initialize ELECTRONIC ASSEMBLY OLED Textseries EA Wxxx
 ********************/
void OLED_init(void)
{
  pinMode (SS_Pin, OUTPUT);
  pinMode (CLK_Pin, OUTPUT);
  pinMode (MOSI_Pin, OUTPUT);

  digitalWrite (SS_Pin, 1);
  //SPI.begin();
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setDataMode(SPI_MODE3);
  //SPI.setClockDivider(SPI_CLOCK_DIV16);

  OLED_cmd(0x39);  //Function set with font selection (european char set)
  OLED_cmd(0x08);
  OLED_cmd(0x06);
  OLED_cmd(0x17);
  OLED_cmd(0x01);
  delay(2);
  OLED_cmd(0x02);
  delay(2);
  //OLED_cmd(0x0F);  //Display on, cursor on, blink on
  OLED_cmd(0x0C);  //Display on, cursor on, blink on
}

/********************
   void OLED_str(char *str)
   Show string at current cursor position
   The string needs to have a 0x00 at the end
 ********************/
void OLED_str(char *str)
{
  while (*str)
    OLED_dat(*str++);
}

/********************
   void OLED_pos (uint8_t line, uint8_t offset)
   Set cursor position
   Home Position: line=1, offset=1
 ********************/
void OLED_pos (uint8_t line, uint8_t offset)
{
  uint8_t pos = 0;
  if (line == 2)
    pos = 0x40;
  else if (line == 3)
    pos = 0x14;
  else if (line == 4)
    pos = 0x54;

  pos += offset - 1;

  OLED_cmd(0x80 + pos);
}

/********************
   void OLED_clear(void)
   Clear dislpay
   Clears whole display and sets cursor to home postion
 ********************/
void OLED_clear(void)
{
  OLED_cmd(0x01);
  delay(2);
  OLED_cmd(0x02);
  delay(2);
}


#elif ANZEIGE == MAX7219

void printStr(char *str)
{
  int seg = 3;
  bool dp = false;
  while (*str)
  {
    if (str[0] == '.')
      str++;
    if (str[1] == '.')
       lc.setChar(0,seg,*str++,true);
     else
      lc.setChar(0,seg,*str++,false);
    seg--;
  }
}

void printNumber(int v) {
    int ones;
    int tens;
    int hundreds;
    boolean negative = false;  

    if(v < -999 || v > 999) 
       return;
    if(v<0) {
        negative=true;
        v=v*-1;
    }
    ones=v%10;
    v=v/10;
    tens=v%10;
    v=v/10;
    hundreds=v;     
    if(negative) {
       //print character '-' in the leftmost column 
       lc.setChar(0,3,'-',false); //(int addr, int digit, char value, boolean dp)
    }
    else {
       //print a blank in the sign column
       lc.setChar(0,3,' ',false);
    }
    //Now print the number digit by digit
    lc.setDigit(0,2,(byte)hundreds,false);
    lc.setDigit(0,1,(byte)tens,false);
    lc.setDigit(0,0,(byte)ones,ledOn);
}

void writeArduinoOn7Segment() {
  lc.setChar(0,0,'a',false);
  delay(delaytime);
  lc.setRow(0,0,0x05);
  delay(delaytime);
  lc.setChar(0,0,'d',false);
  delay(delaytime);
  lc.setRow(0,0,0x1c);
  delay(delaytime);
  lc.setRow(0,0,B00010000);
  delay(delaytime);
  lc.setRow(0,0,0x15);
  delay(delaytime);
  lc.setRow(0,0,0x1D);
  delay(delaytime);
  lc.clearDisplay(0);
  delay(delaytime);
}
#elif ANZEIGE == MAX6958
void set_digit(byte digit, byte value) {
  TinyWireS.begin(I2C_SLAVE_ADDRESS); // address the MAX6959
  TinyWireS.send(0x20 + digit); // register 0x20 is digit 0, 0x21 is digit 1, etc.
  TinyWireS.send(d[value]); // write the value from the array to the register.
}

void set_dp(byte value) {
  TinyWireS.begin(I2C_SLAVE_ADDRESS); // address the MAX6959
  TinyWireS.send(0x24); // register Segments
  TinyWireS.send(value<<4); // bit4-Digit0 bit5-Digit1 bit6-Digit2 bit7-Digit3
}

void set_intensity(byte value) {  
  TinyWireS.begin(I2C_SLAVE_ADDRESS); // address the MAX6959
  TinyWireS.send(0x2); // register intensity
  TinyWireS.send(value); // 0-0x3F (63)
}

void printNumber(int v) {
    int ones;
    int tens;
    int hundreds;

    ones=v%10;
    v=v/10;
    tens=v%10;
    v=v/10;
    hundreds=v;     
    //Now print the number digit by digit
    set_digit(0,(byte)hundreds);
    set_digit(1,(byte)tens);
    set_digit(2,(byte)ones);
    
   

}
#endif

