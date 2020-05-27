#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;// can1 port
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;// can2 port

#include <FaBoLCD_PCF8574.h>                   // include the LCD library
FaBoLCD_PCF8574 lcd;

static CAN_message_t txmsg;
static CAN_message_t rxmsg1;
static CAN_message_t rxmsg2;
String CANStr("");
unsigned char data[6];                         // rxmsg2 size
byte CAN_COUNT = 0;                            // Counter for CAN data LED blink
byte LED_COUNT = 0;                            // Counter for EVSE green LED blink
int led = 13;
int SOC = 25;                                  // IAA SOC gauge PWM 25 to 188 (half 118)
int DTE = 120;                                 // IAA DTE gauge 31-210Hz (half 120Hz)
int MOT_ON = 0;                                // IAA Motor ON gauge 0/1 (Inverter CAN mode off, run)
int ECO = 118;                                 // IAA ECO gauge PWM 25 to 188, mid 118 (Only active in run)
int LO_FAN = 0;                                // IAA Radiator LO fan relay

byte CHG_L = 0;                                // IAA Charge lamp
byte HAZ_L = 0;                                // IAA Hazard lamp
byte PWR_LIM_L = 0;                            // IAA Power limit lamp
byte MIL_L = 0;                                // IAA MIL lamp
byte LO_SOC_L = 0;                             // IAA Low SOC lamp (Only active in run)
byte LO_OIL_L = 0;                             // IAA Low oil lamp

int VOLT = 0;                                  // BMS pack volt
int AMP = 0;                                   // BMS current sensor
int CELL_TMP = 0;                              // BMS average cell temp
int STORED_CELL_TMP = 0;                       // Stored average cell temp
int MIN_CELL_TMP = 0;                          // BMS lowest cell temp
int MAX_CELL_TMP = 0;                          // BMS highest cell temp
int CELL_TMP_COUNT = 200;                      // Time between cell temp reading
int CELL_TMP_CHANGE = 0;                       // Cell temp over time change ( / 100 for C)
int LO_CELL = 0;                               // BMS lowest cell
int HI_CELL = 0;                               // BMS highest cell
byte bmsstatus = 0;                            // BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6

int AUX_V = 0;                                 // Inverter CAN AUX volt (12V battery)
int tmphs = 0;                                 // Inverter CAN heatsink temp neg/pos
int tmpm = 0;                                  // Inverter CAN motor temp neg/pos
byte DIR = 0;                                  // Inverter CAN direction/gear
int RPM = 0;                                   // Inverter CAN RPM

int OILPR = 0;                                 // Oil pressure sensor
//int CAB = 0;                               // CAB CAN main DC current in mA
signed long CAB = 0;                           // CAB CAN main DC current in mA
int TEC_V0;                                    // Voltage for NTC sensor cold (bottom plate)
int TEC_V1;                                    // Voltage for NTC NTC sensor hot (top plate)
int OUT_V2;                                    // Voltage for NTC sensor outdoor
int IN_V3;                                     // Voltage for NTC sensor indoor
int TEC_pwm = 0;                               // Check if PWM is 1kHz!!
float TEC_R100k = 100000;                      // 3.3V-100k NTC-sig-100k-GND
float TEC_logR2, TEC_R2, TEC_T0, TEC_T1, OUT_T2, IN_T3;
float TEC_A = 0.8158589285e-03, TEC_B = 2.076952932e-04, TEC_C = 0.9631468832e-07; // NTCLE203E3104GB0 -  Thermistor, NTC, 100 kohm, NTCLE Series, 4190 K
int TEC_Watt = 0;                              // TEC estimated consumption (add pump and 12v charger)
byte MAIN_PUMP = 0;                            // Main coolant waterpump relay (in AC only bat heat/cool)
byte BAT_PUMP = 0;                             // BAT coolant pump
byte AC_TEC = 0;                               // TEC PSU 230V relay
byte AC_OUT = 1;                               // AC out relay, PTC heater

long Watt = 0;                                 // Calculated Watt neg/pos
int kmh = 0;                                   // Calculated km/h
int Whkm = 0;                                  // Calculated current Wh/km (neg/pos)
int Whkm_trip = 0;                             // Calculated trip Wh/km (neg/pos)
long Whkm_count = 0;                           // Counter to calculate trip Wh/km (neg/pos)
long loop_count = 0;                           // Counter for CAN read loops

byte buttonCount = 0;                          // counter for number of button presses
byte buttonState = 0;                          // current state of the button
byte lastButtonState = 1;                      // previous state of the button
unsigned long lastDebounceTime = 0;            // the last time the button was pressed
unsigned long debounceDelay = 50;              // the debounce time

byte CHG_phase = 0;                            // CHG 1phase 0, 3phase 1
byte EVSE_prox = 0;                            // CHG EVSE prox status
byte CAN_debug = 0;                            // CAN print CANStr
byte DISPLAY_debug = 1;                        // Button press simulation

String espString = 0;                          // ESP serial data input

void setup()
{
  Serial2.begin(115200);
  can1.begin();
  can1.setBaudRate(500000);                    // 500kbps data rate
  can2.begin();
  can2.setBaudRate(500000);                    // 500kbps data rate
  // Must set AIN as input, on Teensy 4.0 to disable internal pullup!
  //  pinMode(7, INPUT);                           // TEST RX2
  //  pinMode(8, INPUT);                           // TEST TX2
  pinMode(A0, INPUT);                          // Oil press sensor
  pinMode(A1, INPUT);                          // Indoor temp
  pinMode(A10, INPUT);                         // TEC outside temp
  pinMode(A12, INPUT);                         // TEC cold temp
  pinMode(A13, INPUT);                         // TEC hot temp
  pinMode(32, INPUT_PULLUP);                   // menu button Z/GND (20-50k pullup)
  pinMode(17, OUTPUT);                         // AC 230V relay (PTC cabin preheat)
  pinMode(20, OUTPUT);                         // TEC PSU 230V relay
  pinMode(10, OUTPUT);                         // TEC PSU heat/cool relay (switch neg/pos)
  pinMode(11, OUTPUT);                         // TEC PSU DC/AC relay
  pinMode(16, OUTPUT);                         // BAT coolant pump
  pinMode(12, OUTPUT);                         // Main coolant pump relay (in AC only bat heat/cool)
  pinMode(6, OUTPUT);                          // IEC 62196 Green status LED
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  digitalWrite(6, HIGH);                       // IEC 62196 Green status LED
  digitalWrite(17, HIGH);                      // AC ON/OFF relay for PTC preheat

  lcd.begin(16, 2);                            // set up columns and rows
  lcd.print(" CanLcd ver: 3.0");               // send version to the LCD.
  lcd.setCursor(0, 1);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  lcd.print((char) 255);
  delay(200);
  Serial.println(F("InCar BMS INV IAA CAN-Tx/Rx Teensy 4.0 150Mhz ver.30"));
  delay(1000);
  lcd.clear();
  digitalWrite(led, LOW);
  digitalWrite(6, LOW);                        // IEC 62196 Green status LED
}

void loop()
{
  int BUT_READ = digitalRead(32);              // Read pushbutton state
  if (BUT_READ != lastButtonState)             // If button pressed
    lastDebounceTime = millis();               // reset the debouncing timer
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (BUT_READ != buttonState)               // if the button state has changed
    {
      buttonState = BUT_READ;
      if (buttonState == LOW)                  // if the state has changed
        buttonCount++;                         // Increment the counter
    }
  }
  lastButtonState = BUT_READ;                  // save state as last state, for next loop
  if (buttonCount > 7)                         // Last screen
    buttonCount = 0;                           // Back to first screen

  if (buttonState == LOW)                      // if the state has changed, show sreen names
  {
    if (buttonCount == 0)                      // Screen "CHARGE"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("CHARGE");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("      ");
    }
    if (buttonCount == 1)                      // Screen "TRIP"
    {
      lcd.clear();
      lcd.setCursor(6, 0);
      lcd.print("TRIP");
      delay(300);
      lcd.setCursor(6, 0);
      lcd.print("    ");
    }
    if (buttonCount == 2)                      // Screen "Wh/km"
    {
      lcd.clear();
      lcd.setCursor(6, 0);
      lcd.print("Wh/km");
      delay(300);
      lcd.setCursor(6, 0);
      lcd.print("     ");
    }
    if (buttonCount == 3)                      // Screen "DRIVE"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("DRIVE");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("     ");
    }
    if (buttonCount == 4)                      // Screen "BATTERY"
    {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("BATTERY");
      delay(300);
      lcd.setCursor(4, 0);
      lcd.print("       ");
    }
    if (buttonCount == 5)                      // Screen "CLIMATE"
    {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("CLIMATE");
      delay(300);
      lcd.setCursor(4, 0);
      lcd.print("       ");
    }
    if (buttonCount == 6)                      // Screen "MOTOR"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("MOTOR");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("     ");
    }
    if (buttonCount == 7)                      // Screen "BAT TEC"
    {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("BAT TEC");
      delay(300);
      lcd.setCursor(4, 0);
      lcd.print("       ");
    }
  }

  // Read CAN bus 1, Charger and IAA instrument cluster uC
  while (can1.read(rxmsg1))
  {
    String CANStr("");
    for (int i = 0; i < 8; i++)
    {
      CANStr += String(rxmsg1.buf[i], HEX);
      CANStr += (" ") ;
    }

    // CAN Tesla Charger
    if (rxmsg1.id == 0x3D8)
    {
      data[0] = rxmsg1.buf[0];                 // Test DEBUG
      data[1] = rxmsg1.buf[1];                 // Test DEBUG
      data[2] = rxmsg1.buf[2];                 // Test DEBUG
      data[3] = rxmsg1.buf[3];                 // Test DEBUG
      CHG_phase = rxmsg1.buf[4];               // parameters.phaseconfig 1phase 0, 3phase 1
      EVSE_prox  = rxmsg1.buf[5];                 // Proximity Unconnected 0, Buttonpress 1, Connected 2. Test DEBUG
      data[6] = rxmsg1.buf[6];                 // parameters.canControl Test DEBUG
      data[7] = rxmsg1.buf[7];                 // Count to 255 Test DEBUG
      if (CAN_debug == 1)
      {
        Serial.print("CHARGER: ");
        Serial.print(rxmsg1.id, HEX);
        Serial.print(' ');
        Serial.print(rxmsg1.len, HEX);
        Serial.print(' ');
        Serial.print(CANStr);
        Serial.println ();
      }
    }
  }

  // Read CAN bus 1, BMS, CAB sensor and inverter
  while (can2.read(rxmsg2))
  {
    String CANStr("");
    for (int i = 0; i < 8; i++)
    {
      CANStr += String(rxmsg2.buf[i], HEX);
      CANStr += (" ") ;
    }

    char buf_LCD[16];

    // CAN BMS High/low cell volt and temp
    if (rxmsg2.id == 0x373)
    {
      data[0] = rxmsg2.buf[0];                 // Min Cell Voltage LSB
      data[1] = rxmsg2.buf[1];                 // Min Cell  Voltage MSB
      data[2] = rxmsg2.buf[2];                 // Max Cell Voltage LSB
      data[3] = rxmsg2.buf[3];                 // Max Cell  Voltage MSB
      data[4] = rxmsg2.buf[4];                 // Min Temperature LSB
      data[5] = rxmsg2.buf[5];                 // Min Temperature MSB
      data[6] = rxmsg2.buf[6];                 // Max Temperature LSB
      data[7] = rxmsg2.buf[7];                 // Max Temperature MSB
      if (CAN_debug == 1)
      {
        Serial.print("BMS HI/LO CELL: ");
        Serial.print(rxmsg2.id, HEX);
        Serial.print(' ');
        Serial.print(rxmsg2.len, HEX);
        Serial.print(' ');
        Serial.print(CANStr);
        Serial.println ();
      }
      LO_CELL = (uint8_t)data[1] << 8;         // read byte 1 << bitshift left
      LO_CELL |= data[0];                      // read byte 0
      HI_CELL = (uint8_t)data[3] << 8;         // read byte 1 << bitshift left
      HI_CELL |= data[2];                      // read byte 0
      MIN_CELL_TMP = (uint8_t)data[5] << 8;    // read byte 5 << bitshift left
      MIN_CELL_TMP |= data[4];                 // read byte 4
      MAX_CELL_TMP = (uint8_t)data[7] << 8;    // read byte 7 << bitshift left
      MAX_CELL_TMP |= data[6];                 // read byte 6
    }

    // CAN current sensor 0x3C2 DEBUG TEST
    if (rxmsg2.id == 0x3c2)
    {
      data[0] = rxmsg2.buf[0];                 //
      data[1] = rxmsg2.buf[1];                 //
      data[2] = rxmsg2.buf[2];                 //
      data[3] = rxmsg2.buf[3];                 //
      data[4] = rxmsg2.buf[4];                 //
      data[5] = rxmsg2.buf[5];                 //
      data[6] = rxmsg2.buf[6];                 //
      data[7] = rxmsg2.buf[7];                 //
      if (CAN_debug == 1)
      {
        Serial.print("CAB AMP: ");
        Serial.print(rxmsg2.id, HEX);
        Serial.print(' ');
        Serial.print(rxmsg2.len, HEX);
        Serial.print(' ');
        Serial.print(CANStr);
        Serial.println ();
      }
      CAB = (uint8_t)data[0] << 24;            // read byte 0 << bitshift left
      CAB |= data[1] << 16;                    // read byte 1 << bitshift left
      CAB |= data[2] << 8;                     // read byte 2 << bitshift left
      CAB |= data[3];                          // read byte 3
      CAB -= 0x80000000UL;
      CAB = CAB / 100;                         // 0.0A precision
      Watt = CAB * (VOLT / 10) / 100;
    }

    // CAN Inverter
    if (rxmsg2.id == 0x3FF)
    {
      tmphs = rxmsg2.buf[0];                   // Heatsink temp
      tmpm = rxmsg2.buf[1];                    // Motor temp
      data[2] = rxmsg2.buf[2];                 // Inverter AUX volt LSB
      data[3] = rxmsg2.buf[3];                 // Inverter AUX volt MSB
      DIR = rxmsg2.buf[4];                     // Motor Rew Fwd P/N
      MOT_ON = rxmsg2.buf[5];                  // Motor mode Off Run
      data[6] = rxmsg2.buf[6];                 // Motor RPM kmh LSB
      data[7] = rxmsg2.buf[7];                 // Motor RPM kmh MSB
      if (CAN_debug == 1)
      {
        Serial.print("INVERTER: ");
        Serial.print(rxmsg2.id, HEX);
        Serial.print(' ');
        Serial.print(rxmsg2.len, HEX);
        Serial.print(' ');
        Serial.print(CANStr);
        Serial.println ();
      }
      RPM = (uint8_t)data[7] << 8;             // read byte 7 << bitshift left
      RPM |= data[6];                          // read byte 6
      RPM = 5000;                              // DEBUG TEST
      kmh = (RPM / 9.57);                      // Max RPM 11500 /95.7 = 120km/h

      if (MOT_ON == 1)
      {
        OILPR = analogRead(A0);                // read analog oil pressure sensor
        OILPR = map(OILPR, 108, 918, 0, 100);
        if (OILPR < 5)
          LO_OIL_L = 1;                        // Low oil lamp
        else LO_OIL_L = 0;
      }
    }

    // CAN BMS errors
    if (rxmsg2.id == 0x35A)
    {
      data[0] = rxmsg2.buf[0];                 // bit3 Undervoltage, bit4 Overvoltage
      data[1] = rxmsg2.buf[1];                 // bit7 Overtemp
      data[2] = rxmsg2.buf[2];                 // bit12 Undertemp
      data[7] = rxmsg2.buf[7];                 // bit1 Cell delta volt (default >200)
      if (CAN_debug == 1)
      {
        Serial.print("BMS ERROR: ");
        Serial.print(rxmsg2.id, HEX);
        Serial.print(' ');
        Serial.print(rxmsg2.len, HEX);
        Serial.print(' ');
        Serial.print(CANStr);
        Serial.println ();
      }

      if (data[0] > 1)                         // Overvoltage/Undervoltage
      {
        if (data[0] == 0x4)                    // bit4 Overvoltage
          HAZ_L = 1;
        if (data[0] == 0x10)                   // bit3 Undervoltage
          HAZ_L = 1;
      }
      else HAZ_L = 0;
    }

    // CAN BMS SOC, bmsstatus
    if (rxmsg2.id == 0x355)
    {
      SOC = rxmsg2.buf[0];                     // SOC Range is 0-100 no scaling
      bmsstatus = rxmsg2.buf[2];               // BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6
      if (CAN_debug == 1)
      {
        Serial.print("BMS SOC, MODE: ");
        Serial.print(rxmsg2.id, HEX);
        Serial.print(' ');
        Serial.print(rxmsg2.len, HEX);
        Serial.print(' ');
        Serial.print(CANStr);
        Serial.println ();
      }
    }

    // BMS Pack volt, Current, cell temp.
    if (rxmsg2.id == 0x356)
    {
      data[0] = rxmsg2.buf[0];                 // VOLT LSB
      data[1] = rxmsg2.buf[1];                 // VOLT MSB
      data[2] = rxmsg2.buf[2];                 // AMP LSB
      data[3] = rxmsg2.buf[3];                 // AMP MSB
      data[4] = rxmsg2.buf[4];                 // AVG CELL TEMP LSB
      data[5] = rxmsg2.buf[5];                 // AVG CELL TEMP MSB
      if (CAN_debug == 1)
      {
        Serial.print("BMS V, A, TMP: ");
        Serial.print(rxmsg2.id, HEX);
        Serial.print(' ');
        Serial.print(rxmsg2.len, HEX);
        Serial.print(' ');
        Serial.print(CANStr);
        Serial.println ();
      }
      VOLT = (uint8_t)data[1] << 8;            // read byte 1 << bitshift left
      VOLT |= data[0];                         // read byte 0
      CELL_TMP = (uint8_t)data[5] << 8;        // read byte 5 << bitshift left
      CELL_TMP |= data[4];                     // read byte 4

      // Send CAN data for every receive of BMS CAN id 0x356

      // Send CAN data to IAA, lamps
      txmsg.len = 8;
      txmsg.id = 0x700;
      txmsg.buf[0] = 0;                        // Charge lamp
      txmsg.buf[1] = HAZ_L;                    // Hazard lamp
      txmsg.buf[2] = 0;                        // Power limit lamp
      txmsg.buf[3] = 0;                        // MIL lamp
      txmsg.buf[4] = 0;                        // Low SOC lamp (Only active in run)
      txmsg.buf[5] = LO_OIL_L;                 // Low oil lamp
      txmsg.buf[6] = 0;
      txmsg.buf[7] = 0;
      if (CAN_debug == 1)
      {
        Serial.print("IAA LAMPS: ");
        Serial.print(txmsg.id, HEX);
        Serial.print(' ');
        Serial.print(txmsg.len, HEX);
        Serial.print(' ');
        Serial.print(txmsg.buf[0]);
        Serial.print(' ');
        Serial.print(txmsg.buf[1]);
        Serial.print(' ');
        Serial.print(txmsg.buf[2]);
        Serial.print(' ');
        Serial.print(txmsg.buf[3]);
        Serial.print(' ');
        Serial.print(txmsg.buf[4]);
        Serial.print(' ');
        Serial.println(txmsg.buf[5]);
      }
      can1.write(txmsg);                       // Transmit first CAN data

      // Send CAN data to IAA, gauges, radiator fan relay
      txmsg.len = 8;
      txmsg.id = 0x701;
      txmsg.buf[0] = SOC;                      // SOC gauge PWM 25 to 188 (half 118)
      txmsg.buf[1] = DTE;                      // DTE gauge 31-210Hz (half 120Hz)
      txmsg.buf[2] = MOT_ON;                   // Motor ON gauge 0/1
      txmsg.buf[3] = ECO;                      // ECO gauge PWM 25 to 188 (Only active in run)
      txmsg.buf[4] = 0;                        // No cell temp to instrument cluster!
      txmsg.buf[5] = LO_FAN;                   // Radiator LO fan relay
      txmsg.buf[6] = 0;
      txmsg.buf[7] = 0;
      if (CAN_debug == 1)
      {
        Serial.print("IAA GAUGES: ");
        Serial.print(txmsg.id, HEX);
        Serial.print(' ');
        Serial.print(txmsg.len, HEX);
        Serial.print(' ');
        Serial.print(txmsg.buf[0], HEX);
        Serial.print(' ');
        Serial.print(txmsg.buf[1], HEX);
        Serial.print(' ');
        Serial.print(txmsg.buf[2], HEX);
        Serial.print(' ');
        Serial.print(txmsg.buf[3], HEX);
        Serial.print(' ');
        Serial.print(txmsg.buf[4], HEX);
        Serial.print(' ');
        Serial.println(txmsg.buf[5], HEX);
      }
      can1.write(txmsg);                       //Transmit next CAN data

      // Read outdoor/indoor temp for every receive of BMS CAN id 0x356
      OUT_V2 = analogRead(A10);                // NTC sensor outdoor
      TEC_R2 = TEC_R100k * (1023.0 / (float)OUT_V2 - 1.0);
      OUT_T2 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
      OUT_T2 = OUT_T2 - 273.15;                // Convert to C

      IN_V3 = analogRead(A1);              // TEC NTC sensor indoor
      TEC_R2 = TEC_R100k * (1023.0 / (float)IN_V3 - 1.0);
      IN_T3 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
      IN_T3 = IN_T3 - 273.15;              // Convert to C

      // Temp control TEC heat/cool for every receive of BMS CAN id 0x356

      if (CELL_TMP_COUNT > 150)              // Avoid STORED_CELL_TMP zero at startup
      {
        STORED_CELL_TMP = CELL_TMP;
        CELL_TMP_COUNT = 0;
      }
      CELL_TMP_COUNT++;                        // Increment
      if (CELL_TMP_COUNT > 110)
      {
        CELL_TMP_CHANGE = CELL_TMP - STORED_CELL_TMP; //Cell temp change between CELL_TMP_COUNT
        STORED_CELL_TMP = CELL_TMP;
        CELL_TMP_COUNT = 0;
      }

      if ((TEC_T1 > 50 || TEC_T0 < -10) || (CELL_TMP > 2500 && TEC_pwm == 20))        // Turn off if PWM is minimum and temp to high/low
      {
        TEC_pwm = 0;
        MAIN_PUMP = 0;                         // Main coolant pump relay (in AC only bat heat/cool)
        digitalWrite(12, LOW);                 // Main coolant pump relay (in AC only bat heat/cool)
        BAT_PUMP = 0;                          // BAT coolant pump
        digitalWrite(16, LOW);                 // BAT coolant pump

      }
      if (MAIN_PUMP == 1 && CELL_TMP < 2500)   // Don't heat if over 25C
      {
        digitalWrite(12, HIGH);                // Main coolant waterpump relay (in AC only bat heat/cool)
        digitalWrite(16, HIGH);                // BAT coolant pump

        TEC_V0 = analogRead(A12);              // TEC NTC sensor cold (bottom plate)
        TEC_R2 = TEC_R100k * (1023.0 / (float)TEC_V0 - 1.0);
        TEC_T0 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
        TEC_T0 = TEC_T0 - 273.15;              // Convert to C

        TEC_V1 = analogRead(A13);              // TEC NTC sensor hot (top plate)
        TEC_R2 = TEC_R100k * (1023.0 / (float)TEC_V1 - 1.0);
        TEC_T1 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
        TEC_T1 = TEC_T1 - 273.15;              // Convert to C


        if (CELL_TMP_COUNT == 100 && TEC_T1 < 40)     // Change power for every CELL_TMP_COUNT, and TEC temp under max
        {
          if (CELL_TMP <= 1800)                // Increase power
            TEC_pwm = (TEC_pwm + 1);
          //          if (CELL_TMP >= 1820)                // Decrease power
          if (CELL_TMP > 1819)                // Decrease power BUG fix!!!???
            TEC_pwm = (TEC_pwm - 1);

          if (CELL_TMP > 1800 && CELL_TMP < 1820)  //
          {
            if (CELL_TMP_CHANGE > 0)           // CELL TMP GOING UP
              TEC_pwm = (TEC_pwm - 1);
            if (CELL_TMP_CHANGE < 0)           // CELL TMP GOING DOWN
              TEC_pwm = (TEC_pwm + 1);
          }
        }

        TEC_pwm = constrain (TEC_pwm, 0, 150);// Min Pos duty for PSU is 10% (20)
        analogWrite (3, TEC_pwm);
      }

      /////////////////////////////////////////////////////////////////////////////

      if (buttonCount < 2)                  // Screen "CHARGE or TRIP"
      {
        lcd.setCursor(0, 0);
        lcd.print (SOC);
        lcd.print ("%");
      }

      if (buttonCount == 0)                  // Screen "CHARGE"
      {
        CAB = CAB / 100;                     // 0.0A precision
        Watt = CAB * (VOLT / 10) / 100;
        lcd.setCursor(5, 0);
        lcd.print(Watt * 0.001, 1);          // 0.0kW precision
        lcd.print("kW");
        lcd.setCursor(13, 0);
        if (CHG_phase == 0)
          lcd.print("1ph");
        if (CHG_phase == 1)
          lcd.print("3ph");
      }

      if (buttonCount == 1)                  // Screen "TRIP"
      {
        lcd.setCursor(0, 0);
        lcd.print (SOC);
        lcd.print ("%");
      }

      if (buttonCount < 2)                   // Screen "CHARGE or TRIP"
      {
        // SOC bar
        byte SOC_bar_count = 0;              // Soc bar counter (0-16)
        byte SOC_bar = map(SOC, 1, 100, 0, 16); // Tested 1 block is 8%, 15 blocks is 94%
        lcd.setCursor(0, 1);
        for (SOC_bar_count = 0; SOC_bar > SOC_bar_count; SOC_bar_count++)   // count from 0 up to SOC_bar
        {
          lcd.print((char) 255);             // print block
        }
        for (SOC_bar_count = 16; SOC_bar < SOC_bar_count; SOC_bar_count--)  // count from 15 down to SOC_bar
        {
          lcd.print(" ");                    // print space (to clear old value)
        }
      }

      if (buttonCount == 3)                  // Screen "DRIVE"
      {
        CAB = CAB / 100;                     // 0.0A precision
        Watt = CAB * (VOLT / 10) / 100;
        lcd.setCursor(0, 0);
        lcd.print(Watt * 0.001, 1);          // 0.0kW precision
        lcd.print("kW");
        lcd.setCursor(10, 0);
        lcd.print(kmh * 0.1, 1);
        lcd.print("kmh");
        // PWR bar
        if (CAB < 0)
          Watt = Watt * -1;
        byte PWR_BAR_count = 0;              // PWR bar counter (0-16)
        byte PWR_BAR = map(Watt, 1000, 50000, 0, 16); // 1kW to 50kW
        lcd.setCursor(0, 1);
        for (PWR_BAR_count = 0; PWR_BAR > PWR_BAR_count; PWR_BAR_count++)   // count from 0 up to PWR_BAR
        {
          if (CAB < 0)
            lcd.print((char) 219);           // print block
          else
            lcd.print((char) 255);           // print block
        }
        for (PWR_BAR_count = 8; PWR_BAR < PWR_BAR_count; PWR_BAR_count--)  // count from 15 down to PWR_BAR
        {
          lcd.print(" ");                    // print space (to clear old value)
        }
      }

      if (buttonCount == 4)                  // Screen "BATTERY"
      {
        lcd.setCursor(0, 0);
        lcd.print (SOC);
        lcd.print ("%");
        lcd.setCursor(4, 0);
        lcd.print("46kWh");
        lcd.setCursor(11, 0);
        lcd.print (CELL_TMP * 0.01, 1);
        lcd.print("C ");
        lcd.setCursor(0, 1);
        lcd.print("BAL");
        lcd.setCursor(4, 1);
        lcd.print (HI_CELL - LO_CELL);
        lcd.print("mV ");
        lcd.setCursor(11, 1);
        //        lcd.print(AVG_cell, 1);
        lcd.print("3.82V");
      }

      if (buttonCount == 5)                  // Screen "CLIMATE"
      {
        lcd.setCursor(0, 0);
        lcd.print ("Out");
        lcd.setCursor(11, 0);
        lcd.print (OUT_T2, 1);
        lcd.print("C ");
        lcd.setCursor(0, 1);
        lcd.print("In");
        lcd.setCursor(4, 1);
        if (AC_OUT == 1)
          lcd.print("AC ON");
        else
          lcd.print("     ");
        lcd.setCursor(11, 1);
        lcd.print(IN_T3, 1);
        lcd.print("C ");
      }

      if (buttonCount == 6)                  // Screen "MOTOR"
      {
        lcd.setCursor(0, 0);
        if (MOT_ON == 1)
          lcd.print ("ON ");
        else
          lcd.print ("OFF");
        lcd.setCursor(7, 0);
        if (DIR == 0)
          lcd.print("N/P");
        if (DIR == 1)
          lcd.print("D/E");
        if (DIR == 15)
          lcd.print("R  ");
        lcd.setCursor(13, 0);
        lcd.print (tmpm);
        lcd.print("C ");
        lcd.setCursor(0, 1);
        if (MOT_ON == 1)
          lcd.print (OILPR);
        else
          lcd.print ("0.0");
        lcd.print ("psi");
        lcd.setCursor(7, 1);
        lcd.print ("INV");
        lcd.setCursor(13, 1);
        lcd.print(tmphs);
        lcd.print("C ");
      }

      if (buttonCount == 7)                  // Screen "BAT TEC"
      {
        lcd.setCursor(0, 0);
        lcd.print ("Heat AC ");
        lcd.setCursor(11, 0);
        lcd.print (TEC_T1, 1);
        lcd.print("C ");
        lcd.setCursor(0, 1);
        lcd.print (TEC_Watt + 20);           // Add 20W for pump and 12v charger
        lcd.print("W ");
        lcd.setCursor(5, 1);
        lcd.print(CELL_TMP_CHANGE);          // Cell temp going up/down 0.01
        lcd.setCursor(11, 1);
        lcd.print(TEC_T0, 1);
        lcd.print("C ");
      }

      /////////////////////////////////////////////////////////////////////////////

      LED_COUNT++;                           // Counter for EVSE green LED blink
      if (LED_COUNT > 7)                     // Counter for EVSE green LED blink
        LED_COUNT = 0;                       // LED counter reset
      if (bmsstatus == 6)                    // BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6
      {
        if (LED_COUNT == 0)                  // Counter for EVSE green LED blink
          digitalWrite(6, HIGH);
        if (LED_COUNT == 1)                  // Counter for EVSE green LED blink
          digitalWrite(6, LOW);
      }
      if (bmsstatus == 3)                    // BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6
      {
        if (LED_COUNT == 0)                  // Counter for EVSE green LED blink
        {
          digitalWrite(6, HIGH);
          CHG_L = 1;                         // Instrument charge lamp
        }
        if (LED_COUNT == 4)                  // Counter for EVSE green LED blink
        {
          digitalWrite(6, LOW);
          CHG_L = 0;                         // Instrument charge lamp
        }
      }

      if (kmh > 0)                           // do not continue if zero km/t
      {
        Whkm_count = (Whkm + Whkm_count);    // add/subtract to counter (for average Wh/km calculation)
        loop_count++;                        // Increment the counter
      }
      CAN_COUNT++;                           // Counter for CAN data LED blink
      if (CAN_COUNT > 3)
      {
        digitalWrite(led, LOW);
        CAN_COUNT = 0;                       // CAN data counter reset

        // TEST send data to ESP for two receives of BMS CAN id 0x356
        Serial2.print("v:");
        Serial2.print(VOLT / 100);           // DEBUG TEST
        Serial2.print(",s:");
        Serial2.print(SOC);                  // DEBUG TEST
        Serial2.print(",cc:");
        Serial2.print(CELL_TMP_COUNT);
        Serial2.print(",pwm:");
        Serial2.print(TEC_pwm);              // DEBUG TEST
        Serial2.print(",t0:");
        Serial2.print(TEC_T0);               // DEBUG TEST
        Serial2.print(",t1:");
        Serial2.print(TEC_T1);               // DEBUG TEST
        Serial2.print(",t2:");
        Serial2.print(OUT_T2);               // DEBUG TEST
        Serial2.print(",t3:");
        Serial2.print(IN_T3);                // DEBUG TEST
        Serial2.print(",tc:");
        Serial2.print(CELL_TMP * 0.01, 2);
        Serial2.print(",AMP:");
        Serial2.print(CAB * 0.1, 1);         // DEBUG TEST
        Serial2.print(",BMS:");
        if (bmsstatus == 0)
          Serial2.print("Boot");
        if (bmsstatus == 1)
          Serial2.print("Ready");
        if (bmsstatus == 2)
          Serial2.print("Drive");
        if (bmsstatus == 3)
          Serial2.print("Charge");
        if (bmsstatus == 4)
          Serial2.print("Precharge");
        if (bmsstatus == 5)
          Serial2.print("Error");
        if (bmsstatus == 6)
          Serial2.print("BatHC");

        Serial2.print(",CHG:");
        if (CHG_phase == 0)
          Serial2.print("1ph");
        if (CHG_phase == 1)
          Serial2.print("3ph");

        Serial2.println("*");

        while (Serial2.available() > 0)      // ESP serial input
        {
          espString = Serial2.readStringUntil('\n');
          //          Serial.println(espString);
        }
      }
      ////////// Send to serial for two receives of BMS CAN id 0x356////////

      // CAN Tesla Charger
      if (CHG_phase == 0)
        Serial.println("CHG: 1phase");
      if (CHG_phase == 1)
        Serial.println("CHG: 3phase");
      if (EVSE_prox == 2)                    // Proximity Unconnected 0, Buttonpress 1, Connected 2.
        Serial.println("EVSE: Connected");
      if (EVSE_prox == 0)                    // Proximity Unconnected 0, Buttonpress 1, Connected 2.
        Serial.println("EVSE: Unconnected");

      // CAN BMS High/low cell volt and temp
      Serial.print("LO_CELL: ");
      Serial.print(LO_CELL);                 // DEBUG TEST
      Serial.print("  HI_CELL: ");
      Serial.println(HI_CELL);               // DEBUG TEST
      Serial.print("MIN CELL TMP: ");        // DEBUG TEST
      Serial.print(MIN_CELL_TMP);            // DEBUG TEST
      Serial.print("  MAX CELL TMP: ");      // DEBUG TEST
      Serial.println(MAX_CELL_TMP);          // DEBUG TEST

      // CAN current sensor
      Serial.print("mA: ");
      Serial.println(CAB);
      Serial.print("Watt: ");
      Serial.println(Watt);

      // CAN Inverter
      Serial.print("DIR: ");
      if (DIR == 0)
        Serial.println("N/P");
      if (DIR == 1)
        Serial.println("D/E");
      if (DIR == 15)
        Serial.println("R  ");
      if (MOT_ON == 1)                       //  Motor mode Off Run
        Serial.println("Motor: ON");
      if (MOT_ON == 0)                       //  Motor mode Off Run
        Serial.println("Motor: OFF");
      Serial.print("kmh: ");
      Serial.println(kmh);
      Serial.print("Oil press: ");           // 0psi=108 10psi=918
      //Serial.println(OILPR * .0049);       // DEBUG TEST
      Serial.println(OILPR);                 // DEBUG TEST

      // CAN BMS errors
      Serial.print("BMS error: ");
      if (HAZ_L = 1)
        Serial.println("Cell high/low volt!! ");
      else
        Serial.println("No error ");

      // CAN BMS SOC, bmsstatus
      Serial.print("SOC: ");
      Serial.println(SOC);                   // DEBUG TEST
      if (bmsstatus == 0)
        Serial.println("BMS: Boot 0 ");
      if (bmsstatus == 1)
        Serial.println("BMS: Ready 1 ");
      if (bmsstatus == 2)
        Serial.println("BMS: Drive 2 ");
      if (bmsstatus == 3)
        Serial.println("BMS: Charge 3 ");
      if (bmsstatus == 4)
        Serial.println("BMS: Precharge 4 ");
      if (bmsstatus == 5)
        Serial.println("BMS: Error 5 ");
      if (bmsstatus == 6)
        Serial.println("BMS: Bat_HC 6 ");

      // BMS Pack volt, Current, cell temp.
      Serial.print("PACK VOLT: ");
      Serial.println(VOLT);                  // DEBUG TEST
      Serial.print("CELL TMP: ");            // DEBUG TEST
      Serial.print(CELL_TMP);
      Serial.print("  STORED CELL TMP: ");   // DEBUG TEST
      Serial.println(STORED_CELL_TMP);       // DEBUG TEST

      // Read temp
      Serial.print("OUT: ");
      Serial.print(OUT_T2);
      Serial.print("C  ");
      Serial.print("IN: ");
      Serial.print(IN_T3);
      Serial.println("C");
      Serial.print("CELL_TMP_COUNT: ");
      Serial.print(CELL_TMP_COUNT);          // DEBUG TEST
      Serial.print(" TMP_CHANGE: ");         // DEBUG TEST
      Serial.println(CELL_TMP_CHANGE);       // DEBUG TEST
      Serial.print("COLD: ");
      Serial.print(TEC_T0);
      Serial.print("C  ");
      Serial.print("HOT: ");
      Serial.print(TEC_T1);
      Serial.println("C");
      Serial.print("MAIN pump: ");           // TEC PSU relay on/off
      Serial.print(MAIN_PUMP);               // Main coolant waterpump relay (in AC only bat heat/cool)
      Serial.print("  BAT pump: ");          // TEC PSU relay on/off
      Serial.println(BAT_PUMP);
      Serial.print("PWM: ");
      Serial.print(TEC_pwm);
      TEC_Watt = map(TEC_pwm, 0, 150, 2, 400); //
      Serial.print("  TEC Watt: ");
      Serial.println(TEC_Watt + 20);         // Add 20W for pumps and 12v charger (add 200W for radiator fan!!)

      Serial.print("AC relay: ");
      Serial.print (digitalRead(17));        // AC 230V relay (PTC cabin preheat)(AC_OUT)
      Serial.print("  TEC AC relay: ");
      Serial.println (digitalRead(20));      // TEC PSU 230V relay (AC_TEC)

      Serial.print("ButtonCount: ");
      Serial.println(buttonCount);
      ////////////////////////////////////////////
      Serial.println("");

      if (CAN_COUNT == 1)                    //  Counter for CAN data LED blink
        digitalWrite(led, HIGH);

      if (DISPLAY_debug == 1 && CELL_TMP_COUNT == 100)
      {
        buttonCount ++;                      // counter for number of button presses
        lcd.clear();
        lcd.setCursor(5, 0);
        if (buttonCount == 0)                // Screen "CHARGE"
          lcd.print("CHARGE");
        if (buttonCount == 1)                // Screen "TRIP"
          lcd.print("TRIP");
        if (buttonCount == 2)                // Screen "Wh/km"
          lcd.print("Wh/km");
        if (buttonCount == 3)                // Screen "DRIVE"
          lcd.print("DRIVE");
        if (buttonCount == 4)                // Screen "BATTERY"
          lcd.print("BATTERY");
        if (buttonCount == 5)                // Screen "CLIMATE"
          lcd.print("CLIMATE");
        if (buttonCount == 6)                // Screen "MOTOR"
          lcd.print("MOTOR");
        if (buttonCount == 7)                // Screen "BAT TEC"
          lcd.print("BAT TEC");
        delay(300);
        lcd.setCursor(4, 0);
        lcd.print("       ");
      }
    }
  }
}
