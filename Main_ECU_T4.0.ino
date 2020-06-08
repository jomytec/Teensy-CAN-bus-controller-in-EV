#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;// can1 port
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;// can2 port

#include <FaBoLCD_PCF8574.h>                 // include the LCD library
FaBoLCD_PCF8574 lcd;

static CAN_message_t txmsg;
static CAN_message_t rxmsg1;
static CAN_message_t rxmsg2;
String CANStr("");
unsigned char data[6];                       // rxmsg2 size
int CAN_COUNT = 0;                           // Counter for CAN data LED blink
int LED_COUNT = 0;                           // Counter for EVSE green LED blink
int led = 13;
int SOC = 25;                                // IAA SOC gauge PWM 25 to 188 (half 118)
int DTE = 120;                               // IAA DTE gauge 31-210Hz (half 120Hz)
int MOT_ON = 0;                              // IAA Motor ON gauge 0/1 (Inverter CAN mode off, run)
int ECO = 118;                               // IAA ECO gauge PWM 25 to 188, mid 118 (Only active in run)
int LO_FAN = 0;                              // IAA Radiator LO fan relay

int CHG_L = 0;                               // IAA Charge lamp
int HAZ_L = 0;                               // IAA Hazard lamp
int PWR_LIM_L = 0;                           // IAA Power limit lamp
int MIL_L = 0;                               // IAA MIL lamp
int LO_SOC_L = 0;                            // IAA Low SOC lamp (Only active in run)
int LO_OIL_L = 0;                            // IAA Low oil lamp

int VOLT = 0;                                // CAN BMS pack volt
//int AMP = 0;                               // CAN BMS current sensor
int CELL_TMP = 0;                            // CAN BMS average cell temp
int STORED_CELL_TMP = 0;                     // Stored average cell temp
int MIN_CELL_TMP = 0;                        // CAN BMS lowest cell temp
int MAX_CELL_TMP = 0;                        // CAN BMS highest cell temp
int DELTA_CELL_TMP = 0;                      // CAN BMS highest cell temp - lowest cell temp
int CELL_TMP_COUNT = 200;                    // Time between cell temp reading
int CELL_TMP_CHANGE = 0;                     // Cell temp over time change ( / 100 for C)
int LO_CELL = 0;                             // CAN BMS lowest cell volt
int HI_CELL = 0;                             // CAN BMS highest cell volt
int AVG_CELL = 0;                            // CAN BMS average cell volt
int bmsstatus = 0;                           // CAN BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6
int bms_bal = 0;                             // CAN BMS balancing OFF 0, ON 1
int bms_kWh = 0;                             // CAN BMS kWh in battry

int AUX_V = 0;                               // CAN Inverter AUX volt (12V battery)
int tmphs = 0;                               // CAN Inverter heatsink temp neg/pos
int tmpm = 0;                                // CAN Inverter motor temp neg/pos
int DIR = 0;                                 // CAN Inverter direction/gear
int RPM = 0;                                 // CAN Inverter RPM

int OILPR = 0;                               // Oil pressure sensor
int CAB = 0;                                 // CAB CAN main DC current in mA
int TEC_V0;                                  // Voltage for NTC sensor cold (bottom plate)
int TEC_V1;                                  // Voltage for NTC NTC sensor hot (top plate)
int OUT_V2;                                  // Voltage for NTC sensor outdoor
int IN_V3;                                   // Voltage for NTC sensor indoor

float TEC_R100k = 100000;                    // 3.3V-100k NTC-sig-100k-GND
float TEC_logR2, TEC_R2, TEC_T0, TEC_T1, OUT_T2, IN_T3;
float TEC_A = 0.8158589285e-03, TEC_B = 2.076952932e-04, TEC_C = 0.9631468832e-07; // NTCLE203E3104GB0 -  Thermistor, NTC, 100 kohm, NTCLE Series, 4190 K
int TEC_pwm = 0;                             // TEC Check if PWM is 1kHz!!
int TEC_Watt = 0;                            // TEC estimated consumption (add pump and 12v charger)

int Watt = 0;                                // Calculated Watt neg/pos
int kmh = 0;                                 // Calculated km/h (0.0 precision)
int len_trip = 0;                            // Calculated RAW km driven since start
int km_trip = 0;                             // Calculated 0.0 km driven since start
int Whkm = 0;                                // Calculated current Wh/km (neg/pos)
int Whkm_trip = 0;                           // Calculated trip Wh/km (neg/pos)
long Whkm_count = 0;                         // Counter to calculate trip Wh/km (neg/pos)
long loop_count = 0;                         // Counter for CAN read loops
int BAR_count = 0;                           // Counter for SOC, PWR and Whkm BAR
int BAR = 0;                                 // Counter for SOC, PWR and Whkm BAR

int buttonCount = 0;                         // counter for number of button presses
int buttonState = 0;                         // current state of the button
int lastButtonState = 1;                     // previous state of the button
unsigned long lastDebounceTime = 0;          // the last time the button was pressed
unsigned long debounceDelay = 50;            // the debounce time

int CHG_phase = 0;                           // CAN CHG 1phase 0, 3phase 1
int EVSE_prox = 0;                           // CAN CHG EVSE prox status
int CAN_debug = 0;                           // CAN print CANStr
int DISPLAY_debug = 0;                       // Display simulation

String espString = 0;                        // ESP serial data input
char rx_byte = 0;                            // DEBUG serial input

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  can1.begin();
  can1.setBaudRate(500000);                  // 500kbps data rate
  can2.begin();
  can2.setBaudRate(500000);                  // 500kbps data rate
  // Must set AIN as input, on Teensy 4.0 to disable internal pullup!
  //  pinMode(7, INPUT);                         // TEST RX2
  //  pinMode(8, INPUT);                         // TEST TX2
  pinMode(A0, INPUT);                        // Oil press sensor
  pinMode(A1, INPUT);                        // Indoor temp
  pinMode(A10, INPUT);                       // TEC outside temp
  pinMode(A12, INPUT);                       // TEC cold temp
  pinMode(A13, INPUT);                       // TEC hot temp
  pinMode(32, INPUT_PULLUP);                 // menu button Z/GND (20-50k pullup)
  pinMode(17, OUTPUT);                       // AC 230V relay (PTC cabin preheat)
  pinMode(20, OUTPUT);                       // TEC PSU 230V relay
  pinMode(10, OUTPUT);                       // TEC PSU heat/cool relay (switch neg/pos)
  pinMode(11, OUTPUT);                       // TEC PSU DC/AC relay
  pinMode(16, OUTPUT);                       // BAT coolant pump
  pinMode(12, OUTPUT);                       // Main coolant pump relay (in AC only bat heat/cool)
  pinMode(6, OUTPUT);                        // IEC 62196 Green status LED
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  digitalWrite(6, HIGH);                     // IEC 62196 Green status LED
  digitalWrite(17, HIGH);                    // AC ON/OFF relay for PTC preheat

  lcd.begin(16, 2);                          // set up columns and rows
  lcd.print(" CanLcd ver: 3.5");             // send version to the LCD.
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
  Serial.println(F("InCar BMS INV IAA CAN-Tx/Rx Teensy 4.0 150Mhz ver.35"));
  delay(1000);
  lcd.clear();
  digitalWrite(led, LOW);
  digitalWrite(6, LOW);                      // IEC 62196 Green status LED

  if (DISPLAY_debug == 1)
  {
    CELL_TMP = -2000;                        // CAN BMS average cell temp
    OUT_T2 = -20.00;
    IN_T3 = -20.00;
    tmpm = -20;
    tmphs = -20;
    Watt = -110000;
    TEC_T0 = -20.00;
    TEC_T1 = -20.00;
    Whkm = -11000;
    Whkm_trip = -1100;
  }
}

void loop()
{
  if (Serial.available() > 0)                // is a character available?
  {
    rx_byte = Serial.read();                 // get the character
    if (rx_byte == '\n')
      buttonCount++;
  }

  int BUT_READ = digitalRead(32);            // Read pushbutton state
  if (BUT_READ != lastButtonState)           // If button pressed
    lastDebounceTime = millis();             // reset the debouncing timer
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (BUT_READ != buttonState)             // if the button state has changed
    {
      buttonState = BUT_READ;
      if (buttonState == LOW)                // if the state has changed
        buttonCount++;                       // Increment the counter
    }
  }
  lastButtonState = BUT_READ;                // save state as last state, for next loop
  if (buttonCount > 7)                       // Last screen
    buttonCount = 0;                         // Back to first screen

  if (buttonState == LOW || rx_byte == '\n') // if the state has changed or serial input, show sreen names
  {
    rx_byte = 0;                             // Run only once, reset rx_byte
    if (buttonCount == 0)                    // Screen "CHARGE"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("CHARGE");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("      ");
    }
    if (buttonCount == 1)                    // Screen "TRIP"
    {
      lcd.clear();
      lcd.setCursor(6, 0);
      lcd.print("TRIP");
      delay(300);
      lcd.setCursor(6, 0);
      lcd.print("    ");
    }
    if (buttonCount == 2)                    // Screen "Wh/km"
    {
      lcd.clear();
      lcd.setCursor(6, 0);
      lcd.print("Wh/km");
      delay(300);
      lcd.setCursor(6, 0);
      lcd.print("     ");
    }
    if (buttonCount == 3)                    // Screen "DRIVE"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("DRIVE");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("     ");
    }
    if (buttonCount == 4)                    // Screen "BATTERY"
    {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("BATTERY");
      delay(300);
      lcd.setCursor(4, 0);
      lcd.print("       ");
    }
    if (buttonCount == 5)                    // Screen "CLIMATE"
    {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("CLIMATE");
      delay(300);
      lcd.setCursor(4, 0);
      lcd.print("       ");
    }
    if (buttonCount == 6)                    // Screen "MOTOR"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("MOTOR");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("     ");
    }
    if (buttonCount == 7)                    // Screen "BAT TEC"
    {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("BAT TEC");
      delay(300);
      lcd.setCursor(4, 0);
      lcd.print("       ");
    }
  }

  ////////// Read CAN bus ///////////

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
      data[0] = rxmsg1.buf[0];               // Test DEBUG
      data[1] = rxmsg1.buf[1];               // Test DEBUG
      data[2] = rxmsg1.buf[2];               // Test DEBUG
      data[3] = rxmsg1.buf[3];               // Test DEBUG
      CHG_phase = rxmsg1.buf[4];             // parameters.phaseconfig 1phase 0, 3phase 1
      EVSE_prox = rxmsg1.buf[5];             // Proximity Unconnected 0, Buttonpress 1, Connected 2
      data[7] = rxmsg1.buf[7];               // Count to 255 Test DEBUG
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

    // CAN BMS High/low cell volt and temp
    if (rxmsg2.id == 0x373)
    {
      data[0] = rxmsg2.buf[0];               // Min Cell Voltage LSB
      data[1] = rxmsg2.buf[1];               // Min Cell  Voltage MSB
      data[2] = rxmsg2.buf[2];               // Max Cell Voltage LSB
      data[3] = rxmsg2.buf[3];               // Max Cell  Voltage MSB
      data[4] = rxmsg2.buf[4];               // Min Temperature LSB
      data[5] = rxmsg2.buf[5];               // Min Temperature MSB
      data[6] = rxmsg2.buf[6];               // Max Temperature LSB
      data[7] = rxmsg2.buf[7];               // Max Temperature MSB
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
      LO_CELL = (uint8_t)data[1] << 8;       // read byte 1 << bitshift left
      LO_CELL |= data[0];                    // read byte 0
      HI_CELL = (uint8_t)data[3] << 8;       // read byte 1 << bitshift left
      HI_CELL |= data[2];                    // read byte 0
      MIN_CELL_TMP = (uint8_t)data[5] << 8;  // read byte 5 << bitshift left
      MIN_CELL_TMP |= data[4];               // read byte 4
      MAX_CELL_TMP = (uint8_t)data[7] << 8;  // read byte 7 << bitshift left
      MAX_CELL_TMP |= data[6];               // read byte 6
    }

    // CAN current sensor 0x3C2
    if (rxmsg2.id == 0x3c2)
    {
      data[0] = rxmsg2.buf[0];               //
      data[1] = rxmsg2.buf[1];               //
      data[2] = rxmsg2.buf[2];               //
      data[3] = rxmsg2.buf[3];               //
      data[4] = rxmsg2.buf[4];               //
      data[5] = rxmsg2.buf[5];               //
      data[6] = rxmsg2.buf[6];               //
      data[7] = rxmsg2.buf[7];               //
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
      CAB = (uint8_t)data[0] << 24;          // read byte 0 << bitshift left
      CAB |= data[1] << 16;                  // read byte 1 << bitshift left
      CAB |= data[2] << 8;                   // read byte 2 << bitshift left
      CAB |= data[3];                        // read byte 3
      CAB -= 0x80000000UL;
      CAB = CAB / 100;                       // 0.0A precision
      Watt = CAB * (VOLT / 10) / 100;
    }

    // CAN Motor Inverter
    if (rxmsg2.id == 0x3FF)
    {
      tmphs = rxmsg2.buf[0];                 // Heatsink temp
      tmpm = rxmsg2.buf[1];                  // Motor temp
      data[2] = rxmsg2.buf[2];               // Inverter AUX volt LSB
      data[3] = rxmsg2.buf[3];               // Inverter AUX volt MSB
      DIR = rxmsg2.buf[4];                   // Motor Rew Fwd P/N
      MOT_ON = rxmsg2.buf[5];                // Motor mode Off Run
      data[6] = rxmsg2.buf[6];               // Motor RPM kmh LSB
      data[7] = rxmsg2.buf[7];               // Motor RPM kmh MSB
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
      RPM = (uint16_t)data[7] << 8;          // read byte 7 << bitshift left (must be uint16_t 8 dosn't work)
      RPM |= data[6];                        // read byte 6
      kmh = (RPM / 9.57);                    // Max RPM 11500 /95.7 = 120km/h
      len_trip = (len_trip + (kmh * 1000) / 36);   // Travel length in 100mS
      km_trip = (len_trip / 10000);          // Precision 0.0

      if (MOT_ON == 1)
      {
        OILPR = analogRead(A0);              // read analog oil pressure sensor
        OILPR = map(OILPR, 108, 918, 0, 100);
        if (OILPR < 5)
          LO_OIL_L = 1;                      // Low oil lamp
        else LO_OIL_L = 0;
      }
    }

    // CAN BMS errors
    if (rxmsg2.id == 0x35A)
    {
      data[0] = rxmsg2.buf[0];               // bit3 Undervoltage, bit4 Overvoltage
      data[1] = rxmsg2.buf[1];               // bit7 Overtemp
      data[2] = rxmsg2.buf[2];               // bit12 Undertemp
      data[7] = rxmsg2.buf[7];               // bit1 Cell delta volt (default >200)
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

      if (data[0] > 1)                       // Overvoltage/Undervoltage
      {
        if (data[0] == 0x4)                  // bit4 Overvoltage
          HAZ_L = 1;
        if (data[0] == 0x10)                 // bit3 Undervoltage
          HAZ_L = 1;
      }
      else HAZ_L = 0;
    }

    // CAN BMS SOC, bmsstatus
    if (rxmsg2.id == 0x355)
    {
      SOC = rxmsg2.buf[0];                   // SOC Range is 0-100 no scaling
      bmsstatus = rxmsg2.buf[2];             // BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6
      bms_bal = rxmsg2.buf[3];               // BMS balancing OFF 0, ON 1
      bms_kWh = rxmsg2.buf[4];               // BMS kWh in battery
      if (CAN_debug == 1)
      {
        Serial.print("BMS SOC, MODE, BAL: ");
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
      data[0] = rxmsg2.buf[0];               // VOLT LSB
      data[1] = rxmsg2.buf[1];               // VOLT MSB
      data[2] = rxmsg2.buf[2];               // AMP LSB (Not used anymore)
      data[3] = rxmsg2.buf[3];               // AMP MSB (Not used anymore)
      data[4] = rxmsg2.buf[4];               // AVG CELL TEMP LSB
      data[5] = rxmsg2.buf[5];               // AVG CELL TEMP MSB
      data[6] = rxmsg2.buf[6];               // AVG CELL VOLT LSB
      data[7] = rxmsg2.buf[7];               // AVG CELL VOLT MSB
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
      VOLT = (uint8_t)data[1] << 8;          // read byte 1 << bitshift left
      VOLT |= data[0];                       // read byte 0
      if (DISPLAY_debug == 0)
      {
        CELL_TMP = (uint8_t)data[5] << 8;    // read byte 5 << bitshift left
        CELL_TMP |= data[4];                 // read byte 4
      }
      AVG_CELL = (uint8_t)data[7] << 8;      // read byte 1 << bitshift left
      AVG_CELL |= data[6];                   // read byte 0

      ////////// Send to CAN ///////////

      // Send CAN data for every receive of BMS CAN id 0x356
      // Send CAN data to IAA, lamps
      txmsg.len = 8;
      txmsg.id = 0x700;
      txmsg.buf[0] = 0;                      // Charge lamp
      txmsg.buf[1] = HAZ_L;                  // Hazard lamp
      txmsg.buf[2] = 0;                      // Power limit lamp
      txmsg.buf[3] = 0;                      // MIL lamp
      txmsg.buf[4] = 0;                      // Low SOC lamp (Only active in run)
      txmsg.buf[5] = LO_OIL_L;               // Low oil lamp
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
      can1.write(txmsg);                     // Transmit first CAN data

      // Send CAN data to IAA, gauges, radiator fan relay
      txmsg.len = 8;
      txmsg.id = 0x701;
      txmsg.buf[0] = SOC;                    // SOC gauge PWM 25 to 188 (half 118)
      txmsg.buf[1] = DTE;                    // DTE gauge 31-210Hz (half 120Hz)
      txmsg.buf[2] = MOT_ON;                 // Motor ON gauge 0/1
      txmsg.buf[3] = ECO;                    // ECO gauge PWM 25 to 188 (Only active in run)
      txmsg.buf[4] = 0;                      // No cell temp to instrument cluster!
      txmsg.buf[5] = LO_FAN;                 // Radiator LO fan relay
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
      can1.write(txmsg);                     //Transmit next CAN data

      ////////// Battery temperature control ///////////

      // Read outdoor/indoor temp for every receive of BMS CAN id 0x356
      if (DISPLAY_debug == 0)
      {
        OUT_V2 = analogRead(A10);            // NTC sensor outdoor
        TEC_R2 = TEC_R100k * (1023.0 / (float)OUT_V2 - 1.0);
        OUT_T2 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
        OUT_T2 = OUT_T2 - 273.15;            // Convert to C

        IN_V3 = analogRead(A1);              // TEC NTC sensor indoor
        TEC_R2 = TEC_R100k * (1023.0 / (float)IN_V3 - 1.0);
        IN_T3 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
        IN_T3 = IN_T3 - 273.15;              // Convert to C


        // Temp control TEC heat/cool for every receive of BMS CAN id 0x356
        if (EVSE_prox == 2 || digitalRead(17) == 1) // TEC PSU on AC (EVSE connected) or DC on (HV AUX)
        {
          if (CELL_TMP_COUNT > 150)          // Avoid STORED_CELL_TMP zero at startup (set to 200)
          {
            STORED_CELL_TMP = CELL_TMP;      //
            CELL_TMP_COUNT = 0;
          }
          CELL_TMP_COUNT++;                  // Increment
          if (CELL_TMP_COUNT > 110)
          {
            CELL_TMP_CHANGE = CELL_TMP - STORED_CELL_TMP; //Cell temp change between CELL_TMP_COUNT
            STORED_CELL_TMP = CELL_TMP;
            CELL_TMP_COUNT = 0;
          }

          // Shut off PWM, PSU and pumps
          if (TEC_T1 > 50 || TEC_T0 < -10 || CELL_TMP > 2500)  // Turn off PWM if temp to high/low
            TEC_pwm = 0;

          if (TEC_pwm == 0)                  // Turn off if TEC heater is min (0 PWM)
          {
            digitalWrite(20, LOW);                 // TEC PSU 230V relay
            digitalWrite(12, LOW);           // Main coolant waterpump relay (in AC only bat heat/cool)
          }
          else
          {
            digitalWrite(12, HIGH);          // Main coolant waterpump relay (in AC only bat heat/cool)
            digitalWrite(20, HIGH);          // TEC PSU 230V relay
          }
          if (CELL_TMP_COUNT == 10)          // Store DELTA_CELL_TMP for every 110sec
            DELTA_CELL_TMP = MAX_CELL_TMP - MIN_CELL_TMP; // to avoid BAT coolant pump going ON/OFF frequently.

          if (TEC_pwm > 0 || (DELTA_CELL_TMP >= 50))  // If more than 0.5C delta high/low battery temp
            digitalWrite(16, HIGH);          // Start BAT coolant pump
          else
            digitalWrite(16, LOW);

          if (TEC_T1 < 45 && TEC_T0 > -5 && CELL_TMP < 1900)  // Start PWM and temp reading if temp is "normal"
          {
            TEC_V0 = analogRead(A12);              // TEC NTC sensor cold (bottom plate)
            TEC_R2 = TEC_R100k * (1023.0 / (float)TEC_V0 - 1.0);
            TEC_T0 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
            TEC_T0 = TEC_T0 - 273.15;              // Convert to C

            TEC_V1 = analogRead(A13);              // TEC NTC sensor hot (top plate)
            TEC_R2 = TEC_R100k * (1023.0 / (float)TEC_V1 - 1.0);
            TEC_T1 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
            TEC_T1 = TEC_T1 - 273.15;              // Convert to C

            // Check if following is correct, or can be improved:
            // Needs more boost when going from "low" temp to "normal" temp. Now it rise slow, 4-5 (0.04-0.05C)

            if (CELL_TMP_COUNT == 100)       // Change power (PWM) for every CELL_TMP_COUNT
            {
              if (CELL_TMP <= 1700)
              {
                TEC_pwm = map(OUT_T2, -20, 20, 150, 0); // Set TEC PWM dependent on outdoor temp
                if (OUT_T2 <= 17)                       // Avoid < 1 in next line
                  TEC_pwm = (TEC_pwm * ((18 - OUT_T2) / 10 + 1)); // Difference between ideal bat temp (18C) and outdor temp
              }
              else
              {
                if (CELL_TMP <= 1800)        // Increase power
                  TEC_pwm = (TEC_pwm + 1);
                if (CELL_TMP >= 1820)        // Decrease power
                  TEC_pwm = (TEC_pwm - 1);

                if (CELL_TMP > 1800 && CELL_TMP < 1820)  //
                {
                  if (CELL_TMP_CHANGE > 0)   // CELL TMP GOING UP
                    TEC_pwm = (TEC_pwm - CELL_TMP_CHANGE); // Decrease PWM (0.01C = -1 PWM)
                  if (CELL_TMP_CHANGE < 0)   // CELL TMP GOING DOWN
                    TEC_pwm = (TEC_pwm - CELL_TMP_CHANGE); // Increase PWM (0.01C = +1 PWM)
                }
              }
            }
            TEC_pwm = constrain (TEC_pwm, 0, 150); // Min Pos duty for PSU is 10%/20PWM?
            analogWrite (3, TEC_pwm);
          }
        }
      }
      else                                   // TEC PSU on AC (EVSE not connected) or DC off (HV AUX)
      {
        TEC_pwm = 0;
        digitalWrite(20, LOW);               // TEC PSU 230V relay
        digitalWrite(12, LOW);               // Main coolant waterpump relay (in AC only bat heat/cool)
      }

      ////////// LCD display ///////////

      if (buttonCount == 0)                  // Screen "CHARGE"
      {
        lcd.setCursor(5, 0);
        if (EVSE_prox == 2)                  // EVSE connected
          lcd.print(Watt * 0.001, 1);        // 0.0kW precision
        else lcd.print("0.0");
        lcd.print("kW ");
        lcd.setCursor(13, 0);
        if (CHG_phase == 0)                  // parameters.phaseconfig 1phase 0, 3phase 1
          lcd.print("1ph");
        if (CHG_phase == 1)
          lcd.print("3ph");
      }

      if (buttonCount == 1)                  // Screen "TRIP"
      {
        if (DISPLAY_debug == 1)              // DEBUG
        {
          km_trip++;                         // DEBUG
          if (km_trip > 120)                 // DEBUG
            km_trip = km_trip + 100;         // DEBUG
          if (km_trip > 3500)                // DEBUG
            km_trip = 0;                     // DEBUG
        }
        lcd.setCursor(9, 0);
        if (km_trip < 100)                   // 0.0 to 9.9
          lcd.print ("  ");
        if (km_trip < 1000 && km_trip > 99)  // 10.0 to 99.9
          lcd.print (" ");
        lcd.print (km_trip * 0.1, 1);        // Over 99.9
        lcd.print ("km");
      }

      if (buttonCount < 2)                   // Screen "CHARGE or TRIP"
      {
        lcd.setCursor(0, 0);
        lcd.print (SOC);
        lcd.print ("%  ");
        // SOC bar
        int SOC_bar_count = 0;               // Soc bar counter (0-16)
        int SOC_bar = map(SOC, 1, 100, 0, 16); // Tested 1 block is 8%, 15 blocks is 94%
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

      if (buttonCount == 2)                  // Screen "Wh/km"
      {
        if (DISPLAY_debug == 1)              // DEBUG
        {
          Whkm = Whkm + 101;                // DEBUG
          if (Whkm > 11000)
            Whkm = -11000;
          Whkm_trip = Whkm_trip + 11;       // DEBUG
          if (Whkm_trip > 1100)
            Whkm_trip = -1100;
        }

        lcd.setCursor(0, 0);
        if (Whkm <= 9999 && Whkm >= -9999)
          lcd.print(Whkm);
        if (Whkm > 9999)
          lcd.print ("++++");
        if (Whkm < -9999)
          lcd.print ("-----");
        lcd.print("Wh    ");

        lcd.setCursor(10, 0);
        if (Whkm_trip < 10 && Whkm_trip >= 0) // 0 to 9
          lcd.print("   ");
        if (Whkm_trip < 100 && Whkm_trip > 9 || Whkm_trip > -10 && Whkm_trip < 0) // 10 to 99 or -1 to -9
          lcd.print("  ");
        if (Whkm_trip < 1000 && Whkm_trip > 99 || Whkm_trip > -100 && Whkm_trip < -9) // 100 to 999 or -10 to -99
          lcd.print(" ");

        if (Whkm_trip <= 999 && Whkm_trip >= -999)
          lcd.print(Whkm_trip);
        if (Whkm_trip > 999)
          lcd.print ("++++");
        if (Whkm_trip < -999)
          lcd.print ("----");
        lcd.print ("Wh");

        // Whkm bar
        BAR = 0;
        BAR_count = 0;                       // Whkm bar counter (0-16)
        if (Whkm  < 0)
          BAR = map(Whkm, -1, -400, 0, 16);  //
        else
          BAR = map(Whkm, 1, 400, 0, 16);    //
        BAR = constrain (BAR, 0, 16);        //
        lcd.setCursor(0, 1);
        for (BAR_count = 0; BAR > BAR_count; BAR_count++)   // count from 0 up to Whkm_bar
          if (Whkm  < 0)
            lcd.print((char) 219);           // print block
          else
            lcd.print((char) 255);           // print block
        for (BAR_count = 16; BAR < BAR_count; BAR_count--)  // count from 15 down to Whkm_bar
          lcd.print(" ");                    // print space (to clear old value)
      }

      if (buttonCount == 3)                  // Screen "DRIVE"
      {
        if (DISPLAY_debug == 1)              // DEBUG
        {
          Watt = Watt + 1100;                // DEBUG
          if (kmh < 100)                     // DEBUG
            kmh++;                           // DEBUG
          else
            kmh = kmh + 101;                 // DEBUG

          if (kmh > 1500)                    // DEBUG
            kmh = 0;                         // DEBUG
          if (Watt > 120000)                 // DEBUG
            Watt = -110000;                  // DEBUG
        }

        lcd.setCursor(0, 0);
        lcd.print(Watt * 0.001, 1);          // 0.0kW precision
        lcd.print("kW  ");
        lcd.setCursor(8, 0);
        if (kmh < 100)
          lcd.print("  ");
        if (kmh < 1000 && kmh >= 100)
          lcd.print(" ");
        lcd.print(kmh * 0.1, 1);
        lcd.print("kmh");
        // PWR bar
        BAR = 0;
        BAR_count = 0;                       // PWR bar counter (0-16)
        if (Watt  < 0)
          BAR = map(Watt, -1, -50000, 0, 16); // 1kW to 50kW
        else
          BAR = map(Watt, 1, 50000, 0, 16);  // 1kW to 50kW
        lcd.setCursor(0, 1);
        for (BAR_count = 0; BAR > BAR_count; BAR_count++)   // count from 0 up to PWR_BAR
          if (Watt  < 0)
            lcd.print((char) 219);           // print block
          else
            lcd.print((char) 255);           // print block
        for (BAR_count = 16; BAR < BAR_count; BAR_count--)  // count from 15 down to PWR_BAR
          lcd.print(" ");                    // print space (to clear old value)
      }

      if (buttonCount == 4)                  // Screen "BATTERY"
      {
        if (DISPLAY_debug == 1)              // DEBUG
        {
          CELL_TMP = CELL_TMP + 110;       // DEBUG
          if (CELL_TMP > 2000)               // DEBUG
            CELL_TMP = -2000;                // DEBUG
        }
        lcd.setCursor(0, 0);
        lcd.print (SOC);
        lcd.print ("% ");
        lcd.setCursor(4, 0);
        lcd.print(bms_kWh);
        lcd.print("kWh");
        lcd.setCursor(10, 0);
        if (CELL_TMP < 1000 && CELL_TMP >= 0)  // 0.00 - 9.99
          lcd.print("  ");
        if (CELL_TMP < 10000 && CELL_TMP > 999 || CELL_TMP > -1000 && CELL_TMP < 0) // 10.00 to 99.99 or -9.99 to -0.01
          lcd.print(" ");
        lcd.print (CELL_TMP * 0.01, 1);
        lcd.print("C");
        lcd.setCursor(0, 1);
        if (bms_bal == 1)
          lcd.print("BAL");
        lcd.setCursor(4, 1);
        lcd.print (HI_CELL - LO_CELL);
        lcd.print("mV ");
        lcd.setCursor(11, 1);
        lcd.print(AVG_CELL * 0.001, 2);
        lcd.print("V");
      }

      if (buttonCount == 5)                  // Screen "CLIMATE"
      {
        if (DISPLAY_debug == 1)              // DEBUG
        {
          OUT_T2 = OUT_T2 + 0.1;             // DEBUG
          if (OUT_T2 > 20)                   // DEBUG
            OUT_T2 = -20;                    // DEBUG
          IN_T3 = IN_T3 + 0.1;               // DEBUG
          if (IN_T3 > 20)                    // DEBUG
            IN_T3 = -20;                     // DEBUG
        }
        lcd.setCursor(0, 0);
        lcd.print ("Out");
        lcd.setCursor(10, 0);
        if (OUT_T2 < 10.00 && OUT_T2 >= 0.00)  // 0.00 - 9.99
          lcd.print("  ");
        if (OUT_T2 < 100.00 && OUT_T2 > 9.99 || OUT_T2 > -10.00 && OUT_T2 < 0.00) // 10.00 to 99.99 or -9.99 to -0.01
          lcd.print(" ");
        lcd.print (OUT_T2, 1);
        lcd.print("C ");
        lcd.setCursor(0, 1);
        lcd.print("In");
        lcd.setCursor(4, 1);
        if (digitalRead(17) == 1)            // AC 230V relay (PTC cabin preheat)
          lcd.print("AC ON");
        else
          lcd.print("     ");
        lcd.setCursor(10, 1);
        if (IN_T3 < 10.00 && IN_T3 >= 0.00)  // 0.00 - 9.99
          lcd.print("  ");
        if (IN_T3 < 100.00 && IN_T3 > 9.99 || IN_T3 > -10.00 && IN_T3 < 0.00) // 10.00 to 99.99 or -9.99 to -0.01
          lcd.print(" ");
        lcd.print(IN_T3, 1);
        lcd.print("C ");
      }

      if (buttonCount == 6)                  // Screen "MOTOR"
      {
        if (DISPLAY_debug == 1)              // DEBUG
        {
          tmpm = tmpm + 1;                   // DEBUG
          if (tmpm > 20)                     // DEBUG
            tmpm = -20;                      // DEBUG
          tmphs = tmphs + 1;                 // DEBUG
          if (tmphs > 20)                    // DEBUG
            tmphs = -20;                     // DEBUG
        }
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
        lcd.setCursor(12, 0);
        if (tmpm < 10 && tmpm >= 0)  // 0 - 9
          lcd.print("  ");
        if (tmpm < 100 && tmpm > 9 || tmpm > -10 && tmpm < 0) // 10 to 99 or -9 to -0
          lcd.print(" ");
        lcd.print (tmpm);
        lcd.print("C ");
        lcd.setCursor(0, 1);
        if (MOT_ON == 1)
          lcd.print (OILPR * 0.1, 1);
        else
          lcd.print ("0.0");
        lcd.print ("psi");
        lcd.setCursor(7, 1);
        lcd.print ("INV");
        lcd.setCursor(12, 1);
        if (tmphs < 10 && tmphs >= 0)  // 0 - 9
          lcd.print("  ");
        if (tmphs < 100 && tmphs > 9 || tmphs > -10 && tmphs < 0) // 10 to 99 or -9 to -0
          lcd.print(" ");
        lcd.print(tmphs);
        lcd.print("C ");
      }

      if (buttonCount == 7)                  // Screen "BAT TEC"
      {
        if (DISPLAY_debug == 1)              // DEBUG
        {
          TEC_T1 = TEC_T1 + 0.1;             // DEBUG
          if (TEC_T1 > 20)                   // DEBUG
            TEC_T1 = -20;                    // DEBUG
          TEC_T0 = TEC_T0 + 0.1;             // DEBUG
          if (TEC_T0 > 20)                   // DEBUG
            TEC_T0 = -20;                    // DEBUG
        }
        lcd.setCursor(0, 0);
        lcd.print ("Heat AC ");
        lcd.setCursor(10, 0);
        if (TEC_T1 < 10.00 && TEC_T1 >= 0.00)  // 0.00 - 9.99
          lcd.print("  ");
        if (TEC_T1 < 100.00 && TEC_T1 > 9.99 || TEC_T1 > -10.00 && TEC_T1 < 0.00) // 10.00 to 99.99 or -9.99 to -0.01
          lcd.print(" ");
        lcd.print (TEC_T1, 1);
        lcd.print("C ");
        lcd.setCursor(0, 1);
        lcd.print (TEC_Watt + 20);           // Add 20W for pump and 12v charger
        lcd.print("W ");
        lcd.setCursor(5, 1);
        lcd.print(CELL_TMP_CHANGE);          // Cell temp going up/down 0.01
        lcd.setCursor(10, 1);
        if (TEC_T0 < 10.00 && TEC_T0 >= 0.00)  // 0.00 - 9.99
          lcd.print("  ");
        if (TEC_T0 < 100.00 && TEC_T0 > 9.99 || TEC_T0 > -10.00 && TEC_T0 < 0.00) // 10.00 to 99.99 or -9.99 to -0.01
          lcd.print(" ");
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

      ////////// Calculate Whkm ////////

      if (loop_count > 0 && kmh > 0)         // if loop has started (trip started) and more than 0kmh (avoid divide by zero)
      {
        Whkm = (Watt / (kmh * 0.1));         //
        Whkm_count = (Whkm + Whkm_count);    // add/subtract to counter (for average Wh/km calculation)
        Whkm_trip = (Whkm_count / (loop_count));  // calculate average Wh/km for whole trip
      }
      //      else                                 // Is this needed??
      //        Whkm = 0;
      loop_count++;                          // Increment the counter

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
        if (CHG_phase == 0)                  // parameters.phaseconfig 1phase 0, 3phase 1
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

      ////////// DEBUG send to serial////////

      // CAN Tesla Charger
      if (CHG_phase == 0)                    // parameters.phaseconfig 1phase 0, 3phase 1
        Serial.println("CHG: 1phase");
      if (CHG_phase == 1)
        Serial.println("CHG: 3phase");
      if (EVSE_prox == 2)                    // Proximity Unconnected 0, Buttonpress 1, Connected 2
        Serial.println("EVSE: Connected");
      if (EVSE_prox == 0)                    // Proximity Unconnected 0, Buttonpress 1, Connected 2
        Serial.println("EVSE: Unconnected");

      // CAN BMS High/low cell volt and temp
      Serial.print("LO_CELL: ");
      Serial.print(LO_CELL);                 // DEBUG TEST
      Serial.print("  HI_CELL: ");
      Serial.print(HI_CELL);                 // DEBUG TEST
      Serial.print("  AVG_CELL: ");
      Serial.println(AVG_CELL);              // DEBUG TEST
      Serial.print("MIN CELL TMP: ");        // DEBUG TEST
      Serial.print(MIN_CELL_TMP);            // DEBUG TEST
      Serial.print("  MAX CELL TMP: ");      // DEBUG TEST
      Serial.print(MAX_CELL_TMP);            // DEBUG TEST
      Serial.print("  Delta: ");             // DEBUG TEST
      Serial.println(DELTA_CELL_TMP);

      // CAN current sensor
      Serial.print("mA: ");
      Serial.print(CAB);
      Serial.print("  Watt: ");
      Serial.println(Watt);
      Serial.print("Wh/km: ");
      Serial.print(Whkm);
      Serial.print("  Wh/km trip: ");
      Serial.println(Whkm_trip);

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
      Serial.print(kmh);                   // 0.0 precision
      Serial.print ("  km trip: ");
      Serial.println (len_trip * 0.0000001, 1); //

      Serial.print("Oil press: ");           // 0psi=108 10psi=918
      //Serial.println(OILPR * .0049);       // DEBUG TEST
      Serial.println(OILPR);                 // DEBUG TEST

      // CAN BMS errors
      Serial.print("BMS error: ");
      if (HAZ_L == 1)
        Serial.println("Cell high/low volt!! ");
      else
        Serial.println("No error ");

      // CAN BMS SOC, bmsstatus
      Serial.print("SOC: ");
      Serial.print(SOC);                     // DEBUG TEST
      Serial.print("  BMS kWh: ");
      Serial.println(bms_kWh);               // CAN BMS kWh in battry
      if (bmsstatus == 0)
        Serial.print("BMS: Boot 0 ");
      if (bmsstatus == 1)
        Serial.print("BMS: Ready 1 ");
      if (bmsstatus == 2)
        Serial.print("BMS: Drive 2 ");
      if (bmsstatus == 3)
        Serial.print("BMS: Charge 3 ");
      if (bmsstatus == 4)
        Serial.print("BMS: Precharge 4 ");
      if (bmsstatus == 5)
        Serial.print("BMS: Error 5 ");
      if (bmsstatus == 6)
        Serial.print("BMS: Bat_HC 6 ");
      Serial.print("  BAL: ");
      Serial.println(bms_bal);                // DEBUG TEST


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
      Serial.print("PWM: ");
      Serial.print(TEC_pwm);
      TEC_Watt = map(TEC_pwm, 0, 150, 2, 400); // Estimated watt fot TEC, pumps, 12V charger
      Serial.print("  TEC Watt: ");
      Serial.println(TEC_Watt + 20);         // Add 20W for pumps and 12v charger (add 200W for radiator fan!!)
      Serial.print("MAIN pump: ");           //
      Serial.print (digitalRead(12));        // Main coolant waterpump relay (in AC only bat heat/cool)
      Serial.print("  BAT pump: ");          //
      Serial.println (digitalRead(16));      // BAT coolant pump
      Serial.print("AC relay: ");
      Serial.print (digitalRead(17));        // AC 230V relay (PTC cabin preheat)
      Serial.print("  TEC AC relay: ");
      Serial.print (digitalRead(20));        // TEC PSU 230V relay
      Serial.print("  TEC AC/DC relay: ");
      Serial.println (digitalRead(11));      // TEC PSU 230V/HV DC relay

      Serial.print("ButtonCount: ");
      Serial.println(buttonCount);
      ////////////////////////////////////////////
      Serial.println("");

      if (CAN_COUNT == 1)                    //  Counter for CAN data LED blink
        digitalWrite(led, HIGH);
    }
  }
}
