#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // can1 port
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;  // can2 port

#include <FaBoLCD_PCF8574.h>                  // include the LCD library
FaBoLCD_PCF8574 lcd;

static CAN_message_t txmsg;
static CAN_message_t rxmsg1;
static CAN_message_t rxmsg2;
String CANStr("");
unsigned char data[6];                       // rxmsg2 size
byte CAN_COUNT = 0;                          // CAN data counter for LED
int led = 13;
int SOC = 25;                                // IAA SOC gauge PWM 25 to 188 (half 118)
int DTE = 120;                               // IAA DTE gauge 31-210Hz (half 120Hz)
int MOT_ON = 0;                              // IAA Motor ON gauge 0/1 (Inverter CAN mode off, run)
int ECO = 118;                               // IAA ECO gauge PWM 25 to 188, mid 118 (Only active in run)
int LO_FAN = 0;                              // IAA Radiator LO fan relay

byte CHG_L = 0;                              // IAA Charge lamp
byte HAZ_L = 0;                              // IAA Hazard lamp
byte PWR_LIM_L = 0;                          // IAA Power limit lamp
byte MIL_L = 0;                              // IAA MIL lamp
byte LO_SOC_L = 0;                           // IAA Low SOC lamp (Only active in run)
byte LO_OIL_L = 0;                           // IAA Low oil lamp

int VOLT = 0;                                // BMS pack volt
int AMP = 0;                                 // BMS current sensor
int CELL_TMP = 0;                            // BMS average cell temp
int STORED_CELL_TMP = 0;                     // Stored average cell temp
int MIN_CELL_TMP = 0;                        // BMS lowest cell temp
int MAX_CELL_TMP = 0;                        // BMS highest cell temp
byte CELL_TMP_CHANGE = 2;                    // Average cell temp, 1 - going up, 0 - going down.
int CELL_TMP_COUNT = 200;
int LO_CELL = 0;                             // BMS lowest cell
int HI_CELL = 0;                             // BMS highest cell
byte bmsstatus = 0;                          // BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6

int AUX_V = 0;                               // Inverter CAN AUX volt (12V battery)
int tmphs = 0;                               // Inverter CAN heatsink temp neg/pos
int tmpm = 0;                                // Inverter CAN motor temp neg/pos
byte DIR = 0;                                // Inverter CAN direction/gear
int RPM = 0;                                 // Inverter CAN RPM

int OILPR = 0;
int CAB = 0;
int TEC_V0;
int TEC_V1;
int TEC_V2;
int TEC_pwm = 20;                             // Check if PWM is 1kHz!!
float TEC_R1 = 100000;                        // 3.3V-100k NTC-sig-100k-GND
float TEC_logR2, TEC_R2, TEC_T0, TEC_T1, TEC_T2;
float TEC_A = 0.8158589285e-03, TEC_B = 2.076952932e-04, TEC_C = 0.9631468832e-07; // NTCLE203E3104GB0 -  Thermistor, NTC, 100 kohm, NTCLE Series, 4190 K
byte TEC_RLY = 1;                             // TEC PSU relay on/off (Waterpump bat pack)
int TEC_COUNT = 0;                            // TEC temp delay counter
int OUT_tmp_factor = 0;

long Watt = 0;                                // Calculated Watt neg/pos
int kmh = 0;                                  // Calculated km/h
int Whkm = 0;                                 // Calculated current Wh/km (neg/pos)
int Whkm_trip = 0;                            // Calculated trip Wh/km (neg/pos)
long Whkm_count = 0;                          // Counter to calculate trip Wh/km (neg/pos)
long loop_count = 0;                          // Counter for CAN read loops

byte buttonCount = 3;                         // counter for number of button presses
byte buttonState = 0;                         // current state of the button
byte lastButtonState = 1;                     // previous state of the button
unsigned long lastDebounceTime = 0;           // the last time the button was pressed
unsigned long debounceDelay = 50;             // the debounce time

void setup()
{
  can1.begin();
  can1.setBaudRate(500000);                   // 500kbps data rate
  can2.begin();
  can2.setBaudRate(500000);                   // 500kbps data rate
  pinMode(A0, INPUT);                         // Must set on Teensy 4.0 (disable internal pullup)
  pinMode(A10, INPUT);                        // TEC outside temp
  pinMode(A12, INPUT);                        // TEC cold temp
  pinMode(A13, INPUT);                        // TEC hot temp
  pinMode(32, INPUT_PULLUP);                  // menu button Z/GND (20-50k pullup)
  pinMode(16, OUTPUT);                        // TEC PSU relay (Waterpump bat pack)
  pinMode(12, OUTPUT);                        // TEC Main colant waterpump (in AC only bat heat/cool)
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  lcd.begin(16, 2);                           // set up columns and rows
  lcd.print(" CanLcd ver: 2.1");              // send version to the LCD.
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
  Serial.println(F("InCar BMS INV IAA CAN-Tx/Rx Teensy 4.0 150Mhz ver.21"));
  delay(1000);
  lcd.clear();
  digitalWrite(led, LOW);
}

void loop()
{
  if (CAN_COUNT == 1)                         // CAN data counter for LED
    digitalWrite(led, HIGH);

  int BUT_READ = digitalRead(32);             // Read pushbutton state
  if (BUT_READ != lastButtonState)            // If button pressed
    lastDebounceTime = millis();              // reset the debouncing timer
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (BUT_READ != buttonState)              // if the button state has changed:
    {
      buttonState = BUT_READ;
      if (buttonState == LOW)                 // if the state has changed
        buttonCount++;                        // Increment the counter
    }
  }
  lastButtonState = BUT_READ;                 // save state as last state, for next loop
  if (buttonCount > 3)                        // Last screen
    buttonCount = 0;                          // Back to first screen

  if (buttonState == LOW)                     // if the state has changed, show sreen names
  {
    if (buttonCount == 0)                     // Screen "BATTERY"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("BATTERY");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("       ");
    }
    if (buttonCount == 1)                     // Screen "ENERGY"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("ENERGY");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("      ");
    }
    if (buttonCount == 2)                     // Screen "MOTOR"
    {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("MOTOR");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("     ");
    }
    if (buttonCount == 3)                     // Screen "BATTERY H/C"
    {
      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("BATTERY H/C");
      delay(300);
      lcd.setCursor(2, 0);
      lcd.print("           ");
    }
  }


  while (can1.read(rxmsg1))
  {
    String CANStr("");
    for (int i = 0; i < 8; i++)
    {
      CANStr += String(rxmsg1.buf[i], HEX);
      CANStr += (" ") ;
    }

    if (rxmsg1.id == 0x3D8)                    // CAN Tesla Charger
    {
      data[0] = rxmsg1.buf[0];                 // Test DEBUG
      data[1] = rxmsg1.buf[1];                 // Test DEBUG
      data[2] = rxmsg1.buf[2];                 // Test DEBUG
      data[3] = rxmsg1.buf[3];                 // Test DEBUG
      data[4] = rxmsg1.buf[4];                 // Test DEBUG
      data[5] = rxmsg1.buf[5];                 // Proximity Unconnected 0, Buttonpress 1, Connected 2. Test DEBUG
      data[6] = rxmsg1.buf[6];                 // parameters.canControl Test DEBUG
      data[7] = rxmsg1.buf[7];                 // Count to 255 Test DEBUG
      Serial.print("CHARGER: ");
      Serial.print(rxmsg1.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg1.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();
      if (data[5] == 0)
        Serial.println("EVSE: Unconnected");
      if (data[5] == 1)
        Serial.println("EVSE: Buttonpress");
      if (data[5] == 2)
        Serial.println("EVSE: Connected");
    }
  }

  while (can2.read(rxmsg2))
  {
    String CANStr("");
    for (int i = 0; i < 8; i++)
    {
      CANStr += String(rxmsg2.buf[i], HEX);
      CANStr += (" ") ;
    }

    char buf_LCD[16];

    if (rxmsg2.id == 0x373)                    // CAN BMS High/low cell volt and temp
    {
      data[0] = rxmsg2.buf[0];                 // Min Cell Voltage LSB
      data[1] = rxmsg2.buf[1];                 // Min Cell  Voltage MSB
      data[2] = rxmsg2.buf[2];                 // Max Cell Voltage LSB
      data[3] = rxmsg2.buf[3];                 // Max Cell  Voltage MSB
      data[4] = rxmsg2.buf[4];                 // Min Temperature LSB
      data[5] = rxmsg2.buf[5];                 // Min Temperature MSB
      data[6] = rxmsg2.buf[6];                 // Max Temperature LSB
      data[7] = rxmsg2.buf[7];                 // Max Temperature MSB
      Serial.print("BMS HI/LO CELL: ");
      Serial.print(rxmsg2.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg2.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();
      LO_CELL = (uint8_t)data[1] << 8;        // read byte 1 << bitshift left
      LO_CELL |= data[0];                     // read byte 0
      Serial.print("LO_CELL: ");
      Serial.println(LO_CELL);                // DEBUG TEST
      HI_CELL = (uint8_t)data[3] << 8;        // read byte 1 << bitshift left
      HI_CELL |= data[2];                     // read byte 0
      Serial.print("HI_CELL: ");
      Serial.println(HI_CELL);                // DEBUG TEST
      MIN_CELL_TMP = (uint8_t)data[5] << 8;   // read byte 5 << bitshift left
      MIN_CELL_TMP |= data[4];                // read byte 4
      Serial.print("MIN CELL TMP: ");          // DEBUG TEST
      Serial.println(MIN_CELL_TMP);           // DEBUG TEST
      MAX_CELL_TMP = (uint8_t)data[7] << 8;   // read byte 7 << bitshift left
      MAX_CELL_TMP |= data[6];                // read byte 6
      Serial.print("MAX CELL TMP: ");          // DEBUG TEST
      Serial.println(MAX_CELL_TMP);           // DEBUG TEST
    }

    if (rxmsg2.id == 0x300)                    // CAN current sensor 0x3C2 DEBUG TEST
    {
      data[0] = rxmsg2.buf[0];                 //
      data[1] = rxmsg2.buf[1];                 //
      data[2] = rxmsg2.buf[2];                 //
      data[3] = rxmsg2.buf[3];                 //
      data[4] = rxmsg2.buf[4];                 //
      data[5] = rxmsg2.buf[5];                 //
      data[6] = rxmsg2.buf[6];                 //
      data[7] = rxmsg2.buf[7];                 //
      Serial.print(rxmsg2.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg2.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();

      CAB = (uint8_t)data[1] << 8;            // read byte 1 << bitshift left
      Serial.println(CAB, HEX);
      CAB |= data[2];                         // read byte 0
      Serial.println(CAB, HEX);
      CAB = (uint8_t)CAB << 8;
      Serial.println(CAB, HEX);
      CAB |= data[3];
      Serial.println(CAB, HEX);
      Serial.print("CAB ");
      Serial.println(CAB);
    }

    if (rxmsg2.id == 0x3FF)                    // Inverter CAN data
    {
      data[0] = rxmsg2.buf[0];                 // Heatsink temp
      data[1] = rxmsg2.buf[1];                 // Motor temp
      data[2] = rxmsg2.buf[2];                 // Inverter AUX volt LSB
      data[3] = rxmsg2.buf[3];                 // Inverter AUX volt MSB
      DIR = rxmsg2.buf[4];                     // Motor Rew Fwd P/N
      MOT_ON = rxmsg2.buf[5];                  // Motor mode Off Run
      data[6] = rxmsg2.buf[6];                 // Motor RPM kmh LSB
      data[7] = rxmsg2.buf[7];                 // Motor RPM kmh MSB
      Serial.print("INVERTER: ");
      Serial.print(rxmsg2.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg2.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();
      if (DIR == 0)
        Serial.println("N/P");
      if (DIR == 1)
        Serial.println("D/E");
      if (DIR == 15)
        Serial.println("R  ");
      if (MOT_ON == 1)                         //  Motor mode Off Run
        Serial.println("Motor ON");

      if (MOT_ON == 1)
      {
        OILPR = analogRead(A0);                // read analog oil pressure sensor
        OILPR = map(OILPR, 108, 918, 0, 100);
        Serial.print("Oil press: ");           // 0psi=108 10psi=918
        //Serial.println(OILPR * .0049);       // DEBUG TEST
        Serial.println(OILPR);                 // DEBUG TEST
        if (OILPR < 5)
        {
          LO_OIL_L = 1;                        // Low oil lamp
          Serial.println("Low oil pressure!!");
        }
        else LO_OIL_L = 0;
      }
    }

    if (rxmsg2.id == 0x35A)                    // BMS errors
    {
      data[0] = rxmsg2.buf[0];                 // bit3 Undervoltage, bit4 Overvoltage
      data[1] = rxmsg2.buf[1];                 // bit7 Overtemp
      data[2] = rxmsg2.buf[2];                 // bit12 Undertemp
      data[7] = rxmsg2.buf[7];                 // bit1 Cell delta volt (default >200)
      Serial.print("BMS ERROR: ");
      Serial.print(rxmsg2.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg2.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();

      if (data[0] > 1)                         // Overvoltage/Undervoltage
      {
        if (data[0] == 0x4)                    // bit4 Overvoltage
        {
          HAZ_L = 1;
          Serial.println("Cell high volt!! ");
        }

        if (data[0] == 0x10)                   // bit3 Undervoltage
        {
          HAZ_L = 1;
          Serial.println("Cell low volt!! ");
        }
      }
      else HAZ_L = 0;
    }

    if (rxmsg2.id == 0x355)                    // BMS SOC
    {
      SOC = rxmsg2.buf[0];                     // SOC Range is 0-100 no scaling
      bmsstatus = rxmsg2.buf[2];               // BMS Boot 0, Ready 1, Drive 2, Charge 3, Precharge 4, Error 5, Bat_HC 6
      Serial.print("BMS MODE: ");
      Serial.print(rxmsg2.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg2.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();
      Serial.print("SOC: ");
      Serial.println(SOC);                     // DEBUG TEST
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

      //      lcd.setCursor(0, 0);             // Program halt!!??? Move down to CELL_TMP works
      //      lcd.print(SOC);
      //      lcd.print("%");
    }

    if (rxmsg2.id == 0x356)                    // BMS Pack volt, Current, cell temp.
    {
      data[0] = rxmsg2.buf[0];                 // VOLT LSB
      data[1] = rxmsg2.buf[1];                 // VOLT MSB
      data[2] = rxmsg2.buf[2];                 // AMP LSB
      data[3] = rxmsg2.buf[3];                 // AMP MSB
      data[4] = rxmsg2.buf[4];                 // AVG CELL TEMP LSB
      data[5] = rxmsg2.buf[5];                 // AVG CELL TEMP MSB
      Serial.print("BMS V, A, TMP: ");
      Serial.print(rxmsg2.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg2.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();

      VOLT = (uint8_t)data[1] << 8;           // read byte 1 << bitshift left
      VOLT |= data[0];                        // read byte 0
      Serial.print("PACK VOLT: ");
      Serial.println(VOLT);                   // DEBUG TEST
      //      lcd.setCursor(0, 1);
      //      sprintf(buf_LCD, "%d.%dv", VOLT / 100, VOLT / 10 % 10);
      //      lcd.print(buf_LCD);

      AMP = (uint8_t)data[3] << 8;            // read byte 3 << bitshift left
      AMP |= data[2];                         // read byte 2
      AMP = (AMP * -1);                       // (/10 for actual value)
      Serial.print("AMP: ");
      Serial.println(AMP);                    // DEBUG TEST
      //      lcd.setCursor(7, 1);
      //      sprintf(buf_LCD, "%d.%dA", AMP / 10, AMP % 10);
      //      lcd.print(buf_LCD);
      CELL_TMP = (uint8_t)data[5] << 8;       // read byte 5 << bitshift left
      CELL_TMP |= data[4];                    // read byte 4
      Serial.print("CELL TMP: ");             // DEBUG TEST
      Serial.println(CELL_TMP);               // DEBUG TEST
      Serial.print("STORED CELL TMP: ");      // DEBUG TEST
      Serial.println(STORED_CELL_TMP);        // DEBUG TEST

      //      lcd.setCursor(0, 0);                    // Program halt!!??? If moved ut to SOC
      //      lcd.print(SOC);
      //      lcd.print("%");
      if (CELL_TMP_COUNT > 150)                    // Avoid STORED_CELL_TMP zero at startup
        STORED_CELL_TMP = CELL_TMP;
      if (CELL_TMP < STORED_CELL_TMP)
      {
        CELL_TMP_CHANGE = 0;
        if (CELL_TMP_COUNT > 1)
          CELL_TMP_COUNT = 0;
        Serial.println("CELL TMP: GOING DOWN ");          // DEBUG TEST
      }
      if (CELL_TMP > STORED_CELL_TMP)
      {
        CELL_TMP_CHANGE = 1;
        if (CELL_TMP_COUNT > 1)
          CELL_TMP_COUNT = 0;
        Serial.println("CELL TMP: GOING UP ");          // DEBUG TEST
      }
      if (CELL_TMP == STORED_CELL_TMP)
      {
        CELL_TMP_CHANGE = 2;
        Serial.println("CELL TMP: STABLE ");          // DEBUG TEST
      }
      if (CELL_TMP_COUNT > 120)
      {
        STORED_CELL_TMP = CELL_TMP;
        CELL_TMP_COUNT = 0;
      }
      CELL_TMP_COUNT++;

      //Send CAN data for every receive of BMS CAN id 0x356
      txmsg.len = 8;
      txmsg.id = 0x700;
      txmsg.buf[0] = 0;                       // Charge lamp
      txmsg.buf[1] = HAZ_L;                   // Hazard lamp
      txmsg.buf[2] = 0;                       // Power limit lamp
      txmsg.buf[3] = 0;                       // MIL lamp
      txmsg.buf[4] = 0;                       // Low SOC lamp (Only active in run)
      txmsg.buf[5] = LO_OIL_L;                // Low oil lamp
      txmsg.buf[6] = 0;
      txmsg.buf[7] = 0;
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
      can1.write(txmsg);                      // Transmit first CAN data

      txmsg.len = 8;
      txmsg.id = 0x701;
      txmsg.buf[0] = SOC;                     // SOC gauge PWM 25 to 188 (half 118)
      txmsg.buf[1] = DTE;                     // DTE gauge 31-210Hz (half 120Hz)
      txmsg.buf[2] = MOT_ON;                  // Motor ON gauge 0/1
      txmsg.buf[3] = ECO;                     // ECO gauge PWM 25 to 188 (Only active in run)
      txmsg.buf[4] = 0;                       // No cell temp to instrument cluster!
      txmsg.buf[5] = LO_FAN;                  // Radiator LO fan relay
      txmsg.buf[6] = 0;
      txmsg.buf[7] = 0;
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
      can1.write(txmsg);                      //Transmit next CAN data

      // Read outside temp for every receive of BMS CAN id 0x356
      TEC_V2 = analogRead(A10);               // NTC sensor outside
      TEC_R2 = TEC_R1 * (1023.0 / (float)TEC_V2 - 1.0);
      TEC_T2 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
      TEC_T2 = TEC_T2 - 273.15;               // Convert to C
      Serial.print("OUT: ");
      Serial.print(TEC_T2);
      Serial.print("C  ");
      //      lcd.setCursor(11, 0);
      //      lcd.print(TEC_T2, 1);
      //      lcd.print("C");

      // Temp control TEC heat/cool for every receive of BMS CAN id 0x356
      if ((TEC_T1 > 50 || TEC_T0 < -10) || (CELL_TMP > 2500 && TEC_pwm == 20))        // Decrease power if temp to high/low
      {
        TEC_pwm = 0;
        TEC_RLY = 0;                          // TEC PSU relay on/off
        digitalWrite(16, LOW);                // TEC PSU relay (Waterpump bat pack)
        digitalWrite(12, LOW);                // TEC Main colant waterpump (in AC only bat heat/cool)

        Serial.println("TEC OFF");

      }
      if (TEC_RLY == 1 && CELL_TMP < 2500)     //
      {
        digitalWrite(16, HIGH);               // TEC PSU relay (Waterpump bat pack)
        digitalWrite(12, HIGH);               // TEC Main colant waterpump (in AC only bat heat/cool)
        OUT_tmp_factor = map(TEC_T2, - 20, 50, 10, 200);
        TEC_COUNT ++;                         // TEC temp delay counter
        if (TEC_COUNT > OUT_tmp_factor)       // TEC temp delay counter
          TEC_COUNT = 0;                      // TEC temp delay counter reset
        Serial.print("tmp_factor: ");
        Serial.print(OUT_tmp_factor);
        Serial.print("  Sec: ");
        Serial.println(TEC_COUNT);

        TEC_V0 = analogRead(A12);             // TEC NTC sensor cold (bottom plate)
        TEC_R2 = TEC_R1 * (1023.0 / (float)TEC_V0 - 1.0);
        TEC_T0 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
        TEC_T0 = TEC_T0 - 273.15;             // Convert to C
        Serial.print("COLD: ");
        Serial.print(TEC_T0);
        Serial.print("C  ");

        TEC_V1 = analogRead(A13);             // TEC NTC sensor hot (top plate)
        TEC_R2 = TEC_R1 * (1023.0 / (float)TEC_V1 - 1.0);
        TEC_T1 = (1.0 / (TEC_A + TEC_B * log(TEC_R2) + TEC_C * log(TEC_R2) * log(TEC_R2) * log(TEC_R2)));   // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
        TEC_T1 = TEC_T1 - 273.15;             // Convert to C
        Serial.print("HOT: ");
        Serial.print(TEC_T1);
        Serial.print("C  ");

        if (CELL_TMP < 1800 && TEC_T1 < 40 && TEC_COUNT == 0)  // Increase power for every TEC_COUNT, if temp reach min
          TEC_pwm = (TEC_pwm + 1);
        if (CELL_TMP > 1805)             //
        {
          if (CELL_TMP_CHANGE == 1)      // CELL TMP GOING UP
            TEC_pwm = (TEC_pwm - 1);
          if (CELL_TMP_CHANGE == 0)      // CELL TMP GOING DOWN
            TEC_pwm = (TEC_pwm + 1);
        }
        if (CELL_TMP > 1820 && TEC_COUNT == 0 || (TEC_T1 > 40 && TEC_COUNT == 0))   // Decrease power for every TEC_COUNT, if temp reach min

          TEC_pwm = (TEC_pwm - 1);
        if (CELL_TMP < 1815)             //
        {
          if (CELL_TMP_CHANGE == 1)      // CELL TMP GOING UP
            TEC_pwm = (TEC_pwm - 1);
          if (CELL_TMP_CHANGE == 0)      // CELL TMP GOING DOWN
            TEC_pwm = (TEC_pwm + 1);
        }


        TEC_pwm = constrain (TEC_pwm, 20, 150); // Min Pos duty for PSU is 10% (20)
        analogWrite (3, TEC_pwm);
        Serial.print(" PWM: ");
        Serial.println(TEC_pwm);

        if (buttonCount == 0 || buttonCount == 1)   // BATTERY/ENERGY screen
        {
          lcd.setCursor(0, 0);
          lcd.print(SOC);                     // Range is 0-100%
          if (SOC < 100)
            lcd.print("%");
          if (SOC < 10)
            lcd.print(" ");

          if (buttonCount == 0)               // BATTERY screen
          {
            // Amp
            lcd.setCursor(4, 0);
            if (bmsstatus == 2)               // BMS Drive 2
              lcd.print(AMP / 10);
            else
              sprintf(buf_LCD, "%d.%d", AMP / 10, AMP % 10);
            lcd.print(buf_LCD);
            lcd.print("A");
            if (AMP < 100)
              lcd.print(" ");
            if (AMP < 10)
              lcd.print(" ");                 // print space (to clear old value)
            // Pack Volt
            lcd.setCursor(0, 1);
            sprintf(buf_LCD, "%dv", VOLT);
            lcd.print(buf_LCD);
            // Average cell temp
            lcd.setCursor(11, 0);
            //            sprintf(buf_LCD, "%d.%dC/", CELL_TMP / 100, CELL_TMP % 10);    // Two decimals
            sprintf(buf_LCD, "%d.%dC/", CELL_TMP / 100, CELL_TMP % 100);        // One decimal
            lcd.print(buf_LCD);
          }

          if (buttonCount == 1)               // Screen "ENERGY"
            // Wh/km
            lcd.setCursor(5, 0);
          // Check if this is needed!?
          if (kmh > 0)                        // Avoid -1 reading when kmh = 0 (no divide with zero)
          {
            sprintf(buf_LCD, "%d/", Whkm);
            lcd.print(buf_LCD);
          }
          // Check if this is needed!?
          else
            lcd.print("0/");
          if (loop_count > 0)                 // if loop has started (trip started)
          {
            sprintf(buf_LCD, "%dWhkm    ", Whkm_trip); // "0/0Whkm    " print space (to clear old value)
            lcd.print(buf_LCD);
          }
          else
            lcd.print("0Whkm");

          if (loop_count > 0)                 // if loop has started (trip started)
          {
            Whkm_trip = (Whkm_count / (loop_count));  // calculate average Wh/km for whole trip
            Serial.print("Wh/km trip: ");
            Serial.println(Whkm_trip);        // DEBUG TEST
          }
          // SOC bar
          byte SOC_bar_count = 0;             // Soc bar counter (0-16)
          byte SOC_bar = map(SOC, 1, 100, 0, 16); // Tested 1 block is 8%, 15 blocks is 94%
          lcd.setCursor(0, 1);
          for (SOC_bar_count = 0; SOC_bar > SOC_bar_count; SOC_bar_count++)   // count from 0 up to SOC_bar
            lcd.print((char) 255);            // print block
          for (SOC_bar_count = 16; SOC_bar < SOC_bar_count; SOC_bar_count--)  // count from 15 down to SOC_bar
            lcd.print(" ");                   // print space (to clear old value)
        }

        if (buttonCount == 3)                 // Screen "BATTERY H/C"
        {
          lcd.setCursor(0, 0);
          sprintf(buf_LCD, "%d.%dC/", CELL_TMP / 100, CELL_TMP / 10 % 10);
          lcd.print("BAT ");
          lcd.print(buf_LCD);
          lcd.print(TEC_T2, 1);
          lcd.print("C ");
          lcd.setCursor(0, 1);
          lcd.print("TEC ");
          lcd.print(TEC_T1, 1);
          lcd.print("C/");
          lcd.print(TEC_T0, 1);
          lcd.print("C ");
        }
      }
    }
    if (kmh > 0)                              // do not continue if zero km/t
    {
      Whkm_count = (Whkm + Whkm_count);       // add/subtract to counter (for average Wh/km calculation)
      loop_count++;                           // Increment the counter
    }
    CAN_COUNT++;
    if (CAN_COUNT > 1)                        // CAN data counter for LED and average CELL_TMP
    {
      digitalWrite(led, LOW);
      CAN_COUNT = 0;                          // CAN data counter reset
    }
  }
}
