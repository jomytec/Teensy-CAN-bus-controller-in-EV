#include <FlexCAN.h>
static CAN_message_t msg, rxmsg;
String CANStr("");
int led = 13;

uint8_t no_data1 = 0;   // No data counter for Data 1
uint8_t no_data2 = 0;   // No data counter for Data 2

IntervalTimer data_timer;
unsigned char data[6];  // rxmsg size

void setup()
{
  Can0.begin(500000);                 //CAN speed is 500kbps
  pinMode(5, OUTPUT);                 // Low oil lamp
  pinMode(6, OUTPUT);                 // Low SOC lamp
  pinMode(9, OUTPUT);                 // MIL lamp
  pinMode(10, OUTPUT);                // Power limit lamp
  pinMode(11, OUTPUT);                // Charge lamp
  pinMode(12, OUTPUT);                // Hazard lamp
  pinMode(30, OUTPUT);                // Motor ON gauge
  pinMode(31, OUTPUT);                // Radiator fan LOW speed
  // 22, PWM OUT,                        DTE gauge half 120Hz
  // 23, PWM OUT,                        SOC gauge 25 to 188
  // 25, PWM OUT,                        ECO gauge 25 to 188
  pinMode(led, OUTPUT);
  analogWriteFrequency(23, 80);        // 20kHz (80Hz Postal)
  analogWriteFrequency(25, 80);        // 20kHz (80Hz Postal)
  digitalWrite(led, HIGH);
  delay(1000);
  Serial.println(F("InCar CAN Bus Rx Teensy 3.2 72Mhz ver.10"));
  digitalWrite(11, HIGH);       // Charge lamp
  digitalWrite(12, HIGH);       // Hazard lamp
  digitalWrite(10, HIGH);       // Power limit lamp
  digitalWrite(9, HIGH);        // MIL lamp
  digitalWrite(6, HIGH);        // Low SOC lamp (Only active in run)
  digitalWrite(5, HIGH);        // Low oil lamp
  delay(4000);
  digitalWrite(led, LOW);

  msg.rtr = 0 ;
  data_timer.begin(data_count, 1000000);  // Start no data interrupt counter
}

/* From Timer Interrupt */
void data_count(void)
{
  no_data1++;
  no_data2++;
}

void loop()
{

  while (Can0.read(rxmsg))
  {
    String CANStr("");
    for (int i = 0; i < 8; i++)
    {
      CANStr += String(rxmsg.buf[i], HEX);
      CANStr += (" ") ;
    }

    if (rxmsg.id == 0x700)
    {
      digitalWrite(led, HIGH);
      data[0] = rxmsg.buf[0];         // Charge lamp
      data[1] = rxmsg.buf[1];         // Hazard lamp
      data[2] = rxmsg.buf[2];         // Power limit lamp
      data[3] = rxmsg.buf[3];         // MIL lamp
      data[4] = rxmsg.buf[4];         // Low SOC lamp (Only active in run)
      data[5] = rxmsg.buf[5];         // Low oil lamp
      Serial.print(rxmsg.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();
      no_data1 = 0;
      if (data[0] == 1)
        digitalWrite(11, HIGH);       // Charge lamp
      else
        digitalWrite(11, LOW);
      if (data[1] == 1)
        digitalWrite(12, HIGH);       // Hazard lamp
      else
        digitalWrite(12, LOW);
      if (data[2] == 1)
        digitalWrite(10, HIGH);       // Power limit lamp
      else
        digitalWrite(10, LOW);
      if (data[3] == 1)
        digitalWrite(9, HIGH);        // MIL lamp
      else
        digitalWrite(9, LOW);
      if (data[4] == 1)
        digitalWrite(6, HIGH);        // Low SOC lamp (Only active in run)
      else
        digitalWrite(6, LOW);
      if (data[5] == 1)
        digitalWrite(5, HIGH);        // Low oil lamp
      else
        digitalWrite(5, LOW);
    }

    if (rxmsg.id == 0x701)
    {
      digitalWrite(led, HIGH);
      data[0] = rxmsg.buf[0];            // SOC gauge PWM 25 to 188 (half 118)
      data[1] = rxmsg.buf[1];            // DTE gauge 31-210Hz (half 120Hz)
      data[2] = rxmsg.buf[2];            // Motor ON gauge 0/1
      data[3] = rxmsg.buf[3];            // ECO gauge PWM 25 to 188 mid 118 (Only active in run)
      data[4] = rxmsg.buf[4];            // No cell temp to instrument cluster!
      data[5] = rxmsg.buf[5];            // Radiator LO fan relay
      Serial.print(rxmsg.id, HEX);
      Serial.print(' ');
      Serial.print(rxmsg.len, HEX);
      Serial.print(' ');
      Serial.print(CANStr);
      Serial.println ();
      no_data2 = 0;

      data[0] = map(data[0], 0, 100, 25, 188);
      analogWrite(23, data[0]);           // SOC gauge 25 to 188
      tone (22, data[1]);                 // DTE gauge half 120Hz
      if (data[2] == 1)
        digitalWrite(30, HIGH);           // Motor ON gauge
      else
        digitalWrite(30, LOW);
      analogWrite(25, data[3]);           // ECO gauge 25 to 188 (Only active in run)
      if (data[5] == 1)
        digitalWrite(31, HIGH);           // Radiator fan LOW speed
      else
        digitalWrite(31, LOW);
    }
  }

  digitalWrite(led, LOW);

  if (no_data1 > 2) //Check data still coming in within 2 second
  {
    Serial.print("700 ");
    Serial.println("---No data---");
    digitalWrite(9, HIGH);        // MIL lamp
    delay(750);
    digitalWrite(9, LOW);         // MIL lamp
    delay(750);
    no_data1 = 3;  // Prevent counter rollover
  }

  if (no_data2 > 2) //Check data still coming in within 2 second
  {
    Serial.print("701 ");
    Serial.println("---No data---");
    digitalWrite(30, LOW);        // Motor ON gauge
    digitalWrite(9, HIGH);        // MIL lamp
    delay(750);
    digitalWrite(9, LOW);         // MIL lamp
    delay(750);
    no_data2 = 3;  // Prevent counter rollover
  }
}
