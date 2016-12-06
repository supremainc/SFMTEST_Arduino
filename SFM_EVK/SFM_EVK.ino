#include <Adafruit_WS2801.h>

//#include <SoftwareSerial.h>

//#include <Adafruit_NeoPixel.h>
//#include <WS2812_Definitions.h>

//---------------------------------------------------------------------------
// neaMetrics - Test to see if Arduino Uno can connect to SFM3520
//
// Uses Software Serial so the the serial can still be monitered with
//  feedback.
//---------------------------------------------------------------------------
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "WS2812_Definitions.h"
#include "pitches.h"
//---------------------------------------------------------------------------
#define SFM_BAUD 19200
#define SFM_START_PACKET 0x40
#define SFM_STOP_PACKET  0x0A
#define SFM_ST_TEMPLATE_SCAN  0x11
#define COM_IS  0x11
#define COM_ES  0x05
#define COM_DA  0X17
#define COM_SW  0x01


#define LED_PIN 11
#define LED_COUNT 1

#define BUZZER_PIN 9

Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//command packets
byte IS[] = { SFM_START_PACKET, COM_IS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51, SFM_STOP_PACKET}; // Identify by Scan
byte ES[] = { SFM_START_PACKET, COM_ES, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xBE, SFM_STOP_PACKET}; // Enroll by Scan
byte DA[] = { SFM_START_PACKET, COM_DA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, SFM_STOP_PACKET}; // Delete all
byte SW_FREESCAN_ON[] = { SFM_START_PACKET, COM_SW, 0x00, 0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x84, 0xF6, SFM_STOP_PACKET}; // Free scan on
byte SW_FREESCAN_OFF[] = { SFM_START_PACKET, COM_SW, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x84, 0xF5, SFM_STOP_PACKET}; // Free scan off

// notes in the melody:
int melody[] = {
  NOTE_C7, NOTE_E7, NOTE_G7, NOTE_C8, NOTE_C8, NOTE_C8
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  8, 8, 8, 4 , 4, 8
};

SoftwareSerial SFM_Serial(3, 2); //, true);// RX, TX
SoftwareSerial lcd(4, 5);

//Actual Functions
void setup() {
  //push button for enroll
  pinMode(6, INPUT);

  //push button for identify
  pinMode(7, INPUT);

  //push button for delete
  pinMode(8, INPUT);


  //
  Serial.begin(115200); // Hardware serial (115200 baud)
  SFM_Serial.begin(SFM_BAUD); // SFM (19200 baud)
  lcd.begin(9600);  // LCD (9600 baud)

  InitDisplay();
  InitTone();
  
  leds.begin();  // Call this to start up the LED strip.
  leds.show();   // ...but the LEDs don't actually update until you call this.

  clearDisplay();  // Clear the display


}


void loop() {

  setLCDCursor(0);  // Set cursor to the 3rd spot, 1st line
  lcd.print("SFM for Arduino");

  bool button_enroll = digitalRead(6);
  bool button_identify = digitalRead(7);
  bool button_delete = digitalRead(8);
  static int freescan_mode = 0;

  if (!freescan_mode)
    leds.setPixelColor(0, BLUE);
  else
    leds.setPixelColor(0, VIOLET);
  leds.setBrightness(50);
  leds.show();

  cleanSerialBuffer();
  // put your main code here, to run repeatedly:



  //Serial.println(button_identify);

  if (button_enroll)
  {
    clearDisplay();
    setLCDCursor(0);
    lcd.print("Enroll");

    SFM_Serial.listen();

    int bytesSent = 0;
    bytesSent = SFM_Serial.write(ES, sizeof(ES));


    for (int i = 0; i < 2; i++)
    {
      leds.setPixelColor(0, ORANGE);
      leds.setBrightness(50);
      leds.show();
      delay(200);

      while (1)
      {

        if (SFM_Serial.available() > 0)
          break;
      }

      delay(200);

      byte rx_buffer[256];
      int count = 0;

      while (SFM_Serial.available() > 0)
      {
        rx_buffer[count] = SFM_Serial.read();

        count++;
      }
      cleanSerialBuffer();
      int response = rx_buffer[10];

      if (response == 0x62)
      {

        while (1)
        {
          if (SFM_Serial.available() > 0)
            break;
        }

        int count = 0;
        while (SFM_Serial.available() > 0) {

          //Serial.println("we got data back");
          //Serial.write(SFM_Serial.read());
          rx_buffer[count] = SFM_Serial.read();
          int v = rx_buffer[count];
          Serial.print(" ");
          Serial.print(v);
          count++;
        }
        cleanSerialBuffer();

        int response = rx_buffer[10];
        unsigned long userid_data[4];
        userid_data[0] = rx_buffer[2] ;
        userid_data[1] = rx_buffer[3] ;
        userid_data[2] = rx_buffer[4] ;
        userid_data[3] = rx_buffer[5] ;
        unsigned long userid = 0;
        userid = userid_data[3] << 24 | userid_data[2] << 16 | userid_data[1] << 8 | userid_data[0];

        unsigned long d = userid_data[0];

        if (response == 0x61)
        {
          clearDisplay();

          if (i == 0)
          {
            setLCDCursor(0);
            lcd.print("Enroll");

          }
          else if (i == 1)
          {
            setLCDCursor(0);
            lcd.print("Enroll Success");
            setLCDCursor(16);
            lcd.print("ID : ");
            lcd.print(userid);

          }

          leds.setPixelColor(0, GREEN);
          leds.setBrightness(50);
          leds.show();

          success();
          delay(1000);
          clearDisplay();

        }
        else
        {
//        delay(200);
          clearDisplay();
          setLCDCursor(0);
          lcd.print("Fail");

          leds.setPixelColor(0, RED);
          leds.setBrightness(50);
          leds.show();

          fail();

          delay(1000);
          clearDisplay();
        }

      }
      else
      {
        clearDisplay();
        setLCDCursor(0);
        lcd.print("Enroll Fail");

        setLCDCursor(16);
        lcd.print("Err Code : ");
        lcd.print(response, HEX);

        leds.setPixelColor(0, RED);
        leds.setBrightness(50);
        leds.show();

        fail();

        delay(1000);
        clearDisplay();
        break;
      }
    }
  }


  else if (button_identify)
  {
    
    
    clearDisplay();
    setLCDCursor(0);
    lcd.print("Identify");

    SFM_Serial.listen();

    int bytesSent = 0;
    bytesSent = SFM_Serial.write(IS, sizeof(IS));

    //serial monitor message
    Serial.print(bytesSent);
    Serial.println(" bytes send");
    Serial.println("_________________________");

    while (1)
    {
      leds.setPixelColor(0, ORANGE);
      leds.setBrightness(50);
      leds.show();
      delay(200);

      if (SFM_Serial.available() > 0)
        break;
    }

    delay(200);

    //int a = SFM_Serial.available();
    //  Serial.print(a);

    byte rx_buffer[256];
    int count = 0;

    while (SFM_Serial.available() > 0) {

      //Serial.println("we got data back");
      //Serial.write(SFM_Serial.read());
      rx_buffer[count] = SFM_Serial.read();
      int v = rx_buffer[count];
      // Serial.print(" ");
      // Serial.print(v);
      count++;
    }
    cleanSerialBuffer();
    int response = rx_buffer[10];

    //serial monitor message
    // Serial.println("count : ");
    // Serial.print(count);
    // Serial.println("");
    // Serial.println("return : ");
    // Serial.print(response, HEX);
    // Serial.println("");

    if (response == 0x62)
    {

      while (1)
      {
        if (SFM_Serial.available() > 0)
          break;
      }

      int count = 0;
      while (SFM_Serial.available() > 0) {

        //Serial.println("we got data back");
        //Serial.write(SFM_Serial.read());
        rx_buffer[count] = SFM_Serial.read();
        int v = rx_buffer[count];
        Serial.print(" ");
        Serial.print(v);
        count++;
      }
      cleanSerialBuffer();

      int response = rx_buffer[10];
      unsigned long userid_data[4];
      userid_data[0] = rx_buffer[2] ;
      userid_data[1] = rx_buffer[3] ;
      userid_data[2] = rx_buffer[4] ;
      userid_data[3] = rx_buffer[5] ;
      unsigned long userid = 0;
      userid = userid_data[3] << 24 | userid_data[2] << 16 | userid_data[1] << 8 | userid_data[0];

      // Serial.println("------");
      // Serial.print(userid_data[0], HEX);
      // Serial.print(userid_data[1], HEX);
      // Serial.print(userid_data[2], HEX);
      // Serial.print(userid_data[3], HEX);
      // Serial.println("------");
      unsigned long d = userid_data[0];
      Serial.println(userid_data[0] , BIN);
      Serial.println(d , BIN);
      if (response == 0x61)
      {
        clearDisplay();
        setLCDCursor(0);
        lcd.print("Identify Success");
        setLCDCursor(16);
        lcd.print("ID : ");
        lcd.print(userid);

        leds.setPixelColor(0, GREEN);
        leds.setBrightness(50);
        leds.show();

        success();
        delay(1000);
        clearDisplay();

      }
      else
      {
//        delay(200);
        clearDisplay();
        setLCDCursor(0);
        lcd.print("Fail");

        leds.setPixelColor(0, RED);
        leds.setBrightness(50);
        leds.show();

        fail();

        delay(1000);
        clearDisplay();
      }

    }
    else
    {
      clearDisplay();
      setLCDCursor(0);
      lcd.print("Idendity Fail");

      setLCDCursor(16);
      lcd.print("Err Code : ");
      lcd.print(response, HEX);

      leds.setPixelColor(0, RED);
      leds.setBrightness(50);
      leds.show();

      fail();

      delay(1000);
      clearDisplay();
    }


  }
  else if (button_delete)
  {
    clearDisplay();
    setLCDCursor(0);
    lcd.print("Delete All");

    SFM_Serial.listen();

    int bytesSent = 0;
    bytesSent = SFM_Serial.write(DA, sizeof(DA));

    while (1)
    {
      leds.setPixelColor(0, ORANGE);
      leds.setBrightness(50);
      leds.show();
      delay(200);

      if (SFM_Serial.available() > 0)
        break;
    }

    delay(200);

    byte rx_buffer[256];
    int count = 0;

    while (SFM_Serial.available() > 0) {

      //Serial.println("we got data back");
      //Serial.write(SFM_Serial.read());
      rx_buffer[count] = SFM_Serial.read();
      int v = rx_buffer[count];
      // Serial.print(" ");
      // Serial.print(v);
      count++;
    }
    cleanSerialBuffer();

    int response = rx_buffer[10];


    if (response == 0x61)
    {
      delay(1000);

      clearDisplay();
      setLCDCursor(0);
      lcd.print("Delete Success");

      leds.setPixelColor(0, GREEN);
      leds.setBrightness(50);
      leds.show();

      success();
      delay(1000);
      clearDisplay();

    }
  }
}

void cleanSerialBuffer()
{
  while (SFM_Serial.available() > 0) {
    SFM_Serial.read();
  }
}

void success()
{
  for (int thisNote = 0; thisNote < 3; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(9, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(9);
  }

}

void fail()
{
  for (int thisNote = 5; thisNote >= 4; thisNote--) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(9, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(9);
  }


}

void initLCD()
{


  clearDisplay();  // Clear the display
  setLCDCursor(0);  // Set cursor to the 3rd spot, 1st line
  lcd.print("  Suprema Inc.  ");
  setLCDCursor(16);  // Set the cursor to the beginning of the 2nd line
  lcd.print("Initializing....");

  // Flash the backlight:
  for (int i = 0; i < 3; i++)
  {
    setBacklight(0);
    delay(250);
    setBacklight(255);
    delay(250);
  }
}
void setBacklight(byte brightness)
{
  lcd.write(0x80);  // send the backlight command
  lcd.write(brightness);  // send the brightness value
}

void clearDisplay()
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x01);  // send the clear screen command
}

void setLCDCursor(byte cursor_position)
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x80);  // send the set cursor command
  lcd.write(cursor_position);  // send the cursor position
}


void InitDisplay()
{
  clearDisplay();  // Clear the display
  setLCDCursor(0);  // Set cursor to the 3rd spot, 1st line
  lcd.print("  Suprema Inc.  ");
  setLCDCursor(16);  // Set the cursor to the beginning of the 2nd line
  lcd.print("Initializing....");

  // Flash the backlight:
  for (int i = 0; i < 3; i++)
  {
    setBacklight(0);
    delay(250);
    setBacklight(255);
    delay(250);
  }

}

void InitTone()
{
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 4; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BUZZER_PIN);
  }
}

