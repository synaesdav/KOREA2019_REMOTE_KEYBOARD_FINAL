//Korea 2019 expanded menu version
//added working code for sound reactive capabilities
//template is mostly filled
//still using weird fix for unexplained non compiling

//radio remote controller with oled display and trellis keypad
//higher transmition power used for greater reliability
//8 menus of 16 buttons
//advanced version has specific keystrokes sent depending on button pressed
//sends numerical values of ascii characters

//modified from https://learn.adafruit.com/remote-effects-trigger/overview-1?view=all
//Ada_remoteFXTrigger_TX
//Remote Effects Trigger Box Transmitter
//by John Park
//for Adafruit Industries
//MIT License

//POST GRAMERCY 2019 version

//written by David Crittenden 7/2018
//updated 6/2019

#include <Keyboard.h>//library allows remote to act as a keyboard over usb
#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_Trellis.h"
#include <Encoder.h>

//added lit top button
#define LITPIN  A3
#define LITBUTTON 6
bool lockStatus;
bool buttonValue;

/********* Encoder Setup ***************/
#define PIN_ENCODER_SWITCH 11
Encoder knob(10, 12);
uint8_t activeRow = 0;
long pos = -999;
long newpos;
int prevButtonState = HIGH;
bool needsRefresh = true;
bool advanced = false;
unsigned long startTime;

/********* Trellis Setup ***************/
#define MOMENTARY 0
#define LATCHING 1
#define MODE LATCHING //all Trellis buttons in latching mode
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis =  Adafruit_TrellisSet(&matrix0);
#define NUMTRELLIS 1
#define numKeys (NUMTRELLIS * 16)
#define INTPIN A2

/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();
#if defined(ESP8266)
#define BUTTON_A 0
#define BUTTON_B 16
#define BUTTON_C 2
#define LED      0
#elif defined(ESP32)
#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14
#define LED      13
#elif defined(ARDUINO_STM32F2_FEATHER)
#define BUTTON_A PA15
#define BUTTON_B PC7
#define BUTTON_C PC5
#define LED PB5
#elif defined(TEENSYDUINO)
#define BUTTON_A 4
#define BUTTON_B 3
#define BUTTON_C 8
#define LED 13
#elif defined(ARDUINO_FEATHER52)
#define BUTTON_A 31
#define BUTTON_B 30
#define BUTTON_C 27
#define LED 17
#else // 32u4, M0, and 328p
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
#define LED      13
#endif

/************ Radio Setup ***************/
// Can be changed to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#endif

#if defined (ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#endif

#if defined (ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#endif

//NEW CODE (wasn't choosing from the feather MO if endif)
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int lastButton = 17; //last button pressed for Trellis logic
char radiopacket[20];//stores information sent

int menuList[8] = {1, 2, 3, 4, 5, 6, 7, 8}; //for rotary encoder choices
int m = 0; //variable to increment through menu list
int lastTB[8] = {16, 16, 16, 16, 16, 16, 16, 16}; //array to store per-menu Trellis button

//for battery level
#define VBATPIN A7

/*******************SETUP************/
void setup()
{
  delay(500);
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  Keyboard.begin();//for keyboard emulation

  pinMode(LITPIN, OUTPUT);//for top LED button to light up
  pinMode(LITBUTTON, INPUT_PULLUP);//for top LED button press
  analogWrite(LITPIN, 255);//turn LED button on

  // INT pin on Trellis requires a pullup
  pinMode(INTPIN, INPUT);
  digitalWrite(INTPIN, HIGH);
  trellis.begin(0x70);

  pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);//set encoder push switch pin to input pullup

  digitalPinToInterrupt(10); //on M0, Encoder library doesn't auto set these as interrupts
  digitalPinToInterrupt(12);

  // Initialize OLED display
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.setTextWrap(false);
  oled.clearDisplay();
  oled.display();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  // pinMode(BUTTON_A, INPUT_PULLUP);
  // pinMode(BUTTON_B, INPUT_PULLUP);
  // pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init())
  {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ))
  {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);//originally 14

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  // light up all the LEDs in order
  for (uint8_t i = 0; i < numKeys; i++)
  {
    trellis.setLED(i);
    trellis.writeDisplay();
    delay(25);
  }

  oled.clearDisplay();
  oled.setCursor(12, 0);
  oled.println("CRISTIANO");
  oled.setCursor(0, 18);
  oled.println("DANCE MUSIC");
  oled.display();
  delay(1100); //pause to let message be read by a human

  buttonValue = digitalRead(LITBUTTON);//check to see if LED button is being pressed (active LOW)
  if (buttonValue == HIGH)//unlocked mode, light turns off
  {
    analogWrite(LITPIN, 0);//turn LED button off
    lockStatus = false;
  }
  else if (buttonValue == LOW)//performance mode, light remains on
  {
    analogWrite(LITPIN, 255);//turn LED button on
    lockStatus = true;
  }

  // then turn them off
  for (uint8_t i = 0; i < numKeys; i++)
  {
    trellis.clrLED(i);
    trellis.writeDisplay();
    delay(25);
  }

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print("2019 SEOUL");
  oled.setCursor(0, 18);
  oled.print("WEBFEST1 2 ");
  oled.display();
  delay (1500);

  //check the battery level
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  measuredvbat *= 100;    // to move the decimal place
  int batteryPercent = map(measuredvbat, 320, 420, 0, 100);//operating voltage ranges from 3.2 - 4.2

  oled.clearDisplay();
  oled.setCursor(12, 0);
  oled.print("Batt ");
  oled.print(batteryPercent);
  oled.println("% ");
  oled.setCursor(0, 18);
  oled.print(measuredvbat / 100);
  oled.println(" volts  ");
  oled.display();
  delay(2000); //pause to let message be read by a human

  m = 7; //Set this to make it display the main menu first
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

void loop()
{
  //debug LED button state
  //buttonValue = digitalRead(LITBUTTON);
  //Serial.println(buttonValue);

  delay(30); // 30ms delay is required, dont remove me! (Trellis)


  //NEW CODE check lockStatus to see what mode we are in
  if (lockStatus == true)//performance mode
  {
    m = 0;
    if (newpos != pos)
    {
      //clear Trellis lights
      for (int t = 0; t <= 16; t++)
      {
        trellis.clrLED(t);
        trellis.writeDisplay();
      }
      //light last saved light for current menu
      trellis.clrLED(lastTB[m]);
      trellis.setLED(lastTB[m]);
      trellis.writeDisplay();

      //write to the display
      oled.setCursor(0, 0);
      oled.clearDisplay();
      oled.print("PERFORMANCE");
      oled.setCursor(42, 18);
      oled.print("MODE");
      //oled.drawLine(0, 31, 127, 31, WHITE);
      oled.display();

      pos = newpos;
    }
  }
  else if (lockStatus == false)//unlocked mode
  {
    //check the encoder knob, set the current position as origin
    newpos = knob.read() / 4;//divide for encoder detents

    /* // for debugging
      Serial.print("pos=");
      Serial.print(pos);
      Serial.print(", newpos=");
      Serial.println(newpos);
    */
    if (newpos != pos)
    {
      int diff = newpos - pos;//check the different between old and new position
      if (diff >= 1)
      {
        m++;
        m = (m + 8) % 8; //modulo to roll over the m variable through the list size
      }

      if (diff == -1)//rotating backwards
      {
        m--;
        m = (m + 8) % 8;
      }
      /* //uncomment for debugging or general curiosity
        Serial.print("Diff = ");
        Serial.print(diff);
        Serial.print("  pos= ");
        Serial.print(pos);
        Serial.print(", newpos=");
        Serial.println(newpos);
        Serial.println(menuList[m]);
      */
      pos = newpos;

      // Serial.print("m is: ");
      //Serial.println(m);

      //clear Trellis lights
      for (int t = 0; t <= 16; t++)
      {
        trellis.clrLED(t);
        trellis.writeDisplay();
      }
      //light last saved light for current menu
      trellis.clrLED(lastTB[m]);
      trellis.setLED(lastTB[m]);
      trellis.writeDisplay();

      //write to the display
      oled.setCursor(0, 3);
      oled.clearDisplay();

      if (m == 0)
      {
        oled.setCursor(0, 0);
        oled.print("Main Menu");
      }

      if (m == 1)
      {
        oled.setCursor(0, 0);
        oled.print("Menu 2");
      }
      if (m == 2)
      {
        oled.setCursor(0, 0);
        oled.print("Menu 3");
      }
      if (m == 3)
      {
        oled.setCursor(0, 0);
        oled.print("Menu 4");
      }
      if (m == 4)
      {
        oled.setCursor(0, 0);
        oled.print("Menu 5");
      }
      if (m == 5)
      {
        oled.setCursor(0, 0);
        oled.print("Menu 6");
      }
      if (m == 6)
      {
        oled.setCursor(0, 0);
        oled.print("SOUND");
        oled.setCursor(0, 18);
        oled.print("REACTIVE");
      }
      if (m == 7)
      {
        oled.setCursor(0, 0);
        oled.print("Text/Logo");
        oled.setCursor(0, 18);
        oled.print("Crawl");
      }

      oled.display();
    }
  }

  // remember that the switch is active low
  int buttonState = digitalRead(PIN_ENCODER_SWITCH);
  if (buttonState == LOW)//button has been pressed
  {
    unsigned long now = millis();
    if (prevButtonState == HIGH)
    {
      prevButtonState = buttonState;
      startTime = now;
      Serial.println("button pressed");

      //send trellis selection
      Serial.print("Sending ");
      Serial.println(radiopacket[0]);
      rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf69.waitPacketSent();

      //NEW CODE
      //send a keyboard command to the computer depending on which button was pressed
      if (m == 0)//sends different values for each button of each menu
      {
        switch (radiopacket[0])
        {
          case 1://ALWAYS AWAY - NO INTRO
            Keyboard.print("1");//send message to computer
            delay(20);
            Keyboard.print(" ");//send message to
            break;
          case 2://EVERYTHING IS BEAUTIFUL - NO INTRO
            Keyboard.print("2");//send message to computer
            delay(20);
            Keyboard.print(" ");//send message to
            break;
          case 3://INFERNO - NO INTRO
            Keyboard.print("3");//send message to computer
            delay(20);
            Keyboard.print(" ");//send message to
            break;
          case 4://ALL AROUND YOU - WITH 8 BEAT COUNT IN
            Keyboard.print("4");//send message to computer
            delay(20);
            Keyboard.print(" ");//send message to computer
            break;
          case 5://RADIO GAGA - NO INTRO
            Keyboard.print("5");//send message to computer
            delay(20);
            Keyboard.print(" ");//send message to
            break;
          case 6://BEAUTIFUL - NO INTRO
            Keyboard.print("6");//send message to computer
            delay(20);
            Keyboard.print(" ");//send message to
            break;
          /* case 16:
             Keyboard.print(" ");//send message to
             break;*/
          default:
            break;
        }
      }
      if (m == 1)//Second Menu
      {
        switch (radiopacket[0])

        default:
        break;
      }
      if (m == 2)//Third Menu
      {
        switch (radiopacket[0])

        default:
        break;
      }
      if (m == 3)//Fourth Menu
      {
        switch (radiopacket[0])

        default:
        break;
      }
      if (m == 4)//Fifth Menu
      {
        switch (radiopacket[0])

        default:
        break;
      }
      if (m == 5)//Sixth Menu
      {
        switch (radiopacket[0])

        default:
        break;
      }
      if (m == 6)//Seventh Menu
      {
        switch (radiopacket[0])

        default:
        break;
      }
      if (m == 7)//Eighth Menu
      {
        switch (radiopacket[0])

        default:
        break;
      }
    }
  }
  else if (buttonState == HIGH && prevButtonState == LOW)
  {
    //Serial.println("button released!");
    prevButtonState = buttonState;
  }

  /*************Trellis Button Presses***********/
  if (MODE == LATCHING)
  {
    if (trellis.readSwitches()) { // If a button was just pressed or released...
      for (uint8_t i = 0; i < numKeys; i++) { // go through every button
        if (trellis.justPressed(i)) { // if it was pressed...
          //Serial.print("v"); Serial.println(i);

          if (i != lastTB[m]) {
            if (trellis.isLED(i)) {
              trellis.clrLED(i);
              lastTB[m] = i; //set the stored value for menu changes
            }
            else {
              trellis.setLED(i);
              //trellis.clrLED(lastButton);//turn off last one
              trellis.clrLED(lastTB[m]);
              lastTB[m] = i; //set the stored value for menu changes
            }
            trellis.writeDisplay();
          }


          //check the rotary encoder menu choice
          if (m == 0)//first menu item: PERFORMANCE MODE
          {
            if (i == 0)
            {
              radiopacket[0] = 1;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("ALWAYS");
              oled.setCursor(0, 18);
              oled.print("AWAY");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 2;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EVERYTHING");
              oled.setCursor(0, 18);
              oled.print("UNDER..SUN");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 3;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("INFERNO");
              oled.display();
            }

            if (i == 3)
            {
              radiopacket[0] = 4;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("ALL AROUND");
              oled.setCursor(0, 18);
              oled.print("YOU");
              oled.display();
            }

            if (i == 4)
            {
              radiopacket[0] = 5;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("RADIO GAGA");
              oled.display();
            }

            if (i == 5)
            {
              radiopacket[0] = 6;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("BEAUTIFUL");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 7;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Sweep");
              oled.setCursor(0, 18);
              oled.print("Heart 1");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 8;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SOUND");
              oled.setCursor(0, 17);
              oled.print("REACTIVE 1");
              oled.display();
            }

            if (i == 8)
            {
              radiopacket[0] = 9;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Text Scroll");
              oled.setCursor(0, 18);
              oled.print("Cristiano Dance Music");
              oled.display();
            }

            if (i == 9)
            {
              radiopacket[0] = 10;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Heart");
              oled.setCursor(0, 18);
              oled.print("Fade");
              oled.display();
            }

            if (i == 10)
            {
              radiopacket[0] = 11;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Circle");
              oled.setCursor(0, 18);
              oled.print("Droplet");
              oled.display();
            }

            if (i == 11)
            {
              radiopacket[0] = 12;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Shock Wave");
              oled.setCursor(0, 17);
              oled.print("Multi-Color");
              oled.display();
            }

            if (i == 12)
            {
              radiopacket[0] = 13;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Text Scroll");
              oled.setCursor(0, 18);
              oled.print("FUCK TRUMP");
              oled.display();
            }

            if (i == 13)
            {
              radiopacket[0] = 14;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rainbow");
              oled.setCursor(0, 18);
              oled.print("Checkers");
              oled.display();
            }

            if (i == 14)
            {
              radiopacket[0] = 15;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Red-White");
              oled.setCursor(0, 18);
              oled.print("Heart");
              oled.display();
            }

            if (i == 15)
            {
              radiopacket[0] = 16;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }

          if (m == 1)//Menu 2
          {
            if (i == 0)
            {
              radiopacket[0] = 17;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Spin Square");
              oled.setCursor(0, 18);
              oled.print("Multi Color");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 18;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Spin Square");
              oled.setCursor(0, 18);
              oled.print("Rainbow");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 19;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Spin Square");
              oled.setCursor(0, 18);
              oled.print("Sine 1");
              oled.display();
            }
            if (i == 3)
            {
              radiopacket[0] = 20;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Spin Square");
              oled.setCursor(0, 18);
              oled.print("Sine 2");
              oled.display();
            }
            if (i == 4)
            {
              radiopacket[0] = 21;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Sweep");
              oled.setCursor(0, 18);
              oled.print("Multi");
              oled.display();
            }
            if (i == 5)
            {
              radiopacket[0] = 22;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Sweep");
              oled.setCursor(0, 18);
              oled.print("Rainbow");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 23;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Sweep");
              oled.setCursor(0, 18);
              oled.print("Heart 1");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 24;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Sweep");
              oled.setCursor(0, 18);
              oled.print("Heart 2");
              oled.display();
            }
            if (i == 8)
            {
              radiopacket[0] = 25;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Droplet");
              oled.setCursor(0, 18);
              oled.print("Single");
              oled.display();
            }
            if (i == 9)
            {
              radiopacket[0] = 26;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Droplet");
              oled.setCursor(0, 18);
              oled.print("Multi");
              oled.display();
            }
            if (i == 10)
            {
              radiopacket[0] = 27;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Droplet");
              oled.setCursor(0, 18);
              oled.print("Rainbow 1");
              oled.display();
            }
            if (i == 11)
            {
              radiopacket[0] = 28;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Droplet");
              oled.setCursor(0, 18);
              oled.print("Rainbow 2");
              oled.display();
            }
            if (i == 12)
            {
              radiopacket[0] = 29;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Droplet");
              oled.setCursor(0, 18);
              oled.print("Heart");
              oled.display();
            }
            if (i == 13)
            {
              radiopacket[0] = 30;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Droplet");
              oled.setCursor(0, 17);
              oled.print("Diagonal");
              oled.display();
            }
            if (i == 14)
            {
              radiopacket[0] = 31;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Droplet");
              oled.setCursor(0, 18);
              oled.print("Rainbow 3");
              oled.display();
            }
            if (i == 15)
            {
              radiopacket[0] = 32;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }
          if (m == 2)//next menu item
          {
            if (i == 0)
            {
              radiopacket[0] = 33;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 18);
              oled.print("Rainbow");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 34;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 18);
              oled.print("Heart multi");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 35;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 18);
              oled.print("Scatter");
              oled.display();
            }
            if (i == 3)
            {
              radiopacket[0] = 36;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 17);
              oled.print("Diagonal");
              oled.display();
            }
            if (i == 4)
            {
              radiopacket[0] = 37;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 17);
              oled.print("Stutter 1");
              oled.display();
            }
            if (i == 5)
            {
              radiopacket[0] = 38;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 17);
              oled.print("Stutter 2");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 39;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 17);
              oled.print("Stutter 3");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 40;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rectangle");
              oled.setCursor(0, 17);
              oled.print("Stutter 4");
              oled.display();
            }
            if (i == 8)
            {
              radiopacket[0] = 41;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Pixel");
              oled.setCursor(0, 17);
              oled.print("Sprinkle");
              oled.display();
            }
            if (i == 9)
            {
              radiopacket[0] = 42;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Pixel");
              oled.setCursor(0, 17);
              oled.print("Multi Color");
              oled.display();
            }
            if (i == 10)
            {
              radiopacket[0] = 43;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Pixel");
              oled.setCursor(0, 17);
              oled.print("Rainbow");
              oled.display();
            }
            if (i == 11)
            {
              radiopacket[0] = 44;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Pixel");
              oled.setCursor(0, 17);
              oled.print("Heart");
              oled.display();
            }
            if (i == 12)
            {
              radiopacket[0] = 45;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Triangle");
              oled.setCursor(0, 17);
              oled.print("Multi");
              oled.display();
            }
            if (i == 13)
            {
              radiopacket[0] = 46;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Triangle");
              oled.setCursor(0, 17);
              oled.print("Rainbow");
              oled.display();
            }
            if (i == 14)
            {
              radiopacket[0] = 47;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Triangle");
              oled.setCursor(0, 17);
              oled.print("Heart");
              oled.display();
            }
            if (i == 15)
            {
              radiopacket[0] = 48;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }
          if (m == 3)//next menu item
          {
            if (i == 0)
            {
              radiopacket[0] = 49;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Box March");
              oled.setCursor(0, 17);
              oled.print("Multi Color");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 50;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Box March");
              oled.setCursor(0, 17);
              oled.print("Heart");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 51;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Box March");
              oled.setCursor(0, 17);
              oled.print("Multi 2");
              oled.display();
            }
            if (i == 3)
            {
              radiopacket[0] = 52;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Box March");
              oled.setCursor(0, 17);
              oled.print("Rainbow  ");
              oled.display();
            }
            if (i == 4)
            {
              radiopacket[0] = 53;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rainbow");
              oled.setCursor(0, 17);
              oled.print("Solid");
              oled.display();
            }
            if (i == 5)
            {
              radiopacket[0] = 54;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rainbow");
              oled.setCursor(0, 17);
              oled.print("Slant Thin");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 55;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rainbow");
              oled.setCursor(0, 17);
              oled.print("Slant Thick");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 56;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Rainbow");
              oled.setCursor(0, 17);
              oled.print("Checkers");
              oled.display();
            }
            if (i == 8)
            {
              radiopacket[0] = 57;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Plaid Thin");
              oled.setCursor(0, 17);
              oled.print("Multi");
              oled.display();
            }
            if (i == 9)
            {
              radiopacket[0] = 58;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Plaid Thin");
              oled.setCursor(0, 17);
              oled.print("Rainbow");
              oled.display();
            }
            if (i == 10)
            {
              radiopacket[0] = 59;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Plaid Thick");
              oled.setCursor(0, 17);
              oled.print("Multi");
              oled.display();
            }
            if (i == 11)
            {
              radiopacket[0] = 60;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Plaid Thick");
              oled.setCursor(0, 17);
              oled.print("Rainbow");
              oled.display();
            }
            if (i == 12)
            {
              radiopacket[0] = 61;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Plaid");
              oled.setCursor(0, 17);
              oled.print("Thin Slant");
              oled.display();
            }
            if (i == 13)
            {
              radiopacket[0] = 62;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Plaid");
              oled.setCursor(0, 17);
              oled.print("Thick Slant");
              oled.display();
            }
            if (i == 14)
            {
              radiopacket[0] = 63;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Plaid");
              oled.setCursor(0, 17);
              oled.print("Checkers");
              oled.display();
            }
            if (i == 15)
            {
              radiopacket[0] = 64;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }

          if (m == 4)//next menu item
          {
            if (i == 0)
            {
              radiopacket[0] = 65;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SixtyFive");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 66;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SixtySix");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 67;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SixtySeven");
              oled.display();
            }
            if (i == 3)
            {
              radiopacket[0] = 68;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SixtyEight");
              oled.display();
            }
            if (i == 4)
            {
              radiopacket[0] = 69;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SixtyNine");
              oled.display();
            }
            if (i == 5)
            {
              radiopacket[0] = 70;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Seventy");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 71;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventyOne");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 72;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventyTwo");
              oled.display();
            }
            if (i == 8)
            {
              radiopacket[0] = 73;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventyThree");
              oled.display();
            }
            if (i == 9)
            {
              radiopacket[0] = 74;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventyFour");
              oled.display();
            }
            if (i == 10)
            {
              radiopacket[0] = 75;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventyFive");
              oled.display();
            }
            if (i == 11)
            {
              radiopacket[0] = 76;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventySix");
              oled.display();
            }
            if (i == 12)
            {
              radiopacket[0] = 77;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventySeven");
              oled.display();
            }
            if (i == 13)
            {
              radiopacket[0] = 78;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventyEight");
              oled.display();
            }
            if (i == 14)
            {
              radiopacket[0] = 79;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SeventyNine");
              oled.display();
            }
            if (i == 15)
            {
              radiopacket[0] = 80;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }
          if (m == 5)//next menu item
          {
            if (i == 0)
            {
              radiopacket[0] = 81;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightyOne");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 82;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightTwo");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 83;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightyThree");
              oled.display();
            }
            if (i == 3)
            {
              radiopacket[0] = 84;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightyFour");
              oled.display();
            }
            if (i == 4)
            {
              radiopacket[0] = 85;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightyFive");
              oled.display();
            }
            if (i == 5)
            {
              radiopacket[0] = 86;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightySix");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 87;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightySeven");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 88;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightyEight");
              oled.display();
            }
            if (i == 8)
            {
              radiopacket[0] = 89;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("EightyNine");
              oled.display();
            }
            if (i == 9)
            {
              radiopacket[0] = 90;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Ninety");
              oled.display();
            }
            if (i == 10)
            {
              radiopacket[0] = 91;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("NinetyOne");
              oled.display();
            }
            if (i == 11)
            {
              radiopacket[0] = 92;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("NinetyTwo");
              oled.display();
            }
            if (i == 12)
            {
              radiopacket[0] = 93;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("NinetyThree");
              oled.display();
            }
            if (i == 13)
            {
              radiopacket[0] = 94;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("NinetyFour");
              oled.display();
            }
            if (i == 14)
            {
              radiopacket[0] = 95;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("NinetyFive");
              oled.display();
            }
            if (i == 15)
            {
              radiopacket[0] = 96;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }
          if (m == 6)//SOUND REACTIVE
          {
            if (i == 0)
            {
              radiopacket[0] = 97;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SOUND");
              oled.setCursor(0, 17);
              oled.print("REACTIVE 1");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 98;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SOUND");
              oled.setCursor(0, 17);
              oled.print("Heart");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 99;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SOUND");
              oled.setCursor(0, 17);
              oled.print("Circle");
              oled.display();
            }
            if (i == 3)
            {
              radiopacket[0] = 100;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SOUND");
              oled.setCursor(0, 17);
              oled.print("CircleHeart");
              oled.display();
            }
            if (i == 4)
            {
              radiopacket[0] = 101;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0One");
              oled.display();
            }
            if (i == 5)
            {
              radiopacket[0] = 102;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Two");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 103;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Three");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 104;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Four");
              oled.display();
            }
            if (i == 8)
            {
              radiopacket[0] = 105;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Five");
              oled.display();
            }
            if (i == 9)
            {
              radiopacket[0] = 106;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Six");
              oled.display();
            }
            if (i == 10)
            {
              radiopacket[0] = 107;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Seven");
              oled.display();
            }
            if (i == 11)
            {
              radiopacket[0] = 108;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Eight");
              oled.display();
            }
            if (i == 12)
            {
              radiopacket[0] = 109;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("One0Nine");
              oled.display();
            }
            if (i == 13)
            {
              radiopacket[0] = 110;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("OneTen");
              oled.display();
            }
            if (i == 14)
            {
              radiopacket[0] = 111;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("OneEleven");
              oled.display();
            }
            if (i == 15)
            {
              radiopacket[0] = 112;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }
          if (m == 7)//next menu item
          {
            if (i == 0)
            {
              radiopacket[0] = 113;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("ALL AROUND");
              oled.setCursor(0, 17);
              oled.print("lyrics");
              oled.display();
            }
            if (i == 1)
            {
              radiopacket[0] = 114;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("BEAUTIFUL");
              oled.setCursor(0, 17);
              oled.print("lyrics");
              oled.display();
            }
            if (i == 2)
            {
              radiopacket[0] = 115;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("RADIO");
              oled.setCursor(0, 17);
              oled.print("logo");
              oled.display();
            }
            if (i == 3)
            {
              radiopacket[0] = 116;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SUN UP");
              oled.setCursor(0, 17);
              oled.print("logo");
              oled.display();
            }
            if (i == 4)
            {
              radiopacket[0] = 117;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Sun Rise");
              oled.setCursor(0, 17);
              oled.print("animation");
              oled.display();
            }
            if (i == 5)
            {
              radiopacket[0] = 118;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Heart Row");
              oled.setCursor(0, 17);
              oled.print("animation");
              oled.display();
            }
            if (i == 6)
            {
              radiopacket[0] = 119;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Wave 1");
              oled.setCursor(0, 17);
              oled.print("animation");
              oled.display();
            }
            if (i == 7)
            {
              radiopacket[0] = 120;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Wave 2");
              oled.setCursor(0, 17);
              oled.print("animation");
              oled.display();
            }
            if (i == 8)
            {
              radiopacket[0] = 121;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Cristiano");
              oled.setCursor(0, 18);
              oled.print("Dance Music");
              oled.display();
            }
            if (i == 9)
            {
              radiopacket[0] = 122;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("SEOUL");
              oled.setCursor(0, 18);
              oled.print("Webfest");
              oled.display();
            }
            if (i == 10)
            {
              radiopacket[0] = 123;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("NOTE");
              oled.setCursor(0, 17);
              oled.print("logo");
              oled.display();
            }
            if (i == 11)
            {
              radiopacket[0] = 124;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("HEART BEAT");
              oled.setCursor(0, 17);
              oled.print("logo");
              oled.display();
            }
            if (i == 12)
            {
              radiopacket[0] = 125;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("FUCK");
              oled.setCursor(0, 18);
              oled.print("TRUMP");
              oled.display();
            }
            if (i == 13)
            {
              radiopacket[0] = 126;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("Red-White");
              oled.setCursor(0, 18);
              oled.print("Heart");
              oled.display();
            }
            if (i == 14)
            {
              radiopacket[0] = 127;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("OneTwentySeven");
              oled.display();
            }
            if (i == 15)
            {
              radiopacket[0] = 128;
              oled.clearDisplay();
              oled.setCursor(0, 0);
              oled.print("CLEAR");
              oled.setCursor(0, 18);
              oled.print("ALL");
              oled.display();
            }
          }
        }
      }
    }
  }
}
