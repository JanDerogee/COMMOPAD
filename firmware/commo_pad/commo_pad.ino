/*
 * 
 * This program reads a simple 4-wire resistive touchpad
 * it converts the movements detected on the touchpad into mouse movements
 * therefore this pad can act as a mousepad for drawing purposes.
 * For accuracy and ease of use, these screens work best when used with a blunt object like a plastic stylus
 * When used with fingers only, the system seems to behave less smooth
 * 
 * This device is based on an Arduino pro micro, which has native USB functionality
 * and can be made to act like a HID device using the existing arduino library Mouse.h
 * 
 * 
 *  1 REM This program views the X/Y-values of the paddles (a.k.a. as POTs). (Control-Port1)
 *  10 POKE 56322, 224 :REM Keyboard deactivated
 *  20 PRINT "Paddle port1 X-value =" ; PEEK(54297) :REM read value X
 *  30 PRINT "Paddle port1 Y-value =" ; PEEK(54298) :REM read value Y
 *  40 POKE 56322, 255 :REM Keyboard activated
 * 
 * simplified:
 *  10 PRINT PEEK(54297);
 *  20 PRINT PEEK(54298):GOTO 10
*/

#include <stdint.h>

#include <EEPROM.h>         /*use the EEPROM functionality of the 32u4 in the Arduino pro micro*/

#include "TouchScreen.h"    /*library for usage of a simple 4-wire touchscreen, connect to only 4 IO-pins*/
#include "Mouse.h"          /*library to make the Arduino Pro Micro act like a HID-device resembling a mouse*/



/*-----------------------------------------------------------------------*/
/*                                CONSTANTS                              */
/*.......................................................................*/

#define YP 18 // can be a digital pin
#define XM 19 // can be a digital pin
#define YM A2 // must be an analog pin, use "An" notation!
#define XP A3 // must be an analog pin, use "An" notation!

#define PIEZO     0   /*speaker output pin*/
#define POTX      3   /*the POT-X signal for the C64 joystick port*/
#define POTY      2   /*the POT-Y signal for the C64 joystick port*/

#define JOY_UP    5   /**/
#define JOY_DOWN  6   /**/
#define JOY_LEFT  7   /**/
#define JOY_RIGHT 8   /**/
#define JOY_FIRE  9   /**/
#define C64SENSE  10  /*this signal senses the C64's power line*/

#define BUTTON1   16  /*simple pushbutton*/
#define BUTTON2   14  /*simple pushbutton*/
#define BUTTON3   15  /*simple pushbutton*/

/*In order to make a circle drawn on the touchpad (which isn't square but a rectangle), look like a circle on the screen, some scaling is required this is*/
/*nothing more then the ratio between X and Y, take your ruler and measure the actiove part of the touchscreen and*/
/*divide y by x to get the required scaling ratio, for example: x=164mm, y=99mm  then scaling ratio = 99mm / 164mm = 0,6*/
#define PANEL_XY_RATIO  60  /*ratio is times 100 (because we are working with integers, and by multiplying with 100 here and dividing by 100 later, we get an effective calculation accuracy of 0.01, not bad for a simple integer*/  

const unsigned int POT_OFFSET = 3800;   /*The CBM paddle ADC uses a deadzone of 256us to clear the measurement capacitor*/
const unsigned int POT_SCALE = 162;     /*The factor (x10) between the timing of Arduino Timer-1 and the CBM ADC timer*/
#define JOYSTICKMODESENSITIVITY   50   /*the number of pixels the sylus needs to move in order to be detected as a joystick thresshold*/
#define AUTOFIREDELAY             40
const int min_x = 240;                  /*value used for paddle (koalapad) mode, to match the size of the screen to the workable size of the CBM screen*/
const int max_x = 751;                  /*                                                                                                             */
const int min_y = 240;                  /*                                                                                                             */
const int max_y = 751;                  /*                                                                                                             */

/*.......................................................................*/

enum DeviceStates{UNDEFINED, C64_KOALAPAD, C64_JOYSTICK, C64_1351, PC_MOUSEPAD}; /*these are the different modes of operation the device can be used in*/

/*-----------------------------------------------------------------------*/
/*                                GLOBALS                                */
/*.......................................................................*/

int mode_of_operation = UNDEFINED;


/*declare the usage of the touchscreen class and initialize it's required values*/
/*For better pressure precision, we need to know the resistance  between X+ and X- Use any multimeter to read it */
/*For example if its 300 ohms across the X plate then use TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);     */
//TouchScreen ts = TouchScreen(XP, YP, XM, YM, 950);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint ts_data;  /*declare an object for the touch screen data, make it global for ease of use*/

/*-----------------------------------------------------------------------*/
/*                            ROUTINE DECLARATIONS                       */
/*.......................................................................*/

void int_handler_potADCstart();
void set_potpos_x(unsigned char value);
void set_potpos_y(unsigned char value);

void Sound_Tone(unsigned int period, unsigned int duration);
void Button_test(void);
 
/*-----------------------------------------------------------------------*/
/*                   I N T E R R U P T   R O U T I N E S                 */
/*-----------------------------------------------------------------------*/

/*this is the interrupt handler for the Timer-1, capture compare A register match*/
ISR(TIMER1_COMPA_vect)          
{
  //pinMode(POTX, INPUT);         /*set IO to high-z, now the pull-up resistor charges the cap, trigger the ADC and is also ready for the discharge sequence required to prepare for a new measurement*/
  DDRD &= ~(1 << 0);            /*clear bit-0 to set the IO-pin as input*/  

  TIMSK1 &= ~(1 << OCIE1A);     /*disable timer compare A interrupt*/  
}

/*this is the interrupt handler for the Timer-1, capture compare B register match*/
ISR(TIMER1_COMPB_vect)
{
  //pinMode(POTY, INPUT);         /*set IO to high-z, now the pull-up resistor charges the cap, trigger the ADC and is also ready for the discharge sequence required to prepare for a new measurement*/
  DDRD &= ~(1 << 1);            /*clear bit-1 to set the IO-pin as input*/    
  
  TIMSK1 &= ~(1 << OCIE1B);     /*disable timer compare B interrupt*/
}

/*When a new pot ADC measurement starts, the signbal goes low, the 2-paddle ADC is sampling 2 paddles completely synchronized.*/
/*So when we know that paddle-X starts we also know that paddle-Y has started, this means that we only need to check one paddle ADC-pin*/
void int_handler_potADCstart()
{ 
  /*when we get here, the SID will start to clear it's measurement capacitor*/
  TCNT1  = 0; /*reset the timer value (the timer of the Arduino is now synced to the timer of the ADC)*/

  digitalWrite(POTX, LOW);  /*make sure IO-pin is low in order to extend the discharge procedure of the ADC,*/
  pinMode(POTX, OUTPUT);    /*so we can raise it later exactly at the moment we desire*/
  digitalWrite(POTY, LOW);
  pinMode(POTY, OUTPUT);

  TIFR1 = _BV(OCF1A) | _BV(OCF1B);        /*make sure int flags are cleared before we enable them*/
  TIMSK1 |= (1 << OCIE1A);                /*enable timer compare A interrupt*/
  TIMSK1 |= (1 << OCIE1B);                /*enable timer compare B interrupt*/
}

/*-----------------------------------------------------------------------*/
/*                        I N I T I A L I S A T I O N                    */
/*-----------------------------------------------------------------------*/

void setup(void)
{
  pinMode(POTX, INPUT);
  pinMode(POTY, INPUT);  
  
  pinMode(JOY_UP, OUTPUT);    /*define port for use as output, to drive the transistor connected to the JOYstick port*/
  digitalWrite(JOY_UP, LOW);  /*low = no action*/
  pinMode(JOY_DOWN, OUTPUT);
  digitalWrite(JOY_DOWN, LOW);
  pinMode(JOY_LEFT, OUTPUT);
  digitalWrite(JOY_LEFT, LOW);  
  pinMode(JOY_RIGHT, OUTPUT);
  digitalWrite(JOY_RIGHT, LOW);  
  pinMode(JOY_FIRE, OUTPUT);
  digitalWrite(JOY_FIRE, LOW);  

  pinMode(PIEZO, OUTPUT);
  digitalWrite(PIEZO, LOW);  

  pinMode(C64SENSE, INPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);


  pinMode(POTX, INPUT);
  //const byte int_potx = POTX;             /*force the IO-pin definition to a byte, in order to prase it correctly to the interrupt-related declaration*/  
  //attachInterrupt(digitalPinToInterrupt(int_potx), int_handler_potADCstart, FALLING);
  attachInterrupt(digitalPinToInterrupt(POTX), int_handler_potADCstart, FALLING);



  /*-----------------*/
  /*For the paddle ADC of the CBM computer we need some accurate timing in order to generate the appriate signals*/
  /*we use timer-1, in combination with capture compare value interrupts, (A for paddle-X and B for paddle-Y)*/
  /*the timer always runs, but it is synchronized (reset) to the start of the ADC conversion, because the compare*/
  /*interrupts are disabled, there is no problem that the timer runs constantly. The timeout value of the timer-1*/
  /*exceeds the time of a paddle ADConversion many times, meaning that the timer is reset before it can overflow*/
  /*Though if there is no CBM connected, the timer will constantly overflow and compares will match, but because we*/
  /*only enabled these interrupts when the timer has been synced to the ADC there is no problem it fires, as it is a dud*/
  // initialize timer1 (see info on : https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
  // for detailed register info     : http://medesign.seas.upenn.edu/index.php/Guides/MaEvArM-timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  /*compare match register-A*/ /*3800=0 (16 steps equals 1 ADC tick)*/
  OCR1A = 65535;  /*Compare match register-A. A safe value that lies far beyond the possible, we may consider this a NAN*/
  OCR1B = 65535;  /*Compare match register-B. A safe value that lies far beyond the possible, we may consider this a NAN*/
  
  //TCCR1B |= (1 << WGM12);   // CTC mode (CTC mode. Clear timer on compare match. When the timer counter reaches the compare match register, the timer will be cleared)
  /*we do no use CTC (clear-time-on-compare-match), because we need to continue counting, otherwise we can only time one paddle and we need to be able to time both*/

  /*define prescaler setting*/
  //TCCR1B |= (0 << CS12); TCCR1B |= (0 << CS11); TCCR1B |= (0 << CS10);    /*no clock source, time counter stopped*/
  TCCR1B |= (0 << CS12); TCCR1B |= (0 << CS11); TCCR1B |= (1 << CS10);    /*clock:1 prescaler*/
  //TCCR1B |= (0 << CS12); TCCR1B |= (1 << CS11); TCCR1B |= (0 << CS10);    /*clock:8 prescaler*/
  //TCCR1B |= (0 << CS12); TCCR1B |= (1 << CS11); TCCR1B |= (1 << CS10);    /*clock:64 prescaler*/
  //TCCR1B |= (1 << CS12); TCCR1B |= (0 << CS11); TCCR1B |= (0 << CS10);    /*clock:256 prescaler*/
  //TCCR1B |= (1 << CS12); TCCR1B |= (0 << CS11); TCCR1B |= (1 << CS10);    /*clock:1024 prescaler*/
  //TCCR1B |= (1 << CS12); TCCR1B |= (1 << CS11); TCCR1B |= (0 << CS10);    /*external clock source on T1-pin, falling edge*/
  //TCCR1B |= (1 << CS12); TCCR1B |= (1 << CS11); TCCR1B |= (1 << CS10);    /*external clock source on T1-pin, rising edge*/
   
  interrupts();             // enable all interrupts


  /*-----------------*/

  Serial.begin(115200);     /*open a serial connection, for debugging purposes only*/ 
  //while (!Serial) ;         /*to make sure we miss nothing printed to the virtual COM-port, keep looping until the serial stream is opened*/
  /*But beware, this means the application WILL stay here until a COM is opened, meaning that an application (arduino IDE or terminal) must be running!!!*/
  Serial.println("Commo pad (2019)");



  /*check if we receive power via the USB cable, if so, then we should act like a USB device*/
  /*if not, then check the buttons to see how we should act on the connected CBM computer*/
  if(digitalRead(C64SENSE) == HIGH)
  {    
    if(digitalRead(BUTTON1) == LOW)
    {
      delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/      
      Serial.println("Changing mode setting to: koalapad");           
      mode_of_operation = C64_KOALAPAD;   /*when no button is pressed on power-on, the system goes into koalapad mode (or paddle mode) is active*/          
      EEPROM.write(0, C64_KOALAPAD);      /*save the new configuration*/      
    }
    else
    if(digitalRead(BUTTON2) == LOW)
    {      
      delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/                 
      Serial.println("Changing mode setting to: joystick");                
      mode_of_operation = C64_JOYSTICK;   /*when button-2 is pressed on power-on, the system goes into joystick mode*/      
      EEPROM.write(0, C64_JOYSTICK);      /*save the new configuration*/            
    }
    else
    if(digitalRead(BUTTON3) == LOW)       
    {
      delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/      
      Serial.println("Changing mode setting to: 1351");                
      mode_of_operation = C64_1351;       /*when button-1 is pressed on power-on, the system goes into mouse mode*/      
      EEPROM.write(0, C64_1351);          /*save the new configuration*/      
    }
    else  /*when no button has been pressed, use the setting stored in EEPROM*/
    {
      mode_of_operation = EEPROM.read(0);      
    }

    /*check the mode value and notify the user (via sound or serial output) about the mode that is being used*/
    switch(mode_of_operation)
    {
      case C64_KOALAPAD:
      {       
        Sound_Tone(2000, 100);                  delay(100);
        delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/
        Serial.println("C64 koalapad mode");                  
        break;  /*not really usefull after a while(1), but added for consistency*/
      }      
  
      case C64_JOYSTICK:
      {                
        Sound_Tone(2000, 100);                  delay(100);
        Sound_Tone(2000, 100);                  delay(100);      
        delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/
        Serial.println("C64 joystick mode");                  
        break;  /*not really usefull after a while(1), but added for consistency*/      
      }

      case C64_1351:
      {
        Sound_Tone(2000, 100);                  delay(100);
        Sound_Tone(2000, 100);                  delay(100);
        Sound_Tone(2000, 100);                  delay(100);
        delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/
        Serial.println("C64 1351 mode");                        
        break;  /*not really usefull after a while(1), but added for consistency*/      
      }
               
      /*if we have an undefined mode value, we set the mode to a safe value*/
      default:
      {
        Sound_Tone(2000, 150);                  delay(100);
        Sound_Tone(1000, 300);                  delay(100);
        Sound_Tone(2000, 150);                  delay(100);
        Sound_Tone(1000, 300);                  delay(100);

        delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/
        Serial.println("Incorrect mode of operation, set to default: koala pad mode");              
        
        mode_of_operation = C64_KOALAPAD;   /*when no button is pressed on power-on, the system goes into koalapad mode (or paddle mode) is active*/          
        EEPROM.write(0, C64_KOALAPAD);      /*save the new configuration*/              
        break;
      }
    }     
  }
  else
  {
    Sound_Tone(3000, 100);                delay(100);
    Sound_Tone(3000, 100);                delay(100); 
    Sound_Tone(2000, 100);                delay(100);
    Sound_Tone(1000, 200);          
    delay(3000);  /*allow for USB virtual COM-port to be connected (if it is not connected, then there's no problem either)*/
    Serial.println("PC Mousepad mode");
    mode_of_operation = PC_MOUSEPAD;    
  }
  
}

/*-----------------------------------------------------------------------*/
/*                                M A I N                                */
/*-----------------------------------------------------------------------*/

void loop(void)
{
  int start_pos_x = 0;
  int start_pos_y = 0;
  unsigned char CBM1351_x = 128;  
  unsigned char CBM1351_y = 128;
  int prev_x = 0;
  int prev_y = 0;
  bool new_pos = false;
  int xDistance = 0;
  int yDistance = 0;
  bool b1_pressed = false;
  bool b2_pressed = false;
  bool b3_pressed = false;

  int autofire_delaycnt = 0;

  float temp = 0;
  float error = 0;
  int value = 0;  

  while(1)
  {
    ts_data = ts.getPoint();  /*a point object holds x y and z coordinates*/          
//    if (ts_data.z > ts.pressureThreshhold)  /*we have some minimum pressure we consider 'valid' pressure of 0 means no pressing!*/
//    {
//      Serial.print("X = "); Serial.print(ts_data.x);
//      Serial.print("\tY = "); Serial.print(ts_data.y);
//      Serial.print("\tPressure = "); Serial.print(ts_data.z);
//      Serial.println();
//    }

    switch(mode_of_operation)
    {
      case C64_KOALAPAD:
      {    
        digitalWrite(JOY_LEFT, !digitalRead(BUTTON1));
        digitalWrite(JOY_FIRE, !digitalRead(BUTTON2));    
        digitalWrite(JOY_RIGHT, !digitalRead(BUTTON3));  
         
        if (ts_data.z > ts.pressureThreshhold)  /*we have some minimum pressure we consider 'valid' pressure of 0 means no pressing!*/
        {  
          if((ts_data.x >= min_x) && (ts_data.x < max_x))
          {
            ts_data.x = (ts_data.x - min_x) / 2;
            //Serial.print("X = ");
            //Serial.println(ts_data.x);
            set_potpos_x(ts_data.x);      
          }
   
          if((ts_data.y >= min_y) && (ts_data.y < max_y))
          {
            ts_data.y = (ts_data.y - min_y) / 2;
            //Serial.print("Y = ");
            //Serial.println(ts_data.y);
            set_potpos_y(ts_data.y);      
          }                      
        }          
               
        delay(5);  /*reduce update speed to a max. of 200 updates per second*/        
        break;  /*not really usefull after a while(1), but added for consistency*/
      }    
  
  
      case C64_JOYSTICK:
      {    
        /*button-2 (the center button) is mapped to function as an auto-fire button*/
        if(digitalRead(BUTTON2) == LOW)
        {
          autofire_delaycnt++;
          if(autofire_delaycnt == (AUTOFIREDELAY/2))
          {
            digitalWrite(JOY_FIRE, HIGH);
          }
  
          if(autofire_delaycnt > AUTOFIREDELAY)
          {
            autofire_delaycnt=0;
            digitalWrite(JOY_FIRE, LOW);
          }          
        }
        else
        {
          /*button-1 and 3 are mapped to the fire button in this mode*/
          if((digitalRead(BUTTON1) == LOW) || (digitalRead(BUTTON3) == LOW))  { digitalWrite(JOY_FIRE, HIGH);  }
          else                                                                { digitalWrite(JOY_FIRE, LOW); }    
        }
  
  
        ts_data = ts.getPoint();  /*a point object holds x y and z coordinates*/      
        if (ts_data.z > ts.pressureThreshhold)  /*we have some minimum pressure we consider 'valid' pressure of 0 means no pressing!*/
        {     
          /*when we are here for the second (or more) time, since the user pressed the screen, send the difference of movement as a mouse movement*/
          if(new_pos == false)
          {
            new_pos = true;
            start_pos_x = ts_data.x;
            start_pos_y = ts_data.y;
          }
  
          if((((start_pos_x - ts_data.x)*100)/PANEL_XY_RATIO) < (0-JOYSTICKMODESENSITIVITY)){ digitalWrite(JOY_RIGHT, HIGH);}
          else                                                                              { digitalWrite(JOY_RIGHT, LOW); }          
          if((((start_pos_x - ts_data.x)*100)/PANEL_XY_RATIO) > JOYSTICKMODESENSITIVITY)    { digitalWrite(JOY_LEFT, HIGH);}
          else                                                                              { digitalWrite(JOY_LEFT, LOW); }
          if((start_pos_y - ts_data.y) < (0-JOYSTICKMODESENSITIVITY))                       { digitalWrite(JOY_DOWN, HIGH);}
          else                                                                              { digitalWrite(JOY_DOWN, LOW); }         
          if((start_pos_y - ts_data.y) > JOYSTICKMODESENSITIVITY)                           { digitalWrite(JOY_UP, HIGH);}
          else                                                                              { digitalWrite(JOY_UP, LOW); }                   
        }
        else
        {
          new_pos = false;
          digitalWrite(JOY_RIGHT, LOW);
          digitalWrite(JOY_LEFT, LOW);
          digitalWrite(JOY_DOWN, LOW);
          digitalWrite(JOY_UP, LOW);
        }
        
        delay(5);  /*reduce update speed to a max. of 200 updates per second*/          
        break;  /*not really usefull after a while(1), but added for consistency*/      
      }

      case C64_1351:
      {
        digitalWrite(JOY_FIRE, !digitalRead(BUTTON1)); /*this is the mouse's left button*/   
        //digitalWrite(JOY..., !digitalRead(BUTTON2)); /*the 1351 does not have a center button, so we have no use for button-2 on our tablet*/
        digitalWrite(JOY_UP, !digitalRead(BUTTON3));   /*this is the mouse's left button*/
        
  
        ts_data = ts.getPoint();  /*a point object holds x y and z coordinates*/      
        if (ts_data.z > ts.pressureThreshhold)  /*we have some minimum pressure we consider 'valid' pressure of 0 means no pressing!*/
        {     
          /*when we are here for the second (or more) time, since the user pressed the screen, send the difference of movement as a mouse movement*/
          if(new_pos == true)
          {
            xDistance = ts_data.x - prev_x;
            yDistance = ts_data.y - prev_y;
      
            /*send mouse movement to the computer, adjust for the touchscreen size/aspect ratio*/
            CBM1351_x = CBM1351_x + ((xDistance*100)/PANEL_XY_RATIO);
            if(CBM1351_x < 64)  { CBM1351_x = CBM1351_x + 128; } /*the value for the pot signal must always be within the 64-192 range*/
            if(CBM1351_x >= 192){ CBM1351_x = CBM1351_x + 128;} /*the value for the pot signal must always be within the 64-192 range*/
            //Serial.print("X:");
            //Serial.print(CBM1351_x);
  
            CBM1351_y = CBM1351_y - yDistance;  /*we use minus to correct for the direction difference between the CBM and the touchscreen*/
            if(CBM1351_y < 64)  { CBM1351_y = CBM1351_y + 128; } /*the value for the pot signal must always be within the 64-192 range*/
            if(CBM1351_y >= 192){ CBM1351_y = CBM1351_y + 128;} /*the value for the pot signal must always be within the 64-192 range*/
            //Serial.print("  Y:");
            //Serial.println(CBM1351_y);
  
            set_potpos_x(CBM1351_x);      
            set_potpos_y(CBM1351_y);                              
          }
          prev_x = ts_data.x ;
          prev_y = ts_data.y;    
          new_pos = true;
        }
        else
        {
          new_pos = false;
        }       
        
        delay(5);  /*reduce update speed to a max. of 200 updates per second*/        
        break;  /*not really usefull after a while(1), but added for consistency*/      
      }      
  
      case PC_MOUSEPAD:
      {
        ts_data = ts.getPoint();  /*a point object holds x y and z coordinates*/      
        if (ts_data.z > ts.pressureThreshhold)  /*we have some minimum pressure we consider 'valid' pressure of 0 means no pressing!*/
        {      
          /*when we are here for the second (or more) time, since the user pressed the screen, send the difference of movement as a mouse movement*/
          if(new_pos == true)
          {
            xDistance = ts_data.x - prev_x;
            yDistance = ts_data.y - prev_y;
      
            if(digitalRead(BUTTON2) == LOW)
            {
              /*Make scrolling less sensitive. BUT prevent throwing away info, this is achieved by storing the error we make whena parsing the position as an integer*/
              temp = (yDistance + (error*10)) / 10;
              value = temp; /*convert float to int*/            
              error = temp - value;
              //Serial.println(error);             
              
              //Mouse.move(0, 0, yDistance); /*send mouse scrollwheel action*/             
              Mouse.move(0, 0, value); /*send mouse scrollwheel action*/                           
            }
            else
            {
              Mouse.move(((xDistance*100)/PANEL_XY_RATIO), yDistance, 0); /*send mouse movement to the computer, adjust for the touchscreen size/aspect ratio*/
            }
          }
          prev_x = ts_data.x ;
          prev_y = ts_data.y;    
          new_pos = true;
        }
        else
        {
          new_pos = false;
        }
  
        /*check for LMB*/
        if((digitalRead(BUTTON1) == LOW) and (b1_pressed == false))
        {
          Mouse.press(MOUSE_LEFT);
          b1_pressed = true;
        }
        if((digitalRead(BUTTON1) == HIGH) and (b1_pressed == true))
        {
          Mouse.release(MOUSE_LEFT);
          b1_pressed = false;
        }
  
        /*check for RMB*/
        if((digitalRead(BUTTON3) == LOW) and (b3_pressed == false))
        {
          Mouse.press(MOUSE_RIGHT);
          b3_pressed = true;
        }
        if((digitalRead(BUTTON3) == HIGH) and (b3_pressed == true))
        {
          Mouse.release(MOUSE_RIGHT);
          b3_pressed = false;
        }
            
        delay(10);  /*prevent the loop from running too fast (this also helps to reduce button bounce effects)*/
        break;  /*not really usefull after a while(1), but added for consistency*/
      }

      /*if we have an undefined mode value, do nothing*/
      default:
      {
        break;
      }
    } 
  }  
}

/*-----------------------------------------------------------------------*/
/*                             R O U T I N E S                           */
/*.......................................................................*/

/*calculate the correct timer value for the compare register*/
void set_potpos_x(unsigned char value)
{
  OCR1A = POT_OFFSET+(((unsigned int)value*POT_SCALE)/10);  /*compare match register-A (position of the paddle-X)*/  
}

/*calculate the correct timer value for the compare register*/
void set_potpos_y(unsigned char value)
{
  OCR1B = POT_OFFSET+(((unsigned int)value*POT_SCALE)/10);  /*compare match register-A (position of the paddle-Y)*/  
}


/*The routine below is a very crude method of producing sound, blocking, which is not a problem if used rarely*/
void Sound_Tone(unsigned int period, unsigned int duration)
{
  unsigned int lp;

  noInterrupts();           /*disable all interrupts (otherwise "Sound_Tone" sounds like crap (make sure to enable it again, otherwise the bootloader won't work)*/          
  period = period/2;
  pinMode(PIEZO, OUTPUT);           /*set IO-pin to output (if it wasn't already)*/
  for(lp=duration;lp>0;lp--)
  {
    digitalWrite(PIEZO, HIGH);      /*produce current for the speaker*/
    delayMicroseconds(period);      /*delay to achieve the desired frequency*/  
    digitalWrite(PIEZO, LOW);  
    delayMicroseconds(period);
  }
  interrupts();             /*enable all interrupts (ATTENTION: !! DO NOT DISABLE THIS LINE !! Otherwise the USB bootloader stops working)*/    
}

void Button_test(void)
{
  if(digitalRead(BUTTON1) == LOW)
  {
    Serial.println("button-1");          
    Sound_Tone(1000, 200);                  /*generate a .. cyles of the desired tone*/          
  }
  
  if(digitalRead(BUTTON2) == LOW)
  {
    Serial.println("button-2");          
    Sound_Tone(1000, 200);                  /*generate a .. cyles of the desired tone*/          
  }
  
  if(digitalRead(BUTTON3) == LOW)
  {
    Serial.println("button-3");          
    Sound_Tone(1000, 200);                  /*generate a .. cyles of the desired tone*/          
  }
}
