#include "main.hpp"
#include <Encoder.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

//
// global
//
Encoder myEnc( ENCODER_B, ENCODER_A );
int ledVal{ START_BRIGHTNESS };

//
// arduino like setup
//
void setup()
{
  // encoder switch for on/off
  pinMode( ENCODER_SW, INPUT_PULLUP );
  //
  // install interrupt for switch (on/off)
  // change of state is switch on/off led
  //
  attachInterrupt( digitalPinToInterrupt( ENCODER_SW ), interruptUseSwitch, CHANGE );
  //
  // define PWM LED Port
  // 1,3 V. Rot: 1,6–2,2 V. Gelb, Grün: 1,9–2,5 V. Blau, Weiß: 2,7–3,5 V.

  pinMode( PWM_LED, OUTPUT );
  //
  // dimm to startvalue
  //
  analogWrite( PWM_LED, START_BRIGHTNESS );
  //
  // pro forma, high activate
  //
  digitalWrite( ENCODER_SW, HIGH );
  //
  // EEPROM value for LED brigtness
  //
  ledVal = static_cast< int >( EEPROM.read( LED_VALUE_ADDR ) );
  // virtual position set
  myEnc.write( ledVal );
  // value = EEPROM.read(address);
  // EEPROM.write(addr, val);
  // EEPROM.update( address, value ); // writes only if val is different!
}

//
// set the device in sleep mode
//
void sleepNow()
{
  set_sleep_mode( SLEEP_MODE_PWR_DOWN );  // sleep mode is set here
  sleep_enable();                         // enables the sleep bit in the mcucr register so sleep is possible
  digitalWrite( PWM_LED, LOW );           // switch off analog / pwm mode
  delayMicroseconds( 100 );               // if there is an mechanical contact in switch
  attachInterrupt( digitalPinToInterrupt( ENCODER_SW ), wakeUpNow,
                   LOW );  // use interrupt 0 (pin 2) and run function wakeUpNow when pin 2 gets LOW
  sleep_mode();            // here the device is actually put to sleep!!
  //
  // here if wake up
  // first thing after waking from sleep: disable sleep...
  //
  sleep_disable();
  //
  // disables interrupton pin  so the wakeUpNow code will not be executed during normal running time.
  //
  detachInterrupt( digitalPinToInterrupt( ENCODER_SW ) );
  //
  // install interrupt for switch (on/off)
  // change of state is switch on/off led
  //
  attachInterrupt( digitalPinToInterrupt( ENCODER_SW ), interruptUseSwitch, CHANGE );
}

//
// int when the device should awaken
//
void wakeUpNow()  // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.

  //
  // led like brevore switch off
  //
  analogWrite( PWM_LED, ledVal );
}

//
// loop forever
//
void loop()
{
  static int32_t oldPosition{ -999 };
  static bool wasLedValChanged{ false };
  static unsigned long nextEEPromCheck{ millis() + 3500UL };
  //
  // read the "new" virtual position of the rotary encoder
  //
  int32_t newPosition = myEnc.read();

  if ( newPosition > MAX_VAL )
  {
    //
    // maximum behalten
    //
    myEnc.write( MAX_VAL );
    newPosition = MAX_VAL;
  }
  if ( newPosition < 0 )
  {
    //
    // minimum behalten
    //
    myEnc.write( MIN_VAL );
    newPosition = MIN_VAL;
  }
  if ( newPosition != oldPosition )
  {
    ledVal = static_cast< int >( newPosition & 0xff );
    oldPosition = newPosition;
    //
    // led makes output
    //
      analogWrite( PWM_LED, ledVal );
    //
    // set next time to eeprom save check
    //
    wasLedValChanged = true;
    nextEEPromCheck = millis() + 3500UL;
  }
  //
  // time to write eeprom?
  //
  if ( wasLedValChanged )
  {
    if ( millis() > nextEEPromCheck )
    {
      wasLedValChanged = false;
      // if the value was changed, update
      EEPROM.update( LED_VALUE_ADDR, static_cast< uint8_t >( ledVal & 0xff ) );
    }
  }
}

//
// interrupt routine for switch
//
void interruptUseSwitch()
{
  static unsigned long debounceTime{ 0UL };
  static int ledVal_shadow;

  int pin = digitalRead( ENCODER_SW );
  if ( pin == LOW )
  {
    // switch is down
    // start debouncing
    // wait for up
    debounceTime = millis() + DEBOUNCE_MS;
    ledVal_shadow = ledVal;
    //analogWrite( PWM_LED, 255 );
  }
  else
  {
    //
    // switch is up
    // check if its long enough up
    //
    if ( debounceTime < millis() )
    {
      // make something
      if( ledVal == 0 )
      {
        myEnc.write( ledVal_shadow );
      }
      else 
      {
        myEnc.write( 0 );
      }
      debounceTime = 0UL;
    }
  }
}

// void blink(uint8_t rounds)
// {
//   for (uint8_t i = 0; i < rounds; ++i)
//   {
//     analogWrite(PWM_LED, 254);
//     delayMicroseconds(200);
//     analogWrite(PWM_LED, 0);
//     delayMicroseconds(300);
//   }
// }

// Quelle: https://github.com/blevien/attiny85-sleep/blob/master/attiny85-sleep.ino
// #include <avr/interrupt.h>
// #include <avr/sleep.h>

// int ledPinLoop = 0;        // LED connected to digital pin 0
// int ledPinWakeUp = 4;      // LED to show the action of a interrupt
// int wakePin = 2;           // active LOW, ground this pin momentary to wake up

// void setup()
// {
//   pinMode(ledPinLoop, OUTPUT);     // sets the digital pin as output
//   pinMode(ledPinWakeUp, OUTPUT);   // sets the digital pin as output
//   pinMode(wakePin, INPUT_PULLUP);  // sets the digital pin as input
//   digitalWrite(wakePin, HIGH);
// }

// void sleepNow()
// {
//   set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
//   sleep_enable();                        // enables the sleep bit in the mcucr register so sleep is possible
//   attachInterrupt(0, wakeUpNow, LOW);    // use interrupt 0 (pin 2) and run function wakeUpNow when pin 2 gets LOW
//   digitalWrite(ledPinLoop, LOW);

//   sleep_mode();                          // here the device is actually put to sleep!!

//   sleep_disable();                       // first thing after waking from sleep: disable sleep...
//   detachInterrupt(0);                    // disables interrupton pin 3 so the wakeUpNow code will not be executed during normal
//   running time. delay(250);                            // wait 2 sec. so humans can notice the interrupt LED to show the interrupt
//   is handled digitalWrite (ledPinWakeUp, LOW);      // turn off the interrupt LED
// }

// void wakeUpNow()        // here the interrupt is handled after wakeup
// {
//   //execute code here after wake-up before returning to the loop() function
//   // timers and code using timers (serial.print and more...) will not work here.
//   digitalWrite(ledPinWakeUp, HIGH);
// }

// void loop()
// {
//   digitalWrite(ledPinLoop, HIGH);   // sets the LED on
//   delay(5000);                      // waits for a second
//   sleepNow();                       // sleep function called here
// }