#include "main.hpp"
#include <Encoder.h>
#include <EEPROM.h>
#include <limits.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

//
// global
//
Encoder myEnc( ENCODER_B, ENCODER_A );
int ledVal{ START_BRIGHTNESS };
bool doSleep{ false };

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
  //
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
  // have to init the EEPROM?
  //
  if ( EEPROM.read( 0 ) != EEPROM_INIT_MAGIC )
  {
    initEEPROM();
  }
  //
  // EEPROM value for LED brigtness
  //
  ledVal = static_cast< int >( EEPROM.read( LED_VALUE_ADDR ) );
  // virtual position set
  if ( ledVal < MIN_VAL_AFTER_SLEEP )
    ledVal = MIN_VAL_AFTER_SLEEP;
  myEnc.write( ledVal );
  // val = EEPROM.read(address);
  // EEPROM.write(addr, val);
  // EEPROM.update( addr, val ); // writes only if val is different!
}

//
// loop forever
//
void loop()
{
  // marker variables
  static int32_t oldPosition{ -999 };
  static bool wasLedValChanged{ false };
  static unsigned long nextEEPromCheck{ millis() + 3500UL };

  //
  // if the interrupt routine says, the controller have to sleep
  //
  if ( doSleep )
  {
    detachInterrupt( digitalPinToInterrupt( ENCODER_SW ) );
    analogWrite( PWM_LED, 0 );
    delay( 220 );
    analogWrite( PWM_LED, 255 );
    delay( 220 );
    analogWrite( PWM_LED, ledVal );
    delay( 800 );
    sleepNow();          // sleep controller
    oldPosition = -999;  // be shure after wakeup set LED
    doSleep = false;     // reset the value
  }
  //
  // read the "new" virtual position of the rotary encoder
  //
  int32_t newPosition = myEnc.read();
  if ( newPosition > START_DOUBLE_SPEED )
  {
    ++newPosition;
    myEnc.write( newPosition );
  }
  //
  // what if the position is wrong
  //
  if ( newPosition > MAX_VAL )
  {
    //
    //  kepp maximum
    //
    myEnc.write( MAX_VAL );
    newPosition = MAX_VAL;
  }
  if ( newPosition < 0 )
  {
    //
    // keep minimum
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
  // make not too often
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
// init at first time the EEPROM values
//
void initEEPROM()
{
  EEPROM.write( LED_VALUE_ADDR, START_BRIGHTNESS );
  EEPROM.write( 0, EEPROM_INIT_MAGIC );
}

//
// set the device in sleep mode
// SOURCE: https://github.com/blevien/attiny85-sleep/blob/master/attiny85-sleep.ino
//
void sleepNow()
{
  EEPROM.update( LED_VALUE_ADDR, static_cast< uint8_t >( ledVal & 0xff ) );  // save the value
  set_sleep_mode( SLEEP_MODE_PWR_DOWN );                                     // sleep mode is set here
  sleep_enable();                // enables the sleep bit in the mcucr register so sleep is possible
  delayMicroseconds( 100 );      // if there is an mechanical contact in switch
  digitalWrite( PWM_LED, LOW );  // switch off analog / pwm mode
  // use interrupt 0 (pin 2) and run function wakeUpNow when pin 2 gets LOW
  attachInterrupt( digitalPinToInterrupt( ENCODER_SW ), wakeUpNow, LOW );
  // here the device is actually put to sleep!!
  sleep_mode();
  //
  // here if wake up
  // first thing after waking from sleep: disable sleep...
  //
  sleep_disable();
  //
  // disables interrupton pin  so the wakeUpNow code will not be executed during normal running time.
  //
  detachInterrupt( digitalPinToInterrupt( ENCODER_SW ) );
  // debounce switch
  while(!debounceSwitch( ENCODER_SW ));
  //
  // install interrupt for switch (on/off)
  // change of state is switch on/off led
  //
  attachInterrupt( digitalPinToInterrupt( ENCODER_SW ), interruptUseSwitch, CHANGE );
}

//
// int when the device should awaken
//
void wakeUpNow()
{
  // here the interrupt is handled after wakeup
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.

  //
  // wakeup ALWAYS with a minimum of light
  //
  if ( ledVal < MIN_VAL_AFTER_SLEEP )
    ledVal = MIN_VAL_AFTER_SLEEP;
  myEnc.write( ledVal );
  //
  // led like brevore switch off
  //
  analogWrite( PWM_LED, ledVal );
}

//
// interrupt routine for push-switch
//
void interruptUseSwitch()
{
  if( debounceSwitch( ENCODER_SW ))
  {
      // say the loop the controller have to sleep
      if ( !doSleep )
        doSleep = true;
  }
}

bool debounceSwitch( uint8_t button )
{
  // for debounce behavior
  static unsigned long debounceTime{ ULONG_MAX };

  //
  // push down prepares the things
  //
  int pin = digitalRead( button );
  if ( pin == LOW )
  {
    // switch is down
    // start debouncing
    // wait for switch release
    debounceTime = millis() + DEBOUNCE_MS;
  }
  else
  {
    //
    // switch is up
    // check if its long enough up or bonced
    //
    if ( debounceTime < millis() )
    {
      // debounce done
      debounceTime = ULONG_MAX;
      // say "unbounced"
      return(true);
    }
  }
  return(false);
}
