/************** ET1016 Intro to Engrg Programming ***************/
/** This program is coded by                                   **/
/** 1. Tan Yan Yu, P2517382                                    **/
/** 2. Ingyin May, P2539007                                    **/
/** Class: DCEP/FT/1A/09 , ET1016                              **/
/****************************************************************/


#include <math.h>
#include <Wire.h>
#include "RichShieldLightSensor.h"
#include "RichShieldTM1637.h"
#include "RichShieldLED.h"
#include "RichShieldPassiveBuzzer.h"
#include "RichShieldDHT.h"

//variable declaration
#define PassiveBuzzerPin 3
#define LED_RED 4
#define LED_GREEN 5
#define LED_BLUE 6
#define LED_YELLOW 7
#define BUTTONK1 8
#define BUTTONK2 9
#define CLK 10
#define DIO 11
#define KNOB_PIN A0
#define lightd A2

#define NOTE_M1  523 //octave above middle c
#define NOTE_M2  587 //d
#define NOTE_M3  659 //e
#define NOTE_M4  698 //f
#define NOTE_M5  784 //g
#define NOTE_M6  880 //a


const static int star[] = 
{
NOTE_M1,NOTE_M1,NOTE_M5,NOTE_M5,NOTE_M6,NOTE_M6,NOTE_M5,0,
NOTE_M4,NOTE_M4,NOTE_M3,NOTE_M3,NOTE_M2,NOTE_M2,NOTE_M1,0,
NOTE_M5,NOTE_M5,NOTE_M4,NOTE_M4,NOTE_M3,NOTE_M3,NOTE_M2,0,
NOTE_M5,NOTE_M5,NOTE_M4,NOTE_M4,NOTE_M3,NOTE_M3,NOTE_M2,0,
NOTE_M1,NOTE_M1,NOTE_M5,NOTE_M5,NOTE_M6,NOTE_M6,NOTE_M5,0,
NOTE_M4,NOTE_M4,NOTE_M3,NOTE_M3,NOTE_M2,NOTE_M2,NOTE_M1,0,
};

const static int tempostar[] = 
{
2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,
};


int knobValue;
int count = 0;
uint8_t mytime[4] = {2,3,5,9};      //  "2359"
uint8_t zerotime[4] = {0,0,0,0};    //  "0000"
uint8_t bye_word[4] = {17,11,4,14}; // "bYE"
uint8_t hi_word[4] = {17,18,1,17};  // "HI"

TM1637 disp(CLK,DIO);                //Clock and DIO of TM1637 IC connected to D10 and D11 of OPEN SMART UNO R3
PassiveBuzzer buz(PassiveBuzzerPin); //buz is name of an object of PassiveBuzzer class
DHT dht;

void counter(void);
void buzzer(void);
void what_word(uint8_t word1);
void button1(void);
void singstar(void);
void activeth(void);
void LightDetector(void);
void highRed(void);
void highYellow(void);
void lowYellow(void);
void highGreen(void);
void lowGreen(void);
void highBlue(void);
void lowBlue(void);
void LEDoff(void);
void LEDall(void);

void setup(void) 
{
  // put our setup code here, to run once:
  pinMode(BUTTONK1, INPUT_PULLUP);
  pinMode(BUTTONK2, INPUT_PULLUP);
  pinMode(LED_RED,OUTPUT);
  pinMode(LED_YELLOW,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_BLUE,OUTPUT);

  disp.init();
	dht.begin();
  Serial.begin(9600);
}



void loop() 
{
  //put our main code here to run in loop

  LEDall();               //Set all LED to ON as a test at the start
  what_word(mytime);      //Show 2359H to test the 7-segment LED
  LightDetector();        //Using light detector to check for user approaching
  LEDoff();               //Off all LED to continue 
  what_word(hi_word);     //Show 2359H to test the 7-segment LED
  button1();              //Wait for User to press Button1 to start countdown
  
  knobValue=analogRead(KNOB_PIN); //Read the variable knob to see if User set to ON or OFF
  Serial.print("The knobValue is "); //Print knobValue to Serial Monitor to see value
  Serial.println(knobValue);      

  if (knobValue >= 512)           //Check for knobValue >=512 i.e. "ON" the circuit
  {
    counter();                    //start countdown process
    buzzer();                     //sound buzzer since countdown ended
    what_word(zerotime);          //Show 0000 on 7-seg LED
    delay(500);                   //wait to allow 0000 to show
    Serial.println("Press Button1 to Restart/End"); //Remind USer to Press Button1 to restart/end
    button1();                    //Wait for User to press Button1
    what_word(bye_word);          //Show "BYE" on 7-seg LED
  }
  if (knobValue < 512)            //Check for knobValue < 512 i.e. circuit is OFF
  {
    LEDoff();                     //OFF all LED
    what_word(bye_word);          //Show "BYE" on 7-seg LED
  }                               //End of loop, code will loop again
  delay(500);                     //wait 1 sec
}

void what_word(uint8_t word1[4])        //function to display on 7-seg LED; to use unsigned integer based on textbook; array of 3 integers
{
  disp.clearDisplay();                  //clear the 7-seg LED display
  for (count=0; count < 4; count++)     //using FOR loop to loop the array to show character bit by bit on 7-Seg LED
  { 
    disp.display (count,word1[count]);  //display char or number. disp.display(uint8_t BitAddr, int8_t DispData) -> DispData range 0-19. 0-9, A,b,c,d,E,F,-,space,H,U
    delay(500);                         //wait 1 sec
  }
}

void counter(void)
{
  highGreen();              //Green LED ON to tell User countdown in progress
  for (count = 10; count> 0; count--)   //change count value here to suit counting down. 10 is used for testing purpose.
  {
    disp.display(count);    //Send counting number to 7-Seg LED
    delay(500);             //wait for 1 sec
    activeth();             //activate temperature and humidity checking
  }
  if (count == 0)           //check for countdown completed
  {
    disp.display(0);        //display 1x 0 on 7-Seg LED 
    lowGreen();
    highRed();              //Red LED ON to tell User countdown ended
  }
}

void buzzer(void)           //function to sound buzz
{
  while (digitalRead(BUTTONK2) != 0) //keep buzzing till BUTTON 2 is pressed  
  {
    buz.playTone(NOTE_M1, 250);      //Buzz tone
    delay(30);                       //pause
    Serial.println("Press BUTTON 2 to stop buzzer."); 
  }
}

void button1(void) //Function wait for User to press BUTTON 1
{
  int read1;

  do
  {
      read1 = digitalRead(BUTTONK1);
  }
  while(read1 != 0);     //while loop keep checking if BUTTON1 pressed
}

void LightDetector(void)  //Function simulate human approach when light is dimmed.
{
  int value[6];           //array to store light source reading 
  int average = 1000, sum = 0, index;
    
  while(average >= 767)
  {
    for (index = 0; index < 6; index ++)
    {
      value[index] = analogRead(lightd);
      delay(10);
      sum += value[index];
    }

    average = sum / 6;    //calculate average of light source reading
    Serial.print("The Light Value is ");
    Serial.println(analogRead(lightd));
    delay(400);

    if (analogRead(lightd) < 767)   //light source lower than 767 simulate User approached.
    {
      Serial.println("User is Here! ");
      singstar();
      if (analogRead(KNOB_PIN) >= 512)
      {
        Serial.println("Please press BUTTON1 to Start Countdown, else turn the knob to < 512 to end the sequence.");
      }
      else
      {
        Serial.println("Please turn the knob to more than 512, then press BUTTON1 to Start Countdown.");
      }
      average = 40;                 //set to a low very to ensure exit while loop
    }
    else
    {
      average = 1000;               //set to a high value to make while loop continue
      sum = 0;                      //reset variable "sum" to zero
    }
  }
}

void singstar(void)                 //Function plays a short tune
{
  int size = sizeof(star) / sizeof(int);
  for (int thisNote = 0; thisNote < size; thisNote++) 
  {
    int noteDuration = 350 / tempostar[thisNote];
    buz.playTone(star[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 0.30;
    delay(pauseBetweenNotes);
  }
}


void activeth(void) 
{
  float h = dht.readHumidity();     //read humidity value
  float t = dht.readTemperature();  //read temperature value

  if (isnan(t) || isnan(h)) 
  {
    displayError();                 //show eror if read in value are not numbers
  } 
  else
  {
    displayTemperature((int8_t) t); //process and display temperature value by passing variable
    displayHumidity((int8_t) h);    //process and display humidity value by passing variable
  }
}

void displayTemperature(int8_t temperature)
{
  int8_t temp[4];         //use of array to store temperature value
  if(temperature < 0)     //check for negative value
	{
		temp[0] = INDEX_NEGATIVE_SIGN;
		temperature = abs(temperature); //make value positive if it is negative
	}
	else
  { 
    if(temperature < 100)
    {
      temp[0] = INDEX_BLANK;    //disply blank
    }
	  else 
    {
      temp[0] = temperature/100;
    }
  }
	temperature %= 100;
	temp[1] = temperature / 10;
	temp[2] = temperature % 10;
	temp[3] = 'C';	          //index of 'C' for celsius degree symbol.
	// disp.display(temp);    //not sent to 7-seg LED as it is used for countdown
  temper(temperature);      //sent to serial monitor instead
  
  // Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C");
  Serial.print(", ");  
}

void temper(int8_t hot)
{
  if (hot >= 40)    //set to <= 40 or >= lower value to test
  {
    highBlue();     //Blue LED ON if exceed 40 degC
  }
  else
  {
    lowBlue();      //Blue LED OFF if within normal range
  }
}

void displayHumidity(int8_t humi)
{
  int8_t temp[4];           //use of array to store humidity value
  if(humi < 100)
  {
    temp[0] = INDEX_BLANK;
  }
  else
  {
    temp[0] = humi/100;
  }

  humi %= 100;
  temp[1] = humi / 10;
  temp[2] = humi % 10;
  temp[3] = 'H';	          //index of 'H' for celsius degree symbol.
  //disp.display(temp);     //not sent to 7-seg LED as it is used for countdown
  hum(humi);                //sent to Serial monitor instead
  // Serial.print("Humidity: ");
  Serial.print(humi);
  Serial.println(" H");
  // Serial.println("%");
}

void hum(int8_t wet)
{
  if (wet >= 75)    //set to <= 75 or >= lower value to test 
  {
    highYellow();   //Yellow LED ON if exceed 75%
  }
  else
  {
    lowYellow();    //Yellow LED OFF if within normal value
  }
}

void displayError(void)
{
  Serial.println("ERROR");  //display "ERROR" on Serial Monitor
}

/**************************************************
** This segment are the codes for the LED lights **
***************************************************/
void highRed(void) 
{
  digitalWrite(LED_RED,HIGH);
}

void highGreen(void) 
{
  digitalWrite(LED_GREEN,HIGH);
}

void lowGreen(void) 
{
  digitalWrite(LED_GREEN,LOW);
}

void highYellow(void) 
{
  digitalWrite(LED_YELLOW,HIGH);
}

void lowYellow(void) 
{
  digitalWrite(LED_YELLOW,LOW);
}

void highBlue(void)
{
  digitalWrite(LED_BLUE,HIGH);
}

void lowBlue(void)
{
  digitalWrite(LED_BLUE,LOW);
}
void LEDoff(void)  //Function OFF all LED at one go 
{
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_YELLOW,LOW);
  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_BLUE,LOW);
}

void LEDall(void)   //Function ON all LED at one go
{
  digitalWrite(LED_RED,HIGH);
  digitalWrite(LED_YELLOW,HIGH);
  digitalWrite(LED_GREEN,HIGH);
  digitalWrite(LED_BLUE,HIGH);
}