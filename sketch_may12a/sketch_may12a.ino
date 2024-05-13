/*
CPE 301 Final
5/12/24
Ivan Uy
*/

//Libraries
#include <Stepper.h> //Stepper Library
#include <dht.h> //install the DHTLib library
#include <LiquidCrystal.h> //LCD

// Defines the number of steps per rotation
  const int stepsPerRevolution = 2038;
  // Creates an instance of stepper class
  // Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
  Stepper myStepper = Stepper(stepsPerRevolution, 7, 5, 6, 4);

//Definitions
  //DHT11 humidity sensor
    dht DHT;
    #define DHT11_PIN 10
  //Water Sensor
    #define POWER_PIN 9
    #define SIGNAL_PIN A2
    int value = 0; // variable to store the sensor value
  //Uart
    #define RDA 0x80
    #define TBE 0x20

//LCD
  const int RS = 12, EN = 11, D4 = 50, D5 = 48, D6 = 3, D7 = 2; //Reversed the pin order from example
  int right=0,up=0;
  int dir1=0,dir2=0;
  LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//ADC
  volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
  volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
  volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
  volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// UART Pointers
  volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
  volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
  volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
  volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
  volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 

// GPIO Pointers
//PK2 On/off 
  volatile unsigned char* port_k = (unsigned char*) 0x108; 
  volatile unsigned char* ddr_k = (unsigned char*) 0x107; 
  volatile unsigned char* pin_k = (unsigned char*) 0x106;
//PB6 Red LED
  volatile unsigned char* port_b = (unsigned char*) 0x25; 
  volatile unsigned char* ddr_b = (unsigned char*) 0x24; 
  volatile unsigned char* pin_b = (unsigned char*) 0x23; 
//PC0 Blue
  volatile unsigned char* port_c = (unsigned char*) 0x28; 
  volatile unsigned char* ddr_c = (unsigned char*) 0x27; 
  volatile unsigned char* pin_c  = (unsigned char*) 0x26;
//PA0 Green
  volatile unsigned char* port_a = (unsigned char*) 0x22; 
  volatile unsigned char* ddr_a = (unsigned char*) 0x21; 
  volatile unsigned char* pin_a = (unsigned char*) 0x20;
  //PL0 Yellow
  volatile unsigned char* port_l = (unsigned char*) 0x10B; 
  volatile unsigned char* ddr_l = (unsigned char*) 0x10A; 
  volatile unsigned char* pin_l = (unsigned char*) 0x109;
  //PF0
  volatile unsigned char* port_f = (unsigned char*) 0x31; 
  volatile unsigned char* ddr_f = (unsigned char*) 0x30; 
  volatile unsigned char* pin_f = (unsigned char*) 0x2F;
  //PD7 Reset
  volatile unsigned char* port_d = (unsigned char*) 0x2B; 
  volatile unsigned char* ddr_d = (unsigned char*) 0x2A; 
  volatile unsigned char* pin_d = (unsigned char*) 0x29;


// Timer Pointers
  volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
  volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
  volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
  volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
  volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
  volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// ISR
   byte in_char;
  //global ticks counter
  unsigned int currentTicks = 65535;
  unsigned int timer_running = 0;

//1 minute timer
  unsigned long myTime;

//variables
  int state;
  unsigned char temperature;
  unsigned char humidity;


void setup() 
{  
  //Start Uart
  U0Init(9600);

  //variables
  state = 4;

   //set PK2 to input on/off button
  *ddr_k &= 11111011;
  *port_k |= 00000100; //Pull up on PK2

  //set PD7 to input reset button
  *ddr_d &= 10111111;
  *port_d |= 01000000; //Pull up on PD7

  // set PB7 to output RED
  *ddr_b |= 10000000;
  *port_b &= 01111111; // set PB6 LOW

  //Set PC0 to output BLUE
  *ddr_c |= 00000001;
  *port_c &= 11111110 ; // set PC0 LOW

  //Set PA0 to output GREEN
  *ddr_a |= 00000001;
  *port_a &= 11111110; // set PA0 LOW

  //Set PL0 to output Yellow
  *ddr_l |= 00000001;
  *port_l &= 11111110; // set PL0 LOW
  
  //ADC setup
  adc_init();

  // setup the Timer for Normal Mode, with the TOV interrupt enabled
  setup_timer_regs();

  //LCD set up
  lcd.begin(16, 2); //set up number of columns and rows

  lcd.setCursor(0, 0); //set cursor to 0,0
  lcd.print("Temp"); // print message at (0, 0)
  lcd.setCursor(0, 1); // move cursor to (0, 1)
  lcd.print("Humidity"); // print message at (0, 1)

  //Water sensor
  pinMode (POWER_PIN, OUTPUT); // configure D7 pin as an OUTPUT
  digitalWrite (POWER_PIN, LOW); // turn the sensor OFF  

  //DC motor
  pinMode(27, OUTPUT); //Forward
  pinMode(25, OUTPUT); //Backward
  pinMode(23, OUTPUT); //Speed
  
}


void loop() 
{
  
  //1 minute timer
  myTime = millis();
  if(*pin_k & 0x04)
  {
        state = 1;

      if(*pin_k & 0x04)
      {
      // set the current ticks to the max value
      currentTicks = 65535;
      // if the timer is running
      if(timer_running)
      {
      // stop the timer
      *myTCCR1B &= 0xF8;
      // set the flag to not running
      timer_running = 0;
      // set PK2 LOW
      *pin_k &= 11111011;
      state = 4; //Go back to disabled
      }
      }
  }      

  else{
  switch(state){

    case 1://Idle
    
      //LED
      *pin_b &= 10000000;
      *pin_c &= 0x01;
      *pin_a |= 0x01;//Green on
      *pin_l &= 0x01;

      waterSensor();
      humiditySensor();
      stepper();

      //DC Fan
      pinMode(27, LOW); //Forward
      pinMode(25, LOW); //Backward
      
      //LCD
        //Temperature
      lcd.setCursor(0, 0); //set cursor to 0,0
      lcd.print("Temp"); // print message at (0, 0)
      while (U0kbhit()==0){}; // wait for RDA = true.
      //temperature = U0getchar(); //get temperature
      temperature = adc_read(0); //Read analog channel 0
      lcd.setCursor(5, 0); 
      lcd.print(temperature);  
        //Humidity
      lcd.setCursor(0, 1); // move cursor to (0, 1)
      lcd.print("Humidity"); // print message at (0, 1)
      while (U0kbhit()==0){};
      //humidity = U0getchar();
      humidity = adc_read(1); //Read analog channel 1
      lcd.setCursor(9, 0);
      lcd.print(humidity);
      
      if(*pin_k & 0x04)
      {
      // set the current ticks to the max value
      currentTicks = 65535;
      // if the timer is running
      if(timer_running)
      {
      // stop the timer
      *myTCCR1B &= 0xF8;
      // set the flag to not running
      timer_running = 0;
      // set PK2 LOW
      *pin_k &= 11111011;
      state = 4; //Go back to disabled
      }
      }
      else{}
      break;

    case 2://Running
    
      //LED
      *pin_b &= 10000000;
      *pin_c |= 0x01; //Blue on
      *pin_a &= 0x01; 
      *pin_l &= 0x01;

      waterSensor();
      humiditySensor();
      stepper();

      //DC Fan
      pinMode(27, HIGH); //Forward
      pinMode(25, LOW); //Backward
      pinMode(23, HIGH); //SPEED
      analogWrite(5, 100); //Set SPEED

      if(*pin_k & 0x04)
      {
      // set the current ticks to the max value
      currentTicks = 65535;
      // if the timer is running
      if(timer_running)
      {
      // stop the timer
      *myTCCR1B &= 0xF8;
      // set the flag to not running
      timer_running = 0;
      // set PK2 LOW
      *port_k &= 11111011;
      state = 4; //Go back to disabled
      }
      }
      else{}
      break;
      
    case 3://Error
  
      //LED
      *pin_b |= 10000000; //Red on
      *pin_c &= 0x01;
      *pin_a &= 0x01; 
      *pin_l &= 0x01;

      waterSensor();
      humiditySensor();
      stepper();
      
      //Print Error Message
      lcd.setCursor(0, 0); //set cursor to 0,0
      lcd.print("Water Level");
      lcd.setCursor(1, 0);
      lcd.print("is too low");

      //If button is pressed, reset to idle
      if(*pin_d & 01000000) 
      {
        state = 1;
      }
      else{}

      //DC Fan
      pinMode(27, LOW); //Forward
      pinMode(25, LOW); //Backward

      if(*pin_k & 0x04)
      {
      // set the current ticks to the max value
      currentTicks = 65535;
      // if the timer is running
      if(timer_running)
      {
      // stop the timer
      *myTCCR1B &= 0xF8;
      // set the flag to not running
      timer_running = 0;
      // set PK2 LOW
      *port_k &= 11111011;
      state = 4; //Go back to disabled
      }
      }

      break;

    case 4://Disabled 
    
      //LED
      *pin_b &= 10000000;
      *pin_c &= 0x01;
      *pin_a &= 0x01;
      *pin_l |= 0x01;  //Yellow on 

      //DC Fan
      pinMode(27, LOW); //Forward
      pinMode(25, LOW); //Backward
      

      break;
    }
  }
  
  //delay(1 * 60 * 1000);

}


//ADC
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

// Timer setup function
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}

// TIMER OVERFLOW ISR
                      //interupt needs to be on pin 2 and 38
ISR(TIMER1_OVF_vect)
{
  // Stop the Timer
  *myTCCR1B &= 0xF8;
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  // Start the Timer
  *myTCCR1B |= 0b00000001;
  // if it's not the STOP amount
  if(currentTicks != 65535)
  {
    // XOR to toggle Pk2
    *port_k ^= 11111011;
  }
}

/*
 * UART FUNCTIONS
 */
void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char getChar()
{
  return *myUDR0;
}
void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

//Water sensor
void waterSensor()
{
digitalWrite (POWER_PIN, HIGH); // turn the sensor ON
delay(10); // wait 10 milliseconds
value = analogRead (SIGNAL_PIN); // read the analog value from sensor
digitalWrite (POWER_PIN, LOW); // turn the sensor OFF
Serial.print("Sensor value: " );
Serial.println (value);
delay(1000);
}

  //DHT11
  void humiditySensor()
  {
  int chk = DHT.read11(DHT11_PIN);
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  delay(1000);
  }
  
  //Stepper
  void stepper()
  {
  // Rotate CW slowly at 5 RPM
  myStepper.setSpeed(5);
  myStepper.step(stepsPerRevolution);
  delay(1000);
  // Rotate CCW quickly at 10 RPM
  myStepper.setSpeed(10);
  myStepper.step(-stepsPerRevolution);
  delay(1000);  
  }