// Sensor pins
#define sensorPower 7
#define sensorPin A0

// Value for storing water level
int val = 0;

//FOR PORT H INPUT / OUTPUT
volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;

volatile unsigned char* myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char* myTCCR1B = (unsigned char*) 0x81;
volatile unsigned char* myTCCR1C = (unsigned char*) 0x82;
volatile unsigned char* myTIMSK1 = (unsigned char*) 0x6F;
volatile unsigned int* myTCNT1  = (unsigned  int*) 0x84;
volatile unsigned char* myTIFR1 =  (unsigned char*) 0x36;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

void setup() {
  // for water sensor 
    // Set D7 as an OUTPUT
    *ddr_h |= 0b100100010001;
    // pinMode(sensorPower, OUTPUT);

    // Set to LOW so no power flows through the sensor
    //digitalWrite(sensorPower, LOW);
    write_ph(4, 0);

    adc_init();
    Serial.begin(9600);
}

void loop() {
    //get the reading from the function below and print it
    int level = readSensor();

    Serial.print("Water level: ");
    Serial.println(level);
    if(level <= 150){
      Serial.println("Water level is too low");
    }

    delay(1000);
}

//This is a function used to get the reading
int readSensor() {
    //digitalWrite(sensorPower, HIGH);    // Turn the sensor ON
    write_ph(4, 1);
    delay(10);                            // wait 10 milliseconds
    //val = analogRead(sensorPin);        // Read the analog value form sensor
    val = adc_read(sensorPower);
    //digitalWrite(sensorPower, LOW);        // Turn the sensor OFF
    write_ph(4, 0);
    return val;                            // send current reading
}





void write_ph(unsigned char pin_num, unsigned char state) // Given function from Lab 3, sets PB at pin number to a given state. Ex. (6, HIGH) makes PB6 HIGH 
{

  if(state == 0)
  {
    *port_h &= ~(0x01 << pin_num);
  }
  else
  {
    *port_h |= 0x01 << pin_num;
  }

}


void my_delay(unsigned int freq) //Given function, acts like the arduino Delay() function by delaying by a certain amount of seconds
{


  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV
  *myTIFR1 |= 0x01;

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
