#include <Arduino.h>
#include <driver_LCD16x2.hpp>
#include <AVR_digitalclock.hpp>
#include <stdio.h>

#ifndef TRUE
  #define TRUE  1
  #define FALSE 0
#endif

#define DIFFERENTIAL_INPUT  0
#define SINGLE_ENDED        1

// register setup definition
#define VREF_AVCC           (1<<REFS0)
#define VREF_INTERNAL       0

// ubah jadi TRUE apabila ingin toggle channel ADC
#define CHANNEL_SEL_ON      FALSE


// GLOBAL VARIABLES ----------------------------------------
bool    change_channel = 0;
double  LM35_sensitivity = 10.0 / 1000.0; // 10 mV / derajat C
char    ADC_ASCII[5];     // char buffers
char    voltage_buf[12];
char    temp_buf[10];
uint8_t channel_position = 0;   // channel selector
uint8_t ADC_channels[8] = {
  0b00000, 0b00001, 0b00010, 0b00011,
  0b00100, 0b00101, 0b00110, 0b00111
};

// FUNCTION PROTOTYPE --------------------------------------
void     init_ADC();
uint16_t baca_ADC(int channel);
double   dec2volt(uint16_t ADC_value, uint8_t channel_type);
void     volt2ASCII(double voltage_value);
void     volt2temp_ASCII(double voltage_value);
void     dec2ASCII(uint16_t ADC_value); 

// MAIN FUNCTION -------------------------------------------
void setup() {

  // Init LCD
  lcd_init(&PORTC, &PORTD, &DDRC, &DDRD, PD0, PD1);
  lcd_command(CLEAR_DISPLAY);

  // Init Jam Digital
  init_jam();

  // Init ADC
  DDRA  = 0x00;
  PORTA = 0x00;
  init_ADC();

  // EXTERNAL INTERRUPT - untuk ganti channel ADC
  // INT0 falling edge trigger
  GICR  |= (1 << INT0);
  MCUCR |= (1 << ISC01);

  // global interrupt enable
  sei();
  delay(2000);
}

void loop() {
  uint16_t ADC_value;

  if(CHANNEL_SEL_ON)
  {
    if(channel_position > 7) channel_position = 0;
    ADC_value = baca_ADC(channel_position);
  }
  else
  {
    ADC_value = baca_ADC(1); // channel 1
  }
  double voltage = dec2volt(ADC_value, SINGLE_ENDED);

  // convert ke ASCII
  volt2ASCII(voltage);
  volt2temp_ASCII(voltage);
  dec2ASCII(ADC_value);

  // display tegangan
  lcd_setpos(0,0);
  lcd_string(voltage_buf);

  // Update jam & Display temperatur dilakukan bersamaan
  if(detik_last != detik)
  {
    // update tampilan temperatur saat detik berubah saja
    lcd_setpos(0, 10);
    lcd_string(temp_buf);

    // update tampilan jam
    update_jam();
    lcd_setpos(1,0);
    lcd_string(jam_buffer);

    // display nilai ADC, untuk keperluan pengukuran
    lcd_setpos(1,10);
    lcd_string(ADC_ASCII);
  }
}


// ISR ----------------------------------------------
// Overflow setiap 1 detik, reset TCNT1 ke 49900
ISR(TIMER1_OVF_vect){
  detik++;
  TCNT1 = 49900;
}

ISR(INT0_vect){
  channel_position++;
}




// FUNCTION DEFINITION ------------------------------
void init_ADC(){
  // Prescaler = 8
  // Enable Auto trigger mode
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADATE);

  // reference voltage -> AVCC, pada minsys, AREF tidak ada header pinout-nya
  // ADMUX = VREF_INT_2_56;
  ADMUX = VREF_AVCC;
  ADMUX &= ~(1 << ADLAR);

  // Auto trigger source = Free running
  SFIOR &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

  // Enable ADC & start the first conversion
  ADCSRA |= (1 << ADEN) | (1 << ADSC);
}

uint16_t baca_ADC(int channel){
  uint16_t ADC_bufferL;
  uint8_t ADC_bufferH;
  uint8_t channel_mask = channel;

  ADMUX &= 0xE0;
  ADMUX |= channel_mask;

  // clear interrupt flag
  ADCSRA |= (1 << ADIF);

  // tunggu ADIF bernilai 1
  while((ADCSRA & (1 << ADIF)) == 0);

  // ambil low nibble terlebih dahulu, agar nilai ADCH & ADCL
  // di lock selama proses fungsi baca_ADC() ini
  ADC_bufferL = ADCL;
  ADC_bufferH = ADCH;
  ADC_bufferL |= (ADC_bufferH << 8);

  // ambil high nibble (ADCH) yang sudah digeser ke kiri 8x, lalu di OR
  // dengan hasil low nibble (ADCL)
  // ADC_buffer |= (ADCH << 8);

  // clear interrupt flag
  ADCSRA |= (1 << ADIF);

  return ADC_bufferL;
}


// HELPER FUNCTIONS ---------------------------------
void dec2ASCII(uint16_t ADC_value){
  sprintf(ADC_ASCII, "%d%d%d%d",
    ADC_value / 1000,
    (ADC_value / 100) % 10,
    (ADC_value / 10) % 10,
    ADC_value % 10
  );
}

double dec2volt(uint16_t ADC_value, uint8_t channel_type)
{
  double LSB;
  if (channel_type == SINGLE_ENDED){
    LSB = 5.0 / 1024.0;
  }
  else if (channel_type == DIFFERENTIAL_INPUT){
    LSB = 5.0 / 512.0;
  }
  return ADC_value * LSB;
}

void volt2ASCII(double voltage_value)
{
  double voltage_mV = voltage_value * 1000;
  unsigned int voltage_integer = voltage_mV;
  unsigned int voltage_comma_values = (voltage_mV - voltage_integer) * 100;

  sprintf(voltage_buf, "%d%d%d%d.%d%dmV", 
           voltage_integer / 1000,
          (voltage_integer / 100) % 10,
          (voltage_integer / 10) % 10,
           voltage_integer % 10,
          (voltage_comma_values / 10) % 10,
           voltage_comma_values % 10);
}

void volt2temp_ASCII(double voltage_value)
{
  double celcius = voltage_value / LM35_sensitivity;
  unsigned int celcius_integer = celcius;
  unsigned int celcius_comma_values = (celcius - celcius_integer) * 100;

  sprintf( temp_buf, "%d%d%d.%d%d",
    (celcius_integer / 100),
    (celcius_integer / 10) % 10,
    celcius_integer % 10,
    (celcius_comma_values / 10) % 10,
    celcius_comma_values % 10
  );
}
