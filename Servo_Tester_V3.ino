/*************************************************/
//            two channel Servo tester
// 
//   Mode 0 = OFF
//   Mode 1 = two channel Servo PPM output
//   Mode 2 = Neutral mode for both outputs
//   Mode 3 = reads an input PPM on channel #2
//
//   Button = short press -> switch mode
//            long press  -> output enable ON/OFF
/*************************************************/

#include <U8g2lib.h>
#include <Servo.h>
#include <SPI.h>
#include <PinChangeInt.h>

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 6, /* dc=*/ 7, /* reset=*/ 8);

Servo ppm1;
Servo ppm2;

#define BUTTON_LONG_PRESS       500

const uint8_t _pin_input_Button = 5;     // input Button; needs internal pullup activated
const uint8_t _pin_input_Poti_ppm1 = A1; // input Poti for PPM1 output signal
const uint8_t _pin_input_Poti_ppm2 = A0; // input Poti for PPM2 output signal

const uint8_t _pin_output_PPM1 = 9;      // PPM1 output pin
const uint8_t _pin_output_PPM2 = 10;     // PPM2 output pin

int tacho_x3;
int tacho_y3;

int int_ppm1_available = 0;
int int_ppm2_available = 0;

uint16_t g16_ppm1_poti_value = 0;
uint16_t g16_ppm2_poti_value = 0;

uint16_t g16_ppm1_output_ms = 0;
uint16_t g16_ppm2_output_ms = 0;

uint8_t g16_ppm1_angle_deg = 0;
uint8_t g16_ppm2_angle_deg = 0;

uint8_t g16_ppm1_angle_deg_backup = 0;
uint8_t g16_ppm2_angle_deg_backup = 0;

uint8_t servo_tester_mode = 0;        // Selector State (Initial state = ALL OFF)

uint8_t servo_output_enable = 0;      // enable PPM output flag

volatile unsigned long ch[7], t[8];   // for PPM input read
volatile int pulse = 0;

//-----------------------------[ Setup ]------------------------------------
void setup () 
{
  // setup Serial console
  Serial.begin(38400);
  Serial.println();
  Serial.println("2 channel Servo Tester");
  Serial.println("======================");

  // setup input/output pins
  pinMode(_pin_output_PPM1, OUTPUT);
  pinMode(_pin_output_PPM2, OUTPUT);
  
  pinMode(_pin_input_Button, INPUT_PULLUP);
  pinMode(_pin_input_Poti_ppm1, INPUT);
  pinMode(_pin_input_Poti_ppm2, INPUT);
  pinMode(4, INPUT);

  // setup Servo pins
  ppm1.detach();  // attached servo on pin 9
  ppm2.detach();  // attached servo on pin 10
  
  // setup SPI speed
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // setup OLED display
  u8g2.begin();

  // init interrrupt function for channel read
  attachPinChangeInterrupt(4, read_ppm_int, CHANGE);
}

//-----------------------------[ Loop ]------------------------------------
void loop(){

  uint8_t button_state = 0;
  button_state = button_short_long_press();
  
  //--------------- checks Servo Test Mode in main loop -------------------------
  if(button_state == 1){  //if short press
    servo_output_enable = 0;
    servo_tester_mode++;
    if(servo_tester_mode > 3){
      servo_tester_mode = 0;
    }
    Serial.print("Mode: ");Serial.println(servo_tester_mode);
  }

  //--------------- Servo output enable -------------------------
  if(button_state == 2){  //if short press
    if(servo_output_enable == 0)
    {
      servo_output_enable = 1;
      Serial.println("Servo Output enabled!");
    }
    else
    {
      servo_output_enable = 0;
      Serial.println("Servo Output disabled!");
    }
  }
  
  //----------------- checks poti positions ------------------------------------
  g16_ppm1_poti_value = get_adc_value(1,16);
  g16_ppm2_poti_value = get_adc_value(0,16);

  g16_ppm1_output_ms = map(g16_ppm1_poti_value, 0, 1023, 1000, 2000);
  g16_ppm2_output_ms = map(g16_ppm2_poti_value, 0, 1023, 1000, 2000);
  
  g16_ppm1_angle_deg = map(g16_ppm1_poti_value, 0, 1023, 0, 180);
  g16_ppm2_angle_deg = map(g16_ppm2_poti_value, 0, 1023, 0, 180);

  //----------------- checks for NO action mode 0 ------------------------------
  if(servo_tester_mode == 0){

    u8g2.firstPage();  
    do {
        start_mode();
    } while( u8g2.nextPage() );
       
    ppm1.detach();  // attached servo on pin 9
    ppm2.detach();  // attached servo on pin 10
   
    g16_ppm1_angle_deg_backup = 0;
  }

  //----------------- checks for Normal mode -----------------------------------
  if(servo_tester_mode == 1){
    
    //Serial.println("Normal Mode:");
    
    //attach servo pins to switch servo control on if not already attached
    check_servo_attached();
    
    if(servo_output_enable == 1)
    {
      ppm1.writeMicroseconds(g16_ppm1_output_ms);
      ppm2.writeMicroseconds(g16_ppm2_output_ms);
    }
    else if(servo_output_enable == 0)
    {
      ppm1.write(90);
      ppm2.write(90);
    }
    
    //update the Oled display only if Poti values are changed
    if(g16_ppm1_angle_deg != g16_ppm1_angle_deg_backup  || 
       g16_ppm1_angle_deg != g16_ppm1_angle_deg_backup  ||
       g16_ppm2_angle_deg != g16_ppm2_angle_deg_backup  || 
       g16_ppm2_angle_deg != g16_ppm2_angle_deg_backup)
    {
       
      u8g2.firstPage();  
      do {
        normal_mode();
      } while( u8g2.nextPage() );
    
      g16_ppm1_angle_deg_backup = g16_ppm1_angle_deg;
      g16_ppm2_angle_deg_backup = g16_ppm2_angle_deg;   
     }
  }

//----------------- checks for Servo neutral mode -----------------------------------
  if(servo_tester_mode == 2){

    //Serial.println("Neutral Mode:");
    
    //attach servo pins to switch servo control on if not already attached
    check_servo_attached();

    g16_ppm1_angle_deg = 90;
    g16_ppm2_angle_deg = 90;
    
    ppm1.write(g16_ppm1_angle_deg);
    ppm2.write(g16_ppm2_angle_deg);
    
    //update the Oled display only if Poti values are changed
    
    if(g16_ppm1_angle_deg != g16_ppm1_angle_deg_backup  || 
       g16_ppm1_angle_deg != g16_ppm1_angle_deg_backup  ||
       g16_ppm2_angle_deg != g16_ppm2_angle_deg_backup  || 
       g16_ppm2_angle_deg != g16_ppm2_angle_deg_backup)
    {
        
      u8g2.firstPage();  
      do {
        neutral_mode();
      } while( u8g2.nextPage() );

      g16_ppm1_angle_deg_backup = g16_ppm1_angle_deg;
      g16_ppm2_angle_deg_backup = g16_ppm2_angle_deg;
      
    }
  }

//----------------- checks for Servo input mode -----------------------------------
  if(servo_tester_mode == 3){

    //Serial.println("Read Mode:");

    //attach servo pins to switch servo control on if not already attached
    check_servo_attached();
    
    ppm2.writeMicroseconds(g16_ppm2_output_ms); 
    g16_ppm1_angle_deg = map(ch[1], 1000, 2000, 0, 180);

    //update the Oled display only if Poti values are changed 
    
    if(g16_ppm1_angle_deg != g16_ppm1_angle_deg_backup  || 
       g16_ppm1_angle_deg != g16_ppm1_angle_deg_backup  ||
       g16_ppm2_angle_deg != g16_ppm2_angle_deg_backup  || 
       g16_ppm2_angle_deg != g16_ppm2_angle_deg_backup)
    {
    
      //Serial.println(ch[1]);  
      u8g2.firstPage();  
      do {
        read_mode();
      } while( u8g2.nextPage() );

      g16_ppm1_angle_deg_backup = g16_ppm1_angle_deg;
      g16_ppm2_angle_deg_backup = g16_ppm2_angle_deg;
      
    }
  }
}

//-----------------------------[ OLED Screen ]-------------------------------
void start_mode(){
  
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(2, 32, "Servo Tester");
  u8g2.setFont(u8g_font_unifont);
  u8g2.drawStr(10,50, "press Button!");
}
//--------------------------------------------------------------------------

void normal_mode(){
  
  u8g2.drawLine(64, 0, 64, 32);
  
  draw_tacho(32,32);
  draw_tacho(96,32);

  u8g2.setFont(u8g_font_unifont);
  u8g2.drawStr(60,60,"M");
  u8g2.drawStr(1,10, "1");
  u8g2.drawStr(68,10,"2");

  if(servo_output_enable == 1)
  {
    u8g2.drawStr(52,47,"[X]");
  }
  else
  {
    u8g2.drawStr(52,47,"[ ]");
  }
  
  //Position and display PPM1
  calc_tacho_bar(g16_ppm1_angle_deg, 32, 32, 20);   //abgle, bar x_start, bar y_start, bar lenght; returnes tacho_x3,tacho_y3 as global variable
  u8g2.drawLine(32,32,tacho_x3,tacho_y3);

  //Position and display PPM2
  calc_tacho_bar(g16_ppm2_angle_deg, 96, 32, 20);
  u8g2.drawLine(96,32,tacho_x3,tacho_y3);
  
  char buffer[3];
  dtostrf(g16_ppm1_angle_deg,5,0,buffer);
  u8g2.drawStr(4, 55, buffer);

  dtostrf(g16_ppm2_angle_deg,5,0,buffer);
  u8g2.drawStr(68, 55, buffer);
}
//--------------------------------------------------------------------------

void neutral_mode(){
  
  u8g2.drawLine(64, 0, 64, 32);
  
  draw_tacho(32,32);
  draw_tacho(96,32);

  u8g2.setFont(u8g_font_unifont);
  u8g2.drawStr(60,60,"N");
  u8g2.drawStr(1,10, "1");
  u8g2.drawStr(68,10,"2");

  
  //Position and display PPM1
  calc_tacho_bar(g16_ppm1_angle_deg, 32, 32, 20);   //abgle, bar x_start, bar y_start, bar lenght; returnes tacho_x3,tacho_y3 as global variable
  u8g2.drawLine(32,32,tacho_x3,tacho_y3);

  //Position and display PPM2
  calc_tacho_bar(g16_ppm2_angle_deg, 96, 32, 20);
  u8g2.drawLine(96,32,tacho_x3,tacho_y3);
  
  char buffer[3];
  dtostrf(g16_ppm1_angle_deg,5,0,buffer);
  u8g2.drawStr(4, 55, buffer);

  dtostrf(g16_ppm2_angle_deg,5,0,buffer);
  u8g2.drawStr(68, 55, buffer);
}
//--------------------------------------------------------------------------

void read_mode(){
  
  draw_tacho(64,32);
  
  u8g2.setFont(u8g_font_unifont);
  u8g2.drawStr(60,64,"R");

  //Position and display PPM1
  calc_tacho_bar(g16_ppm1_angle_deg, 64, 32, 20);
  u8g2.drawLine(64, 32, tacho_x3, tacho_y3);

  char buffer[3];
  dtostrf(g16_ppm1_angle_deg,5,0,buffer);
  u8g2.drawStr(60, 55, buffer);

}
//--------------------------------------------------------------------------

//-----------------------------[ gets ADC value ]-------------------------------
uint16_t get_adc_value(uint8_t channel, uint16_t count){

  uint32_t adc_value = 0;

  if(count > 128){ //limits adc cycles to max 128, else use 1 cycle
    count = 1;
  }

  for (uint16_t i = 0; i < count; i++) {
        adc_value += analogRead(channel);
        delay(1);
    }

  return adc_value /= count;
}

//-----------------------------[ check button short/long press ]-------------------------------
uint8_t button_short_long_press(void){
  
  uint8_t button_state = 0;
  
  static uint8_t button_active = 0;
  static uint8_t button_long_press = 0;
  static uint32_t time_start = 0; 
  static uint32_t time_stop = 0;
  
  if(digitalRead(_pin_input_Button) == LOW){
    
    if(time_start <= 0 && button_active == 0)
    {
      button_active = 1;
      time_start = millis();
    }
     
    time_stop = millis();

    if((time_stop > (time_start + BUTTON_LONG_PRESS)) && button_long_press == 0)
    {
      //Serial.println("long press");
      button_state = 2;
      button_long_press = 1;
      time_start = 0;
      time_stop = 0;
    }
  }
  else
  {
    if(button_active == 1)
    {    
      if(button_long_press == 1)
      {
        button_long_press = 0;
        time_start = 0;
        time_stop = 0;
      }
      else
      {
        button_state = 1;
        //Serial.println("short press");
        time_start = 0;
        time_stop = 0;
      }
    
      button_active = 0;
    }  
   }
  return button_state;
}

//-----------------------------[ check servo attached ]-----------------------------------
uint8_t check_servo_attached(void){
  
  uint8_t result = 0;
  
  //attach servo pins to switch servo control on if not already attached
  if(ppm1.attached() == 0 && ppm2.attached() == 0)
  {
    ppm1.attach(_pin_output_PPM1);  // attached servo on pin 9
    ppm2.attach(_pin_output_PPM2);  // attached servo on pin 10
    
    result = 1;
  }
  
  return result;
}
//-----------------------------------[ draw Tacho ]--------------------------------------
void draw_tacho(uint8_t x_pos, uint8_t y_pos){
  
  u8g2.drawLine  (x_pos, 8, x_pos, 13);
  u8g2.drawCircle(x_pos, y_pos, 2,  U8G2_DRAW_ALL);
  u8g2.drawCircle(x_pos, y_pos, 25, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(x_pos, y_pos, 25, U8G2_DRAW_UPPER_LEFT);
}

//-----------------------[ calculate analog tacho bar angle ]---------------------------
void calc_tacho_bar(uint8_t ppm_deg, uint8_t x3_start, uint8_t y3_start, uint8_t bar_lenght){
  
  float angle = ppm_deg ;       //Retrieve stored seconds and apply
  
  angle = (angle / -57) ;       //Convert degrees to radians  angle=(angle/57.29577951) ; //Convert degrees to radians  
  tacho_x3 = (x3_start-(cos(angle)*(bar_lenght)));
  tacho_y3 = (y3_start+(sin(angle)*(bar_lenght)));
}

//-----------------------[ reads input PPM Signal ]---------------------------
void read_ppm_int(void) {

    detachPinChangeInterrupt(4);
    
    t[pulse] = micros();
    
    if(pulse == 1){
      ch[1] = (t[1] - t[0]) - 8; //-8 = timing differnenz
        pulse = 0;
        if (ch[1] > 3000) {
          t[0] = 0;
          t[1] = 0;
          pulse = 0;
        }
    }else if(pulse == 0){
      pulse++;
    }
    
    attachPinChangeInterrupt(4, read_ppm_int, CHANGE);
} 

//-----------------------------------[ END ]---------------------------------------
