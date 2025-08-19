const int BLINK_PERIOD = 250;
const int LED_1_OUTPUT = 13;
const int LED_2_OUTPUT = 12;
const int LED_3_OUTPUT = 11;
const int LFT_TRN_INPUT = 2;
const int RIGHT_TRN_INPUT = 3;
const int HZRD_INPUT = 4;
const int BRK_INPUT = 5;

int led_pins[] = {8, 9, 10, 11, 12, 13};
int left_led[] = {11, 12, 13};
int right_led[] = {8, 9, 10};

int pin_count = 6;
bool brake_depress = true; 
bool brake_press = false; 
int lft_trn_state, right_trn_state, hzrd_state, brk_state = 0;
unsigned long current_millis, previous_millis = 0; 
int interval = 0; 
int i = 0; 


void shut_off_all_leds()
{
  if(!brake_press)
  {
    for(int i=0; i < pin_count; i++)
    {
      digitalWrite(led_pins[i], LOW);
    }
  }
  
}

void delay_or_brake(int ms_time)
{
  current_millis = millis();
  brk_state = digitalRead(BRK_INPUT);
   
  previous_millis = current_millis; 

  Serial.println("in function");
  while(current_millis - previous_millis <= ms_time && brk_state == HIGH)
  {
    Serial.println("in while");
    brk_state = digitalRead(BRK_INPUT);
    current_millis = millis();
  }
  Serial.println("leaving function");
}


void setup() {

  Serial.begin(9600);

  for(int i=0; i < pin_count; i++)
  {
    pinMode(led_pins[i], OUTPUT);
  }
  
  pinMode(LFT_TRN_INPUT, INPUT);
  pinMode(RIGHT_TRN_INPUT, INPUT);
  pinMode(HZRD_INPUT, INPUT);
  pinMode(BRK_INPUT, INPUT_PULLUP);

}


void loop() {
  // put your main code here, to run repeatedly:
  lft_trn_state = digitalRead(LFT_TRN_INPUT);
  right_trn_state = digitalRead(RIGHT_TRN_INPUT);
  hzrd_state = digitalRead(HZRD_INPUT);
  brk_state = digitalRead(BRK_INPUT);

  if(brake_depress == true)
  {
    shut_off_all_leds();
  }
  
  if(brk_state == LOW)
  {
    for(i=0; i < pin_count; i++)
    {
      digitalWrite(led_pins[i], HIGH);
    }

    brake_depress = false;
    brake_press = true; 
  } 
  else if(brk_state == HIGH && brake_depress == false)
  {
    brake_depress = true; 
    brake_press = false; 
  }
  else if(hzrd_state == HIGH)
  {
    for(i=0; i < pin_count/2; i++)
    {
      digitalWrite(left_led[i], HIGH);
      digitalWrite(right_led[i], HIGH);
      delay_or_brake(BLINK_PERIOD);
    }
    shut_off_all_leds();
    delay_or_brake(BLINK_PERIOD);    
  }
  else if(lft_trn_state == HIGH && right_trn_state != HIGH)
  {
    for(i=0; i < pin_count/2; i++)
    {
      digitalWrite(left_led[i], HIGH);
      delay_or_brake(BLINK_PERIOD);
    }
    shut_off_all_leds();
    delay_or_brake(BLINK_PERIOD);
  }
  else if(right_trn_state == HIGH && lft_trn_state != HIGH)
  {
    for(i=0; i < pin_count/2; i++)
    {
      digitalWrite(right_led[i], HIGH);
      delay_or_brake(BLINK_PERIOD);
    }
    shut_off_all_leds();
    delay_or_brake(BLINK_PERIOD);
  }
  
}
