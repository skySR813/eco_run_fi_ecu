#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);


volatile bool pinA = false;
volatile bool pinB = false;
bool lc2 = true;
bool lc3 = true;
bool lc4 = true;
bool lc5 = true;
volatile byte current = 0;
volatile byte previous = 0;
volatile double  counter = 0;

static double old_counter = -1;
        
int cw[] = {1, 3, 0, 2}; //エンコーダー用
int ccw[] = {2, 0, 3, 1};

int input_A = 2;//ピン番号設定
int input_B = 3;
int btn_1 = 8;


int oldSW = 1;
char line1[15] = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE};//アイウエオカキクケコサシスセ
char line2[15];
char floatString[10];   
char fcount[10];

volatile int btn_flag = 1;
volatile int old_btn_flag = 1;


void setup() {
  Serial.begin(9600);
  pinMode(input_A, INPUT);
  pinMode(input_B, INPUT);
  pinMode(btn_1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(input_A), rotary, CHANGE);
  attachInterrupt(digitalPinToInterrupt(input_B), rotary, CHANGE);
  

   rotary();
   
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Ready...");
  delay(1500);
 
}

void loop(){
  int SW = digitalRead(btn_1);
  dtostrf(counter, 3, 1, floatString);
  snprintf(line2, 15, "Counter: %s", floatString);
  snprintf(fcount, 15, "fcount: %s",btn_flag);
  


 if(oldSW != SW){  //画面を切り替えるための値 5画面　
     btn_flag = btn_flag + 1;
     if(btn_flag == 6){
      //ここのif文の数は増やしたい画面の数＋１にしてね
      btn_flag = 1;
      }
  }
   
   if(btn_flag == 1){
    //画面1 メイン画面の予定、速度タコメーターなど　
    lc2 = true;
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
      if(old_counter != counter){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(line1);
      lcd.setCursor(0, 1);
      lcd.print(line2);
      old_counter = counter;
       }
    }else if(btn_flag == 2) { 
      //画面２　燃調設定画面その１
      lc3 = true;
      if (lc2){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("fuel setting");
      lcd.setCursor(0, 1);
      lcd.print("No,1");
      lc2 = false;
      }
    }else if(btn_flag == 3){
      //画面３　燃調設定画面その２ 
      lc4 = true;
      if(lc3){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("fuel setting");
      lcd.setCursor(0, 1);
      lcd.print("No,2");
      lc3 = false;
      }
    }else if(btn_flag == 4){
      //画面４　点火進角設定その１
      lc5 = true;
      if(lc4){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ign setting");
      lcd.setCursor(0, 1);
      lcd.print("No,1");
      lc4 = false;
      }
    }else if(btn_flag == 5){
      //画面５　点火進角設定その２
      if(lc5){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ign setting");
      lcd.setCursor(0, 1);
      lcd.print("No,2");
      lc5 = false;
      }
    }
  Serial.print("btnflag: ");
  Serial.println(btn_flag);
  delay(100);
}

















void rotary()
{
  //現在の状態を取得
  pinA = digitalRead(input_A);
  pinB = digitalRead(input_B);

  //2ビットで0～3の数字に変換
  current = pinA + pinB * 2;

  //currentの値が、CW/CCWのどちらか期待する方向だったらカウントを増減
  if (current == cw[previous]){
    counter = counter+0.125;
  }
  if (current == ccw[previous]){
    counter = counter-0.125;
  }

  if(counter <= 0){
    counter = 0;
  }

  previous = current;
}
