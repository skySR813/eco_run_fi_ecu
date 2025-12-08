#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
//こっちはメインとの通信をUARTを使う場合
//長距離は不可

LiquidCrystal_I2C lcd(0x27,16,2);

String rxBuffer = "";

volatile bool pinA = false;
volatile bool pinB = false;

volatile byte current = 0;
volatile byte previous = 0;

int cw[] = {1, 3, 0, 2};
int ccw[] = {2, 0, 3, 1};

int input_A = 2;
int input_B = 3;
int btn_1 = 8;

int oldSW = 1;
volatile int btn_flag = 1;

// 各画面ごとのカウンタ変数
volatile double counter_main = 0;
volatile double counter_fuel1 = 0;
volatile double counter_fuel2 = 0; 
volatile double counter_ign1 = 0;
volatile double counter_ign2 = 0;

//カウンタ変数直前の値を保存（eeprom保護用）
static double old_main = -1;
static double old_fuel1 = -1;
static double old_fuel2 = -1;
static double old_ign1 = -1;
static double old_ign2 = -1;

// 現在操作中のカウンタを指すポインタ
volatile double* activeCounter = &counter_main;

void setup() {
  Serial.begin(115200);
  pinMode(input_A, INPUT);
  pinMode(input_B, INPUT);
  pinMode(btn_1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(input_A), rotary, CHANGE);
  attachInterrupt(digitalPinToInterrupt(input_B), rotary, CHANGE);

  //eeoromからデータ持ってくる
  EEPROM.get(0, counter_main);
  EEPROM.get(sizeof(double) * 1, counter_fuel1);
  EEPROM.get(sizeof(double) * 2, counter_fuel2);
  EEPROM.get(sizeof(double) * 3, counter_ign1);
  EEPROM.get(sizeof(double) * 4, counter_ign2);
  //
  
  //メインに初期数値送信
  Serial.print("<SET:main=");
  Serial.print(counter_main,3);
  Serial.println(">");
  delay(1);

  Serial.print("<SET:fuel1=");
  Serial.print(counter_fuel1,3);
  Serial.println(">");
  delay(1);

  Serial.print("<SET:fuel2=");
  Serial.print(counter_fuel2,3);
  Serial.println(">");
  delay(1);

  Serial.print("<SET:ign1=");
  Serial.print(counter_ign1,3);
  Serial.println(">");
  delay(1);

  Serial.print("<SET:ign2=");
  Serial.print(counter_ign2,3);
  Serial.println(">");
  //

  lcd.init(); 
  lcd.backlight();
  lcd.print("loading...");
  delay(1000);
  lcd.clear();
  //unsigned int eeprom_size = EEPROM.length();
  //Serial.println(eeprom_size);
}

void loop(){
  int SW = digitalRead(btn_1);

  // ボタンで画面切り替え
  if(oldSW != SW && SW == LOW){
    btn_flag++;
    if(btn_flag > 5) btn_flag = 1;

    // 画面に応じてポインタ切り替え
    switch(btn_flag){
      case 1: activeCounter = &counter_main; break;
      case 2: activeCounter = &counter_fuel1; break;
      case 3: activeCounter = &counter_fuel2; break;
      case 4: activeCounter = &counter_ign1; break;
      case 5: activeCounter = &counter_ign2; break;
    }

    lcd.clear();
    showScreen(); // 画面更新
    delay(200);
  }
  oldSW = SW;

  showScreen();
  delay(100);
}

void showScreen() {
  lcd.setCursor(0,0);
  switch(btn_flag){
    case 1: lcd.print("Main Screen"); break;
    case 2: lcd.print("Fuel Setting 1"); break;
    case 3: lcd.print("Fuel Setting 2"); break;
    case 4: lcd.print("Ign Setting 1"); break;
    case 5: lcd.print("Ign Setting 2"); break;
  }
  lcd.setCursor(0,1);
  lcd.print("Value: ");
  lcd.print(*activeCounter, 3); // ポインタ先の値を表示


  //ポインタ内の値が変わったときのみEEPROMに保存
  if (counter_main != old_main) {
  EEPROM.put(0, counter_main);
  Serial.print("<SET:main=");
  Serial.print(counter_main,3);
  Serial.println(">");
  old_main = counter_main;
  }

  if (counter_fuel1 != old_fuel1) {
  EEPROM.put(sizeof(double) * 1, counter_fuel1);
  Serial.print("<SET:fuel1=");
  Serial.print(counter_fuel1,3);
  Serial.println(">");
  old_fuel1 = counter_fuel1;
  }

  if(counter_fuel2 != old_fuel2){
    EEPROM.put(sizeof(double) * 2,counter_fuel2);
    Serial.print("<SET:fuel2=");
    Serial.print(counter_fuel2,3);
    Serial.println(">");
    old_fuel2 = counter_fuel2;
  }

  if(counter_ign1 != old_ign1){
    EEPROM.put(sizeof(double) * 3,counter_ign1);
    Serial.print("<SET:ign1=");
    Serial.print(counter_ign1,3);
    Serial.println(">");
    old_ign1 = counter_ign1;
  }

  if(counter_ign2 != old_ign2){
    EEPROM.put(sizeof(double) * 4,counter_ign2);
    Serial.print("<SET:ign2=");
    Serial.print(counter_ign2,3);
    Serial.println(">");
    old_ign2 = counter_ign2;
  }

}

// --- ロータリーエンコーダー割込み処理 ---
void rotary() {
  pinA = digitalRead(input_A);
  pinB = digitalRead(input_B);
  current = pinA + pinB * 2;

  if (current == cw[previous]) *activeCounter += 0.125;
  if (current == ccw[previous]) *activeCounter -= 0.125;

  if (*activeCounter < 0) *activeCounter = 0;
  previous = current;
}
