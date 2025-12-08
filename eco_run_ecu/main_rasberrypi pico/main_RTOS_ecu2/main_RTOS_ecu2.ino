/*
 * ecorun_ecu ver0.0
 * メインecuのプログラムです
 * このプログラムは現在開発中です
 * <使用方法>
 * 未定、これから考える
 * <トラブルがあったら↓>
 * 
 * 
 */
#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <event_groups.h>

//　時間的な処理がシビアな処理はコア0、それ以外はコア1にする。
//　RP2350を採用しているマイコンを使用すること。
//　ラズパイpico2使用

//エンジンマップ設定値
double map_main = 0.0;
double map_fuel1 = 0.0;
double map_fuel2 = 0.0;
double map_ign1 = 0.0;
double map_ign2 = 0.0;

//速度、タイムなど
double speedKmh = 0.0;
double timesecmin = 0.0;

//温度管理用（センサーの定数など)
const float BETA = 3950.0;     // サーミスタのB定数
const float R25 = 10000.0;     // 25℃でのサーミスタの抵抗値 (10kΩ)
const float T0 = 298.15;       // 25℃ = 298.15K
const float VREF = 3.3;        // 参照電圧
const float R_FIXED = 10000.0; // 分圧抵抗 (10kΩ)

//通信用
String recvLine = "";  // 1行分の受信バッファ

// ピン定義
const int Cranksensor_PIN = ;
const int Camsensor _PIN = ;
const int Igoutput_PIN =;
const int Fueloutput_PIN =;
const int Throttlesensor_PIN = 26;//ADC 0
const int Tmpsensor_PIN = 27;//ADC 1
const int serial_1_TX = 0;//メインサブ通信用 GP0
const int serial_1_RX = 1;//メインサブ通信用 GP1

// RTOSオブジェクト
QueueHandle_t sensorQueue;
SemaphoreHandle_t controlDoneSemaphore;
EventGroupHandle_t initEventGroup;

// イベントビット定義
#define BIT_SENSOR_READY   (1 << 0)
#define BIT_CONTROL_READY  (1 << 1)
#define BIT_COMM_READY     (1 << 2)
#define BIT_ALL_READY      (BIT_SENSOR_READY | BIT_CONTROL_READY | BIT_COMM_READY)

//==================================================
// クランク・カム角検出タスク（Core 0）
//==================================================
void taskCrank(void *pvParameters) {
  int sensorValue = 0;
  for (;;) {
    sensorValue = analogRead(SENSOR_PIN);
    xQueueSend(sensorQueue, &sensorValue, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hzで取得
  }
}

//==================================================
// 点火制御タスク（Core 0）
//==================================================
void taskIgnition(void *pvParameters) {
  int recvValue;
  float outputValue = 0.0f;

  for (;;) {
    if (xQueueReceive(sensorQueue, &recvValue, portMAX_DELAY) == pdPASS) {
      // 仮の制御計算（例：比例制御）
      outputValue = recvValue * 0.1f;
      Serial.printf("[CTRL] Sensor=%d, Output=%.2f\n", recvValue, outputValue);

      // 制御が完了したらセマフォを解放
      xSemaphoreGive(controlDoneSemaphore);
    }
  }
}

//==================================================
// 燃料噴射タスク（Core 0）
//==================================================
void taskFuel(void *pvParameters) {
  for (;;) {
    // セマフォ待ち：制御完了通知を受ける
    if (xSemaphoreTake(controlDoneSemaphore, portMAX_DELAY) == pdTRUE) {
      // 通信処理（例：シリアル送信）
      Serial.println("[COMM] Sending control data...");
    }
  }
}

//==================================================
// スロットル開度検出タスク（Core 0）
//==================================================
void taskThrottle(void *pvParameters) {
  for (;;) {
    int THinput = analogRead(Throttlesensor_PIN);
    float THper = (THinput * 100.0)/4095; 
    
    }
  }
}

//==================================================
// 温度検出タスク（Core 0）
//==================================================
void taskTmp(void *pvParameters) {
  for (;;) {
    int TMPinput = analogRead(Tmpsensor_PIN);
    float TMP =  GetTMP(adc_value);
    }
  }
}

float GetTMP(int TMPinput){
  float voltage = (TMPinput / 4095.0) * VREF;//ADC値を電圧に変換
  float resistance = (R_FIXED * voltage) / (VREF - voltage); // サーミスタの抵抗値計算

  // Steinhart-Hart 方程式を用いて温度 (K) を算出
    float temperatureK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * logf(resistance / R25));

    // 絶対温度 (K) から摂氏 (°C) に変換
    return temperatureK - 273.15;
  }

//==================================================
// 設定・ログ通信用タスクなど（メインの受信用）（Core 1）※通信はUARTをRS232Cレベルに変換
//==================================================
void taskLogger(void *pvParameters) {
  for (;;) {
    while(Serial1.available()>o){
      char c = Serial1.read();
      if(c == '\n'){
        parseLine(recvLine);
        recvLine = "";
        }else{
          recvLine += c;
          }
      }
  }
}

// 受信した1行を解析する
void parseLine(String line) {

  line.trim();  // 不要な空白削除

  // 例: <SET:fuel1=1.234>
  if (!line.startsWith("<SET:") || !line.endsWith(">")) {
    Serial.print("Invalid: ");
    Serial.println(line);
    return;
  }

  // かっこを除去
  line = line.substring(5, line.length() - 1);
  // line = "fuel1=1.234"

  int equalPos = line.indexOf('=');
  if (equalPos < 0) return;

  String key = line.substring(0, equalPos);
  String val = line.substring(equalPos + 1);

  double value = val.toFloat();

  // ---- キーごとに更新する ----
  if (key == "main") {
    map_main = value;
  } else if (key == "fuel1") {
    map_fuel1 = value;
  } else if (key == "fuel2") {
    map_fuel2 = value;
  } else if (key == "ign1") {
    map_ign1 = value;
  } else if (key == "ign2") {
    map_ign2 = value;
  } else {
    Serial.print("Unknown key: ");
    Serial.println(key);
    return;
  }

  // --- 受信確認表示 ---
  Serial.print("Updated ");
  Serial.print(key);
  Serial.print(" = ");
  Serial.println(value);
}


//==================================================
// UI更新通信用タスク　速度、タイムなど　（メインの送信用）（Core 1）※通信はUARTをRS232Cレベルに変換
//==================================================
void taskUI(void *pvParameters) {
  for (;;) {

    
  }
}

//==================================================
// セットアップ
//==================================================
void setup() {
  Serial.begin(115200);//USB（シリアルモニタ用）
  Serial1.begin(115200);//サブとの通信用　serial 1
  pinMode(Cranksensor_PIN, INPUT);
  pinMode(Camsensor_PIN, INPUT);
  pinMode(Igoutpit_PIN, OUTPUT);
  pinMode(Fueloutput_PIN, OUTPUT);
  pinMode(Throttlesensor_PIN, INPUT);
  pinMode(Tmpsensor_PIN, INPUT);

  // --- RTOSオブジェクト生成 ---
  sensorQueue = xQueueCreate(10, sizeof(int));
  controlDoneSemaphore = xSemaphoreCreateBinary();
  initEventGroup = xEventGroupCreate();

  // --- タスク生成 ---
  // Core指定は Arduino-Pico の SMP版で自動割り当てに任せてもOK
  xTaskCreateAffinitySet(taskCrank, "Crank", 512, NULL, 3, 1 << 0, NULL); // Core0専用
  xTaskCreateAffinitySet(taskIgnition, "Ignition", 512, NULL, 2, 1 << 0, NULL); // Core0専用
  xTaskCreateAffinitySet(taskFuel,   "Fuel",   512, NULL, 1, 1 << 0, NULL); // Core0専用
  xTaskCreateAffinitySet(taskThrottle,   "Throttle",   512, NULL, 1, 1 << 0, NULL); // Core0専用
  xTaskCreateAffinitySet(taskTmp,    "Tmp",   512, NULL, 1, 1 << 0, NULL); // Core0専用
  xTaskCreateAffinitySet(taskLogger,    "Logger",    256, NULL, 1, 1 << 1, NULL); // Core1専用
  xTaskCreateAffinitySet(taskUI,    "UI",    256, NULL, 1, 1 << 1, NULL); // Core1専用


  // 全タスク初期化完了フラグ設定
  xEventGroupSetBits(initEventGroup, BIT_ALL_READY);

  // スケジューラ開始
  vTaskStartScheduler();


  //初期設定値更新
  while(Serial1.available()>o){
      char c = Serial1.read();
      if(c == '\n'){
        parseLine(recvLine);
        recvLine = "";
        }else{
          recvLine += c;
          }
      }
}

void loop() {
  // RTOS使用中はloop()は空でOK
}
