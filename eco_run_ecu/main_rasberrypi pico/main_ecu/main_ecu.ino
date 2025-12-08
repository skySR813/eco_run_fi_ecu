#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <event_groups.h>

# define LED 2

void TaskBlink(void *pvParameters);
void TaskChecker(void *pvParameters);


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  xTaskCreate(TaskBlink, "Blink", 128, NULL, 2, NULL);

  xTaskCreate(TaskChecker, "Checker", 128, NULL, 1, NULL);

}

void loop() {
}


unsigned char led_state;

void TaskBlink(void *pvParameters)
{
  (void)pvParameters;

  pinMode(LED, OUTPUT);

  for (;;)
  {
    digitalWrite(LED, HIGH);
    led_state = HIGH;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    led_state = LOW;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void TaskChecker(void *pvParameters)
{
  (void)pvParameters;

  int value = analogRead(A0);
  int i = 0;
  for (;;)
  {

    value = analogRead(A0);

    Serial.println(value);
    delay(100);
    i++;
    if (i == 30)
    {
      if ((value < 700) && (led_state == HIGH))
      {
        Serial.println("くらい");
        i = 0;
      }
      else if ((value > 900) && (led_state == HIGH))
      {
        Serial.println("明るすぎ！");
        i = 0;
      }
      else
      {
        i = 0;
      }
    }
    vTaskDelay(1);
  }
}
