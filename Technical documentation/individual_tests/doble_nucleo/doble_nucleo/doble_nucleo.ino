TaskHandle_t Task1;

void loop2(void *parameter) {
for(;;){
  Serial.println("\t\t\tNúcleo tarea: " + String(xPortGetCoreID()));
  delay(50);
  }
  vTaskDelay(10);
}

void setup() {
  xTaskCreatePinnedToCore(
    loop2,
    "Task_1",
    10000,
    NULL,
    1,
    &Task1,
    0);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Núcleo tarea: " + String(xPortGetCoreID()));
  delay(50);
}
