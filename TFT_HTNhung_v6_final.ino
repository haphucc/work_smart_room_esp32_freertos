#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

/*Include Libs*/
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <DHT.h>
#include <ESP32Servo.h>

/*GPIO Def*/
#define cb_dht 23
#define cb_mq2 27
#define coi_quat 25
#define echo_sr04 21               
#define trig_sr04 19
#define mo_cua 26

#define button_coi_quat 18
#define button_cua 5
#define button_mode 22

SemaphoreHandle_t xSema_coi_quat;
SemaphoreHandle_t xSema_cua;
SemaphoreHandle_t xSema_mode;

TickType_t btnDebounce1 = 0;
TickType_t btnDebounce2 = 0;
TickType_t btnDebounce3 = 0;
int state = 1;

void IRAM_ATTR ISR_coi_quat() {
  btnDebounce1 = xTaskGetTickCountFromISR();
  xSemaphoreGiveFromISR(xSema_coi_quat, NULL);
  state =! state;
}

void IRAM_ATTR ISR_cua() {
  btnDebounce2 = xTaskGetTickCountFromISR();
  xSemaphoreGiveFromISR(xSema_cua, NULL);
}

void IRAM_ATTR ISR_mode() {
  btnDebounce3 = xTaskGetTickCountFromISR();
  xSemaphoreGiveFromISR(xSema_mode, NULL);
}

Servo servo_mo_cua;
#define SOUND_SPEED 0.034

#define TFT_CS         15  //chip select connect to pin 15
#define TFT_RST        2   //reset connect to pin 2
#define TFT_A0         4   //AO/DC connect to pin 4  
#define TFT_SDA        13  //Data = SDA/MOSI connect to pin 33
#define TFT_SCK        14  //Clock = SCK connect to pin 14

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_A0, TFT_SDA, TFT_SCK, TFT_RST);

#define BLACK   0x0000    
#define BLUE    0x001F    
#define RED     0xF800      
#define GREEN   0x07E0 
#define CYAN    0x07FF
#define MAGENTA 0xF81F  
#define YELLOW  0xFFE0
#define WHITE   0xFFFF 

//==============Task Def=============
void TaskReadDHT(void *pvParam);
void TaskReadMQ2(void *pvParam);
void TaskReadSR04(void *pvParam);
void TaskTFT(void *pvParam);

void TaskCoiQuat(void *pvParameters);
void TaskMoCua(void *pvParameters);

void Task_coi_quat(void *pvParameters);
void Task_cua(void *pvParameters);
void Task_mode(void *pvParameters);
  
//============QUEUE==========
typedef struct {
  int deviceID = 0;
  float value1 = 0;
  float value2 = 0;
} SENSOR;

QueueHandle_t QueueSensor = xQueueCreate(3, sizeof(SENSOR));

QueueHandle_t QueueMoCua = xQueueCreate(1, sizeof(int));
QueueHandle_t QueueCoiQuat = xQueueCreate(1, sizeof(int));

TaskHandle_t DHTTask, MQ2Task, SR04Task, TFTTask, CoiQuatTask, MoCuaTask;

static const uint8_t temperature[] = {
  0x00, 0x00, 0x00, 0x00, 0x47, 0x00, 0x00, 0x4c, 0x80, 0x00, 0x48, 0xc0, 0x18, 0x0a, 0x70, 0x09, 
  0xfa, 0x70, 0x03, 0x1a, 0x40, 0x06, 0x0a, 0x70, 0x04, 0x0a, 0xc0, 0x0c, 0x0a, 0x70, 0xec, 0x0a, 
  0xc0, 0x04, 0x0a, 0x40, 0x04, 0x13, 0x60, 0x02, 0x17, 0xa0, 0x01, 0xf7, 0x20, 0x08, 0x5a, 0x60, 
  0x18, 0x0c, 0xc0, 0x00, 0x47, 0x80, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00
};

static const uint8_t humidity[] = {
  0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0xd0, 0x00, 0x01, 0x98, 0x00, 0x03, 
  0x0c, 0x00, 0x06, 0x06, 0x00, 0x04, 0x03, 0x00, 0x08, 0x01, 0x00, 0x18, 0x01, 0x80, 0x10, 0x00, 
  0x80, 0x10, 0x00, 0xc0, 0x14, 0x00, 0xc0, 0x14, 0x00, 0xc0, 0x16, 0x00, 0x80, 0x1b, 0x00, 0x80, 
  0x09, 0xf1, 0x00, 0x0c, 0xf3, 0x00, 0x07, 0x0e, 0x00, 0x01, 0xf8, 0x00
};

const unsigned char smoke[] ={
  0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x80,0x00,0x0C,0xDC,0x00,0x18,0x62,0x00,0x10,0x8D,0x00,
  0x10,0x82,0x80,0x38,0x02,0x80,0x40,0x00,0xC0,0x50,0x00,0xE0,0x20,0x01,0x20,0x50,0x00,0x20,
  0x40,0x04,0x20,0x64,0x44,0x40,0x34,0x39,0x80,0x0E,0x03,0x00,0x01,0x06,0x00,0x00,0xF8,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00
};

const unsigned char distance [] ={
  0x1C,0x00,0x00,0x2E,0x00,0x00,0x5F,0x00,0x00,0x95,0x80,0x00,0x82,0xC0,0x00,0xC1,0xE0,0x00,
  0x60,0x70,0x00,0x20,0xB8,0x00,0x08,0x1C,0x00,0x0C,0x16,0x00,0x06,0x0B,0x00,0x03,0x05,0x80,
  0x01,0x8B,0xC0,0x00,0xC0,0xE0,0x00,0x60,0x70,0x00,0x30,0x38,0x00,0x18,0x6C,0x00,0x0C,0x56,
  0x00,0x06,0x0B,0x00,0x03,0x07,0x00,0x01,0x81,0x00,0x00,0xC2,0x00,0x00,0x64,0x00,0x00,0x38
};

const unsigned char fan[] PROGMEM = {
  0x00,0x00,0x00,0x0F,0x80,0x00,0x08,0xC0,0x00,0x10,0x47,0x80,0x10,0x4C,0x60,0x10,0x58,0x20,
  0x18,0xD0,0x20,0x0C,0xE0,0x20,0x06,0x6C,0x60,0x03,0xFF,0xC0,0x3F,0xFC,0x00,0x63,0x66,0x00,
  0x40,0x73,0x00,0x40,0xF1,0x80,0x41,0xA0,0x80,0x63,0x20,0x80,0x1E,0x20,0x80,0x00,0x31,0x00,
  0x00,0x1F,0x00,0x00,0x00,0x00
};

const unsigned char door[] PROGMEM = {
  0x00,0x00,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x0C,0x03,0x00,0x0C,0x03,0x00,0x0C,0x03,0x00,
  0x0C,0x03,0x00,0x0C,0x03,0x00,0x0C,0x03,0x00,0x07,0x03,0x00,0x07,0x03,0x00,0x0C,0x03,0x00,
  0x0C,0x03,0x00,0x0C,0x03,0x00,0x0C,0x03,0x00,0x0C,0x03,0x00,0x04,0x02,0x00,0x1F,0xFF,0x80,
  0x1F,0xFF,0x80,0x00,0x00,0x00
};

void setup() {
  Serial.begin(115150);
  analogReadResolution(10);
  pinMode(coi_quat, OUTPUT);
  digitalWrite(coi_quat, LOW);   
  pinMode(cb_mq2, INPUT);
  pinMode(trig_sr04, OUTPUT);
  pinMode(echo_sr04, INPUT);
  servo_mo_cua.attach(mo_cua);
  servo_mo_cua.write(73);
  
  tft.initR(INITR_BLACKTAB);                      //Thiet lap LCD TFT
  tft.fillScreen(BLACK);
  tft.setRotation(0);

  tft.setCursor(20, 0);
  tft.setTextColor(YELLOW, BLACK);
  tft.setTextSize(1);
  tft.print("Embedded System");

  Serial.println("Wait a few seconds for the system to boot!");
  delay(3000);

  tft.drawBitmap(10,15,temperature,20,20,CYAN);  //Hien thi icon nhiet do va do am
  tft.drawBitmap(10,40,humidity,20,20,CYAN);
  tft.drawBitmap(10,65,smoke,20,20,CYAN);
  tft.drawBitmap(10,90,distance,20,20,CYAN);

  tft.drawLine(0, 112, 128, 112, WHITE);
  tft.drawLine(64, 115, 64, 160, WHITE);
  
  tft.drawBitmap(23, 114, fan, 20, 20, CYAN);
  tft.drawBitmap(86, 114, door, 20, 20, CYAN);

  xTaskCreatePinnedToCore(TaskReadDHT
                          , "ReadDHT"
                          , 1024*4
                          , NULL
                          , 3
                          , &DHTTask
                          , ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(TaskReadMQ2
                          , "ReadMQ2"
                          , 1024*4
                          , NULL
                          , 3
                          , &MQ2Task
                          , ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(TaskReadSR04
                          , "ReadSR04"
                          , 1024*4
                          , NULL
                          , 3
                          , &SR04Task
                          , ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(TaskTFT
                          , "TFT"
                          , 1024*2
                          , NULL
                          , 1
                          , &TFTTask
                          , ARDUINO_RUNNING_CORE);
                          
  xTaskCreatePinnedToCore(TaskCoiQuat
                          , "CoiQuat"
                          , 1024*2
                          , NULL
                          , 2
                          , &CoiQuatTask
                          , ARDUINO_RUNNING_CORE); 
                                                    
  xTaskCreatePinnedToCore(TaskMoCua
                          , "MoCua"
                          , 1024*2
                          , NULL
                          , 2
                          , &MoCuaTask
                          , ARDUINO_RUNNING_CORE);                         

  xSema_coi_quat = xSemaphoreCreateBinary(); 
  xSema_cua = xSemaphoreCreateBinary(); 
  xSema_mode = xSemaphoreCreateBinary();

  xTaskCreate(Task_coi_quat
              , "coi_quat"
			        , 1024 * 4
			        , NULL
			        , 4
			        , NULL);
  xTaskCreate(Task_cua
              , "cua"
			        , 1024 * 4
			        , NULL
			        , 4
			        , NULL);
  xTaskCreate(Task_mode
              , "mode"
	        		, 1024 * 4
	        		, NULL
			        , 4
			        , NULL);

  pinMode(button_coi_quat, INPUT_PULLUP);
  attachInterrupt(button_coi_quat, ISR_coi_quat, FALLING);
  
  pinMode(button_cua, INPUT_PULLUP);
  attachInterrupt(button_cua, ISR_cua, FALLING);

  pinMode(button_mode, INPUT_PULLUP);
  attachInterrupt(button_mode, ISR_mode, FALLING);
}

void loop() {}

void TaskReadDHT (void *pvParam){
  float oldtemp = 0;
  float oldhumi = 0;
  float humi, temp;

  SENSOR DHT22Sensor ;
  DHT22Sensor.deviceID = 1;
  
  DHT dht(cb_dht, DHT22);
  dht.begin();

  for (;;){
    temp = dht.readTemperature();
    humi = dht.readHumidity();

    if (isnan(temp) || isnan(humi)){
      Serial.println("DHT22 Failed to Read!!!");
      temp = oldtemp;
      humi = oldhumi;
    }
    else {
      oldtemp = temp;
      oldhumi = humi;
    }
    DHT22Sensor.value1 = temp;
    DHT22Sensor.value2 = humi;
    xQueueSend(QueueSensor, &DHT22Sensor, 50);    
    
    Serial.println("Temp: " + String(temp, 1) + "C");
    Serial.println("Humi: " + String(humi, 1) + "%");
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskReadMQ2 (void *pvParam){ 
  int smoke_val=0;
  
  SENSOR MQ2Sensor;
  MQ2Sensor.deviceID = 2;
  
  for(;;){
    smoke_val = analogRead(cb_mq2);
    
    MQ2Sensor.value1 = smoke_val;
    MQ2Sensor.value2 = 0;
    
    xQueueSend(QueueSensor, &MQ2Sensor, 50);    

    xQueueSend(QueueCoiQuat, &smoke_val, 50);
    
    Serial.println("Smoke Value: " + String(smoke_val));
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskReadSR04 (void *pvParam){
  int distanceCm = 0; 
  
  SENSOR SR04Sensor;
  SR04Sensor.deviceID = 3;

  for(;;){
    digitalWrite(trig_sr04, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_sr04, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_sr04, LOW);
    distanceCm = pulseIn(echo_sr04, HIGH) * SOUND_SPEED/2;

    SR04Sensor.value1 = distanceCm;
    SR04Sensor.value2 = 0;
    
    xQueueSend(QueueSensor, &SR04Sensor, 50);

    xQueueSend(QueueMoCua, &distanceCm, 50);
    
    Serial.println("Distance Value: " + String(distanceCm));
    Serial.println("==============================");
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskCoiQuat (void *pvParam){
  int smoke_val = 0;

  for(;;){
    xQueueReceive(QueueCoiQuat, &smoke_val, 50);
    if (smoke_val >= 300){
      digitalWrite(coi_quat, HIGH);
    } 
    else {
      digitalWrite(coi_quat, LOW);
    }
  }
}

void TaskMoCua (void *pvParam){
  int distanceCm = 0;
  
  for (;;){
    xQueueReceive(QueueMoCua, &distanceCm, 50);
    if (distanceCm <= 10){      
      servo_mo_cua.write(0); 
      vTaskDelay(2000 / portTICK_PERIOD_MS);  
      servo_mo_cua.write(73);   
    }
  }
}

void TaskTFT (void *pvParam){
  SENSOR TFT_DISP;
  
  for (;;){    
    xQueueReceive(QueueSensor, &TFT_DISP, 50); 
    if (TFT_DISP.deviceID == 1){
      tft.setCursor(40, 18);                 
      tft.setTextColor(GREEN,BLACK);
      tft.setTextSize(2);
      String nhiet_do = String(TFT_DISP.value1,1) + "C";
      tft.print(nhiet_do);
      
      tft.setCursor(40, 43);                   //Hien thi do am
      tft.setTextColor(GREEN,BLACK);
      tft.setTextSize(2);
      String do_am = String(TFT_DISP.value2,1) + "%";
      tft.print(do_am);
    }

    if (TFT_DISP.deviceID == 2){      
      tft.setCursor(40, 68);                 
      tft.setTextColor(GREEN,BLACK);
      tft.setTextSize(2);
      char convertSmoke[4];
      sprintf(convertSmoke, "%03d", int (TFT_DISP.value1));
      String smoke = String(convertSmoke) + "ppm";
      tft.print(smoke);

      if(int (TFT_DISP.value1) >= 300){
        tft.setCursor(24, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("ONN");
      }
      else{
        tft.setCursor(24, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("OFF");
      }
    }

    if (TFT_DISP.deviceID == 3){      
      tft.setCursor(40, 93);                   
      tft.setTextColor(GREEN,BLACK);
      tft.setTextSize(2);
      char convertCm[4];
      sprintf(convertCm, "%03d", int (TFT_DISP.value1));
      String distanceCm = String(convertCm) + "cm";
      tft.print(distanceCm);

      if(int (TFT_DISP.value1) <= 10){        
        tft.setCursor(80, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("OPENN");

        vTaskDelay(2000 / portTICK_PERIOD_MS);

        tft.setCursor(80, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("CLOSE");
      }
    }
  }
}

void Task_coi_quat(void *pvParam) {
  while (1) {
    xSemaphoreTake(xSema_coi_quat, portMAX_DELAY);
    vTaskSuspend(MQ2Task);
    vTaskSuspend(CoiQuatTask);
    vTaskSuspend(TFTTask);

    if ((xTaskGetTickCount() - btnDebounce1) < 100) {
      if(state == 0){
        digitalWrite(coi_quat, HIGH);
        state = 0;

        tft.setCursor(24, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("ONN");
      }
      else{
        digitalWrite(coi_quat, LOW);
        state = 1;

        tft.setCursor(24, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("OFF");
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void Task_cua(void *pvParam) {  
  while (1) {
    xSemaphoreTake(xSema_cua, portMAX_DELAY);
    vTaskSuspend(SR04Task);
    vTaskSuspend(MoCuaTask);
    vTaskSuspend(TFTTask);

    if ((xTaskGetTickCount() - btnDebounce2) < 100) {
      servo_mo_cua.write(0);

      tft.setCursor(80, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("OPENN");

      vTaskDelay(2000 / portTICK_PERIOD_MS);

      servo_mo_cua.write(73);

      tft.setCursor(80, 143);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(1);
        tft.print("CLOSE");
    }
  }
}

void Task_mode(void *pvParam) {
  while (1) {
    xSemaphoreTake(xSema_mode, portMAX_DELAY);
    if ((xTaskGetTickCount() - btnDebounce3) < 100) {
      vTaskResume(MQ2Task);
      vTaskResume(CoiQuatTask);
      vTaskResume(SR04Task);
      vTaskResume(MoCuaTask);
      vTaskResume(TFTTask);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

