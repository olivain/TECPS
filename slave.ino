
// ICOC fev 2022 Oporry

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


// EACH SLAVE MUST HAVE ITS OWN ADDRESS
// CHANGE ONLY THE LAST BYTE ACCORDING TO MASTER.INO
uint8_t mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};


int dc_pin = 12;
int servopin = 27;
int builtinLed = 13;

int DC_SPEED = 0;
int SERVO_POS = 0;
int old_dc= DC_SPEED;
int old_servo= SERVO_POS;


// setting PWM properties
const int dcFreq = 5000;
const int dcChannel = 2;
const int dcResolution = 8;
const int servoFreq = 50;
const int servoChannel = 1;
const int servoResolution = 16;

struct __attribute__((packed)) DataStruct {
  char text[32];
  unsigned int time;
};
DataStruct myData;


void initVariant(){
  WiFi.mode(WIFI_AP);
  esp_wifi_set_mac(ESP_IF_WIFI_AP, &mac[0]);
}

uint32_t deg2duty(int pos){
  // convert 0-180 degrees to 0-65536
  uint32_t duty = (((pos/180.0)*2000)/20000.0*65536.0) + 1634;
  return duty;
}


void receiveCallBackFunction(const uint8_t *senderMac, const uint8_t *incomingData, int len){

  char *ptr;
  char *i;
  memcpy(&myData, incomingData, sizeof(myData));

  Serial.print(myData.text);

  ptr = strtok_r(myData.text,":",&i);

  if(strcmp("dc",ptr) == 0) {
    char *speed_nb = strtok_r(NULL, ":", &i);
    DC_SPEED = atoi(speed_nb); // 0-65536
  } else if(strcmp("sv",ptr) == 0) {
    char *pos_nb = strtok_r(NULL, ":", &i);
    SERVO_POS = atoi(pos_nb); // 0-180
  }
}

void setup() {

  pinMode(dc_pin,OUTPUT);
  ledcSetup(dcChannel, dcFreq, dcResolution);
  ledcAttachPin(dc_pin, dcChannel);

  pinMode(servopin,OUTPUT);
  ledcSetup(servoChannel,servoFreq,servoResolution);
  ledcAttachPin(servopin,servoChannel);

  Serial.begin(115200);
  pinMode(builtinLed,OUTPUT);


  while(esp_now_init() != ESP_OK){
    digitalWrite(builtinLed,HIGH);
    delay(500);
    digitalWrite(builtinLed,LOW);
    Serial.println("waiting for master...");
  }

  esp_now_register_recv_cb(receiveCallBackFunction);

  ledcWrite(servoChannel, deg2duty(180));
  delay(2000);
  ledcWrite(servoChannel, deg2duty(0));
}


void loop(){

  if(old_dc != DC_SPEED){
    ledcWrite(dcChannel, DC_SPEED);
    old_dc= DC_SPEED;
  }

  if(old_servo != SERVO_POS){
    ledcWrite(servoChannel, deg2duty(SERVO_POS));
    old_servo= SERVO_POS;
  }
}
