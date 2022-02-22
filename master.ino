

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define INPUT_SIZE 32

#define NB_SLAVES 6
uint8_t remoteMac[][6] = {
  {0x36, 0x33, 0x33, 0x33, 0x33, 0x34},
  {0x36, 0x33, 0x33, 0x33, 0x33, 0x35},
  {0x36, 0x33, 0x33, 0x33, 0x33, 0x36},
  {0x36, 0x33, 0x33, 0x33, 0x33, 0x37},
  {0x36, 0x33, 0x33, 0x33, 0x33, 0x38},
  {0x36, 0x33, 0x33, 0x33, 0x33, 0x40}
};
esp_now_peer_info_t slave[NB_SLAVES];

struct __attribute__((packed)) DataStruct {
  char text[32];
  unsigned int time;
};

int curmode= 0;

DataStruct myData;
String serial_input;

int old_st = 0;
long servo_time = 0;
long dc_time = 1000;
long delay_between_robots = 250*4; // 3secs
long rnd_time_of_seq = 0;

char *cmd[5] = {"sv:180","sv:90","sv:45","sv:135","sv:0"}; // Positions du gouvernail
int idx = 0;
//int t = 34;
bool seq_state = false;
bool dc_state = true;

long millis_timer = 0;


void sendCallBackFunction(const uint8_t* destination_mac_addr, esp_now_send_status_t uploadStatus) {
  Serial.println(uploadStatus == ESP_NOW_SEND_SUCCESS ? " Success" : " Fail");
}

void sendData(const uint8_t *dest_mac) {
  myData.time = millis();
  uint8_t bs[sizeof(myData)];
  memcpy(bs, &myData, sizeof(myData));
  esp_now_send(dest_mac, bs, sizeof(bs));
  memset(&myData, 0, sizeof(myData));
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  Serial.setTimeout(10);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  while(esp_now_init()!=ESP_OK) {
    Serial.println("*** ESP_Now init failed");
    delay(500);
  }

  for(int y = 0; y < NB_SLAVES; y++) {
    for (int ii = 0; ii < 6; ++ii ) {
      slave[y].peer_addr[ii] = (uint8_t) remoteMac[y][ii];
    }
    esp_now_add_peer(&slave[y]);
    Serial.println(y);
  }
  esp_now_register_send_cb(sendCallBackFunction);

  strcpy(myData.text, "dc:0");
  sendData((uint8_t*)NULL);
  delay(1000);
  strcpy(myData.text, "sv:180");
  sendData((uint8_t*)NULL);
  delay(1000);
  strcpy(myData.text, "sv:0");
  sendData((uint8_t*)NULL);
  delay(1000);

  servo_time = 500; //random(500,2000);

  dc_time = 800; //random(1000,2500);

  rnd_time_of_seq = random(2,5)*(60*1000); // entre 2 et 5 minutes par sequences

  Serial.println("");
  Serial.println("");
  Serial.println("TECPS ONLINE :: ");
  Serial.println("send requests thru serial using [cmd/mqtt_topic]");
  Serial.println("example : sv:180/robot4 -> send instruction sv:180 to robot number 4");
  Serial.println("original use case :");
  Serial.println(" JS on server send mqtt msg-> python receive mqtt msg & transmit thru serial -> esp32 controler receive from serial and send thru ESPNOW to robot");
}




void aleatoire() {
    Serial.println("aleatoire");

  if(idx < 5) { // 5 = Nombre de commandes Ã  envoyer aux robots
      String desiredString = "sv:180";
    for(int idxx = 0; idxx < NB_SLAVES; ++idxx) { // Un robot Ã  la fois, les uns aprÃ¨s les autres
      
      strcpy(myData.text, cmd[random(0,5)]); // On dÃ©finit la commande de position du gouvernail Ã  envoyer (numÃ©ro idx du tableau cmd[])
      sendData(remoteMac[idxx]); // On envoie cette commande au robot numÃ©ro idxx
      delay(servo_time); // on attend une seconde (temps maximal pour l'execution de la commande)

      strcpy(myData.text, "dc:0"); // On dÃ©finit une commande de dÃ©sactivation du moteur Ã  vibrations
      sendData(remoteMac[idxx]); // On envoie cette commande au robot numÃ©ro idxx
      delay(random(50,500)); // On attend une demi-seconde  (temps maximal pour l'execution de la commande)
    }
    idx += 1; // On passe Ã  la commande suivante

  } else {
    idx = 0; // On recommence Ã  zero
  }

}



void ensemble() {
    Serial.println("ensemble");

  if(idx < 5) { // 5 = Nombre de commandes Ã  envoyer aux robots
    strcpy(myData.text, cmd[idx]); // On sÃ©lectionne la position numÃ©ro idx du tableau cmd[] (voir plus haut)
    sendData((uint8_t*)NULL); // On envoie la commande Ã  tous les robots Ã  la fois
    delay(servo_time); // On attend une seconde (1000 millisecondes)

    if(dc_state == true) {
      Serial.print("dc on ?");
      strcpy(myData.text, "dc:1"); // On sÃ©lectionne l'activation du moteur Ã  vibrations
      sendData((uint8_t*)NULL); // On envoie la commande Ã  tous les robots Ã  la fois
      delay(dc_time); // On attend une seconde
    }

    strcpy(myData.text, "dc:0"); // On sÃ©lectionne l'activation du moteur Ã  vibrations
    sendData((uint8_t*)NULL); // On envoie la commande Ã  tous les robots Ã  la fois
    delay(3000); // On attend

    idx += 1; // on passe Ã  la valeur suivante du tableau cmd[]
  } else {
    idx = 0; // on a Ã©tÃ© jusqu'au bout du tableau cmd[], on recommence au dÃ©but
  }
}



void sequence() {
  Serial.println("sequence");

  if(idx < 5) { // 5 = Nombre de commandes Ã  envoyer aux robots

    for(int idxx = 0; idxx < NB_SLAVES; ++idxx) { // Un robot Ã  la fois, les uns aprÃ¨s les autres

      strcpy(myData.text, cmd[idx]); // On dÃ©finit la commande de position du gouvernail Ã  envoyer (numÃ©ro idx du tableau cmd[])
      sendData(remoteMac[idxx]); // On envoie cette commande au robot numÃ©ro idxx
      delay(servo_time); // on attend une seconde (temps maximal pour l'execution de la commande)

      if(dc_state == true) {
        strcpy(myData.text, "dc:1"); // On definit une commande d'activation du moteur Ã  vibrations
        sendData(remoteMac[idxx]); // On envoie cette commande au robot numÃ©ro idxx
        delay(dc_time); // On attend une seconde (temps maximal de mouvement dÃ©fini arbitrairement)
      }

      strcpy(myData.text, "dc:0"); // On dÃ©finit une commande de dÃ©sactivation du moteur Ã  vibrations
      sendData(remoteMac[idxx]); // On envoie cette commande au robot numÃ©ro idxx
      delay(delay_between_robots); // On attend une demi-seconde  (temps maximal pour l'execution de la commande)
    }
    idx += 1; // On passe Ã  la commande suivante

  } else {
    idx = 0; // On recommence Ã  zero
  }

}


void receiveCmdFromSerial() {
  // read data from serial
  // receive "sv:180/robot1"
  // extract robot id from receive string
  char input[INPUT_SIZE + 1];
  byte size = Serial.readBytes(input, INPUT_SIZE);
  if(size < 10) {
    if(size >=2){
      
    Serial.println("less than 10 !!!");
    Serial.println(input);

        if(strcmp("sq\n",input) == 0) {
          Serial.print("sequence()");
          curmode=1;
          sequence();
        } else if(strcmp("sy",input) == 0) {
          Serial.print("synchro()");
          curmode=2;
          ensemble();
        } else if (strcmp("rd\n",input) == 0) {
          Serial.print("random()");
          curmode=3;
          aleatoire();
        } else if (strcmp("st\n",input) == 0) {
          Serial.print("stop()");
          curmode=0;
        }
    } else {
      
  if(curmode==1){
  sequence();
  } else if(curmode==2){
  ensemble();
  } else if(curmode==3){
    aleatoire();
  }
    }
    return;
  }
  // Add the final 0 to end the C string
  input[size] = 0;
  char *p = input;
  char *cmd;
  char *robot;
  if((cmd = strtok_r(p, "/", &p)) != NULL){
    Serial.println(cmd);
    robot = strtok_r(p, "/", &p);
    int len = sizeof(robot)+1;
    char v = robot[len];
    int nb_addr = atoi(&v)-1;
    Serial.println(nb_addr);
    //  int nb_addr = atoi(robot[strlen(robot)]);
    strcpy(myData.text, cmd); // On definit la commande de position du gouvernail a envoyer (numero idx du tableau cmd[])
    sendData(remoteMac[nb_addr]);
  }
}

void loop(){
 sequence();
 //aleatoire();
 //ensemble();
//receiveCmdFromSerial();
}
