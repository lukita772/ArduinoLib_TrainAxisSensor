/*
   brief: Sensor de ejes con deteccion de rueda por ultra sonido/laser
   conectividad: wi-fi, GSM/GPRS
   protocolo: tcp/ip mqtt
   rev : 3.04.18 (Agostini, Luca)
*/
// Select your modem: 
#define TINY_GSM_MODEM_SIM800
//#define TINY_GSM_MODEM_ESP8266

#include <TinyGsmClient.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <ESP8266HTTPClient.h>
#include "MLX90621.h"

/***********DISPOSITIVOS ACTIVOS*************************/
//#define USE_OLED
//#define USE_LASER
//#define USE_LASER_LONG_RANGE
//#define USE_US
/********************************************************/

#if defined USE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET LED_BUILTIN
Adafruit_SSD1306 display(OLED_RESET);
#endif

#if defined USE_LASER
#include <VL53L0X.h>
VL53L0X sensorLaser1;
VL53L0X sensorLaser2;
#endif

//#define VL53L0X_ADDR_2 42
#define XSHUT_PIN1 12
#define XSHUT_PIN2 14

//#define US_TRIGGER 14  //D2
//#define US_ECHO    12  //D1
//#define US_TPULSO_MAX 6380 //110 cm
#define US_DETECTION_MIN 5//cm <----- distancia donde detecto rueda!
#define US_DETECTION_MAX 18

//#define IR_RESOLUTION 16*4
//#define IR_SAMPLES 9
#define IR_WHEELS 4
#define IR_WAGONS 100

#define GSM_TX 15
#define GSM_RX 13
//#define IR_SCL 5VL53L0X_ADDR_2
//#define IR_SDA 4

#define TOTAL_MEMORY 4096
#define SERIAL_BAUD_RATE 57600//38400

#define EEPROM_IR_DATA_INDEX 10

#define THRESHOLD_TEMP 0
#define THRESHOLD_TEMP_AVG 0

#define DISABLE_RESET_PIN 2

#define CONNECTION_TIMEOUT 30000L

/*********Print IR in matrix************/
#define TEMP_MIN 10
#define TEMP_MAX 80
#define ROWS 4
#define COLS 16
/***************************************/

//permite editar rapidamente conociendo las coordenadas de la matriz IR
const uint8_t axisIRPosition[16] = {
  24, 25, 26, 27,
  28, 29, 30, 31,
  32, 33, 34, 35,
  36, 37, 38, 39
};

const uint8_t footIRPosition[24] = {
  0, 1, 2, 3, 4, 5,
  6, 7, 8, 9, 10, 11,
  52, 53, 54, 55, 56, 57,
  58, 59, 60, 61, 62, 63
};

enum wheelStatus {
  LEFT = 1,  //viaja en direccion a la izquierda (viendo el sensor de frente)
  RIGHT = 2, //viaja en direccion a la derecha
  ERR_LEFT = 3, //viaja en direccion a la izquierda / error de lectura de IR
  ERR_RIGHT = 4, //viaja en direccion a la derecha / error de lectura de IR
  NODIR = 5, //no se detecta direccion
  ERR = 6 //no se detecta direccion / error de lectura de IR
};

enum laserSensor {
  LASER_SENSOR1 = 1,
  LASER_SENSOR2 = 2
};

enum irSensor {
  IR_SENSOR1 = 1,
  IR_SENSOR2 = 2
};

typedef union _data2Bytes {
  uint16_t uint16Data;
  byte byteData[2];
};

struct wheelTempData {
  _data2Bytes wheel; //wheel nÂ°
  uint8_t tempAvgAxis;
  uint8_t tempAvgFoot;
  wheelStatus ts; // 1- axis, 2-foot, 3-axis & foot
};

enum state {
  ST_standBy,
  ST_sensingIR,
  ST_sleepMode,
  ST_connectingToNetwork,
  ST_sendingData,
  ST_resetEEPROM
};

enum connectionType
{
  _default, //wifi->gsm
  _wifi,    //force wifi
  _gprs     //force gsm/gprs
} currentConnectionType = _wifi;

//MQTT login
//const char* broker = "";
const char* broker = "";
const uint16_t brokerPort = ;
const char MQTT_clientID[] = "";
const char MQTT_user[] = "";
const char MQTT_passworld[] = "";

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "datos.personal.com";
const char user[] = "";
const char pass[] = "";

//WIFI login
const char* ssid = "";
const char* password = "";

//Topicos
const char* topicEjesIR = "23002TopicEjesIR";

state newState = ST_standBy;

wheelTempData wheelBuff[IR_WHEELS * IR_WAGONS];
MLX90621 sensorIR;
SoftwareSerial SerialAT(GSM_RX, GSM_TX); // RX, TX
TinyGsm modem(SerialAT);
WiFiClient WIFI_client;
TinyGsmClient GSM_client;
PubSubClient mqtt;

long currentTime = 0;
long time_to_sleep = 60000 * 20; //20 min
int32_t LastDir = 0; //direccion en la que camina la rueda

long lastReconnectAttempt = 0;
uint16_t wheelIndex = 0;

#if defined USE_OLED
void printThroughOLED(String data,  uint8_t textSize = 2) {
  uint8_t dataLength = data.length() + 1;
  char buff[dataLength];

  display.clearDisplay();
  display.setTextSize(textSize);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  data.toCharArray(buff, dataLength);

  for (uint8_t i = 0; i < dataLength; i++) {
    display.write(buff[i]);
  }
  display.println();
  display.display();
  delay(1);
}
#endif

bool initializeNetwork(connectionType currentConnectionType)
{
  if (currentConnectionType == _wifi  || currentConnectionType == _default)
  {
    mqtt.setClient(WIFI_client);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    Serial.println();

    if ( currentConnectionType == _wifi )
    {
      currentTime = millis();

      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");

        if (millis() > currentTime + CONNECTION_TIMEOUT) {
          return false;
        }
      }
      return true;
    }
    else if ( currentConnectionType == _default )
    {
      for (unsigned long start = millis(); millis() - start < CONNECTION_TIMEOUT; )
      {
        Serial.print(".");
        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.print("\n\rConnection success\n\r");
          return true;
        }
        delay(500);
      }
      Serial.print("\n\rFailed to connect through wifi, trying to connect with gsm..\n\r");
      return connectThroughGSM();

    }
  }
  else //gprs
  {
    return connectThroughGSM();
  }

}

bool connectThroughGSM()
{
  GSM_client.init(&modem);
  mqtt.setClient(GSM_client);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("\n\rInitializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" OK");

  //Serial.print("Connecting to ");
  //Serial.print(apn);
  //if (!modem.gprsConnect(apn, user, pass))
  //{
  //  Serial.println(" fail");
  //  return false;
  //}
  //Serial.println(" OK");
  return true;

}

void initializeMQTT()
{
  // MQTT Broker setup
  mqtt.setServer(broker, brokerPort);
  mqtt.setCallback(mqttCallback);
}

boolean mqttConnect()
{
  Serial.print("Connecting to ");
  Serial.print(broker);

  if (!mqtt.connect(MQTT_clientID, MQTT_user, MQTT_passworld))  //
  {
    Serial.println(" MQTT connection failed\n\r");
    return false;
  }
  Serial.println(" OK");

  return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len)
{
  String string;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();

  if (String(topic) == topicEjesIR)
  { /// select topic "output"
    //EEPROM_Write(1, string);
    //resultado = string.toInt();

    delay(1000);
  }
}


/*
   brief: funcion que implementa los sensores de distancia
   return: distancia medida por alguno de los sensores
   nota: el sensor utilizado se selecciona segun un define
   nota2: sensorNum solo usado para sensor laser
*/
int getDistance(uint8_t sensorNum = LASER_SENSOR1, uint8_t samplesQt = 1)
{
  int distance = 0;

#if defined USE_LASER
  switch (sensorNum)
  {
    case LASER_SENSOR1:
      distance = sensorLaser1.readRangeContinuousMillimeters();
      break;
    case LASER_SENSOR2:
      distance = sensorLaser2.readRangeContinuousMillimeters();
      break;
    default:
      distance = sensorLaser1.readRangeContinuousMillimeters();
      break;
  }
  distance = distance / 10;

#elif defined USE_US
  uint16_t duration = 0;

  samplesQt < 1 ? 1 : samplesQt;

  for ( uint8_t i = 0; i < samplesQt; i ++)
  {
    digitalWrite(US_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIGGER, HIGH);
    delayMicroseconds(20);
    digitalWrite(US_TRIGGER, LOW);
    duration = pulseIn(US_ECHO, HIGH, US_TPULSO_MAX); //retorna la long. de la seÃ±al de echo, en uS
    distance += (duration / 2) / 29.1;
  }

  distance = distance / samplesQt;
#else
#warning "Sensor de distancia no definido"
#endif

#if defined USE_OLED
  char str[30];
  sprintf(str, "Distance: %d cm.", distance);
  printThroughOLED(str);
#endif

  return distance;
}

void setup()
{

  I2C_initialize();
  Serial.begin (SERIAL_BAUD_RATE);
  EEPROM.begin(TOTAL_MEMORY);

  pinMode(DISABLE_RESET_PIN, OUTPUT);
  delay(5);
  digitalWrite(DISABLE_RESET_PIN, LOW);
  
  currentTime = millis();
  Serial.print("\n\rstand by\n\r");
}

void I2C_initialize()
{
  pinMode(XSHUT_PIN1, OUTPUT);
  pinMode(XSHUT_PIN2, OUTPUT);

  Wire.begin();
  pinMode(XSHUT_PIN1, INPUT);
  delay(10);
  sensorLaser1.setAddress(45);
  pinMode(XSHUT_PIN2, INPUT);
  delay(10);
  sensorLaser2.setAddress(46);
  delay(10);

  sensorLaser1.init();
  sensorLaser1.setTimeout(100);
  sensorLaser1.startContinuous();
  delay(10);

  sensorLaser2.init();
  sensorLaser2.setTimeout(100);
  sensorLaser2.startContinuous();
  delay(10);

  //TODO: IMP. SWITCH DE SENSORES INFRAROJOS
  sensorIR.initialise(128);
}

/*
   brief: etapa de sensado por infra rojo para detencion de ejes y superficie de ruedas
   calientes.
*/
void sensingIR(uint8_t irSensorNum = IR_SENSOR2)
{
  static uint16_t currentWheel = 0;
  int16_t maxTemp = sensorIR.getMaxTemp();
  int16_t minTemp = sensorIR.getMinTemp();

  if (currentWheel == 0) { // si detecto una rueda. cambio el temporizador a 1min
    time_to_sleep = 60000 * 1;
  }

  //es coherente
  if ( maxTemp < TEMP_MAX && (maxTemp - minTemp) < 60 )
  {

    if (maxTemp > THRESHOLD_TEMP) //es imporante
    {
      int32_t tempAvgAxis = 0; //debe ser 32bits ya que antes de sacar el prom. suma todas las temp!
      int32_t tempAvgFoot = 0;
      //uint8_t footScreenLength = sizeof(footIRPosition) / sizeof(footIRPosition[0]);
      //uint8_t axisScreenLength = sizeof(axisIRPosition) / sizeof(axisIRPosition[0]);

      if ( irSensorNum == IR_SENSOR1 )
      {
        for (uint8_t i = 0; i < 64; i++)
        {
          tempAvgAxis += sensorIR.getTemperature(i);
        }
        tempAvgAxis = tempAvgAxis / 64;
      }

      if ( irSensorNum == IR_SENSOR2 )
      {
        for (uint8_t i = 0; i < 64; i++)
        {
          tempAvgFoot += sensorIR.getTemperature(i);
        }
        tempAvgFoot = tempAvgFoot / 64;
      }


      if (tempAvgFoot > THRESHOLD_TEMP_AVG || tempAvgAxis > THRESHOLD_TEMP_AVG)
      {
        wheelBuff[wheelIndex].wheel.uint16Data = currentWheel;

        if ( irSensorNum == IR_SENSOR1)
        {
          wheelBuff[wheelIndex].tempAvgAxis = tempAvgAxis;

        }

        if ( irSensorNum == IR_SENSOR2 )
        {
          wheelBuff[wheelIndex].tempAvgFoot = tempAvgFoot;
        }

        if (LastDir > 0)
        {
          wheelBuff[wheelIndex].ts = RIGHT;
        }
        else if (LastDir < 0)
        {
          wheelBuff[wheelIndex].ts = LEFT;
        }
        else
        {
          wheelBuff[wheelIndex].ts = NODIR;
        }


        printTemperature(currentWheel);
        //printSensingResult(currentWheel);
        wheelIndex++;
      }
    }
    else
    {
      printTemperature(currentWheel);
    }
  }
  else
  {
    Serial.print("Wheel not detected.\n\r");
    wheelBuff[wheelIndex].wheel.uint16Data = currentWheel;
    wheelBuff[wheelIndex].ts = ERR;
    wheelIndex++;
  }
  //printIRMatrix();
  currentWheel++;

  Serial.print("current wheel: ");
  Serial.print(currentWheel);
  Serial.print(" wheel index: ");
  Serial.print(wheelIndex);
  Serial.print("\n");
  newState = ST_standBy;
  LastDir = 0;
  //delay(200); //debo darle tiempo a la proxima rueda (sino corro el riesgo de sensar la misma rueda 2 veces)
  Serial.print("\n\rstand by..\n\r");
  currentTime = millis();
  return;
}

void loop()
{
  switch (newState)
  {
    case ST_standBy:
      standBy();
      break;
    case ST_sensingIR: //x vagon
      sensingIR();
      break;
    case ST_sleepMode:
      sleepMode();
      break;
    case ST_connectingToNetwork:
      if (initializeNetwork(currentConnectionType)) {
        Serial.println(getPublicIP());
        
        initializeMQTT();
        currentTime = millis();
        newState = ST_sendingData;
      }
      else {
        //todo: guardar en eeprom un log de error (tipo numerico)
        currentTime = millis();
        newState = ST_sleepMode;
      }
      break;
    case ST_sendingData:
      //if (millis() >= 5000 == 0) {
      //  newState = ST_sleepMode;
      //}
      sendingData();
      break;
    default:
      if (millis() % 1000 == 0) {
        Serial.print("Unknown state\n\r");
      }
      break;
  }
}

/*
   brief: estado en el cual se espera la detencion de rueda para posterior
*/
void standBy()
{
  static long timeOut1 = 0;
  static long timeOut2 = 0;
  static uint8_t counter = 5;
  //int8_t dir = 0;

  int distanceS1 = getDistance(LASER_SENSOR1);
  //delay(10);
  int distanceS2 = getDistance(LASER_SENSOR2);
  //delay(10);
  static uint8_t measured = false;

  if (millis() > timeOut1 + 300) //cada 0.3ms
  {
    timeOut1 = millis();
    Serial.print("D: ");
    Serial.print(distanceS1);
    Serial.print(",");
    Serial.print(distanceS2);
    Serial.print("\n\r");
  }

  if (counter > 0 )
  {
    if (distanceS1 > US_DETECTION_MIN && distanceS1 < US_DETECTION_MAX)
    {
      LastDir += 1;
      counter --;
    }
    if (distanceS2 > US_DETECTION_MIN && distanceS2 < US_DETECTION_MAX)
    {
      LastDir -= 1;
      counter --;
    }
  }

  if (millis() > currentTime + time_to_sleep) {
    newState = ST_connectingToNetwork;
    //saveTrainData();
    Serial.print("\n\rConnecting to network\n\r");
    currentTime = millis();
  }
  else if ((distanceS1 > US_DETECTION_MIN && distanceS1 < US_DETECTION_MAX) && (distanceS2 > US_DETECTION_MIN && distanceS2 < US_DETECTION_MAX))
  {
    if (!measured)
    {
      //TODO: IMP. SWITCH DE SENSORES INFRAROJOS
      newState = ST_sensingIR;

      Serial.print("\n\r----------------------------------------------------\n\rsensing IR..\n\r");
      currentTime = millis();
      measured = true;
    }

  }
  else
  {
    if (millis() > timeOut2 + 10) //cada 10ms refresco el vector de temps. del ir pero espero 10ms (por el i2c x las dudas)
    {
      delay(10);
      timeOut2 = millis();
      sensorIR.measure(true);
    }

    measured = false;
  }


}

void sendingData()
{
  if ( millis() % 10000 == 0) // entrar cada 10seg
  {
    if (mqtt.connected())
    {
      char messagePackage[10 * wheelIndex];

      Serial.print("Connected to MQTT\n\r");
      //sensors.requestTemperatures(); // Send the command to get temperatures
      Serial.print("MQTT loop response:");
      Serial.println(mqtt.loop());
      Serial.print("\n\r");
      Serial.print("MQTT state:");
      Serial.print(mqtt.state());
      Serial.print("\n\r");
      Serial.print("wheel index:");
      Serial.print(wheelIndex);
      Serial.print("\n\r");

      if ( wheelIndex > 0 )
      {
        for (uint16_t i = 0; i < wheelIndex; i++)
        {
          char buffAux[10]; //3(nÂºdrueda)+1("/")+3(temp)+1("/")+3(temp)+1("/")+1(estado)

          sprintf(buffAux, "%d/0/%d/%d-",
                  wheelBuff[i].wheel.uint16Data,
                  //wheelBuff[i].tempAvgAxis,
                  wheelBuff[i].tempAvgFoot,
                  wheelBuff[i].ts);
          if (i == 0)
            strcpy(messagePackage, buffAux);
          else
            strcat(messagePackage, buffAux);
        }
        Serial.print("Trama final: ");
        Serial.print(messagePackage);
        Serial.print("\n\r");

        mqtt.publish(topicEjesIR, messagePackage);

        mqtt.subscribe("sleeptime", 1);

        Serial.print("powering off the GSM module in 5 seconds...\n\r");
        delay(1000);
        Serial.print("powering off GSM..\n\r");
        modem.poweroff();
        Serial.print("Turning ESP to sleep mode\n\r");
        delay(5000);

      }
      else
      {
        mqtt.publish(topicEjesIR, "No wheel detected");
        Serial.print("No wheel detected\n\r");
        delay(2000);
      }

      ESP.deepSleep(1e6); //1seg

      mqtt.loop(); //?

    }
    else
    {
      // Reconnect every 10 seconds
      unsigned long t = millis();
      if (t - lastReconnectAttempt > 10000L)
      {
        lastReconnectAttempt = t;
        if (mqttConnect()) {
          lastReconnectAttempt = 0;
        }
      }
    }
  }
}


/*
   brief: guardo la informacion  de memoria estatica (el struct) a eeprom
   return : true : guardo todos los datos almacenados correspondientes a un vagon, false: hubo errores
*/
/*
  bool saveTrainData()
  {
  uint16_t indexEEPROM = EEPROM_IR_DATA_INDEX;

  for ( uint16_t index = 0; index < wheelIndex; index ++ )
  {
    EEPROM.write(indexEEPROM , wheelBuff[index].wheel.byteData[0]);
    indexEEPROM++;
    EEPROM.write(indexEEPROM , wheelBuff[index].wheel.byteData[1]);
    indexEEPROM++;
    EEPROM.write(indexEEPROM , wheelBuff[index].tempAvgAxis);
    indexEEPROM++;
    EEPROM.write(indexEEPROM , wheelBuff[index].tempAvgFoot);
    indexEEPROM++;
    EEPROM.write(indexEEPROM , (uint8_t)wheelBuff[index].ts);
    indexEEPROM++;

  }

  EEPROM.write(EEPROM_IR_DATA_INDEX - 1 , wheelIndex ); // en el primer registro (previo a donde comienzan los datos de IR, almaceno la cant de ruedas

  if (wheelIndex == 0)
  {
    Serial.print("Couldnt find data\n");
  }
  return EEPROM.commit();
  }
*/

void sleepMode()
{
  Serial.print("\n\rGoing to sleep..");
  ESP.deepSleep(1e6); //1seg
  //ESP.deepSleep(60*1e6); //1hora
}

/*
   En construccion

  void loadWagonData()
  {
  uint8_t dataLength = 0;
  uint8_t buff = 0;

  dataLength = (EEPROM.read(EEPROM_IR_DATA_INDEX - 1)) * 3; //3 registros por rueda

  if (dataLength > 0 && dataLength < 255)
  {
    for (uint16_t addr = EEPROM_IR_DATA_INDEX; addr < EEPROM_IR_DATA_INDEX + dataLength; addr++)
    {
      buff = EEPROM.read(addr);
      addr++;
      Serial.print(buff);
      Serial.print("\n");
    }
  }
  else
  {
    Serial.print("No data available in EEPROM\n");
  }
  return;
  }


  /*
   brief: Llena la memoria eeprom de unos (valor por defecto)

  void EEPROM_Reset()
  {
  Serial.print("Memory reset in progress..\n");
  for (uint16_t addr = 0; addr < TOTAL_MEMORY - 1; addr++)
  {
    EEPROM.write(addr, 0xFF);
    Serial.print("Memory register:");
    Serial.print(addr);
    Serial.print("\n");

    if (EEPROM.commit()) {
      Serial.print("State: OK\n");
    }
    else {
      Serial.print("State: ERROR\n");
    }
    delay(10);
  }

  newState = ST_sleepMode;
  currentTime = millis();
  }
*/

void printTemperature(uint16_t currentWheel)
{
  Serial.print("\n\rT: ");
  Serial.print(wheelBuff[wheelIndex].tempAvgFoot);
  Serial.print(" W.N: ");
  Serial.println(currentWheel);
  Serial.print("Dir: ");
  Serial.println(LastDir);

}

/*
  void printSensingResult(uint16_t currentWheel)
  {
  Serial.print("\n\r\n\rDelay time: ");
  int sensingLapse = millis() - currentTime;
  Serial.print(sensingLapse);
  Serial.print(" ms");

  Serial.print("\n\rTemp avg axis:");
  Serial.print(wheelBuff[wheelIndex].tempAvgAxis);
  Serial.print(" C\n\rTemp avg foot:");
  Serial.print(wheelBuff[wheelIndex].tempAvgFoot);
  Serial.print(" C\n\rWheel number:");
  Serial.print(currentWheel);
  Serial.print("\n\r");

  if (wheelBuff[wheelIndex].ts == BOTH)
  {
    Serial.print("Abnormal temperature ts: All\n\r");
  }
  else if (wheelBuff[wheelIndex].ts == FOOT)
  {
    Serial.print("Abnormal temp. ts: Foot\n\r");
  }
  else if (wheelBuff[wheelIndex].ts == AXIS)
  {
    Serial.print("Abnormal temp. ts: Axis\n");
  }
  }
*/

/*
   brief: funcion que permite dialogar con una hyperterminal true color.
   nota: el envio demora alrededor de 103 ms. (a 115200 bauds)

void printIRMatrix()
{

  for (uint8_t y = 0; y < ROWS; y++)
  {
    char strLast[500];
    //go through all the rows
    strcpy(strLast, "\r\n");

    for (uint8_t x = 0; x < COLS; x++)
    {
      char str[30];
      int16_t currentTemp = sensorIR.getTemperature(y + x * ROWS); //de -127 a 128
      uint8_t color = map(currentTemp, TEMP_MIN, TEMP_MAX, 0, 255);
      sprintf(str, "\x1b[48;2;%d;%d;%dm%d ", color, 20, 255 - color, currentTemp);  //R,G,B,temp
      strcat(strLast, str);
    }

    Serial.print(strLast);
  }
  char aux2[30];
  sprintf(aux2, "\x1b[48;2;%d;%d;%dm .", 0, 0, 0);  //R,G,B,temp
  Serial.print(aux2);
  //char aux[20];
  //sprintf(aux, "\x1b[48;2;%d;%d;%dm", 0, 0, 0);  //R,G,B,temp
  //strcat(strLast, aux);
}
*/

String getPublicIP()
{
  String payload;

  HTTPClient http;

  http.begin("http://api.ipify.org");
  int httpCode = http.GET();

  if (httpCode > 0) {
    payload = http.getString();
    Serial.println("Public IP:");
    Serial.println(payload);
  }
  else {
    Serial.print("Connection to http failed\n");
  }

  http.end();

  return payload;
}
