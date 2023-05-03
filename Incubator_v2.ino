
#include <avr/eeprom.h>
#include "GyverTimers.h"
#include "GyverRelay.h"
#include "GyverEncoder.h"
#include "GyverTM1637.h"
#include "GyverWDT.h"
#include <microDS18B20.h>
#include <dht.h>

dht DHT;
MicroDS18B20<A0> sensor; // датчик DS18b20 на A0
MicroDS18B20<A3> sensor2; // датчик DS18b20 на A3

#define DHT22_PIN 2     // Сюда подключен DHT22
#define MAX_TEMP  29.5  // Максимально допустимая температура
#define TMAXSENSTEMP 3  //1 - Использовать DS 2- Использовать DHT 3- Использовать наибольшую температуру 4- Использовать наименьшую температуру

#define HEATER_PIN    7  // Реле нагревателя
#define HUMMIDITY_PIN 8  // Реле генератора тумана
 
#define LED_ON  digitalWrite(13, HIGH)
#define LED_OFF digitalWrite(13, LOW)

#define ON  LOW
#define OFF HIGH



//volatile unsigned char led_status=0,LedRefresh=1,DHT_read_data=1, Stage=0, Mode=0, StartCoocking=0,cClock=0, AnalogVal=0;
//volatile unsigned char EncoderValue=0, EditSettings=0, Row=0, Col=0;
volatile unsigned char led_status=0,LedRefresh=1,DHT_read_data=1,LCDCounter=0, LCDShow=0;
long data = 0;
float DSTemp=0, tDHT=0, tDS=0, tDS2=0, tDS_Max=0;
int8_t bytes[4];      // буфер

GyverRelay regulator(REVERSE);
GyverRelay regulator_hummidity(REVERSE);
GyverTM1637 disp(A4, A5);

struct
{
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
} stat = { 0,0,0,0,0,0,0,0};

void setup() 
{

  regulator.setpoint = 29.3;    // установка (ставим на 29.3 градусов)
  regulator.hysteresis = 0.2;   // ширина гистерезиса
  regulator.k = 0.5;          // коэффициент обратной связи (подбирается по факту)

  regulator_hummidity.setpoint = 80;    // установка (ставим на 29.3 градусов)
  regulator_hummidity.hysteresis = 5;   // ширина гистерезиса
  regulator_hummidity.k = 0.5;          // коэффициент обратной связи (подбирается по факту)
  

  pinMode(13, OUTPUT);           // будем мигать

  pinMode(HEATER_PIN,    OUTPUT);
  digitalWrite(HEATER_PIN,    OFF);  // Выключить нагреватель

  pinMode(HUMMIDITY_PIN,    OUTPUT);
  digitalWrite(HUMMIDITY_PIN,    OFF);  // Выключить увлажнитель
//  dht.begin();

  Serial.begin(115200);
  Serial.println("Загрузка...");
  Timer1.setFrequency(4);               // Высокоточный таймер 1 для первого прерывания, частота - 4 Герца
  Timer1.enableISR();                   // Запускаем прерывание (по умолч. канал А)

  disp.clear();
  disp.brightness(7);  // яркость, 0 - 7 (минимум - максимум)
  disp.clear();

  Watchdog.enable(RESET_MODE, WDT_PRESCALER_128); // Режим сторжевого сброса , таймаут ~1с

}


void loop() 
{
  if(LedRefresh==1)   // Обновить экран
  {
    LedRefresh=0;
//    LedShow();
  }

  if(led_status==1) LED_ON;
  else              LED_OFF;

  if(DHT_read_data==1)
  {
    DHT_read_data=0;

    if(LCDCounter==0)   // DHT22 температура считывается 1 раз в 2 сек
    {
      DHT.read22(DHT22_PIN);
      Serial.print("DHT22 ");
      Serial.print(DHT.humidity, 1);
      Serial.print(",\t");
      Serial.print(DHT.temperature, 1);
      Serial.print(",\t");
      Serial.print(regulator.getResult());
      Serial.println("");
      tDHT=DHT.temperature;
//      regulator.input = DHT.temperature;
      regulator_hummidity.input = DHT.humidity;

      sensor.requestTemp(); // Отправить команду считывания температуры на DS18b20, считать которую можно через 920uSec
      sensor2.requestTemp(); // Отправить команду считывания температуры на DS18b20, считать которую можно через 920uSec
    }

    if(LCDCounter==4)   // DS18b20 температура считывается 1 раз в 2 сек
    {
      Serial.print("DS18b20 ");

      Serial.print(" DS1 ");
      if (sensor.readTemp())  // Если температура DS1 считана, загружаем значение в регулятор
      {
        tDS=sensor.getTemp();
//        DHT.temperature=tDS;
        Serial.print(tDS);
      }
      else
      {
              Serial.print("Error");
      }

      Serial.print(" DS2 ");
      if (sensor2.readTemp())  // Если температура DS2 считана, загружаем значение в регулятор
      {
        tDS2=sensor2.getTemp();
        Serial.print(tDS2);
      }
      else
      {
              Serial.print("Error");
      }
      Serial.println("");
      if(tDS>tDS2) tDS_Max=tDS;
      else tDS_Max=tDS2;

      DHT.temperature=tDS_Max;

    }

    if(LCDCounter==4 || LCDCounter==0)
    {
      if(tDHT<MAX_TEMP && tDS<MAX_TEMP && tDS2<MAX_TEMP)
      {
        if(TMAXSENSTEMP==1) regulator.input = max(tDS,tDS2);
        if(TMAXSENSTEMP==2) regulator.input = tDHT;
        if(TMAXSENSTEMP==3) // использовать наибольшую температуру
        {
          if( tDHT>max(tDS,tDS2) )    regulator.input = tDHT;
          else                        regulator.input = max(tDS,tDS2);
        }
        if(TMAXSENSTEMP==4) // использовать наименьшую температуру
        if( tDHT<min(tDS,tDS2) )      regulator.input = tDHT;
        else                          regulator.input = min(tDS,tDS2);
      }
      else
      {
        Serial.println("High temperature !!!");
        regulator.input =   regulator.setpoint+regulator.hysteresis;
      }
    }
      


    if(LCDCounter>=0 && LCDCounter<6) LCDShow=0;      // Показать температуру
    else                              LCDShow=1;      // Показать влажность

    if(LCDCounter==0 || LCDCounter==4 || LCDCounter==6)
    {
      if(LCDShow==0)  data=DHT.temperature*10;
      else            data=DHT.humidity;
      for (byte i = 0; i < 4; i++) 
      { //>
        bytes[i] = data % 10; // записываем остаток в буфер
        data /= 10;         // "сдвигаем" число
        if (data == 0) 
        {
          i=5;
        }
      } // массив bytes хранит цифры числа data в обратном порядке!
      if(LCDShow==1)  disp.clear();
      if(LCDShow==0)                  // Показать символ "градус"
      {
        disp.display(0, bytes[2]);
        disp.point(true, false);
        disp.display(1, bytes[1]);
        disp.point(false, false);
        disp.display(2, bytes[0]);
        disp.displayByte(3, 0x63);
      }
      else
      {
        disp.point(false);
        disp.display(1, bytes[1]);
        disp.display(2, bytes[0]);
      }
    }
    
  }

  digitalWrite(HEATER_PIN, regulator.getResult());
  digitalWrite(HUMMIDITY_PIN, regulator_hummidity.getResult());

  Watchdog.reset(); // Переодический сброс watchdog, означающий, что устройство не зависло

}




// Прерывание А таймера 1
ISR(TIMER1_A) 
{  // пишем  в сериал
//  Serial.println("timer1");
     if(led_status==0)
     {
        led_status=1;
     }
     else
     {
        led_status=0;
     }
     LedRefresh=1; // Обновить экран
     DHT_read_data=1;   // Перечитать термодатчик
     if(LCDCounter<8) LCDCounter=LCDCounter+1;
     else             LCDCounter=0;
}





void DHT_Read()
{
//  int AnalogVal=0;
  int TempDecimal=0, TempInt=0;

  if(DHT_read_data==1)
  {
//     CalckTemp();
     DHT_read_data=0;
   }
}
