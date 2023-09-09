#include <EEPROM.h>
#include "GyverTimers.h"
#include "GyverRelay.h"
#include "GyverEncoder.h"
#include "GyverTM1637.h"
#include "GyverWDT.h"
#include <microDS18B20.h>
#include <dht.h>

#define ENC_CLK 4
#define ENC_DT 3
#define ENC_SW 5
Encoder enc1(ENC_CLK, ENC_DT, ENC_SW);

dht DHT;
MicroDS18B20<A0> sensor; // датчик DS18b20 на A0
MicroDS18B20<A3> sensor2; // датчик DS18b20 на A3

#define DHT22_DATA_PIN A2     // Сюда подключен DHT22
#define DHT22_PWR_PIN  2     // Сюда подключен DHT22

#define HEATER_PIN    7  // Реле нагревателя
#define HUMMIDITY_PIN 8  // Реле генератора тумана
 
#define LED_ON  digitalWrite(13, HIGH)
#define LED_OFF digitalWrite(13, LOW)

#define ON  LOW
#define OFF HIGH



volatile unsigned char led_status=0,LedRefresh=1,DHT_read_data=1,LCDCounter=0, LCDShow=0, Settings_mode=0, ShowDisplayAttr=0, Timer_Seconds=0, DHT_Reset_Mode=0, Encoder_Turn=0, Encoder_Status=0;
long data = 0, Timer_Min=0;
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

struct Data {
  float MAX_TEMP = 29.5;          // Максимально допустимая температура
  float MAX_HEATER_TEMP= 2.5;     // Максимальная температура нагревателя
  float INCUBATOR_TEMP = 29.0;    // Температура инкубации
  float HYSTERESIS= 0.1;          // Ширина гистерезиса
  byte SENSTEMP_MODE = 3;         // Режим работы //1 - Использовать DS1 2- Использовать DS2 3- Использовать наибольшую температуру 4- Использовать наименьшую температуру
  byte LCD_BRIGHTNESS=3;          // Яркость дисплея
  float DHT_RST=60;               // Время сброса питания DHT
};

Data Preset_Data;


void setup() 
{
  pinMode(DHT22_PWR_PIN,    OUTPUT);
  digitalWrite(DHT22_PWR_PIN, HIGH);

  sensor.requestTemp(); // Отправить команду считывания температуры на DS18b20, считать которую можно через 920uSec
  sensor2.requestTemp(); // Отправить команду считывания температуры на DS18b20, считать которую можно через 920uSec
  DHT.read22(DHT22_DATA_PIN);

  disp.clear();
  disp.brightness(Preset_Data.LCD_BRIGHTNESS);  // яркость, 0 - 7 (минимум - максимум)
//  disp.displayByte(_i, _n, _c, _u);
  disp.displayByte(_dash, _dash, _dash, _dash);
//  disp.clear();

  Serial.begin(115200);
  Serial.println("Загрузка...");
  EEPROM.get(0, Preset_Data);   // прочитать из адреса 0
  Serial.print(Preset_Data.MAX_TEMP);
  if(Preset_Data.MAX_TEMP>33 || Preset_Data.MAX_TEMP < 25 || isnan(Preset_Data.MAX_TEMP) || isnan(Preset_Data.HYSTERESIS) || isnan(Preset_Data.DHT_RST))
  {
    Preset_Data.MAX_TEMP=29.5;
    Preset_Data.MAX_HEATER_TEMP= 2.5;
    Preset_Data.INCUBATOR_TEMP = 29.0;
    Preset_Data.HYSTERESIS = 0.1;
    Preset_Data.SENSTEMP_MODE = 3;
    Preset_Data.LCD_BRIGHTNESS=3;
    Preset_Data.DHT_RST=60;
    Save_eeprom();
  }

  regulator.setpoint = Preset_Data.INCUBATOR_TEMP;    // установка температуры инкубации
  regulator.hysteresis = Preset_Data.HYSTERESIS;   // ширина гистерезиса
  regulator.k = 0.5;          // коэффициент обратной связи (подбирается по факту)

  regulator_hummidity.setpoint = 80;    // установка (ставим на 29.3 градусов)
  regulator_hummidity.hysteresis = 5;   // ширина гистерезиса
  regulator_hummidity.k = 0.5;          // коэффициент обратной связи (подбирается по факту)
  

  pinMode(13, OUTPUT);           // будем мигать

  pinMode(HEATER_PIN,    OUTPUT);
  digitalWrite(HEATER_PIN,    OFF);  // Выключить нагреватель

  pinMode(HUMMIDITY_PIN,    OUTPUT);
  digitalWrite(HUMMIDITY_PIN,    OFF);  // Выключить увлажнитель

  Timer1.setFrequency(4);               // Высокоточный таймер 1 для первого прерывания, частота - 4 Герца
  Timer1.enableISR();                   // Запускаем прерывание (по умолч. канал А)

  Serial.println("Загрузка завершена");

  Watchdog.enable(RESET_MODE, WDT_PRESCALER_128); // Режим сторжевого сброса , таймаут ~1с

  enc1.setType(TYPE2);    // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый.
  enc1.setTickMode(AUTO);

}


void loop() 
{
  char EncNotTurned=0, EncTurned=0, TimerPrevSeconds=0;

  if (enc1.isHolded())  // ---------------------Удержание кнопки, вход в режим настроек
  {
    if(Settings_mode == 0)
    {
      Settings_mode = 1;
      Serial.println("Settings mode ON ");
      ShowDisplayAttr=1;
    }
    else 
    {
      Serial.println("Settings mode OFF ");
      ShowDisplayAttr=0;
      Settings_mode = 0;
      Save_eeprom();
    }
  }
  if(enc1.isSingle() && Settings_mode > 0)
  {
    Settings_mode++;
    if(Settings_mode == 6)  //------------------------------------------------------------------------ Сохранить в память и выйти из режима настроек
    {
      Settings_mode = 0;
      Save_eeprom();
    }
  }
  if( enc1.isTurn() )      // если был совершён поворот (индикатор поворота в любую сторону)
  {
    Encoder_Turn=1;
    Encoder_Status =Timer_Seconds+3;
    if(Encoder_Status>59) Encoder_Status=Encoder_Status-60;
    // ---------------------------------------- Регулировка яркости
    if(Settings_mode == 0)
    {
      if (enc1.isRight())
      {
        if(Preset_Data.LCD_BRIGHTNESS<7)  
        {
          Serial.print("Brightness = ");
          Preset_Data.LCD_BRIGHTNESS++;
          disp.brightness(Preset_Data.LCD_BRIGHTNESS);
          Serial.println(Preset_Data.LCD_BRIGHTNESS);
        }
      }
      if (enc1.isLeft())
      {
        if(Preset_Data.LCD_BRIGHTNESS!=0)  
        {
          Serial.print("Brightness = ");
          Preset_Data.LCD_BRIGHTNESS--;
          disp.brightness(Preset_Data.LCD_BRIGHTNESS);
          Serial.println(Preset_Data.LCD_BRIGHTNESS);
        }
      }
    }
    else
    {
      Serial.print("Settings_mode = ");
      Serial.println(Settings_mode);
      if(Settings_mode == 1)  //------------------------------------------------------------------------ Корректировка температуры инкубации
      {
        if (enc1.isRight())
        {
          if(Preset_Data.INCUBATOR_TEMP<33) Preset_Data.INCUBATOR_TEMP=Preset_Data.INCUBATOR_TEMP+0.1;
        }
        if (enc1.isLeft())
        {
          if(Preset_Data.INCUBATOR_TEMP>25) Preset_Data.INCUBATOR_TEMP=Preset_Data.INCUBATOR_TEMP-0.1;
        }
        Serial.print("Incubator temp = ");
        Serial.println(Preset_Data.INCUBATOR_TEMP);
      }
      if(Settings_mode == 2)  //------------------------------------------------------------------------ Корректировка гистерезиса температуры инкубации
      {
        if (enc1.isRight())
        {
          if(Preset_Data.HYSTERESIS<Preset_Data.MAX_HEATER_TEMP) Preset_Data.HYSTERESIS=Preset_Data.HYSTERESIS+0.1;
        }
        if (enc1.isLeft())
        {
          if(Preset_Data.HYSTERESIS>0) Preset_Data.HYSTERESIS=Preset_Data.HYSTERESIS-0.1;
        }
        Serial.print("Incubator temp hysteresis = ");
        Serial.println(Preset_Data.HYSTERESIS);
      }
      if(Settings_mode == 3)  //------------------------------------------------------------------------ Корректировка максимальной температуры нагревателя
      {
        if (enc1.isRight())
        {
          if((Preset_Data.MAX_HEATER_TEMP+Preset_Data.INCUBATOR_TEMP)<40)   Preset_Data.MAX_HEATER_TEMP=Preset_Data.MAX_HEATER_TEMP+0.1;
        }
        if (enc1.isLeft())
        {
          if((Preset_Data.MAX_HEATER_TEMP+Preset_Data.INCUBATOR_TEMP)>Preset_Data.INCUBATOR_TEMP)   Preset_Data.MAX_HEATER_TEMP=Preset_Data.MAX_HEATER_TEMP-0.1;
        }
        Serial.print("MAX_HEATER_TEMP = ");
        Serial.println(Preset_Data.MAX_HEATER_TEMP);
      }
      if(Settings_mode == 4)  //------------------------------------------------------------------------ Корректировка максимальной температуры нагревателя
      {
        if (enc1.isRight())
        {
          if(Preset_Data.DHT_RST<(24*60))   Preset_Data.DHT_RST=Preset_Data.DHT_RST+1;
        }
        if (enc1.isLeft())
        {
          if(Preset_Data.DHT_RST>0)   Preset_Data.DHT_RST=Preset_Data.DHT_RST-1;
        }
        Serial.print("DHT Reset Time = ");
        Serial.println(Preset_Data.DHT_RST);
      }
      if(Settings_mode == 5)  //------------------------------------------------------------------------ Корректировка режима работы инкубатора
      {
        if (enc1.isRight())
        {
          if(Preset_Data.SENSTEMP_MODE<4)   Preset_Data.SENSTEMP_MODE=Preset_Data.SENSTEMP_MODE+1;
        }
        if (enc1.isLeft())
        {
          if(Preset_Data.SENSTEMP_MODE>1)   Preset_Data.SENSTEMP_MODE=Preset_Data.SENSTEMP_MODE-1;
        }
        Serial.print("Incubator work mode = ");
        Serial.println(Preset_Data.SENSTEMP_MODE);
      }
    }
    //------------------------------------------------------ режим настроек
  }
  else    // Поворотов енкодера не было
  {
    if(Timer_Seconds>=Encoder_Status)
    {
      Encoder_Status=0;
      Encoder_Turn=0;
    }
  }

  if( Timer_Seconds == 60 )
  {
    Timer_Seconds=0;
    if( Preset_Data.DHT_RST > 0 )  Timer_Min=Timer_Min+1;
  }


  if(led_status==1) LED_ON;
  else              LED_OFF;


  if(DHT_read_data==1)
  {
    DHT_read_data=0;

    if(LCDCounter==1)
    {
        if( Preset_Data.DHT_RST > 0 && Timer_Min>= Preset_Data.DHT_RST )      // Проверка таймера на сброс (отключение) DHT22
        {
          DHT_Reset_Mode=1;
          DHT_off();
        }
    }
    if(LCDCounter==7)
    {
        if(DHT_Reset_Mode==1)     // Проверка на включение DHT22
        {
          DHT_Reset_Mode=0;
          DHT_on();
        }

    }

    if(LCDCounter==0)   // DHT22 температура считывается 1 раз в 2 сек
    {
      DHT.read22(DHT22_DATA_PIN);
      Serial.print("DHT22 ");
      Serial.print(DHT.humidity, 1);
      Serial.print(",\t");
      Serial.print(DHT.temperature, 1);
      Serial.print(",\t");
      Serial.print(regulator.getResult());
      Serial.println("");
      tDHT=DHT.temperature;
      regulator_hummidity.input = DHT.humidity;

      sensor.requestTemp(); // Отправить команду считывания температуры на DS18b20, считать которую можно через 920uSec
//      sensor2.requestTemp(); // Отправить команду считывания температуры на DS18b20, считать которую можно через 920uSec
    }


    if(LCDCounter==4)   // DS18b20 температура считывается 1 раз в 2 сек
    {
      sensor2.requestTemp(); // Отправить команду считывания температуры на DS18b20, считать которую можно через 920uSec
      
      Serial.print("DS18b20 ");

      Serial.print(" DS1 ");
      if (sensor.readTemp())  // Если температура DS1 считана, загружаем значение в регулятор
      {
        tDS=sensor.getTemp();
        Serial.print(tDS);
      }
      else
      {
              Serial.print("Tempreture Sensor1 Error");
              tDS=0;
      }
    }

    if(LCDCounter==6)   // DS18b20 температура считывается 1 раз в 2 сек
    {
      Serial.print(" DS2 ");
      if (sensor2.readTemp())  // Если температура DS2 считана, загружаем значение в регулятор
      {
        tDS2=sensor2.getTemp();
        Serial.print(tDS2);
      }
      else
      {
              Serial.print("Tempreture Sensor2 Error");
              tDS2=0;
      }
      Serial.println("");

    }

    if(LCDCounter==4 || LCDCounter==6)
    {
      tDS_Max=max(tDS,tDS2);
      DHT.temperature=tDS_Max;
    }

    if(LCDCounter==4 || LCDCounter==0)
    {
      if(tDHT<Preset_Data.MAX_TEMP && tDS<Preset_Data.MAX_TEMP && tDS2<Preset_Data.MAX_TEMP)
      {
        if(Preset_Data.SENSTEMP_MODE==1) regulator.input = tDS;
        if(Preset_Data.SENSTEMP_MODE==2) regulator.input = tDS2;
        if(Preset_Data.SENSTEMP_MODE==3) // использовать наибольшую температуру
        {
/*
          if( tDHT>max(tDS,tDS2) )    regulator.input = tDHT;
          else                        regulator.input = max(tDS,tDS2);
*/
          regulator.input = max(tDS,tDS2);
        }
        if(Preset_Data.SENSTEMP_MODE==4) regulator.input = min(tDS,tDS2); // использовать наименьшую температуру
//        if( tDHT<min(tDS,tDS2) )      regulator.input = tDHT;
//        else                          regulator.input = min(tDS,tDS2);
      }
      else
      {
        Serial.println("High temperature !!!");
        regulator.input =   regulator.setpoint+regulator.hysteresis+Preset_Data.MAX_TEMP;
      }
    }
      


    if(LCDCounter>=0 && LCDCounter<6) LCDShow=0;      // Показать температуру
    else                              LCDShow=1;      // Показать влажность
    
    
    if(LedRefresh==1)   // Обновить экран
    {
      LedRefresh=0;
      if(Settings_mode == 0)
      {
        if(LCDCounter==0 || LCDCounter==4 || LCDCounter==6)
        {
/*          if(LCDShow==0)  data=DHT.temperature*10;
          else            data=DHT.humidity;
          Show_LCD_Data(data, LCDShow);
*/
          if(LCDCounter==0)   Show_LCD_Data(tDS*10, LCDShow);
          if(LCDCounter==4)   Show_LCD_Data(tDS2*10, LCDShow);
          if(LCDCounter==6)   Show_LCD_Data(DHT.humidity, LCDShow);
/*
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
*/
        }
      } // if(Settings_mode == 0)
      else
      {
//        if(led_status==1) 
        if(ShowDisplayAttr>3 && Encoder_Turn==0)
        {
//          disp.clear();
          if(Settings_mode == 1) disp.displayByte(_t, _i, _n, _c);
          if(Settings_mode == 2) disp.displayByte(_H, _y, _S, _t);
          if(Settings_mode == 3) disp.displayByte(_H, _E, _A, _t);
          if(Settings_mode == 4) disp.displayByte(_d, _h, _t, 0x00);
          if(Settings_mode == 5) disp.displayByte(_E, _o, _d, _E);
        }
        else
        {
          if(Settings_mode == 1)
          {
            Show_LCD_Data( long (Preset_Data.INCUBATOR_TEMP*10), 0);
          }
          if(Settings_mode == 2)
          {
            Show_LCD_Data( long (Preset_Data.HYSTERESIS*10), 0);
          }
          if(Settings_mode == 3)
          {
            Show_LCD_Data( long ( (Preset_Data.INCUBATOR_TEMP+Preset_Data.MAX_HEATER_TEMP)*10 ), 0);
          }
          if(Settings_mode == 4)
          {
              disp.displayInt(int(Preset_Data.DHT_RST));
//            Show_LCD_Data( long (Preset_Data.DHT_RST), 1);
          }
          if(Settings_mode == 5)
          {
            disp.displayInt(int(Preset_Data.SENSTEMP_MODE));
          }
        }
        if(ShowDisplayAttr>0) ShowDisplayAttr=ShowDisplayAttr+1;
        if(ShowDisplayAttr==7) ShowDisplayAttr=1;
      }
    } //LedRefresh==1
    
  }

  digitalWrite(HEATER_PIN, regulator.getResult());
  digitalWrite(HUMMIDITY_PIN, regulator_hummidity.getResult());

  Watchdog.reset(); // Переодический сброс watchdog

}


void DHT_on()
{
  Timer_Min=0;
  Timer_Seconds=0;
  DHT_Reset_Mode=0;
  digitalWrite(DHT22_PWR_PIN, HIGH);
  Serial.println("----------- DHT ON -----------");
}


void DHT_off()
{
  DHT_Reset_Mode=1;
  digitalWrite(DHT22_PWR_PIN, LOW);
  Serial.println("----------- DHT OFF -----------");
}


void Save_eeprom()
{
      Preset_Data.MAX_TEMP = Preset_Data.MAX_HEATER_TEMP+Preset_Data.INCUBATOR_TEMP;
      regulator.setpoint = Preset_Data.INCUBATOR_TEMP;
      regulator.hysteresis = Preset_Data.HYSTERESIS;   // ширина гистерезиса
      EEPROM.put(0, Preset_Data);   // поместить в EEPROM по адресу 0
      DHT_on();
}



void Show_LCD_Data(long ldata, char cLCDShow)
{
  char heater_pin=0;
  heater_pin = digitalRead(HEATER_PIN);
    for (byte i = 0; i < 4; i++) bytes[i]=0x00;

    for (byte i = 0; i < 4; i++) 
    { //>
      bytes[i] = ldata % 10; // записываем остаток в буфер
      ldata /= 10;         // "сдвигаем" число
      if (ldata == 0) 
      {
        i=5;
      }
    } // массив bytes хранит цифры числа data в обратном порядке!
    if(cLCDShow==1)  disp.displayByte(0x00, 0x00, 0x00, 0x00);  //disp.clear();
    if(cLCDShow==0)                  // Показать символ "градус"
    {
      disp.display(0, bytes[2]);
      disp.point(true, false);
      disp.display(1, bytes[1]);
      disp.point(false, false);
      disp.display(2, bytes[0]);
      if(Settings_mode==0)
      {
        if(heater_pin)  disp.displayByte(3, 0b01101011);  // 0b01100011 0b01101011
        else            disp.displayByte(3, 0x63);  // 0b01100011 0b01101011
      }
      else              disp.displayByte(3, 0x63);
      
    }
    else
    {
      disp.point(false);
      disp.display(1, bytes[1]);
      disp.display(2, bytes[0]);
      if(Settings_mode==0)
      {
        if(heater_pin)  disp.displayByte(3, 0b00001000);
        else            disp.displayByte(3, 0b00000000);
      }
      else              disp.displayByte(3, 0b00000000);
//      disp.displayByte(3, 0x63);  // 0b00001000
    }
}



// Прерывание А таймера 1
ISR(TIMER1_A) 
{  // пишем  в сериал
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

     if(LCDCounter==0 || LCDCounter==4) Timer_Seconds=Timer_Seconds+1;  // Прошла 1 сек
}





void DHT_Read()
{
  int TempDecimal=0, TempInt=0;

  if(DHT_read_data==1)
  {
     DHT_read_data=0;
   }
}
