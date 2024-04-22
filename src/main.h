#include <Arduino.h>

// Основной заголовочник библиотеки NeoGPS
#include <NMEAGPS.h>
// Автовыбор железного порта Serial1 для GPS и дебаг-порта
#include <GPSport.h>

// Нужно для работы с SD-картой, так как она подключена по шине SPI
#include <SPI.h>
// Библиотека для работы с SD-картой
#include <SD.h>

// Библиотека для работы с шиной I2C
#include <Wire.h>
// Библиотека для работы с датчиком давления BMP280
#include <Adafruit_BMP280.h>

// Библиотека для работы с шиной OneWire
#include <OneWire.h>
// Универсальная библиотека для Maxim/Dallas термометров
#include <DallasTemperature.h>


// Можно заворачивать куски кода в #ifdef DEBUG... #endif чтобы сразу включить/выключить одним переключателем
#define DEBUG

// Пин Chip Select для работы SPI карты памяти
#define CS_PIN (7)
// Пин для внешнего датчика температуры DS18B20
#define DS18B20_PIN (2)

// Константа для пересчёта км/ч в м/с
#define KMH_TO_MS (1000.0f / 3600)

// Структура для хранения строки данных Meteorological Probe Data
// Дефолтные данные неверные изначально. Они заменяются реальными только если валидны
struct MP_Data
{
    uint8_t year = 255;
    uint8_t month = 255;
    uint8_t date = 255;
    uint8_t hours = 255;
    uint8_t minutes = 255;
    uint8_t seconds = 255;
    uint16_t ms = 65535; // мс не передаются модулем TODO - заменить на секунды со старта МК

    float latitude = 0.0f;
    float longitude = 0.0f;
    float altitude = 0.0f;
    float speed = -1000.0f; // м/с
    float heading = -1000.0f;
    float velocity_north = -1000.0f; // м/с
    float velocity_east = -1000.0f;
    float velocity_down = -1000.0f;
    float hdop = -1000.0f;
    float vdop = -1000.0f;
    uint8_t satellites_count = 255;

    float internal_temp = -1000.0f; // C
    float external_temp = -1000.0f;
    float humidity = -1000.0f; // отн. проценты
    float pressure = -1000.0f; // Па

};

// Объявление функций

// Сохраняет данные GPS в структуре
// Структура передаётся по ссылке чтобы можно было писать в неё
bool saveGPS(MP_Data &data);
// Сохраняет данные давления и температуры в структуре
void saveBMP280(MP_Data &data);
// Сохраняет данные с внешнего датчика температуры
void saveDS18B20(MP_Data &data);

// Печатает данные на карту памяти
// Передаём структуру по ссылке, но говорим что это константа. Это на случай защиты от непреднамеренного изменения
bool printToSD(const MP_Data &data);
// Печатает данные в консоль для дебага - самое простое решение
void printToConsole(const MP_Data &data);

// Печатает заголовок данных на карту памяти
void printHeaderToSD();

// Печатает простую строку в дебаг. Только строка!!!
void debugInfo(const char *str)
{
#ifdef DEBUG
    DEBUG_PORT.println(str);
    DEBUG_PORT.flush();
#endif
}

// Перегрузка функции выше - то же имя, но другие параметры
// Печатает Flash-строку в дебаг. Flash-строка экономит оперативку!
void debugInfo(const __FlashStringHelper *flash)
{
#ifdef DEBUG
    DEBUG_PORT.println(flash);
    DEBUG_PORT.flush();
#endif
}

