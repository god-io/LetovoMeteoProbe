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

// Библиотека для работы с шиной I2C для акселерометра/гироскопа
#include <I2Cdev.h>
// Библиотека акселерометра/гироскопа/магнетометра
#include <MPU9250.h>

// Библиотека для работы с EEPROM
#include <EEPROM.h>

// Можно заворачивать куски кода в #ifdef DEBUG... #endif чтобы сразу включить/выключить одним переключателем
#define DEBUG

// Режим калибровки акселерометра и гироскопа
// #define ACCEL_GYRO_CALIBRATE
// Режим калибровки магнитометра.
// #define MAGNETOMETER_CALIBRATE

// Оба параметра включать сразу нельзя! Калибровка акселя/гиро требует покоя, а магнитометра - вращения
#ifdef ACCEL_GYRO_CALIBRATE
#ifdef MAGNETOMETER_CALIBRATE
#error "You must to calibrate only one sensor: accel/gyro OR magnetometer! undef one of the parameter or both"
#endif
#endif

// Пин Chip Select для работы SPI карты памяти
#define CS_PIN (7)
// Пин для внешнего датчика температуры DS18B20
#define DS18B20_PIN (2)

// Адрес ячейки EEPROM для хранения данных о номере блока
#define BLACKBOX_COUNT_EEPROM_ADDRESS (4050)
// Адрес ячейки EEPROM для хранения размера структуры. Для будущих подсчётов
#define BLACKBOX_SIZE_EEPROM_ADDRESS (4051)

// Адрес начала ячеек EEPROM для хранения данных калибровки акселерометра/гироскопа
#define CALIBRATION_ACCEL_GYRO_EEPROM_ADDRESS (4001)
#define CALIBRATION_MAGNETOMETER_EEPROM_ADDRESS (4030)

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

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float magHeading;
    float magTiltHeading;
};

// Заставить компилятор максимально упаковать данные структуры в памяти без пробелов. Влияет на sizeof()
#pragma pack(push, 1)
// Структура для хранения блекбокса в EEPROM
struct EEPROMBlackbox
{
    uint8_t date;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;

    float latitude;
    float longitude;
    float altitude;
    float speed; // м/с
    float heading;

    float internal_temp; // C
    float external_temp;
    float humidity; // отн. проценты
    float pressure; // Па

};
// Вернуть всё как было
#pragma pack(pop)

// Структура для хранения данных калибровки магнетометра. Так как их нельзя занести в память датчика, носим с собой
struct magCalibration
{
    float mx;
    float my;
    float mz;
};


// Размер структуры для сохранения в байтах - автоподсчёт
#define BLACKBOX_STRUCT_SIZE (sizeof(struct EEPROMBlackbox))
// Период записи в ЕЕПРОМ в мс, из расчёта на 8 часов
#define BLACKBOX_PERIOD_MS (8 * 60 * 60 * 1000UL / (4000 / (BLACKBOX_STRUCT_SIZE)))

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

// Сохраняет нужные данные в EEPROM блекбокс.
void saveDataToBlackbox(const MP_Data &data, unsigned long &timer);


// Калибрует акселерометр и гироскоп. Записывает калибровку в EEPROM
void calibrateAccelGyro();
// Калибрует магнетометр. Записывает калибровку в EEPROM
void calibrateMagnetometer();
// Читает оффсеты из EEPROM и загружает в IMU и память
void setAccelGyroMagOffsets();
// Сохраняет данные с датчика MPU9250 в структуру
void saveMPU9250(MP_Data &data);


// Печатает простую строку в дебаг. Только строка!!!
void debugInfo(const char *str, bool crlf = true)
{
#ifdef DEBUG
    if (crlf)
        DEBUG_PORT.println(str);
    else
        DEBUG_PORT.print(str);

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
