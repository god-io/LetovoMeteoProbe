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
// Библиотека для работы с датчиком температуры/влажности SHT2x
#include <SHT2x.h>

// Библиотека для работы с шиной OneWire
#include <OneWire.h>
// Универсальная библиотека для Maxim/Dallas термометров
#include <DallasTemperature.h>

// Библиотека для работы с шиной I2C для акселерометра/гироскопа
#include <I2Cdev.h>
// Библиотека акселерометра/гироскопа/магнетометра
#include <MPU9250.h>

// Библиотека для работы с датчиком пыли
#include <GP2YDustSensor.h>

// Библиотека для работы с EEPROM
#include <EEPROM.h>

// Библиотека watchdog-timer
#include <GyverWDT.h>

// Можно заворачивать куски кода в #ifdef DEBUG... #endif чтобы сразу включить/выключить одним переключателем
// #define DEBUG

// Режим калибровки акселерометра и гироскопа
// #define ACCEL_GYRO_CALIBRATE
// Режим калибровки магнитометра.
// #define MAGNETOMETER_CALIBRATE

// Режим сброса EEPROM Blackbox
// #define EEPROM_BLACKBOX_RESET


// Оба параметра включать сразу нельзя! Калибровка акселя/гиро требует покоя, а магнитометра - вращения
#ifdef ACCEL_GYRO_CALIBRATE
#ifdef MAGNETOMETER_CALIBRATE
#error "You must to calibrate only one sensor: accel/gyro OR magnetometer! undef one of the parameter or both"
#endif
#endif

// Пин Chip Select для работы SPI карты памяти
#define CS_PIN (53)
// Пин для внешнего датчика температуры DS18B20
#define DS18B20_PIN (36)
// Пин для аналогового датчика УФ
#define UV_SENSOR_PIN (A8)

// Пин для выхода реле/ключа
#define RELAY_1_PIN (37)
#define RELAY_2_PIN (42)

// Пин аналогового сигнала с датчика пыли
#define DUST_SENSOR_PIN (A9)
// Цифровой пин управления светодиодом датчика пыли
#define DUST_SENSOR_LED_PIN (39)

// Пины для статусных светодиодов. На меге много портов
#define SUCCESS_POWER_PIN (13)

#define SUCCESS_BMP280_PIN (31)
#define SUCCESS_DS18B20_PIN (30)
#define SUCCESS_MPU9250_PIN (33)
#define SUCCESS_SHT2X_PIN (32)
#define SUCCESS_SD_PIN (35)
#define SUCCESS_GPS_PIN (34)


// Адрес ячейки EEPROM для хранения данных о номере блока
#define BLACKBOX_COUNT_EEPROM_ADDRESS (4050)
// Адрес ячейки EEPROM для хранения размера структуры. Для будущих подсчётов
#define BLACKBOX_SIZE_EEPROM_ADDRESS (4051)

// Адрес начала ячеек EEPROM для хранения данных калибровки акселерометра/гироскопа
#define CALIBRATION_ACCEL_GYRO_EEPROM_ADDRESS (4001)
#define CALIBRATION_MAGNETOMETER_EEPROM_ADDRESS (4030)

// Константа для пересчёта км/ч в м/с
#define KMH_TO_MS (1000.0f / 3600)

// Раз в 4 минуты включать камеры
#define RELAY_SWITCHON_TIMEOUT_S (240)
// через минуту выключать
#define RELAY_SWITCHOFF_TIMEOUT_S (75)

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
    unsigned long millis = 4000000000UL; // мс не передаются модулем, заменены на секунды со старта МК

    float latitude = 0.0f;
    float longitude = 0.0f;
    float altitude = 0.0f;
    float speed = -1000.0f; // м/с
    float heading = -1000.0f;
    float hdop = -1000.0f;
    float vdop = -1000.0f;
    uint8_t satellites_count = 255;

    float bmp280_temp = -1000.0f; // C
    float external_temp = -1000.0f;
    float sht2x_temp = -1000.0f; 
    float humidity = -1000.0f; // отн. проценты
    float pressure = -1000.0f; // Па

    float ax = -1000.0f; // в единицах g
    float ay= -1000.0f;
    float az= -1000.0f;
    float aAmp = -1000.0f; // суммарная амплитуда

    float gx = -1000.0f; // град/с
    float gy = -1000.0f;
    float gz = -1000.0f;

    float mx = -1000.0f;
    float my = -1000.0f;
    float mz = -1000.0f;

    float magHeading = -1000.0f;  // градусы

    int analogUV = -32000;  // 0 - 1023
    uint16_t analogDust = 65535; // 0 - 600 мкг/м3
    uint16_t dustRunningAverage = 65535; // 60 семплов
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
// Сохраняет данные с датчика SHT-21 в структуру
void saveSHT2x(MP_Data &data);
// Сохраняет данные с аналогового датчика УФ
void saveAnalogUV(MP_Data &data);
// Сохраняет данные с аналогового датчика пыли
void saveAnalogDust(MP_Data &data);
// Устанавливает режимы пинов и обнуляет всё
void resetPinModes();


// Сбрасывает счётчик записанных блоков в EEPROM блекбоксе
void eepromBlackboxReset();

// Включает и выключает реле
void relay();

// Макросы, прикидывающиеся функциями. Зажигает и тушит светодиод
#define LED_ON(pin) (digitalWrite((pin), (1)))
#define LED_OFF(pin) (digitalWrite((pin), (0)))

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
