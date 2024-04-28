#include <main.h>

// Глобальные переменные
// Объект (экземпляр класса NMEAGPS из NeoGPS), необходимый для работы с модулем GNSS
NMEAGPS gps;

// Объект (экземпляр класса Adafruit_BMP280), необходимый для работы с датчиком давления
Adafruit_BMP280 bmp280;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(DS18B20_PIN);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature outsideTemperatureSensor(&oneWire);
// arrays to hold device address
DeviceAddress outsideThermometerAddress;

// Объект (экземпляр класса File из SD), необходимый для работы с картой памяти
File sd;

// Таймер для записи блекбокса в EEPROM
unsigned long eepromBlackboxTimer = 0;

void setup()
{
    // Блок кода в условном операторе препроцессора. Не объявлено DEBUG - этот блок кода будет удалён
#ifdef DEBUG
    DEBUG_PORT.begin(9600);
    while (!DEBUG_PORT)
    {
        ;
    }
#endif

    debugInfo("Initialization process started...");
    debugInfo("Start GPS port on Serial1 at 9600 bps");
    // Начать работу на порту Serial1 для GPS-модуля
    gpsPort.begin(9600);

    // TODO: добавить проверку что GPS подключён


    debugInfo("Check BMP280 pressure sensor");
    if (bmp280.begin(BMP280_ADDRESS_ALT))
    {
        debugInfo("BMP280 sensor present, let's configure");
        bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                           Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                           Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                           Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                           Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */
    }
    else
    {
        debugInfo("ERROR - BMP280 not present!");
        while (true)
        {
            ;
        }
    }

    // Поиск устройств на шине oneWire
    debugInfo("Locating oneWire devices...");
    outsideTemperatureSensor.begin();
    if (outsideTemperatureSensor.getDeviceCount() == 0)
    {
        debugInfo("ERROR - cannot find DS18B20 temp sensor!");
        while (true)
        {
            ;
        }
    }
    else
    {
        // датчик может работать на паразитном питании (в пин данных), но его показания тогда не такие верные
        debugInfo("Found oneWire sensor (DS18B20)");

        if (outsideTemperatureSensor.isParasitePowerMode())
        {
            debugInfo("oneWire bus requires parasite power");
        }
        else
        {
            debugInfo("oneWire bus doesnt require parasite power");
        }

        // Must be called before search()
        oneWire.reset_search();
        // assigns the first address found to insideThermometer
        if (!oneWire.search(outsideThermometerAddress))
        {
            debugInfo("ERROR - Unable to find address for oneWire Device");
            while (true)
            {
                ;
            }
        }

        debugInfo("Found oneWire device address (DS18B20)");

        // Разрешение АЦП датчика. До 12 бит (4096 градации), но медленно получает данные (750 мс!)
        outsideTemperatureSensor.setResolution(outsideThermometerAddress, 10);
    }

    debugInfo("Initializing SD card...");

    // Проверяем что карта памяти может работать
    if (!SD.begin(CS_PIN))
    {
        debugInfo("ERROR - Initialization of SD card FAILED! Check wiring and format SD to FAT32");
        while (true) // Не работает, уходим в зависон
        {
            ;
        }
    }

    debugInfo("SD Card OK, try to open file...");

    // Пробуем открыть файл для записи (если он есть, выполнится append, если нет, создастся)
    sd = SD.open("meteolog.txt", FILE_WRITE);
    if (sd)
    {

        debugInfo("File from SD Card opened SUCCESSFULLY!");

        printHeaderToSD(); // Печатаем заголовок в файл
    }
    else
    {
        debugInfo("ERROR - FAILED to open file from SD Card!");

        while (true) // Не работает файл, уходим в зависон
        {
            ;
        }
    }
}

void loop()
{
    // На каждое измерение своя структура! Она равна 1 строке данных
    MP_Data meteodata;

    if (saveGPS(meteodata)) // Нет смысла опрашивать датчики, если gps невалиден вообще
    {
        saveBMP280(meteodata);
        saveDS18B20(meteodata);

        printToSD(meteodata);

        if ((millis() - eepromBlackboxTimer) > BLACKBOX_PERIOD_MS)
        {
            // Сохраним нужные данные из meteodata в структуру blackbox
            saveDataToBlackbox(meteodata, eepromBlackboxTimer);
        }

#ifdef DEBUG
        printToConsole(meteodata);
#endif
    }
}

// Возвращает true, если gps валиден, false если нет
bool saveGPS(MP_Data &data)
{
    // The available(...) function return the number of *fixes* that
    //   are available to be "read" from the fix buffer.
    while (gps.available(gpsPort))
    {
#ifdef DEBUG
        // Измерим время обработки
        unsigned long start = millis();
#endif

        // Если был обработан полный набор предложений в интервале, available вернёт что-то больше 0
        // Можно прочесть данные в облегчённую структуру gps_fix
        const gps_fix fix = gps.read();

        debugInfo("GPS Fix read attempt");

        if (fix.valid.status) // Если статус захвата в норме, выполняем обработку
        {
            debugInfo("0. GPS Status Valid");

            // Проверим что данные даты валидны
            if (fix.valid.date)
            {
                // Никакого использования String пока не убедимся что памяти хватит на всё
                data.year = fix.dateTime.year;
                data.month = fix.dateTime.month;
                data.date = fix.dateTime.date;

                debugInfo("1. GPS Date Valid");
            }

            /*
                Можно так без if вообще
                data.year = fix.valid.date ? fix.dateTime.year : 255;
                data.month = fix.valid.date ? fix.dateTime.month : 255;
                data.date = fix.valid.date ? fix.dateTime.date : 255;
            */

            // Тоже самое делаем со временем
            if (fix.valid.time)
            {
                data.hours = fix.dateTime.hours;
                data.minutes = fix.dateTime.minutes;
                data.seconds = fix.dateTime.seconds;
                data.ms = fix.dateTime_ms();

                debugInfo("2. GPS Time Valid");
            }

            // Проверяем локацию (широта и долгота)
            if (fix.valid.location)
            {
                data.latitude = fix.latitude();
                data.longitude = fix.longitude();

                debugInfo("3. GPS Position Valid");
            }

            // Проверяем высоту, она отдельна от локации
            if (fix.valid.altitude)
            {
                data.altitude = fix.altitude();

                debugInfo("4. GPS Altitude Valid");
            }

            // Скорость полёта
            if (fix.valid.speed)
            {
                data.speed = fix.speed_kph() * KMH_TO_MS;

                debugInfo("5. GPS Speed Valid");
            }

            // Направление движения в градусах
            if (fix.valid.heading)
            {
                data.heading = fix.heading();

                debugInfo("6. GPS Heading Valid");
            }

            // скорости NED (зависят от heading и speed)
            if (fix.valid.velned)
            {
                data.velocity_north = fix.velocity_north / 100.0f; // NED в целых см/с, делаем м/с
                data.velocity_east = fix.velocity_east / 100.0f;
                data.velocity_down = fix.velocity_down / 100.0f;

                debugInfo("7. GPS VelNED Valid"); // NED в целых см/с, делаем м/с
            }

            // Horizontal dilution of precision
            if (fix.valid.hdop)
            {
                data.hdop = fix.hdop / 1000.0f; // hdop в целых тысячных долях, поэтому hdop 8.5 выглядит как  8500

                debugInfo("8. GPS HDOP Valid");
            }

            // Vertical dilution of precision
            if (fix.valid.vdop)
            {
                data.vdop = fix.vdop / 1000.0f; // vdop в целых тысячных долях, поэтому hdop 8.5 выглядит как  8500

                debugInfo("9. GPS VDOP Valid");
            }

            // Number of satellites used to calculate a fix.
            if (fix.valid.satellites)
            {
                data.satellites_count = fix.satellites;

                debugInfo("10. GPS Satellites Valid");
            }

#ifdef DEBUG
            // Сколько времени заняло получение данных
            DEBUG_PORT.print("GPS save loop took: ");
            DEBUG_PORT.print(millis() - start);
            DEBUG_PORT.println(" ms");
#endif

            return true; // GPS хоть частично ОК, возвращаем тру
        }
        else
        {
            debugInfo("0. GPS is not fixed now!");
        }
    }

    return false; // GPS не ОК, возвращаем false
}

void saveBMP280(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    debugInfo("Reads BMP280 temperature and pressure");
    data.internal_temp = bmp280.readTemperature();
    data.pressure = bmp280.readPressure();

#ifdef DEBUG
    DEBUG_PORT.print("Save BMP280 took: ");
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif
}

void saveDS18B20(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    debugInfo("Reads DS18B20 outside temperature");

    outsideTemperatureSensor.requestTemperaturesByAddress(outsideThermometerAddress);
    float tempC = outsideTemperatureSensor.getTempC(outsideThermometerAddress);
    if (tempC == DEVICE_DISCONNECTED_C)
    {
        debugInfo("ERROR - Could not read temperature data!");
        return;
    }
    data.external_temp = tempC;

#ifdef DEBUG
    DEBUG_PORT.print("Save DS18B20 took: ");
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif
}

bool printToSD(const MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif
    if (!sd)
    {
        debugInfo("ERROR - cannot write to file!");
        return false;
    }

    debugInfo("*** Send data to SD ***");

    sd.print(data.year);
    sd.print(',');
    sd.print(data.month);
    sd.print(',');
    sd.print(data.date);
    sd.print(',');
    sd.print(data.hours);
    sd.print(',');
    sd.print(data.minutes);
    sd.print(',');
    sd.print(data.seconds);
    sd.print(',');
    sd.print(data.ms);
    sd.print(',');
    sd.print(data.latitude, 6);
    sd.print(',');
    sd.print(data.longitude, 6);
    sd.print(',');
    sd.print(data.altitude, 3);
    sd.print(',');
    sd.print(data.speed, 3);
    sd.print(',');
    sd.print(data.heading, 2);
    sd.print(',');
    sd.print(data.velocity_north, 3);
    sd.print(',');
    sd.print(data.velocity_east, 3);
    sd.print(',');
    sd.print(data.velocity_down, 3);
    sd.print(',');
    sd.print(data.hdop, 2);
    sd.print(',');
    sd.print(data.vdop, 2);
    sd.print(',');
    sd.print(data.satellites_count);
    sd.print(',');
    sd.print(data.internal_temp, 1);
    sd.print(',');
    sd.print(data.external_temp, 1);
    sd.print(',');
    sd.print(data.humidity, 1);
    sd.print(',');
    sd.print(data.pressure, 3);

    sd.println();
    sd.flush(); // Обязательно сброс буферов!

#ifdef DEBUG
    DEBUG_PORT.print("Save to SD took: ");
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif

    return true;
}

void printToConsole(const MP_Data &data)
{
    DEBUG_PORT.println("*** Send data to console ***");

    DEBUG_PORT.print("Date/Time: ");
    DEBUG_PORT.print(data.year);
    DEBUG_PORT.print('-');
    DEBUG_PORT.print(data.month);
    DEBUG_PORT.print('-');
    DEBUG_PORT.print(data.date);
    DEBUG_PORT.print(' ');
    DEBUG_PORT.print(data.hours);
    DEBUG_PORT.print(':');
    DEBUG_PORT.print(data.minutes);
    DEBUG_PORT.print(':');
    DEBUG_PORT.print(data.seconds);
    DEBUG_PORT.print('.');
    DEBUG_PORT.print(data.ms);
    DEBUG_PORT.println();

    DEBUG_PORT.print("Lat/Lon/Alt: ");
    DEBUG_PORT.print(data.latitude, 6);
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(data.longitude, 6);
    DEBUG_PORT.print(" - ");
    DEBUG_PORT.print(data.altitude, 3);
    DEBUG_PORT.println(" m");

    DEBUG_PORT.print("Spd/Hdg: ");
    DEBUG_PORT.print(data.speed, 3);
    DEBUG_PORT.print(" m/s, ");
    DEBUG_PORT.print(data.heading, 2);
    DEBUG_PORT.println(" grad");

    DEBUG_PORT.println("Velocity N/E/D: ");
    DEBUG_PORT.print(data.velocity_north, 3);
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(data.velocity_east, 3);
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(data.velocity_down, 3);
    DEBUG_PORT.println(" m/s");

    DEBUG_PORT.print("HDOP/VDOP: ");
    DEBUG_PORT.print(data.hdop, 2);
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(data.vdop, 2);
    DEBUG_PORT.println();

    DEBUG_PORT.print("Sats: ");
    DEBUG_PORT.print(data.satellites_count);
    DEBUG_PORT.println();

    DEBUG_PORT.print("Internal Temp: ");
    DEBUG_PORT.print(data.internal_temp, 1);
    DEBUG_PORT.println(" C");

    DEBUG_PORT.print("External Temp: ");
    DEBUG_PORT.print(data.external_temp, 1);
    DEBUG_PORT.println(" C");

    DEBUG_PORT.print("Humidity: ");
    DEBUG_PORT.print(data.humidity, 1);
    DEBUG_PORT.println("rel. percent");

    DEBUG_PORT.print("Pressure: ");
    DEBUG_PORT.print(data.pressure, 3);
    DEBUG_PORT.println(" Pa");

    DEBUG_PORT.println();
    DEBUG_PORT.flush();
}

void printHeaderToSD()
{
    if (sd)
    {
        debugInfo("Print SD header");

        sd.println(F("------------"));
        sd.print(F("year,month,date,hours,minutes,seconds,ms,"));
        sd.print(F("latitude,longitude,altitude,speed,heading,V_N,V_E,V_D,HDOP,VDOP,satellites,"));
        sd.print(F("in_temp,out_temp,humidity,pressure"));

        sd.println();
        sd.flush();
    }
    else
    {
        debugInfo(F("SD card writing error!"));
    }
}

void saveDataToBlackbox(const MP_Data &data, unsigned long &timer)
{
    // Структура для записи
    EEPROMBlackbox blackbox;

    blackbox.date = data.date;
    blackbox.hours = data.hours;

    blackbox.minutes = data.minutes;
    blackbox.seconds = data.seconds;

    blackbox.latitude = data.latitude;
    blackbox.longitude = data.longitude;
    blackbox.altitude = data.altitude;
    blackbox.speed = data.speed; // м/с
    blackbox.heading = data.heading;

    blackbox.internal_temp = data.internal_temp; // C
    blackbox.external_temp = data.external_temp;
    blackbox.humidity = data.humidity; // отн. проценты
    blackbox.pressure = data.pressure; // Па

    // Номер записанного блока
    uint8_t blackboxBlock;
    // Читаем текущее значение записанного блока
    EEPROM.get(BLACKBOX_COUNT_EEPROM_ADDRESS, blackboxBlock);

    // Преобразуем блок в стартовый адрес
    uint16_t nextWriteAddress = BLACKBOX_STRUCT_SIZE * (blackboxBlock + 1);

    // Сохраним новый номер блока
    EEPROM.put(BLACKBOX_COUNT_EEPROM_ADDRESS, blackboxBlock + 1);
    // Теперь пихаем в EEPROM структуру
    EEPROM.put(nextWriteAddress, blackbox);

    // Поставим таймер для следующей итерации
    timer = millis();
}