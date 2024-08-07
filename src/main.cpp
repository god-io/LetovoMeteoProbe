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

// Объект (экземпляр класса MPU9250) для работы с акселерометром/гироскопом
MPU9250 imu;
// Экземпляр класса I2Cdev для работы с шиной i2c
I2Cdev I2C_M;
// Экземпляр класса SHT2x для работы с датчиком SHT-21
SHT2x sht2x;
// Экземпляр класса GP2YDustSensor для работы с датчиков пыли Sharp
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1014AU0F, DUST_SENSOR_LED_PIN, DUST_SENSOR_PIN);

// Таймер для записи блекбокса в EEPROM
unsigned long eepromBlackboxTimer = 0;
// Таймеры для работы реле
unsigned long relayOnTimer = 0;
unsigned long relayOffTimer = 0;
// Признак включения реле
bool relayON = false;

// Состояния датчиков при старте. Нужны для случая определения отвала датчика
bool isBMP280Present = false;
bool isDS18B20Present = false;
bool isMPU9250Present = false;
bool isSHT2xPresent = false;

// Признак захвата позиции GPS для светодиодной индикации
bool isGPSFixed = false;

// Экземпляр структуры для хранения параметров калибровки магнетометра
magCalibration magCal;

void setup()
{
#ifndef ACCEL_GYRO_CALIBRATE
#ifndef MAGNETOMETER_CALIBRATE
#ifndef EEPROM_BLACKBOX_RESET
    // Режим сторжевого таймера - сброс , таймаут ~8с
    // Запуск только в рабочем режиме без калибровок и стираний, а то они долгие
    Watchdog.enable(RESET_MODE, WDT_PRESCALER_1024);
    Watchdog.reset();
#endif
#endif
#endif

    // Блок кода в условном операторе препроцессора. Не объявлено DEBUG - этот блок кода будет удалён
#ifdef DEBUG
    DEBUG_PORT.begin(9600);
    while (!DEBUG_PORT)
    {
        ;
    }
#endif

#ifdef EEPROM_BLACKBOX_RESET
    // Сброс счётчика хранения блекбокса в EEPROM
    eepromBlackboxReset();
#endif
    // Выключаем панель статуса и устанавливаем пины
    resetPinModes();

    LED_ON(SUCCESS_POWER_PIN);

    debugInfo(F("Initialization process started..."));
    debugInfo(F("Start GPS port on Serial1 at 9600 bps"));
    // Начать работу на порту Serial1 для GPS-модуля
    gpsPort.begin(9600);

    debugInfo(F("Check BMP280 pressure sensor"));
    if (bmp280.begin(BMP280_ADDRESS_ALT))
    {
        debugInfo(F("BMP280 sensor present, let's configure"));
        bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                           Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                           Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                           Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                           Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

        LED_ON(SUCCESS_BMP280_PIN);
        isBMP280Present = true;
    }
    else
    {
        debugInfo(F("ERROR - BMP280 not present!"));
    }

    // Поиск устройств на шине oneWire
    debugInfo(F("Locating oneWire devices..."));
    outsideTemperatureSensor.begin();
    if (outsideTemperatureSensor.getDeviceCount() == 0)
    {
        debugInfo(F("ERROR - cannot find DS18B20 temp sensor!"));
    }
    else
    {
        // датчик может работать на паразитном питании (в пин данных), но его показания тогда не такие верные
        debugInfo(F("Found oneWire sensor (DS18B20)"));

        if (outsideTemperatureSensor.isParasitePowerMode())
        {
            debugInfo(F("oneWire bus requires parasite power"));
        }
        else
        {
            debugInfo(F("oneWire bus doesnt require parasite power"));
        }

        // Must be called before search()
        oneWire.reset_search();
        // assigns the first address found to insideThermometer

        if (oneWire.search(outsideThermometerAddress))
        {
            debugInfo(F("Found oneWire device address (DS18B20)"));

            // Разрешение АЦП датчика. До 12 бит (4096 градации), но медленно получает данные (750 мс!)
            outsideTemperatureSensor.setResolution(outsideThermometerAddress, 10);

            LED_ON(SUCCESS_DS18B20_PIN);
            isDS18B20Present = true;
        }
        else
        {
            debugInfo(F("ERROR - Unable to find address for oneWire Device"));
        }
    }

    // Акселерометр/гироскоп/магнетометр
    debugInfo(F("Check MPU9250 IMU and magnetometer"));
    Wire.begin(); // МБ не надо?

    debugInfo(F("IMU initialize"));
    imu.initialize();

    debugInfo(F("Testing device connections..."));
    if (imu.testConnection())
    {
        debugInfo(F("MPU9250 connection successful"));
        debugInfo(F("Set Accel and Gyro ranges"));
        // Установка диапазона акселерометра/гироскопа
        imu.setFullScaleAccelRange(MPU9250_ACCEL_FS_8); // 8G
        imu.setFullScaleGyroRange(MPU9250_GYRO_FS_500); // 500 град/с

#ifdef DEBUG
        debugInfo("Accel range now is:", false);
        DEBUG_PORT.println(imu.getFullScaleAccelRange());
        debugInfo("Gyro range now is:", false);
        DEBUG_PORT.println(imu.getFullScaleGyroRange());
#endif

#ifdef ACCEL_GYRO_CALIBRATE
        calibrateAccelGyro();
#endif

#ifdef MAGNETOMETER_CALIBRATE
        calibrateMagnetometer();
#endif

        debugInfo(F("Read and set AGM offsets..."));
        setAccelGyroMagOffsets();

        LED_ON(SUCCESS_MPU9250_PIN);
        isMPU9250Present = true;
    }
    else
    {
        debugInfo(F("ERROR - MPU9250 connection failed!"));
    }

    Wire.begin();

    sht2x.begin(); // Надо обязательно удостовериться что перед этой операцией вызвана Wire.begin();

    if (sht2x.isConnected())
    {
        debugInfo(F("SHT-21 Connected"));
        debugInfo(F("Error and status: "));
#ifdef DEBUG

        sht2x.reset();
        debugInfo(F("Temp and Humidity"));
        sht2x.read();

        DEBUG_PORT.println(sht2x.getTemperature());

        DEBUG_PORT.println(sht2x.getHumidity());

#endif

        LED_ON(SUCCESS_SHT2X_PIN);
        isSHT2xPresent = true;
    }
    else
    {
        debugInfo(F("ERROR - Initialization of SHT2X FAILED!"));
    }

    debugInfo(F("Initializing SD card..."));

    // Проверяем что карта памяти может работать
    if (!SD.begin(CS_PIN))
    {
        debugInfo(F("ERROR - Initialization of SD card FAILED! Check wiring and format SD to FAT32"));
        while (true) // Не работает, уходим в зависон
        {
            ;
        }
    }

    debugInfo(F("SD Card OK, try to open file..."));

    // Пробуем открыть файл для записи (если он есть, выполнится append, если нет, создастся)
    sd = SD.open("meteolog.txt", FILE_WRITE);
    if (sd)
    {

        debugInfo(F("File from SD Card opened SUCCESSFULLY!"));

        printHeaderToSD(); // Печатаем заголовок в файл

        LED_ON(SUCCESS_SD_PIN);
    }
    else
    {
        debugInfo(F("ERROR - FAILED to open file from SD Card!"));

        while (true) // Не работает файл, уходим в зависон
        {
            ;
        }
    }

    // Датчик пыли
    dustSensor.begin();

    // Нет нужны окружать ifdef калибровок, так как функции калибровок блокирующие исполнение далее
    Watchdog.reset();
}

void loop()
{
    // На каждое измерение своя структура! Она равна 1 строке данных
    MP_Data meteodata;

    if (saveGPS(meteodata)) // Нет смысла опрашивать датчики, если gps невалиден вообще
    {
        // Обязательно проверим что датчик существует перед попыткой сбора данных
        if (isBMP280Present)
        {
            saveBMP280(meteodata);
        }

        if (isDS18B20Present)
        {
            saveDS18B20(meteodata);
        }

        if (isMPU9250Present)
        {
            saveMPU9250(meteodata);
        }

        if (isSHT2xPresent)
        {
            saveSHT2x(meteodata);
        }

        saveAnalogUV(meteodata);
        saveAnalogDust(meteodata);

        printToSD(meteodata);

        if ((millis() - eepromBlackboxTimer) > BLACKBOX_PERIOD_MS)
        {
            // Сохраним нужные данные из meteodata в структуру blackbox
            saveDataToBlackbox(meteodata, eepromBlackboxTimer);
        }

        relay();

#ifdef DEBUG
        printToConsole(meteodata);
#endif
    }

    if (isGPSFixed) { LED_ON(SUCCESS_GPS_PIN); }
    else { LED_OFF(SUCCESS_GPS_PIN); }

    // Обнулить (покормить) таймер, чтобы он не перезапускал МК
    Watchdog.reset();
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

        debugInfo(F("GPS Fix read attempt"));

        if (fix.valid.status) // Если статус захвата в норме, выполняем обработку
        {
            debugInfo(F("0. GPS Status Valid"));
            if (fix.status == fix.STATUS_STD)
                debugInfo(F("0. GPS Fixed "));
            else
                debugInfo(F("0. GPS NOT FIXED"));

            data.millis = millis();

            // Проверим что данные даты валидны
            if (fix.valid.date)
            {
                // Никакого использования String пока не убедимся что памяти хватит на всё
                data.year = fix.dateTime.year;
                data.month = fix.dateTime.month;
                data.date = fix.dateTime.date;

                debugInfo(F("1. GPS Date Valid"));
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

                debugInfo(F("2. GPS Time Valid"));
            }

            // Проверяем локацию (широта и долгота)
            if (fix.valid.location)
            {
                data.latitude = fix.latitude();
                data.longitude = fix.longitude();

                debugInfo(F("3. GPS Position Valid"));
                isGPSFixed = true;
            }
            else {
                isGPSFixed = false;
            }

            // Проверяем высоту, она отдельна от локации
            if (fix.valid.altitude)
            {
                data.altitude = fix.altitude();

                debugInfo(F("4. GPS Altitude Valid"));
            }

            // Скорость полёта
            if (fix.valid.speed)
            {
                data.speed = fix.speed_kph() * KMH_TO_MS;

                debugInfo(F("5. GPS Speed Valid"));
            }

            // Направление движения в градусах
            if (fix.valid.heading)
            {
                data.heading = fix.heading();

                debugInfo(F("6. GPS Heading Valid"));
            }

            // скорости NED (зависят от heading и speed)
            if (fix.valid.velned)
            {
                debugInfo(F("7. GPS VelNED not available in this module")); // NED недоступен в модуле
            }

            // Horizontal dilution of precision
            if (fix.valid.hdop)
            {
                data.hdop = fix.hdop / 1000.0f; // hdop в целых тысячных долях, поэтому hdop 8.5 выглядит как  8500

                debugInfo(F("8. GPS HDOP Valid"));
            }

            // Vertical dilution of precision
            if (fix.valid.vdop)
            {
                data.vdop = fix.vdop / 1000.0f; // vdop в целых тысячных долях, поэтому hdop 8.5 выглядит как  8500

                debugInfo(F("9. GPS VDOP Valid"));
            }

            // Number of satellites used to calculate a fix.
            if (fix.valid.satellites)
            {
                data.satellites_count = fix.satellites;

                debugInfo(F("10. GPS Satellites Valid"));
            }

#ifdef DEBUG
            // Сколько времени заняло получение данных
            DEBUG_PORT.print(F("GPS save loop took: "));
            DEBUG_PORT.print(millis() - start);
            DEBUG_PORT.println(" ms");
#endif

            return true; // GPS хоть частично ОК, возвращаем тру
        }
        else
        {
            debugInfo(F("0. GPS is not fixed now!"));
        }
    }

    return false; // GPS не ОК, возвращаем false
}

void saveBMP280(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    debugInfo(F("Reads BMP280 temperature and pressure"));
    data.bmp280_temp = bmp280.readTemperature();
    data.pressure = bmp280.readPressure();

#ifdef DEBUG
    DEBUG_PORT.print(F("Save BMP280 took: "));
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif
}

void saveDS18B20(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    debugInfo(F("Reads DS18B20 outside temperature"));

    outsideTemperatureSensor.requestTemperaturesByAddress(outsideThermometerAddress);
    float tempC = outsideTemperatureSensor.getTempC(outsideThermometerAddress);
    if (tempC == DEVICE_DISCONNECTED_C)
    {
        debugInfo(F("ERROR - Could not read temperature data!"));
        return;
    }
    data.external_temp = tempC;

#ifdef DEBUG
    DEBUG_PORT.print(F("Save DS18B20 took: "));
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
        debugInfo(F("ERROR - cannot write to file!"));
        return false;
    }

    debugInfo(F("*** Send data to SD ***"));

    LED_OFF(SUCCESS_SD_PIN);

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
    sd.print(data.millis);
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
    sd.print(data.hdop, 2);
    sd.print(',');
    sd.print(data.vdop, 2);
    sd.print(',');
    sd.print(data.satellites_count);
    sd.print(',');
    sd.print(data.bmp280_temp, 2);
    sd.print(',');
    sd.print(data.external_temp, 2);
    sd.print(',');
    sd.print(data.sht2x_temp, 2);
    sd.print(',');
    sd.print(data.humidity, 2);
    sd.print(',');
    sd.print(data.pressure, 2);
    sd.print(',');
    sd.print(data.ax, 3);
    sd.print(',');
    sd.print(data.ay, 3);
    sd.print(',');
    sd.print(data.az, 3);
    sd.print(',');
    sd.print(data.aAmp, 3);
    sd.print(',');
    sd.print(data.gx, 3);
    sd.print(',');
    sd.print(data.gy, 3);
    sd.print(',');
    sd.print(data.gz, 3);
    sd.print(',');
    sd.print(data.mx, 3);
    sd.print(',');
    sd.print(data.my, 3);
    sd.print(',');
    sd.print(data.mz, 3);
    sd.print(',');
    sd.print(data.magHeading, 3);
    sd.print(',');
    sd.print(data.analogUV);
    sd.print(',');
    sd.print(data.analogDust);
    sd.print(',');
    sd.print(data.dustRunningAverage);

    sd.println();
    sd.flush(); // Обязательно сброс буферов!

#ifdef DEBUG
    DEBUG_PORT.print(F("Save to SD took: "));
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif

    LED_ON(SUCCESS_SD_PIN);
    return true;
}

void printToConsole(const MP_Data &data)
{
    DEBUG_PORT.println(F("*** Send data to console ***"));

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
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(data.millis);
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

    DEBUG_PORT.print("HDOP/VDOP: ");
    DEBUG_PORT.print(data.hdop, 2);
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(data.vdop, 2);
    DEBUG_PORT.println();

    DEBUG_PORT.print("Sats: ");
    DEBUG_PORT.print(data.satellites_count);
    DEBUG_PORT.println();

    DEBUG_PORT.print("Internal Temp: ");
    DEBUG_PORT.print(data.bmp280_temp, 2);
    DEBUG_PORT.println(" C");

    DEBUG_PORT.print("External Temp: ");
    DEBUG_PORT.print(data.external_temp, 2);
    DEBUG_PORT.println(" C");

    DEBUG_PORT.print("SHT Internal Temp: ");
    DEBUG_PORT.print(data.sht2x_temp, 2);
    DEBUG_PORT.println(" C");

    DEBUG_PORT.print("Humidity: ");
    DEBUG_PORT.print(data.humidity, 2);
    DEBUG_PORT.println(" rel. percent");

    DEBUG_PORT.print("Pressure: ");
    DEBUG_PORT.print(data.pressure, 2);
    DEBUG_PORT.println(" Pa");

    DEBUG_PORT.print("Ax: ");
    DEBUG_PORT.print(data.ax, 3);
    DEBUG_PORT.println(" G");

    DEBUG_PORT.print("Ay: ");
    DEBUG_PORT.print(data.ay, 3);
    DEBUG_PORT.println(" G");

    DEBUG_PORT.print("Az: ");
    DEBUG_PORT.print(data.az, 3);
    DEBUG_PORT.println(" G");

    DEBUG_PORT.print("A Amp: ");
    DEBUG_PORT.print(data.aAmp, 3);
    DEBUG_PORT.println(" G");

    DEBUG_PORT.print("Gx: ");
    DEBUG_PORT.print(data.gx, 3);
    DEBUG_PORT.println(" grad/s");

    DEBUG_PORT.print("Gy: ");
    DEBUG_PORT.print(data.gy, 3);
    DEBUG_PORT.println(" grad/s");

    DEBUG_PORT.print("Gz: ");
    DEBUG_PORT.print(data.gz, 3);
    DEBUG_PORT.println(" grad/s");

    DEBUG_PORT.print("Mx: ");
    DEBUG_PORT.print(data.mx, 3);
    DEBUG_PORT.println(" ");

    DEBUG_PORT.print("My: ");
    DEBUG_PORT.print(data.my, 3);
    DEBUG_PORT.println(" ");

    DEBUG_PORT.print("Mz: ");
    DEBUG_PORT.print(data.mz, 3);
    DEBUG_PORT.println(" ");

    DEBUG_PORT.print("Heading: ");
    DEBUG_PORT.print(data.magHeading, 3);
    DEBUG_PORT.println(" ");

    DEBUG_PORT.print("UV Level: ");
    DEBUG_PORT.print(data.analogUV);
    DEBUG_PORT.println(" ");

    DEBUG_PORT.print("Dust Level ug/m3: ");
    DEBUG_PORT.print(data.analogDust);
    DEBUG_PORT.println(" ");

    DEBUG_PORT.print("Dust average ug/m3: ");
    DEBUG_PORT.print(data.dustRunningAverage);
    DEBUG_PORT.println(" ");

    DEBUG_PORT.println();
    DEBUG_PORT.flush();
}

void printHeaderToSD()
{
    if (sd)
    {
        debugInfo(F("Print SD header"));

        sd.println(F("------------"));
        sd.print(F("year,month,date,hours,minutes,seconds,millis,"));
        sd.print(F("latitude,longitude,altitude,speed,heading,HDOP,VDOP,satellites,"));
        sd.print(F("bmp280_temp,out_temp,sht2x_temp,humidity,pressure,"));
        sd.print(F("ax,ay,az,aAmp,gx,gy,gz,mx,my,mz,magHeading,UV,dust,dustAverage"));

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

    blackbox.internal_temp = data.sht2x_temp; // C
    blackbox.external_temp = data.external_temp;
    blackbox.humidity = data.humidity; // отн. проценты
    blackbox.pressure = data.pressure; // Па

    // Номер записанного блока. Может быть равен -1
    int8_t blackboxBlock;
    // Читаем текущее значение записанного блока
    EEPROM.get(BLACKBOX_COUNT_EEPROM_ADDRESS, blackboxBlock);

    // Преобразуем блок в стартовый адрес
    uint16_t nextWriteAddress = BLACKBOX_STRUCT_SIZE * (blackboxBlock + 1);

    // Проверим что не вышли за границы памяти EEPROM
    if (nextWriteAddress < (4000 - BLACKBOX_STRUCT_SIZE))
    {
        // Сохраним новый номер блока
        EEPROM.put(BLACKBOX_COUNT_EEPROM_ADDRESS, blackboxBlock + 1);
        // Теперь пихаем в EEPROM структуру
        EEPROM.put(nextWriteAddress, blackbox);
    }

    // Поставим таймер для следующей итерации
    timer = millis();
}

void calibrateAccelGyro()
{

    unsigned long start = millis();

    debugInfo(F("Accel/Gyro Calibrating..."));
    debugInfo(F("Your MPU9250 should be placed in horizontal position, with package letters facing up"));

    // Установка дефолтного диапазона акселерометра/гироскопа
    imu.setFullScaleAccelRange(MPU9250_ACCEL_FS_2); // 2G
    imu.setFullScaleGyroRange(MPU9250_GYRO_FS_250); // 250 град/с

    // TODO: калибровка для 2/250, а использовать надо для 8/500. Что надо изменить, чтобы это работало?

    debugInfo(F("Reset Accel and Gyro offsets"));
    // сбросить оффсеты
    imu.setXAccelOffset(0);
    imu.setYAccelOffset(0);
    imu.setZAccelOffset(0);
    imu.setXGyroOffset(0);
    imu.setYGyroOffset(0);
    imu.setZGyroOffset(0);

    delay(100); // Ждём пока датчик одумается

    // Взял отсюда https://alexgyver.ru/arduino-mpu6050/
    long offsets[6];
    long offsetsOld[6] = {0, 0, 0, 0, 0, 0}; // Важно обнулить!!!
    int16_t mpuGet[6];
    uint8_t bufferSize = 100;

    uint8_t calibrationIterations = 50;

    debugInfo(F("Calibration start. It will take about 25 seconds"));
    for (byte n = 0; n < calibrationIterations; n++)
    { // 10 итераций калибровки
        for (byte j = 0; j < 6; j++)
        { // обнуляем калибровочный массив
            offsets[j] = 0;
        }

        for (byte i = 0; i < 100 + bufferSize; i++)
        { // делаем BUFFER_SIZE измерений для усреднения
            imu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
            if (i >= 99)
            { // пропускаем первые 99 измерений (мусор)
                for (byte j = 0; j < 6; j++)
                {
                    offsets[j] += (long)mpuGet[j]; // записываем в калибровочный массив
                }
            }
        }

        for (byte i = 0; i < 6; i++)
        {
            offsets[i] = offsetsOld[i] - ((long)offsets[i] / bufferSize); // учитываем предыдущую калибровку
            if (i == 2)
                offsets[i] += 16384; // если ось Z, калибруем в 16384
            offsetsOld[i] = offsets[i];
        }
        // ставим новые оффсеты
        imu.setXAccelOffset(offsets[0] / 8);
        imu.setYAccelOffset(offsets[1] / 8);
        imu.setZAccelOffset(offsets[2] / 8);
        imu.setXGyroOffset(offsets[3] / 4);
        imu.setYGyroOffset(offsets[4] / 4);
        imu.setZGyroOffset(offsets[5] / 4);
        delay(10);

        if (n % 10 == 0)
        {
            debugInfo(".", false);
        }
    }

    // пересчитываем для хранения
    for (byte i = 0; i < 6; i++)
    {
        if (i < 3)
            offsets[i] /= 8;
        else
            offsets[i] /= 4;
    }

#ifdef DEBUG
    debugInfo(F("Calibrated offsets are: ax, ay, az, gx, gy, gz"));
    for (byte k = 0; k < 6; k++)
    {
        DEBUG_PORT.print(offsets[k]);
        DEBUG_PORT.print(',');
    }
#endif

    debugInfo(F("Save offsets to EEPROM"));
    // запись в память. Массивы тоже можно! Запишется 6*4=24 байта
    EEPROM.put(CALIBRATION_ACCEL_GYRO_EEPROM_ADDRESS, offsets);

#ifdef DEBUG
    DEBUG_PORT.println();
    debugInfo(F("Accel/Gyro Calibration took: "));
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
    DEBUG_PORT.flush();
#endif

    debugInfo(F("Infinite loop readings - check it:"));
    int16_t ax, ay, az, gx, gy, gz;
    // Бесконечно печатаем результаты измерений в консоль
    while (true)
    {
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#ifdef DEBUG
        DEBUG_PORT.println("ax ay az");
        DEBUG_PORT.print((float)ax / 16384);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.print((float)ay / 16384);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.println((float)az / 16384);
        DEBUG_PORT.println("gx gy gz");
        DEBUG_PORT.print((float)gx * 250 / 32768);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.print((float)gy * 250 / 32768);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.println((float)gz * 250 / 32768);
#endif
        delay(1000);
    }
}

void calibrateMagnetometer()
{
    // Код скомпилирован из примера
    unsigned long start = millis();

    debugInfo(F("Magnetometer Calibrating..."));

    uint16_t calibrationIterations = 1500;
    uint8_t magBuffer[6];
    int16_t mx, my, mz;
    float Mxyz[3];
    volatile float mx_sample[3];
    volatile float my_sample[3];
    volatile float mz_sample[3];

    static float mx_centre = 0;
    static float my_centre = 0;
    static float mz_centre = 0;

    volatile int mx_max = 0;
    volatile int my_max = 0;
    volatile int mz_max = 0;

    volatile int mx_min = 0;
    volatile int my_min = 0;
    volatile int mz_min = 0;

    // Это взято из из функции getMotion9(), в примере нет байпаса
    I2C_M.writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02); // set i2c bypass enable pin to true to access magnetometer
    delay(10);
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); // enable the magnetometer
    delay(10);

    debugInfo(F("Your must to rotate magnetometer at any side for 20-30 s"));
    for (unsigned int i = 0; i < calibrationIterations; i++)
    {
        I2C_M.writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02); // set i2c bypass enable pin to true to access magnetometer
        delay(10);
        I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); // enable the magnetometer
        delay(10);
        // Это копипаст из функции getMotion9()
        I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, magBuffer);

        mx = ((int16_t)(magBuffer[1]) << 8) | magBuffer[0];
        my = ((int16_t)(magBuffer[3]) << 8) | magBuffer[2];
        mz = ((int16_t)(magBuffer[5]) << 8) | magBuffer[4];

        Mxyz[0] = (double)mx * 1200 / 4096;
        Mxyz[1] = (double)my * 1200 / 4096;
        Mxyz[2] = (double)mz * 1200 / 4096;
        mx_sample[2] = Mxyz[0];
        my_sample[2] = Mxyz[1];
        mz_sample[2] = Mxyz[2];

        if (mx_sample[2] >= mx_sample[1])
            mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])
            my_sample[1] = my_sample[2]; // find max value
        if (mz_sample[2] >= mz_sample[1])
            mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])
            mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])
            my_sample[0] = my_sample[2]; // find min value
        if (mz_sample[2] <= mz_sample[0])
            mz_sample[0] = mz_sample[2];

        if (i % 100 == 0)
        {
            debugInfo(".", false);
        }
    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];

    mx_centre = (mx_max + mx_min) / 2.f;
    my_centre = (my_max + my_min) / 2.f;
    mz_centre = (mz_max + mz_min) / 2.f;

#ifdef DEBUG
    DEBUG_PORT.println();
    debugInfo(F("Calibrated offsets are: mx, my, mz"));
    DEBUG_PORT.print(mx_centre);
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(my_centre);
    DEBUG_PORT.print(',');
    DEBUG_PORT.println(mz_centre);
#endif

    magCalibration offsets = {mx_centre, my_centre, mz_centre};
    // запись в память. Массивы тоже можно! Запишется 3*4=12 байтd
    debugInfo(F("Save calibration data to EEPROM"));
    EEPROM.put(CALIBRATION_MAGNETOMETER_EEPROM_ADDRESS, offsets);

#ifdef DEBUG
    debugInfo(F("Magnetometer Calibration took: "));

    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
    DEBUG_PORT.flush();
#endif

    debugInfo(F("Infinite loop readings - check it:"));
    // Бесконечно печатаем результаты измерений в консоль
    while (true)
    {
        I2C_M.writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02); // set i2c bypass enable pin to true to access magnetometer
        delay(10);
        I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); // enable the magnetometer
        delay(10);
        // Это копипаст из функции getMotion9()
        I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, magBuffer);

        mx = ((int16_t)(magBuffer[1]) << 8) | magBuffer[0];
        my = ((int16_t)(magBuffer[3]) << 8) | magBuffer[2];
        mz = ((int16_t)(magBuffer[5]) << 8) | magBuffer[4];

        Mxyz[0] = (float)mx * 1200 / 4096 - mx_centre;
        Mxyz[1] = (float)my * 1200 / 4096 - my_centre;
        Mxyz[2] = (float)mz * 1200 / 4096 - mz_centre;

#ifdef DEBUG
        DEBUG_PORT.println("RAW - mx my mz");
        DEBUG_PORT.print(mx);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.print(my);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.println(mz);
        DEBUG_PORT.println("FIXED - Mx My Mz");
        DEBUG_PORT.print(Mxyz[0]);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.print(Mxyz[1]);
        DEBUG_PORT.print(" ");
        DEBUG_PORT.println(Mxyz[2]);
#endif
        delay(1000);
    }
}

void setAccelGyroMagOffsets()
{
    debugInfo(F("Reading offsets from EEPROM"));
    // Храним оффсеты временно в массиве
    long agOffsets[6];
    // Читаем из EEPROM
    EEPROM.get(CALIBRATION_ACCEL_GYRO_EEPROM_ADDRESS, agOffsets);
    EEPROM.get(CALIBRATION_MAGNETOMETER_EEPROM_ADDRESS, magCal);

    debugInfo(F("Set offsets for IMU..."));
    // Установка оффсетов IMU в память сенсора
    // Оффсеты магнитометра нельзя залить в память сенсора.
    imu.setXAccelOffset(agOffsets[0]);
    imu.setYAccelOffset(agOffsets[1]);
    imu.setZAccelOffset(agOffsets[2]);
    imu.setXGyroOffset(agOffsets[3]);
    imu.setYGyroOffset(agOffsets[4]);
    imu.setZGyroOffset(agOffsets[5]);

#ifdef DEBUG
    debugInfo(F("Offsets: ax,ay,az,gx,gy,gz"));
    for (long o : agOffsets)
    {
        DEBUG_PORT.print(o);
        DEBUG_PORT.print(',');
    }

    DEBUG_PORT.println();
    debugInfo(F("Offsets: mx,my,mz"));
    DEBUG_PORT.print(magCal.mx);
    DEBUG_PORT.print(',');
    DEBUG_PORT.print(magCal.my);
    DEBUG_PORT.print(',');
    DEBUG_PORT.println(magCal.mz);

#endif
}

void saveMPU9250(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    // Сырые данные с датчика
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    // Обработанные данные с магнетометра
    float c_mx, c_my, c_mz;
    // Данные указания на север прямо
    float heading;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // Преобразуем данные и записываем в структуру
    // Оффсеты уже применены датчиком
    data.ax = (float)ax / 4096; // Для 8 G и 500 град/с, см тут! https://mjwhite8119.github.io/Robots/mpu6050
    data.ay = (float)ay / 4096;
    data.az = (float)az / 4096;

    data.aAmp = sqrtf(data.ax * data.ax + data.ay * data.ay + data.az * data.az);

    data.gx = (float)gx * 500 / 32768;
    data.gy = (float)gy * 500 / 32768;
    data.gz = (float)gz * 500 / 32768;

    // Обрабатываем и корректируем данные магнитометра с помощью оффсетов
    c_mx = (float)mx * 1200 / 4096 - magCal.mx;
    c_my = (float)my * 1200 / 4096 - magCal.my;
    c_mz = (float)mz * 1200 / 4096 - magCal.mz;

    data.mx = c_mx;
    data.my = c_my;
    data.mz = c_mz;

    // Направления взяты из примеров
    heading = 180 * atan2(c_my, c_mx) / PI;
    if (heading < 0)
        heading += 360;

    data.magHeading = heading;

#ifdef DEBUG
    debugInfo("MAG HEADING: ", false);
    DEBUG_PORT.println(heading);

    debugInfo(F("Get MPU9250 AGM data took: "));
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
    DEBUG_PORT.flush();
#endif
}

void resetPinModes()
{
    // Режим пинов - вывод
    pinMode(SUCCESS_POWER_PIN, OUTPUT);
    pinMode(SUCCESS_GPS_PIN, OUTPUT);
    pinMode(SUCCESS_BMP280_PIN, OUTPUT);
    pinMode(SUCCESS_DS18B20_PIN, OUTPUT);
    pinMode(SUCCESS_MPU9250_PIN, OUTPUT);
    pinMode(SUCCESS_SD_PIN, OUTPUT);
    pinMode(SUCCESS_SHT2X_PIN, OUTPUT);
    pinMode(DUST_SENSOR_LED_PIN, OUTPUT);

    pinMode(RELAY_1_PIN, OUTPUT);
    pinMode(RELAY_2_PIN, OUTPUT);

    // Выключаем зелёные
    LED_OFF(SUCCESS_POWER_PIN);
    LED_OFF(SUCCESS_GPS_PIN);
    LED_OFF(SUCCESS_BMP280_PIN);
    LED_OFF(SUCCESS_DS18B20_PIN);
    LED_OFF(SUCCESS_MPU9250_PIN);
    LED_OFF(SUCCESS_SD_PIN);
    LED_OFF(SUCCESS_SHT2X_PIN);
    LED_OFF(DUST_SENSOR_LED_PIN);

    // Выключить реле
    LED_OFF(RELAY_1_PIN);
    LED_OFF(RELAY_2_PIN);
}

void eepromBlackboxReset()
{
    debugInfo(F("RESET EEPROM BLACKBOX COUNTER"));
    EEPROM.put(BLACKBOX_COUNT_EEPROM_ADDRESS, -1);
    EEPROM.put(BLACKBOX_SIZE_EEPROM_ADDRESS, BLACKBOX_STRUCT_SIZE);

    while (true)
    {
        debugInfo(".");
        delay(1000);
    }
}

void saveSHT2x(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    //  Датчик тут?
    if (sht2x.isConnected())
    {
        debugInfo(F("Reads SHT-21 inside temperature and humidity"));
        sht2x.read(); // Запрос на чтение данных
        if (sht2x.getError() != 0)
        {
            sht2x.reset(); // Сброс при ошибках
        }

        sht2x.read();
        // При ошибках сырые данные = 0, игнорим их
        if (sht2x.getRawTemperature() != 0)
        {
            data.sht2x_temp = sht2x.getTemperature(); // Сохраняем данные
        }

        if (sht2x.getRawHumidity() != 0)
        {
            data.humidity = sht2x.getHumidity(); // Сохраняем данные
        }
    }

#ifdef DEBUG
    DEBUG_PORT.print(F("Save SHT-21 took: "));
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif
}

void saveAnalogUV(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    // Нет возможности проверить что сенсор УФ доступен, поскольку это просто напряжение
    data.analogUV = analogRead(UV_SENSOR_PIN); // 0 - 1023

#ifdef DEBUG
    DEBUG_PORT.print(F("Save Analog UV took: "));
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif
}

void relay()
{
    // Включаем реле по интервалу
    if (((millis() - relayOnTimer) > RELAY_SWITCHON_TIMEOUT_S * 1000UL) && !relayON)
    {
        debugInfo(F("Switch ON relays"));

        relayOffTimer = millis();
        relayON = true;

        LED_ON(RELAY_1_PIN);
        LED_ON(RELAY_2_PIN);
    }

    // Выключаем реле по интервалу
    if (((millis() - relayOffTimer) > RELAY_SWITCHOFF_TIMEOUT_S * 1000UL) && relayON)
    {
        debugInfo(F("Switch OFF relays"));

        relayOnTimer = millis();
        relayON = false;
        LED_OFF(RELAY_1_PIN);
        LED_OFF(RELAY_2_PIN);
    }
}

void saveAnalogDust(MP_Data &data)
{
#ifdef DEBUG
    unsigned long start = millis();
#endif

    // Нет возможности проверить что сенсор пыли доступен, поскольку это просто напряжение
    data.analogDust = dustSensor.getDustDensity(20U); // 0 - 600
    data.dustRunningAverage = dustSensor.getRunningAverage();


#ifdef DEBUG
    DEBUG_PORT.println(analogRead(DUST_SENSOR_PIN));
    DEBUG_PORT.print(F("Save Analog Dust took: "));
    DEBUG_PORT.print(millis() - start);
    DEBUG_PORT.println(" ms");
#endif
}