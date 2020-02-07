#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <BH1750FVI.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <MPU9250.h>


#define LED0 PB0
#define LED1 PB1
#define gpsSerial Serial1
char filename[16];
TinyGPS gpsDecoder;
Adafruit_BME280 BME0; 
Adafruit_BME280 BME1;
MPU9250 IMU(Wire,0x68);
BH1750FVI LightSensor;

static void gpsdump(File& fd);
static void printValues();
static void setGPSFlightMode();

// Arduino tool does not generate forward declarations of template functions properly.
// An explicit forward declaration preceding all executable code stops Arduino from these attempts. 
template <typename T> static void printCSV(File& fd, T x);
template <typename Out> static void printFloat(Out& out, double number, int digits);

template <typename T> static void printCSV(File& fd, T x) {
    fd.print(x);
    fd.print(";");
}

static void printCSV(File& fd, float x, int digits) {
    printFloat(fd, x, digits);
    fd.print(";");
}
static void printCSVfloat(File& fd, float x) {
    fd.print(x);
    fd.print(";");
}
static void printCSVint(File& fd, int x) {
    fd.print(x);
    fd.print(";");
}
static void printCSVuint16_t(File& fd, uint16_t x) {
    fd.print(x);
    fd.print(";");
}
static void printCSVchar(File& fd, char* x) {
    fd.print(x);
    fd.print(";");
}

template <typename Out> static void printFloat(Out& out, double number, int digits) {
  // Handle negative numbers
    if (number < 0.0) {
        out.print('-');
        number = - number;
    }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
      rounding /= 10.0;
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  out.print(int_part);
  if (digits > 0) {  
    out.print("."); 
  }
  
  // Extract digits from the remainder one at a time
      while (digits-- > 0) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        out.print(toPrint);  
    }
}

static void generateFilename() {
    unsigned n = 0;
    do {
        snprintf(filename, sizeof(filename), "data%03d.csv", n); // includes a three-digit sequence number in the file name
        ++n;
    }
        while(SD.exists(filename));
        Serial.print("Writing to ");
        Serial.println(filename);
}

static void Runtime(File& fd) { 
  unsigned long runtime = millis();
  Serial.print("Runtime = ");
  Serial.println(runtime); 
  fd.print(runtime);
  fd.print(";");
}

static void hang() {
    Serial.println("System halted.");
    LED0_on();
    LED1_on();
    while (true);
}

static void LED0_on(){
    digitalWrite(LED0, 0);  
}
static void LED1_on(){
    digitalWrite(LED1, 0);
}

static void LED0_off(){
  digitalWrite(LED0, 1);
}
static void LED1_off(){
  digitalWrite(LED1, 1);
}

//////////////  BME 0 & 1 ///// Start ////////////////

static void printBMEValue(uint8_t bmeIdx, const char *name, float value, const char *unit) {
    Serial.print(name);
    Serial.print("_BME_");
    Serial.print(bmeIdx);
    Serial.print(" = ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(unit);  
}


static void printBME(File& fd, Adafruit_BME280& bme, uint8_t bmeIdx) {
    float temperature = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0f;
    float humidity = bme.readHumidity();

    printBMEValue(bmeIdx, "Temperature", temperature, "*C");
    printBMEValue(bmeIdx, "Pressure", pressure, "hPa");
    printBMEValue(bmeIdx, "Humidity", humidity, "%");

    printCSV(fd, temperature);
    printCSV(fd, pressure);
    printCSV(fd, humidity); 
}

//////////////  BME 0 & 1 ///// End ////////////////
//////////////  IMU ///// Start ////////////////

static void printIMUValue(const char* name, float value, const char *unit) {
    Serial.print(name);
    Serial.print(" ");
    Serial.print("IMU");
    Serial.print(" = ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(unit);  
}

static void printIMU(File& fd, MPU9250& IMU) {
    IMU.readSensor();
    float ax = IMU.getAccelX_mss();
    float ay = IMU.getAccelY_mss();
    float az = IMU.getAccelZ_mss();

    float rx = IMU.getGyroX_rads();
    float ry = IMU.getGyroY_rads();
    float rz = IMU.getGyroZ_rads();

    float bx = IMU.getMagX_uT();
    float by = IMU.getMagY_uT();
    float bz = IMU.getMagZ_uT();

    printIMUValue("AccelX", ax, "mss");
    printIMUValue("AccelY", ay, "mss");
    printIMUValue("AccelZ", az, "mss");

    printIMUValue("GyroX", rx, "rads");    
    printIMUValue("GyroY", ry, "rads");
    printIMUValue("GyroZ", rz, "rads");

    printIMUValue("MagX", bx, "uT");
    printIMUValue("MagY", by, "uT");
    printIMUValue("MagZ", bz, "uT");
            
    printCSV(fd, ax);
    printCSV(fd, ay);
    printCSV(fd, az);
    
    printCSV(fd, rx);
    printCSV(fd, ry);
    printCSV(fd, rz);
    
    printCSV(fd, bx);
    printCSV(fd, by);
    printCSV(fd, bz); 
}
//////////////  IMU ///// End ////////////////

//////////////  Light Sensor ///// Start ////////////////
static void printBH1750Value(const char *name, uint16_t value, const char *unit) {
    Serial.print(name);
    Serial.print(" ");
    Serial.print("BH1750");
    Serial.print(" = ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(unit);  
}

static void printBH1750(File& fd) {
    uint16_t lux = LightSensor.GetLightIntensity();

    printBH1750Value("Light", lux, "lx");
    printCSVuint16_t(fd, lux);
}
//////////////  Light Sensor ///// End ////////////////
static void printLightValue(const char *name, int value) {
    Serial.print(name);
    Serial.print(" ");
    Serial.print("Light");
    Serial.print(" = ");
    Serial.print(value);
      
}

static void Light(File& fd){
  int Light = analogRead(A1);

  printLightValue("Light2", Light);
  printCSVint(fd, Light);
}

//////////////  GPS Data ///// Start ////////////////

static bool pollGPS() {
   const unsigned long timeout = 5000; // milliseconds
   bool newdata = false;
   
   unsigned long start = millis();
   while (millis() - start < timeout) {
       if (gpsSerial.available()) {
           char c = gpsSerial.read();
           // Serial.print(c);  // uncomment to see raw GPS data
           if (gpsDecoder.encode(c)) {
               newdata = true;
               break;  // uncomment to print new data immediately!
           }
       }
   }
   
   return newdata;
}

static void gpsdump(File& fd) {
    unsigned long age = 0;
    int year = 0;
    byte month = 0;
    byte day = 0;
    byte hour = 0;
    byte minute = 0;
    byte second = 0;
    byte hundredths = 0;
    gpsDecoder.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

    float flat = 0;
    float flon = 0;
    gpsDecoder.f_get_position(&flat, &flon, &age);

    float altitude = gpsDecoder.f_altitude();
    float speed = gpsDecoder.f_speed_kmph(); 
    unsigned satellites = gpsDecoder.satellites();

    char date[11] = { 0 };
    sprintf(date, "%04d/%02d/%02d", year, month, day);
    char time[12] = { 0 };
    sprintf(time, "%02d:%02d:%02d.%02d",
        (hour + 1), /* UTC +01:00 Europe/Berlin */
        minute, second, hundredths
    ); 

    Serial.print("Date: ");
    Serial.println(date);
    Serial.print("Time: ");
    Serial.println(time);

    Serial.print("Lat/Long(float): ");
    printFloat(Serial, flat, 5);
    Serial.print(", ");
    printFloat(Serial, flon, 5);
    Serial.print(" Alt(float): ");
    printFloat(Serial, altitude, 2);
    Serial.print(" kmph: ");
    printFloat(Serial, speed, 2);      
    Serial.print(" Satellites: ");
    if (satellites != TinyGPS::GPS_INVALID_SATELLITES) { 
        Serial.println(satellites);
    } else {
        Serial.println("unknown");
    }
    printCSVchar(fd, date);
    printCSVchar(fd, time);
    printCSV(fd, flat, 5);
    printCSV(fd, flon, 5);  
    printCSV(fd, altitude, 2);
    printCSV(fd, speed, 2);
    fd.print(satellites);
}

//////////////  GPS Data ///// End ////////////////

//////////////  Flight Mode ///// Start ///////////
static void ubxFinalize(byte *data, uint8_t size) {
    data[0] = 0xB5;
    data[1] = 0x62;
    data[4] = size - 8;

    byte ck1 = 0;
    byte ck2 = 0;
    for (uint8_t i = 2; i < size - 2; ++i) {
        ck1 += data[i];
        ck2 += ck1;
    }

    data[size - 2] = ck1;
    data[size - 1] = ck2;
}

static void ubxIO(byte *data, uint8_t size) {
    ubxFinalize(data, size);
    byte ack[] = {
        0xB5, 0x62,
        0x05, 0x01, // ack
        2, 0,
        data[2], data[3],
        0, 0
    };
    ubxFinalize(ack, sizeof(ack));

    for (uint8_t attempt = 0; attempt < 10; ++attempt) {
        gpsSerial.flush();
        gpsSerial.write(0xFF);
        delay(500);
        gpsSerial.write(data, size);

        unsigned long start = millis();
        uint8_t pos = 0;
        const unsigned long TIMEOUT = 3000;
        while (millis() - start < TIMEOUT) {
            if (gpsSerial.available()) {
                byte b = gpsSerial.read();
                if (b == ack[pos]) {
                    ++pos;
                    if (pos >= sizeof(ack))
                        return;
                } else {
                    pos = 0;
                }
            }
        }
    }
    Serial.println("Init error - No reply from GPS module");
    hang();
}

static void setGPSFlightMode() {
    byte ubx[] = {
        0xB5, 0x62,
        0x06, // CFG 
        0x24, // CFG-NAV5, see page 119 of the datasheet 
        36, 0, // size etc.
        0xFF, 0xFF, // set all
        
        6, // Airborne with <1g acceleration
        3, // Auto 2D/3D
        0, 0, 0, 0, // fixed altitude for 2D mode, meter * 0.01
        0x10, 0x27, 0x00, 0x00, // 0x00002710 fixed altitude variance, meter * 0.0001
        5, // Minimum satellite elevation, degrees
        0, // Maximum time to perform dead reckoning
        0xFA, 0x00, // 0x00FA Position DOP mask
        0xFA, 0x00, // 0x00FA Time DOP mask
        0x64, 0x00, // 0x0064 Position accuracy mask        
        0x2C, 0x01, // 0x012C Time accuracy mask
        0, // Static hold threshold, cm/s
        0, // DGPS timeout, seconds  
        
        // reserved, always 0
        0x00, 0x00, 0x00, 0x00,         
        0x00, 0x00, 0x00, 0x00,         
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00
    };
    ubxIO(ubx, sizeof(ubx));
    Serial.println("GPS switched to flight mode");
}
//////////////  Flight Mode ///// End ///////////

void setup() {
    pinMode (LED0, OUTPUT);
    pinMode (LED1, OUTPUT);
    pinMode (A1, INPUT);

    Serial.begin(9600);
    gpsSerial.begin(9600);
    SD.begin(PB4);
    
    Serial.println(F("BME280 test"));
    generateFilename();
    
    {
        File fd = SD.open(filename, FILE_WRITE);
        fd.println(
            "T_BME280_0_C;p_BME280_0_hPa;rH_BME280_0_%;"
            "T_BME280_1_C;p_BME280_1_hPa;rH_BME280_1_%;"
            "AccelX_mss;AccelY_mss;AccelZ_mss;"
            "GyroX_rads;GyroY_rads;GyroZ_rads;"
            "MagX_uT;MagY_uT;MagZ_uT;"
            "lux_lx;"
            "Light;"
            "millis_ms;"
            "Jahr/Monat/Tag;Stunden:Minuten:Sekunden;"
            "f_lat;f_lon;f_altitude;f_kmph;Sateliten"
        );
        fd.close();
    }

    {
        bool bme0Status = BME0.begin();  
        bool bme1Status = BME1.begin(0x77);
        int statusIMU = IMU.begin();
        LightSensor.begin();
        
        // Show BMEs on the serial monitor
        if (!bme0Status) {
            Serial.println("Could not find a valid BME280 Nr. 0 sensor, check wiring!");
            hang();
        }
        if (!bme1Status) {
            Serial.println("Could not find a valid BME280 Nr. 1 sensor, check wiring!");
            hang();
        }
        if (statusIMU < 0) {
            Serial.println("IMU initialization unsuccessful");
            hang();
        }
    }

    setGPSFlightMode();
    Serial.println("-- Default Test --");
} // END setup

void loop() { 
    LED0_on();
    bool newdata = pollGPS();
    LED0_off();
    // on = DigitalWrite(0), off = DigitalWrite(1)
    
    File fd = SD.open(filename, FILE_WRITE);    
    printBME(fd, BME0, 0);
    printBME(fd, BME1, 1);
    printIMU(fd, IMU);
    printBH1750(fd);
    Light(fd);
    Runtime(fd);
   
    if (newdata) {
        LED1_on();
        Serial.println("Acquired GPS Data");
        Serial.println("-------------");
        gpsdump(fd);
        Serial.println("-------------");
        Serial.println();
    } else {
        Serial.println("No GPS data available");
    }
    
    fd.println("");
    fd.close();
    LED1_off();
} 
