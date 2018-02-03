/******V2.2******/
/***Umais Khan***/
/***27-07-2017***/

/*Lib*/
#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
/*Lib END*/

/*Serial Setup*/
Uart Serial2(&sercom2, 3, 4, SERCOM_RX_PAD_1, UART_TX_PAD_0); //initialising software serial on pin 3 and 4

#define ss Serial2                               //Setting up software serial used for GPS module
#define gprsSerial Serial5                       //Setting up software serial used for GPRS module

void SERCOM2_Handler() { Serial2.IrqHandler(); }
/*Serial Setup END*/

/*LCD*/
LiquidCrystal_I2C lcd(0x27, 20, 4);             //Seting up the LCD (HEX is specific for the LCD type)
/*LCD END*/

/*Baud Var's/ Setup Var's*/
static const uint32_t SERIAL_BAUD = 115200;     //Setup baud rate for Arduino serial
static const uint32_t GPS_BAUD = 9600;          //Setup baudrate for GPS module
static const uint32_t GPRS_BAUD = 19200;        //Setup baudrate for GPRS/GSM module

static int SETUP_DELAY = 5000;                  //This delay allows time for the GPRS sheild to boot
static bool DISABLE_GPRS_SERIAL = false;        //Toggle GPRS serial output
static bool DISABLE_GPS_PRINT_DATA = false;     //Toggle GPS nema serial printout
static bool DISABLE_LCD = false;                //Toggle LCD on/off
static bool DISABLE_V_STOP_CHECK = false;

int p1 = 0;                                     //Make sure the string above is only printed once (GPRS)
int p2 = 0;                                     //Make sure the string above is only printed once (GPS)
/*Baud Var/ Setup Var's END*/

/*Other Var's*/
int count = 0;

TinyGPSPlus gps; //Creating GPS Object

/*
*lo = Longitude 
*la = Latitude 
*speed_mph = Speed in mph
*speed_kmph = Speed in kmph
*alt_feet = Altitude in feet
*sat_count = Shows the number of satellites connected 
*/

double lo, la, speed_mph, speed_kmph, alt_feet, temp, hum;
uint32_t sat_count;

/*Other Var's END*/


/*Vehicle stop/ start*/
bool v_stopped = false;                 //Bool set based on in vehicle is moving or stationary
int stop_count = 0;                     //Counts the number of loops passed while vehicle is stopped
double min_speed = 1.5;                 //Set speed that the vehicle must reach before the tracker sends data (mph)
/*End vehicle stop/ start*/

/*USERNAME*/

static String user = "Sufi";

double max_speed = 70.0;                //If vehicle travels faster than this speed the App user is notified (float/mph)
static String reg = "VF17 AEY";
bool speed_alert = true;                //SMS speed alerts only sent if this function is set to true

bool sms_delay = false;
int sms_delay_count = 0;
/*Vehicle details*/

/*End of vehicle details*/

/*DHT11  Var's*/
#define DHTPIN            4         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT11     // DHT 11 
DHT_Unified dht(DHTPIN, DHTTYPE);

//float temp, hum;
uint32_t delayMS;

/*END of DHT11*/

void setup() {

    delay(SETUP_DELAY);
    delay(500);

    lcd.init();                            //Initialised LCD line 1 and 2
    lcd.init();
    lcd.backlight();                      //Start LCD backlight

    lcd.clear();
    lcd.print("Project Snowy");
    lcd.setCursor(0, 1);
    lcd.print("Loading...");

    //Asign baud rate to GPS, GPRS/GSM and Serial monitor
    Serial.begin(SERIAL_BAUD);
    ss.begin(GPS_BAUD);
    gprsSerial.begin(GPRS_BAUD);

    // Assign pins 3 & 4 SERCOM functionality
    pinPeripheral(3, PIO_SERCOM_ALT);
    pinPeripheral(4, PIO_SERCOM_ALT);

    gprsSerial.flush();
    smartDelay(700);//2
    gprsSerial.println("AT+CGATT?");
    smartDelay(350);//1
    toSerial();

    gprsSerial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");     //Setting up connection type (GPRS)
    smartDelay(700); //2
    toSerial();
    gprsSerial.println("AT+SAPBR=3,1,\"APN\",\"giffgaff.com\""); //Setting up APN
    smartDelay(700); //2
    gprsSerial.println("AT+SAPBR=1,1");
    smartDelay(700); //2
    toSerial();
    /*DHT11*/
    dht.begin();
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.println("Temperature");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" *C");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" *C");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" *C");
    Serial.println("------------------------------------");
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.println("Humidity");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println("%");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println("%");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println("%");
    Serial.println("------------------------------------");
    // Set delay between sensor readings based on sensor details.
    delayMS = sensor.min_delay / 1000;
    /*DHT11 END*/
}

void loop() {
    lo = gps.location.lng();
    la = gps.location.lat();

    dht_data();
    Serial.println(temp);
    Serial.println(hum);
    speed_mph = gps.speed.mph();
    speed_kmph = gps.speed.kmph();
    alt_feet = gps.altitude.feet();
    sat_count = gps.satellites.value();

    printGPSData(); //Un comment for serial debugging
    toLCD(gps.location.isValid());

    if (DISABLE_LCD != true) {
        toLCD(gps.location.isValid());
    }

    /*
    *The code below is designed to stop the transmittion of GPS data via GPRS
    *when the vehicle is stationary...
    */
    if (DISABLE_V_STOP_CHECK == false) {
        if (speed_mph < min_speed) {

            stop_count++;
            smartDelay(500);
            Serial.print(" \n\nVehicle stopped! Count = ");
            Serial.println(stop_count);

        }
    }


    if (v_stopped == false && stop_count < 2) {
        Serial.print(" \n\nSending to DB = ");
        Serial.println(stop_count);
        toDB();
    }

    if (stop_count >= 2 && v_stopped == false) {
        Serial.print(" \n\nSetting v_stopped to true! Count = ");
        Serial.println(stop_count);
        v_stopped = true;
    }

    if (v_stopped == true && speed_mph >= min_speed && speed_alert == true) {

        v_stopped = false;
        stop_count = 0;
        Serial.print(" \n\nv_stopped set to false and count reset! Count = ");
        Serial.println(stop_count);
        toDB();
    }

    /*
    *This if statement waits for 10 loops before setting the sms_delay function to false
    *if the function is set to true the system would send a speed warning notification
    *every 6-7 seconds if the user maintains a speed higher than the set maximum limit
    */
    if (sms_delay == true) {

        sms_delay_count++;

        if (sms_delay_count >= 10) {

            sms_delay = false;
            sms_delay_count = 0;

        }

    }

    /*
     * If speed is greater than the max speed limit set and the sms_delay_count
     * is equal to 0 a SMS message is sent to the user if the driver of the vehicle
     * exceeds the maximum set limit
     */
    if (speed_mph > max_speed && sms_delay_count == 0) {

        sms_notification();
        sms_delay = true;
        sms_delay_count++;

    }

    count++;

}

void dht_data() {
    // Delay between measurements.
    delay(delayMS);
    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println("Error reading temperature!");
    } else {
        Serial.print("Temperature: ");
        temp = event.temperature;
        Serial.println(" *C");
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println("Error reading humidity!");
    } else {
        Serial.print("Humidity: ");
        hum = event.relative_humidity;
        Serial.println("%");
    }
}

void printGPSData() {
    if (DISABLE_GPS_PRINT_DATA == false) {
        Serial.print("\n\n*** Data In - Count = ");
        Serial.print(count);
        Serial.println(" ***");
        Serial.print("\nLat: ");
        Serial.print(la, 6);
        Serial.print("\nLong: ");
        Serial.print(lo, 6);

        Serial.println("\n\n*** END *** \n\n");
        smartDelay(100);
    } else if (DISABLE_GPS_PRINT_DATA == true && p1 == 0) {
        Serial.println("GPS data printing to serial has been disabled!");
        p1++;
    }
}


/*
*Prints GPS data to lcd GPS data validity boolean must be passed to this function
*If bool is set to valid the data is printed to the LCD and if set to false no a
*string of '*' are printed in place of latitude and longitude data.
*/
void toLCD(bool valid) {
    if (!valid) {
        lcd.clear();
        lcd.print("Lat: ****** MS");
        lcd.print(max_speed);
        lcd.setCursor(0, 1);
        lcd.print("Lon: ******");
    } else {
        lcd.clear();
        lcd.print("Lat: ");
        lcd.print(la, 6);
        lcd.setCursor(0, 1);
        lcd.print("Lon: ");
        lcd.print(lo, 6);
    }
}

/*This function when executed sends data to the database via GPRS*/
void toDB() {
    gprsSerial.flush();
    smartDelay(1000);
    gprsSerial.println("AT+HTTPINIT");
    smartDelay(1000);

    gprsSerial.print("AT+HTTPPARA=\"URL\",\"http://www.projectsnowy.co.uk/include/php/gps_log.php?la=");
    smartDelay(50);
    gprsSerial.print(la, 6);
    smartDelay(200);
    gprsSerial.print("&lo=");
    smartDelay(50);
    gprsSerial.print(lo, 6);
    smartDelay(200);
    gprsSerial.print("&speed_mph=");
    smartDelay(50);
    gprsSerial.print(speed_mph);
    smartDelay(200);
    gprsSerial.print("&speed_kmph=");
    smartDelay(50);
    gprsSerial.print(speed_kmph);
    smartDelay(200);
    gprsSerial.print("&alt_feet=");
    smartDelay(50);
    gprsSerial.print(alt_feet);
    smartDelay(200);
    gprsSerial.print("&sat_count=");
    smartDelay(50);
    gprsSerial.print(sat_count);
    smartDelay(200);
    gprsSerial.print("&temp=");
    smartDelay(50);
    gprsSerial.print(temp);
    smartDelay(200);
    gprsSerial.print("&hum=");
    smartDelay(50);
    gprsSerial.print(hum);
    smartDelay(200);
    gprsSerial.print("&user=");
    smartDelay(50);
    gprsSerial.print(user);
    smartDelay(200);
    gprsSerial.print("&reg=");
    smartDelay(50);
    gprsSerial.print(reg);
    smartDelay(200);
    gprsSerial.print("\"\r\n"); // close url

    smartDelay(1000);
    toSerial();

    // set http action type 0 = GET, 1 = POST, 2 = HEAD
    gprsSerial.println("AT+HTTPACTION=0");
    smartDelay(1000);
    toSerial();

    // read server response
    gprsSerial.println("AT+HTTPREAD");
    smartDelay(1000);
    toSerial();

    gprsSerial.println("");
    gprsSerial.println("AT+HTTPTERM");
    toSerial();
    smartDelay(1000);

    gprsSerial.println("");
    smartDelay(500);
}

/*
*  This function sends the vehicles current speed and reg
* as a SMS notification if the driver exceeds the set max speed
*/
void sms_notification() {

    Serial.println("Sending text!");
    gprsSerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
    delay(100);
    gprsSerial.println("AT + CMGS = \"+447582197155\"");
    delay(100);
    //the content of the message
    gprsSerial.print("The vehicle reg: ");
    gprsSerial.println(reg);
    gprsSerial.print("Is exceeding the max set speed limit. Current speed = ");
    gprsSerial.println(speed_mph);

    gprsSerial.print("Current set speed = ");
    gprsSerial.print(max_speed);
    gprsSerial.print(" Overspeed by:  ");
    gprsSerial.println(speed_mph - max_speed);
    delay(100);
    gprsSerial.println((char) 26);//the ASCII code of the ctrl+z is 26
    delay(100);
    gprsSerial.println();

    if (gprsSerial.available())
        Serial.write(gprsSerial.read());

}

/****************************************************************Other functions!**********************************************/

static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (ss.available())
            gps.encode(ss.read());
    } while (millis() - start < ms);
}

void toSerial() {
    while (gprsSerial.available() != 0 && DISABLE_GPRS_SERIAL == false) {
        Serial.write(gprsSerial.read());
    }

    if (DISABLE_GPRS_SERIAL == true && p2 == 0) {
        Serial.println("GPRS Serial has been disabled!");
        p2++;
    }
}
