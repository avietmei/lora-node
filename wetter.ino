// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic


#include <lmic.h>
#include <hal/hal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h> 

#ifdef CREDENTIALS
static const u1_t NWKSKEY[16] = NWKSKEY1;
static const u1_t APPSKEY[16] = APPSKEY1;
static const u4_t DEVADDR = DEVADDR1;
#else
static const u1_t NWKSKEY[16] = { 0x6F, 0xFC, 0xD5, 0xFF, 0xB0, 0xBD, 0x16, 0xAB, 0xED, 0xA0, 0xAB, 0x48, 0xAD, 0x26, 0x1B, 0x0F };
static const u1_t APPSKEY[16] = { 0xC5, 0x98, 0x15, 0xDC, 0x9B, 0xFB, 0x26, 0x07, 0xD7, 0x83, 0x69, 0x81, 0x57, 0x56, 0xB5, 0xE4 };
static const u4_t DEVADDR = 0x260113C5;
#endif

// byte message to send
// byte 1-2 > Temperature
// byte 3-4 > Wind Speed
// byte 5-6 > Wind Direction
 static uint8_t message[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
 unsigned long time = 0;
// Temp wire is plugged into port 4 on the Arduino
#define ONE_WIRE_BUS 4

// Temp Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Remp:Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


// WindSpeed variables 

#define WindSensorPin (3) // The pin location of the anemometer sensor 
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine 
float WindSpeedMiles; // speed miles per hour 
const int m_time = 5;      //Meassuretime in Seconds

//WindDirection variables
int VaneValue;// raw analog value from wind vane 
int Direction;// translated 0 - 360 direction 
int CalDirection;// converted value with offset applied 
#define Offset 180; 

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300;

// Pin mapping Dragino Shield
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};
void onEvent (ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    }
}

void do_send(osjob_t* j){
    // Payload to send (uplink)
    
    get_temp();
    get_WindDirection();
    get_WindSpeed();
  //Print message array
      for (int i=0; i<sizeof(message); i++)
  {
      Serial.print("Byte");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(message[i]);
  }
    
    

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        //        Serial.println(message);
        LMIC_setTxData2(1, message, sizeof(message)-1, 0);
        Serial.println(F("Sending uplink packet..."));
    }
    // Next TX is scheduled after TX_COMPLETE event.


}

void get_temp () {
  //float temperature = 0;
  float SensorTemp = 0; //Daten vom Sensor
  int offsetTemp = 100; //Offset. Benötigt um im + Bereich zu bleiben
  //float humidity = 0;

  //DS18B20 Temperatur Sensor
  sensors.begin();  
  sensors.requestTemperatures(); // Send the command to get temperatures
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  SensorTemp = sensors.getTempCByIndex(0);
  Serial.print("Temperatur: "); 
  Serial.println(SensorTemp);
  //Offset hinzurechnen um aus dem Minus bereich zu kommen
  // und mal 10 um das Komma zu verschieben
  int payloadTemp = (SensorTemp + offsetTemp)*10; 

  //payloadTemp in bytes zerlegen.
  message[0] = highByte(payloadTemp);
  message[1] = lowByte(payloadTemp);

}


void get_WindDirection() {
  
VaneValue = analogRead(A4); 
Direction = map(VaneValue, 0, 1023, 0, 360); 
CalDirection = Direction + Offset; 

if(CalDirection > 360) 
CalDirection = CalDirection - 360; 

if(CalDirection < 0) 
CalDirection = CalDirection + 360; 

message[4] = highByte(CalDirection);
message[5] = lowByte(CalDirection);
Serial.print("Windrichtung: ");Serial.println(CalDirection);
} 


// This is the function that the interrupt calls to increment the rotation count 
void isr_rotation () { 

if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact. 
Rotations++; 
ContactBounceTime = millis(); 
} 
}


void get_WindSpeed() { 

Rotations = 0; // Set Rotations count to 0 ready for calculations 

sei(); // Enables interrupts 
delay(1000 * m_time);
cli(); // Disable interrupts 

// convert to mp/h using the formula V=P(2.25/T) 
// V = P(2.25/3) = P * 0.75 

WindSpeedMiles = Rotations * 0.75; 
int WindSpeedKilometers = WindSpeedMiles * 100 * 1.60934;


message[2] = highByte(WindSpeedKilometers);
message[3] = lowByte(WindSpeedKilometers);
Serial.print("Windgeschwindigkeit: ");Serial.print(WindSpeedMiles);Serial.println(" mp/h");
Serial.print("Windgeschwindigkeit: ");Serial.print(WindSpeedKilometers);Serial.println(" km/h");
Serial.print("Windgeschwindigkeit: ");Serial.print(Rotations);Serial.println(" Umdrehungen");

} 
void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting..."));
    time = millis();

    //WindSpeed init
    pinMode(WindSensorPin, INPUT); 
    attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 


    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF12,14);

    // Start job
    do_send(&sendjob);
}




void loop() {
  
    os_runloop_once();
//    get_WindSpeed();
//    int WindSpeedDelay = (TX_INTERVAL * 1000) /5;
//    delay(WindSpeedDelay);
}
