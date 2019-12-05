/* --------------------------- STANDARD LIBRARIES --------------------------- */
/* --------------------------- DO NOT REMOVE !!!! --------------------------- */

#include <Arduino.h>
#include <WiFi.h>

/* -------------------------- ADDITIONAL LIBRARIES -------------------------- */

#include <heltec.h>
#include <AccelStepper.h>
#include <StringSplitter.h>

// #include <SparkFunMPU9250-DMP.h>
// #include <MPU6050_6Axis_MotionApps_V6_12.h>
// #include <MPU9250.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BAUDRATE 115200

/* ----------------------------- WIFI DEFINITION ---------------------------- */

static const String SSID = "ROB";
static const String PSWD = "secure106erb";

/* ---------------------------- MOTOR DEFINITION ---------------------------- */

// static const int R_DIR = 12;
// static const int R_PUL = 13;
static const int Y_DIR = 26;
static const int Y_PUL = 27;
static const int P_DIR = 32;
static const int P_PUL = 33;

static const int motorSpeed = 3200;
static const int motorAccel = 8000;

// static const int motionThreshold = 5;

bool zeroed = false;

int initPitch = 0;
int initYaw   = 0;

// AccelStepper R( AccelStepper::DRIVER, R_PUL, R_DIR ); // ROLL
AccelStepper P( AccelStepper::DRIVER, P_PUL, P_DIR ); // PITCH
AccelStepper Y( AccelStepper::DRIVER, Y_PUL, Y_DIR ); // YAW

/* ----------------------- TRANSMISSION FROM COMPUTER ----------------------- */

float incoming[3] = { 0, 0, 0 };

/* ---------------------------- SENSOR DEFINITION --------------------------- */

static const int oledSDA = 4;
static const int oledSCL = 15;
static const int oledRST = 16;

static const int pubSDA = 21;
static const int pubSCL = 22;

int euler[3] = { 0, 0, 0 };

// MPU9250 imu( Wire1, 0x68 );
// MPU9250_DMP imu;
Adafruit_BNO055 bno( -1, 0x29, &Wire1 );

/* -------------------------------------------------------------------------- */

void setup() 
{
    // init on-board chip, clear display
    // enable OLED and serial, disable LoRa (doesn't have anyway)
    Heltec.begin( true, false, true );
    Heltec.display->clear();
    Heltec.display->display();
    Serial.begin( BAUDRATE );

    // init IMU
    Wire1.begin( pubSDA, pubSCL );
    bool status = bno.begin();

    // set up motor accel/decel and velocity
    P.setMaxSpeed( motorSpeed );
    P.setAcceleration( motorAccel );
    Y.setMaxSpeed( motorSpeed );
    Y.setAcceleration( motorAccel );

    Y.setCurrentPosition( 0 );
    P.setCurrentPosition( 0 );

    // enable WiFi
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin( SSID.c_str(), PSWD.c_str() );
    Heltec.display->drawString( 0, 0, "Connecting to " + SSID );
    Heltec.display->display();

    while( WiFi.status() != WL_CONNECTED ) {}

    Heltec.display->clear();
    ( status == false ) ? Heltec.display->drawString( 0,  0, "IMU Failed!" ) : Heltec.display->drawString( 0,  0, "IMU Active!" ); 
    Heltec.display->drawString( 0, 10, "Connected to " + SSID );
    Heltec.display->drawString( 0, 20, "IP = " + WiFi.localIP().toString() );
    Heltec.display->drawString( 0, 30, "Motor speed is " + String( motorSpeed ) );
    Heltec.display->drawString( 0, 40, "Motor accel is " + String( motorAccel ) );
    Heltec.display->drawString( 0, 50, "Continuing in 5..." );
    Heltec.display->display();

    // delay(5000);
}

void loop() 
{
    // if ( Y.currentPosition() == 800 )
    // {
    //     Y.moveTo( -800 );
    // }
    // else if ( Y.currentPosition() == -800 )
    // {
    //     Y.moveTo( 800 );
    // }
    // // Y.run();

    if ( Serial.available() > 0 )
    {
        if ( !zeroed )
        {
            String fromPC = Serial.readStringUntil('\r');
            StringSplitter* splitter = new StringSplitter( fromPC, ' ', 3 );
            initPitch = ( splitter->getItemAtIndex( 0 ) ).toInt();
            initYaw   = ( splitter->getItemAtIndex( 1 ) ).toInt();
            zeroed = !zeroed;
        }
        else
        {
            String fromPC = Serial.readStringUntil('\r');
            StringSplitter* splitter = new StringSplitter( fromPC, ' ', 3 );
            int newPitch = ( splitter->getItemAtIndex( 0 ) ).toInt();
            int newYaw   = ( splitter->getItemAtIndex( 1 ) ).toInt();

            incoming[0] = ( initPitch > 0 ) ? newPitch + initPitch : newPitch - initPitch;
            incoming[1] = (   initYaw > 0 ) ?   newYaw -   initYaw :   newYaw +   initYaw;

            if ( incoming[0] > 90 ) incoming[0] = 90;
            else if ( incoming[0] < -90 ) incoming[0] = -90;

            if ( incoming[1] > 90 ) incoming[1] = 90;
            else if ( incoming[1] < -90 ) incoming[1] = -90;

            // incoming[0] = ( abs( abs( newPitch ) - abs( incoming[0] ) ) > motionThreshold ) ? newPitch : incoming[0];
            // incoming[1] = ( abs( abs(   newYaw ) - abs( incoming[1] ) ) > motionThreshold ) ?   newYaw : incoming[1];
        }
        delayMicroseconds( 50 );
    }

    P.moveTo( -incoming[0] * 80 / 9 );
    Y.moveTo(  incoming[1] * 80 / 9 );

    P.run();
    Y.run();

    // sensors_event_t event;
    // bno.getEvent( &event, Adafruit_BNO055::VECTOR_EULER );

    // int x = int( event.orientation.x );
    // int y = int( event.orientation.y );
    // int z = int( event.orientation.z );

    // euler[0] = ( x != 0 ) ? x : euler[0]; //yaw
    // euler[1] = ( y != 0 ) ? y : euler[1]; //pitch
    // euler[2] = ( z != 0 ) ? z : euler[2];
    
    // Heltec.display->clear();
    // Heltec.display->drawString( 0,  0, "VR X = " + String( int( incoming[0] ) ) );
    // Heltec.display->drawString( 0, 10, "VR Y = " + String( int( incoming[1] ) ) );
    // Heltec.display->drawString( 0, 20, "VR Z = " + String( int( incoming[2] ) ) );
    // Heltec.display->drawString( 0, 30, "IMU X  = " + String( euler[0] ) );
    // Heltec.display->drawString( 0, 40, "IMU Y  = " + String( euler[1] ) );
    // Heltec.display->drawString( 0, 50, "IMU Z  = " + String( euler[2] ) );
    // Heltec.display->display();
}