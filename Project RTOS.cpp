#include "mbed.h"
#include "LSM9DS1.h"
#define PI 3.14159
#include "rtos.h"
#include "uLCD_4DGL.h"

// added comment about thing

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -4.94 // Declination (degrees) in Atlanta,GA.



#define ANGLES_SCREEN 0;
#define CURVE_SCREEN 1;
#define POWER_SCREEN 2;
#define SPM_SCREEN 3;
// Calculate pitch, roll, and heading.
// Pitch/roll calculations taken from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
// void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
// {
//     float roll = atan2(ay, az);
//     float pitch = atan2(-ax, sqrt(ay * ay + az * az));
// // touchy trig stuff to use arctan to get compass heading (scale is 0..360)
//     mx = -mx;
//     float heading;
//     if (my == 0.0)
//         heading = (mx < 0.0) ? 180.0 : 0.0;
//     else
//         heading = atan2(mx, my)*360.0/(2.0*PI);
//     //pc.printf("heading atan=%f \n\r",heading);
//     heading -= DECLINATION; //correct for geo location
//     if(heading>180.0) heading = heading - 360.0;
//     else if(heading<-180.0) heading = 360.0 + heading;
//     else if(heading<0.0) heading = 360.0  + heading;


//     // Convert everything from radians to degrees:
//     //heading *= 180.0 / PI;
//     pitch *= 180.0 / PI;
//     roll  *= 180.0 / PI;

//     pc.printf("Pitch: %f,    Roll: %f degress\n\r",pitch,roll);
//     pc.printf("Magnetic Heading: %f degress\n\r",heading);
// }

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
Serial pc(USBTX, USBRX);
uLCD_4DGL uLCD(p9,p10,p11);             // serial tx, serial rx, reset pin
InterruptIn button(p12);

Thread thread1;
Thread thread2;
Thread thread3;
Thread thread4;
Thread thread5;

// asdf

Timer timer;
Mutex LCD;

volatile int state;     // Different states for different displays
volatile bool stroke;
volatile int catch_angle;
volatile int finish_angle;


void state_machine()
{
    if (state <= 2) state++;
    else state = 0;
}

void read_magnet();
{
    while(1)
    {
        IMU.readMag();
        if(IMU.my >= 4.0 || IMU.my <= -4.0)
        {
            timer.reset();
        }
        Thread::wait(100);
    }
}

void angles()
{
    while(1)
    {
        if (state == ANGLES_SCREEN)
        {
            LCD.lock();
            uLCD.text_width(2);
            uLCD.text_height(2);
            uLCD.locate(2,1);
            uLCD.printf("%2d", catch_angle);
            uLCD.locate(2,2);
            uLCD.printf("%2d", finish_angle);
            LCD.unlock();
        }
        Thread::wait(1000);
    }
}

void force_curve()
{
    while(1)
    {
        if (state == CURVE_SCREEN)
        {

        }
        Thread::wait(500);
    }
}

void power()
{
    while(1)
    {
        if (state == POWER_SCREEN)
        {

        }
        Thread::wait(500);
    }
}

void stroke_rate();
{
    while(1)
    {
        if (state == SPM_SCREEN)
        {

        }
        Thread::wait(500);
    }
}


int main()
{

    button.mode(PullUp);
    button.rise(&state_machine);
    //LSM9DS1 lol(p9, p10, 0x6B, 0x1E);
    LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    IMU.calibrate(1);
    IMU.calibrateMag(0);


    timer.reset();
    timer.start();

    LCD.cls();
    LCD.background_color(BLACK);
    LCD.baudrate(300000);

    thread1.start(angles);
    thread2.start(force_curve);
    thread4.start(read_magnet);
    thread5.start(power);
    thread6.start(stroke_rate);

    float finish;
    float catch1;
    float value2;
    int finish_angle;
    int catch_angle;


    while(1) { //Main Thread - reading in sensor data
        IMU.readGyro();
        IMU.readMag();
            float magRead = IMU.calcMag(IMU.my);
            float value1 = IMU.calcGyro(IMU.gy)*2.6;
            //pc.printf("%f\n\r",value1);

//            if (magRead > 4.0 || magRead < -4.0)    // Checking if the gyro has passed the zero state
//            {
//                timer.reset();
//                led1 = 1;
//            }
            if ((value1 > 4.0) && (magRead > 4.0))                         // Start measuring for the FINISH Angle
            {
                IMU.readGyro();
                value2 = IMU.calcGyro(IMU.my);
                catch1 = 0.0;
                while(value2 > 4.0)
                {
                    led2 = 1;
                    IMU.readGyro();
                    value2 = IMU.calcGyro(IMU.my);
                    finish += (value2 * (timer.read_ms() / 1000));
                    timer.reset();
                    Thread::wait(100);
                }
                finish_angle = (int) finish;
                led2 = 0;
            }
            pc.printf("Finish Angle is:     %d\n\r",finish_angle);
            led1 = 0;


            if ((value1 < -4.0) && (magRead > 4.0))
            {
                IMU.readGyro();
                value2 = IMU.calcGyro(IMU.my);
                finish = 0.0;
                while(value2 < -4.0)
                {
                    led3 = 1;
                    IMU.readGyro();
                    value2 = IMU.calcGyro(IMU.my);
                    catch1 += (value2 * (timer.read_ms() / 1000));
                    timer.reset();
                    Thread::wait(100);
                }
                catch_angle = (int) catch1;
                led3 = 0;
            }
            pc.printf("Catch Angle is:      %d\n\r",catch_angle);

            // if (value1 > 4.0)
            // {
            //     led1 = 1;
            //     pc.printf("Drive\n\r");
            // }
            // else led1 = 0;

            // if (value1 < -4.0)
            // {
            //     led2 = 1;
            //     pc.printf("Recovery\n\r");
            // }
            // else led2 = 0;
            // float value2 = value1;


        Thread::wait(100);
    }
}


//     while(1) { //Main Thread - reading in sensor data
//         IMU.readGyro();
//             float value1 = IMU.calcGyro(IMU.gy)*2.6;
//             pc.printf("%f\n\r",value1);

//             if (value1 > 4.0)
//             {
//                 led1 = 1;
//                 pc.printf("Drive\n\r");
//             }
//             else led1 = 0;

//             if (value1 < -4.0)
//             {
//                 led2 = 1;
//                 pc.printf("Recovery\n\r");
//             }
//             else led2 = 0;
//             float value2 = value1;


//         Thread::wait(100);
//     }
// }
