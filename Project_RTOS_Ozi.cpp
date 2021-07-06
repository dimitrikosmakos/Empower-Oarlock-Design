#include "mbed.h"
#include "LSM9DS1.h"
#include "rtos.h"
#include "uLCD_4DGL.h"
#include "PinDetect.h"

Serial pc(USBTX, USBRX);
uLCD_4DGL uLCD(p28,p27,p30);             // serial tx, serial rx, reset pin
RawSerial  blue(p13,p14); // bluetooth - tx, rx
PinDetect button(p12);

Thread thread1; // LCD
Thread thread2; // bluetooth

Timer timer;
Mutex LCD;

volatile bool stroke;
volatile int state = 0;     // Different states for different displays
volatile int catch_angle = 0;
volatile int finish_angle = 0;
volatile float strokelength = 0;
volatile float maxaccel = 0;
volatile float stroke_time = 0;
volatile float half_stroke_time = 0;
volatile float prevstate = 3;


void state_machine()
{
    prevstate = state;
    if (state <= 2) state++;
    else state = 0;

}

void LCD_Screen()
{
    float power;
    while(1)
    {
        if (prevstate != state) {
            LCD.lock();
            uLCD.cls();
            if (state == 0){
                uLCD.text_width(1);
                uLCD.text_height(1);
                uLCD.color(RED);
                //uLCD.locate(0,0);
                // uLCD.printf("Catch and Finish Angles:");
                uLCD.locate(1,3);
                uLCD.printf("Catch:");
                uLCD.locate(1,4);
                uLCD.printf("Finish:");
                prevstate = 0;
            } else if (state == 1){
                uLCD.text_width(1);
                uLCD.text_height(1);
                uLCD.locate(1,0);
                uLCD.color(RED);
                uLCD.printf("Power Curve");
                // draw filled rectangles for graph
                uLCD.line(10,10,11,110, WHITE);  // x1, y1, x2, y2
                uLCD.line(10,110,110,112, WHITE);
                uLCD.locate(2,1); // x1, y1, x2, y2 for filled_rectangle
                prevstate = 1;
            } else if (state == 2){
                uLCD.locate(2,1);
                uLCD.color(RED);
                uLCD.printf("Power:");
                // only have to calculate power
                uLCD.locate(2,3);
                uLCD.color(WHITE);
                prevstate = 2;
            } else if (state == 3){
                uLCD.locate(2,2);
                uLCD.color(RED);
                uLCD.printf("SPM:");
                // strokes per minute is a running total
                uLCD.locate(2,2);
                uLCD.color(WHITE);
                prevstate = 3;
            }
        }
        LCD.unlock();

        if (state == 0)
        {
            //uLCD.locate(2,0);
            //uLCD.color(RED);
            //uLCD.printf("Catch and Finish Angles:");
            LCD.lock();
            uLCD.locate(3,3);
            uLCD.color(WHITE);
            uLCD.printf("%2d", -catch_angle);
            uLCD.locate(3,4);
            uLCD.printf("-%2d", finish_angle);
            LCD.unlock();
        }

        if (state == 1)
        {
            LCD.lock();
            /*
            uLCD.locate(2,0);
            uLCD.color(RED);
            uLCD.printf("Power Curve");
            // draw filled rectangles for graph
            uLCD.line(10,10,11,110, RED);  // x1, y1, x2, y2
            uLCD.line(10,110,110,112, RED);
            uLCD.locate(2,1); // x1, y1, x2, y2 for filled_rectangle
            uLCD.color(WHITE);
            */
            LCD.unlock();
        }

        if (state == 2)
        {
            LCD.lock();
            /*
            uLCD.locate(2,1);
            uLCD.color(RED);
            uLCD.printf("Power:");
            // only have to calculate power
            uLCD.locate(2,1);
            uLCD.color(WHITE);
            */
            if (maxaccel != 0 && half_stroke_time != 0)
            {
                power = ((catch_angle - finish_angle) * 2.6 * maxaccel * 1.8) / half_stroke_time; // average mass of an oar
            }
            uLCD.locate(2,4);
            uLCD.printf("%2d W", power);
            LCD.unlock();
        }

        if (state == 3)
        {
            LCD.lock();
            /*
            uLCD.locate(2,1);
            uLCD.color(RED);
            uLCD.printf("SPM:");
            // strokes per minute is a running total
            uLCD.locate(2,2);
            uLCD.color(WHITE);
            */
            uLCD.locate(2,3);
            if (stroke_time != 0)
            {
                uLCD.printf("%2d", (1/stroke_time) * 60); // rate of strokes/second * 60 seconds
            }
            LCD.unlock();
        }

        Thread::wait(500);
    }
}

void bt_thread() { // bluetooth
    char bnum = 0;
    while(1) {
        if (blue.getc()=='!') {
            if (blue.getc()=='B') { //button data
                bnum = blue.getc(); //button number
                if (bnum == '1')
                    state = 0;
                if (bnum == '2') 
                    state = 1;
                if (bnum == '3') 
                    state = 2;
                if (bnum == '4') 
                    state = 3;
            }
        }
        Thread::wait(100);
    }
}

int main()
{
    button.mode(PullUp);
    button.attach_asserted(&state_machine);
    button.setSampleFrequency();

    LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    IMU.calibrate(1);
    IMU.calibrateMag(0);

    timer.start();
    timer.reset();

    uLCD.cls();
    uLCD.background_color(BLACK);
    uLCD.baudrate(300000);

    thread1.start(LCD_Screen);
    thread2.start(bt_thread);
    
    float finish;
    float catch1;

    float accel;
    float timedefault;
    float magRead;

    float stroke_start = 0;

    bool finish_read = true;
    bool catch_read = false;

    while(1) { //Main Thread - reading in sensor data and outputting angles
        timedefault = timer.read_ms();
        IMU.readGyro();
        IMU.readMag();
        magRead = IMU.calcMag(IMU.my);
        accel = IMU.calcGyro(IMU.gy);

        pc.printf("MAG: %f,  GYRO: %f\n\r", magRead, accel);


        // Calculating Maximum Acceleration
        if (accel > maxaccel)
            maxaccel = accel;

        // Reading in Finish Angles

        if (accel > 3.0 && ((magRead > 4.0) || (magRead < -4.0))) finish_read = true;


        // Incrementing Finish Angles over one stroke
        if (accel > 3.0 && finish_read) finish += (accel * ((timer.read_ms()-timedefault) / 1000));

        else if (accel < -3.0 && finish_read) // When acceleration changes direction, stop reading finish angles
        {
            if (stroke_start != 0)
            {
                half_stroke_time = (timer.read_ms() - stroke_start) / 1000;
            }
            finish_read = false;
            finish_angle = ((int) finish) + 13;
            finish = 0;
            pc.printf("Finish angle:                                            %d\n\r", finish_angle);

        }



        // Reading in Catch Angles

        if (accel < -3.0 && ((magRead > 4.0) || (magRead < -4.0))) catch_read = true;

        // Increment catch angle over one stroke
        if (accel < -3.0 && catch_read) catch1 += (accel * ((timer.read_ms()-timedefault) / 1000));

        else if (accel > 3.0 && catch_read) // When acceleration changes direction, stop reading catch angles
        { // only happens once per cycle
            if (stroke_start != 0)
            {
                stroke_time = ((timer.read_ms() - stroke_start) / 1000);
            }
            stroke_start = timer.read_ms(); // calculate start time of stroke
            catch_read = false;
            catch_angle = ((int) catch1) - 13;
            catch1 = 0;
            maxaccel = 0; // maxaccel reset to 0 because another stroke is starting
            pc.printf("Catch angle:                                             %d\n\r", catch_angle);
        }
        Thread::wait(10);
    }
}
