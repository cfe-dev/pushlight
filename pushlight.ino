/*
  Button Read
  Reads Button, activates LED when pressed.
  Button signal needs to be pulled down with a resistor
  cfe-dev
  cfe.co.at
  2022/10
*/

#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Pin D7
#define GPIO_BTN 13

// Pin D0
#define GPIO_SERVO 16

// Pin D2
#define GPIO_GPSRX 4
// Pin D1
#define GPIO_GPSTX 5

#define GPIO_COUNT 17
//#define GPIO_COUNT 16

// array indices for jobs
#define JOB_LED 0
#define JOB_MOTOR 1
#define JOB_TRACKPINS 2
#define JOB_GPS 3

unsigned long cur_run_ms = 0;

// ******************
// Job 1, LED setting vars
int brightness = 0; // how bright the LED is (0 = full, 512 = dim, 1023 = off)
int fadeAmount = 8; // how many points to fade the LED by

// const int brightness_max = 1023;
const int brightness_max = 384;
const int brightness_min = 0;
const int fadeAmount_max = 35;
const int fadeAmount_min = 4;

// ******************
// Job 2, Servo vars
Servo servo;

int servo_steps = 1;
int servo_angle = 0;
const int servo_angle_max = 180;
const int servo_angle_min = 0;
bool direction_up = true;

// **** Disable for now ****
// ******************
// Job 3, GPS

SoftwareSerial gpsSerial(GPIO_GPSRX, GPIO_GPSTX);
TinyGPS gps;
//
// **** Disable for now ****

// float lat = 28.5458, lon = 77.1703;
float lat = 0, lon = 0;

// Job data
struct t_job {
    unsigned long last_run_ms;
    const long interval;
} jobs[4] = {
    {0, 10}, // index JOB_LED, 10ms interval
    {0, 20}, // index JOB_MOTOR, 20ms interval
    {0, 10}, // index JOB_TRACKPINS, 10ms interval
    {0, 500} // index JOB_GPS, 0.5s interval
};

// Pin Value tracking
struct t_pinvals {
    int pin;
    int val;
} pinvals[GPIO_COUNT] = {};

void setup() {
    Serial.begin(115200);
    Serial.println(F("Pushlight Start!"));

    gpsSerial.begin(9600);

    // JOB_LED
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(GPIO_BTN, INPUT);

    // JOB_TRACKPINS
    // initialize pinvals
    for (int i = 1; i < GPIO_COUNT; i++) {
        pinvals[i].pin = i;
        pinvals[i].val = -1;
    }

    pinvals[GPIO_BTN].val = 0;
    pinvals[GPIO_GPSRX].val = 0;
    pinvals[GPIO_GPSTX].val = 0;

    // JOB_MOTOR
    servo.attach(GPIO_SERVO);
    servo.write(0);
}

void loop() {
    // // test gps serial directly
    // while (gpsSerial.available()) {
    //     Serial.write(gpsSerial.read());
    // }

    // delay(500);
    // Serial.println(F("loop"));

    // // test button directly
    // int pinval = digitalRead(GPIO_BTN);
    // Serial.println(pinval);

    cur_run_ms = millis();
    if (check_job_interval(jobs[JOB_LED])) fade_led();
    if (check_job_interval(jobs[JOB_MOTOR])) turn_servo();
    if (check_job_interval(jobs[JOB_TRACKPINS])) track_pins();
    if (check_job_interval(jobs[JOB_GPS])) read_gps();
}

bool check_job_interval(t_job &job) {
    if (cur_run_ms - job.last_run_ms > job.interval) {
        job.last_run_ms = cur_run_ms;
        return true;
    }
    return false;
}

void track_pins() {
    for (int i = 1; i < GPIO_COUNT; i++) {
        if (pinvals[i].val != -1) {
            int pinval = digitalRead(i);
            if (pinvals[i].val != pinval) {
                pinvals[i].val = pinval;
                Serial.print(i);
                Serial.print(F(" "));
                Serial.print(pinval);
                Serial.println();
            }
        }
    }
}

void fade_led() {
    // // original; fade based on button pressed time along a curve
    //
    // int pinval = digitalRead(GPIO_BTN);
    // if (pinval == HIGH)
    //     brightness += fadeAmount;
    // else
    //     brightness -= fadeAmount;
    // // limit to 10-bit (0-1023)
    // constrain(brightness, brightness_min, brightness_max);
    // // https://www.wolframalpha.com/input?i=plot+%28+%28+131072%2F%28-%2860x%29-4096%29+%29+%2B+32++%29+%2C+x%3D0..1023
    // fadeAmount = (131072 / ((-3 * brightness) - 4096)) + 32;
    // constrain(fadeAmount, fadeAmount_min, fadeAmount_max);

    // new: map from servo angle
    brightness = map_angle_to_brightness(servo_angle);
    constrain(brightness, brightness_min, brightness_max);

    analogWrite(LED_BUILTIN, brightness);
    // Serial.println(fadeAmount);
}

void turn_servo() {
    int pinval = digitalRead(GPIO_BTN);
    if (pinval == LOW) {
        // servo_angle = servo_angle + servo_steps;
        // if (servo_angle > 180) servo_angle = 0;

        if (servo_angle > 180) servo_angle = 180;
        if (servo_angle < 0) servo_angle = 0;

        if (servo_angle == 180) direction_up = false;
        if (servo_angle == 0) direction_up = true;

        if (direction_up) {
            servo_angle = servo_angle + servo_steps;
        } else {
            servo_angle = servo_angle - servo_steps;
        }

        servo.write(servo_angle);
        // Serial.println(servo_angle);
    }
}

int map_angle_to_brightness(int angle) {
    float ratio = (brightness_max - brightness_min) / (servo_angle_max - servo_angle_min);
    float fval = ratio * (angle - servo_angle_min) + brightness_min;
    return round(fval);
}

void read_gps() {
    // if (gpsSerial.available()) {
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            gps.f_get_position(&lat, &lon);
        }
    }

    // if (lat != 0 || lon != 0) {
    //     // display position
    //     // Serial.print("Position: ");
    //     Serial.print("Latitude:");
    //     Serial.print(lat, 6);
    //     Serial.print(";");
    //     Serial.print("Longitude:");
    //     Serial.println(lon, 6);
    // }
}