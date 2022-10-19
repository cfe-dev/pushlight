/*
  Pushlight
  cfe-dev
  * controls a servo based on gestures implemented in a simple state machine.
  * dims the internal LED, brightness mapped from the servo position
  * collects GPS data
*/

// #include <Blinker.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <YA_FSM.h>

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
#define JOB_GESTURES 4

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

// ******************
// Job 3, Pin Value tracking
struct t_pinvals {
    int pin;
    int val;
} pinvals[GPIO_COUNT] = {};

// ******************
// Job 4, GPS
SoftwareSerial gpsSerial(GPIO_GPSRX, GPIO_GPSTX);
TinyGPS gps;

// float lat = 28.5458, lon = 77.1703;
float lat = 0, lon = 0;

// ******************
// Job 5, Gestures
YA_FSM gesture_FSM;
enum State { IDLE,
             MOVING,
             MOVING_TO_POSITION,
             AWAIT_GESTURE };
const char *StateName[4] = {
    "Idle",
    "Moving",
    "Moving to Target",
    "Awaiting Gesture"};
bool gesture_turn_servo = false;
bool last_btn_state = false;
unsigned long last_btn_up;
unsigned long last_btn_down;
int click_counts = 0;
int servo_target_pos = 0;
const int servo_tolerance = 5;

const int THRESHOLD_CLICK_MIN = 100;
const int THRESHOLD_CLICK_MAX = 1000;
const int THRESHOLD_MOVE_MAX = 10000;

// Job mgmt data
struct t_job {
    unsigned long last_run_ms;
    const long interval;
} jobs[5] = {
    {0, 50},  // index JOB_LED, 50ms interval
    {0, 50},  // index JOB_MOTOR, 50ms interval
    {0, 10},  // index JOB_TRACKPINS, 10ms interval
    {0, 500}, // index JOB_GPS, 0.5s interval
    {0, 20}   // index JOB_GESTURES, 10ms interval
};

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
    servo.write(servo_target_pos);

    // JOB_GESTURES
    setup_gesture_FSM();
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
    if (check_job_interval(jobs[JOB_GESTURES])) gesture_FSM.Update();
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
    // switch to gesture control instead of raw GPIO_BTN
    if (gesture_turn_servo) {

        // int pinval = digitalRead(GPIO_BTN);
        // if (pinval == LOW) {
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
    // while (gpsSerial.available() > 0) {
    //     if (gps.encode(gpsSerial.read())) {
    //         gps.f_get_position(&lat, &lon);
    //     }
    // }

    // if (lat != 0 || lon != 0) {
    //     // display position
    //     // Serial.print("Position: ");
    //     Serial.print("Latitude:");
    //     Serial.print(lat, 6);
    //     Serial.print(";");
    //     Serial.print("Longitude:");
    //     Serial.println(lon, 6);
    // }

    Serial.println(gesture_FSM.ActiveStateName());
}

void setup_gesture_FSM() {
    gesture_FSM.AddState(StateName[IDLE], nullptr, nullptr, nullptr);
    gesture_FSM.AddState(StateName[MOVING], gesture_enter_move, nullptr, gesture_leave_move);
    gesture_FSM.AddState(StateName[MOVING_TO_POSITION], gesture_enter_move, nullptr, gesture_leave_move);
    gesture_FSM.AddState(StateName[AWAIT_GESTURE], nullptr, gesture_check, nullptr);

    gesture_FSM.AddTransition(IDLE, AWAIT_GESTURE, gesture_first_press);
    gesture_FSM.AddTransition(AWAIT_GESTURE, MOVING, gesture_hold);
    gesture_FSM.AddTransition(AWAIT_GESTURE, MOVING_TO_POSITION, gesture_select);

    gesture_FSM.AddTransition(AWAIT_GESTURE, IDLE, gesture_cancel);
    gesture_FSM.AddTransition(MOVING_TO_POSITION, IDLE, gesture_in_position);
    gesture_FSM.AddTransition(MOVING, IDLE, gesture_release);
}

bool gesture_first_press() {
    if (digitalRead(GPIO_BTN) == LOW) {
        last_btn_down = cur_run_ms;
        last_btn_state = true;
        click_counts = 0;
        return true;
    }
    return false;
}

bool gesture_release() {
    if (digitalRead(GPIO_BTN) != LOW) {
        last_btn_up = cur_run_ms;
        last_btn_state = false;
        return true;
    }
    return false;
}

bool gesture_hold() {
    if (digitalRead(GPIO_BTN) == LOW && last_btn_state == true && ((cur_run_ms - last_btn_down) > THRESHOLD_CLICK_MAX)) {
        if (click_counts > 0)
            direction_up = !direction_up;
        return true;
    }
    return false;
}

void gesture_check() {
    // on button up, add click if minimum threshold is reached
    // and the last click was not longer back than the max click threshold
    if (digitalRead(GPIO_BTN) != LOW && last_btn_state == true && ((cur_run_ms - last_btn_down) > THRESHOLD_CLICK_MIN) && ((cur_run_ms - last_btn_up) < THRESHOLD_CLICK_MAX)) {
        last_btn_up = cur_run_ms;
        last_btn_state = false;
        click_counts += 1;
    }
    if (digitalRead(GPIO_BTN) == LOW && last_btn_state == false) {
        last_btn_down = cur_run_ms;
        last_btn_state = true;
    }
}

bool gesture_in_position() {
    if ((servo_angle + servo_tolerance) > servo_target_pos && (servo_angle - servo_tolerance) < servo_target_pos)
        return true;
    else
        return false;
}

void gesture_enter_move() {
    gesture_turn_servo = true;
}

void gesture_leave_move() {
    gesture_turn_servo = false;
}

bool gesture_select() {
    if (digitalRead(GPIO_BTN) != LOW && last_btn_state == false && (last_btn_up > last_btn_down) && ((cur_run_ms - last_btn_down) > THRESHOLD_CLICK_MAX)) {
        switch (click_counts) {
        case 1:
            servo_target_pos = 20;
            break;

        case 2:
            servo_target_pos = 140;
            break;

        default:
            break;
        }
        return true;
    }
    return false;
}

bool gesture_cancel() {
    if ((cur_run_ms - last_btn_down) > THRESHOLD_MOVE_MAX || (cur_run_ms - last_btn_up) > THRESHOLD_MOVE_MAX) {
        return true;
    }
    return false;
}
