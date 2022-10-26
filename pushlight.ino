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

unsigned long cur_run_ms = 0;

// ******************
// Job 1, LED setting vars
const int JOB_INTV_LED = 40;

int brightness = 0; // how bright the LED is (0 = full, 512 = dim, 1023 = off)
int fadeAmount = 4; // how many points to fade the LED by

// const int brightness_max = 1023;
const int brightness_max = 384;
const int brightness_min = 0;
const int fadeAmount_max = 35;
const int fadeAmount_min = 4;

// ******************
// Job 2, Servo vars
const int JOB_INTV_MOTOR = 400;

Servo servo;

int servo_steps = 4;
int servo_angle = 0;
const int servo_angle_max = 90;
const int servo_angle_min = 10;
bool direction_up = true;

// ******************
// Job 3, Pin Value tracking
const int JOB_INTV_TRACKPINS = 10;

struct t_pinvals {
    int pin;
    int val;
} pinvals[GPIO_COUNT] = {};

// ******************
// Job 4, GPS
const int JOB_INTV_GPS = 500;
SoftwareSerial gpsSerial(GPIO_GPSRX, GPIO_GPSTX);
TinyGPS gps;
float lat = 0, lon = 0;

// ******************
// Job 5, Gestures
const int JOB_INTV_GESTURES = 20;
YA_FSM gesture_FSM;
enum State { IDLE,
             MOVING,
             MOVING_TO_POSITION,
             AWAIT_GESTURE };

const char *StateName[4] = {
    "Idle",
    "Moving",
    "Moving to Position",
    "Awaiting Gesture"};

struct t_gesture {
    int last_state = 0;
    bool turn_servo = false;
    bool last_btn_state = false;
    unsigned long last_btn_up;
    unsigned long last_btn_down;
    int click_counts = 0;
    int servo_target_pos = 0;

    const int SERVO_TOLERANCE = 4;
    const int THRESHOLD_CLICK_MIN = 50;
    const int THRESHOLD_CLICK_MAX = 1200;
    const int THRESHOLD_MOVE_MAX = 5000;
} gesture = {};

// ******************
// Job 6, Read Button
// reads button in a fixed, slower interval
// to even out jumps;
// global value for button state
const int JOB_INTV_READBTN = 200;
bool button_state = false;

// Job mgmt data

enum Jobs {
    JOB_LED,
    JOB_MOTOR,
    JOB_TRACKPINS,
    JOB_GPS,
    JOB_GESTURES,
    JOB_READBTN
};

struct t_job {
    unsigned long last_run_ms;
    const long interval;
} jobs[6] = {
    {0, JOB_INTV_LED},       // ix JOB_LED
    {0, JOB_INTV_MOTOR},     // ix JOB_MOTOR
    {0, JOB_INTV_TRACKPINS}, // ix JOB_TRACKPINS
    {0, JOB_INTV_GPS},       // ix JOB_GPS
    {0, JOB_INTV_GESTURES},  // ix JOB_GESTURES
    {0, JOB_INTV_READBTN}    // ix JOB_READBTN
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
        pinvals[i].val = -1; // skip all by default
    }

    // enable relevant pins
    pinvals[GPIO_BTN].val = 0;
    pinvals[GPIO_GPSRX].val = 0;
    pinvals[GPIO_GPSTX].val = 0;

    // JOB_MOTOR
    servo.attach(GPIO_SERVO);
    servo.write(gesture.servo_target_pos);

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
    // if (check_job_interval(jobs[JOB_TRACKPINS])) track_pins();
    if (check_job_interval(jobs[JOB_GPS])) read_gps();
    if (check_job_interval(jobs[JOB_GESTURES])) process_gestures();
    if (check_job_interval(jobs[JOB_READBTN])) read_btn();
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
    // int pinval = button_state; //digitalRead(GPIO_BTN);
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
    if (gesture.turn_servo) {

        // int pinval = button_state;
        // if (pinval == LOW) {
        // servo_angle = servo_angle + servo_steps;
        // if (servo_angle > 180) servo_angle = 0;

        int servo_angle_prv = servo_angle;

        if (servo_angle > servo_angle_max) servo_angle = servo_angle_max;
        if (servo_angle < servo_angle_min) servo_angle = servo_angle_min;

        if (servo_angle == servo_angle_max) direction_up = false;
        if (servo_angle == servo_angle_min) direction_up = true;

        if (direction_up) {
            servo_angle = servo_angle + servo_steps;
        } else {
            servo_angle = servo_angle - servo_steps;
        }

        // smoothe out servo movement; unnessecary writes cause janks
        if (servo_angle_prv != servo_angle) {
            servo.write(servo_angle);
            // Serial.println(servo_angle);
        }
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
    if (button_state == LOW) {
        gesture.last_btn_down = cur_run_ms;
        gesture.last_btn_state = true;
        gesture.click_counts = 0;
        return true;
    }
    return false;
}

bool gesture_release() {
    if (button_state != LOW) {
        gesture.last_btn_up = cur_run_ms;
        gesture.last_btn_state = false;
        return true;
    }
    return false;
}

bool gesture_hold() {
    if (button_state == LOW && gesture.last_btn_state == true                    // button is already pressed
        && ((cur_run_ms - gesture.last_btn_down) > gesture.THRESHOLD_CLICK_MAX)) // and more than time CLICK_MAX time has passed since last button down
    {
        if (gesture.click_counts > 0) // switch direction if a click has been entered
            direction_up = !direction_up;
        return true;
    }
    return false;
}

void gesture_check() {
    // on button up, add click if minimum threshold is reached
    // and the last click was not longer back than the max click threshold
    if (button_state != LOW && gesture.last_btn_state == true                   // button is being released
        && ((cur_run_ms - gesture.last_btn_down) > gesture.THRESHOLD_CLICK_MIN) // and at least CLICK_MIN has occured since last button down
        && ((cur_run_ms - gesture.last_btn_up) < gesture.THRESHOLD_CLICK_MAX))  // and less than CLICK_MAX has occured since last button up
    {
        gesture.last_btn_up = cur_run_ms;
        gesture.last_btn_state = false;
        gesture.click_counts += 1;
    }
    if (button_state == LOW && gesture.last_btn_state == false) { // button is being pressed
        gesture.last_btn_down = cur_run_ms;
        gesture.last_btn_state = true;
    }
}

bool gesture_in_position() {
    if ((servo_angle + gesture.SERVO_TOLERANCE) > gesture.servo_target_pos     // upper range of tolerance window is greater than target
        && (servo_angle - gesture.SERVO_TOLERANCE) < gesture.servo_target_pos) // AND lower range of tolerance window is less than target
        return true;
    else
        return false;
}

void gesture_enter_move() {
    gesture.turn_servo = true;
}

void gesture_leave_move() {
    gesture.turn_servo = false;
}

bool gesture_select() {
    if ((button_state != LOW && gesture.last_btn_state == false)                 // button is already released
        && (gesture.last_btn_up > gesture.last_btn_down)                         // and button has actually been pressed before
        && ((cur_run_ms - gesture.last_btn_down) > gesture.THRESHOLD_CLICK_MAX)) // and at least CLICK_MAX time has passed since button down
    {
        switch (gesture.click_counts) {
        case 1:
            gesture.servo_target_pos = 20;
            Serial.println(F("Pos 1"));
            break;

        case 2:
            gesture.servo_target_pos = 70;
            Serial.println(F("Pos 2"));
            break;

        default:
            break;
        }
        if (gesture.servo_target_pos > servo_angle)
            direction_up = true;
        if (gesture.servo_target_pos < servo_angle)
            direction_up = false;
        return true;
    }
    return false;
}

bool gesture_cancel() {
    if ((cur_run_ms - gesture.last_btn_down) > gesture.THRESHOLD_MOVE_MAX  // at least MOVE_MAX time has passed since last button down
        && (cur_run_ms - gesture.last_btn_up) > gesture.THRESHOLD_MOVE_MAX // at least MOVE_MAX time has passed since last button up
        && (gesture.last_btn_up > gesture.last_btn_down)) {                // button has actually been pressed once
        return true;
    }
    return false;
}

void process_gestures() {
    if (gesture_FSM.Update()) {
        int new_state = gesture_FSM.GetState();
        if (new_state != gesture.last_state)
            Serial.println(gesture_FSM.ActiveStateName());
        gesture.last_state = new_state;
    }
}

void read_btn() {
    bool old_state = button_state;
    button_state = (digitalRead(GPIO_BTN) == LOW) ? false : true;
    if (old_state != button_state)
        Serial.println(button_state);
}