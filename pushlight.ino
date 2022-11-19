/*
 * Pushlight
 * cfe-dev
 * controls a servo based on gestures implemented in a simple state machine.
 * dims the internal LED, brightness mapped from the servo position
 * collects GPS data
 */

// #include <Blinker.h>
// #include <AsyncHTTPSRequest_Generic.h>
#include <ArduinoJson.h>
#include <AsyncHTTPRequest_Generic.h>
#include <ESP8266WiFi.h>
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

#define GPS_CACHE_SIZE 100

// client id for server communication
// change for every unique consumer
#define PUSHLIGHT_CLIENT_ID 1

unsigned long cur_run_ms = 0;

// ******************
// Job 1, LED setting vars
const int JOB_INTV_LED = 40;

struct t_led_ctrl {
    int brightness = 0; // how bright the LED is (0 = full, 512 = dim, 1023 = off)
    int fadeAmount = 4; // how many points to fade the LED by

    // const int brightness_max = 1023;
    const int brightness_max = 384;
    const int brightness_min = 0;
    const int fadeAmount_max = 35;
    const int fadeAmount_min = 4;
} led_ctrl = {};

// ******************
// Job 2, Servo vars
const int JOB_INTV_MOTOR = 50;

Servo servo;

struct t_servo_ctrl {
    int steps = 1;
    int angle = 0;
    const int angle_max = 85;
    const int angle_min = 40;
    const int target_pos_1 = 45;
    const int target_pos_2 = 75;
    bool direction_up = true;
} servo_ctrl = {};

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
unsigned long gps_last_age = 0;
struct t_gpsdata {
    float lat = 0,
          lon = 0;
    unsigned long age = 0;
    int servo_angle = 0;
} gpsdata[GPS_CACHE_SIZE] = {};

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
    unsigned long last_btn_up = 0;
    unsigned long last_btn_down = 0;
    int click_counts = 0;
    int servo_target_pos = 0;
    int diff_down = 0;
    int diff_up = 0;

    const int SERVO_TOLERANCE = 4;
    const int THRESHOLD_CLICK_MIN = 5;
    const int THRESHOLD_CLICK_MAX = 1500;
    const int THRESHOLD_MOVE_MAX = 5000;
} gesture_ctrl = {};

// ******************
// Job 6, Read Button
// reads button in a fixed, slower interval
// to even out jumps;
// global value for button state
const int JOB_INTV_READBTN = 200;
bool button_state = false;

// ******************
// Job 7, http send data
const int JOB_INTV_HTTPSEND = 5000;
WiFiEventHandler WiFiDisconnectHandler;
const size_t capacity =
    JSON_ARRAY_SIZE(GPS_CACHE_SIZE) + GPS_CACHE_SIZE * JSON_OBJECT_SIZE(10);
DynamicJsonDocument http_json_doc(capacity);
struct t_http_ctrl {
    const char *AP_ssid = "";
    const char *AP_password = "";
    const char *PUSHLIGHT_SRV_ENDPOINT = "";

    AsyncHTTPRequest request;
} http_ctrl = {};

// Job mgmt data

enum Jobs {
    JOB_LED,
    JOB_MOTOR,
    JOB_TRACKPINS,
    JOB_GPS,
    JOB_GESTURES,
    JOB_READBTN,
    JOB_HTTPSEND
};

struct t_job {
    unsigned long last_run_ms;
    const long interval;
} jobs[7] = {
    {0, JOB_INTV_LED},       // ix JOB_LED
    {0, JOB_INTV_MOTOR},     // ix JOB_MOTOR
    {0, JOB_INTV_TRACKPINS}, // ix JOB_TRACKPINS
    {0, JOB_INTV_GPS},       // ix JOB_GPS
    {0, JOB_INTV_GESTURES},  // ix JOB_GESTURES
    {0, JOB_INTV_READBTN},   // ix JOB_READBTN
    {0, JOB_INTV_HTTPSEND}   // ix JOB_HTTPSEND
};

// change this to enable debug output via Serial Monitor
bool print_debug = false;

void setup() {
    Serial.begin(115200);
    if (print_debug)
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

    init_gps_data(gpsdata);

    // JOB_MOTOR
    servo.attach(GPIO_SERVO);
    servo.write(gesture_ctrl.servo_target_pos);

    // JOB_GESTURES
    setup_gesture_FSM();

    WiFi_setup();
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
    if (check_job_interval(jobs[JOB_HTTPSEND])) WiFi_sendRequest();
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
    led_ctrl.brightness = map_angle_to_brightness(servo_ctrl.angle);
    constrain(led_ctrl.brightness, led_ctrl.brightness_min, led_ctrl.brightness_max);

    analogWrite(LED_BUILTIN, led_ctrl.brightness);
    if (print_debug)
        Serial.println(led_ctrl.fadeAmount);
}

void turn_servo() {
    // switch to gesture_ctrl control instead of raw GPIO_BTN
    if (gesture_ctrl.turn_servo) {

        // int pinval = button_state;
        // if (pinval == LOW) {
        // servo_angle = servo_angle + servo_steps;
        // if (servo_angle > 180) servo_angle = 0;

        int servo_angle_prv = servo_ctrl.angle;

        if (servo_ctrl.angle > servo_ctrl.angle_max) servo_ctrl.angle = servo_ctrl.angle_max;
        if (servo_ctrl.angle < servo_ctrl.angle_min) servo_ctrl.angle = servo_ctrl.angle_min;

        if (servo_ctrl.angle == servo_ctrl.angle_max) servo_ctrl.direction_up = false;
        if (servo_ctrl.angle == servo_ctrl.angle_min) servo_ctrl.direction_up = true;

        if (servo_ctrl.direction_up) {
            servo_ctrl.angle = servo_ctrl.angle + servo_ctrl.steps;
        } else {
            servo_ctrl.angle = servo_ctrl.angle - servo_ctrl.steps;
        }

        // smoothe out servo movement; unnessecary writes cause janks
        if (servo_angle_prv != servo_ctrl.angle) {
            servo.write(servo_ctrl.angle);
            if (print_debug)
                Serial.println(servo_ctrl.angle);
        }
    }
}

int map_angle_to_brightness(int angle) {
    float ratio = (led_ctrl.brightness_max - led_ctrl.brightness_min) / (servo_ctrl.angle_max - servo_ctrl.angle_min);
    float fval = ratio * (angle - servo_ctrl.angle_min) + led_ctrl.brightness_min;
    return round(fval);
}

void read_gps() {
    float lat = 0, lon = 0;
    unsigned long age = 0;
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            gps.f_get_position(&lat, &lon, &age);
        }
    }

    if (lat != 0 &&
        lon != 0 &&
        age != 0 &&
        lat != TinyGPS::GPS_INVALID_F_ANGLE &&
        lon != TinyGPS::GPS_INVALID_F_ANGLE &&
        age != TinyGPS::GPS_INVALID_AGE &&
        age != gps_last_age) {

        gps_last_age = age;

        t_gpsdata gpsdata_new = {lat, lon, age, servo_ctrl.angle};
        bool is_new = true;
        int empty_index = -1;

        for (int i = 0; i < GPS_CACHE_SIZE; i++) {
            if (gpsdata[i].age == age) {
                is_new = false;
                break;
            }
            if (empty_index == -1 &&
                (gpsdata[i].age == 0 ||
                 gpsdata[i].age == TinyGPS::GPS_INVALID_AGE)) {
                empty_index = i;
            }
        }
        if (empty_index != -1 &&
            is_new) {
            gpsdata[empty_index] = gpsdata_new;
        }
    }

    if (print_debug && (lat != 0 || lon != 0)) {
        // display position
        // Serial.print("Position: ");
        Serial.print("Latitude:");
        Serial.print(lat, 6);
        Serial.print(";");
        Serial.print("Longitude:");
        Serial.print(lon, 6);
        Serial.print("Age:");
        Serial.println(age);
    }
}

void setup_gesture_FSM() {
    // order of AddState() calls has to match order of enum State!!
    gesture_FSM.AddState(StateName[IDLE], gesture_enter_idle, nullptr, nullptr);
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
        gesture_ctrl.last_btn_down = cur_run_ms;
        gesture_ctrl.last_btn_state = true;
        gesture_ctrl.click_counts = 0;
        if (print_debug)
            Serial.println(F("Gesture First Press"));
        return true;
    }
    return false;
}

bool gesture_release() {
    if (button_state != LOW) {
        gesture_ctrl.last_btn_up = cur_run_ms;
        gesture_ctrl.last_btn_state = false;
        gesture_ctrl.diff_down = 0;
        gesture_ctrl.diff_up = 0;
        gesture_ctrl.last_btn_down = 0;
        if (print_debug)
            Serial.println(F("Gesture Release"));
        return true;
    }
    return false;
}

bool gesture_hold() {
    // int diff_down = cur_run_ms - gesture_ctrl.last_btn_down;
    // Serial.print(F("Diff down:"));
    // Serial.println(diff_down);
    if (button_state == LOW && gesture_ctrl.last_btn_state == true      // button is already pressed
        && (gesture_ctrl.last_btn_down > gesture_ctrl.last_btn_up)      // and button hasn't been released
        && (gesture_ctrl.diff_down > gesture_ctrl.THRESHOLD_CLICK_MAX)) // and more than time CLICK_MAX time has passed since last button down
    {
        if (gesture_ctrl.click_counts > 0) { // switch direction if a click has been entered
            servo_ctrl.direction_up = !servo_ctrl.direction_up;
            if (print_debug)
                Serial.println(F("Gesture Reverse Hold"));
        } else {
            if (print_debug)
                Serial.println(F("Gesture Hold"));
        }
        return true;
    }
    return false;
}

void gesture_check() {
    // on button up, add click if minimum threshold is reached
    // and the last click was not longer back than the max click threshold
    // int diff_down = cur_run_ms - gesture_ctrl.last_btn_down;
    // int diff_up = cur_run_ms - gesture_ctrl.last_btn_up;
    if (print_debug) {
        Serial.print(F("Diff down:"));
        Serial.print(gesture_ctrl.diff_down);
        Serial.print(F("; Diff Up:"));
        Serial.println(gesture_ctrl.diff_up);
    }
    if (button_state != LOW && gesture_ctrl.last_btn_state == true     // button is being released
                                                                       // && (gesture_ctrl.last_btn_down > gesture_ctrl.last_btn_up)      // and button down has occurred already
        && (gesture_ctrl.diff_down > gesture_ctrl.THRESHOLD_CLICK_MIN) // and at least CLICK_MIN has occured since last button down
        && (gesture_ctrl.diff_up < gesture_ctrl.THRESHOLD_CLICK_MAX))  // and less than CLICK_MAX has occured since last button up
    {
        gesture_ctrl.last_btn_up = cur_run_ms;
        gesture_ctrl.last_btn_state = false;
        gesture_ctrl.click_counts += 1;
        if (print_debug) {
            Serial.print(F("Last Btn Up: "));
            Serial.print(gesture_ctrl.last_btn_up);
            Serial.print(F("; Click Count: "));
            Serial.println(gesture_ctrl.click_counts);
        }
    }
    if (button_state == LOW && gesture_ctrl.last_btn_state == false) { // button is being pressed
        gesture_ctrl.last_btn_down = cur_run_ms;
        gesture_ctrl.last_btn_state = true;
        if (print_debug) {
            Serial.print(F("Last Btn Down: "));
            Serial.println(gesture_ctrl.last_btn_down);
            Serial.print(F("; Click Count: "));
            Serial.println(gesture_ctrl.click_counts);
        }
    }
}

bool gesture_in_position() {
    if ((servo_ctrl.angle + gesture_ctrl.SERVO_TOLERANCE) > gesture_ctrl.servo_target_pos     // upper range of tolerance window is greater than target
        && (servo_ctrl.angle - gesture_ctrl.SERVO_TOLERANCE) < gesture_ctrl.servo_target_pos) // AND lower range of tolerance window is less than target
        return true;
    else
        return false;
}

void gesture_enter_move() {
    gesture_ctrl.turn_servo = true;
}

void gesture_enter_idle() {
    gesture_ctrl.diff_down = 0;
    gesture_ctrl.diff_up = 0;
    gesture_ctrl.last_btn_down = 0;
    gesture_ctrl.last_btn_up = 0;
    gesture_ctrl.click_counts = 0;
    gesture_ctrl.last_btn_state = false;
}

void gesture_leave_move() {
    gesture_ctrl.turn_servo = false;
}

bool gesture_select() {
    if ((button_state != LOW && gesture_ctrl.last_btn_state == false)   // button is already released
        && (gesture_ctrl.last_btn_up > gesture_ctrl.last_btn_down)      // and button has actually been pressed before
        && (gesture_ctrl.diff_down > gesture_ctrl.THRESHOLD_CLICK_MAX)) // and at least CLICK_MAX time has passed since button down
    {
        switch (gesture_ctrl.click_counts) {
        case 1:
            gesture_ctrl.servo_target_pos = servo_ctrl.target_pos_1;
            if (print_debug)
                Serial.println(F("Pos 1"));
            break;

        case 2:
            gesture_ctrl.servo_target_pos = servo_ctrl.target_pos_2;
            if (print_debug)
                Serial.println(F("Pos 2"));
            break;

        default:
            if (print_debug)
                Serial.println(F("Pos Def"));
            break;
        }
        if (gesture_ctrl.servo_target_pos > servo_ctrl.angle)
            servo_ctrl.direction_up = true;
        if (gesture_ctrl.servo_target_pos < servo_ctrl.angle)
            servo_ctrl.direction_up = false;
        return true;
    }
    return false;
}

bool gesture_cancel() {
    // int diff_down = cur_run_ms - gesture_ctrl.last_btn_down;
    // int diff_up = cur_run_ms - gesture_ctrl.last_btn_up;
    // if (gesture_ctrl.diff_down > gesture_ctrl.THRESHOLD_MOVE_MAX      // at least MOVE_MAX time has passed since last button down
    //     && gesture_ctrl.diff_up > gesture_ctrl.THRESHOLD_MOVE_MAX     // at least MOVE_MAX time has passed since last button up
    //     && (gesture_ctrl.last_btn_up > gesture_ctrl.last_btn_down)) { // button has actually been pressed once
    if (gesture_ctrl.diff_up > (gesture_ctrl.THRESHOLD_CLICK_MAX * 4)) { // 4* click time has passed
        if (print_debug) {
            Serial.print("Diff up: ");
            Serial.println(gesture_ctrl.diff_up);
            Serial.println(F("Gesture Cancel"));
        }
        return true;
    }
    return false;
}

void process_gestures() {
    if (gesture_ctrl.last_btn_down > 0)
        gesture_ctrl.diff_down = cur_run_ms - gesture_ctrl.last_btn_down;
    if (gesture_ctrl.last_btn_up > 0)
        gesture_ctrl.diff_up = cur_run_ms - gesture_ctrl.last_btn_up;
    // Serial.print(F("Diff down:"));
    // Serial.print(gesture_ctrl.diff_down);
    // Serial.print(F("; Diff Up:"));
    // Serial.println(gesture_ctrl.diff_up);
    if (gesture_FSM.Update()) {
        int new_state = gesture_FSM.GetState();
        if (new_state != gesture_ctrl.last_state) {
            if (print_debug)
                Serial.println(gesture_FSM.ActiveStateName());
            gesture_ctrl.last_state = new_state;
        }
    }
}

void read_btn() {
    bool old_state = button_state;
    button_state = (digitalRead(GPIO_BTN) == LOW) ? false : true;
    if (old_state != button_state && print_debug) {
        char buffer[32];
        snprintf(buffer, 32, "Button: %d", button_state);
        Serial.println(buffer);
    }
}

void WiFi_setup() {

    WiFi.mode(WIFI_STA);
    WiFi.begin(http_ctrl.AP_ssid, http_ctrl.AP_password);

    // disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected &event) {
    //     WiFi.begin(http_ctrl.AP_ssid, http_ctrl.AP_password);
    // });

    WiFiDisconnectHandler = WiFi.onStationModeDisconnected(onWiFiDisconnect);
}

void onWiFiDisconnect(const WiFiEventStationModeDisconnected &event) {
    Serial.println("Disconnected from Wi-Fi, trying to reconnect...");
    WiFi.disconnect();
    WiFi.begin(http_ctrl.AP_ssid, http_ctrl.AP_password);
}

void WiFi_sendRequest() {
    if (WiFi.status() != WL_CONNECTED) return;

    AsyncHTTPRequest &req = http_ctrl.request;

    bool requestOpenResult, sendResult = false;
    // int datachg_count = count_gps_data_changes(gpsdata);
    int datachg_count = 1;

    if (print_debug)
        Serial.println("start wifi request");

    if (WiFi.status() == WL_CONNECTED &&
        datachg_count > 0 &&
        (req.readyState() == readyStateUnsent ||
         req.readyState() == readyStateDone)) {

        if (print_debug)
            Serial.println("connected & active");

        http_json_doc.clear();
        http_json_doc["pushlight_client_id"] = PUSHLIGHT_CLIENT_ID;
        http_json_doc["sensor"] = "gps";
        // JsonArray http_json_array = http_json_doc.to<JsonArray>();
        // JsonArray http_gpsdata_array = http_json_doc.createNestedArray("gpsdata");
        // JsonObject http_gpsdata = http_gpsdata_array.createNestedObject();

        int j = 0;
        for (int i = 0; i < GPS_CACHE_SIZE; i++) {
            if (gpsdata[i].age != TinyGPS::GPS_INVALID_AGE &&
                gpsdata[i].age != 0) {

                http_json_doc["gpsdata"][j]["lat"] = gpsdata[i].lat;
                http_json_doc["gpsdata"][j]["lon"] = gpsdata[i].lon;
                http_json_doc["gpsdata"][j]["age"] = gpsdata[i].age;
                http_json_doc["gpsdata"][j]["servo_angle"] = gpsdata[i].servo_angle;
                j++;
            }
        }

        req.setReqHeader("Content-Type", "application/x-www-form-urlencoded");

        String httpRequestData = "";
        serializeJson(http_json_doc, httpRequestData);
        if (print_debug)
            Serial.println(httpRequestData);

        requestOpenResult = req.open("POST", http_ctrl.PUSHLIGHT_SRV_ENDPOINT);

        if (requestOpenResult) {
            // Only send() if open() returns true, otherwise the program crashes
            bool httpSendSuccess = req.send(httpRequestData);

            // reset gps data buffer after successful transfer
            if (httpSendSuccess &&
                200 <= req.responseHTTPcode() < 300) {
                init_gps_data(gpsdata);
            }
            if (httpSendSuccess && print_debug)
                Serial.println(req.responseHTTPcode());
        }
    } else {
        Serial.println("http request not ready.");
    }
}

static void init_gps_data(t_gpsdata data[]) {
    for (int i = 0; i < GPS_CACHE_SIZE; i++) {
        data[i].lat = TinyGPS::GPS_INVALID_F_ANGLE;
        data[i].lon = TinyGPS::GPS_INVALID_F_ANGLE;
        data[i].age = TinyGPS::GPS_INVALID_AGE;
    }
}

static int count_gps_data_changes(t_gpsdata data[]) {
    int datachg_count = 0;
    for (int i = 0; i < GPS_CACHE_SIZE; i++) {
        if (data[i].age != 0 &&
            data[i].age != TinyGPS::GPS_INVALID_AGE) {
            datachg_count++;
        }
    }
    return datachg_count;
}