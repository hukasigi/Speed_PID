#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder  myenc;
unsigned long encoderLastToggled;
bool          encoderPaused = false;

const int8_t PIN_ROTARY_A = 26;
const int8_t PIN_ROTARY_B = 27;
const int8_t PIN_PWM      = 25;
const int8_t PIN_DIR      = 33;

const double TARGET_RPM = 100.;
const double RESOLUTION = 4096.;

const double KP = 1.3;
const double KI = 1.2;
const double KD = 0.1;

double du        = 0;
double output    = 0;
double integral  = 0;
double deriv     = 0;
double prop      = 0;
double pre_prop  = 0;
double pre_error = 0;

volatile long rolls          = 0;
volatile long rolls_is       = 0;
volatile bool value_rotary_b = 0;
volatile long speed          = 0;

static unsigned long last       = 0;
static double        lastRpm    = 0;
static long          prev_count = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
    // 読み込みを安定させるための呪文
    portENTER_CRITICAL_ISR(&timerMux);
    rolls_is = rolls;
    rolls    = 0;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ROTARY_A, INPUT);
    pinMode(PIN_ROTARY_B, INPUT);
    pinMode(PIN_DIR, OUTPUT);

    ledcSetup(0, 20000, 8);
    ledcAttachPin(PIN_PWM, 0);

    ESP32Encoder::useInternalWeakPullResistors = puType::none;
    myenc.attachHalfQuad(PIN_ROTARY_A, PIN_ROTARY_B);

    myenc.clearCount();

    // timer0使用して、分周80 1カウント1us
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);

    // 10,000 10ms
    timerAlarmWrite(timer, 10000, true);

    // timer開始
    timerAlarmEnable(timer);
}

void loop() {
    unsigned long now_t = micros();

    if (now_t - last >= 10000) { // 10ms
        long   now_count;
        double dt = (double)(now_t - last) / 1.e6;
        last      = now_t;
        portENTER_CRITICAL(&timerMux);
        now_count = myenc.getCount();
        portEXIT_CRITICAL(&timerMux);

        long delta = now_count - prev_count;
        prev_count = now_count;

        double rpm = (double)delta / RESOLUTION * 60. / dt;

        // 目標値 - 実測値でエラーを出す
        double error = TARGET_RPM - rpm;

        double prop  = error - pre_error;
        double deriv = prop - pre_prop;
        double du    = KP * prop + KI * error * dt + KD * deriv;
        output += du;
        digitalWrite(PIN_DIR, output >= 0 ? HIGH : LOW);
        double clamped_output = constrain(abs(output), 0., 255.);
        ledcWrite(0, clamped_output);

        Serial.printf("RPM: %.1f  PWM: %.1f ERROR : %.1f\n", rpm, clamped_output, error);

        pre_error = error;
        pre_prop  = prop;
    }
}