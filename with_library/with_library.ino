#include <MPU6050_light.h>
#include <Wire.h>
#include <PID_v1.h>

MPU6050 mpu(Wire);

#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 6
#define IN3 5
#define IN4 4
#define MIN_SPEED 75

double setpoint = 178.8;
double Kp = 13.2;
double Ki = 97.8;
double Kd = 0.2;

double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    stopMotors();

    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("X فشل MPU: ");
        Serial.println(status);
        while (true);
    }

    Serial.println("معايرة... لا تحرك الروبوت");
    mpu.calcOffsets();
    Serial.println("OK جاهز");

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
}

void loop() {
    // ── 1 استقبال أوامر MATLAB ──
    if (Serial.available() > 0) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        char type = msg[0];
        double val = msg.substring(1).toDouble();
        if      (type == 'P') { Kp = val; Serial.print("Kp="); Serial.println(Kp); }
        else if (type == 'I') { Ki = val; Serial.print("Ki="); Serial.println(Ki); }
        else if (type == 'D') { Kd = val; Serial.print("Kd="); Serial.println(Kd); }
        else if (type == 'S') { setpoint = val; Serial.print("SP="); Serial.println(setpoint); }
        pid.SetTunings(Kp, Ki, Kd);
    }

    // ── 2 قراءة الزاوية ──
    mpu.update();
    input = mpu.getAngleX() + 180;

    // ── 3 منطقة خاملة صحيحة ──
    if (abs(setpoint - input) < 0.5) {
        stopMotors();
        Serial.println(input, 2);
        return;
    }

    // ── 4 حساب PID ──
    pid.Compute();

    // ── 5 تحريك المحركات ──
    if (input > 150 && input < 210) {
        if      (output > 0) forward(output);
        else if (output < 0) backward(output);
        else                 stopMotors();
    } else {
        stopMotors();
    }

    // ── 6 إرسال لـ MATLAB ──
    Serial.println(input, 2);

    delay(5);
}

void forward(double spd) {
    int s = map(abs(spd), 0, 255, MIN_SPEED, 255);
    s = constrain(s, MIN_SPEED, 255);
    analogWrite(ENA, s);
    analogWrite(ENB, s);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void backward(double spd) {
    int s = map(abs(spd), 0, 255, MIN_SPEED, 255);
    s = constrain(s, MIN_SPEED, 255);
    analogWrite(ENA, s);
    analogWrite(ENB, s);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}