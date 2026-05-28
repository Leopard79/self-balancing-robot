#include <MPU6050_light.h>
#include <Wire.h>

MPU6050 mpu(Wire);

#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 6
#define IN3 5
#define IN4 4
#define MIN_SPEED 75

double setpoint = 178.8;
double Kp = 13.2;//13.2
double Ki = 97.8;//97.8
double Kd = 0.2;//0.33

double input, output;
double error, lastError = 0;
double P, I = 0, D;
unsigned long lastTime = 0;

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
    }

    // ── 2 قراءة الزاوية ──
    mpu.update();
    input = mpu.getAngleX() + 180;

    // ── 3 حساب PID ──
    unsigned long now = millis();
    double dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    lastTime = now;

    error = setpoint - input;

    // منطقة خاملة
    if (abs(error) < 0.5) {
        error = 0;
        I = 0;
    }

    P  = Kp * error;
    I += Ki * error * dt;
    I  = constrain(I, -255, 255);
    D  = Kd * (error - lastError) / dt;

    output = P + I + D;
    output = constrain(output, -255, 255);
    lastError = error;

    // ── 4 تحريك المحركات ──
    if (input > 150 && input < 210) {
        if      (output > 0) forward(output);
        else if (output < 0) backward(output);
        else                 stopMotors();
    } else {
        stopMotors();
    }

    // ── 5 إرسال لـ MATLAB ──
    Serial.println(input, 2);

    delay(10);
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
