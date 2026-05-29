#include <MPU6050_light.h>
#include <Wire.h>

class PID {
  private:
    double kp, ki, kd;
    double dispKp, dispKi, dispKd;

    double *myInput, *myOutput, *mySetpoint;

    double ITerm, lastInput;
    double outMin, outMax;
    unsigned long lastTime, SampleTime;

  public:
    PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd) {

    myInput    = Input;
    myOutput   = Output;
    mySetpoint = Setpoint;
    SampleTime = 10;

    SetOutputLimits(-255, 255);
    SetTunings(Kp, Ki, Kd);
    lastTime = millis() - SampleTime;
    }

    void Initialize(){
        ITerm     = *myOutput;
        lastInput = *myInput;
        if (ITerm > outMax) ITerm = outMax;
        if (ITerm < outMin) ITerm = outMin;
    }

    void Compute() {
        unsigned long now = millis();
        if ((now - lastTime) >= SampleTime) {

            double input = *myInput;
            double error = *mySetpoint - input;

            ITerm += (ki * error);
            if (ITerm > outMax) ITerm = outMax;
            else if (ITerm < outMin) ITerm = outMin;

            double dInput = (input - lastInput);
            double output = (kp * error) + ITerm - (kd * dInput);

            if (output > outMax) output = outMax;
            else if (output < outMin) output = outMin;

            *myOutput = output;
            lastInput = input;
            lastTime  = now;
        }
    }

    void SetTunings(double Kp, double Ki, double Kd) {
        if (Kp < 0 || Ki < 0 || Kd < 0) return;

        dispKp = Kp; dispKi = Ki; dispKd = Kd;

        double SampleTimeInSec = (double)SampleTime / 1000.0;
        kp = Kp;
        ki = Ki * SampleTimeInSec;
        kd = Kd / SampleTimeInSec;
    }

    void SetOutputLimits(double Min, double Max) {
        if (Min >= Max) return;
        outMin = Min;
        outMax = Max;

        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;
        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;
        
    }
};

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

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd);

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

    mpu.update();
    input = mpu.getAngleX() + 180;

    myPID.Initialize();
    myPID.SetOutputLimits(-255, 255);

}

void loop() {
    if (Serial.available() > 0) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        char type = msg[0];
        double val = msg.substring(1).toDouble();
        if      (type == 'P') { Kp = val; Serial.print("Kp="); Serial.println(Kp); }
        else if (type == 'I') { Ki = val; Serial.print("Ki="); Serial.println(Ki); }
        else if (type == 'D') { Kd = val; Serial.print("Kd="); Serial.println(Kd); }
        else if (type == 'S') { setpoint = val; Serial.print("SP="); Serial.println(setpoint); }
        myPID.SetTunings(Kp, Ki, Kd);
    }

    mpu.update();
    input = mpu.getAngleX() + 180;

    if (abs(setpoint - input) < 0.5) {
        stopMotors();
        Serial.println(input, 2);
        return;
    }

    myPID.Compute();

    if (input > 150 && input < 210) {
        if      (output > 0) forward(output);
        else if (output < 0) backward(output);
        else                 stopMotors();
    } else {
        stopMotors();
    }

    Serial.println(input, 2);

    delay(5);
}

void forward(double spd) {
    int s = map(abs(spd), 0, 255, MIN_SPEED, 255);
    s = constrain(s, MIN_SPEED, 255);
    analogWrite(ENA, s);
    analogWrite(ENB, s);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void backward(double spd) {
    int s = map(abs(spd), 0, 255, MIN_SPEED, 255);
    s = constrain(s, MIN_SPEED, 255);
    analogWrite(ENA, s);
    analogWrite(ENB, s);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}