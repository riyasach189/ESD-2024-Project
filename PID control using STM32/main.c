#include <stdint.h>
#include <stdio.h>

// PID structure
typedef struct {
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain

    float setpoint; // Desired value
    float integral; // Integral error
    float lastError; // Last error for derivative calculation
    float outputLimit; // Limit for the PID output (e.g., max motor speed)
} PIDController;

// Function to initialize a PID controller
void PID_Init(PIDController* pid, float kp, float ki, float kd, float outputLimit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0;
    pid->integral = 0;
    pid->lastError = 0;
    pid->outputLimit = outputLimit;
}

// PID update function, returns control output
float PID_Update(PIDController* pid, float current) {
    float error = pid->setpoint - current;
    pid->integral += error; // Integrate error
    float derivative = error - pid->lastError; // Calculate derivative of error

    // PID output before limiting
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // Limit output to max specified
    if(output > pid->outputLimit) output = pid->outputLimit;
    if(output < -pid->outputLimit) output = -pid->outputLimit;

    pid->lastError = error; // Save error for next derivative calculation

    return output;
}

// Example function to set motor speed (to be implemented based on your hardware)
void setMotorSpeed(float speed) {
    // Implementation depends on your specific motor controller and hardware setup
    printf("Setting motor speed to: %f\n", speed);
}

// Example usage
int main() {
    PIDController motorPID;
    PID_Init(&motorPID, 0.1, 0.01, 0.05, 100.0); // Initialize PID controller with arbitrary values

    motorPID.setpoint = 50; // Desired speed or position

    // Simulate a loop where we update motor speed based on current position or speed
    // This would realistically be triggered by a timer or main control loop
    for(int i = 0; i < 100; i++) {
        float currentSpeed = 50; // Example current speed, replace with actual sensor/encoder reading
        float controlSignal = PID_Update(&motorPID, currentSpeed);
        setMotorSpeed(controlSignal);
    }

    return 0;
}
