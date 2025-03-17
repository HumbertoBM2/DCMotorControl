#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>

// --------------------------------------------------------------------
// Pin Definitions for L298N
// --------------------------------------------------------------------
#define PWM_PIN   33    // Enable pin (PWM) on L298N
#define DIR_PIN1  25    // IN1 on L298N
#define DIR_PIN2  26    // IN2 on L298N
#define LED_PIN   32    // Optional debug LED

// --------------------------------------------------------------------
// Encoder Pins
// --------------------------------------------------------------------
#define ENCODER_A_PIN 27
#define ENCODER_B_PIN 14

// Pulses per revolution (adjust to your hardware)
#define PULSES_PER_REV 1024.0

// --------------------------------------------------------------------
// PWM Configuration on ESP32
// --------------------------------------------------------------------
#define PWM_CHANNEL     0
#define PWM_FREQUENCY   20000   // 20 kHz
#define PWM_RESOLUTION  8       // 8-bit => 0..255

// --------------------------------------------------------------------
// Deadband and PWM Mapping Parameters
// --------------------------------------------------------------------
#define DEAD_BAND_THRESHOLD 0.1    // Normalized threshold below which motor is off
#define MIN_PWM 180                // PWM corresponding to normalized 0.1 (or -0.1)
#define MAX_PWM 255                // PWM corresponding to normalized 1.0 (or -1.0)

// --------------------------------------------------------------------
// micro-ROS Entities
// --------------------------------------------------------------------
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Subscription to /set_point
rcl_subscription_t subscriber;
std_msgs__msg__Float64 setpoint_msg;

// Publisher for motor speed (output)
rcl_publisher_t speed_pub;

// Timer for PID + speed publication
rcl_timer_t control_timer;

// --------------------------------------------------------------------
// Timing and Encoder Variables
// --------------------------------------------------------------------
static volatile long encoder_count = 0;
static long last_encoder_count = 0;
static const float dt = 0.035f;  // Sampling time: 35 ms

// --------------------------------------------------------------------
// PID Variables
// --------------------------------------------------------------------
static double setpoint      = 0.0;
static double current_speed = 0.0;
static double error_prev    = 0.0;
static double integral      = 0.0;

// Example PID gains (tune as needed)
static double Kp = 15.0;
static double Ki = 9.0;
static double Kd = 1.0;

// Maximum speed in rev/s; from your test, MAX_RPS = 0.6 rev/s
static const double MAX_RPS = 3;

// --------------------------------------------------------------------
// Larger Moving Average Filter for Speed Measurement
// --------------------------------------------------------------------
#define AVERAGE_SIZE 45
static double speed_buffer[AVERAGE_SIZE];
static int speed_index = 0;
static bool buffer_full = false;

// --------------------------------------------------------------------
// Helper: clamp()
// --------------------------------------------------------------------
static double clamp(double val, double min_val, double max_val) {
  if (val > max_val) return max_val;
  if (val < min_val) return min_val;
  return val;
}

// --------------------------------------------------------------------
// set_motor_speed() with Deadband Compensation
// --------------------------------------------------------------------
void set_motor_speed(double value)
{
  // Clamp to [-1, 1]
  value = clamp(value, -1.0, 1.0);
  
  int pwm_value = 0;
  double abs_value = fabs(value);
  
  // If within deadband, output zero PWM
  if (abs_value < DEAD_BAND_THRESHOLD) {
    pwm_value = 0;
  } else {
    // Map normalized value from [DEAD_BAND_THRESHOLD, 1] to [MIN_PWM, MAX_PWM]
    // First, shift the range so DEAD_BAND_THRESHOLD becomes 0:
    double normalized = (abs_value - DEAD_BAND_THRESHOLD) / (1.0 - DEAD_BAND_THRESHOLD);
    normalized = clamp(normalized, 0.0, 1.0);
    pwm_value = MIN_PWM + (int)(normalized * (MAX_PWM - MIN_PWM));
  }
  
  // Set motor direction based on the sign of value
  if (value > 0.0) {
    digitalWrite(DIR_PIN1, HIGH);   // Forward
    digitalWrite(DIR_PIN2, LOW);
  } else if (value < 0.0) {
    digitalWrite(DIR_PIN1, LOW);    // Reverse
    digitalWrite(DIR_PIN2, HIGH);
  } else {
    digitalWrite(DIR_PIN1, LOW);
    digitalWrite(DIR_PIN2, LOW);
  }
  
  // Apply PWM using LEDC
  ledcWrite(PWM_CHANNEL, pwm_value);
  
  // Debug print
  Serial.print("Applied PWM: ");
  Serial.println(pwm_value);
}

// --------------------------------------------------------------------
// Encoder ISR: Basic quadrature on rising edge of A
// --------------------------------------------------------------------
void IRAM_ATTR encoderISR()
{
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);
  encoder_count += (a == b) ? 1 : -1;
}

// --------------------------------------------------------------------
// setpoint_callback()
// Called when a new Float64 arrives on /set_point
// --------------------------------------------------------------------
void setpoint_callback(const void * msg_in)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msg_in;
  setpoint = msg->data;
  Serial.print("Received setpoint: ");
  Serial.println(setpoint);
}

// --------------------------------------------------------------------
// control_timer_callback()
// Runs every dt, performs:
//   1) Speed measurement with moving average filtering
//   2) PID calculation with anti-windup
//   3) Motor control and speed publication
// --------------------------------------------------------------------
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  if (timer == NULL) return;
  
  // 1) Measure instant speed (rev/s)
  long current_count_local = encoder_count;
  long delta = current_count_local - last_encoder_count;
  last_encoder_count = current_count_local;
  double instant_speed = (delta / PULSES_PER_REV) / dt;
  
  // 2) Store in ring buffer for moving average filtering
  speed_buffer[speed_index] = instant_speed;
  speed_index++;
  if (speed_index >= AVERAGE_SIZE) {
    speed_index = 0;
    buffer_full = true;
  }
  
  double sum = 0.0;
  int count = buffer_full ? AVERAGE_SIZE : speed_index;
  for (int i = 0; i < count; i++) {
    sum += speed_buffer[i];
  }
  current_speed = sum / count;  // Filtered speed (rev/s)
  
  // 3) PID Control Calculation
  // Convert normalized setpoint to desired speed (rev/s)
  double sp_rps = setpoint * MAX_RPS;
  double error = sp_rps - current_speed;
  
  // Anti-windup: reset integral when setpoint is zero
  if (setpoint == 0.0) {
    integral = 0.0;
  } else {
    double potential_integral = integral + error * dt;
    double max_integral = MAX_RPS / Ki;
    if (fabs(potential_integral) < max_integral) {
      integral = potential_integral;
    }
  }
  
  double derivative = (error - error_prev) / dt;
  double control_signal = Kp * error + Ki * integral + Kd * derivative;
  error_prev = error;
  
  // Normalize control signal (expected to be within [-1,1])
  double max_control_signal = MAX_RPS * Kp;
  double normalized_control = clamp(control_signal / max_control_signal, -1.0, 1.0);
  
  // 4) Apply motor control using deadband-compensated mapping
  set_motor_speed(normalized_control);
  
  // 5) Publish the filtered speed
  std_msgs__msg__Float64 speed_msg;
  speed_msg.data = current_speed;
  rcl_publish(&speed_pub, &speed_msg, NULL);
  
  // Debug output
  Serial.print("Speed = ");
  Serial.print(current_speed);
  Serial.print(" | Error = ");
  Serial.print(error);
  Serial.print(" | Integral = ");
  Serial.print(integral);
  Serial.print(" | Control = ");
  Serial.print(control_signal);
  Serial.print(" | Normalized = ");
  Serial.println(normalized_control);
}

// --------------------------------------------------------------------
// setup()
// Initializes micro-ROS, PWM, pins, subscription, and timer
// --------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(2000);
  set_microros_transports();
  
  // Pin setup
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);
  
  // Setup PWM with LEDC on ESP32
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  
  // micro-ROS initialization
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_node", "", &support);
  
  // Subscription to /set_point
  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "/set_point");
  
  // Publisher for motor speed on /motor_output
  rclc_publisher_init_default(&speed_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "/motor_output");
  
  // Create timer for control loop (PID + publishing) every dt ms
  rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS((int)(dt * 1000)), control_timer_callback);
  
  // Setup executor with 2 handles (1 subscription + 1 timer)
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &setpoint_msg, &setpoint_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &control_timer);
}

// --------------------------------------------------------------------
// loop()
// Runs micro-ROS executor callbacks
// --------------------------------------------------------------------
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}