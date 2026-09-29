/*
 * Mecanum AGV low-level firmware: PID velocity control for 4 wheels + encoder feedback.
 * Hardware: Arduino MEGA 2560, 2x L298N, 4x JGB37-520 12V 333RPM gearmotor with hall encoder,
 * 80 mm mecanum wheels. The Mega is needed for its 4+ interrupt-capable encoder pins (Uno has 2).
 *
 * Serial protocol (matches mecanum_drive_node.py), one line per message, newline-terminated:
 *   RX "V <w_fl> <w_fr> <w_rl> <w_rr>\n"  target wheel angular velocities, rad/s (output shaft, post-gearbox)
 *   TX "E <w_fl> <w_fr> <w_rl> <w_rr>\n"  measured wheel angular velocities, rad/s
 *
 * !!! EVERYTHING BELOW MARKED "PLACEHOLDER" MUST BE UPDATED FOR YOUR ACTUAL WIRING !!!
 */

// ---- Motor drivers: 2x L298N, one EN (PWM) + IN1/IN2 (direction) per wheel ----
// Remove the ENA/ENB jumpers on the L298N boards so EN can be PWM-driven.
// PLACEHOLDER pins: copied from the previous group's robot (dabom config.h) — check against your wiring.
const int MOTOR_EN[4]  = {2, 3, 4, 5};      // FL, FR, RL, RR — must be PWM pins
const int MOTOR_IN1[4] = {35, 41, 31, 45};
const int MOTOR_IN2[4] = {37, 39, 33, 43};
// Flip a wheel here if it spins backwards for a positive command (mirrored mounting on one side).
const bool MOTOR_INVERT[4] = {true, false, true, false};

// ---- Encoders: JGB37-520 hall encoder, channels A + B ----
// A must be interrupt-capable (Mega: 2,3,18,19,20,21); B can be any digital pin.
const int ENCODER_A[4] = {18, 19, 20, 21};
const int ENCODER_B[4] = {23, 25, 27, 29};

// JGB37-520 12V 333RPM: 11 pulses/motor rev, 1:30 gearbox. Counting both edges of A
// (x2 decoding) gives 11 * 2 * 30 = 660 counts per output-shaft revolution.
// Verify by turning a wheel exactly 10 revolutions by hand and dividing the count by 10.
const float TICKS_PER_REV = 660.0;

const unsigned long CONTROL_INTERVAL_MS = 50;  // 20 Hz PID + odometry report rate

// ---- PLACEHOLDER: PID gains — tune these on the actual robot ----
const float KP = 2.0, KI = 5.0, KD = 0.0;
const float PID_OUTPUT_LIMIT = 255.0;  // PWM range

struct WheelPid {
  float integral = 0.0;
  float prev_error = 0.0;
};

volatile long g_ticks[4] = {0, 0, 0, 0};  // FL, FR, RL, RR
long g_last_ticks[4] = {0, 0, 0, 0};
float g_target_w[4] = {0.0, 0.0, 0.0, 0.0};
WheelPid g_pid[4];

unsigned long g_last_control_ms = 0;
String g_serial_buffer;

// On each edge of A: if A == B the shaft turns one way, otherwise the other.
void countEdge(int i) {
  bool a = digitalRead(ENCODER_A[i]);
  bool b = digitalRead(ENCODER_B[i]);
  long step = (a == b) ? 1 : -1;
  g_ticks[i] += MOTOR_INVERT[i] ? -step : step;
}
void isrFL() { countEdge(0); }
void isrFR() { countEdge(1); }
void isrRL() { countEdge(2); }
void isrRR() { countEdge(3); }

void setup() {
  Serial.begin(115200);

  void (*isrs[4])() = {isrFL, isrFR, isrRL, isrRR};
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_EN[i], OUTPUT);
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(ENCODER_A[i], INPUT_PULLUP);
    pinMode(ENCODER_B[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A[i]), isrs[i], CHANGE);
  }

  g_last_control_ms = millis();
}

void setMotor(int i, float command) {
  if (MOTOR_INVERT[i]) command = -command;
  bool forward = command >= 0.0;
  digitalWrite(MOTOR_IN1[i], forward ? HIGH : LOW);
  digitalWrite(MOTOR_IN2[i], forward ? LOW : HIGH);
  int pwm = constrain((int)fabs(command), 0, 255);
  analogWrite(MOTOR_EN[i], pwm);
}

float updatePid(WheelPid &pid, float target, float measured, float dt) {
  float error = target - measured;
  pid.integral += error * dt;
  float derivative = (dt > 0.0) ? (error - pid.prev_error) / dt : 0.0;
  pid.prev_error = error;
  float output = KP * error + KI * pid.integral + KD * derivative;
  return constrain(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
}

void parseSerialLine(const String &line) {
  if (!line.startsWith("V ")) return;
  float w[4];
  int start = 2;
  bool ok = true;
  for (int i = 0; i < 4; i++) {
    int next_space = (i < 3) ? line.indexOf(' ', start) : line.length();
    if (next_space < 0) { ok = false; break; }
    w[i] = line.substring(start, next_space).toFloat();
    start = next_space + 1;
  }
  if (!ok) return;
  for (int i = 0; i < 4; i++) g_target_w[i] = w[i];
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseSerialLine(g_serial_buffer);
      g_serial_buffer = "";
    } else if (c != '\r') {
      g_serial_buffer += c;
    }
  }
}

void runControlLoop() {
  unsigned long now_ms = millis();
  float dt = (now_ms - g_last_control_ms) / 1000.0;
  if (dt <= 0.0) return;
  g_last_control_ms = now_ms;

  long ticks_now[4];
  noInterrupts();
  for (int i = 0; i < 4; i++) ticks_now[i] = g_ticks[i];
  interrupts();

  float measured_w[4];

  for (int i = 0; i < 4; i++) {
    long delta_ticks = ticks_now[i] - g_last_ticks[i];
    g_last_ticks[i] = ticks_now[i];
    measured_w[i] = (delta_ticks / TICKS_PER_REV) * 2.0 * PI / dt;

    float command = updatePid(g_pid[i], g_target_w[i], measured_w[i], dt);
    setMotor(i, command);
  }

  Serial.print("E ");
  Serial.print(measured_w[0], 4); Serial.print(' ');
  Serial.print(measured_w[1], 4); Serial.print(' ');
  Serial.print(measured_w[2], 4); Serial.print(' ');
  Serial.println(measured_w[3], 4);
}

void loop() {
  readSerial();
  if (millis() - g_last_control_ms >= CONTROL_INTERVAL_MS) {
    runControlLoop();
  }
}
