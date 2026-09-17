/*
 * Mecanum AGV low-level firmware: PID velocity control for 4 wheels + encoder feedback.
 * Target board: Arduino MEGA 2560 (needs 4 interrupt-capable pins for the encoders;
 * an Uno only has 2, which isn't enough for 4 wheels).
 *
 * Serial protocol (matches mecanum_drive_node.py), one line per message, newline-terminated:
 *   RX "V <w_fl> <w_fr> <w_rl> <w_rr>\n"  target wheel angular velocities, rad/s (output shaft, post-gearbox)
 *   TX "E <w_fl> <w_fr> <w_rl> <w_rr>\n"  measured wheel angular velocities, rad/s
 *
 * !!! EVERYTHING BELOW MARKED "PLACEHOLDER" MUST BE UPDATED FOR YOUR ACTUAL WIRING !!!
 */

// ---- PLACEHOLDER: motor driver pins (PWM + DIR per wheel, e.g. BTS7960 / L298N) ----
const int MOTOR_FL_PWM = 4,  MOTOR_FL_DIR = 22;
const int MOTOR_FR_PWM = 5,  MOTOR_FR_DIR = 24;
const int MOTOR_RL_PWM = 6,  MOTOR_RL_DIR = 26;
const int MOTOR_RR_PWM = 7,  MOTOR_RR_DIR = 28;

// ---- PLACEHOLDER: encoder pins — must be interrupt-capable (Mega: 2,3,18,19,20,21) ----
// Only counting one channel per encoder (magnitude only); direction is inferred from the
// last commanded direction, not decoded from quadrature. Upgrade to full A/B decoding if
// you need to detect stalls or the wheel being back-driven against the commanded direction.
const int ENCODER_FL_PIN = 18;
const int ENCODER_FR_PIN = 19;
const int ENCODER_RL_PIN = 20;
const int ENCODER_RR_PIN = 21;

// ---- PLACEHOLDER: encoder ticks per output-shaft revolution (after gear reduction) ----
// Check your motor stack's datasheet — this varies a lot (a few hundred to ~1000+ CPR).
const float TICKS_PER_REV = 980.0;

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
float g_last_sign[4] = {0.0, 0.0, 0.0, 0.0};
WheelPid g_pid[4];

unsigned long g_last_control_ms = 0;
String g_serial_buffer;

void isrFL() { g_ticks[0]++; }
void isrFR() { g_ticks[1]++; }
void isrRL() { g_ticks[2]++; }
void isrRR() { g_ticks[3]++; }

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_FL_PWM, OUTPUT); pinMode(MOTOR_FL_DIR, OUTPUT);
  pinMode(MOTOR_FR_PWM, OUTPUT); pinMode(MOTOR_FR_DIR, OUTPUT);
  pinMode(MOTOR_RL_PWM, OUTPUT); pinMode(MOTOR_RL_DIR, OUTPUT);
  pinMode(MOTOR_RR_PWM, OUTPUT); pinMode(MOTOR_RR_DIR, OUTPUT);

  pinMode(ENCODER_FL_PIN, INPUT_PULLUP);
  pinMode(ENCODER_FR_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RL_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PIN), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PIN), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RL_PIN), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RR_PIN), isrRR, RISING);

  g_last_control_ms = millis();
}

void setMotor(int pwm_pin, int dir_pin, float command) {
  bool forward = command >= 0.0;
  digitalWrite(dir_pin, forward ? HIGH : LOW);
  int pwm = constrain((int)fabs(command), 0, 255);
  analogWrite(pwm_pin, pwm);
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
  for (int i = 0; i < 4; i++) {
    g_target_w[i] = w[i];
    if (w[i] != 0.0) g_last_sign[i] = (w[i] > 0.0) ? 1.0 : -1.0;
  }
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
  const int pwm_pins[4] = {MOTOR_FL_PWM, MOTOR_FR_PWM, MOTOR_RL_PWM, MOTOR_RR_PWM};
  const int dir_pins[4] = {MOTOR_FL_DIR, MOTOR_FR_DIR, MOTOR_RL_DIR, MOTOR_RR_DIR};

  for (int i = 0; i < 4; i++) {
    long delta_ticks = ticks_now[i] - g_last_ticks[i];
    g_last_ticks[i] = ticks_now[i];
    float speed_mag = (delta_ticks / TICKS_PER_REV) * 2.0 * PI / dt;
    measured_w[i] = speed_mag * g_last_sign[i];

    float command = updatePid(g_pid[i], g_target_w[i], measured_w[i], dt);
    setMotor(pwm_pins[i], dir_pins[i], command);
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
