/*
 * rover_motor_controller.ino
 * Autonomous Delivery Rover — Arduino Uno Firmware
 *
 * Hardware:
 *   - Sabertooth 12A motor driver on pins 9 (CH1/left) and 11 (CH2/right)
 *     via Servo library (RC mode). Servo 90 = stop, >90 = fwd, <90 = rev.
 *   - HC-SR04 ultrasonic sensor: TRIG=7, ECHO=8
 *   - Serial at 9600 baud with Raspberry Pi
 *
 * Commands (single character):
 *   F  — Forward          → ACK:F
 *   b  — Reverse          → ACK:b
 *   R  — Pivot right      → ACK:R
 *   L  — Pivot left       → ACK:L
 *   r  — Gentle right     → ACK:r
 *   l  — Gentle left      → ACK:l
 *   S  — Stop             → ACK:S
 *   D  — Distance reading → D:<cm>  (or D:-1 on timeout)
 *   P  — Ping             → OK
 *
 * Speed values are intentionally conservative — closer to neutral (90)
 * means slower and safer indoors. Increase toward 105/78 if more speed needed.
 */

#include <Servo.h>

// ── Pin assignments ────────────────────────────────────────────────────────
const int PIN_LEFT   =  9;   // CH1 — left  motors (Sabertooth S1)
const int PIN_RIGHT  = 11;   // CH2 — right motors (Sabertooth S2)
const int PIN_TRIG   =  7;   // HC-SR04 trigger
const int PIN_ECHO   =  8;   // HC-SR04 echo

// ── Speed values (Servo degrees) ─────────────────────────────────────────
// Neutral = 90. Increase → faster forward. Decrease → faster reverse.
// Keep these close to 90 for safe indoor use.
const int SPD_STOP         = 90;

const int SPD_FORWARD      = 100;  // straight forward  (slowed from 105)
const int SPD_REVERSE      = 80;   // straight reverse  (slowed from 78)

// Pivot MUST be at least 10 units from neutral (90) to clear Sabertooth deadband.
// Values like 96/84 are inside the deadband and cause NO movement.
const int SPD_PIVOT_FWD    = 100;  // pivot: forward side
const int SPD_PIVOT_REV    = 80;   // pivot: reverse side

const int SPD_GENTLE_OUTER = 97;   // gentle curve: faster wheel
const int SPD_GENTLE_INNER = 93;   // gentle curve: slower wheel

// ── HC-SR04 timeout ───────────────────────────────────────────────────────
const unsigned long ECHO_TIMEOUT_US = 30000UL;   // 30 ms ≈ 5 m max

// ── Objects ───────────────────────────────────────────────────────────────
Servo leftMotors;
Servo rightMotors;

// ── Helpers ───────────────────────────────────────────────────────────────

inline void setMotors(int left, int right) {
  leftMotors.write(left);
  rightMotors.write(right);
}

long readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1L;
  return (duration * 34L) / 2000L;
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  leftMotors.attach(PIN_LEFT);
  rightMotors.attach(PIN_RIGHT);
  setMotors(SPD_STOP, SPD_STOP);   // safety: start stopped

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  delay(50);
}

// ── Main loop ─────────────────────────────────────────────────────────────
void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {

    // ── Forward ────────────────────────────────────────────────────────
    case 'F':
      setMotors(SPD_FORWARD, SPD_FORWARD);
      Serial.println("ACK:F");
      break;

    // ── Reverse ────────────────────────────────────────────────────────
    case 'b':
      setMotors(SPD_REVERSE, SPD_REVERSE);
      Serial.println("ACK:b");
      break;

    // ── Pivot right: left fwd, right rev ───────────────────────────────
    case 'R':
      setMotors(SPD_PIVOT_FWD, SPD_PIVOT_REV);
      Serial.println("ACK:R");
      break;

    // ── Pivot left: left rev, right fwd ───────────────────────────────
    case 'L':
      setMotors(SPD_PIVOT_REV, SPD_PIVOT_FWD);
      Serial.println("ACK:L");
      break;

    // ── Gentle right: both fwd, right side slightly slower ─────────────
    case 'r':
      setMotors(SPD_GENTLE_OUTER, SPD_GENTLE_INNER);
      Serial.println("ACK:r");
      break;

    // ── Gentle left: both fwd, left side slightly slower ──────────────
    case 'l':
      setMotors(SPD_GENTLE_INNER, SPD_GENTLE_OUTER);
      Serial.println("ACK:l");
      break;

    // ── Stop ───────────────────────────────────────────────────────────
    case 'S':
      setMotors(SPD_STOP, SPD_STOP);
      Serial.println("ACK:S");
      break;

    // ── Distance (HC-SR04) ─────────────────────────────────────────────
    case 'D': {
      long cm = readDistanceCm();
      Serial.print("D:");
      Serial.println(cm);
      break;
    }

    // ── Ping ───────────────────────────────────────────────────────────
    case 'P':
      Serial.println("OK");
      break;

    default:
      break;
  }
}
