/*
  rattracker_io.ino
  Digital I/O slave for the rat-tracker rig.

  Protocol (newline-terminated ASCII over Serial @ 115200 baud):
    PC -> Arduino:
      "PING"         -> Arduino replies "PONG"
      "D2".."D12"    -> Arduino sets pin HIGH for 500 ms, then LOW
      "D13"          -> Arduino TOGGLES pin (HIGH<->LOW, latches)
      anything else  -> Arduino replies "ERR <msg>"

    Arduino -> PC:
      On boot:        "READY" (so the PC can confirm the sketch is alive)
      For PING:       "PONG"
      For D2..D13:    no reply (fire-and-forget); use PING for liveness
      For garbage:    "ERR ..."

  Notes:
    D0, D1 are reserved for the USB-Serial bridge — do NOT use as outputs.
    All other digital pins are configured as OUTPUT, LOW at boot.
    For D13 the on-board LED reflects the toggle state.
*/

const uint8_t MIN_PIN = 2;
const uint8_t MAX_PIN = 13;
const uint8_t TOGGLE_PIN = 13;
const uint16_t PULSE_MS = 500;
const uint32_t BAUD = 115200;

// We don't block in delay() while pulsing; track each pulse independently
// so commands that arrive during a pulse on a different pin still fire.
// (A second command on the same pin while it's still pulsing extends the
// pulse — last-write-wins on the deadline.)
uint32_t pulse_deadline_ms[MAX_PIN + 1] = {0};

String inbuf;

void setup() {
  Serial.begin(BAUD);
  for (uint8_t p = MIN_PIN; p <= MAX_PIN; ++p) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  inbuf.reserve(16);
  // Give the host a moment to open the port, then announce.
  delay(50);
  Serial.println("READY");
}

void handle_command(const String& cmd) {
  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  // Expect "D<n>"
  if (cmd.length() < 2 || cmd[0] != 'D') {
    Serial.print("ERR unknown ");
    Serial.println(cmd);
    return;
  }

  long pin = cmd.substring(1).toInt();
  if (pin < MIN_PIN || pin > MAX_PIN) {
    Serial.print("ERR pin_out_of_range ");
    Serial.println(cmd);
    return;
  }

  if ((uint8_t)pin == TOGGLE_PIN) {
    // Toggle: read current state, flip it, write it back.
    int cur = digitalRead(TOGGLE_PIN);
    digitalWrite(TOGGLE_PIN, cur == HIGH ? LOW : HIGH);
    // Cancel any pending pulse deadline on D13 (toggle latches).
    pulse_deadline_ms[TOGGLE_PIN] = 0;
  } else {
    // Pulse: HIGH now, schedule LOW for now+500ms.
    digitalWrite((uint8_t)pin, HIGH);
    pulse_deadline_ms[pin] = millis() + PULSE_MS;
  }
}

void loop() {
  // Non-blocking pulse end check — handle each pin's deadline.
  uint32_t now = millis();
  for (uint8_t p = MIN_PIN; p <= MAX_PIN; ++p) {
    if (p == TOGGLE_PIN) continue;
    if (pulse_deadline_ms[p] != 0 && (int32_t)(now - pulse_deadline_ms[p]) >= 0) {
      digitalWrite(p, LOW);
      pulse_deadline_ms[p] = 0;
    }
  }

  // Read serial up to newline. Discard '\r' so CRLF works the same as LF.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      inbuf.trim();
      if (inbuf.length() > 0) {
        handle_command(inbuf);
      }
      inbuf = "";
    } else {
      inbuf += c;
      if (inbuf.length() > 24) {
        // Runaway input; reset to avoid heap thrash.
        inbuf = "";
      }
    }
  }
}
