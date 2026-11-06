// gantry_fw 2.0: motion firmware for the gantry robot's ESP32-C3-Zero.
//
// Everything that moves the gantry runs here: the three axis controllers, the sensor noise filter,
// homing, the end-stop checks and the step test. The Raspberry Pi Zero W (Gantry CLI) only reads the
// five limit switches, forwards their states with SW lines, and sends one command at a time.
// The BOOT button (GPIO9) is not used.
//
// ------------------------------------------------------------------------------------------------------
// PROTOCOL (115200 8N1, one line per command, '\n' terminated). Unknown lines are ignored without a reply,
// so boot text from either side can never start anything.
//
//   PING                   -> PONG
//   POS?                   -> OK X<mm> Y<mm> Z<mm> HOMED=<axes or -> SW=<5 bits | off | stale>
//   JOG Y+10               -> closed-loop relative move of one axis (unhomed: at most 50 mm, no soft limits)
//   MOVE X300 Y500         -> closed-loop absolute move (homed axes only)
//   HOME [XYZ]             -> home the named axes (all if none), order Z X Y: drive each axis to its switch (Z top, X min, Y min);
//                             homed = switch pressed, the count there is 0, no back-off, no park. With SET SW 0 (no switches)
//                             the current position becomes 0 and nothing moves.
//   STEP Y 3.0 + 100 [30]  -> step test: wait until the axis is still, then drive Y at 3.0 V in + direction
//                             until 100 filtered pulses, 30 s, a stall (no pulse for 2 s), the end switch or STOP.
//                             Every raw rising edge on the sensor pin (unfiltered) is timestamped in us from the
//                             moment the voltage was applied. Reply: D <k>,<t_us> for each edge (k from 1), then
//                             OK STEP Y edges=<n> pulses=<filtered> end=<ticks|timeout|stall|limit|stop|sw_lost>
//                             t_end=<us> lost=<edges past the buffer>
//   SPAN X [passes]        -> X or Y: drive min, max, min ... switch to switch (2 x passes + 1 legs) at the homing
//                             fast voltage. Reply OK SPAN X C=<raw count at each trip, comma separated>
//   SPRAY 250              -> pump (and valve) on for 250 ms (1..3000). OK SPRAY 250
//   ESTOP / RESET          -> brake, pump off, latch: motion and spray refused (ERR ESTOP latched) until RESET
//   ABORT                  -> end the running command without a reply to it, brake, pump off. OK ABORT
//   PAUSE / RESUME         -> OK PAUSE / OK RESUME (the Pi pauses between commands)
//   STOP                   -> brake everything now. The running command ends with ERR STOPPED (a step test still
//                             sends its data, end=stop), then OK STOP X.. Y.. Z..
//   SW 01001               -> switch states from the Pi: x_min x_max y_min y_max z_top, 1 = pressed. No reply.
//                             The Pi sends it on every change and at least every 50 ms; none for 0.5 s = ERR SW_LOST.
//   KA                     -> keep-alive from the Pi when no switches are connected. No reply.
//   SET SW 1|0             -> limit switches connected or not
//   SET MMPC X 1.0661      -> mm per count             SET SOFT X 5 615  -> soft limits (homed axes)
//   SET GAIN X kp kd vlim v0  -> controller gains      SET SIGN X -1     -> flip the motor direction
//   SET RAIL 12            -> motor supply [V]
//   SET STALLV X 3.0       -> stall check voltage of one axis (0 = off for that axis); SET STALLT 1000 -> stall time [ms]
//                             (the old one-number form SET STALLV 2.5 is accepted and ignored: one voltage for all
//                             three axes would sit below the Y dead band and trip on a healthy axis)
//   SET HOME 4 2.5 5       -> homing: approach V (the first number; the other two are accepted, unused by HOME) [the park offset is used by SPAN's Pi-side park]
//   CFG?                   -> OK CFG ... (active settings)
// While a command runs: PROG X.. Y.. Z.. (or PROG STEP n=<pulses>) every 0.5 s.
// Unprompted: BOOT gantry-fw <version> after a reset.
// Errors: ERR ESTOP latched · ERR BUSY · ERR BAD_ARG · ERR NOT_HOMED <axis> · ERR SOFT_LIMIT <axis> · ERR LIMIT <switch> ·
//         ERR TIMEOUT (a move took longer than 15 s + its distance at 3 mm/s, Z 1 mm/s) · ERR HOME <axis> <why> · ERR SW_LOST (no SW line for 0.5 s) ·
//         ERR WATCHDOG (nothing from the Pi for 1 s during a command) · ERR STOPPED ·
//         ERR STALL <axis> (driven at or above its stall voltage with no pulse for the stall time; the axis is un-homed)
//
// SENSORS: the pins are sampled every loop pass. A level has to hold for a net 1 ms before it counts, counts
// closer than 3 ms are dropped, and an axis that has not been driven for 0.6 s ignores its sensor. One count
// per slot, credited in the direction the axis was last driven (single-channel sensors). The step test also
// logs the raw edges through an interrupt, with no filter at all.
//
// CONTROLLER (every 5 ms, every axis): u = clamp(V0 + Kp*e - Kd*v + nudge, 0, Vlim), slew 50 V/s, never
// driven against the move, nudge +2 V/s after 150 ms without a pulse, brake within one count of the target.
// STALL DETECTION (2.5): during a move, an axis whose commanded voltage is at or above its stall voltage (default
// the identified dead band + 1 V: X 3.0, Y 4.1, Z 3.3 V) and that gives no counted pulse for the stall time
// (default 1 s, about nine pulse intervals at that voltage) is stopped with ERR STALL <axis> and un-homed. Below the
// stall voltage a missing pulse is normal (the axis is creeping onto the target or the nudge is building up), so
// it is not judged. Homing approaches and span legs drive at the homing voltage and stop with ERR HOME <axis> stall /
// ERR SPAN <axis> stall after 2 s without any sensor change. The move timeout and the 180 s approach limit stay as
// a second line of defence. The step test ends with end=stall after 2 s without a pulse, as before.
//
// PINS: sensors X GPIO8, Y GPIO19, Z GPIO7 · L298 board 1: Y IN1/IN2 GPIO0/1, X IN3/IN4 GPIO2/3 ·
//   L298 board 2: Z IN1/IN2 GPIO4/5, pump IN3 GPIO6 (IN4 to GND; held low) · Pi UART1 RX GPIO20, TX GPIO21.
// GPIO19 is also the USB D+ pin (GPIO18 is D-). Once this sketch configures it as the Y sensor input, the native USB port
//   (USB serial / bench commands, the Serial monitor) is expected to stop working: the Pi link on UART1 is not affected.
//   To flash again, hold BOOT while resetting or plugging in the USB cable (ROM download mode), because the running sketch no longer
//   enumerates on USB. While a PC is plugged into the USB port its D+ line is pulled to 3.3 V and fights the sensor: run with USB unplugged.
//   Not verified on hardware. GPIO10 (RGB LED) and GPIO9 (BOOT, strapping) are not used for the Y sensor.
// CHANGES (version string kept at 2.2 so the Pi's CFG? check still matches): Y sensor GPIO10 -> GPIO19; default gains from the controller design
//   (controller_design.md): X [0.35 0.053 6 2.46], Y [0.637 0.051 8 3.97], Z [1.7 0.391 5 3.1] as [Kp Kd Vlim V0]. The Pi still overrides them with SET GAIN.
// ARDUINO IDE: board "ESP32C3 Dev Module", Tools > USB CDC On Boot = Enabled. esp32 core 3.x.
// USB serial accepts the same commands for bench tests; replies go to the port that sent the command.

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(ARDUINO_USB_CDC_ON_BOOT) && !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot = Enabled. Otherwise Serial is UART0 on GPIO20/21, the same pins as the Pi link."
#endif

#define FW_VERSION "2.5"

// ---------- types (all above the first function: the Arduino IDE inserts prototypes there) ----------
struct AxisCfg { char name; int sensorPin; int inA; int inB; float mmpc; int sign; float kp; float kd; float vlim; float v0; float lo; float hi; float stallV; };
struct AxisRt {
  int32_t counts; uint32_t ticks; uint32_t lastTickUs; uint32_t prevTickUs; int8_t credit;
  bool driving; uint32_t lastDriveMs; uint32_t lastLevelMs;
  int32_t zero; bool homed;
  bool active; float target; int dir; float u; float nudge; uint32_t seenTicks; uint32_t lastChangeMs; uint32_t startMs;
  uint32_t hardSinceMs;                                       // when u last rose to the stall voltage (0 = below it)
};
struct SensorFilt { uint32_t lastUs; int32_t acc; bool lvl; };
struct Port { Stream *s; bool isPi; char buf[72]; uint8_t n; bool overflow; };
enum Job : uint8_t { J_IDLE, J_MOVE, J_HOME, J_STEP, J_SPAN, J_SPRAY };
enum HomePhase : uint8_t { H_FAST };
enum StepPhase : uint8_t { S_STILL, S_DRIVE };
enum AxisResult : uint8_t { AR_RUNNING, AR_DONE, AR_LIMIT, AR_STALL };

// ---------- hardware ----------
AxisCfg CFG[3] = {
  // name sensor inA inB  mm/count  sign  Kp    Kd    Vlim  V0    soft lo  soft hi  stall V   (the Pi overrides these with SET)
  {'X', 8,  2, 3, 1.0661f, 1, 0.35f, 0.053f, 6.0f, 2.46f, 5.0f, 615.0f,  3.0f},
  {'Y', 19, 0, 1, 1.0661f, 1, 0.637f, 0.051f, 8.0f, 3.97f, 5.0f, 1055.0f, 4.1f},
  {'Z', 7,  4, 5, 0.2667f, 1, 1.70f, 0.391f, 5.0f, 3.10f, 5.0f, 245.0f,  3.3f},
};
const int PUMP_PIN = 6;
const int PI_RX_PIN = 20;
const int PI_TX_PIN = 21;
const uint32_t PI_BAUD = 115200;
// limit switches, as bits in the SW line: x_min x_max y_min y_max z_top
const char *SW_NAMES[5] = {"x_min", "x_max", "y_min", "y_max", "z_top"};
const int MIN_SW[3] = {0, 2, 4};     // the switch at the - end of each axis (Z is measured down: - is up)
const int MAX_SW[3] = {1, 3, -1};    // the switch at the + end (Z has none)

// ---------- timing and limits ----------
const int32_t FILT_US = 1000;            // a sensor level must hold (net) 1 ms to count
const uint32_t FILT_DT_MAX_US = 250;     // one sample never adds more than this
const uint32_t MIN_PERIOD_US = 3000;     // two counts closer than this are noise
const uint32_t COAST_MS = 600;           // an axis not driven for this long ignores its sensor
const uint32_t CTRL_US = 5000;
const uint32_t MOVE_TIMEOUT_BASE_MS = 15000;   // a move may take 15 s plus its distance at the slowest speed below
const float MOVE_MIN_SPEED_XY = 3.0f;          // mm/s
const float MOVE_MIN_SPEED_Z = 1.0f;           // mm/s
const uint32_t SETTLE_MS = 200;          // braked and no pulse for this long = axis done
const uint32_t NUDGE_AFTER_MS = 150;
const float NUDGE_RATE = 2.0f;           // V/s
const float SLEW = 50.0f;                // V/s
const uint32_t PROG_MS = 500;
const uint32_t WATCHDOG_MS = 1000;
const uint32_t SW_STALE_MS = 500;      // the Pi sends SW at least every 50 ms; this much silence = stop
const float JOG_MAX_UNHOMED = 50.0f;
const uint32_t HOME_PAUSE_MS = 150;      // brake at least this long between direction changes
const uint32_t STILL_MS = 120;           // no sensor change for this long = the axis has stopped coasting
const uint32_t STILL_MAX_MS = 1500;      // give up waiting for still after this long
const uint32_t HOME_APPROACH_MAX_MS = 180000;
const uint32_t SPRAY_MAX_MS = 3000;
const int SPAN_MAX_PASSES = 10;
const uint32_t STEP_STILL_MS = 300;
const uint32_t STEP_STILL_MAX_MS = 3000;
const uint32_t STEP_STALL_MS = 2000;
const uint32_t HOME_STALL_MS = 2000;     // homing approach / span leg: no sensor change for this long = stall
const int MAX_RAW = 8000;                // raw edges kept per step test (32 KB)
const uint32_t PWM_HZ = 20000;
const uint8_t PWM_BITS = 10;
const uint32_t DMAX = (1u << PWM_BITS) - 1;

// ---------- settings (SET) ----------
float vRail = 12.0f;
uint32_t stallMs = 1000;                 // move stall time (SET STALLT)
bool swEnabled = false;                  // the Pi sends SET SW 1 when the switches are connected
float homeFastV = 4.0f, homeSlowV = 2.5f, homeOffset = 5.0f;

// ---------- state ----------
HardwareSerial &PiSerial = Serial1;
Port usbPort = {&Serial, false, {0}, 0, false};
Port piPort  = {&PiSerial, true, {0}, 0, false};
AxisRt AX[3];
SensorFilt SF[3];
Job job = J_IDLE;
Stream *jobOut = nullptr;
uint32_t jobStartMs = 0, lastProgMs = 0, lastRxMs = 0, lastCtrlUs = 0;
uint8_t swBits = 0; uint32_t swMs = 0; bool swSeen = false;
bool estop = false;                       // latched by ESTOP from the Pi, cleared by RESET
uint32_t sprayMs = 0, sprayEndMs = 0;
int spanAxis = -1, spanLegs = 0, spanLeg = 0; int32_t spanC[21]; bool spanWaiting = true; uint32_t spanLegMs = 0;

float moveTgt[3]; bool moveUse[3]; bool moveWaiting = false; uint32_t moveWaitMs = 0, moveStartMs = 0, moveTimeoutMs = 0;
int homeAxes[3]; int homeN = 0, homeI = 0; HomePhase hph = H_FAST; uint32_t hphMs = 0; bool hDriving = false;

int stepAxis = -1; float stepV = 0; int stepDir = 1; uint32_t stepTicks = 0, stepTimeoutMs = 30000;
StepPhase stepPhase = S_STILL; uint32_t stepPhaseMs = 0, stepPulses0 = 0, stepLastPulseMs = 0, stepSeen = 0;
volatile bool rawLogging = false;
volatile int rawAxis = -1;
volatile uint32_t rawT0 = 0;
volatile uint16_t rawN = 0;
volatile uint32_t rawLost = 0;
uint32_t rawT[MAX_RAW];

// ---------- raw edges (step test only, no filter) ----------
void IRAM_ATTR rawEdge(int i) {
  if (!rawLogging || i != rawAxis) return;
  uint32_t t = micros() - rawT0;
  if (rawN < MAX_RAW) rawT[rawN++] = t; else rawLost++;
}
void IRAM_ATTR rawX() { rawEdge(0); }
void IRAM_ATTR rawY() { rawEdge(1); }
void IRAM_ATTR rawZ() { rawEdge(2); }

// ---------- filtered counting ----------
void countPulse(int i, uint32_t now) {
  AxisRt &a = AX[i];
  if (!a.driving && millis() - a.lastDriveMs > COAST_MS) return;
  if (a.ticks && now - a.lastTickUs < MIN_PERIOD_US) return;
  a.counts += a.credit;
  a.ticks++;
  a.prevTickUs = a.lastTickUs;
  a.lastTickUs = now;
}

void sampleSensors() {
  uint32_t now = micros();
  for (int i = 0; i < 3; i++) {
    SensorFilt &f = SF[i];
    uint32_t dt = now - f.lastUs;
    f.lastUs = now;
    if (dt > FILT_DT_MAX_US) dt = FILT_DT_MAX_US;
    bool raw = digitalRead(CFG[i].sensorPin) == HIGH;
    f.acc += raw ? (int32_t)dt : -(int32_t)dt;
    if (f.acc >= FILT_US) {
      f.acc = FILT_US;
      if (!f.lvl) { f.lvl = true; AX[i].lastLevelMs = millis(); countPulse(i, now); }
    } else if (f.acc <= 0) {
      f.acc = 0;
      if (f.lvl) { f.lvl = false; AX[i].lastLevelMs = millis(); }
    }
  }
}

// ---------- motors ----------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmAttach(int pin, int ch) { (void)ch; ledcAttach(pin, PWM_HZ, PWM_BITS); }
void pwmOut(int pin, int ch, uint32_t d) { (void)ch; ledcWrite(pin, d); }
#else
void pwmAttach(int pin, int ch) { ledcSetup(ch, PWM_HZ, PWM_BITS); ledcAttachPin(pin, ch); }
void pwmOut(int pin, int ch, uint32_t d) { (void)pin; ledcWrite(ch, d); }
#endif

uint32_t dutyOf(float v) {
  if (v <= 0) return 0;
  float f = v / vRail;
  if (f > 1) f = 1;
  return (uint32_t)(f * DMAX + 0.5f);
}

void axisOut(int i, int mmDir, float v) {                    // mmDir: +1 / -1 in mm, 0 = brake
  const AxisCfg &c = CFG[i];
  uint32_t d = dutyOf(v);
  if (d == 0 || mmDir == 0) {                                // both inputs low = L298 brake
    if (AX[i].driving) { AX[i].driving = false; AX[i].lastDriveMs = millis(); }
    pwmOut(c.inA, 2 * i, 0); pwmOut(c.inB, 2 * i + 1, 0);
    return;
  }
  AX[i].credit = mmDir > 0 ? 1 : -1;
  AX[i].driving = true;
  if (mmDir * c.sign > 0) { pwmOut(c.inB, 2 * i + 1, 0); pwmOut(c.inA, 2 * i, d); }
  else                    { pwmOut(c.inA, 2 * i, 0);     pwmOut(c.inB, 2 * i + 1, d); }
}
void brakeAll() { for (int i = 0; i < 3; i++) { axisOut(i, 0, 0); AX[i].active = false; AX[i].u = 0; } }

// ---------- helpers ----------
int axisIdx(char ch) { for (int i = 0; i < 3; i++) if (CFG[i].name == (char)toupper(ch)) return i; return -1; }
float posMm(int i) { return (AX[i].counts - AX[i].zero) * CFG[i].mmpc; }
bool swPressed(int k) { return k >= 0 && ((swBits >> k) & 1); }
bool swFresh() { return !swEnabled || (swSeen && millis() - swMs < SW_STALE_MS); }
int limitSwitch(int i, int dir) {                            // the switch hit when moving axis i in dir, or -1
  if (!swEnabled) return -1;
  int k = dir < 0 ? MIN_SW[i] : MAX_SW[i];
  return swPressed(k) ? k : -1;
}

bool axisStill(int i) { return millis() - AX[i].lastLevelMs >= STILL_MS; }

bool usbUp() { return (bool)Serial; }
void emit(Stream *s, const char *txt) {
  if (!s) return;
  if (s == (Stream *)&Serial && !usbUp()) return;            // never write to USB with nobody listening
  s->print(txt);
}
void sayTo(Stream *s, const char *fmt, ...) {
  char b[200]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  emit(s, b);
}
void posText(char *b, size_t n) { snprintf(b, n, "X%.1f Y%.1f Z%.1f", posMm(0), posMm(1), posMm(2)); }
void homedText(char *b) {
  int k = 0;
  for (int i = 0; i < 3; i++) if (AX[i].homed) b[k++] = CFG[i].name;
  if (!k) b[k++] = '-';
  b[k] = 0;
}
void swText(char *b) {
  if (!swEnabled) { strcpy(b, "off"); return; }
  if (!swFresh()) { strcpy(b, "stale"); return; }
  for (int k = 0; k < 5; k++) b[k] = swPressed(k) ? '1' : '0';
  b[5] = 0;
}

void endJob() { job = J_IDLE; rawLogging = false; brakeAll(); digitalWrite(PUMP_PIN, LOW); }
void failJob(const char *fmt, ...) {                          // brake, end the job, send the error to its port
  char b[160]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  Stream *o = jobOut;
  endJob();
  emit(o, b);
}

// ---------- closed-loop axis control (moves, jogs, homing park) ----------
void startAxis(int i, float target) {
  AxisRt &a = AX[i];
  float d = target - posMm(i);
  uint32_t now = millis();
  a.target = target; a.dir = d > 0 ? 1 : -1; a.u = 0; a.nudge = 0; a.hardSinceMs = 0;
  a.seenTicks = a.ticks; a.lastChangeMs = now; a.startMs = now;
  a.active = fabsf(d) >= 0.5f * CFG[i].mmpc;
}

AxisResult ctrlAxis(int i, float dt) {
  AxisRt &a = AX[i];
  if (!a.active) return AR_DONE;
  const AxisCfg &c = CFG[i];
  uint32_t nowUs = micros(), nowMs = millis();
  if (a.ticks != a.seenTicks) { a.seenTicks = a.ticks; a.nudge = 0; a.lastChangeMs = nowMs; }
  float ed = a.dir * (a.target - posMm(i));                   // distance left in the move direction
  float v = 0;
  if (a.prevTickUs != 0 && a.lastTickUs != a.prevTickUs) {
    float iv = (a.lastTickUs - a.prevTickUs) / 1e6f, since = (nowUs - a.lastTickUs) / 1e6f;
    v = c.mmpc / (since > iv ? since : iv);
  }
  float u = 0;
  if (ed >= c.mmpc) {
    if (limitSwitch(i, a.dir) >= 0) { axisOut(i, 0, 0); a.active = false; return AR_LIMIT; }
    if (nowMs - a.lastChangeMs > NUDGE_AFTER_MS) a.nudge += NUDGE_RATE * dt;
    u = c.v0 + c.kp * ed - c.kd * v + a.nudge;
    if (u < 0) u = 0;
    if (u > c.vlim) u = c.vlim;
    if (u > a.u + SLEW * dt) u = a.u + SLEW * dt;
  }
  a.u = u;
  if (c.stallV > 0 && u >= c.stallV) {                       // stall check: only while the drive should clearly move the axis
    if (!a.hardSinceMs) a.hardSinceMs = nowMs;
    uint32_t ref = a.lastChangeMs > a.hardSinceMs ? a.lastChangeMs : a.hardSinceMs;
    if (nowMs - ref > stallMs) { axisOut(i, 0, 0); a.active = false; return AR_STALL; }
  } else {
    a.hardSinceMs = 0;
  }
  axisOut(i, u > 0 ? a.dir : 0, u);
  if (u == 0 && nowMs - a.lastChangeMs > SETTLE_MS) { a.active = false; return AR_DONE; }
  return AR_RUNNING;
}

void ctrlMove(float dt) {
  if (moveWaiting) {                                          // single-channel sensors: never start while still coasting
    bool still = true;
    for (int i = 0; i < 3; i++) if (moveUse[i] && !axisStill(i)) still = false;
    if (!still && millis() - moveWaitMs < STILL_MAX_MS) return;
    moveWaiting = false;
    uint32_t now = millis();
    float far = 0;
    for (int i = 0; i < 3; i++) if (moveUse[i]) {
      startAxis(i, moveTgt[i]); AX[i].startMs = AX[i].lastChangeMs = now;
      float d = fabsf(moveTgt[i] - posMm(i)) / (i == 2 ? MOVE_MIN_SPEED_Z : MOVE_MIN_SPEED_XY);
      if (d > far) far = d;
    }
    moveStartMs = now; moveTimeoutMs = MOVE_TIMEOUT_BASE_MS + (uint32_t)(far * 1000);
    bool any = false;
    for (int i = 0; i < 3; i++) any = any || AX[i].active;
    if (!any) { Stream *o = jobOut; endJob(); char p[48]; posText(p, sizeof(p)); sayTo(o, "OK %s\n", p); return; }
  }
  if (millis() - moveStartMs > moveTimeoutMs) {               // second line of defence after the stall check
    char p[48]; posText(p, sizeof(p));
    failJob("ERR TIMEOUT move took longer than %lu s (axis blocked, motor not turning or sensor not counting?) %s\n",
            (unsigned long)(moveTimeoutMs / 1000), p);
    return;
  }
  bool busy = false;
  for (int i = 0; i < 3; i++) {
    AxisResult r = ctrlAxis(i, dt);
    if (r == AR_LIMIT) { int k = limitSwitch(i, AX[i].dir); failJob("ERR LIMIT %s\n", SW_NAMES[k >= 0 ? k : (AX[i].dir < 0 ? MIN_SW[i] : MAX_SW[i])]); return; }
    if (r == AR_STALL) {
      float u = AX[i].u; AX[i].homed = false;
      char p[48]; posText(p, sizeof(p));
      failJob("ERR STALL %c no pulse for %lu ms at %.1f V (blocked, belt off or sensor not counting?) %s\n",
              CFG[i].name, (unsigned long)stallMs, u, p);
      return;
    }
    if (r == AR_RUNNING) busy = true;
  }
  if (!busy) {
    Stream *o = jobOut;
    endJob();
    char p[48]; posText(p, sizeof(p));
    sayTo(o, "OK %s\n", p);
  }
}

// ---------- homing ----------
// Open-loop drive for the switch approaches (stall: no sensor change for HOME_STALL_MS; plus the approach time limit).
void homeDrive(int i, int dir, float v) {
  uint32_t now = millis();
  if (!hDriving && (now - hphMs < HOME_PAUSE_MS || (!axisStill(i) && now - hphMs < STILL_MAX_MS))) { axisOut(i, 0, 0); return; }
  AxisRt &a = AX[i];
  if (!hDriving) { hDriving = true; a.seenTicks = a.ticks; a.lastChangeMs = a.startMs = now; }
  axisOut(i, dir, v);
}
bool driveStalled(int i) {                                    // open-loop drive: no sensor level change for HOME_STALL_MS
  const AxisRt &a = AX[i];
  uint32_t ref = a.lastLevelMs > a.startMs ? a.lastLevelMs : a.startMs;
  return millis() - ref > HOME_STALL_MS;
}
void homePhase(HomePhase p) { hph = p; hphMs = millis(); hDriving = false; axisOut(homeAxes[homeI], 0, 0); }

// Homed = the axis' home switch is pressed (Z top, X min, Y min). Brake, take the count where it closed as 0.
// No back-off and no parking: the axis stays on its switch.
void homeAxisDone(int i) {
  axisOut(i, 0, 0);
  AX[i].zero = AX[i].counts;
  AX[i].homed = true;
  homeNextAxis();
}

void homeStartAxis() {
  int i = homeAxes[homeI];
  AX[i].homed = false;
  if (swPressed(MIN_SW[i])) { homeAxisDone(i); return; }     // already on the switch: this is home
  homePhase(H_FAST);
}

void homeNextAxis() {
  homeI++;
  if (homeI < homeN) { homeStartAxis(); return; }
  Stream *o = jobOut;
  endJob();
  char p[48], h[4]; posText(p, sizeof(p)); homedText(h);
  sayTo(o, "OK HOME %s HOMED=%s\n", p, h);
}

void ctrlHome(float dt) {
  int i = homeAxes[homeI];
  if (swPressed(MIN_SW[i])) { homeAxisDone(i); return; }
  if (millis() - hphMs > HOME_APPROACH_MAX_MS) { failJob("ERR HOME %c no switch within %lu s\n", CFG[i].name, (unsigned long)(HOME_APPROACH_MAX_MS / 1000)); return; }
  homeDrive(i, -1, homeFastV);
  if (hDriving && driveStalled(i)) { failJob("ERR HOME %c stall (no sensor change for %lu s at %.1f V)\n", CFG[i].name, (unsigned long)(HOME_STALL_MS / 1000), homeFastV); return; }
}

// ---------- step test ----------
void finishStep(const char *reason) {
  axisOut(stepAxis, 0, 0);
  bool logged = stepPhase == S_DRIVE;
  noInterrupts(); rawLogging = false; uint16_t n = rawN; uint32_t lost = rawLost; uint32_t tEnd = micros() - rawT0; interrupts();
  if (!logged) { n = 0; lost = 0; tEnd = 0; }
  uint32_t pulses = logged ? AX[stepAxis].ticks - stepPulses0 : 0;
  Stream *o = jobOut;
  int ax = stepAxis;
  job = J_IDLE; brakeAll();
  char b[40];
  for (uint16_t k = 0; k < n; k++) { snprintf(b, sizeof(b), "D %u,%lu\n", (unsigned)(k + 1), (unsigned long)rawT[k]); emit(o, b); }
  sayTo(o, "OK STEP %c edges=%u pulses=%lu end=%s t_end=%lu lost=%lu v=%.2f dir=%c\n", CFG[ax].name, (unsigned)n,
        (unsigned long)pulses, reason, (unsigned long)tEnd, (unsigned long)lost, stepV, stepDir > 0 ? '+' : '-');
}

void ctrlStep() {
  uint32_t now = millis();
  AxisRt &a = AX[stepAxis];
  if (stepPhase == S_STILL) {
    if ((now - a.lastLevelMs >= STEP_STILL_MS && now - stepPhaseMs >= STEP_STILL_MS) || now - stepPhaseMs > STEP_STILL_MAX_MS) {
      if (limitSwitch(stepAxis, stepDir) >= 0) { stepPhase = S_STILL; finishStep("limit"); return; }
      noInterrupts(); rawN = 0; rawLost = 0; rawAxis = stepAxis; rawT0 = micros(); rawLogging = true; interrupts();
      stepPulses0 = a.ticks; stepLastPulseMs = now; stepSeen = 0;
      axisOut(stepAxis, stepDir, stepV);                      // THE STEP
      stepPhase = S_DRIVE; stepPhaseMs = now;
    }
    return;
  }
  uint32_t pulses = a.ticks - stepPulses0;
  if (pulses != stepSeen) { stepSeen = pulses; stepLastPulseMs = now; }
  if (pulses >= stepTicks) { finishStep("ticks"); return; }
  if (now - stepPhaseMs > stepTimeoutMs) { finishStep("timeout"); return; }
  if (now - stepLastPulseMs > STEP_STALL_MS) { finishStep("stall"); return; }
  if (limitSwitch(stepAxis, stepDir) >= 0) { finishStep("limit"); return; }
  if (a.homed) {
    float p = posMm(stepAxis);
    if ((stepDir > 0 && p > CFG[stepAxis].hi) || (stepDir < 0 && p < CFG[stepAxis].lo)) { finishStep("limit"); return; }
  }
}

// ---------- span: drive between the two end switches of X or Y and report the count at each trip ----------
void ctrlSpan() {
  int i = spanAxis;
  uint32_t now = millis();
  int dir = (spanLeg % 2 == 0) ? -1 : 1;                      // legs: min, max, min, ... ending on min
  int sw = dir < 0 ? MIN_SW[i] : MAX_SW[i];
  if (spanWaiting) {                                           // brake until the axis has stopped coasting
    axisOut(i, 0, 0);
    if ((!axisStill(i) && now - spanLegMs < STILL_MAX_MS) || now - spanLegMs < HOME_PAUSE_MS) return;
    spanWaiting = false; spanLegMs = now; hDriving = false;
  }
  if (swPressed(sw)) {
    spanC[spanLeg++] = AX[i].counts;
    axisOut(i, 0, 0);
    if (spanLeg >= spanLegs) {
      Stream *o = jobOut;
      endJob();
      char b[200]; int k = snprintf(b, sizeof(b), "OK SPAN %c C=", CFG[i].name);
      for (int n = 0; n < spanLegs; n++) k += snprintf(b + k, sizeof(b) - k, n ? ",%ld" : "%ld", (long)spanC[n]);
      snprintf(b + k, sizeof(b) - k, "\n");
      emit(o, b);
      return;
    }
    spanWaiting = true; spanLegMs = now;
    return;
  }
  if (now - spanLegMs > HOME_APPROACH_MAX_MS) { failJob("ERR SPAN %c no %s within %lu s\n", CFG[i].name, SW_NAMES[sw], (unsigned long)(HOME_APPROACH_MAX_MS / 1000)); return; }
  AxisRt &a = AX[i];
  if (!hDriving) { hDriving = true; a.seenTicks = a.ticks; a.lastChangeMs = a.startMs = now; }
  axisOut(i, dir, homeFastV);
  if (driveStalled(i)) { failJob("ERR SPAN %c stall (no sensor change for %lu s at %.1f V)\n", CFG[i].name, (unsigned long)(HOME_STALL_MS / 1000), homeFastV); return; }
}

void ctrlSpray() {
  if ((int32_t)(millis() - sprayEndMs) < 0) return;
  Stream *o = jobOut;
  endJob();
  sayTo(o, "OK SPRAY %lu\n", (unsigned long)sprayMs);
}

// ---------- background checks ----------
void progTick() {
  uint32_t now = millis();
  if (now - lastProgMs < PROG_MS) return;
  lastProgMs = now;
  char p[48]; posText(p, sizeof(p));
  if (job == J_STEP) sayTo(jobOut, "PROG STEP n=%lu %s\n", (unsigned long)(stepPhase == S_DRIVE ? AX[stepAxis].ticks - stepPulses0 : 0), p);
  else if (job == J_SPRAY) sayTo(jobOut, "PROG SPRAY\n");
  else sayTo(jobOut, "PROG %s\n", p);
}

bool safetyChecks() {                                         // false = the job was ended
  if (job == J_IDLE) return false;
  if (jobOut == (Stream *)&PiSerial && millis() - lastRxMs > WATCHDOG_MS) {
    if (job == J_STEP) { finishStep("stop"); return false; }
    failJob("ERR WATCHDOG\n"); return false;
  }
  if (job != J_SPRAY && !swFresh()) {
    if (job == J_STEP) { finishStep("sw_lost"); return false; }
    failJob("ERR SW_LOST\n"); return false;
  }
  return true;
}

// ---------- commands ----------
bool busy(Stream *o) {
  if (job != J_IDLE) { sayTo(o, "ERR BUSY\n"); return true; }
  return false;
}
bool refuse(Stream *o) {                                      // motion and spray: e-stop latched or busy
  if (estop) { sayTo(o, "ERR ESTOP latched\n"); return true; }
  return busy(o);
}

void cmdSpan(Stream *o, char **tok, int nt) {
  // SPAN <X|Y> [passes]: legs min, max, min ... (2 x passes + 1), reply OK SPAN X C=<count at each trip>
  if (nt < 2) { sayTo(o, "ERR BAD_ARG\n"); return; }
  int i = axisIdx(tok[1][0]); int passes = nt >= 3 ? atoi(tok[2]) : 2;
  if (i < 0 || tok[1][1] || MAX_SW[i] < 0 || passes < 1 || passes > SPAN_MAX_PASSES) { sayTo(o, "ERR BAD_ARG\n"); return; }
  if (refuse(o)) return;
  if (!swEnabled) { sayTo(o, "ERR BAD_ARG switches off (SET SW 1)\n"); return; }
  if (!swFresh()) { sayTo(o, "ERR SW_LOST\n"); return; }
  spanAxis = i; spanLegs = 2 * passes + 1; spanLeg = 0; spanWaiting = true; spanLegMs = millis(); hDriving = false;
  brakeAll();
  job = J_SPAN; jobOut = o; jobStartMs = lastProgMs = millis();
}

void startMove(const float *tgt, const bool *use, Stream *o, bool unhomedOk) {
  if (refuse(o)) return;
  if (!swFresh()) { sayTo(o, "ERR SW_LOST\n"); return; }
  for (int i = 0; i < 3; i++) {
    if (!use[i]) continue;
    if (AX[i].homed) {
      float here = posMm(i);                                     // moving back toward the range is always allowed (homed on the switch = 0, below the soft limit)
      if ((tgt[i] < CFG[i].lo - 1e-3f && tgt[i] < here) || (tgt[i] > CFG[i].hi + 1e-3f && tgt[i] > here)) { sayTo(o, "ERR SOFT_LIMIT %c\n", CFG[i].name); return; }
    } else if (!unhomedOk) {
      sayTo(o, "ERR NOT_HOMED %c\n", CFG[i].name); return;
    }
    float d = tgt[i] - posMm(i);
    int k = limitSwitch(i, d > 0 ? 1 : -1);
    if (fabsf(d) >= 0.5f * CFG[i].mmpc && k >= 0) { sayTo(o, "ERR LIMIT %s\n", SW_NAMES[k]); return; }
  }
  bool any = false;
  for (int i = 0; i < 3; i++) {
    AX[i].active = false;
    moveUse[i] = use[i]; moveTgt[i] = tgt[i];
    if (use[i] && fabsf(tgt[i] - posMm(i)) >= 0.5f * CFG[i].mmpc) any = true;
  }
  char p[48]; posText(p, sizeof(p));
  if (!any) { sayTo(o, "OK %s\n", p); return; }
  brakeAll();
  moveWaiting = true; moveWaitMs = millis();
  job = J_MOVE; jobOut = o; jobStartMs = lastProgMs = millis();
}

void cmdHome(Stream *o, char **tok, int nt) {
  if (refuse(o)) return;
  bool want[3] = {nt < 2, nt < 2, nt < 2};
  for (int k = 1; k < nt; k++)
    for (const char *p = tok[k]; *p; p++) { int i = axisIdx(*p); if (i < 0) { sayTo(o, "ERR BAD_ARG\n"); return; } want[i] = true; }
  if (!swEnabled) {                                           // no switches: this spot is the datum
    for (int i = 0; i < 3; i++) if (want[i]) { AX[i].zero = AX[i].counts; AX[i].homed = true; }
    char p[48], h[4]; posText(p, sizeof(p)); homedText(h);
    sayTo(o, "OK HOME %s HOMED=%s\n", p, h);
    return;
  }
  if (!swFresh()) { sayTo(o, "ERR SW_LOST\n"); return; }
  const int order[3] = {2, 0, 1};                             // Z first, so the nozzle is up before X and Y move
  homeN = 0;
  for (int k = 0; k < 3; k++) if (want[order[k]]) homeAxes[homeN++] = order[k];
  homeI = 0;
  job = J_HOME; jobOut = o; jobStartMs = lastProgMs = millis();
  homeStartAxis();
}

void cmdStep(Stream *o, char **tok, int nt) {
  // STEP <axis> <volts> <+|-> <ticks> [timeout s]
  if (nt < 5) { sayTo(o, "ERR BAD_ARG\n"); return; }
  int i = axisIdx(tok[1][0]); float v = atof(tok[2]); long n = atol(tok[4]);
  int dir = !strcmp(tok[3], "+") || !strcmp(tok[3], "+1") ? 1 : (!strcmp(tok[3], "-") || !strcmp(tok[3], "-1") ? -1 : 0);
  float tmo = nt >= 6 ? atof(tok[5]) : 30.0f;
  if (i < 0 || tok[1][1] || v <= 0 || v > vRail + 1e-3f || !dir || n < 1 || n > 100000 || tmo <= 0 || tmo > 300) { sayTo(o, "ERR BAD_ARG\n"); return; }
  if (refuse(o)) return;
  if (!swFresh()) { sayTo(o, "ERR SW_LOST\n"); return; }
  stepAxis = i; stepV = v; stepDir = dir; stepTicks = (uint32_t)n; stepTimeoutMs = (uint32_t)(tmo * 1000);
  brakeAll();
  stepPhase = S_STILL; stepPhaseMs = millis();
  job = J_STEP; jobOut = o; jobStartMs = lastProgMs = millis();
}

void cmdStop(Stream *o) {
  if (job == J_STEP) finishStep("stop");
  else if (job != J_IDLE) failJob("ERR STOPPED\n");
  brakeAll();
  char p[48]; posText(p, sizeof(p));
  sayTo(o, "OK STOP %s\n", p);
}

void cmdSet(Stream *o, char **tok, int nt) {
  if (busy(o)) return;
  if (nt < 3) { sayTo(o, "ERR BAD_ARG\n"); return; }
  const char *key = tok[1];
  if (!strcmp(key, "RAIL")) { float v = atof(tok[2]); if (v < 3 || v > 30) { sayTo(o, "ERR BAD_ARG\n"); return; } vRail = v; }
  else if (!strcmp(key, "STALLV") && axisIdx(tok[2][0]) < 0) { }   // old form SET STALLV <v>: accepted, ignored (see SET)
  else if (!strcmp(key, "STALLT")) { long ms = atol(tok[2]); if (ms < 100 || ms > 10000) { sayTo(o, "ERR BAD_ARG\n"); return; } stallMs = (uint32_t)ms; }
  else if (!strcmp(key, "SW")) { swEnabled = atoi(tok[2]) != 0; swSeen = false; }
  else if (!strcmp(key, "HOME")) {
    if (nt < 5) { sayTo(o, "ERR BAD_ARG\n"); return; }
    float f = atof(tok[2]), s = atof(tok[3]), off = atof(tok[4]);
    if (f <= 0 || s <= 0 || f > vRail || s > vRail || off < 0) { sayTo(o, "ERR BAD_ARG\n"); return; }
    homeFastV = f; homeSlowV = s; homeOffset = off;
  } else {
    int i = axisIdx(tok[2][0]);
    if (i < 0 || tok[2][1]) { sayTo(o, "ERR BAD_ARG\n"); return; }
    AxisCfg &c = CFG[i];
    if (!strcmp(key, "MMPC") && nt >= 4) { float v = atof(tok[3]); if (v <= 0 || v > 20) { sayTo(o, "ERR BAD_ARG\n"); return; } c.mmpc = v; }
    else if (!strcmp(key, "SOFT") && nt >= 5) { float lo = atof(tok[3]), hi = atof(tok[4]); if (hi <= lo) { sayTo(o, "ERR BAD_ARG\n"); return; } c.lo = lo; c.hi = hi; }
    else if (!strcmp(key, "GAIN") && nt >= 6) { c.kp = atof(tok[3]); c.kd = atof(tok[4]); c.vlim = atof(tok[5]); c.v0 = nt >= 7 ? atof(tok[6]) : 0.0f; }
    else if (!strcmp(key, "SIGN") && nt >= 4) { c.sign = atoi(tok[3]) < 0 ? -1 : 1; }
    else if (!strcmp(key, "STALLV") && nt >= 4) { float v = atof(tok[3]); if (v < 0 || v > vRail) { sayTo(o, "ERR BAD_ARG\n"); return; } c.stallV = v; }
    else { sayTo(o, "ERR BAD_ARG\n"); return; }
  }
  sayTo(o, "OK SET %s\n", key);
}

void cmdCfg(Stream *o) {
  char b[600]; int k = snprintf(b, sizeof(b), "OK CFG FW=%s RAIL=%.2f SW=%d HOME=%.2f,%.2f,%.1f STALLT=%lu", FW_VERSION, vRail,
                                swEnabled ? 1 : 0, homeFastV, homeSlowV, homeOffset, (unsigned long)stallMs);
  for (int i = 0; i < 3; i++) {
    const AxisCfg &c = CFG[i];
    k += snprintf(b + k, sizeof(b) - k, " %c:mmpc=%.4f,sign=%d,kp=%.3f,kd=%.3f,vlim=%.2f,v0=%.2f,soft=%.1f..%.1f,stallv=%.2f",
                  c.name, c.mmpc, c.sign, c.kp, c.kd, c.vlim, c.v0, c.lo, c.hi, c.stallV);
  }
  snprintf(b + k, sizeof(b) - k, "\n");
  emit(o, b);
}

void handleLine(Port &P, char *line) {
  char *tok[8]; int nt = 0;
  for (char *p = strtok(line, " \t\r"); p && nt < 8; p = strtok(nullptr, " \t\r")) tok[nt++] = p;
  if (!nt) return;
  Stream *o = P.s;
  const char *c = tok[0];
  if (!strcmp(c, "KA")) return;
  if (!strcmp(c, "SW")) {
    if (nt < 2 || strlen(tok[1]) != 5) return;
    uint8_t b = 0;
    for (int k = 0; k < 5; k++) { if (tok[1][k] == '1') b |= 1 << k; else if (tok[1][k] != '0') return; }
    swBits = b; swMs = millis(); swSeen = true;
    return;
  }
  if (!strcmp(c, "PING")) { sayTo(o, "PONG\n"); return; }
  if (!strcmp(c, "POS?")) { char p[48], h[4], s[8]; posText(p, sizeof(p)); homedText(h); swText(s); sayTo(o, "OK %s HOMED=%s SW=%s\n", p, h, s); return; }
  if (!strcmp(c, "STOP")) { cmdStop(o); return; }
  if (!strcmp(c, "ESTOP")) { estop = true; if (job != J_IDLE) endJob(); brakeAll(); digitalWrite(PUMP_PIN, LOW); sayTo(o, "OK ESTOP\n"); return; }
  if (!strcmp(c, "RESET")) { estop = false; sayTo(o, "OK RESET\n"); return; }
  if (!strcmp(c, "ABORT")) { if (job != J_IDLE) endJob(); brakeAll(); digitalWrite(PUMP_PIN, LOW); sayTo(o, "OK ABORT\n"); return; }
  if (!strcmp(c, "PAUSE") || !strcmp(c, "RESUME")) { sayTo(o, "OK %s\n", c); return; }
  if (!strcmp(c, "SPAN")) { cmdSpan(o, tok, nt); return; }
  if (!strcmp(c, "SPRAY")) {
    long ms = nt >= 2 ? atol(tok[1]) : 0;
    if (ms < 1 || ms > (long)SPRAY_MAX_MS) { sayTo(o, "ERR BAD_ARG\n"); return; }
    if (refuse(o)) return;
    sprayMs = ms; sprayEndMs = millis() + ms;
    digitalWrite(PUMP_PIN, HIGH);
    job = J_SPRAY; jobOut = o; jobStartMs = lastProgMs = millis();
    return;
  }
  if (!strcmp(c, "SET")) { cmdSet(o, tok, nt); return; }
  if (!strcmp(c, "CFG?")) { cmdCfg(o); return; }
  if (!strcmp(c, "HOME")) { cmdHome(o, tok, nt); return; }
  if (!strcmp(c, "STEP")) { cmdStep(o, tok, nt); return; }
  if (!strcmp(c, "MOVE")) {
    if (nt < 2) { sayTo(o, "ERR BAD_ARG\n"); return; }
    float tgt[3] = {posMm(0), posMm(1), posMm(2)}; bool use[3] = {false, false, false};
    for (int k = 1; k < nt; k++) {
      int i = axisIdx(tok[k][0]); char *end = nullptr;
      float v = strtof(tok[k] + 1, &end);
      if (i < 0 || end == tok[k] + 1 || *end) { sayTo(o, "ERR BAD_ARG %s\n", tok[k]); return; }
      tgt[i] = v; use[i] = true;
    }
    startMove(tgt, use, o, false);
    return;
  }
  if (!strcmp(c, "JOG")) {
    if (nt < 2) { sayTo(o, "ERR BAD_ARG\n"); return; }
    int i = axisIdx(tok[1][0]); char *end = nullptr;
    float d = strtof(tok[1] + 1, &end);
    if (i < 0 || end == tok[1] + 1 || *end) { sayTo(o, "ERR BAD_ARG %s\n", tok[1]); return; }
    if (!AX[i].homed && fabsf(d) > JOG_MAX_UNHOMED) { sayTo(o, "ERR NOT_HOMED %c (unhomed jogs are 50 mm at most)\n", CFG[i].name); return; }
    float tgt[3] = {posMm(0), posMm(1), posMm(2)}; bool use[3] = {false, false, false};
    tgt[i] += d; use[i] = true;
    startMove(tgt, use, o, true);
    return;
  }
  // anything else is ignored
}

void pollPort(Port &P) {
  int budget = 128;
  while (budget-- > 0 && P.s->available()) {
    int ch = P.s->read();
    if (ch < 0) break;
    if (P.isPi) lastRxMs = millis();
    if (ch == '\n' || ch == '\r') {
      if (P.n && !P.overflow) { P.buf[P.n] = 0; handleLine(P, P.buf); }
      P.n = 0; P.overflow = false;
    } else if (P.n < sizeof(P.buf) - 1) {
      P.buf[P.n++] = (char)ch;
    } else {
      P.overflow = true;                                   // too long to be a command: drop the line
    }
  }
}

void setup() {
  pinMode(PUMP_PIN, OUTPUT); digitalWrite(PUMP_PIN, LOW);
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  PiSerial.setRxBufferSize(512);
  PiSerial.setTxBufferSize(8192);
  PiSerial.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  for (int i = 0; i < 3; i++) {
    memset(&AX[i], 0, sizeof(AxisRt));
    AX[i].credit = 1;
    pwmAttach(CFG[i].inA, 2 * i); pwmAttach(CFG[i].inB, 2 * i + 1);
    pinMode(CFG[i].sensorPin, INPUT);                        // Y is GPIO19 (USB D+): this takes the pin away from the USB port, see PINS
  }
  brakeAll();
  for (int i = 0; i < 3; i++) {                              // start from the level the sensor shows now: no count at boot
    bool raw = digitalRead(CFG[i].sensorPin) == HIGH;
    SF[i].lastUs = micros(); SF[i].lvl = raw; SF[i].acc = raw ? FILT_US : 0;
  }
  attachInterrupt(digitalPinToInterrupt(CFG[0].sensorPin), rawX, RISING);
  attachInterrupt(digitalPinToInterrupt(CFG[1].sensorPin), rawY, RISING);
  attachInterrupt(digitalPinToInterrupt(CFG[2].sensorPin), rawZ, RISING);
  lastRxMs = millis(); lastCtrlUs = micros();
  sayTo(&PiSerial, "BOOT gantry-fw %s\n", FW_VERSION);
  sayTo(&Serial, "BOOT gantry-fw %s (USB takes the same commands: PING, POS?, CFG?)\n", FW_VERSION);
}

void loop() {
  sampleSensors();
  pollPort(piPort);
  pollPort(usbPort);
  uint32_t now = micros();
  if (now - lastCtrlUs >= CTRL_US) {
    float dt = (now - lastCtrlUs) / 1e6f;
    lastCtrlUs = now;
    if (safetyChecks()) {
      if (job == J_MOVE) ctrlMove(dt);
      else if (job == J_HOME) ctrlHome(dt);
      else if (job == J_STEP) ctrlStep();
      else if (job == J_SPAN) ctrlSpan();
      else if (job == J_SPRAY) ctrlSpray();
      if (job != J_IDLE) progTick();
    }
  }
}
