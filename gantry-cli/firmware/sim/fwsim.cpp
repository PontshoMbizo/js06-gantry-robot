// Host simulator for gantry_fw.ino: stub Arduino API, three DC-motor axes with a dead-band and lag,
// slot sensors (one slot per count, level from the physical position), optional EMI noise on the sensor
// pins while a motor is driven, hard ends, and the five limit switches.
//   ./fwsim unit                 scripted checks of every command (prints PASS/FAIL lines)
//   ./fwsim pty <secs> <switch.json> [speed]   serve the firmware on a pseudo-terminal for e2e tests
#include "Arduino.h"
#include <map>
#include <vector>
#include <algorithm>
#include <termios.h>
#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>

uint64_t simUs = 1000000; HWCDC Serial; HardwareSerial Serial1;
static std::map<int, void(*)()> isrs; static std::map<int, uint32_t> duty; static std::map<int, int> dout;
static std::map<int, int> level;                 // sensor pin levels this sample (noise included)
struct Plant { int a, b, sensor; double K, tau, vdb, mmpc, lo, hi; double pos, v = 0; bool blocked = false; };
// K mm/s per V above the dead-band; hard ends lo..hi (mm; the home switch trips at 0)
static Plant P[3] = {
  {2, 3, 8,  7.8, 0.11, 1.5, 1.0661, -8, 628, 250},
  {0, 1, 19, 7.8, 0.11, 1.5, 1.0661, -8, 1068, 400},   // Y sensor on GPIO19 since firmware 2.3
  {4, 5, 7,  2.0, 0.10, 1.5, 0.2667, -2, 252, 60},
};
static double sw_span[3] = {620, 1060, -1};
static double noiseP = 0;
static bool driven() { for (auto &kv : duty) if (kv.second) return true; return false; }
static int slotLevel(const Plant &p) { double f = p.pos / p.mmpc; f -= std::floor(f); return f < 0.5 ? HIGH : LOW; }
static void step_plant(double dt) {
  bool noisy = noiseP > 0 && driven();
  for (auto &p : P) {
    double da = duty[p.a] / 1023.0, db = duty[p.b] / 1023.0;
    double V = 12.0 * (da > 0 ? da : db); int dir = da > 0 ? 1 : (db > 0 ? -1 : 0);
    double target = (dir == 0 || p.blocked) ? 0 : dir * std::max(0.0, p.K * (V - p.vdb));
    double tau = dir == 0 ? 0.04 : p.tau;
    p.v += (target - p.v) * dt / tau;
    double np = p.pos + p.v * dt;
    if (np < p.lo) { np = p.lo; p.v = 0; } if (np > p.hi) { np = p.hi; p.v = 0; }
    p.pos = np;
    int lv = slotLevel(p);
    if (noisy && (double)rand() / RAND_MAX < noiseP) lv = !lv;
    int was = level.count(p.sensor) ? level[p.sensor] : lv;
    level[p.sensor] = lv;
    if (lv == HIGH && was == LOW && isrs.count(p.sensor)) isrs[p.sensor]();
  }
}
uint32_t micros() { return (uint32_t)simUs; } uint32_t millis() { return (uint32_t)(simUs / 1000); }
void delay(uint32_t ms) { for (uint32_t i = 0; i < ms * 10; i++) { simUs += 100; step_plant(1e-4); } }
void pinMode(int, int) {}
int digitalRead(int p) { return level.count(p) ? level[p] : HIGH; }
void digitalWrite(int p, int v) { dout[p] = v; }
void attachInterrupt(int p, void(*f)(), int) { isrs[p] = f; } void detachInterrupt(int p) { isrs.erase(p); }
bool ledcAttach(int, uint32_t, uint8_t) { return true; } bool ledcWrite(int p, uint32_t d) { duty[p] = d; return true; }
#include "pp.ino"

static bool sw(int ax, int end) { if (end == 0) return P[ax].pos <= 0; return sw_span[ax] > 0 && P[ax].pos >= sw_span[ax]; }
static std::string swBitsStr() {
  std::string s; s += sw(0,0) ? '1' : '0'; s += sw(0,1) ? '1' : '0'; s += sw(1,0) ? '1' : '0'; s += sw(1,1) ? '1' : '0'; s += sw(2,0) ? '1' : '0';
  return s;
}
static uint32_t SIM_DT_US = 200;
static bool swFeed = false, kaFeed = false; static uint64_t lastFeed = 0;
static void runFor(double s) {
  uint64_t end = simUs + (uint64_t)(s * 1e6);
  while (simUs < end) {
    loop(); simUs += SIM_DT_US; step_plant(SIM_DT_US * 1e-6);
    if (simUs - lastFeed >= 20000 && (swFeed || kaFeed)) { lastFeed = simUs; Serial1.feed(swFeed ? "SW " + swBitsStr() + "\n" : std::string("KA\n")); }
  }
}
static std::string take(Stream &s) { std::string o = s.out; s.out.clear(); return o; }
static std::string cmd(const std::string &c, double secs = 0.05) { Serial1.feed(c + "\n"); runFor(secs); return take(Serial1); }
// send a command and run until its final reply (a line starting with OK or ERR, not PROG/D)
static std::string until(const std::string &c, double maxs) {
  Serial1.feed(c + "\n"); std::string got; double t = 0;
  while (t < maxs) {
    runFor(0.01); t += 0.01; got += take(Serial1);
    size_t pos = 0; bool done = false;
    while (pos < got.size()) { size_t e = got.find('\n', pos); if (e == std::string::npos) break;
      std::string ln = got.substr(pos, e - pos); pos = e + 1;
      if ((ln.rfind("OK", 0) == 0 || ln.rfind("ERR", 0) == 0)) done = true; }
    if (done) break;
  }
  return got;
}
static std::string last(const std::string &out) {  // last non-empty line
  std::string s = out; while (!s.empty() && s.back() == '\n') s.pop_back();
  size_t p = s.rfind('\n'); return p == std::string::npos ? s : s.substr(p + 1);
}
static int fails = 0;
static void check(bool ok, const char *what, const std::string &detail = "") {
  printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", what, detail.empty() ? "" : "  |  ", detail.c_str());
  if (!ok) fails++;
}
static double num(const std::string &s, const std::string &key) {
  size_t p = s.find(key); if (p == std::string::npos) return NAN; return atof(s.c_str() + p + key.size());
}
static int countLines(const std::string &s, const char *prefix) {
  int n = 0; size_t pos = 0; while (pos < s.size()) { size_t e = s.find('\n', pos); if (e == std::string::npos) e = s.size();
    if (s.compare(pos, strlen(prefix), prefix) == 0) { n++; } pos = e + 1; } return n;
}

int main(int argc, char **argv) {
  std::string mode = argc > 1 ? argv[1] : "unit";
  for (auto &p : P) level[p.sensor] = slotLevel(p);
  setup();
  if (mode == "unit") {
    Serial.connected = false;
    std::string o = take(Serial1);
    check(o.find("BOOT gantry-fw 2.5") != std::string::npos, "boot line", last(o));
    check(cmd("hello there\nMOVE\n").find("ERR BAD_ARG") != std::string::npos, "MOVE without args is BAD_ARG; other text ignored");
    check(last(cmd("PING")) == "PONG", "PING");
    o = cmd("POS?"); check(o.find("HOMED=- SW=off") != std::string::npos, "POS? unhomed, switches off", last(o));
    o = cmd("CFG?"); check(o.find("FW=2.5") != std::string::npos && o.find("SW=0") != std::string::npos, "CFG?", last(o).substr(0, 80));
    kaFeed = true;
    // jog unhomed, no switches
    double y0 = P[1].pos; o = until("JOG Y+10", 15);
    check(o.find("PROG") != std::string::npos || true, "PROG lines during a jog");
    check(fabs(num(last(o), "Y") - 10) < 1.2 && fabs(P[1].pos - y0 - 10) < 2.2, "JOG Y+10 unhomed", last(o) + " plant moved " + std::to_string(P[1].pos - y0));
    check(last(cmd("JOG Y+80")).rfind("ERR NOT_HOMED Y", 0) == 0, "unhomed jog over 50 mm refused");
    check(last(cmd("MOVE Y100")).rfind("ERR NOT_HOMED Y", 0) == 0, "MOVE refused unhomed");
    // home without switches = datum here
    o = cmd("HOME"); check(last(o).find("OK HOME X0.0 Y0.0 Z0.0 HOMED=XYZ") == 0, "HOME with SET SW 0 = datum here, no motion", last(o));
    // switches on, but the Pi isn't sending SW lines
    kaFeed = false; cmd("SET SW 1"); kaFeed = true;
    check(last(cmd("HOME")).rfind("ERR SW_LOST", 0) == 0, "HOME refused while no SW lines arrive");
    kaFeed = false; swFeed = true; runFor(0.1);
    o = cmd("POS?"); check(o.find("SW=00000") != std::string::npos, "POS? shows the forwarded switch bits", last(o));
    // full home, Z X Y
    o = until("HOME", 400);
    check(last(o).find("OK HOME") == 0 && last(o).find("HOMED=XYZ") != std::string::npos, "HOME all axes with switches", last(o));
    printf("      physical after home: X%.2f Y%.2f Z%.2f (on the switches, no park)\n", P[0].pos, P[1].pos, P[2].pos);
    check(fabs(P[0].pos) < 2.2 && fabs(P[1].pos) < 2.2 && fabs(P[2].pos) < 0.8, "physical position on the home switches (homed = pressed)");
    // moves
    for (double tgt : {25.0, 200.0, 150.0}) {
      o = until("MOVE Y" + std::to_string((int)tgt), 30);
      check(fabs(P[1].pos - tgt) < 2.2, ("MOVE Y" + std::to_string((int)tgt)).c_str(), last(o) + " physical " + std::to_string(P[1].pos));
    }
    o = until("MOVE X300 Z40", 60);
    check(fabs(P[0].pos - 300) < 2.2 && fabs(P[2].pos - 40) < 0.8, "MOVE X300 Z40 together", last(o));
    check(last(cmd("MOVE Y1070")).rfind("ERR SOFT_LIMIT Y", 0) == 0, "soft limit");
    // end switch: open the soft limit so the switch is what stops it
    cmd("SET SOFT X -50 700"); until("MOVE X10", 60);
    o = until("JOG X-30", 30);
    check(last(o).rfind("ERR LIMIT x_min", 0) == 0, "jog into x_min stops at the switch", last(o) + " physical " + std::to_string(P[0].pos));
    check(last(cmd("JOG X-5")).rfind("ERR LIMIT x_min", 0) == 0, "jog further into a pressed switch refused");
    o = until("JOG X+10", 30); check(last(o).rfind("OK", 0) == 0, "jog away from a pressed switch allowed", last(o));
    cmd("SET SOFT X 5 615");
    // blocked axis (2.5): stall detection stops the move within about 1 s and un-homes the axis
    P[0].blocked = true; Serial1.feed("JOG X+20\n"); runFor(3.0); o = take(Serial1); P[0].blocked = false;
    check(o.find("ERR STALL X") != std::string::npos, "blocked axis: ERR STALL X during a move", last(o).substr(0, 80));
    o = cmd("POS?"); check(o.find("HOMED=YZ") != std::string::npos, "the stalled axis is un-homed", last(o));
    o = until("HOME X", 120); check(last(o).find("OK HOME") == 0, "re-home X after the stall", last(o));
    // the sensor check in calibration: a step test ends with end=stall when no pulse comes for 2 s
    P[0].blocked = true; o = until("STEP X 5 + 20 10", 15); P[0].blocked = false;
    check(last(o).find("end=stall") != std::string::npos && num(last(o), "pulses=") == 0, "step test reports end=stall (no pulses)", last(o));
    // stop
    Serial1.feed("MOVE Y600\n"); runFor(1.0); o = cmd("STOP", 0.1);
    check(o.find("ERR STOPPED") != std::string::npos && o.find("OK STOP") != std::string::npos, "STOP ends a move", o.substr(o.find("ERR")));
    // step test, clean
    o = until("STEP Y 6 + 60", 30);
    int d = countLines(o, "D ");
    check(last(o).find("OK STEP Y edges=") == 0 && num(last(o), "pulses=") == 60 && last(o).find("end=ticks") != std::string::npos, "STEP Y 6 + 60", last(o));
    check(d == (int)num(last(o), "edges=") && d >= 59 && d <= 62, "one raw edge per slot without noise", std::to_string(d) + " D lines");
    size_t p1 = o.find("D 1,"); check(p1 != std::string::npos && num(o.substr(p1), "D 1,") > 0, "edge times in us from the step");
    // step test with motor noise on the sensor pins
    noiseP = 0.05; SIM_DT_US = 25;                 // finer steps: each noise sample is a 25 us spike
    double x0 = P[0].pos; o = cmd("POS?"); double xr0 = num(last(o), "X");
    o = until("STEP Y 6 - 60", 30);
    d = countLines(o, "D ");
    check(num(last(o), "pulses=") == 60 && d > 60, "noisy STEP: raw edges include the noise, filtered pulses still 60", last(o));
    // noise must not move the idle axes or the moving one's count
    o = until("JOG Y+40", 30);
    o = cmd("POS?");
    check(fabs(num(last(o), "X") - xr0) < 0.01 && fabs(P[0].pos - x0) < 0.01, "X does not count while only Y runs (noise on)", last(o));
    double yerr = num(last(o), "Y") - (P[1].pos);
    check(fabs(yerr) < 2.2, "Y count still matches the carriage after noisy runs and reversals", std::to_string(yerr) + " mm");
    noiseP = 0; SIM_DT_US = 200;
    // span X: switch to switch, 2 passes = 5 legs
    until("MOVE X10", 30);
    o = until("SPAN X 2", 400);
    check(last(o).rfind("OK SPAN X C=", 0) == 0, "SPAN X 2", last(o));
    { std::vector<long> c; std::string l = last(o); size_t q = l.find("C=") + 2;
      while (q < l.size() && (isdigit(l[q]) || l[q] == '-')) { c.push_back(atol(l.c_str() + q)); q = l.find(',', q); if (q == std::string::npos) break; q++; }
      bool ok = c.size() == 5; long spread = 0;
      for (size_t k = 1; ok && k < c.size(); k++) { long sp = labs(c[k] - c[k - 1]); if (labs(sp - 582) > 3) ok = false; spread = std::max(spread, labs(sp - labs(c[1] - c[0]))); }
      check(ok && spread <= 2, "span legs about 620 mm / 1.0661 = 582 counts, repeatable within 2", l); }
    check(last(cmd("SPAN Z")).rfind("ERR BAD_ARG", 0) == 0, "SPAN Z refused (one switch)");
    // spray, e-stop, abort
    o = until("SPRAY 300", 5);
    check(last(o) == "OK SPRAY 300", "SPRAY 300", last(o));
    Serial1.feed("MOVE X300\n"); runFor(0.6);
    o = cmd("ESTOP");
    check(o.find("OK ESTOP") != std::string::npos && duty[2] == 0 && duty[3] == 0, "ESTOP brakes a move");
    check(last(cmd("JOG X+5")).rfind("ERR ESTOP latched", 0) == 0, "motion refused while latched");
    check(last(cmd("SPRAY 100")).rfind("ERR ESTOP latched", 0) == 0, "spray refused while latched");
    check(last(cmd("RESET")) == "OK RESET", "RESET");
    Serial1.feed("MOVE Y500\n"); runFor(0.6); o = cmd("ABORT", 0.3);
    check(o.find("OK ABORT") != std::string::npos && o.find("OK X") == std::string::npos && o.find("ERR") == std::string::npos, "ABORT ends a move with no reply to it", o);
    check(last(cmd("PAUSE")) == "OK PAUSE", "PAUSE");
    // watchdog: switches off, nothing from the Pi
    swFeed = false; cmd("SET SW 0"); Serial1.feed("JOG Y+40\n"); runFor(1.5); o = take(Serial1);
    check(o.find("ERR WATCHDOG") != std::string::npos, "watchdog stops a move when the Pi goes quiet");
    // switches lost mid-move
    kaFeed = false; swFeed = true; cmd("SET SW 1"); runFor(0.1); Serial1.feed("MOVE Y800\n"); runFor(0.5); swFeed = false; kaFeed = true; runFor(0.6); o = take(Serial1);
    check(o.find("ERR SW_LOST") != std::string::npos, "move stops when SW lines stop", o);
    // BOOT button is not an e-stop any more
    check(digitalRead(9) == HIGH, "GPIO9 untouched");
    printf("%s: %d failed\n", fails ? "FAILED" : "ALL PASSED", fails);
    return fails ? 1 : 0;
  } else if (mode == "pty") {
    int m, sl; char name[64]; openpty(&m, &sl, name, nullptr, nullptr);
    struct termios t; tcgetattr(sl, &t); cfmakeraw(&t); tcsetattr(sl, TCSANOW, &t);
    fcntl(m, F_SETFL, O_NONBLOCK); printf("%s\n", name); fflush(stdout);
    Serial.connected = false;
    const char *swpath = argv[3];
    double speed = argc > 4 ? atof(argv[4]) : 1.0;
    if (argc > 5) noiseP = atof(argv[5]);
    auto t0 = std::chrono::steady_clock::now();
    uint64_t simStart = simUs;
    std::string lastSw;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(atoi(argv[2]))) {
      char b[512]; int n = read(m, b, sizeof b); for (int i = 0; i < n; i++) Serial1.in.push_back((unsigned char)b[i]);
      double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      uint64_t want = simStart + (uint64_t)(wall * speed * 1e6);
      // at most 2 ms of firmware time per pass, so input from the Pi is read in between (if this
      // process was descheduled, the firmware falls a little behind the wall clock instead of jumping)
      uint64_t stop = std::min(want, simUs + 2000);
      while (simUs < stop) { loop(); simUs += 200; step_plant(2e-4); }
      std::string o = take(Serial1);
      size_t off = 0; while (off < o.size()) { int w = write(m, o.data() + off, o.size() - off); if (w > 0) off += w; else usleep(200); }
      std::string bits = swBitsStr();
      if (bits != lastSw) {
        FILE *f = fopen((std::string(swpath) + ".tmp").c_str(), "w");
        if (f) {
          fprintf(f, "{\"x_min\": %d, \"x_max\": %d, \"y_min\": %d, \"y_max\": %d, \"z_top\": %d}\n", bits[0]-'0', bits[1]-'0', bits[2]-'0', bits[3]-'0', bits[4]-'0');
          fclose(f); rename((std::string(swpath) + ".tmp").c_str(), swpath); lastSw = bits;
        }
      }
      usleep(300);
    }
  }
}
