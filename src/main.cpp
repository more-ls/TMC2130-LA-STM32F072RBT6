// TMC2130 test bench: play with microsteps, speed, current and chopper mode over serial.
// Motor: Nanotec SC2818S0604-A (NEMA11, 1.8 deg, 0.67 A, 6.2 ohm, 5.76 mH)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ---- wiring: Mocroscope hat, Y axis (U7), same as frog-code ----
#define PIN_MOSI  PA7
#define PIN_MISO  PA6
#define PIN_SCK   PA5
#define PIN_CS    PC6         // Y_CS
#define PIN_STEP  PB_1_ALT2   // Y_STEP on PB1, TIM14_CH1 -> hardware step pulses
#define PIN_EN    PB11        // TMC_EN, shared by all drivers, HIGH = enabled
#define PIN_CS_X  PC5         // other drivers on the same SPI bus, kept deselected
#define PIN_CS_Z  PC10
#define PIN_SDA   PB7         // DIR comes from MCP23017 GPB1
#define PIN_SCL   PB6
#define MCP_ADDR  0x20
#define DIR_MASK  (1 << 1)
#define PIN_RX    PA3         // board serial
#define PIN_TX    PA2

// ---- driver / motor ----
const float RSENSE    = 0.11f;  // ohm, sense resistor on your driver board (most modules: 0.11)
const int   FULLSTEPS = 200;    // 1.8 deg motor

// ---- settings you can change at runtime ----
int   microsteps = 16;    // 1..256, interpolated to 256 internally
float rpmTarget  = 0;     // negative = reverse
float accel      = 600;   // rpm per second
int   run_mA     = 500;   // RMS; motor is rated 0.67 A
int   hold_mA    = 250;
bool  stealth    = true;  // true = stealthChop (quiet), false = spreadCycle

enum { GCONF = 0x00, GSTAT = 0x01, IOIN = 0x04, IHOLD_IRUN = 0x10, TPOWERDOWN = 0x11,
       TSTEP = 0x12, TPWMTHRS = 0x13, CHOPCONF = 0x6C, DRV_STATUS = 0x6F, PWMCONF = 0x70 };

SPIClass spi(PIN_MOSI, PIN_MISO, PIN_SCK);
HardwareSerial uart(PIN_RX, PIN_TX);
HardwareTimer *stepTimer;
uint32_t stepCh;
float rpm = 0;  // current ramped speed

uint32_t tmcXfer(uint8_t addr, uint32_t data) {
  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  spi.transfer(addr);  // status byte, ignored
  uint32_t r = 0;
  for (int s = 24; s >= 0; s -= 8) r = (r << 8) | spi.transfer(data >> s);
  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
  return r;
}
void tmcWrite(uint8_t reg, uint32_t val) { tmcXfer(reg | 0x80, val); }
uint32_t tmcRead(uint8_t reg) { tmcXfer(reg, 0); return tmcXfer(reg, 0); }  // reply comes one datagram later

// I_rms = (CS+1)/32 * Vfs/(Rsense+0.02) / sqrt(2). Use the low range (vsense=1) when it fits, for finer steps.
void tmcApply() {
  float perVfs = 32 * 1.41421f * (RSENSE + 0.02f) / 1000;  // (CS+1)*Vfs per mA
  bool vsense = run_mA * perVfs / 0.18f <= 32;
  float vfs = vsense ? 0.18f : 0.32f;
  auto cs = [&](int mA) { return (uint32_t)constrain(lroundf(mA * perVfs / vfs) - 1, 0L, 31L); };
  uint32_t mres = 8 - __builtin_ctz(microsteps);  // 256 -> 0 ... 1 -> 8

  tmcWrite(GCONF, stealth ? 1 << 2 : 0);                     // en_pwm_mode, internal current reference
  tmcWrite(IHOLD_IRUN, cs(hold_mA) | cs(run_mA) << 8 | 6UL << 16);
  tmcWrite(TPOWERDOWN, 10);                                  // ~0.2 s to hold current
  tmcWrite(TPWMTHRS, 0);                                     // stealthChop at all speeds (if enabled)
  tmcWrite(CHOPCONF, 3 | 4 << 4 | 1 << 7 | 2UL << 15         // TOFF=3 HSTRT=4 HEND=1 TBL=2 (datasheet defaults)
                     | (uint32_t)vsense << 17 | mres << 24 | 1UL << 28);  // intpol: 256 microsteps internally
  tmcWrite(PWMCONF, 200 | 1 << 8 | 1UL << 18);               // PWM_AMPL=200 PWM_GRAD=1 autoscale
}

void mcpWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
void setDir(bool reverse) {
  static int last = -1;
  if (reverse == last) return;
  last = reverse;
  mcpWrite(0x15, reverse ? DIR_MASK : 0);  // OLATB
}

void setStepRate(float hz) {
  if (hz < 1) { stepTimer->setCaptureCompare(stepCh, 0, TICK_COMPARE_FORMAT); return; }  // output held low
  stepTimer->setOverflow(hz, HERTZ_FORMAT);
  stepTimer->setCaptureCompare(stepCh, 50, PERCENT_COMPARE_FORMAT);
}

void updateRamp() {
  static uint32_t last = micros();
  uint32_t now = micros();
  float dv = accel * (now - last) * 1e-6f;
  last = now;
  rpm += constrain(rpmTarget - rpm, -dv, dv);
  setDir(rpm < 0);
  setStepRate(fabsf(rpm) / 60 * FULLSTEPS * microsteps);
}

void printStatus() {
  uint32_t d = tmcRead(DRV_STATUS);
  uart.printf("\nmicrosteps %d  rpm %d/%d  accel %d  run %d mA  hold %d mA  %s\n", microsteps, (int)rpm,
              (int)rpmTarget, (int)accel, run_mA, hold_mA, stealth ? "stealthChop" : "spreadCycle");
  uart.printf("TSTEP %lu  CS_ACTUAL %lu  SG %lu%s%s%s%s%s%s%s\n", tmcRead(TSTEP), (d >> 16) & 31,
              d & 1023, d & 1UL << 31 ? "  STANDSTILL" : "", d & 1UL << 30 ? "  OPEN_B" : "",
              d & 1UL << 29 ? "  OPEN_A" : "", d & 1UL << 28 ? "  SHORT_B" : "", d & 1UL << 27 ? "  SHORT_A" : "",
              d & 1UL << 26 ? "  OVERTEMP_WARN" : "", d & 1UL << 25 ? "  OVERTEMP" : "");
}

void printHelp() {
  uart.println("\nv<rpm> speed (neg = reverse, 0 = stop)   a<rpm/s> accel   m<1..256> microsteps\n"
               "i<mA> run current   h<mA> hold current   s<0/1> stealthChop   e<0/1> driver enable   ? status");
}

void handleSerial() {
  if (!uart.available()) return;
  char c = uart.read();
  if (c == '?') { printStatus(); return; }
  if (!strchr("vamihse", c)) { if (c > ' ') printHelp(); return; }
  float x = uart.parseFloat();
  switch (c) {
    case 'v': rpmTarget = x; break;
    case 'a': accel = max(x, 1.0f); break;
    case 'm': if (x >= 1 && x <= 256 && !((int)x & ((int)x - 1))) microsteps = x;
              else uart.println("microsteps must be 1,2,4..256"); break;
    case 'i': run_mA = constrain(x, 0, 1000); break;
    case 'h': hold_mA = constrain(x, 0, 1000); break;
    case 's': stealth = x; break;
    case 'e': digitalWrite(PIN_EN, x ? HIGH : LOW); break;
  }
  if (strchr("mihs", c)) tmcApply();
  printStatus();
}

void setup() {
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);  // off until configured
  for (uint32_t cs : {PIN_CS, PIN_CS_X, PIN_CS_Z}) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  mcpWrite(0x01, 0xFF & ~DIR_MASK);  // IODIRB: only DIR pin is an output
  uart.begin(115200);
  uart.setTimeout(50);
  spi.begin();

  PinName p = PIN_STEP;
  stepTimer = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(p, PinMap_TIM));
  stepCh = STM_PIN_CHANNEL(pinmap_function(p, PinMap_TIM));
  stepTimer->setPWM(stepCh, PIN_STEP, 1000, 0);  // running, output low

  delay(100);
  uint8_t version = tmcRead(IOIN) >> 24;
  uart.printf("\nTMC version 0x%02x %s\n", version,
              version == 0x11   ? "TMC2130 ok"
              : version == 0x30 ? "TMC5130/5160: has its own ramp generator, STEP/DIR only work with SD_MODE=1"
                                : "NOT FOUND: check SPI wiring, CS pin and motor supply (VM)");
  tmcRead(GSTAT);  // clear reset flag
  tmcApply();
  digitalWrite(PIN_EN, HIGH);
  printHelp();
  printStatus();
}

void loop() {
  handleSerial();
  updateRamp();
  // driver lost power (VM off/on) -> registers are back at defaults, configure again
  static uint32_t lastCheck;
  if (millis() - lastCheck > 500) {
    lastCheck = millis();
    uint32_t g = tmcRead(GSTAT);  // reading clears it
    if (g < 8 && (g & 1)) { uart.println("driver reset detected, reconfiguring"); tmcApply(); }
  }
  delay(1);
}
