/********************************************************************
  Oven PID + LCD Example (EkimPid ecosystem)
  - 100°C setpoint ile fırın iç sıcaklığını PID ile sabitler.
  - 16x2 LCD ekranda sıcaklık ve PID çıkışını gösterir.
  - 120°C acil durdurma eşiği uygular.
  - Zaman + sıcaklık + çıkış verilerini Serial monitöre yazar.
  - 100K NTC (B=3950 varsayımı) ile ölçüm yapar.

  Donanım notları:
  - Sıcaklık sensörünüzün kalibrasyonunu readTemperatureC() içinde ayarlayın.
  - Solid State Relay (SSR) çıkışı ssrPin üzerinden sürülür.
  - Başlatma butonu ile güvenli olarak çalıştırılır.
  - LCD için standart LiquidCrystal pinleri kullanılmıştır.
********************************************************************/

#include <ekim_pid.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <math.h>

// Pinler
const uint8_t inputPin = A0;    // Sıcaklık sensörü analog girişi
const uint8_t ssrPin = 6;       // SSR sürme pini
const uint8_t startPin = 7;     // Başlatma butonu pini (INPUT_PULLUP)

// LCD pin dizilimi (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// PID ayarları (örnek değerler, fırınınıza göre değiştirin)
const double Kp = 10.0;
const double Ki = 0.5;
const double Kd = 30.0;

// Setpoint ve güvenlik
const double setpointC = 100.0;   // Hedef sıcaklık
const double emergencyStopC = 120.0; // Acil durdurma eşiği

// PWM penceresi (ms)
const uint32_t windowSizeMs = 2000;  // Yazılımsal PWM penceresi
const uint8_t debounceMs = 5;        // Röle/SSR bounce gecikmesi

// NTC (100K, B=3950) parametreleri
const double ntcNominal = 100000.0;    // 25°C'deki direnç (ohm)
const double tempNominal = 25.0;       // Nominal sıcaklık (°C)
const double betaCoefficient = 3950.0; // Beta katsayısı
const double seriesResistor = 100000.0; // Seri direnç (ohm)

// PID değişkenleri
double inputC = 0;
double outputMs = 0;
double setpoint = setpointC;

PID ovenPID(&inputC, &outputMs, &setpoint, Kp, Ki, Kd, DIRECT);

// EkimPid sadece softPwm fonksiyonunu kullanmak için
EkimPid tuner;

// Güvenlik bayrakları
bool emergencyStop = false;
bool sensorFault = false;
bool runEnabled = false;
int lastRaw = 0;
int lastStartState = HIGH;
uint32_t lastStartMs = 0;

// LCD ve seri güncelleme zamanlayıcıları
uint32_t lastLcdMs = 0;
uint32_t lastSerialMs = 0;
const uint32_t lcdPeriodMs = 500;
const uint32_t serialPeriodMs = 1000;

void setup() {
  pinMode(ssrPin, OUTPUT);
  digitalWrite(ssrPin, LOW);
  pinMode(startPin, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial) delay(10);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Oven PID Hazir");

  ovenPID.SetOutputLimits(0, windowSizeMs);
  ovenPID.SetSampleTime(1000);
  ovenPID.SetMode(AUTOMATIC);

  delay(1500);
  lcd.clear();
}

void loop() {
  inputC = readTemperatureC();
  uint32_t nowMs = millis();

  // Baslatma butonu (basit debounce)
  int startState = digitalRead(startPin);
  if (startState != lastStartState) {
    lastStartMs = nowMs;
    lastStartState = startState;
  }
  if ((nowMs - lastStartMs) > 50 && startState == LOW) {
    runEnabled = true;
  }

  // Sensör arızası (kopuk/kısa devre) gibi durumlarda güvenli durdurma
  sensorFault = (lastRaw < 5 || lastRaw > 1018);
  if (sensorFault) {
    emergencyStop = true;
  }

  // Acil durdurma kontrolü
  if (inputC >= emergencyStopC) {
    emergencyStop = true;
  }

  if (emergencyStop || !runEnabled) {
    outputMs = 0;
    digitalWrite(ssrPin, LOW);
  } else {
    ovenPID.Compute();
    // EkimPid softPwm ile SSR sürme
    tuner.softPwm(ssrPin, inputC, outputMs, setpointC, windowSizeMs, debounceMs);
  }

  // LCD güncelleme
  if (nowMs - lastLcdMs >= lcdPeriodMs) {
    lastLcdMs = nowMs;
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(inputC, 1);
    lcd.print("C   ");
    lcd.setCursor(0, 1);
    if (emergencyStop) {
      if (sensorFault) {
        lcd.print("SENSOR HATASI ");
      } else {
        lcd.print("ACIL DURDURMA");
      }
    } else if (!runEnabled) {
      lcd.print("BASLAT BEKLIYOR");
    } else {
      lcd.print("SP:");
      lcd.print(setpointC, 0);
      lcd.print(" Out:");
      lcd.print((int)outputMs);
      lcd.print("   ");
    }
  }

  // Seri loglama (zaman + sıcaklık + çıkış)
  if (nowMs - lastSerialMs >= serialPeriodMs) {
    lastSerialMs = nowMs;
    Serial.print("t(s)=");
    Serial.print(nowMs / 1000.0, 1);
    Serial.print("  temp(C)=");
    Serial.print(inputC, 2);
    Serial.print("  out(ms)=");
    Serial.print(outputMs, 0);
    Serial.print("  run=");
    Serial.print(runEnabled ? "ON" : "OFF");
    Serial.print("  stop=");
    Serial.print(emergencyStop ? "YES" : "NO");
    Serial.print("  sensor=");
    Serial.println(sensorFault ? "FAULT" : "OK");
  }
}

// Sensör okuma fonksiyonu (örnek, kendi sensörünüze göre uyarlayın)
double readTemperatureC() {
  int raw = analogRead(inputPin);
  lastRaw = raw;
  if (raw <= 0 || raw >= 1023) {
    return 0;
  }
  double resistance = seriesResistor / ((1023.0 / raw) - 1.0);
  double steinhart = resistance / ntcNominal;
  steinhart = log(steinhart);
  steinhart /= betaCoefficient;
  steinhart += 1.0 / (tempNominal + 273.15);
  steinhart = 1.0 / steinhart;
  return steinhart - 273.15;
}
