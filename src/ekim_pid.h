#pragma once  // Bu başlığın yalnızca bir kez dahil edilmesini sağlar.
#ifndef EKIM_PID_H_  // Çoklu dahil etmeyi önlemek için koruyucu makro başı.
#define EKIM_PID_H_  // Koruyucu makro tanımı.

class EkimPid {  // Eğri büküm (inflection point) otomatik ayarlayıcı sınıfı.

  public:  // Dışarıdan erişilebilir üyeler.

    enum Action : uint8_t {directIP, direct5T, reverseIP, reverse5T};  // Kontrol yönü ve test tipi seçenekleri.
    enum SerialMode : uint8_t {printOFF, printALL, printSUMMARY, printDEBUG};  // Seri yazdırma modları.
    enum TunerStatus : uint8_t {sample, test, tunings, runPid, timerPid};  // Ayarlayıcı durumları.
    enum TuningMethod : uint8_t { ZN_PID, DampedOsc_PID, NoOvershoot_PID, CohenCoon_PID, Mixed_PID,  // PID ayar yöntemleri.
                                  ZN_PI, DampedOsc_PI, NoOvershoot_PI, CohenCoon_PI, Mixed_PI  // PI ayar yöntemleri.
                                };  // TuningMethod enum sonu.
    EkimPid();  // Varsayılan yapıcı.
    EkimPid(float *input, float *output, TuningMethod tuningMethod, Action action, SerialMode serialMode);  // Parametreli yapıcı.
    ~EkimPid() {};  // Yıkıcı; özel işlem yok.

    void Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,  // Test ayarlarını yapılandırır.
                   uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples);  // Test süre ve örnek sayısını alır.
    uint8_t Run();  // Ana durum makinesi; her çağrıda bir adım çalıştırır.
    void Reset();  // Tüm iç değişkenleri sıfırlar.
    void printTestRun();  // Test esnası bilgilerini seri porta basar.
    void printResults();  // Sonuç bilgilerini seri porta basar.
    void printTunings();  // Ayar değerlerini seri porta basar.
    void printPidTuner(uint8_t everyNth);  // PID ayarı için periyodik yazdırma.
    void plotter(float input, float output, float setpoint, float outputScale = 1, uint8_t everyNth = 1);  // Plotter çıktısı üretir.
    float softPwm(const uint8_t relayPin, float input, float output, float setpoint = 0, uint32_t windowSize = 1000, uint8_t debounce = 0);  // Yazılımsal PWM üretir.

    // Set functions  // Ayar (set) fonksiyonları.
    void SetEmergencyStop(float e_Stop);  // Acil durdurma eşiğini ayarlar.
    void SetControllerAction(Action Action);  // Kontrol yönünü ayarlar.
    void SetSerialMode(SerialMode SerialMode);  // Seri yazdırma modunu ayarlar.
    void SetTuningMethod(TuningMethod TuningMethod);  // Ayarlama yöntemini seçer.

    // Query functions  // Sorgu fonksiyonları.
    float GetKp();                  // Oransal kazancı döndürür.
    float GetKi();                  // İntegral kazancı döndürür.
    float GetKd();                  // Türev kazancı döndürür.
    float GetTi();                  // İntegral zamanı döndürür.
    float GetTd();                  // Türev zamanı döndürür.
    float GetProcessGain();         // Proses kazancını döndürür.
    float GetDeadTime();            // Proses ölü zamanı (saniye) döndürür.
    float GetTau();                 // Proses zaman sabiti (saniye) döndürür.
    uint8_t GetControllerAction();  // Kontrol yönünü döndürür.
    uint8_t GetSerialMode();        // Seri modu döndürür.
    uint8_t GetTuningMethod();      // Ayar yöntemini döndürür.
    void GetAutoTunings(float * kp, float * ki, float * kd);  // Otomatik hesaplanan PID değerlerini döndürür.

  private:  // Sadece sınıf içinde erişilebilen üyeler.

    Action _action;  // Seçilen kontrol yönü.
    SerialMode _serialMode;  // Seçilen seri yazdırma modu.
    TunerStatus _tunerStatus;  // Durum makinesi durumu.
    TuningMethod _tuningMethod;  // Seçilen ayar yöntemi.

    float *_input, *_output, _settlePeriodUs, _samplePeriodUs, _tangentPeriodUs;  // Giriş/çıkış işaretçileri ve zamanlar.
    float  _inputSpan, _outputSpan, _outputStart, _outputStep;  // Giriş/çıkış aralık ve adım değerleri.
    float eStop, pvInst, pvAvg, pvIp, pvMax, pvPk, pvInstRes, pvAvgRes, slopeIp, pvTangent, pvTangentPrev = 0, pvStart;  // Proses değişkenleri ve ölçüm değerleri.
    float _kp, _ki, _kd, _Ku, _Tu, _td, _R, _Ko;  // PID ve proses parametreleri.

    uint16_t _bufferSize, _samples, sampleCount = 0, pvPkCount = 0, ipCount = 0, plotCount = 0, eStopAbort = 0;  // Sayısal sayaçlar.
    uint32_t _settleTimeSec, _testTimeSec, usPrev = 0, settlePrev = 0, usStart, us, ipUs;  // Zamanlayıcı değişkenleri.

    const float kexp = 4.3004; // (1 / exp(-1)) / (1 - exp(-1)) sabit katsayı.
    const float epsilon = 0.0001f;  // Hassasiyet eşiği.
};  // EkimPid sınıfı sonu.
#endif  // EKIM_PID_H_ koruyucu makro sonu.
