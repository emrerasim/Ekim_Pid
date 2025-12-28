/****************************************************************************************  
   EkimPid Library for Arduino - Version 2.4.0  
   by dlloydev https://github.com/Dlloydev/EkimPid  
   Licensed under the MIT License.  

  This is an open loop PID autotuner using a novel s-curve inflection point test method.  
  Tuning parameters are determined in about ½Tau on a first-order system with time delay.  
  Full 5Tau testing and multiple serial output options are provided.  
 ****************************************************************************************/  

#if ARDUINO >= 100  // Arduino çekirdek sürümüne göre başlık seçimi.
#include "Arduino.h"  // Arduino temel API başlığı.
#else  // Arduino çekirdeği 100'den küçükse.
#include "WProgram.h"  // Eski Arduino çekirdek başlığı.
#endif  // Ön işlemci koşulu sonu.
#include "ekim_pid_tan.h"  // EkimPidTan sınıfı bildirimi.
#include "ekim_pid.h"  // EkimPid sınıfı bildirimi.

EkimPidTan::EkimPidTan() {}  // EkimPidTan için boş varsayılan yapıcı.
EkimPidTan tangent = EkimPidTan();  // Kaydırmalı teğet hesabı için global nesne.

EkimPid::EkimPid() {  // EkimPid varsayılan yapıcı başlangıcı.
  _input = nullptr;  // Giriş işaretçisini null yap.
  _output = nullptr;  // Çıkış işaretçisini null yap.
  EkimPid::Reset();  // Tüm iç değişkenleri sıfırla.
}  // EkimPid varsayılan yapıcı sonu.

EkimPid::EkimPid(float *input, float *output, TuningMethod tuningMethod, Action action, SerialMode serialMode) {  // Parametreli yapıcı başlangıcı.
  _input = input;  // Giriş işaretçisini ata.
  _output = output;  // Çıkış işaretçisini ata.
  _tuningMethod = tuningMethod;  // Ayar yöntemini ata.
  _action = action;  // Kontrol yönünü ata.
  _serialMode = serialMode;  // Seri yazdırma modunu ata.
  EkimPid::Reset();  // Tüm iç değişkenleri sıfırla.
}  // Parametreli yapıcı sonu.

void EkimPid::Reset() {  // Ayarlayıcı durumunu sıfırlayan fonksiyon.
  _tunerStatus = test;  // Durumu test moduna al.
  *_output = _outputStart;  // Çıkışı başlangıç değerine çek.
  usPrev = micros();  // Önceki zaman damgasını kaydet.
  settlePrev = usPrev;  // Yerleşme zamanı başlangıcı.
  ipUs = 0;  // Eğri büküm zamanı sıfırla.
  us = 0;  // Geçen süreyi sıfırla.
  _Ku = 0.0f;  // Proses kazancını sıfırla.
  _Tu = 0.0f;  // Zaman sabitini sıfırla.
  _td = 0.0f;  // Ölü zamanı sıfırla.
  _kp = 0.0f;  // Kp sıfırla.
  _ki = 0.0f;  // Ki sıfırla.
  _kd = 0.0f;  // Kd sıfırla.
  pvIp = 0.0f;  // İnfleksiyon PV değerini sıfırla.
  pvMax = 0.0f;  // Maksimum PV'yi sıfırla.
  pvPk = 0.0f;  // Tepe PV'yi sıfırla.
  slopeIp = 0.0f;  // İnfleksiyon eğimini sıfırla.
  pvTangent = 0.0f;  // Teğet değerini sıfırla.
  pvTangentPrev = 0.0f;  // Önceki teğet değerini sıfırla.
  pvAvg = pvInst;  // Ortalama PV'yi anlık PV'ye eşitle.
  pvStart = pvInst;  // Başlangıç PV'yi anlık PV'ye eşitle.
  pvInstRes = pvInst;  // Anlık PV çözünürlüğünü ayarla.
  ipCount = 0;  // İnfleksiyon sayaç sıfırla.
  plotCount = 0;  // Plot sayacını sıfırla.
  sampleCount = 0;  // Örnek sayacını sıfırla.
  pvPkCount = 0;  // Tepe altı sayaç sıfırla.
}  // Reset fonksiyonu sonu.

void EkimPid::Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,  // Konfigürasyon fonksiyonu başı.
                      uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples) {  // Test ve örnek parametreleri.
  EkimPid::Reset();  // Ayarları sıfırla.
  _inputSpan = inputSpan;  // Giriş aralığını kaydet.
  eStop = inputSpan;  // Acil durdurma eşiğini giriş aralığına ayarla.
  _outputSpan = outputSpan;  // Çıkış aralığını kaydet.
  _outputStart = outputStart;  // Çıkış başlangıç değerini kaydet.
  _outputStep = outputStep;  // Çıkış adım değerini kaydet.
  _testTimeSec = testTimeSec;  // Test süresini kaydet.
  _settleTimeSec = settleTimeSec;  // Yerleşme süresini kaydet.
  _samples = samples;  // Örnek sayısını kaydet.
  _bufferSize = (uint16_t)(_samples * 0.06);  // Ortalama için tampon boyutunu hesapla.
  _samplePeriodUs = (float)(_testTimeSec * 1000000.0f) / _samples;  // Örnekleme periyodunu mikro saniye cinsinden hesapla.
  _tangentPeriodUs = _samplePeriodUs * (_bufferSize - 1);  // Teğet periyodunu hesapla.
  _settlePeriodUs = (float)(_settleTimeSec * 1000000.0f);  // Yerleşme periyodunu mikro saniyeye çevir.
  tangent.begin(_bufferSize);  // Teğet tamponunu başlat.
}  // Configure fonksiyonu sonu.

uint8_t EkimPid::Run() {  // Ana döngü fonksiyonu.

  uint32_t usNow = micros();  // Şu anki zamanı al.
  uint32_t usElapsed = usNow - usPrev;  // Son örnekten beri geçen süre.
  uint32_t settleElapsed = usNow - settlePrev;  // Yerleşme başlangıcından beri geçen süre.
  us = usNow - usStart;  // Test başlangıcından beri geçen süre.

  switch (_tunerStatus) {  // Durum makinesi seçimi.

    case sample:  // Örnekleme durumunda.
      _tunerStatus = test;  // Bir sonraki durumu test olarak ayarla.
      return test;  // Test durumunu geri döndür.
      break;  // Güvenlik için break.

    case test: // run inflection point test method  // İnfleksiyon noktası testi.
      if (pvInst > eStop && !eStopAbort) {  // Acil durdurma koşulu kontrolü.
        EkimPid::Reset();  // Ayarları sıfırla.
        sampleCount = _samples + 1;  // Testin bittiğini işaretle.
        eStopAbort = 1;  // Acil durdurmayı işaretle.
        Serial.println(F(" ABORT: pvInst > eStop"));  // Seri porta hata yazdır.
        break;  // Durumdan çık.
      }  // Acil durdurma bloğu sonu.
      if (settleElapsed >= _settlePeriodUs) { // if settling period has expired  // Yerleşme süresi doldu mu?
        if (sampleCount == 1) *_output = _outputStep;  // İlk örnekten sonra adımı uygula.
        if (usElapsed >= _samplePeriodUs) { // ready to process a sample  // Örnekleme zamanı geldi mi?
          usPrev = usNow;  // Son örnek zamanını güncelle.
          if (sampleCount <= _samples) { // continue testing  // Test devam ediyor mu?

            // get pvInst and pvAvg (buffer) resolution  // Anlık ve ortalama PV çözünürlüğü hesapla.
            float lastPvInst = pvInst;  // Önceki anlık PV.
            float lastPvAvg = pvAvg;  // Önceki ortalama PV.
            pvInst = *_input;  // Yeni anlık PV oku.
            pvAvg = tangent.avgVal(pvInst);  // Tampondan ortalama PV hesapla.
            float pvInstResolution = fabs(pvInst - lastPvInst);  // Anlık PV değişim miktarı.
            float pvAvgResolution = fabs(pvAvg - lastPvAvg);  // Ortalama PV değişim miktarı.
            if (pvInstResolution > epsilon && pvInstResolution < pvInstRes) pvInstRes = pvInstResolution;  // En küçük anlık çözünürlüğü güncelle.
            if (pvAvgResolution > epsilon && pvAvgResolution < pvAvgRes) pvAvgRes = pvAvgResolution;  // En küçük ortalama çözünürlüğü güncelle.

            if (sampleCount == 0) { // initialize at first sample  // İlk örnekte başlatma.
              tangent.init(pvInst);  // Tamponu ilk PV ile doldur.
              pvAvg = pvInst;  // Ortalama PV'yi anlık PV'ye eşitle.
              pvInstResolution = 0.0f;  // Anlık çözünürlüğü sıfırla.
              pvAvgResolution = 0.0f;  // Ortalama çözünürlüğü sıfırla.
              pvInstRes = pvInst;  // Anlık çözünürlük referansını ayarla.
              pvAvgRes = pvInst;  // Ortalama çözünürlük referansını ayarla.
              pvStart = pvInst;  // Başlangıç PV'yi kaydet.
              usStart = usNow;  // Test başlangıç zamanını kaydet.
              us = 0;  // Geçen süreyi sıfırla.
            }  // İlk örnek başlatma sonu.

            //To determine the point of inflection, we'll use a sliding tangent line.  // İnfleksiyon noktasını kaydırmalı teğet ile bulacağız.
            //https://en.wikipedia.org/wiki/Inflection_point#/media/File:Animated_illustration_of_inflection_point.gif  // Açıklayıcı bağlantı.
            pvTangent = pvAvg - tangent.startVal();  // Teğet değerini hesapla.

            // check for dead time  // Ölü zamanı kontrol et.
            bool dt = false;  // Ölü zaman bayrağı.
            if (_action == directIP || _action == direct5T) {  // Doğrudan yön kontrolü.
              (pvAvg > pvStart + pvInstRes + epsilon) ?  dt = true : dt = false;  // Başlangıçtan anlamlı sapma var mı?
            } else { // reverse  // Ters yönde kontrol.
              (pvAvg < pvStart - pvInstRes - epsilon) ?  dt = true : dt = false;  // Ters yönde sapma var mı?
            }  // Yön kontrolü sonu.
            if (!_td && dt) _td = us * 0.000001f;  // Ölü zamanı saniye cinsinden kaydet.

            // check for inflection point  // İnfleksiyon noktasını kontrol et.
            bool ipcount = false;  // İnfleksiyon sayaç tetikleyici.
            if (_action == directIP || _action == direct5T) {  // Doğrudan yön için kontrol.
              if (pvTangent > slopeIp + epsilon) ipcount = true;  // Teğet eğimi artıyorsa.
              if (pvTangent < 0 + epsilon) ipCount = 0;  // Teğet düz ya da negatifse sayaç sıfırla.
            } else { // reverse  // Ters yön için kontrol.
              if (pvTangent < slopeIp - epsilon) ipcount = true;  // Teğet eğimi azalıyorsa.
              if (pvTangent > 0 - epsilon) ipCount = 0;  // Teğet düz ya da pozitifse sayaç sıfırla.
            }  // Yön kontrolü sonu.
            if (ipcount) {  // İnfleksiyon sayaç tetiklendi mi?
              ipCount = 0;  // Sayaç sıfırla.
              slopeIp = pvTangent;  // İnfleksiyon eğimini güncelle.
            }  // ipcount bloğu sonu.
            ipCount++;  // İnfleksiyon sayacını artır.
            if ((_action == directIP || _action == reverseIP) && (ipCount == ((uint16_t)(_samples / 16)))) { // reached inflection point  // İnfleksiyon noktası yakalandı mı?
              sampleCount = _samples;  // Testi sonlandırmak için örnek sayısını ayarla.
              ipUs = us;  // İnfleksiyon zamanını kaydet.
              pvIp = pvAvg;  // İnfleksiyon PV değerini kaydet.

              // apparent pvMax  Re: "Step response with arbitrary initial conditions" https://en.wikipedia.org/wiki/Time_constant  // Görünür maksimum PV hesabı.
              pvMax = pvIp + (slopeIp * kexp);  // Görünür maksimum PV'yi hesapla.

              //apparent tangent from pvStart to pvMax crossing points  // pvStart ile pvMax arasında teğet süre.
              _Tu = (((pvMax - pvStart) / slopeIp) * _tangentPeriodUs * 0.000001f) - _td;  // Zaman sabitini hesapla.
            }  // İnfleksiyon noktası bloğu sonu.

            if (_action == direct5T || _action == reverse5T) { // continue testing to maximum input  // 5τ testini sürdür.
              if (sampleCount >= _samples - 1) sampleCount = _samples - 2;  // Örnek sayısını sınırla.
              if (us > _testTimeSec * 100000) {  // 10% of testTimeSec has elapsed  // Test süresinin %10'u geçti mi?
                if (pvAvg > pvPk) {  // Yeni bir tepe oluştu mu?
                  pvPk = pvAvg + (_bufferSize * 0.2f * pvAvgRes); // set a new boosted peak  // Tepeyi tampon ve çözünürlükle güçlendir.
                  pvPkCount = 0;  // reset the "below peak" counter  // Tepe altı sayacını sıfırla.
                } else {  // Tepe oluşmadıysa.
                  pvPkCount++;  // count up while pvAvg is below the boosted peak  // Tepe altı sayacı artır.
                }  // Tepe kontrolü sonu.
                if (pvPkCount == ((uint16_t)(1.2 * _bufferSize))) {  // test done  // Test bitti mi?
                  pvPkCount++;  // Sayaç artır.
                  sampleCount = _samples;  // Testi bitirmek için örnek sayısını ayarla.
                  pvMax = pvAvg + (pvInst - pvStart) * 0.05f;  // assume 3τ, so increase pvMax by 5%  // pvMax'i %5 artır.
                  _Tu = (us * 1.6667 * 0.000001f * 0.286f) - _td; // scale us to 5τ in seconds, then multiply by 0.286 for τ  // Tau hesapla.
                }  // Test bitiş kontrolü sonu.
              }  // %10 süre kontrolü sonu.
            }  // 5τ test bloğu sonu.

            if (sampleCount == _samples) {  // testing complete  // Test tamamlandı.
              _R = _td / _Tu;  // R oranını hesapla.

              // process gain  // Proses kazancını hesapla.
              _Ku =  fabs(((pvMax - pvStart) / _inputSpan) / ((_outputStep - _outputStart) / _outputSpan));  // Ku hesapla.

              _kp = EkimPid::GetKp();  // Kp değerini hesapla.
              _ki = EkimPid::GetKi();  // Ki değerini hesapla.
              _kd = EkimPid::GetKd();  // Kd değerini hesapla.

              EkimPid::printResults();  // Sonuçları yazdır.
              _tunerStatus = tunings;  // Durumu tunings yap.
              return tunings;  // Tunings durumunu döndür.
              break;  // Güvenlik için break.
            }  // Test tamamlandı bloğu sonu.
            EkimPid::printTestRun();  // Test akışını yazdır.
            pvTangentPrev = pvTangent;  // Önceki teğet değerini güncelle.
          } else _tunerStatus = tunings;  // Örnek sayısı aşıldıysa tunings durumuna geç.
          sampleCount++;  // Örnek sayacını artır.
          _tunerStatus = sample;  // Bir sonraki durumu sample yap.
          return sample;  // Sample durumunu döndür.
        }  // Örnek zamanı kontrolü sonu.

      } else {  // settling  // Yerleşme süresi içindeyiz.
        if (usElapsed >= _samplePeriodUs && !eStopAbort) {  // Örnekleme zamanı geldi mi?
          *_output = _outputStart;  // Çıkışı başlangıç değerinde tut.
          usPrev = usNow;  // Zaman damgasını güncelle.
          pvInst = *_input;  // Anlık PV değerini oku.
          if (_serialMode == printALL || _serialMode == printDEBUG) {  // Seri yazdırma modu kontrolü.
            Serial.print(F(" sec: "));     Serial.print((float)((_settlePeriodUs - settleElapsed) * 0.000001f), 4);  // Kalan süreyi yazdır.
            Serial.print(F("  out: ")); Serial.print(*_output);  // Çıkış değerini yazdır.
            Serial.print(F("  pv: "));     Serial.print(pvInst, 3);  // PV değerini yazdır.
            Serial.println(F("  settling  ⤳⤳"));  // Yerleşme durumunu yazdır.
          }  // Seri yazdırma bloğu sonu.
          _tunerStatus = sample;  // Durumu sample yap.
          return sample;  // Sample durumunu döndür.
        }  // Yerleşme sırasında örnek zamanı kontrolü sonu.
      }  // Yerleşme bloğu sonu.
      break;  // test durumu sonu.

    case tunings:  // Tunings durumu.
      _tunerStatus = timerPid;  // Bir sonraki durumu timerPid yap.
      return timerPid;  // timerPid durumunu döndür.
      break;  // Güvenlik için break.

    case runPid:  // PID çalıştırma durumu.
      if (pvInst > eStop && !eStopAbort) {  // Acil durdurma kontrolü.
        EkimPid::Reset();  // Reset çağır.
        sampleCount = _samples + 1;  // Testi bitir.
        eStopAbort = 1;  // Acil durdurmayı işaretle.
        Serial.println(F(" ABORT: pvInst > eStop"));  // Seri uyarı yazdır.
      }  // Acil durdurma bloğu sonu.
      _tunerStatus = timerPid;  // Bir sonraki durumu timerPid yap.
      return timerPid;  // timerPid durumunu döndür.
      break;  // Güvenlik için break.

    case timerPid:  // PID zamanlayıcı durumu.
      if (usElapsed >= _samplePeriodUs) {  // Örnek zamanı geldi mi?
        usPrev = usNow;  // Zaman damgasını güncelle.
        _tunerStatus = runPid;  // Bir sonraki durumu runPid yap.
        return runPid;  // runPid durumunu döndür.
      } else {  // Zaman dolmadıysa.
        _tunerStatus = timerPid;  // Durumu aynı tut.
        return timerPid;  // timerPid durumunu döndür.
      }  // Zaman kontrolü sonu.
      break;  // Güvenlik için break.

    default:  // Beklenmeyen durum.
      _tunerStatus = timerPid;  // Varsayılan olarak timerPid yap.
      return timerPid;  // timerPid durumunu döndür.
      break;  // Güvenlik için break.
  }  // switch sonu.
  return timerPid;  // Varsayılan dönüş.
}  // Run fonksiyonu sonu.

void EkimPid::SetEmergencyStop(float e_Stop) {  // Acil durdurma eşiğini ayarlar.
  eStop = e_Stop;  // eStop değerini güncelle.
}  // SetEmergencyStop sonu.

void EkimPid::SetControllerAction(Action Action) {  // Kontrol yönünü ayarlar.
  _action = Action;  // _action alanını güncelle.
}  // SetControllerAction sonu.

void EkimPid::SetSerialMode(SerialMode SerialMode) {  // Seri modu ayarlar.
  _serialMode = SerialMode;  // _serialMode alanını güncelle.
}  // SetSerialMode sonu.

void EkimPid::SetTuningMethod(TuningMethod TuningMethod) {  // Ayar yöntemini ayarlar.
  _tuningMethod = TuningMethod;  // _tuningMethod alanını güncelle.
}  // SetTuningMethod sonu.

void EkimPid::printPidTuner(uint8_t everyNth) {  // PID tuner çıktısı.
  if (sampleCount < _samples) {  // Test devam ediyor mu?
    if (plotCount == 0 || plotCount >= everyNth) {  // Yazdırma aralığı kontrolü.
      plotCount = 1;  // Plot sayacını sıfırla.
      Serial.print(us * 0.000001f, 4);  Serial.print(F(", "));  // Zamanı yazdır.
      Serial.print(*_output);           Serial.print(F(", "));  // Çıkışı yazdır.
      Serial.println(pvAvg);  // Ortalama PV'yi yazdır.
    } else plotCount++;  // Aksi halde sayacı artır.
  }  // Test devam kontrolü sonu.
}  // printPidTuner sonu.

void EkimPid::plotter(float input, float output, float setpoint, float outputScale, uint8_t everyNth) {  // Plotter çıktısı.
  if (plotCount >= everyNth) {  // Yazdırma sıklığını kontrol et.
    plotCount = 1;  // Plot sayacını sıfırla.
    Serial.print(F("Setpoint:"));  Serial.print(setpoint);              Serial.print(F(", "));  // Setpoint yazdır.
    Serial.print(F("Input:"));     Serial.print(input);                 Serial.print(F(", "));  // Input yazdır.
    Serial.print(F("Output:"));    Serial.print(output * outputScale);  Serial.print(F(","));  // Output yazdır.
    Serial.println();  // Satır sonu yazdır.
  } else plotCount++;  // Aksi halde sayacı artır.
}  // plotter sonu.

void EkimPid::printTestRun() {  // Test çalışma çıktısı.

  if (sampleCount < _samples) {  // Test devam ediyor mu?
    if (_serialMode == printALL || _serialMode == printDEBUG) {  // Seri yazdırma modu kontrolü.
      Serial.print(F(" sec: "));           Serial.print(us * 0.000001f, 4);  // Geçen süreyi yazdır.
      Serial.print(F("  out: "));          Serial.print(*_output);  // Çıkışı yazdır.
      Serial.print(F("  pv: "));           Serial.print(pvInst, 3);  // PV değerini yazdır.
      //Serial.print(F("  pvAvg: "));      Serial.print(pvAvg, 3);  // Ortalama PV yazdırma (kapalı).
      if (_serialMode == printDEBUG && (_action == direct5T || _action == reverse5T)) {  // Debug ve 5T modunda.
        Serial.print(F("  pvPk: "));       Serial.print(pvPk, 3);  // Tepe PV yazdır.
        Serial.print(F("  pvPkCount: "));  Serial.print(pvPkCount);  // Tepe sayacını yazdır.
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);  // İnfleksiyon sayacını yazdır.
      }  // 5T debug bloğu sonu.
      if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {  // Debug ve IP modunda.
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);  // İnfleksiyon sayacını yazdır.
      }  // IP debug bloğu sonu.
      Serial.print(F("  tan: "));                       Serial.print(pvTangent, 3);  // Teğet değerini yazdır.
      if (pvInst > 0.9f * eStop)                        Serial.print(F(" ⚠"));  // Acil durdurmaya yakınsa uyarı.
      if (pvTangent - pvTangentPrev > 0 + epsilon)      Serial.println(F(" ↗"));  // Teğet artıyorsa ok yazdır.
      else if (pvTangent - pvTangentPrev < 0 - epsilon) Serial.println(F(" ↘"));  // Teğet azalıyorsa ok yazdır.
      else                                              Serial.println(F(" →"));  // Teğet sabitse ok yazdır.
    }  // Seri yazdırma bloğu sonu.
  }  // Test devam kontrolü sonu.
}  // printTestRun sonu.

void EkimPid::printTunings() {  // Ayarları yazdır.
  Serial.print(F(" Tuning Method: "));  // Ayar yöntemi başlığı.
  if (_tuningMethod == ZN_PID) Serial.println(F("ZN_PID"));  // ZN_PID seçildiyse yazdır.
  else if (_tuningMethod == DampedOsc_PID) Serial.println(F("Damped_PID"));  // Damped PID yazdır.
  else if (_tuningMethod == NoOvershoot_PID) Serial.println(F("NoOvershoot_PID"));  // Aşmasız PID yazdır.
  else if (_tuningMethod == CohenCoon_PID) Serial.println(F("CohenCoon_PID"));  // Cohen-Coon PID yazdır.
  else if (_tuningMethod == Mixed_PID) Serial.println(F("Mixed_PID"));  // Karışık PID yazdır.
  else if (_tuningMethod == ZN_PI) Serial.println(F("ZN_PI"));  // ZN_PI yazdır.
  else if (_tuningMethod == DampedOsc_PI) Serial.println(F("Damped_PI"));  // Damped PI yazdır.
  else if (_tuningMethod == NoOvershoot_PI) Serial.println(F("NoOvershoot_PI"));  // Aşmasız PI yazdır.
  else if (_tuningMethod == CohenCoon_PI) Serial.println(F("CohenCoon_PI"));  // Cohen-Coon PI yazdır.
  else Serial.println(F("Mixed_PI"));  // Varsayılan Mixed PI yazdır.
  Serial.print(F("  Kp: ")); Serial.println(EkimPid::GetKp(), 3);  // Kp değerini yazdır.
  Serial.print(F("  Ki: ")); Serial.print(EkimPid::GetKi(), 3); Serial.print(F("  Ti: ")); Serial.println(EkimPid::GetTi(), 3);  // Ki ve Ti yazdır.
  Serial.print(F("  Kd: ")); Serial.print(EkimPid::GetKd(), 3); Serial.print(F("  Td: ")); Serial.println(EkimPid::GetTd(), 3);  // Kd ve Td yazdır.
  Serial.println();  // Boş satır yazdır.
}  // printTunings sonu.

void EkimPid::printResults() {  // Sonuçları yazdır.
  if (_serialMode == printALL || _serialMode == printDEBUG || _serialMode == printSUMMARY) {  // Yazdırma modu kontrolü.
    Serial.println();  // Boş satır yazdır.
    Serial.print(F(" Controller Action: "));  // Kontrol yönü başlığı.
    if (_action == directIP) Serial.println(F("directIP"));  // directIP yazdır.
    else if (_action == direct5T) Serial.println(F("direct5T"));  // direct5T yazdır.
    else if (_action == reverseIP) Serial.println(F("reverseIP"));  // reverseIP yazdır.
    else Serial.println(F("reverse5T"));  // reverse5T yazdır.
    Serial.println();  // Boş satır yazdır.
    Serial.print(F(" Output Start:      "));  Serial.println(_outputStart);  // Çıkış başlangıcı yazdır.
    Serial.print(F(" Output Step:       "));  Serial.println(_outputStep);  // Çıkış adımı yazdır.
    Serial.print(F(" Sample Sec:        "));  Serial.println(_samplePeriodUs * 0.000001f, 4);  // Örnek periyodu yazdır.
    Serial.println();  // Boş satır yazdır.
    if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {  // Debug ve IP modunda.
      Serial.print(F(" Ip Sec:            "));  Serial.println(ipUs * 0.000001f, 4);  // İnfleksiyon zamanı yazdır.
      Serial.print(F(" Ip Slope:          "));  Serial.print(slopeIp, 3);  // İnfleksiyon eğimi yazdır.
      if (_action == directIP || _action == direct5T) Serial.println(F(" ↑"));  // Yön oku yazdır.
      else  Serial.println(F(" ↓"));  // Ters yön oku yazdır.
      Serial.print(F(" Ip Pv:             "));  Serial.println(pvIp, 3);  // İnfleksiyon PV yazdır.
    }  // IP debug bloğu sonu.
    Serial.print(F(" Pv Start:          "));  Serial.println(pvStart, 3);  // Başlangıç PV yazdır.
    if (_action == directIP || _action == direct5T) Serial.print(F(" Pv Max:            "));  // Doğrudan yön için maksimum başlığı.
    else Serial.print(F(" Pv Min:            "));  // Ters yön için minimum başlığı.
    Serial.println(pvMax, 3);  // PV maksimum/minimum yazdır.
    Serial.print(F(" Pv Diff:           "));  Serial.println(pvMax - pvStart, 3);  // PV farkını yazdır.
    Serial.println();  // Boş satır yazdır.
    Serial.print(F(" Process Gain:      "));  Serial.println(_Ku, 3);  // Proses kazancını yazdır.
    Serial.print(F(" Dead Time Sec:     "));  Serial.println(_td, 3);  // Ölü zamanı yazdır.
    Serial.print(F(" Tau Sec:           "));  Serial.println(_Tu, 3);  // Tau değerini yazdır.
    Serial.println();  // Boş satır yazdır.

    // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png  // Kontrol edilebilirlik açıklaması.
    float controllability = _Tu / _td + epsilon;  // Kontrol edilebilirlik hesapla.
    if (controllability > 99.9) controllability = 99.9;  // Üst sınırla.
    Serial.print(F(" Tau/Dead Time:     "));  Serial.print(controllability, 1);  // Oranı yazdır.
    if (controllability > 0.75) Serial.println(F(" (easy to control)"));  // Kolay kontrol uyarısı.
    else if (controllability > 0.25) Serial.println(F(" (average controllability)"));  // Orta kontrol uyarısı.
    else Serial.println(F(" (difficult to control)"));  // Zor kontrol uyarısı.

    // check “best practice” rule that sample time should be ≥ 10 times per process time constant  // Örnekleme kuralı açıklaması.
    // https://controlguru.com/sample-time-is-a-fundamental-design-and-tuning-specification/  // Açıklama bağlantısı.
    float sampleTimeCheck = _Tu / (_samplePeriodUs * 0.000001f);  // Örnekleme oranını hesapla.
    Serial.print(F(" Tau/Sample Period: "));  Serial.print(sampleTimeCheck, 1);  // Oranı yazdır.
    if (sampleTimeCheck >= 10) Serial.println(F(" (good sample rate)"));  // İyi örnekleme uyarısı.
    else Serial.println(F(" (low sample rate)"));  // Düşük örnekleme uyarısı.
    Serial.println();  // Boş satır yazdır.
    EkimPid::printTunings();  // Ayarları yazdır.
    sampleCount++;  // Örnek sayacını artır.
  }  // Yazdırma modu bloğu sonu.
}  // printResults sonu.

// Query functions  // Sorgu fonksiyonları.

void EkimPid::GetAutoTunings(float * kp, float * ki, float * kd) {  // Otomatik ayarları döndürür.
  *kp = _kp;  // Kp çıkışına yaz.
  *ki = _ki;  // Ki çıkışına yaz.
  *kd = _kd;  // Kd çıkışına yaz.
}  // GetAutoTunings sonu.

// https://blog.opticontrols.com/archives/477  // Ayar formülleri referansı.

float EkimPid::GetKp() {  // Kp hesaplama fonksiyonu.
  float znPid = ((1.2f * _Tu) / (_Ku * _td)) / 2;  // ZN PID Kp hesabı.
  float doPid = (0.66f * _Tu) / (_Ku * _td);  // Damped PID Kp hesabı.
  float noPid = (0.6f / _Ku) * (_Tu / _td);  // Aşmasız PID Kp hesabı.
  float ccPid = _Ku * (1.33f + (_R / 4.0f));  // Cohen-Coon PID Kp hesabı.
  float znPi = ((0.9f * _Tu) / (_Ku * _td)) / 2;  // ZN PI Kp hesabı.
  float doPi = (0.495f * _Tu) / (_Ku * _td);  // Damped PI Kp hesabı.
  float noPi = (0.35f / _Ku) * (_Tu / _td);  // Aşmasız PI Kp hesabı.
  float ccPi = _Ku * (0.9f + (_R / 12.0f));  // Cohen-Coon PI Kp hesabı.
  if (_tuningMethod == ZN_PID)                _kp = znPid;  // ZN PID seçimi.
  else if (_tuningMethod == DampedOsc_PID)    _kp = doPid;  // Damped PID seçimi.
  else if (_tuningMethod == NoOvershoot_PID)  _kp = noPid;  // Aşmasız PID seçimi.
  else if (_tuningMethod == CohenCoon_PID)    _kp = ccPid;  // Cohen-Coon PID seçimi.
  else if (_tuningMethod == Mixed_PID)        _kp = 0.25f * (znPid + doPid + noPid + ccPid);  // Karışık PID seçimi.
  else if (_tuningMethod == ZN_PI)            _kp = znPi;  // ZN PI seçimi.
  else if (_tuningMethod == DampedOsc_PI)     _kp = doPi;  // Damped PI seçimi.
  else if (_tuningMethod == NoOvershoot_PI)   _kp = noPi;  // Aşmasız PI seçimi.
  else if (_tuningMethod == CohenCoon_PI)     _kp = ccPi;  // Cohen-Coon PI seçimi.
  else                                        _kp = 0.25f * (znPi + doPi + noPi + ccPi); // Mixed_PI  // Karışık PI seçimi.
  return _kp;  // Kp değerini döndür.
}  // GetKp sonu.

float EkimPid::GetKi() {  // Ki hesaplama fonksiyonu.
  float znPid = 1 / (2.0f * _td);  // ZN PID Ki hesabı.
  float doPid = 1 / (_Tu / 3.6f);  // Damped PID Ki hesabı.
  float noPid = 1 / (_Tu);  // Aşmasız PID Ki hesabı.
  float ccPid = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));  // Cohen-Coon PID Ki hesabı.
  float znPi = 1 / (3.3333f * _td);  // ZN PI Ki hesabı.
  float doPi = 1 / (_Tu / 2.6f);  // Damped PI Ki hesabı.
  float noPi = 1 / (1.2f * _Tu);  // Aşmasız PI Ki hesabı.
  float ccPi = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));  // Cohen-Coon PI Ki hesabı.

  if (_tuningMethod == ZN_PID)                _ki = znPid;  // ZN PID seçimi.
  else if (_tuningMethod == DampedOsc_PID)    _ki = doPid;  // Damped PID seçimi.
  else if (_tuningMethod == NoOvershoot_PID)  _ki = noPid;  // Aşmasız PID seçimi.
  else if (_tuningMethod == CohenCoon_PID)    _ki = ccPid;  // Cohen-Coon PID seçimi.
  else if (_tuningMethod == Mixed_PID)        _ki = 0.25f * (znPid + doPid + noPid + ccPid);  // Karışık PID seçimi.
  else if (_tuningMethod == ZN_PI)            _ki = znPi;  // ZN PI seçimi.
  else if (_tuningMethod == DampedOsc_PI)     _ki = doPi;  // Damped PI seçimi.
  else if (_tuningMethod == NoOvershoot_PI)   _ki = noPi;  // Aşmasız PI seçimi.
  else if (_tuningMethod == CohenCoon_PI)     _ki = ccPi;  // Cohen-Coon PI seçimi.
  else                                        _ki = 0.25f * (znPi + doPi + noPi + ccPi); // Mixed_PI  // Karışık PI seçimi.
  return _ki;  // Ki değerini döndür.
}  // GetKi sonu.

float EkimPid::GetKd() {  // Kd hesaplama fonksiyonu.
  float znPid = 1 / (0.5f * _td);  // ZN PID Kd hesabı.
  float doPid = 1 / (_Tu / 9.0f);  // Damped PID Kd hesabı.
  float noPid = 1 / (0.5f * _td);  // Aşmasız PID Kd hesabı.
  float ccPid = 1 / ((4.0f * _td) / (11.0f + (2.0f * _R)));  // Cohen-Coon PID Kd hesabı.
  if (_tuningMethod == ZN_PID)                _kd = znPid;  // ZN PID seçimi.
  else if (_tuningMethod == DampedOsc_PID)    _kd = doPid;  // Damped PID seçimi.
  else if (_tuningMethod == NoOvershoot_PID)  _kd = noPid;  // Aşmasız PID seçimi.
  else if (_tuningMethod == CohenCoon_PID)    _kd = ccPid;  // Cohen-Coon PID seçimi.
  else if (_tuningMethod == Mixed_PID)        _kd = 0.25f * (znPid + doPid + noPid + ccPid);  // Karışık PID seçimi.
  else                                        _kd = 0.0f; // PI controller  // PI seçildiyse Kd sıfır.
  return _kd;  // Kd değerini döndür.
}  // GetKd sonu.

float EkimPid::GetTi() {  // Ti hesaplama fonksiyonu.
  return _kp / _ki;  // Ti = Kp / Ki.
}  // GetTi sonu.

float EkimPid::GetTd() {  // Td hesaplama fonksiyonu.
  if (_tuningMethod == ZN_PID ||  // PID yöntemlerinden biri mi?
      _tuningMethod == DampedOsc_PID ||  // Damped PID kontrolü.
      _tuningMethod == NoOvershoot_PID ||  // Aşmasız PID kontrolü.
      _tuningMethod == CohenCoon_PID ||  // Cohen-Coon PID kontrolü.
      _tuningMethod == Mixed_PID) {  // Karışık PID kontrolü.
    return _kp / _kd;  // Td = Kp / Kd.
  }  // PID koşulu sonu.
  else return 0;  // PI ise Td sıfır.
}  // GetTd sonu.

float EkimPid::GetProcessGain() {  // Proses kazancını döndürür.
  return _Ku;  // Ku değerini döndür.
}  // GetProcessGain sonu.

float EkimPid::GetDeadTime() {  // Ölü zamanı döndürür.
  return _td;  // td değerini döndür.
}  // GetDeadTime sonu.

float EkimPid::GetTau() {  // Zaman sabitini döndürür.
  return _Tu;  // Tu değerini döndür.
}  // GetTau sonu.

uint8_t EkimPid::GetControllerAction() {  // Kontrol yönü döndürür.
  return static_cast<uint8_t>(_action);  // Action enumunu sayıya çevir.
}  // GetControllerAction sonu.

uint8_t EkimPid::GetSerialMode() {  // Seri modunu döndürür.
  return static_cast<uint8_t>(_serialMode);  // SerialMode enumunu sayıya çevir.
}  // GetSerialMode sonu.

uint8_t EkimPid::GetTuningMethod() {  // Ayar yöntemini döndürür.
  return static_cast<uint8_t>(_tuningMethod);  // TuningMethod enumunu sayıya çevir.
}  // GetTuningMethod sonu.

float EkimPid::softPwm(const uint8_t relayPin, float input, float output, float setpoint, uint32_t windowSize, uint8_t debounce) {  // Yazılımsal PWM fonksiyonu.

  // software PWM timer  // Yazılımsal PWM zamanlayıcısı.
  uint32_t msNow = millis();  // Şu anki zamanı al.
  static uint32_t  windowStartTime, nextSwitchTime;  // Pencere ve bir sonraki anahtarlama zamanı.
  if (msNow - windowStartTime >= windowSize) {  // PWM penceresi doldu mu?
    windowStartTime = msNow;  // Pencere başlangıcını güncelle.
  }  // Pencere güncelleme sonu.
  // SSR optimum AC half-cycle controller  // SSR için optimum yarım periyot kontrolü.
  static float optimumOutput;  // Optimum çıkış değeri.
  static bool reachedSetpoint;  // Setpoint'e ulaşıldı mı bayrağı.

  if (input > setpoint) reachedSetpoint = true;  // Setpoint aşıldıysa bayrakla.
  if (reachedSetpoint && !debounce && setpoint > 0 && input > setpoint) optimumOutput = output - 8;  // Aşımdan sonra çıkışı azalt.
  else if (reachedSetpoint && !debounce && setpoint > 0 && input < setpoint) optimumOutput = output + 8;  // Düşüşte çıkışı artır.
  else  optimumOutput = output;  // Aksi halde çıkışı olduğu gibi kullan.
  if (optimumOutput < 0) optimumOutput = 0;  // Negatif çıkışı sıfıra sınırla.

  // PWM relay output  // PWM röle çıkışı.
  static bool relayStatus;  // Röle durumu.
  if (!relayStatus && optimumOutput > (msNow - windowStartTime)) {  // Röle kapalıysa ve açma süresi geldiyse.
    if (msNow > nextSwitchTime) {  // Debounce süresi geçti mi?
      nextSwitchTime = msNow + debounce;  // Bir sonraki anahtarlama zamanını ayarla.
      relayStatus = true;  // Röleyi açık yap.
      digitalWrite(relayPin, HIGH);  // Röleyi HIGH seviyesine çek.
    }  // Debounce kontrolü sonu.
  } else if (relayStatus && optimumOutput < (msNow - windowStartTime)) {  // Röle açıksa ve kapama süresi geldiyse.
    if (msNow > nextSwitchTime) {  // Debounce süresi geçti mi?
      nextSwitchTime = msNow + debounce;  // Bir sonraki anahtarlama zamanını ayarla.
      relayStatus = false;  // Röleyi kapalı yap.
      digitalWrite(relayPin, LOW);  // Röleyi LOW seviyesine çek.
    }  // Debounce kontrolü sonu.
  }  // Röle kontrol bloğu sonu.
  return optimumOutput;  // Kullanılan PWM değerini döndür.
}  // softPwm sonu.
