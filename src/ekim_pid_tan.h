#pragma once  // Bu başlığın yalnızca bir kez dahil edilmesini sağlar.
#ifndef EKIM_PID_TAN_H_  // Çoklu dahil etmeyi önlemek için koruyucu makro başı.
#define EKIM_PID_TAN_H_  // Koruyucu makro tanımı.

class EkimPidTan  // Kaydırmalı teğet hattı için dairesel tampon sınıfı.
{  // Sınıf gövdesi başı.
  public:  // Dışarıdan erişilebilen üyeler.

    EkimPidTan();  // Varsayılan yapıcı bildirim.
    EkimPidTan(uint16_t bufferSize) : bufSize(bufferSize) {}  // Tampon boyutu alan yapıcı.
    ~EkimPidTan() {};  // Yıkıcı; özel işlem yok.

    void begin(uint16_t bufferSize);  // Tamponu başlatır.
    void init(float reading);  // Tamponu tek değerle doldurur.
    float avgVal(float reading);  // Yeni değeri ekleyip ortalama döndürür.
    float startVal();  // Tampondaki en eski değeri döndürür.
    float slope(float reading);  // Yeni değere göre eğimi hesaplar.
    uint16_t length();  // Tampon uzunluğunu döndürür.

  private:  // Sadece sınıf içinde erişilebilen üyeler.
    uint16_t bufSize;   // Tampon boyutu.
    uint16_t index;     // Geçerli indeks konumu.
    float sum;          // Tampondaki değerlerin toplamı.
    float *inputArray;  // Tampon dizisine işaretçi.
};  // Sınıf gövdesi sonu.
#endif  // EKIM_PID_TAN_H_ koruyucu makro sonu.
