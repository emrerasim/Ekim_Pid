#include "Arduino.h"  // Arduino temel fonksiyonları için başlık.
#include "ekim_pid_tan.h"  // EkimPidTan sınıfı bildirimi.

void EkimPidTan::begin(uint16_t bufferSize) {  // Tamponu başlatan fonksiyon.
  inputArray = new float[bufferSize];  // Tampon için dinamik bellek ayır.
  bufSize = bufferSize;  // Tampon boyutunu sakla.
  EkimPidTan::init(0);  // Tamponu sıfır değeriyle başlat.
}  // begin fonksiyonu sonu.

void EkimPidTan::init(float reading) {  // Tamponu belirli bir değerle doldurur.
  index = 0;  // Başlangıç indeksi sıfırla.
  sum = reading * bufSize;  // Toplamı tampon boyutu kadar ayarla.
  for (uint16_t i = 0; i < bufSize; i++) {  // Tüm tampon elemanlarını dolaş.
    inputArray[i] = reading;  // Her elemana aynı okuma değerini yaz.
  }  // Döngü sonu.
}  // init fonksiyonu sonu.

float EkimPidTan::avgVal(float reading) {  // Yeni değer ekleyip ortalama döndürür.
  index++;  // İndeksi bir sonraki konuma taşı.
  if (index >= bufSize) index = 0;  // Dairesel tampon için başa sar.
  sum += reading - inputArray[index];  // Toplamdan eskiyi çıkarıp yeniyi ekle.
  inputArray[index] = reading;  // Yeni okumayı tampona yaz.
  return float (sum / bufSize);  // Güncel ortalamayı döndür.
}  // avgVal fonksiyonu sonu.

float EkimPidTan::startVal() {  // Tampondaki en eski değeri döndürür.
  uint16_t tailIndex = index + 1;  // En eski değerin bulunduğu indeks.
  if (tailIndex >= bufSize) tailIndex = 0;  // Dairesel tampon için başa sar.
  return inputArray[tailIndex];  // En eski değeri döndür.
}  // startVal fonksiyonu sonu.

float EkimPidTan::slope(float reading) {  // Eğimi hesaplayan fonksiyon.
  return reading - EkimPidTan::startVal();  // Yeni değer ile başlangıç değeri farkı.
}  // slope fonksiyonu sonu.

uint16_t EkimPidTan::length() {  // Tampon boyutunu döndürür.
  return bufSize;  // Kaydedilen boyutu döndür.
}  // length fonksiyonu sonu.
