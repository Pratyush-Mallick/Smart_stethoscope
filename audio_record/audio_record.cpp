
#include <Seeed_FS.h>
#include "SD_Handle.h"

#define SERIAL Serial

#ifdef USESPIFLASH
#define DEV SPIFLASH
#include "SFUD/Seeed_SFUD.h"
#else
#define DEV SD
#include "SD/Seeed_SD.h"
#endif 

#ifdef _SAMD21_
#define SDCARD_SS_PIN 1
#define SDCARD_SPI SPI
#endif 

#define SAMPLING_FREQUENCY 16000
#define SAMPLES 16000*5

int16_t recording[16000*5];
unsigned int sampling_period_us;
uint8_t sample_count = 0;

void create_template(File *sFile)
{
  
struct soundhdr {
  char  riff[4];        /* "RIFF"                                  */
  long  flength;        /* file length in bytes                    */
  char  wave[4];        /* "WAVE"                                  */
  char  fmt[4];         /* "fmt "                                  */
  long  chunk_size;     /* size of FMT chunk in bytes (usually 16) */
  short format_tag;     /* 1=PCM, 257=Mu-Law, 258=A-Law, 259=ADPCM */
  short num_chans;      /* 1=mono, 2=stereo                        */
  long  srate;          /* Sampling rate in samples per second     */
  long  bytes_per_sec;  /* bytes per second = srate*bytes_per_samp */
  short bytes_per_samp; /* 2=16-bit mono, 4=16-bit stereo          */
  short bits_per_samp;  /* Number of bits per sample               */
  char  data[4];        /* "data"                                  */
  long  dlength;        /* data length in bytes (filelength - 44)  */
} wavh;

// It's easy enough to initialize the strings
strncpy(wavh.riff,"RIFF", 4);
strncpy(wavh.wave,"WAVE", 4);
strncpy(wavh.fmt,"fmt ", 4);
strncpy(wavh.data,"data", 4);

// size of FMT chunk in bytes
wavh.chunk_size = 16;
wavh.format_tag = 1; // PCM
wavh.num_chans = 1; // mono
wavh.srate = 16000;
wavh.bytes_per_sec = (16000 * 1 * 16 * 1)/8;
wavh.bytes_per_samp = 2;
wavh.bits_per_samp = 16;
wavh.dlength = 16000 * 2 *  1 * 16/2;

sFile->seek(0);
sFile->write((byte *)&wavh, 44);
}

void finalize_template(File *sFile)
{
  unsigned long fSize = sFile->size()-8;
  sFile->seek(4); 
  byte data[4] = {lowByte(fSize), highByte(fSize), fSize >> 16, fSize >> 24};
  sFile->write(data,4);
  byte tmp;
  sFile->seek(40);
  fSize = fSize - 36;
  data[0] = lowByte(fSize);
  data[1]= highByte(fSize);
  data[2]= fSize >> 16;
  data[3]= fSize >> 24;
  sFile->write((byte*)data, 4);
  sFile->close();
}


void setup() {
    analogReadResolution(16);
    pinMode(BCM27, INPUT_PULLDOWN);
    analogReference(AR_INTERNAL2V23);
    pinMode(WIO_KEY_A, INPUT_PULLUP);
    SERIAL.begin(115200);
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);

    while (!SERIAL) {};
    #ifdef SFUD_USING_QSPI
    while (!DEV.begin(104000000UL)) {
        SERIAL.println("Card Mount Failed");
        return;
    }
    #else
    while (!DEV.begin(SDCARD_SS_PIN,SDCARD_SPI,4000000UL)) {
        SERIAL.println("Card Mount Failed");
        return;
    }
    #endif 
    SERIAL.println("initialization done.");

    sampling_period_us = round(600000 * (1.0 / SAMPLING_FREQUENCY));
}

void loop() {
    unsigned long StopTime,StartTime,ElapsedTime = 0;
    if (digitalRead(WIO_KEY_A) == LOW) {
    static char print_buf[128] = {0};
    int r = sprintf(print_buf, "test_%d.wav",millis());
    // initialize to ope function, ony declaring won't help
    File sFile = DEV.open(print_buf, FILE_WRITE); //Writing Mode
    create_template(&sFile);

    sprintf(print_buf,"Starting sample %d ",sample_count++);
    Serial.println(print_buf);

    StartTime = millis();
    for (int i = 0; i < SAMPLES; i++) {
    recording[i] = map(analogRead(BCM27), 0, 65536, -32768, 32767);
    delayMicroseconds(sampling_period_us);
    }
    StopTime = millis();
    ElapsedTime = StopTime - StartTime;
    sprintf(print_buf,"Finished sample with time taken = %lu ms", ElapsedTime);
    Serial.println(print_buf);
    for (int i = 0; i < SAMPLES; i++) {

    sFile.write(recording[i] & 0xFF);
    sFile.write((recording[i] >> 8) & 0xFF);
    }
    Serial.println("Finished writing");
    finalize_template(&sFile);
  }
}