// Receiver (node B) for Transmitter.ino (node A)
// ESP32 + Ra-01H (SX1276)  або  ESP32 + Ra-01SH (SX1262, комплект організаторів)
//
// Що робить:
//  - приймає 32-байтні пакети передавача (type=1)
//  - логує RSSI/SNR ПРЯМОГО каналу і рахує PDR на боці приймача
//  - у режимі MODE=1 (вказаний у самому пакеті) відповідає ACK (type=2)
//  - пише CSV у Serial і у флеш (/rx.csv), команди dump / erase YES як у передавача
//
// ВАЖЛИВО: пресет приймача треба виставити ТАКИЙ САМИЙ, як на передавачі
// (команда "preset N"), бо в протоколі немає службового каналу.

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

// ================== ВИБІР МОДУЛЯ (розкоментуйте ОДИН) ==================
#define MODULE_RA01H     // Ra-01H, SX1276 (ваша пайка)
// #define MODULE_RA01SH // Ra-01SH, SX1262 (комплект організаторів)

#if defined(MODULE_RA01H)
  // Схема з фото: SCK18 MISO19 MOSI23 NSS27 RST14 DIO0=26
  constexpr int PIN_SCK=18,PIN_MISO=19,PIN_MOSI=23,PIN_NSS=27,PIN_RST=14,PIN_IRQ=26;
  Module mod(PIN_NSS,PIN_IRQ,PIN_RST,RADIOLIB_NC);
  SX1276 radio(&mod);
#elif defined(MODULE_RA01SH)
  // !!! ВПИШІТЬ піни shield-а організаторів (IRQ = DIO1, плюс BUSY) !!!
  constexpr int PIN_SCK=18,PIN_MISO=19,PIN_MOSI=23,PIN_NSS=5,PIN_RST=14,PIN_IRQ=26,PIN_BUSY=27;
  // Якщо init дає помилку -706/-707, поставте TCXO_V = 0.0 (модуль без TCXO), або навпаки 1.8
  constexpr float TCXO_V=0.0;
  Module mod(PIN_NSS,PIN_IRQ,PIN_RST,PIN_BUSY);
  SX1262 radio(&mod);
#else
  #error "Select MODULE_RA01H or MODULE_RA01SH"
#endif

// ================== ПАРАМЕТРИ (мають збігатися з передавачем) ==================
constexpr float FREQ_MHZ=868.1;
constexpr uint8_t SYNC_WORD=0x12;
constexpr uint16_t PREAMBLE=8;
constexpr size_t PACKET_BYTES=32;
constexpr uint32_t MAX_PACKETS=10000;
constexpr uint32_t ACK_DELAY_MS=200;   // пауза, щоб передавач встиг перейти в прийом

struct Profile{uint8_t sf;uint32_t bw;uint8_t cr;};
constexpr Profile profiles[]={{7,125000,5},{9,125000,5},{12,125000,5},{7,250000,5},{7,125000,8}};
constexpr int NPROF=sizeof(profiles)/sizeof(profiles[0]);

uint8_t preset=1;
int8_t ackPower=14;

// ================== СТАН ==================
Preferences prefs;
bool radioOK=false,fsOK=false;
volatile bool rxFlag=false;
uint32_t bootId=0;

// статистика поточного прогону (run)
uint32_t curRun=0;bool haveRun=false;
uint16_t curPoint=0;uint8_t curMode=0;uint32_t curCount=0;
uint32_t rxUnique=0,rxDup=0,crcErr=0,foreign=0,maxSeq=0,acksSent=0;
double sumRssi=0,sumSnr=0;
uint8_t seen[(MAX_PACKETS+7)/8];

String inputLine;

void IRAM_ATTR onRx(){rxFlag=true;}

inline uint16_t get16(const uint8_t*p){return uint16_t(p[0])|uint16_t(p[1])<<8;}
inline uint32_t get32(const uint8_t*p){uint32_t n=0;for(int i=0;i<4;i++)n|=uint32_t(p[i])<<(8*i);return n;}
inline void put16(uint8_t*p,uint16_t n){p[0]=n;p[1]=n>>8;}

const Profile&profile(){return profiles[preset-1];}

double toaMs(const Profile&p,unsigned bytes=PACKET_BYTES){
  const double ts=1000.0*(1UL<<p.sf)/p.bw;
  const int de=ts>16.0?1:0;
  const double n=ceil((8.0*bytes-4*p.sf+28+16)/(4*(p.sf-2*de)));
  return(PREAMBLE+4.25+8+fmax(n*p.cr,0.0))*ts;
}

// Пакет даних від передавача: та сама перевірка, що в Transmitter.ino, але без знання run заздалегідь
bool validData(const uint8_t*b){
  if(b[0]!='L'||b[1]!='B'||b[2]!=1||b[3]!=1||b[21]!=0)return false;
  const uint32_t seq=get32(b+8),count=get32(b+16);
  if(count<1||count>MAX_PACKETS||seq<1||seq>count||b[15]>1||b[14]<1||b[14]>NPROF)return false;
  for(size_t i=26;i<PACKET_BYTES;i++)if(b[i]!=uint8_t(0xa0+i))return false;
  return true;
}

// ================== CSV ==================
const char LOG_PATH[]="/rx.csv";
const char CSV_HEADER[]="schema,node,boot_id,event,run_id,point_id,preset,mode,seq,t_ms,freq_hz,sf,bw_hz,cr_den,tx_power_dbm,packet_bytes,planned_count,toa_ms,rssi_dbm,snr_db,rx_unique,pdr_pct,acks_sent";

double pdrNow(){
  // PDR приймача: унікальні прийняті / найбільший номер, який мав би прийти
  return maxSeq?100.0*rxUnique/maxSeq:0.0;
}

void logRow(const char*ev,uint32_t seq=0,int txPower=-1,double rssi=NAN,double snr=NAN){
  String r;r.reserve(300);
  r="1,B,";r+=String(bootId);r+=',';r+=ev;r+=',';
  r+=String(curRun);r+=',';r+=String(curPoint);r+=',';r+=String(preset);r+=',';
  r+=String(curMode);r+=',';r+=String(seq);r+=',';r+=String(millis());r+=',';
  r+=String((uint32_t)lround(FREQ_MHZ*1000.0)*1000UL);r+=',';
  r+=String(profile().sf);r+=',';r+=String(profile().bw);r+=',';r+=String(profile().cr);r+=',';
  if(txPower>=0)r+=String(txPower);r+=',';
  r+=String(PACKET_BYTES);r+=',';r+=String(curCount);r+=',';
  r+=String(toaMs(profile()),3);r+=',';
  if(!isnan(rssi))r+=String(rssi,2);r+=',';
  if(!isnan(snr))r+=String(snr,2);r+=',';
  r+=String(rxUnique);r+=',';r+=String(pdrNow(),2);r+=',';r+=String(acksSent);
  Serial.println(r);
  if(fsOK&&LittleFS.totalBytes()-LittleFS.usedBytes()>16384){
    File f=LittleFS.open(LOG_PATH,FILE_APPEND);
    if(f){if(f.size()==0)f.println(CSV_HEADER);f.println(r);f.close();}
  }
}

// ================== РАДІО ==================
bool applyRadio(){
  int16_t e=0,st;
  radio.standby();
  if((st=radio.setFrequency(FREQ_MHZ))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setSpreadingFactor(profile().sf))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setBandwidth(profile().bw/1000.0f))!=RADIOLIB_ERR_NONE)e=st; // кГц
  if((st=radio.setCodingRate(profile().cr))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setPreambleLength(PREAMBLE))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setSyncWord(SYNC_WORD))!=RADIOLIB_ERR_NONE)e=st;
#if defined(MODULE_RA01H)
  if((st=radio.setCRC(true))!=RADIOLIB_ERR_NONE)e=st;
#else
  if((st=radio.setCRC(2))!=RADIOLIB_ERR_NONE)e=st;
#endif
  if((st=radio.setOutputPower(ackPower))!=RADIOLIB_ERR_NONE)e=st;
  if(e){Serial.printf("#ERROR radio config, code %d\n",e);return false;}
  rxFlag=false;
  radio.startReceive();
  return true;
}

void resetStats(){
  rxUnique=rxDup=crcErr=foreign=maxSeq=acksSent=0;sumRssi=sumSnr=0;
  memset(seen,0,sizeof(seen));
}

void summary(){
  Serial.printf("# SUMMARY run=%lu point=%u preset=%u SF=%u BW=%lu CR=4/%u planned=%lu rx_unique=%lu max_seq=%lu PDR=%.2f%% (vs planned %.2f%%) avgRSSI=%.1f avgSNR=%.1f dup=%lu crc_err=%lu acks=%lu\n",
    (unsigned long)curRun,curPoint,preset,profile().sf,(unsigned long)profile().bw,profile().cr,
    (unsigned long)curCount,(unsigned long)rxUnique,(unsigned long)maxSeq,pdrNow(),
    curCount?100.0*rxUnique/curCount:0.0,
    rxUnique?sumRssi/rxUnique:NAN,rxUnique?sumSnr/rxUnique:NAN,
    (unsigned long)rxDup,(unsigned long)crcErr,(unsigned long)acksSent);
}

void handlePacket(){
  const size_t n=radio.getPacketLength();
  uint8_t b[PACKET_BYTES];
  const int16_t st=radio.readData(b,PACKET_BYTES);   // не більше 32 байт
  const double rssi=radio.getRSSI(),snr=radio.getSNR();

  if(st==RADIOLIB_ERR_CRC_MISMATCH){crcErr++;radio.startReceive();return;}
  if(st!=RADIOLIB_ERR_NONE||n!=PACKET_BYTES||!validData(b)){foreign++;radio.startReceive();return;}

  if(b[14]!=preset){
    Serial.printf("#WARN packet preset=%u but receiver preset=%u\n",b[14],preset);
  }

  const uint32_t run=get32(b+4),seq=get32(b+8);
  // новий прогін → обнуляємо статистику
  if(!haveRun||run!=curRun){
    if(haveRun)summary();
    resetStats();haveRun=true;curRun=run;
    curPoint=get16(b+12);curMode=b[15];curCount=get32(b+16);
    logRow("RUN_START",0,b[20]);
  }

  bool dup=(seen[(seq-1)>>3]>>((seq-1)&7))&1;
  if(dup)rxDup++;
  else{
    seen[(seq-1)>>3]|=1<<((seq-1)&7);
    rxUnique++;sumRssi+=rssi;sumSnr+=snr;
    if(seq>maxSeq)maxSeq=seq;
  }
  logRow(dup?"RX_DUP":"RX",seq,b[20],rssi,snr);

  // ACK тільки в режимі 1: копія пакета, type=2, плюс RSSI і SNR прямого каналу
  if(b[15]==1){
    b[3]=2;
    put16(b+22,uint16_t(int16_t(lround(rssi))));
    put16(b+24,uint16_t(int16_t(lround(snr*4))));
    delay(ACK_DELAY_MS);
    if(radio.transmit(b,PACKET_BYTES)==RADIOLIB_ERR_NONE){acksSent++;logRow("ACK_TX",seq);}
    else logRow("ACK_FAIL",seq);
  }
  if(seq==curCount)summary();
  rxFlag=false;
  radio.startReceive();
}

// ================== КОМАНДИ ==================
void status(){
  Serial.printf("# node=B radio=%s fs=%s preset=%u SF=%u BW=%lu CR=4/%u ack_power=%d ToA_ms=%.1f\n",
    radioOK?"OK":"FAIL",fsOK?"OK":"FAIL",preset,profile().sf,(unsigned long)profile().bw,profile().cr,ackPower,toaMs(profile()));
}
void help(){
  Serial.println("# preset N (1..5) | power DBM (2..17) | status | summary | reset | table | dump | erase YES | format YES");
}

void command(String line){
  line.trim();if(!line.length())return;
  if(line=="status"){status();return;}
  if(line=="help"||line=="?"){help();return;}
  if(line=="summary"){summary();return;}
  if(line=="reset"){resetStats();haveRun=false;Serial.println("# stats reset");return;}
  if(line=="table"){
    for(int i=0;i<NPROF;i++)Serial.printf("# preset=%d SF=%u BW=%lu CR=4/%u ToA32_ms=%.1f\n",
      i+1,profiles[i].sf,(unsigned long)profiles[i].bw,profiles[i].cr,toaMs(profiles[i]));
    return;
  }
  if(line.startsWith("preset ")){
    int p=line.substring(7).toInt();
    if(p<1||p>NPROF){Serial.println("#ERROR preset 1..5");return;}
    preset=p;prefs.putUChar("preset",preset);
    if(radioOK)radioOK=applyRadio();
    status();return;
  }
  if(line.startsWith("power ")){
    int p=line.substring(6).toInt();
    if(p<2||p>17){Serial.println("#ERROR power 2..17");return;}
    ackPower=p;prefs.putChar("power",ackPower);
    if(radioOK)radioOK=applyRadio();
    status();return;
  }
  if(line=="dump"){
    if(!fsOK){Serial.println("#ERROR no filesystem");return;}
    File f=LittleFS.open(LOG_PATH,FILE_READ);
    Serial.println("#DUMP_BEGIN");
    if(f){uint8_t buf[256];while(f.available()){size_t k=f.read(buf,sizeof(buf));Serial.write(buf,k);delay(1);}f.close();}
    else Serial.println(CSV_HEADER);
    Serial.println("#DUMP_END");return;
  }
  if(line=="erase YES"){
    bool ok=fsOK&&(!LittleFS.exists(LOG_PATH)||LittleFS.remove(LOG_PATH));
    Serial.println(ok?"# journal erased":"#ERROR erase");return;
  }
  if(line=="format YES"){
    LittleFS.end();fsOK=LittleFS.format()&&LittleFS.begin(false);
    Serial.println(fsOK?"# filesystem formatted":"#ERROR format");return;
  }
  Serial.println("#ERROR unknown command; help");
}

void serviceSerial(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\n'){command(inputLine);inputLine="";}
    else if(ch!='\r'&&inputLine.length()<100)inputLine+=ch;
  }
}

// ================== SETUP / LOOP ==================
void setup(){
  Serial.begin(115200);delay(400);bootId=esp_random();
  prefs.begin("labrxv1",false);
  preset=prefs.getUChar("preset",1);if(preset<1||preset>NPROF)preset=1;
  ackPower=prefs.getChar("power",14);
  fsOK=LittleFS.begin(false);
  SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI,PIN_NSS);

#if defined(MODULE_RA01H)
  int16_t st=radio.begin(FREQ_MHZ,125.0,7,5,SYNC_WORD,ackPower,PREAMBLE);
#else
  int16_t st=radio.begin(FREQ_MHZ,125.0,7,5,SYNC_WORD,ackPower,PREAMBLE,TCXO_V);
#endif
  if(st!=RADIOLIB_ERR_NONE){Serial.printf("#ERROR radio init, code %d\n",st);help();return;}
  radio.setPacketReceivedAction(onRx);
  radioOK=applyRadio();
  Serial.println(CSV_HEADER);help();status();
  if(!fsOK)Serial.println("# First use only: format YES");
}

void loop(){
  serviceSerial();
  if(radioOK&&rxFlag){rxFlag=false;handlePacket();}
  delay(1);
}
