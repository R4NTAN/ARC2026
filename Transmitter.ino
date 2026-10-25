#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

// PIN MAP
constexpr int PIN_SCK=18,PIN_MISO=19,PIN_MOSI=23,PIN_NSS=27,PIN_RST=14,PIN_DIO0=26;

// RF VAL CONF
constexpr float FREQ_HZ=868.1;
constexpr uint8_t SYNC_WORD=0x12;
constexpr uint16_t PREAMBLE=8;
constexpr size_t PACKET_BYTES=32;
constexpr uint32_t MAX_PACKETS=10000;
constexpr uint32_t ACK_DELAY_MS=200;
constexpr double DUTY_BUDGET=0.01;

struct Profile{uint8_t sf;uint32_t bw;uint8_t cr;};
constexpr Profile profiles[]={{7,125000,5},{9,125000,5},{12,125000,5},{7,250000,5},{7,125000,8}};

struct Config{
  uint32_t run=1,count=100,interval=200000;
  uint16_t point=1;
  uint8_t preset=1,mode=0;
  int8_t power=10;
};

// RADIOLIB OBJ
Module mod(PIN_NSS,PIN_DIO0,PIN_RST,RADIOLIB_NC);
SX1276 radio(&mod); // Ra-01H = SX1276 (868 MHz). SX1278 class rejects >525 MHz

// STATS
Config cfg;
Preferences prefs;
bool fsOK=false,radioOK=false,prefsOK=false,active=false,fault=false;
bool onAir=false,waitingAck=false,ackPending=false;
uint32_t bootId=0,seq=0,nextTx=0,txStart=0,requestStart=0,ackDeadline=0,ackDue=0;
uint32_t ackCount=0; // ACK+PDR
uint8_t txBuffer[PACKET_BYTES];
uint8_t seen[(MAX_PACKETS+7)/8]={};
uint32_t rxUnique=0;
String inputLine;
bool inputOverflow=false;

// PROTOCOL HELP
const Profile&profile(){return profiles[cfg.preset-1];}
bool due(uint32_t now,uint32_t d){return int32_t(now-d)>=0;}

inline double toaMs(const Profile&p,unsigned bytes=PACKET_BYTES){
  const double ts=1000.0*(1UL<<p.sf)/p.bw;
  const int de=ts>16.0?1:0;
  const double n=ceil((8.0*bytes-4*p.sf+28+16)/(4*(p.sf-2*de)));
  return(PREAMBLE+4.25+8+fmax(n*(p.cr+0.0),0.0))*ts;
}
inline uint32_t minInterval(const Profile&p){
  return(uint32_t)ceil(toaMs(p)/DUTY_BUDGET)+500;
}
inline void put16(uint8_t*p,uint16_t n){p[0]=n;p[1]=n>>8;}
inline void put32(uint8_t*p,uint32_t n){for(int i=0;i<4;i++)p[i]=n>>(8*i);}
inline uint16_t get16(const uint8_t*p){return uint16_t(p[0])|uint16_t(p[1])<<8;}
inline uint32_t get32(const uint8_t*p){uint32_t n=0;for(int i=0;i<4;i++)n|=uint32_t(p[i])<<(8*i);return n;}
inline int16_t signed16(uint16_t x){return x<32768?int16_t(x):int16_t(int32_t(x)-65536);}

inline void packet(uint8_t*b,const Config&c,uint32_t seq,uint8_t type,int16_t rssi=0,int16_t snrQ4=0){
  memset(b,0,PACKET_BYTES);b[0]='L';b[1]='B';b[2]=1;b[3]=type;
  put32(b+4,c.run);put32(b+8,seq);put16(b+12,c.point);
  b[14]=c.preset;b[15]=c.mode;put32(b+16,c.count);b[20]=uint8_t(c.power);
  put16(b+22,uint16_t(rssi));put16(b+24,uint16_t(snrQ4));
  for(size_t i=26;i<PACKET_BYTES;i++)b[i]=uint8_t(0xa0+i);
}
inline bool matches(const uint8_t*b,size_t n,const Config&c,uint8_t type){
  if(n!=PACKET_BYTES||b[0]!='L'||b[1]!='B'||b[2]!=1||b[3]!=type||
     get32(b+4)!=c.run||get16(b+12)!=c.point||b[14]!=c.preset||
     b[15]!=c.mode||get32(b+16)!=c.count||b[20]!=uint8_t(c.power)||
     b[21]!=0||get32(b+8)<1||get32(b+8)>c.count)return false;
  for(size_t i=26;i<PACKET_BYTES;i++)if(b[i]!=uint8_t(0xa0+i))return false;
  return true;
}

// CSV STATS
const char LOG_PATH[]="/measurements.csv";
const char CSV_HEADER[]="schema,node,boot_id,event,run_id,point_id,preset,mode,seq,t_ms,freq_hz,sf,bw_hz,cr_den,tx_power_dbm,packet_bytes,planned_count,interval_ms,toa_ms,rssi_dbm,snr_db,pdr_pct,ack_count";

void failLog(){fault=true;active=false;onAir=false;waitingAck=false;ackPending=false;radio.sleep();Serial.println("#ERROR LOG_WRITE_OR_SPACE");}

bool logRow(const char*event,uint32_t packetSeq=0,double rssi=NAN,double snr=NAN){
  String row;row.reserve(350);
  row="1,";row+="A,";row+=String(bootId);row+=',';row+=event;row+=',';
  row+=String(cfg.run);row+=',';row+=String(cfg.point);row+=',';
  row+=String(cfg.preset);row+=',';row+=String(cfg.mode);row+=',';
  row+=String(packetSeq);row+=',';row+=String(millis());row+=',';
  row+=String((uint32_t)lround(FREQ_HZ*1000.0)*1000UL);row+=',';row+=String(profile().sf);row+=',';
  row+=String(profile().bw);row+=',';row+=String(profile().cr);row+=',';
  row+=String(int(cfg.power));row+=',';row+=String(PACKET_BYTES);row+=',';
  row+=String(cfg.count);row+=',';row+=String(cfg.interval);row+=',';
  row+=String(toaMs(profile()),3);
  
  // RSSI+SNR
  row+=',';if(!isnan(rssi))row+=String(rssi,3);
  row+=',';if(!isnan(snr))row+=String(snr,3);
  
  // PDR+ACK
  double pdr = (seq > 0) ? ((double)ackCount / seq) * 100.0 : 0.0;
  row+=',';row+=String(pdr, 2);
  row+=',';row+=String(ackCount);

  if(!fsOK||LittleFS.totalBytes()-LittleFS.usedBytes()<16384){failLog();return false;}
  File f=LittleFS.open(LOG_PATH,FILE_APPEND);
  if(!f){failLog();return false;}
  bool ok=true;
  if(f.size()==0)ok=f.println(CSV_HEADER)==strlen(CSV_HEADER)+2;
  ok=(f.println(row)==row.length()+2)&&ok;
  f.flush();f.close();
  if(!ok){failLog();return false;}
  Serial.println(row);return true;
}

// RF CONF APPL
bool applyRadio(){
  int16_t e=0,st;
  radio.standby();
  if((st=radio.setFrequency(FREQ_HZ))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setSpreadingFactor(profile().sf))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setBandwidth(profile().bw/1000.0f))!=RADIOLIB_ERR_NONE)e=st; // RadioLib expects kHz
  if((st=radio.setCodingRate(profile().cr))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setPreambleLength(PREAMBLE))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setSyncWord(SYNC_WORD))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setCRC(true))!=RADIOLIB_ERR_NONE)e=st;
  if((st=radio.setOutputPower(cfg.power))!=RADIOLIB_ERR_NONE)e=st;
  if(e){Serial.printf("#ERROR radio config, code %d\n",e);return false;}
  return true;
}

// COMMANDS HANDLER
void status(){
  Serial.printf("# node=A radio=%s fs=%s active=%d fault=%d run=%lu point=%u preset=%u mode=%u count=%lu interval_ms=%lu power_dbm=%d\n",
    radioOK?"OK":"FAIL",fsOK?"OK":"FAIL",active,fault,
    (unsigned long)cfg.run,cfg.point,cfg.preset,cfg.mode,
    (unsigned long)cfg.count,(unsigned long)cfg.interval,cfg.power);
  Serial.printf("# SF=%u BW=%lu CR=4/%u ToA_ms=%.3f min_interval_ms=%lu rx_unique=%lu\n",
    profile().sf,(unsigned long)profile().bw,profile().cr,
    toaMs(profile()),(unsigned long)minInterval(profile()),(unsigned long)rxUnique);
  if(fsOK)Serial.printf("# flash_used=%lu flash_total=%lu\n",
    (unsigned long)LittleFS.usedBytes(),(unsigned long)LittleFS.totalBytes());
}

void help(){
  Serial.println("# config RUN POINT PRESET MODE COUNT INTERVAL_MS POWER_DBM");
  Serial.println("# Example: config 1001 1 1 0 100 200000 10");
  Serial.println("# MODE 0=one-way 1=ACK; PRESET 1..5");
  Serial.println("# start, stop, reset, status, table, dump, erase YES, format YES");
}

void finishTxRun(){
  active=false;
  waitingAck=false;
  radio.standby();
  logRow("END",seq);
  
  double finalPdr = (seq > 0) ? ((double)ackCount / seq) * 100.0 : 0.0;
  Serial.printf("# DONE; Attempted: %lu, ACKs: %lu, Final PDR: %.2f%%\n", seq, ackCount, finalPdr);
}

void command(String line){
  line.trim();if(!line.length())return;
  if(line=="status"){status();return;}
  if(line=="help"||line=="?"){help();return;}
  if(line=="table"){
    for(int i=0;i<5;i++)Serial.printf("# preset=%d SF=%u BW=%lu CR=4/%u ToA32_ms=%.3f min_interval_ms=%lu\n",
      i+1,profiles[i].sf,(unsigned long)profiles[i].bw,profiles[i].cr,
      toaMs(profiles[i]),(unsigned long)minInterval(profiles[i]));
    return;
  }
  if(line=="reset"){
    seq=0;
    ackCount=0;
    fault=false;
    Serial.println("#RESET OK");
    return;
  }
  if(line=="stop"){
    const bool wasActive=active;
    if(onAir)logRow("TX_UNCERTAIN",seq);
    active=false;onAir=false;waitingAck=false;ackPending=false;
    radio.standby();
    if(wasActive&&!fault)logRow("STOP",seq);
    return;
  }
  if(active||onAir){Serial.println("#ERROR busy; stop first");return;}
  if(line=="format YES"){
    LittleFS.end();fsOK=false;
    if(LittleFS.format())fsOK=LittleFS.begin(false);
    fault=!fsOK;Serial.println(fsOK?"# filesystem formatted":"#ERROR format");return;
  }
  if(line=="erase YES"){
    if(!fsOK){Serial.println("#ERROR no filesystem");return;}
    bool ok=!LittleFS.exists(LOG_PATH)||LittleFS.remove(LOG_PATH);
    if(ok)fault=false;
    Serial.println(ok?"# journal erased":"#ERROR erase");return;
  }
  if(line=="dump"){
    if(!fsOK){Serial.println("#ERROR no filesystem");return;}
    File f=LittleFS.open(LOG_PATH,FILE_READ);
    Serial.println("#DUMP_BEGIN");
    if(f){uint8_t buf[256];while(f.available()){size_t n=f.read(buf,sizeof(buf));Serial.write(buf,n);delay(1);}f.close();}
    else Serial.println(CSV_HEADER);
    Serial.println("#DUMP_END");return;
  }
  if(line.startsWith("config ")){
    uint32_t v[7];const char*p=line.c_str()+7;
    for(int i=0;i<7;i++){
      while(*p==' ')++p;
      if(*p<'0'||*p>'9'){Serial.println("#ERROR config syntax");return;}
      uint64_t value=0;
      while(*p>='0'&&*p<='9'){value=value*10+(*p++-'0');if(value>UINT32_MAX){Serial.println("#ERROR overflow");return;}}
      if(*p&&*p!=' '){Serial.println("#ERROR config syntax");return;}
      v[i]=uint32_t(value);
    }
    while(*p==' ')++p;
    if(*p||v[1]>65535||v[2]>5||v[3]>1||v[6]>17){Serial.println("#ERROR config values");return;}
    Config c;c.run=v[0];c.point=v[1];c.preset=v[2];c.mode=v[3];c.count=v[4];c.interval=v[5];c.power=v[6];
    if(!prefsOK||prefs.putBytes("cfg",&c,sizeof(c))!=sizeof(c)){Serial.println("#ERROR NVS");return;}
    cfg=c;if(radioOK&&!applyRadio()){Serial.println("#ERROR preset not applied");}status();return;
  }
  if(line=="start"){
    if(!radioOK||!fsOK||!prefsOK||fault){status();return;}
    if(prefs.getUInt("last_run",0)==cfg.run){Serial.println("#ERROR choose a NEW run ID");return;}
    if(!logRow("START"))return;
    if(!prefs.putUInt("last_run",cfg.run)){Serial.println("#ERROR NVS");return;}
    seq=0;
    ackCount=0;
    active=true;nextTx=millis()+5000;
    Serial.println("# Starting in 5 seconds");return;
  }
  Serial.println("#ERROR unknown command; help");
}

// SERIAL HANDLER
void serviceSerial(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\n'){
      if(!inputOverflow)command(inputLine);else Serial.println("#ERROR line too long");
      inputLine="";inputOverflow=false;
    }else if(ch!='\r'){
      if(inputLine.length()<159)inputLine+=ch;else inputOverflow=true;
    }
  }
}

// PCK RECV
void serviceReceived(){
  if(!radioOK)return;
  // only read when a NEW packet has arrived (RX_DONE flag)
  if(!(radio.getIrqFlags()&RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_DONE))return;
  const size_t n=radio.getPacketLength();
  uint8_t b[PACKET_BYTES];
  // read at most PACKET_BYTES: a longer foreign packet must not overflow the buffer
  const int16_t st=radio.readData(b,PACKET_BYTES);
  const double rssi=radio.getRSSI(),snr=radio.getSNR();
  radio.startReceive();
  if(st!=RADIOLIB_ERR_NONE||n!=PACKET_BYTES)return; // CRC error / wrong size
  if(!matches(b,n,cfg,2))return;
  const uint32_t packetSeq=get32(b+8);
  if(!waitingAck||packetSeq!=seq)return;
  waitingAck=false;
  
  ackCount++; 
  if(!logRow("ACK_RX",seq,rssi,snr))return;
  
  double instantPdr = (seq > 0) ? ((double)ackCount / seq) * 100.0 : 0.0;
  Serial.printf("# Instant PDR: %.2f%% (%lu/%lu)\n", instantPdr, ackCount, seq);

  if(seq==cfg.count)finishTxRun();
}

// SETUP
void labSetup(){
  Serial.begin(115200);delay(400);inputLine.reserve(160);bootId=esp_random();
  prefsOK=prefs.begin("labtxv1",false);
  Config saved;
  if(prefsOK&&prefs.getBytesLength("cfg")==sizeof(saved)&&prefs.getBytes("cfg",&saved,sizeof(saved))==sizeof(saved))cfg=saved;
  fsOK=LittleFS.begin(false);
  SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI,PIN_NSS);
  
  radioOK=(radio.begin(FREQ_HZ,125.0,7,5,SYNC_WORD,10,PREAMBLE)==RADIOLIB_ERR_NONE);
  if(!radioOK){Serial.println("#ERROR radio init");return;}
  radioOK=applyRadio();
  if(!radioOK)return;
  Serial.println(CSV_HEADER);help();status();
  if(!fsOK)Serial.println("# First use only: format YES");
}

// LOOP
void labLoop(){
  serviceSerial();
  if(!active||fault){delay(1);return;}
  uint32_t now=millis();
  if(!waitingAck&&due(now,nextTx)){
    ++seq;
    if(!logRow("TX_ATTEMPT",seq))return;
    packet(txBuffer,cfg,seq,1);
    txStart=millis();
    if(radio.startTransmit(txBuffer,PACKET_BYTES)!=RADIOLIB_ERR_NONE){
      logRow("TX_FAIL",seq);active=false;return;
    }
    onAir=true;nextTx=txStart+cfg.interval;
  }
  if(onAir){
    uint32_t irq = radio.getIrqFlags();
    if(irq&RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE){
      radio.clearIrqFlags(RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE);
      onAir=false;const uint32_t elapsed=millis()-txStart;
      radio.startReceive();
      waitingAck=(cfg.mode==1);
      ackDeadline=millis()+uint32_t(ceil(toaMs(profile())))+ACK_DELAY_MS+2000;
      if(!logRow("TX_DONE",seq,NAN,NAN))return;
      if(!waitingAck&&seq==cfg.count)finishTxRun();
    }else if(uint32_t(now-txStart)>uint32_t(ceil(toaMs(profile())))+2000){
      radio.standby();onAir=false;
      logRow("TX_UNCERTAIN",seq);active=false;
    }
    delay(1);return;
  }
  if(waitingAck)serviceReceived();
  if(waitingAck&&due(millis(),ackDeadline)){
    waitingAck=false;
    if(!logRow("ACK_TIMEOUT",seq))return;
    if(seq==cfg.count)finishTxRun();
  }
  delay(1);
}

void setup(){labSetup();}
void loop(){labLoop();}