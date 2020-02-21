//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-04-16 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>

// Arduino Pro mini 8 Mhz
// Arduino pin for the config button
#define CONFIG_BUTTON_PIN  8
#define LED_PIN            4

#define SENSOR_PIN         14

// number of available peers per channel
#define PEERS_PER_CHANNEL 2

// all library classes are placed in the namespace 'as'
using namespace as;

//Korrekturfaktor der Clock-Ungenauigkeit, wenn keine RTC verwendet wird
#define SYSCLOCK_FACTOR    0.88
#define MAX_MEASURE_COUNT  5

enum UltrasonicSensorTypes {
  VOTRONIC_12_24_K,
  VOTRONIC_15_50_K,
  VOTRONIC_30_110_K,
  VOTRONIC_20_K_WC
};

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xFA, 0xB4, 0x01},          // Device ID
  "HBLEV_VO01",                // Device Serial
  {0xFA, 0xB4},                // Device Model
  0x10,                        // Firmware Version
  0x53,                        // Device Type
  {0x01, 0x01}                 // Info Bytes
};

/**
   Configure the used hardware
   NoBattery
   BatterySensorUni<BATT_SENS_PIN, BATT_EN_PIN, 0>
*/
typedef AskSin<StatusLed<LED_PIN>,NoBattery , Radio<AvrSPI<10, 11, 12, 13>, 2>> BaseHal;
class Hal : public BaseHal {
  public:
    void init (const HMID& id) {
      BaseHal::init(id);
      //battery.init(seconds2ticks(60UL * 60) * SYSCLOCK_FACTOR, sysclock); //battery measure once an hour
      //battery.low(22);
      //battery.critical(19);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;


DEFREGISTER(UReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x20, 0x21)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    void defaults () {
      clear();
      lowBatLimit(22);
      Sendeintervall(180);
    }
};

DEFREGISTER(UReg1, CREG_CASE_HIGH, CREG_CASE_WIDTH, CREG_CASE_DESIGN, CREG_CASE_LENGTH, 0x01, 0x02, 0x03)
class UList1 : public RegList1<UReg1> {
  public:
    UList1 (uint16_t addr) : RegList1<UReg1>(addr) {}

    bool distanceOffset (uint16_t value) const {
      return this->writeRegister(0x01, (value >> 8) & 0xff) && this->writeRegister(0x02, value & 0xff);
    }
    uint16_t distanceOffset () const {
      return (this->readRegister(0x01, 0) << 8) + this->readRegister(0x02, 0);
    }

    bool sensorType (uint16_t value) const {
      return this->writeRegister(0x03, value & 0xff);
    }
    uint16_t sensorType () const {
      return this->readRegister(0x03, 0);
    }

    void defaults () {
      clear();
      caseHigh(100);
      caseWidth(100);
      caseLength(100);
      caseDesign(0);
      distanceOffset(0);
      sensorType(0);
    }
};

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint8_t percent, uint32_t liter, uint8_t volt) {
      Message::init(0x0f, msgcnt, 0x53, (msgcnt % 20 == 1) ? BIDI : BCAST , percent & 0xff, volt & 0xff);
      pload[0] = (liter >>  24) & 0xff;
      pload[1] = (liter >>  16) & 0xff;
      pload[2] = (liter >>  8) & 0xff;
      pload[3] = liter & 0xff;
    }
};

class MeasureChannel : public Channel<Hal, UList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>, public Alarm {
    MeasureEventMsg msg;
    uint8_t  fillingPercent;
    uint32_t fillingLiter;
    uint16_t distance;

    uint8_t last_flags = 0xff;

  public:
    MeasureChannel () : Channel(), Alarm(0), fillingLiter(0), fillingPercent(0)  {}
    virtual ~MeasureChannel () {}

    void measure() {
      uint32_t caseHeight = this->getList1().caseHigh();
      uint32_t caseWidth = this->getList1().caseWidth();
      uint32_t caseLength = this->getList1().caseLength();
      uint32_t caseDesign  = this->getList1().caseDesign();
      uint32_t distanceOffset = this->getList1().distanceOffset();

      float m_value = 0;
      float temp = 0;
      float temp1 = 0;

      switch (this->getList1().sensorType()) {
        case VOTRONIC_12_24_K:
          // Messwert = 0.44V 
          temp1 = analogRead(A0);
          temp = temp1 * (3.3/1024);
          m_value = (temp/100);
          break;
        case VOTRONIC_15_50_K:
          //
          // Lesen des Eingangs am analogen Pin A0:
          int sensorValue = analogRead(A0);
          // Umrechnung des Analogwertes (welcher von 0 bis 1023 reicht) in eine Spannung von (0V bis 5V):
          float voltage = sensorValue * (3.3 / 1023.0);
          DPRINT(F("Sondenwert (V): ")); DDECLN(voltage);
          m_value = (voltage);
          break;

        case VOTRONIC_30_110_K:
          //
          temp1 = analogRead(A0);
          temp = temp1 * (3.3/1024);
          m_value = (temp/100);
          break;
        case VOTRONIC_20_K_WC:
          //
          temp1 = analogRead(A0);
          temp = temp1 * (3.3/1024);
          m_value = (temp/100);
          break;
        default:
          DPRINTLN(F("Invalid Sensor Type selected"));
          break;
      }

      distance = distanceOffset;
      DPRINT(F("Offset in Litern    : ")); DDECLN(distance);
     
      float caseVolume; float r;
      switch (caseDesign) {
        case 0:
          caseVolume = (PI * pow((caseWidth >> 1), 2) * caseHeight) / 1000000L;
          //fillingLiter = (PI * pow((caseWidth >> 1), 2) * fillingHeight) / 1000000L;
          break;
        case 1:
          caseVolume = (PI * pow((caseHeight >> 1), 2) * caseWidth) / 1000000L;
          r = caseHeight  / 2;
          //fillingLiter = (r * r * 2 * acos(1 - fillingHeight / r) / 2 - 2 * sqrt(caseHeight * fillingHeight - fillingHeight * fillingHeight) * (r - fillingHeight) / 2) * caseWidth / 1000000L;
          break;
        case 2:
          caseVolume = ((caseHeight * caseWidth * caseLength) / 1000000L) - distanceOffset; //SSt.: Abzug Volumenkorrektur
          caseVolume = (caseVolume / 100);
          //fillingLiter = ((fillingHeight * caseWidth * caseLength) / 1000000L) - distanceOffset; //SSt.: Abzug Volumenkorrektur
          break;
        default:
          DPRINTLN(F("Invalid caseDesign")); DDECLN(caseDesign);
          break;
      }

      //fillingPercent = (fillingLiter * 100) / caseVolume;
      DPRINT(F("Behaeltervolumen (gesamt): ")); DDECLN(caseVolume);
      fillingPercent = (100/2.2) * m_value;
      fillingLiter = caseVolume * fillingPercent * 10;
      DPRINT(F("Messwert (V): ")); DDECLN(m_value);
      DPRINT(F("Füllinhalt (%): ")); DDECLN(fillingPercent);
      DPRINT(F("Füllinhalt (L): ")); DDECLN(fillingLiter);
      DPRINT(F("Inhalt                   : ")); DDEC(fillingLiter); DPRINT(F("L (")); DDEC(fillingPercent); DPRINTLN(F("%)"));
      DPRINTLN("");
    }
    virtual void trigger (__attribute__ ((unused)) AlarmClock & clock) {
      if (last_flags != flags()) {
        this->changed(true);
        last_flags = flags();
      }
      measure();
      tick = delay();
      msg.init(device().nextcount(), fillingPercent, fillingLiter, device().battery().current());
      device().sendPeerEvent(msg, *this);
      sysclock.add(*this);
    }

    uint32_t delay () {
      uint16_t _txMindelay = 20;
      _txMindelay = device().getList0().Sendeintervall();
      if (_txMindelay == 0) _txMindelay = 20;
      return seconds2ticks(_txMindelay  * SYSCLOCK_FACTOR);
    }

    void configChanged() {
      DPRINTLN(F("Config changed List1"));
      DPRINT(F("*CASE_HIGH:       "));
      DDECLN(this->getList1().caseHigh());
      DPRINT(F("*CASE_WIDTH:      "));
      DDECLN(this->getList1().caseWidth());
      DPRINT(F("*CASE_LENGTH:     "));
      DDECLN(this->getList1().caseLength());
      DPRINT(F("*CASE_DESIGN:     "));
      DDECLN(this->getList1().caseDesign());
      DPRINT(F("*DISTANCE_OFFSET: "));
      DDECLN(this->getList1().distanceOffset());
      DPRINT(F("*SENSOR_TYPE:     "));
      DDECLN(this->getList1().sensorType());
    }

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      pinMode(SENSOR_PIN, INPUT);
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      
      uint8_t flags = this->device().battery().low() ? 0x80 : 0x00;
      return flags;
    }
};

class UType : public MultiChannelDevice<Hal, MeasureChannel, 1, UList0> {
  public:
    typedef MultiChannelDevice<Hal, MeasureChannel, 1, UList0> TSDevice;
    UType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr) {}
    virtual ~UType () {}

    virtual void configChanged () {
      TSDevice::configChanged();
      //DPRINT(F("*LOW BAT Limit: "));
      //DDECLN(this->getList0().lowBatLimit());
      //this->battery().low(this->getList0().lowBatLimit());
      DPRINT(F("*Sendeintervall: ")); DDECLN(this->getList0().Sendeintervall());
    }
};

UType sdev(devinfo, 0x20);
ConfigButton<UType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  
  DDEVINFO(sdev);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    if ( hal.battery.critical() ) {
      hal.activity.sleepForever(hal);
    }
    hal.activity.savePower<Sleep<>>(hal);
  }
}
