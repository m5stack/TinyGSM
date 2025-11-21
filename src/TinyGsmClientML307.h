/**
 * @file       TinyGsmClientML307.h
 * @author     junhuang
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2025 M5Stack Technology CO LTD
 * @date       November 2025
 */

#ifndef SRC_TINYGSMCLIENTML307_H_
#define SRC_TINYGSMCLIENTML307_H_
// #pragma message("TinyGSM:  TinyGsmClientML307")

//#define TINY_GSM_DEBUG Serial

#define TINY_GSM_MUX_COUNT 10
#define TINY_GSM_BUFFER_READ_AND_CHECK_SIZE
#ifdef AT_NL
#undef AT_NL
#endif
#define AT_NL "\r\n"

#include "TinyGsmModem.tpp"
#include "TinyGsmTCP.tpp"
#include "TinyGsmSSL.tpp"
#include "TinyGsmGPRS.tpp"
#include "TinyGsmCalling.tpp"
#include "TinyGsmSMS.tpp"
#include "TinyGsmGPS.tpp"
#include "TinyGsmTime.tpp"
#include "TinyGsmNTP.tpp"
#include "TinyGsmBattery.tpp"
#include "TinyGsmTemperature.tpp"

enum ML307RegStatus {
  REG_NO_RESULT    = -1,
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

class TinyGsmML307 : public TinyGsmModem<TinyGsmML307>,
                    public TinyGsmGPRS<TinyGsmML307>,
                    public TinyGsmTCP<TinyGsmML307, TINY_GSM_MUX_COUNT>,
                    public TinyGsmSSL<TinyGsmML307, TINY_GSM_MUX_COUNT>,
                    public TinyGsmCalling<TinyGsmML307>,
                    public TinyGsmSMS<TinyGsmML307>,
                    public TinyGsmGPS<TinyGsmML307>,
                    public TinyGsmTime<TinyGsmML307>,
                    public TinyGsmNTP<TinyGsmML307>,
                    public TinyGsmBattery<TinyGsmML307>,
                    public TinyGsmTemperature<TinyGsmML307> {
  friend class TinyGsmModem<TinyGsmML307>;
  friend class TinyGsmGPRS<TinyGsmML307>;
  friend class TinyGsmTCP<TinyGsmML307, TINY_GSM_MUX_COUNT>;
  friend class TinyGsmSSL<TinyGsmML307, TINY_GSM_MUX_COUNT>;
  friend class TinyGsmCalling<TinyGsmML307>;
  friend class TinyGsmSMS<TinyGsmML307>;
  friend class TinyGsmGPS<TinyGsmML307>;
  friend class TinyGsmTime<TinyGsmML307>;
  friend class TinyGsmNTP<TinyGsmML307>;
  friend class TinyGsmBattery<TinyGsmML307>;
  friend class TinyGsmTemperature<TinyGsmML307>;

  /*
   * Inner Client
   */
 public:
  class GsmClientML307 : public GsmClient {
    friend class TinyGsmML307;

   public:
    GsmClientML307() {}

    explicit GsmClientML307(TinyGsmML307& modem, uint8_t mux = 0) {
      ssl_sock = false;
      init(&modem, mux);
    }

    bool init(TinyGsmML307* modem, uint8_t mux = 0) {
      this->at       = modem;
      sock_available = 0;
      prev_check     = 0;
      sock_connected = false;
      got_data       = false;

      if (mux < TINY_GSM_MUX_COUNT) {
        this->mux = mux;
      } else {
        this->mux = (mux % TINY_GSM_MUX_COUNT);
      }
      at->sockets[this->mux] = this;

      return true;
    }

    virtual int connect(const char* host, uint16_t port, int timeout_s) {
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, timeout_s);
      return sock_connected;
    }
    TINY_GSM_CLIENT_CONNECT_OVERRIDES

    virtual void stop(uint32_t maxWaitMs) {
      uint32_t startMillis = millis();
      dumpModemBuffer(maxWaitMs);
      at->sendAT(GF("+MIPCLOSE="), mux);
      sock_connected = false;
      at->waitResponse((maxWaitMs - (millis() - startMillis)));
    }
    void stop() override {
      stop(15000L);
    }

    /*
     * Extended API
     */

    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;

   protected:
    bool ssl_sock;
  };

  /*
   * Constructor
   */
 public:
  explicit TinyGsmML307(Stream& stream) : stream(stream) {
    memset(sockets, 0, sizeof(sockets));
  }

  /*
   * Basic functions
   */
 protected:
  bool initImpl(const char* pin = nullptr) {
    DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
    DBG(GF("### TinyGSM Compiled Module:  TinyGsmClientML307"));

    if (!testAT()) { return false; }

#ifdef TINY_GSM_DEBUG
    sendAT(GF("+CMEE=2"));  // turn on verbose error codes
#else
    sendAT(GF("+CMEE=0"));  // turn off error codes
#endif
    waitResponse();

    DBG(GF("### Modem:"), getModemName());

    // Disable time and time zone URC's
    sendAT(GF("+CTZR=0"));
    if (waitResponse(10000L) != 1) { return false; }

    // Enable automatic time zone update
    sendAT(GF("+CTZU=1"));
    if (waitResponse(10000L) != 1) { return false; }

    SimStatus ret = getSimStatus();
    // if the sim isn't ready and a pin has been provided, try to unlock the sim
    if (ret != SIM_READY && pin != nullptr && strlen(pin) > 0) {
      simUnlock(pin);
      return (getSimStatus() == SIM_READY);
    } else {
      // if the sim is ready, or it's locked but no pin has been provided,
      // return true
      return (ret == SIM_READY || ret == SIM_LOCKED);
    }
  }

  /*
   * Power functions
   */
 protected:
  bool restartImpl(const char* pin = nullptr) {
    if (!testAT()) { return false; }
    if (!setPhoneFunctionality(1, true)) { return false; }
    waitResponse(10000L, GF("APP RDY"));
    return init(pin);
  }

  bool powerOffImpl() {
    sendAT(GF("+MPOF=0"));
    waitResponse(300);
    return waitResponse(300, GF("POWER OFF")) == 1;
  }

  // When entering into sleep mode is enabled, DTR is pulled up, and WAKEUP_IN
  // is pulled up, the module can directly enter into sleep mode.If entering
  // into sleep mode is enabled, DTR is pulled down, and WAKEUP_IN is pulled
  // down, there is a need to pull the DTR pin and the WAKEUP_IN pin up first,
  // and then the module can enter into sleep mode.
  bool sleepEnableImpl(bool enable = true) {
     if (enable) {
        sendAT(GF("+MLPMCFG=\"sleepmode\",2,0"));
    } else {
        sendAT(GF("+MLPMCFG=\"sleepmode\",0,0"));
    }
    return waitResponse() == 1;
  }

  bool setPhoneFunctionalityImpl(uint8_t fun, bool reset = false,
                                 uint32_t timeout_ms = 15500L) {
    sendAT(GF("+CFUN="), fun, reset ? ",1" : "");
    return waitResponse(timeout_ms, GF("OK")) == 1;
  }

  /*
   * Generic network functions
   */
 public:
  ML307RegStatus getRegistrationStatus() {
    // Check first for EPS registration
    ML307RegStatus epsStatus = (ML307RegStatus)getRegistrationStatusXREG("CEREG");

    // If we're connected on EPS, great!
    if (epsStatus == REG_OK_HOME || epsStatus == REG_OK_ROAMING) {
      return epsStatus;
    } else {
      // Otherwise, check generic network status
      return (ML307RegStatus)getRegistrationStatusXREG("CREG");
    }
  }

 protected:
  bool isNetworkConnectedImpl() {
    ML307RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
  }

  String getLocalIPImpl() {
    sendAT(GF("+CGPADDR=1"));
    String res;
    if (waitResponse(10000L, res) != 1) { return ""; }
    //+CGPADDR: 1,"10.87.9.219","2409:..."
    int firstQuote = res.indexOf('"');
    int secondQuote = res.indexOf('"', firstQuote + 1);
    if (firstQuote >= 0 && secondQuote > firstQuote) {
        return res.substring(firstQuote + 1, secondQuote); // IPv4
    }
    return "";
  }
  /*
   * Secure socket layer (SSL) functions
   */
  // Follows functions as inherited from TinyGsmSSL.tpp

  /*
   * WiFi functions
   */
  // No functions of this type supported
  /*
   * GPRS functions
   */
 protected:
  bool gprsConnectImpl(const char* apn, const char* user = nullptr,
                       const char* pwd = nullptr) {
    gprsDisconnect();

    // Define the PDP context (This uses context #3!)
    sendAT(GF("+CGDCONT=1,\"IPV4V6\",\""), apn, '"');
    waitResponse();

    // Set authentication
    if (user && strlen(user) > 0) {
      sendAT(GF("+CGAUTH=1,1,\""), user, GF("\",\""), pwd, GF("\""));
      waitResponse();
    }

    sendAT(GF("+MIPCALL=1,1"));
    if (waitResponse(120000L, GF("+MIPCALL:")) != 1) {
      DBG("### Failed to activate PDN dial-up");
      return false;
    }

    // Activate the PDP context
    sendAT(GF("+CGACT=1,1"));
    waitResponse(60000L);

    // Attach to GPRS
    sendAT(GF("+CGATT=1"));
    if (waitResponse(60000L) != 1) { return false; }

    return true;
  }

  bool gprsDisconnectImpl() {
    // Detach from PS network
    sendAT(GF("+CGATT=0"));
    if (waitResponse(60000L) != 1) { return false; }
    // Dectivate all PDP contexts
    sendAT(GF("+CGACT=0"));
    if (waitResponse(60000L) != 1) { return false; }

    return true;
  }
  /*
   * SIM card functions
   */
 protected:
  String getSimCCIDImpl() {
    sendAT(GF("+ICCID"));
    if (waitResponse(GF(AT_NL "+ICCID:")) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  // The ML307 returns UTC time instead of local time as other modules do in
  // response to CCLK, so we're using QLTS where we can specifically request
  // local time.
  bool getNetworkUTCTimeImpl(int* year, int* month, int* day, int* hour,
                           int* minute, int* second, float* timezone) {
    sendAT(GF("+CCLK?"));
    if (waitResponse(2000L, GF("+CCLK: \"")) != 1) {
        return false;
    }
    int iyear     = 0;
    int imonth    = 0;
    int iday      = 0;
    int ihour     = 0;
    int imin      = 0;
    int isec      = 0;
    int itimezone = 0;
    iyear       = streamGetIntBefore('/');
    imonth      = streamGetIntBefore('/');
    iday        = streamGetIntBefore(',');
    ihour       = streamGetIntBefore(':');
    imin        = streamGetIntBefore(':');
    isec        = streamGetIntLength(2);
    char tzSign = stream.read();
    itimezone   = streamGetIntBefore('"');
    if (tzSign == '-') itimezone = -itimezone;
    streamSkipUntil('\n');
    if (iyear < 100) iyear += 2000;
    if (year     != nullptr) *year     = iyear;
    if (month    != nullptr) *month    = imonth;
    if (day      != nullptr) *day      = iday;
    if (hour     != nullptr) *hour     = ihour;
    if (minute   != nullptr) *minute   = imin;
    if (second   != nullptr) *second   = isec;
    if (timezone != nullptr) *timezone = static_cast<float>(itimezone) / 4.0;

    waitResponse();
    return true;
  }
  /*
   * Phone Call functions
   */
  // Follows all phone call functions as inherited from TinyGsmCalling.tpp

  /*
   * Audio functions
   */
  // No functions of this type supported

  /*
   * Text messaging (SMS) functions
   */
  // Follows all text messaging (SMS) functions as inherited from TinyGsmSMS.tpp

  /*
   * GSM Location functions
   */
 protected:
  // NOTE:  As of application firmware version 01.016.01.016 triangulated
  // locations can be obtained via the QuecLocator service and accompanying AT
  // commands.  As this is a separate paid service which I do not have access
  // to, I am not implementing it here.

 
  /*
   * NTP server functions
   */

  byte NTPServerSyncImpl(String server = "pool.ntp.org", byte = -5) {
    // Request network synchronization
    // AT+MNTP=<contextID>,<server>[,<port>][,<autosettime>]
    sendAT(GF("+MNTP=1,\""), server, '"');
    if (waitResponse(10000L, GF("+MNTP:"))) {
      String result = stream.readStringUntil(',');
      streamSkipUntil('\n');
      result.trim();
      if (TinyGsmIsValidNumber(result)) { return result.toInt(); }
    } else {
      return -1;
    }
    return -1;
  }

  String ShowNTPErrorImpl(byte error) TINY_GSM_ATTR_NOT_IMPLEMENTED;

  /*
   * BLE functions
   */
  // No functions of this type supported

  /*
   * Battery functions
   */
  // Follows all battery functions as inherited from TinyGsmBattery.tpp

  /*
   * Temperature functions
   */
 protected:
  // get temperature in degree celsius
  uint16_t getTemperatureImpl() {
    sendAT(GF("+MCHIPINFO=\"temp\""));
    if (waitResponse(GF(AT_NL "+MCHIPINFO:")) != 1) { 
        return 0; 
    }
    streamSkipUntil(',');
    uint16_t tempC = streamGetIntBefore('\n');
    waitResponse();
    return tempC;
  }

  uint16_t getVbatImpl() {
      sendAT(GF("+MCHIPINFO=\"vbat\""));
      if (waitResponse(GF(AT_NL "+MCHIPINFO:")) != 1) { 
          return 0; 
      }
      streamSkipUntil(',');
      uint16_t vbatMv = streamGetIntBefore('\n');
      waitResponse();
      return vbatMv;
  }

  /*
 * Client related functions (ML307R)
 */
 protected:
  bool modemConnect(const char* host, uint16_t port, uint8_t mux,
                     int timeout_s = 15) {

    sendAT(GF("+MIPCFG=\"cid\","), mux, ",1");
    if (waitResponse(5000L) != 1) {
      DBG("### MIPCFG cid config failed");
      return false;
    }

    sendAT(GF("+MIPCFG=\"encoding\","), mux, ",1,1");
    if (waitResponse(5000L) != 1) {
      DBG("### MIPCFG encoding config failed");
      return false;
    }

    sendAT(GF("+MIPCFG=\"cached\","), mux, ",1");
    if (waitResponse(5000L) != 1) {
      DBG("### MIPCFG cached config failed");
      return false;
    }

    sendAT(GF("+MIPCFG=\"timeout\","), mux, ",30");
    if (waitResponse(5000L) != 1) {
      DBG("### MIPCFG timeout config failed");
      return false;
    }

    sendAT(GF("+MIPOPEN="), mux, ",\"TCP\",\"", host, GF("\","), port, ",60,2");
    if (waitResponse(5000L, GF("+MIPOPEN:")) != 1) {
      DBG("### MIPOPEN failed or timeout");
      return false;
    }

    //+MIPOPEN: <id>,<code>
    String mStr = stream.readStringUntil(',');
    uint8_t opened_mux = mStr.toInt();
    String rStr = stream.readStringUntil('\n');
    uint8_t opened_result = rStr.toInt();
    waitResponse();

    if (opened_mux != mux || opened_result != 0) {
      DBG("### Connection open failed, result: ", opened_result);
      return false;
    }

    delay(2000);

    sendAT(GF("+MIPSTATE="), mux);
    if (waitResponse(5000L, GF("+MIPSTATE:")) != 1) {
      DBG("### MIPSTATE check failed");
      return false;
    }
    DBG("### MUX", mux, " connected to ", host, ":", port);
    sockets[mux]->sock_connected = true;
    return true;
  }

  int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
    if (!sockets[mux]->sock_connected) {
        DBG("### Socket ", mux, " not connected");
        return -1;
    }

    const uint8_t* data = reinterpret_cast<const uint8_t*>(buff);

    String dataStr;
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) dataStr += '0';
        dataStr += String(data[i], HEX);
    }

    sendAT(GF("+MIPSEND="), mux, ',', (uint16_t)len, ",\"", dataStr, "\"");

    if (waitResponse(15000L, GF("+MIPSEND:")) != 1) {
        DBG("### MIPSEND timeout or no response");
        return 0;
    }

    String muxStr = stream.readStringUntil(',');
    uint8_t resp_mux = muxStr.toInt();
    String lenStr = stream.readStringUntil('\n');
    int16_t confirmed = lenStr.toInt();
    waitResponse();

    DBG("### +MIPSEND: mux=", resp_mux, " confirmed=", confirmed, "/", len);
    return confirmed;
  }

  size_t modemRead(size_t size, uint8_t mux) {
    if (!sockets[mux] || !sockets[mux]->sock_connected) return 0;

    sendAT(GF("+MIPRD="), mux, ',', (uint16_t)size);
    if (waitResponse(GF("+MIPRD:")) != 1) {
      DBG("### MIPRD command failed for mux", mux);
      return 0;
    }

    //+MIPRD: <mux>,<unread_len>,<data_len>,<data>
    int resp_mux   = streamGetIntBefore(',');
    int unread_len = streamGetIntBefore(',');
    int data_len   = streamGetIntBefore(',');

    String hexData = stream.readStringUntil('\n');
    hexData.trim();
    waitResponse();

    DBG("### MIPRD HEX DATA:", hexData);

    for (int i = 0; i + 1 < hexData.length(); i += 2) {
      uint8_t b = strtoul(hexData.substring(i, i + 2).c_str(), NULL, 16);
      sockets[mux]->rx.put((char)b);
    }

    sockets[mux]->sock_available = unread_len;
    sockets[mux]->got_data = (unread_len > 0);
    return data_len;
  }

  size_t modemGetAvailable(uint8_t mux) {
    if (!sockets[mux]) return 0;
    if (!sockets[mux]->sock_connected) return 0;

    sendAT(GF("+MIPRD="), mux);
    if (waitResponse(GF("+MIPRD:")) == 1) {
      int resp_mux = streamGetIntBefore(',');
      int unread_len = streamGetIntBefore('\n');
      waitResponse();

      DBG("### MIPRD: mux=", resp_mux, " unread_len=", unread_len);
      sockets[mux]->sock_available = (unread_len > 0) ? unread_len : 0;
      sockets[mux]->got_data = (unread_len > 0);
      return sockets[mux]->sock_available;
    }
    waitResponse();
    return 0;
  }

  bool modemGetConnected(uint8_t mux) {
    if (!sockets[mux]) return false;
    sendAT(GF("+MIPSTATE="), mux);
      
    int8_t response = waitResponse(5000L, GF("+MIPSTATE:"));
    
    if (response != 1) {
      DBG("### Failed to get MIPSTATE response for mux=", mux);
      sockets[mux]->sock_connected = false;
      return false;
    }

    uint8_t resp_mux = streamGetIntBefore(',');
    String remaining = stream.readStringUntil('\n');
    remaining.trim();
    
    DBG("### MIPSTATE full response for mux", mux, ":", remaining);

    bool connected = (remaining.indexOf("CONNECTED") >= 0);
    
    if (remaining.indexOf("CLOSED") >= 0 || 
        remaining.indexOf("INITIAL") >= 0 ||
        remaining == ",,,") {
      connected = false;
    }

    sockets[mux]->sock_connected = connected;
    
    DBG("### Connection status for mux", mux, ":", connected ? "CONNECTED" : "DISCONNECTED");
    
    waitResponse();
    return connected;
  }

  /*
   * Utilities - URC handler
   */
public:
  bool handleURCs(String& data) {
    if (data.endsWith(GF(AT_NL "+MIPURC:"))) {
      streamSkipUntil('"');
      String urc = stream.readStringUntil('"');
      streamSkipUntil(',');
      int8_t mux = streamGetIntBefore(',');
      DBG("### URC:", urc, " mux:", mux);

      if (urc == "rtcp") {
        if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
          sockets[mux]->got_data = true;
          String lenStr = stream.readStringUntil(',');
          int16_t length = lenStr.toInt();
          sockets[mux]->sock_available = length;
          DBG("### RTCP Data arrived:", length, "bytes on MUX", mux);
          String extra = stream.readStringUntil('\n');
          if (extra.length() > 0) {
            DBG("### Extra data after length:", extra);
          }
        } else {
          streamSkipUntil('\n');
        }
      } else if (urc == "rudp") {
        if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux])
          sockets[mux]->got_data = true;
        streamSkipUntil('\n');
      } else if (urc == "disconn") {
         if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
          sockets[mux]->sock_connected = false;
          sockets[mux]->sock_available = 0;
          DBG("### Connection disconnected via URC, mux=", mux);
          
          
          sendAT(GF("+MIPCLOSE="), mux);
          waitResponse(10000);
        }
        streamSkipUntil('\n');
      }
      else if (urc == "ack") {
        streamSkipUntil('\n');
      } else if (urc == "drop") {
        streamSkipUntil('\n');
      } else if (urc.indexOf("pdpdeact") >= 0) {
        for (auto& s : sockets) if (s) {
          s->sock_connected = false;
          s->sock_available = 0;
        }
      } else {
        streamSkipUntil('\n');
      }

      data = "";
      return true;
    }
    return false;
  }

 public:
  Stream& stream;

 protected:
  GsmClientML307* sockets[TINY_GSM_MUX_COUNT];
  String         certificates[TINY_GSM_MUX_COUNT];
  void moveCharFromStreamToFifo(uint8_t mux) {
    if (!sockets[mux]) return;
    char c = stream.read();
    sockets[mux]->rx.put(c);
  }
};

#endif  // SRC_TINYGSMCLIENTML307_H_
