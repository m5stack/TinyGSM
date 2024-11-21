/**
 * @file       TinyGsmClientSIM7028.h
 * @author      junhuang
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2024 M5Stack Technology CO LTD
 * @date       May 2024
 */

#ifndef SRC_TINYGSMCLIENTSIM7028_H_
#define SRC_TINYGSMCLIENTSIM7028_H_
// #pragma message("TinyGSM:  TinyGsmClientSIM7028")

//#define TINY_GSM_DEBUG Serial
// #define TINY_GSM_USE_HEX

#ifdef __AVR__
#define TINY_GSM_RX_BUFFER 32
#else
#define TINY_GSM_RX_BUFFER 1024
#endif

#if !defined(TINY_GSM_YIELD_MS)
#define TINY_GSM_YIELD_MS 0
#endif

#define TINY_GSM_MUX_COUNT 1
#define TINY_GSM_BUFFER_READ_AND_CHECK_SIZE

#include "TinyGsmModem.tpp"
#include "TinyGsmTCP.tpp"
#include "TinyGsmTime.tpp"
//#define MODE_NB_IOT      //Comment this macro definition when using CAT mode
#ifdef MODE_NB_IOT
    #include "TinyGsmNBIOT.tpp"
#else
   #include "TinyGsmGPRS.tpp"
#endif
#define GSM_NL "\r\n"
// static const char GSM_OK[] TINY_GSM_PROGMEM    = "OK" GSM_NL;
// static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;
#if defined       TINY_GSM_DEBUG
static const char GSM_CME_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CME ERROR:";
static const char GSM_CMS_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CMS ERROR:";
#endif

enum RegStatus
{
    REG_NO_RESULT    = -1,
    REG_UNREGISTERED = 0,
    REG_SEARCHING    = 2,
    REG_DENIED       = 3,
    REG_OK_HOME      = 1,
    REG_OK_ROAMING   = 5,
    REG_OK_SMS       = 6,
    REG_OK_SMS_ROAMING = 7,
    REG_UNKNOWN      = 4,
};
class TinyGsmSim7028 :  public TinyGsmModem<TinyGsmSim7028>,
                        #ifdef MODE_NB_IOT
                            public TinyGsmNBIOT<TinyGsmSim7028>,
                        #else
                            public TinyGsmGPRS<TinyGsmSim7028>,
                        #endif
                        public TinyGsmTCP<TinyGsmSim7028, TINY_GSM_MUX_COUNT>,
                        public TinyGsmTime<TinyGsmSim7028> {
    friend class TinyGsmModem<TinyGsmSim7028>;
    #ifdef MODE_NB_IOT
        friend class TinyGsmNBIOT<TinyGsmSim7028>;
    #else
        friend class TinyGsmGPRS<TinyGsmSim7028>;
    #endif
    friend class TinyGsmTCP<TinyGsmSim7028, TINY_GSM_MUX_COUNT>;
    friend class TinyGsmTime<TinyGsmSim7028>;

    /*
     * Inner Client
     */
  public:
    class GsmClientSim7028 : public GsmClient {
        friend class TinyGsmSim7028;

      public:
        GsmClientSim7028() {}

        explicit GsmClientSim7028(TinyGsmSim7028 &modem, uint8_t mux = 0) { init(&modem, mux); }

        bool init(TinyGsmSim7028 *modem, uint8_t mux = 0)
        {
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

      public:
        virtual int connect(const char *host, uint16_t port, int timeout_s)
        {
            stop();
            TINY_GSM_YIELD();
            rx.clear();
            sock_connected = at->modemConnect(host, port, mux, timeout_s);
            return sock_connected;
        }
        TINY_GSM_CLIENT_CONNECT_OVERRIDES

        void stop(uint32_t maxWaitMs) {
            dumpModemBuffer(maxWaitMs);
            at->sendAT(GF("+CIPCLOSE="), mux);
            sock_connected = false;
            at->waitResponse();
        }
        void stop() override {
            stop(15000L);
        }


        /*
         * Extended API
         */

        String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    };

    /*
     * Inner Secure Client
     */
    // TODO: SSL Client
    /*
     * Constructor
     */
  public:
    explicit TinyGsmSim7028(Stream &stream, uint8_t reset_pin=-1) : stream(stream), reset_pin(reset_pin) { memset(sockets, 0, sizeof(sockets)); }

    /*
     * Basic functions
     */
  protected:
    bool initImpl(const char *pin = NULL)
    {
        restart();

        DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
        DBG(GF("### TinyGSM Compiled Module:  TinyGsmClientSIM7028"));

        if (!testAT()) {
            return false;
        }

        sendAT(GF("E0"));     // Echo Off
        if (waitResponse() != 1) {
            return false;
        }
        sendAT("+CFUN=1");
        waitResponse();
        sendAT("+QCLEDMODE=1");
        waitResponse();

#ifdef TINY_GSM_DEBUG
        sendAT(GF("+CMEE=2"));     // turn on verbose error codes
#else
        sendAT(GF("+CMEE=0"));     // turn off error codes
#endif
        waitResponse();

        DBG(GF("### Modem:"), getModemName());
        // Save config
        // sendAT(GF("&w"));
        // waitResponse();
        // Disable time and time zone URC's
        sendAT(GF("+CTZR=0"));
        if (waitResponse(10000L) != 1) { return false; }

        // Enable automatic time zome update
        sendAT(GF("+CTZU=1"));
        if (waitResponse(10000L) != 1) { return false; }
        SimStatus ret = getSimStatus();
        // if the sim isn't ready and a pin has been provided, try to unlock the sim
        if (ret != SIM_READY && pin != NULL && strlen(pin) > 0) {
            simUnlock(pin);
            return (getSimStatus() == SIM_READY);
        } else {
            // if the sim is ready, or it's locked but no pin has been provided,
            // return true
            return (ret == SIM_READY || ret == SIM_LOCKED);
        }
    }

    String getModemNameImpl()
    {
        sendAT(GF("+CGMI"));
        String res1;
        if (waitResponse(1000L, res1) != 1) {
            return "unknown";
        }
        res1.replace("\r\nOK\r\n", "");
        res1.replace("\rOK\r", "");
        res1.trim();

        sendAT(GF("+CGMM"));
        String res2;
        if (waitResponse(1000L, res2) != 1) {
            return "unknown";
        }
        res2.replace("\r\nOK\r\n", "");
        res2.replace("\rOK\r", "");
        res2.trim();

        String name = res1 + String(' ') + res2;
        DBG("### Modem:", name);
        return name;
    }

    bool factoryDefaultImpl()
    {
        return false;
    }

    /*
     * Power functions
     */
  protected:
    bool restartImpl(const char* pin = NULL)
    {
        /* Hardware Reset */
        pinMode(this->reset_pin, OUTPUT);
        digitalWrite(this->reset_pin, LOW);
        delay(300);
        digitalWrite(this->reset_pin, HIGH);
        delay(5000);

        return true;
    }

    bool powerOffImpl()
    {
        sendAT(GF("+CPSMS=1"));
        return waitResponse(10000L, GF("OK")) == 1;
    }

    bool sleepEnableImpl()
    {
        sendAT(GF("+QCSIMSLEEP="), 1);
        return waitResponse() == 1;
    }

    /*
     * Generic network functions
     */
  public:
    RegStatus getRegistrationStatus() { 
        #ifdef MODE_NB_IOT
            sendAT(GF("+CIPCCFG=10,0,0,0,1,0,25000"));
            waitResponse();
            sendAT(GF("+NETOPEN"));
            waitResponse();
        #else
        
        #endif   
        return (RegStatus)getRegistrationStatusXREG("CREG"); 
    }

  protected:
    bool isNetworkConnectedImpl()
    {
        RegStatus s = getRegistrationStatus();
        return (s == REG_OK_HOME || s == REG_OK_ROAMING || s == REG_OK_SMS || s == REG_OK_SMS_ROAMING);
    }

    String getLocalIPImpl() {
        sendAT(GF("+IPADDR"));  // Inquire Socket PDP address
        // sendAT(GF("+CGPADDR=1"));  // Show PDP address
        String res;
        if (waitResponse(10000L, res) != 1) { return ""; }
        res.replace("+IPADDR:", "");
        res.trim();
        return res;
    }


    /*
     * GPRS functions
     */
  protected:
    // No functions of this type supported
        bool gprsConnectImpl(const char* apn, const char* user = NULL,
                       const char* pwd = NULL) {
    #ifdef MODE_NB_IOT
        
    #else
        gprsDisconnect();  // Make sure we're not connected first
    #endif           

    // Define the PDP context

    // The CGDCONT commands set up the "external" PDP context

    // Set the external authentication
    if (user && strlen(user) > 0) {
      sendAT(GF("+CGAUTH=1,0,\""), user, GF("\",\""), pwd, '"');
      waitResponse();
    }

    // Define external PDP context 1
    sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"', ",\"0.0.0.0\",0,0");
    waitResponse();

    // Configure TCP parameters

    // Select TCP/IP application mode (command mode)
    sendAT(GF("+CIPMODE=0"));
    waitResponse();

    // Set Sending Mode - send without waiting for peer TCP ACK
    sendAT(GF("+CIPSENDMODE=0"));
    waitResponse();

    // Configure socket parameters
    // AT+CIPCCFG= <NmRetry>, <DelayTm>, <Ack>, <errMode>, <HeaderType>,
    //            <AsyncMode>, <TimeoutVal>
    // NmRetry = number of retransmission to be made for an IP packet
    //         = 10 (default)
    // DelayTm = number of milliseconds to delay before outputting received data
    //          = 0 (default)
    // Ack = sets whether reporting a string "Send ok" = 0 (don't report)
    // errMode = mode of reporting error result code = 0 (numberic values)
    // HeaderType = which data header of receiving data in multi-client mode
    //            = 1 (+RECEIVE,<link num>,<data length>)
    // AsyncMode = sets mode of executing commands
    //           = 0 (synchronous command executing)
    // TimeoutVal = minimum retransmission timeout in milliseconds = 75000
    sendAT(GF("+CIPCCFG=10,0,0,0,1,0,75000"));
    if (waitResponse() != 1) { return false; }

    // Configure timeouts for opening and closing sockets
    // AT+CIPTIMEOUT=<netopen_timeout> <cipopen_timeout>, <cipsend_timeout>
    sendAT(GF("+CIPTIMEOUT="), 75000, ',', 15000, ',', 15000);
    waitResponse();

    // Start the socket service

    // This activates and attaches to the external PDP context that is tied
    // to the embedded context for TCP/IP (ie AT+CGACT=1,1 and AT+CGATT=1)
    // Response may be an immediate "OK" followed later by "+NETOPEN: 0".
    // We to ignore any immediate response and wait for the
    // URC to show it's really connected.
    sendAT(GF("+NETOPEN"));
    if (waitResponse(75000L, GF(GSM_NL "+NETOPEN: 0")) != 1) { return false; }

    return true;
  }

  bool gprsDisconnectImpl() {
    // Close all sockets and stop the socket service
    // Note: On the LTE models, this single command closes all sockets and the
    // service
    sendAT(GF("+NETCLOSE"));
    if (waitResponse(60000L, GF(GSM_NL "+NETCLOSE: 0")) != 1) { return false; }

    return true;
  }

  bool isGprsConnectedImpl() {
    sendAT(GF("+NETOPEN?"));
    // May return +NETOPEN: 1, 0.  We just confirm that the first number is 1
    if (waitResponse(GF(GSM_NL "+NETOPEN: 1")) != 1) { return false; }
    waitResponse();

    sendAT(GF("+IPADDR"));  // Inquire Socket PDP address
    // sendAT(GF("+CGPADDR=1")); // Show PDP address
    if (waitResponse() != 1) { return false; }

    return true;
  }
    /*
     * NBIOT functions
     */
  protected:
    bool nbiotConnectImpl(const char *apn, uint8_t band = 0)
    {
        // Set APN  
        // Define the PDP context
        sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"');
        waitResponse(); 
        // Set Band
        printf("nbiotConnectImpl\n");
        sendAT("+QCBAND=", band);
        if (waitResponse() != 1) {
            return false;
        }
        return true;
    }

    /*
     * SIM card functions
     */
  protected:
    // May not return the "+CCID" before the number
    String getSimCCIDImpl()
    {
        sendAT(GF("+CICCID"));
        if (waitResponse(GF(GSM_NL)) != 1) {
            return "";
        }
        String res = stream.readStringUntil('\n');
        waitResponse();
        // Trim out the CICCID header in case it is there
        res.replace("CICCID:", "");
        res.trim();
        return res;
    }

    /*
     * Phone Call functions
     */
  public:
    /*
     * Messaging functions
     */
  protected:
    // Follows all messaging functions per template

    /*
     * GSM Location functions
     */
  protected:
    /*
     * GPS/GNSS/GLONASS location functions
     */
  protected:
    // No functions of this type supported

    /*
     * Time functions
     */
  protected:
    // Can follow the standard CCLK function in the template

    /*
     * Battery functions
     */
  protected:
    // Follows all battery functions per template

    /*
     * NTP server functions
     */
  public:
    boolean isValidNumber(String str)
    {
        if (!(str.charAt(0) == '+' || str.charAt(0) == '-' || isDigit(str.charAt(0))))
            return false;

        for (byte i = 1; i < str.length(); i++) {
            if (!(isDigit(str.charAt(i)) || str.charAt(i) == '.')) {
                return false;
            }
        }
        return true;
    }
     bool NTPServerSync(String server = "pool.ntp.org", byte TimeZone = 32)
    {
        // AT+CURTC Control CCLK Show UTC Or RTC Time
        // Use AT CCLK? command to get UTC Or RTC Time
        // Start to query network time
        // AT+CNTP="120.25.115.20",32 // set the NTP server and local time zone
        sendAT(GF("+CNTP=") , server,TimeZone);
        if (waitResponse(50000L) != 1) {
            return false;
        }

      
    }
   
    /*
     * Client related functions
     */
  protected:
    bool modemConnect(const char* host, uint16_t port, uint8_t mux,
                    bool ssl = false, int timeout_s = 15) {
        if (ssl) { DBG("SSL not yet supported on this module!"); }
        // Make sure we'll be getting data manually on this connection
        sendAT(GF("+CIPRXGET=1"));
        if (waitResponse() != 1) { return false; }

        // Establish a connection in multi-socket mode
        uint32_t timeout_ms = ((uint32_t)timeout_s) * 1000;
        sendAT(GF("+CIPOPEN="), mux, ',', GF("\"TCP"), GF("\",\""), host, GF("\","),
            port);
        // The reply is OK followed by +CIPOPEN: <link_num>,<err> where <link_num>
        // is the mux number and <err> should be 0 if there's no error
        if (waitResponse(timeout_ms, GF(GSM_NL "+CIPOPEN:")) != 1) { return false; }
        uint8_t opened_mux    = streamGetIntBefore(',');
        uint8_t opened_result = streamGetIntBefore('\n');
        if (opened_mux != mux || opened_result != 0) return false;
        return true;
    }

    int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
        sendAT(GF("+CIPSEND="), mux, ',', (uint16_t)len);
        if (waitResponse(GF(">")) != 1) { return 0; }
        stream.write(reinterpret_cast<const uint8_t*>(buff), len);
        stream.flush();
        if (waitResponse(GF(GSM_NL "+CIPSEND:")) != 1) { return 0; }
        streamSkipUntil(',');  // Skip mux
        streamSkipUntil(',');  // Skip requested bytes to send
        // TODO(?):  make sure requested and confirmed bytes match
        return streamGetIntBefore('\n');
    }

    size_t modemRead(size_t size, uint8_t mux) {
    if (!sockets[mux]) return 0;
#ifdef TINY_GSM_USE_HEX
    sendAT(GF("+CIPRXGET=3,"), mux, ',', (uint16_t)size);
    if (waitResponse(GF("+CIPRXGET:")) != 1) { return 0; }
#else
    sendAT(GF("+CIPRXGET=2,"), mux, ',', (uint16_t)size);
    if (waitResponse(GF("+CIPRXGET:")) != 1) { return 0; }
#endif
    streamSkipUntil(',');  // Skip Rx mode 2/normal or 3/HEX
    streamSkipUntil(',');  // Skip mux/cid (connecion id)
    int16_t len_requested = streamGetIntBefore(',');
    //  ^^ Requested number of data bytes (1-1460 bytes)to be read
    int16_t len_confirmed = streamGetIntBefore('\n');
    // ^^ The data length which not read in the buffer
    for (int i = 0; i < len_requested; i++) {
      uint32_t startMillis = millis();
#ifdef TINY_GSM_USE_HEX
      while (stream.available() < 2 &&
             (millis() - startMillis < sockets[mux]->_timeout)) {
        TINY_GSM_YIELD();
      }
      char buf[4] = {
          0,
      };
      buf[0] = stream.read();
      buf[1] = stream.read();
      char c = strtol(buf, NULL, 16);
#else
      while (!stream.available() &&
             (millis() - startMillis < sockets[mux]->_timeout)) {
        TINY_GSM_YIELD();
      }
      char c = stream.read();
#endif
      sockets[mux]->rx.put(c);
    }
    // DBG("### READ:", len_requested, "from", mux);
    // sockets[mux]->sock_available = modemGetAvailable(mux);
    sockets[mux]->sock_available = len_confirmed;
    waitResponse();
    return len_requested;
  }

  size_t modemGetAvailable(uint8_t mux) {
    if (!sockets[mux]) return 0;
    sendAT(GF("+CIPRXGET=4,"),mux);
    size_t result = 0;
    if (waitResponse(GF("+CIPRXGET:")) == 1) {
      streamSkipUntil(',');  // Skip mode 4
      streamSkipUntil(',');  // Skip mux
      result = streamGetIntBefore('\n');
      waitResponse();
    }
    // DBG("### Available:", result, "on", mux);
    if (!result) { sockets[mux]->sock_connected = modemGetConnected(mux); }
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    // Read the status of all sockets at once
    sendAT(GF("+CIPCLOSE?"));
    if (waitResponse(GF("+CIPCLOSE:")) != 1) {
         return false;  // TODO:  Why does this not read correctly?
    }
    for (int muxNo = 0; muxNo < TINY_GSM_MUX_COUNT; muxNo++) {
      // +CIPCLOSE:<link0_state>,<link1_state>,...,<link9_state>
      bool muxState = stream.parseInt();
      if (sockets[muxNo]) { sockets[muxNo]->sock_connected = muxState; }
    }
    waitResponse();  // Should be an OK at the end
    if (!sockets[mux]) return false;
    return sockets[mux]->sock_connected;
  }

    /*
     * Utilities
     */
  public:
    int8_t waitResponse(uint32_t timeout_ms, String &data, GsmConstStr r1 = GFP(GSM_OK), GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                        GsmConstStr r3 = GFP(GSM_CME_ERROR), GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                        GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                        GsmConstStr r5 = NULL)
    {
        data.reserve(64);
        uint8_t  index       = 0;
        uint32_t startMillis = millis();
        do {
            TINY_GSM_YIELD();
            while (stream.available() > 0) {
                TINY_GSM_YIELD();
                int8_t a = stream.read();
                if (a <= 0)
                    continue;     // Skip 0x00 bytes, just in case
                data += static_cast<char>(a);
                if (r1 && data.endsWith(r1)) {
                    index = 1;
                    goto finish;
                } else if (r2 && data.endsWith(r2)) {
                    index = 2;
                    goto finish;
                } else if (r3 && data.endsWith(r3)) {
#if defined TINY_GSM_DEBUG
                    if (r3 == GFP(GSM_CME_ERROR)) {
                        streamSkipUntil('\n');     // Read out the error
                    }
#endif
                    index = 3;
                    goto finish;
                } else if (r4 && data.endsWith(r4)) {
                    index = 4;
                    goto finish;
                } else if (r5 && data.endsWith(r5)) {
                    index = 5;
                    goto finish;
                } else if (data.endsWith(GF(GSM_NL "+CIPRXGET:"))) {
                    int8_t mode = streamGetIntBefore(',');
                    if (mode == 1) {
                        int8_t mux = 0;
                        if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
                            sockets[mux]->got_data = true;
                        }
                        data = "";
                        // DBG("### Got Data:", mux);
                    } else {
                        data += mode;
                    }
                }else if (data.endsWith(GF(GSM_NL "+RECEIVE:"))) {
                int8_t  mux = streamGetIntBefore(',');
                int16_t len = streamGetIntBefore('\n');
                if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
                    sockets[mux]->got_data = true;
                    if (len >= 0 && len <= 1024) { sockets[mux]->sock_available = len; }
                }
                data = "";
                // DBG("### Got Data:", len, "on", mux);
                } else if (data.endsWith(GF("+IPCLOSE:"))) {
                int8_t mux = streamGetIntBefore(',');
                streamSkipUntil('\n');  // Skip the reason code
                if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
                    sockets[mux]->sock_connected = false;
                }
                data = "";
                DBG("### Closed: ", mux);
                } else if (data.endsWith(GF("+CIPEVENT:"))) {
                // Need to close all open sockets and release the network library.
                // User will then need to reconnect.
                DBG("### Network error!");
                #ifdef MODE_NB_IOT
                #else
                    if (!isGprsConnected()) { gprsDisconnect(); }
                #endif    
                data = "";
                }
            }
            } while (millis() - startMillis < timeout_ms);
    finish:
        if (!index) {
            data.trim();
            if (data.length()) {
                DBG("### Unhandled:", data);
            }
            data = "";
        }
        // data.replace(GSM_NL, "/");
        // DBG('<', index, '>', data);
        return index;
    }

    int8_t waitResponse(uint32_t timeout_ms, GsmConstStr r1 = GFP(GSM_OK), GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                        GsmConstStr r3 = GFP(GSM_CME_ERROR), GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                        GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                        GsmConstStr r5 = NULL)
    {
        String data;
        return waitResponse(timeout_ms, data, r1, r2, r3, r4, r5);
    }

    int8_t waitResponse(GsmConstStr r1 = GFP(GSM_OK), GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                        GsmConstStr r3 = GFP(GSM_CME_ERROR), GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                        GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                        GsmConstStr r5 = NULL)
    {
        return waitResponse(1000, r1, r2, r3, r4, r5);
    }

  public:
    Stream &      stream;
    uint8_t       reset_pin;
    unsigned long baud;

  protected:
    GsmClientSim7028 *sockets[TINY_GSM_MUX_COUNT];
    const char *      gsmNL = GSM_NL;
};

#endif     // SRC_TINYGSMCLIENTSIM7028_H_
