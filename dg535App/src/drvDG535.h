/*
drvDG535.h
*/

#include "asynPortDriver.h"

#define CMD_BUF_LEN 256

#define trigRateString      "TRIG_RATE"   /* asynFloat64,    r/w */
#define trigModeString      "TRIG_MODE"   /* asynInt32,      r/w */
#define trigSlopeString     "TRIG_SLOPE"  /* asynInt32,      r/w */
#define trigLevelString     "TRIG_LEVEL"  /* asynFloat64,    r/w */
#define errStatString       "ERR_STAT"    /* asynInt32,      r   */
#define instStatString      "INST_STAT"   /* asynInt32,      r   */
#define impedString         "IMPED"       /* asynInt32,      r/w */
#define outModeString       "OUT_MODE"    /* asynInt32,      r/w */
#define outPolarityString   "OUT_POL"     /* asynInt32,      r/w */
#define outAmplitudeString  "OUT_AMP"     /* asynInt32,      r/w */
#define outOffsetString     "OUT_OFF"     /* asynInt32,      r/w */
#define delayString         "DELAY"       /* asynInt32,      r/w */
#define clearString         "CLEAR"       /* asynInt32,      w   */
#define singleShotString    "SINGLE"      /* asynInt32,      w   */
#define burstRateString     "BURST_RATE"  /* asynFloat64,    r/w */
#define burstCountString    "BURST_CNT"   /* asynInt32,      r/w */
#define burstPeriodString   "BURST_PER"   /* asynInt32,      r/w */
#define storeString         "STORE"       /* asynInt32,      w   */
#define recallString        "RECALL"      /* asynInt32,      w   */
#define displayStrString    "DSP_STR"     /* asynOctet,      w   */

class drvDG535 : public asynPortDriver{
public:
    drvDG535(const char* port, const char* udp, int addr, int nchan, double timeout);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);
    virtual void pollerThread(void);

protected:
    asynStatus _write(const char *buffer);
    asynStatus _writeRead(const char *buffer);
    void _readInt(int function, const char *cmd, int addr);
    void _readDouble(int function, const char *cmd, int addr);
    void _readDelay(int addr);

    int trigRate;
    int trigMode;
    int trigSlope;
    int trigLevel;
    int errStat;
    int instStat;
    int imped;
    int outMode;
    int outPol;
    int outAmp;
    int outOff;
    int delay;
    int clear;
    int single;
    int burstRate;
    int burstCnt;
    int burstPer;
    int store;
    int recall;
    int displayStr;
    #define FIRST_COMMAND trigRate
    #define LAST_COMMAND displayStr

private:
    char cmdBuffer_[CMD_BUF_LEN];
    double pollTime_;
    int forceCallback_;
    int nchans_;
    double timeout_;
};

#define N_PARAMS ((int)(&LAST_COMMAND - &FIRST_COMMAND + 1))


