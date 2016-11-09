/*
drvDG535.cpp
*/

#include <iostream>     // std::cout
#include <cstdlib>
#include <cstring>

#include <epicsThread.h>
#include <epicsExport.h>
#include <asynOctetSyncIO.h>
#include <iocsh.h>

#include "drvDG535.h"

const int MAX_CHAN = 8;
const double DEFAULT_TIMEOUT = 1.0;
const double DEFAULT_POLL_TIME = 2.0;

const double TRIG_RATE_MIN = 1e-3;
const double TRIG_RATE_MAX = 1e6;
const double TRIG_LEVEL_MIN = -2.5;
const double TRIG_LEVEL_MAX = 2.5;
const double OUT_AMP_MIN = -3.0;
const double OUT_AMP_MAX = 4.0;
const double OUT_AMP_STEP = 0.1;
const int BURST_CNT_MIN = 2;
const int BURST_CNT_MAX = 32766;
const int BURST_PER_MIN = 4;
const int BURST_PER_MAX = 32766;

static const char *driverName = "drvDG535";

static void pollerThreadC(void * pPvt) {
    drvDG535 *pdrvDG535 = (drvDG535 *)pPvt;
    pdrvDG535->pollerThread();
}

/* Constructor for the drvDG535 class */
drvDG535::drvDG535(const char *port, const char* udp, int addr, double timeout) 
   : asynPortDriver(port, MAX_CHAN, N_PARAMS,
                    asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags  */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/    
                    pollTime_(DEFAULT_POLL_TIME),
                    forceCallback_(1),
                    timeout_(timeout)
{
    const char *functionName = "drvDG535";
    asynStatus status;
    
    if (!timeout_) timeout_  = DEFAULT_TIMEOUT;

    status = pasynOctetSyncIO->connect(udp, addr, &pasynUser, 0);
    
    if (status != asynSuccess) {
        printf("%s::%s:connect: failed to connect to port %s\n", driverName, functionName, udp);
    }
    else {
        printf("%s::%s:connect: connected to port %s\n", driverName, functionName, udp);
    }

    createParam(trigRateString,                asynParamFloat64,         &trigRate);
    createParam(trigModeString,                asynParamInt32,           &trigMode);
    createParam(trigSlopeString,               asynParamInt32,           &trigSlope);
    createParam(trigLevelString,               asynParamFloat64,         &trigLevel);
    createParam(errStatString,                 asynParamInt32,           &errStat);
    createParam(instStatString,                asynParamInt32,           &instStat);
    createParam(impedString,                   asynParamInt32,           &imped);
    createParam(outModeString,                 asynParamInt32,           &outMode);
    createParam(outPolarityString,             asynParamInt32,           &outPol);
    createParam(outAmplitudeString,            asynParamFloat64,         &outAmp);
    createParam(outOffsetString,               asynParamFloat64,         &outOff);
    createParam(delayString,                   asynParamFloat64,         &delay);
    createParam(clearString,                   asynParamInt32,           &clear);
    createParam(singleShotString,              asynParamInt32,           &single);
    createParam(burstRateString,               asynParamFloat64,         &burstRate);
    createParam(burstCountString,              asynParamInt32,           &burstCnt);
    createParam(burstPeriodString,             asynParamInt32,           &burstPer);
    createParam(storeString,                   asynParamInt32,           &store);
    createParam(recallString,                  asynParamInt32,           &recall);
    createParam(displayStrString,              asynParamOctet,           &displayStr);
    
    /* Start the thread to poll inputs and do callbacks to device support */
    epicsThreadCreate("drvDG535Poller",
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)pollerThreadC, this);
}

void drvDG535::pollerThread()
{
/* This function runs in a separate thread.  It waits for the poll time */
    //const char *functionName = "pollerThread";
    int i;

    while(1) { 
        lock();
        
        // Trig mode
        _readInt(trigMode, "TM", 0);

        // Trig slope
        _readInt(trigSlope, "TS", 0);

        // Error status
        _readInt(errStat, "ES", 0);

        // Instrument status
        _readInt(instStat, "IS", 0);
    
        // Impedance; Read all channels
        for(i=2; i<MAX_CHAN; i++) {
            sprintf(str_, "TZ %d", i);
            _readInt(imped, str_, i);
        }
        
        // Output mode; Skip Chan. 0
        for(i=1; i<MAX_CHAN; i++) {
            sprintf(str_, "OM %d", i);
            _readInt(outMode, str_, i);
        }
        
        // Output polarity; Read all channels
        for(i=0; i<MAX_CHAN; i++) {
        // Skip Chan. 0, 1, 4, 7
            if ((i == 2) || (i == 3) || (i == 5) || (i == 6)) {
                sprintf(str_, "OP %d", i);
                _readInt(outPol, str_, i);
            }
        }
        
        // Burst count
        _readInt(burstCnt, "BC", 0);
        // Burst period
        _readInt(burstPer, "BP", 0);

        // Trig rate
        _readDouble(trigRate, "TR 0", 0);
        // Trig level
        _readDouble(trigLevel, "TL", 0);
    
        // Output amplitude; Skip Chan. 0
        for(i=1; i<MAX_CHAN; i++) {
            sprintf(str_, "OA %d", i);
            _readDouble(outAmp, str_, i);
        }
        
        // Output offset; Skip Chan. 0
        for(i=1; i<MAX_CHAN; i++) {
            sprintf(str_, "OO %d", i);
            _readDouble(outOff, str_, i);
        }
       
        // Delay; Skip Chan. 0, 1, 4, 7
        for(i=0; i<MAX_CHAN; i++) {
            if ((i == 2) || (i == 3) || (i == 5) || (i == 6)) {
            _readDelay(i);
            }
        }
       
        // Burst rate 
        _readDouble(burstRate, "TR 1", 0);
        
        unlock();
        epicsThreadSleep(pollTime_);
    }
}

void drvDG535::_readInt(int function, const char *cmd, int addr) {
/*-----------------------------------------------------------------------------
 * Writes a string to device, reads response,
 * sets value in parameter library, does callback.
 *---------------------------------------------------------------------------*/
    const char *functionName = "_readInt";
    asynStatus status = asynSuccess; 
    int tempVal;
    
    status = _writeRead(cmd);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: ERROR: function=%d, status=%d, \n", 
            driverName, functionName, function, status);
    }
    tempVal = atoi(cmdBuffer_);
    
    //printf("%s::%s: status=%d, function=%d, cmd=%s, cmdBuffer_=%s\n", 
    //    driverName, functionName, status, function, cmd, cmdBuffer_);
    
    if (forceCallback_) setIntegerParam(addr, function, tempVal);
    callParamCallbacks(addr);
}
    
void drvDG535::_readDouble(int function, const char *cmd, int addr) {
/*-----------------------------------------------------------------------------
 * Writes a string to device, reads response,
 * sets value in parameter library, does callback.
 *---------------------------------------------------------------------------*/
    const char *functionName = "_readDouble";
    asynStatus status = asynSuccess; 
    double tempVal;
    
    status = _writeRead(cmd);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: ERROR: function=%d, status=%d, \n", 
            driverName, functionName, function, status);
    }
    tempVal = atof(cmdBuffer_);
    
    //printf("%s::%s: status=%d, function=%d, cmd=%s, cmdBuffer_=%s\n", 
    //    driverName, functionName, status, function, cmd, cmdBuffer_);
    
    if (forceCallback_) setDoubleParam(addr, function, tempVal);
    callParamCallbacks(addr);
}

void drvDG535::_readDelay(int addr) {
/*-----------------------------------------------------------------------------
 * Writes a string to device, reads response,
 * sets value in parameter library, does callback.
 *---------------------------------------------------------------------------*/
    const char *functionName = "_readDelay";
    asynStatus status = asynSuccess; 
    char *ptoken;
    double tempVal;
    int function = delay;

    sprintf(str_, "DT %d", addr);
    status = _writeRead(str_);
    
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: ERROR: function=%d, status=%d, \n", 
            driverName, functionName, function, status);
    }
    
    // Split string at comma and take what's after comma 
    ptoken = strchr(cmdBuffer_, ',');
    if(!ptoken) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
            "%s:%s, function=%d, ERROR reading from address %d, cmdBuffer_=%s\n",
            driverName, functionName, function, addr, cmdBuffer_);
    } else {
        tempVal = atof(ptoken + 1);
        if (forceCallback_) setDoubleParam(addr, function, tempVal);
        callParamCallbacks(addr);
    }
    
    //printf("%s::%s: status=%d, function=%d, addr=%d, cmdBuffer_=%s\n", 
    //    driverName, functionName, status, function, addr, cmdBuffer_);
}

asynStatus drvDG535::_write(const char *buffer) {
/*-----------------------------------------------------------------------------
 * Writes a string to device.
 *---------------------------------------------------------------------------*/
    const char *functionName = "_write";
    asynStatus status = asynSuccess; 
    size_t nbytesTransfered;

    status = pasynOctetSyncIO->flush(pasynUser);
    status = pasynOctetSyncIO->write(pasynUser, buffer, strlen(buffer), timeout_, &nbytesTransfered);

    //printf("%s::%s: status=%d, buffer=%s, nbytesTrans=%d\n", driverName, 
    //    functionName, status, buffer, (int)nbytesTransfered);

    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s::%s: Error: buffer=%s, status=%d, nbytesTransfered=%d\n",
          driverName, functionName, buffer, status, (int)nbytesTransfered);
    }
  
    return(status);
}

asynStatus drvDG535::_writeRead(const char *buffer) {
/*-----------------------------------------------------------------------------
 * Writes a string to device and reads response.
 *---------------------------------------------------------------------------*/
    const char *functionName = "_writeRead";
    asynStatus status = asynSuccess; 
    size_t nbytesOut, nbytesIn; 
    int eomReason;
  
    // pasynOctetSyncIO->writeRead calls flush, write, and read
    status = pasynOctetSyncIO->writeRead(pasynUser, buffer, strlen(buffer),
    cmdBuffer_, CMD_BUF_LEN, timeout_, &nbytesOut, &nbytesIn, &eomReason);
    
    //printf("%s::%s: buffer=%s, status=%d, nbytesOut=%d, cmdBuffer_=%s, nbytesIn=%d\n",
    //    driverName, functionName, buffer, status, (int)nbytesOut, cmdBuffer_, (int)nbytesIn);
  
    if((status != asynSuccess) || !nbytesIn || ((int)nbytesIn > CMD_BUF_LEN)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
          "%s::%s: Error: buffer=%s, status=%d, nbytesOut=%d, cmdBuffer_=%s, nbytesIn=%d\n",
          driverName, functionName, buffer, status, (int)nbytesOut, cmdBuffer_, (int)nbytesIn);
    }
    
    return(status);
}

asynStatus drvDG535::readInt32(asynUser *pasynUser, epicsInt32 *value) {
    const char *functionName = "readInt32";
    int addr;
    int function = pasynUser->reason;
    int status = 0;
    const char *paramName;

    getAddress(pasynUser, &addr);

    getParamName(function, &paramName);

    //printf("%s::%s: function=%d (%s), addr=%d\n", driverName, functionName, function, paramName, addr);

    //printf("paramName=%s, value=%d\n", paramName, *value);

    // Trig mode
    if (function == trigMode) {
        status = _writeRead("TM");
        *value = atoi(cmdBuffer_);
    // Trig slope
    } else if (function == trigSlope) {
        status = _writeRead("TS");
        *value = atoi(cmdBuffer_);
    // Error Status
    } else if (function == errStat) {
        status = _writeRead("ES");
        *value = atoi(cmdBuffer_);
    // Instrument Status
    } else if (function == instStat) {
        status = _writeRead("IS");
        *value = atoi(cmdBuffer_);
    // Impedance
    } else if (function == imped) {
        sprintf(str_, "TZ %d", addr);
        status = _writeRead(str_);
        *value = atoi(cmdBuffer_);
    // Output mode
    } else if (function == outMode) {
        sprintf(str_, "OM %d", addr);
        status = _writeRead(str_);
        *value = atoi(cmdBuffer_);
    // Polarity
    } else if (function == outPol) {
        sprintf(str_, "OP %d", addr);
        status = _writeRead(str_);
        *value = atoi(cmdBuffer_);
    // Burst count
    } else if (function == burstCnt) {
        status = _writeRead("BC");
        *value = atoi(cmdBuffer_);
    // Burst period
    } else if (function == burstPer) {
        status = _writeRead("BP");
        *value = atoi(cmdBuffer_);
    // Other functions we call the base class method
    } else {
       //status = asynPortDriver::readInt32(pasynUser, value);
    }
    
    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s::%s: ERROR: port %s, function=%d, addr=%d, status=%d\n",
        driverName, functionName, portName, function, addr, status);
    }

    setIntegerParam(addr, function, *value);
  
    callParamCallbacks();
    return (status==0) ? asynSuccess : asynError;
}

asynStatus drvDG535::readFloat64(asynUser *pasynUser, epicsFloat64 *value) {
    const char *functionName = "readFloat64";
    int addr;
    int function = pasynUser->reason;
    int status = 0;
    char *ptoken;
    const char *paramName;

    getAddress(pasynUser, &addr);

    getParamName(function, &paramName);
    
    //printf("%s::%s: function=%d (%s), addr=%d\n", driverName, functionName, function, paramName, addr);
    
    // Trig rate
    if (function == trigRate) {
        status = _writeRead("TR 0");
        *value = atof(cmdBuffer_);
    // Trig level
    } else if (function == trigLevel) {
        status = _writeRead("TL");
        *value = atof(cmdBuffer_);
    // Output amplitude
    } else if (function == outAmp) {
        sprintf(str_, "OA %d", addr);
        status = _writeRead(str_);
        *value = atof(cmdBuffer_);
    // Output offset
    } else if (function == outOff) {
        sprintf(str_, "OO %d", addr);
        status = _writeRead(str_);
        *value = atof(cmdBuffer_);
    // Delay
    } else if (function == delay) {
        sprintf(str_, "DT %d", addr);
        status = _writeRead(str_);
        ptoken = strchr(cmdBuffer_, ',');
        if(!ptoken) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                "%s:%s, ERROR reading from address %d, cmdBuffer_=%s\n",
                driverName, functionName, addr, cmdBuffer_);
            return(asynError);
        }
        *value = atof(ptoken + 1);
    // Burst rate
    } else if (function == burstRate) {
        status = _writeRead("TR 1");
        *value = atof(cmdBuffer_);
    } else {
        //status = asynPortDriver::readFloat64(pasynUser, value);
    }

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s::%s: ERROR: port %s, function=%d, addr=%d, status=%d\n",
        driverName, functionName, portName, function, addr, status);
    }
    
    setDoubleParam(addr, function, *value);
    
    callParamCallbacks(addr);
    return (status==0) ? asynSuccess : asynError;
}

asynStatus drvDG535::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    const char* functionName = "writeInt32";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr;
    int tempVal;
    const char *paramName;

    getAddress(pasynUser, &addr);
    
    getParamName(function, &paramName);
    
    //printf("%s::%s: function=%d (%s), addr=%d, value=%d\n", 
    //    driverName, functionName, function, paramName, addr, value);
    
    // Trig mode
    if (function == trigMode) {
        sprintf(str_, "TM %d", value);
        status = _write(str_);
    // Trig slope
    } else if (function == trigSlope) {
        sprintf(str_, "TS %d", value);
        status = _write(str_);
    // Impedance
    } else if (function == imped) {
        sprintf(str_, "TZ %d,%d", addr, value);
        status = _write(str_);
    // Output mode
    } else if (function == outMode) {
        sprintf(str_, "OM %d,%d", addr, value);
        status = _write(str_);
    // Polarity
    } else if (function == outPol) {
        sprintf(str_, "OP %d,%d", addr, value);
        status = _write(str_);
    // Clear instrument
    } else if (function == clear) {
        sprintf(str_, "CL");
        status = _write(str_);
    // Single-shot
    } else if (function == single) {
        sprintf(str_, "SS");
        status = _write(str_);
    // Burst count
    } else if (function == burstCnt) {
        if (value < BURST_CNT_MIN) value = BURST_CNT_MIN;
        if (value > BURST_CNT_MAX) value = BURST_CNT_MAX;
        sprintf(str_, "BC %d", value);
        status = _write(str_);
    // Burst period
    } else if (function == burstPer) {
        if (value < BURST_PER_MIN) value = BURST_PER_MIN;
        if (value > BURST_PER_MAX) value = BURST_PER_MAX;
        // Burst period must be >= burst count + 1
        getIntegerParam(burstCnt, &tempVal);
        if (value < (tempVal + 1)) value = (tempVal + 1);
        sprintf(str_, "BP %d", value);
        status = _write(str_);
    // Store settings
    } else if (function == store) {
        sprintf(str_, "ST %d", value);
        status = _write(str_);
    // Recall settings
    } else if (function == recall) {
        sprintf(str_, "RC %d", value);
        status = _write(str_);
    } else {
        //status = asynPortDriver::writeInt32(pasynUser, value);
    }
    
    /* Set the parameter in the parameter library. */
    status = (asynStatus)setIntegerParam(addr, function, value);
    
    status = (asynStatus)callParamCallbacks(addr);
    
    if (status) { 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                 "%s:%s, port %s, ERROR writing %d to address %d, status=%d\n",
                 driverName, functionName, portName, value, addr, status);
    } else {        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
                 "%s:%s, port %s, wrote %d to address %d\n",
                 driverName, functionName, portName, value, addr);
    }
    
    return status;
}

asynStatus drvDG535::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    const char* functionName = "writeFloat64";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr;
    double tempVal;
    const char *paramName;

    getAddress(pasynUser, &addr);
    
    getParamName(function, &paramName);
    
    //printf("%s::%s: function=%d (%s), addr=%d, value=%f\n", 
    //    driverName, functionName, function, paramName, addr, value);
    
    // Trig rate
    if (function == trigRate) {
        if (value < TRIG_RATE_MIN) value = TRIG_RATE_MIN;
        if (value > TRIG_RATE_MAX) value = TRIG_RATE_MAX;
        sprintf(str_, "TR 0,%f", value);
        status = _write(str_);
    // Trig level
    } else if (function == trigLevel) {
        if (value < TRIG_LEVEL_MIN) value = TRIG_LEVEL_MIN;
        if (value > TRIG_LEVEL_MAX) value = TRIG_LEVEL_MAX;
        sprintf(str_, "TL %f", value);
        status = _write(str_);
    // Output amplitude
    } else if (function == outAmp) {
        if (value > -OUT_AMP_STEP && value < OUT_AMP_STEP) value = OUT_AMP_STEP;
        if (value < OUT_AMP_MIN) value = OUT_AMP_MIN;
        if (value > OUT_AMP_MAX) value = OUT_AMP_MAX;
        getDoubleParam(addr, outOff, &tempVal);
        //printf("value=%f, outOff=%d, tempVal=%f\n", value, outOff, tempVal);
        if ((value + tempVal) < OUT_AMP_MIN) value = OUT_AMP_MIN - tempVal;
        if ((value + tempVal) > OUT_AMP_MAX) value = OUT_AMP_MAX - tempVal;
        sprintf(str_, "OA %d,%f", addr, value);
        status = _write(str_);
    // Output offset
    } else if (function == outOff) {
        if (value < OUT_AMP_MIN) value = OUT_AMP_MIN;
        if (value > OUT_AMP_MAX) value = OUT_AMP_MAX;
        getDoubleParam(addr, outAmp, &tempVal);
        if ((value + tempVal) < OUT_AMP_MIN) value = OUT_AMP_MIN - tempVal;
        if ((value + tempVal) > OUT_AMP_MAX) value = OUT_AMP_MAX - tempVal;
        sprintf(str_, "OO %d,%f", addr, value);
        status = _write(str_);
    // Delay
    } else if (function == delay) {
        //if (value < OUT_AMP_MIN) value = OUT_AMP_MIN;
        //if (value > OUT_AMP_MAX) value = OUT_AMP_MAX;
        sprintf(str_, "DT %d,1,%.12E", addr, value);
        status = _write(str_);
    // Burst rate
    } else if (function == burstRate) {
        if (value < TRIG_RATE_MIN) value = TRIG_RATE_MIN;
        if (value > TRIG_RATE_MAX) value = TRIG_RATE_MAX;
        sprintf(str_, "TR 1,%f", value);
        status = _write(str_);
    } else {
        //status = asynPortDriver::writeFloat64(pasynUser, value);
    }
    
    /* Set the parameter in the parameter library. */
    status = (asynStatus)setDoubleParam(addr, function, value);

    status = (asynStatus)callParamCallbacks(addr);
    
    if (status) { 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                 "%s:%s, port %s, ERROR writing %f to address %d, status=%d\n",
                 driverName, functionName, portName, value, addr, status);
    } else {        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
                 "%s:%s, port %s, wrote %f to address %d\n",
                 driverName, functionName, portName, value, addr);
    }
    
    return status;
}

asynStatus drvDG535::writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual) {
    const char* functionName = "writeOctet";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr;
    const char *paramName;

    getAddress(pasynUser, &addr);
    
    getParamName(function, &paramName);
    
    printf("%s::%s: function=%d (%s), addr=%d, value=%s, nChars=%d, nActual=%d\n", 
        driverName, functionName, function, paramName, addr, value, (int)nChars, (int)*nActual);
    
    // Display string
    if (function == displayStr) {
        sprintf(str_, "DS %s", value);
        status = _write(str_);
    } else {
        /* All other parameters just get set in parameter list, no need to
         * act on them here */
    }
    
    /* Set the parameter in the parameter library. */
    status = (asynStatus) setStringParam(addr, function, value);

    status = (asynStatus) callParamCallbacks(addr);
    
    if (status) { 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                 "%s:%s, port %s, ERROR writing %s to address %d, status=%d\n",
                 driverName, functionName, portName, value, addr, status);
    } else {        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
                 "%s:%s, port %s, wrote %s to address %d\n",
                 driverName, functionName, portName, value, addr);
    }
    
    return status;
}


// Configuration routine.  Called directly, or from the iocsh function below
extern "C" {

int drvDG535Configure(const char* port, const char* udp, int addr, double timeout) {
/*------------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvDG535 class.
 *  port    The name of the asyn port driver to be created.
 *  udp     The communication port name.
 *  addr    The hardware address.
 *  timeout The timeout for I/O.
 *----------------------------------------------------------------------------*/
    new drvDG535(port, udp, addr, timeout);
    return(asynSuccess);
}

static const iocshArg initArg0 = {"port", iocshArgString};
static const iocshArg initArg1 = {"udp", iocshArgString};
static const iocshArg initArg2 = {"addr", iocshArgInt};
static const iocshArg initArg3 = {"timeout", iocshArgDouble};
static const iocshArg* const initArgs[] = {&initArg0, &initArg1, &initArg2, &initArg3};
static const iocshFuncDef initFuncDef = {"drvDG535Configure", 4, initArgs};

static void initCallFunc(const iocshArgBuf *args) {
    drvDG535Configure(args[0].sval, args[1].sval, args[2].ival, args[3].dval);
}

void drvDG535Register(void){
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(drvDG535Register);
}



