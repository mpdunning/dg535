#!../../bin/linux-x86_64/dg535

< envPaths
epicsEnvSet("P",         "ESB:DG01")
epicsEnvSet("ETHER",     "esb-gpib-03")
epicsEnvSet("GPIB_ADDR", "13")
epicsEnvSet("DESC",      "ESB TEST")
epicsEnvSet("PORT",      "DG535")

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/dg535.dbd"
dg535_registerRecordDeviceDriver pdbbase

vxi11Configure("DG", $(ETHER), 0, 0.0, "gpib0", 0, 0)


# drvDG535Configure(const char* port, const char* udp, int addr, int nchan, double timeout){
#/*------------------------------------------------------------------------------
# * EPICS iocsh callable function to call constructor for the drvDG535 class.
# *  port    The name of the asyn port driver to be created.
# *  udp     The communication port name.
# *  addr    The hardware addreess.
# *  nchan   The number of active (defined) channels.
# *  timeout The timeout for I/O.
# *----------------------------------------------------------------------------*/
drvDG535Configure("$(PORT)", "DG", $(GPIB_ADDR), 8, 1);

#asynSetTraceMask("DG", $(GPIB_ADDR), 0x9)
asynSetTraceIOMask("DG", $(GPIB_ADDR), 0x6)

dbLoadRecords("db/dg535.db", "P=$(P), DESC=$(DESC), PORT=$(PORT)")
#dbLoadRecords("db/timestamp.db", "P=$(P)")
dbLoadRecords("db/asynRecord.db", "P=$(P):, R=asyn, PORT=DG, ADDR=$(GPIB_ADDR), OMAX=0, IMAX=0")

cd ${TOP}/iocBoot/${IOC}

iocInit

