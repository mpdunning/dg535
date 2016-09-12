#!../../bin/linux-x86_64/dg535

< envPaths
epicsEnvSet( "EPICS_CA_BEACON_ADDR_LIST","134.79.180.106")
epicsEnvSet( "ETHER","192.168.1.51")
#epicsEnvSet( "ETHER","192.168.1.11")
epicsEnvSet( "LOC","TEST:DG535")

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/dg535.dbd"
dg535_registerRecordDeviceDriver pdbbase
vxi11Configure( "DG",$(ETHER),0,0.0,"gpib0",0,0)

drvDG535Configure( "DG535","DG",15);

#asynSetTraceMask( "DG",15,0x9)
asynSetTraceIOMask( "DG",15,0x6)

dbLoadRecords("db/dg535.db","P=$(LOC),NAME=SRS-DG535,PORT=DG535")
dbLoadRecords( "db/timestamp.db","P=$(LOC)")

cd ${TOP}/iocBoot/${IOC}
iocInit

#dbpf( "$(LOC):AO:TRIG:RATE.DRVH",2000)
dbpf( "$(LOC):SI:NAME",$(IOC))
dbpf( "$(LOC):BO:READ:ERROR",0)
