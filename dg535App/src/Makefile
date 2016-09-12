TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

PROD_IOC = dg535

DBD += dg535.dbd
dg535_DBD += base.dbd
dg535_DBD += asyn.dbd
dg535_DBD += drvDG535.dbd
dg535_DBD += drvVxi11.dbd

dg535_SRCS += dg535_registerRecordDeviceDriver.cpp
dg535_SRCS += drvDG535.cpp
dg535_SRCS_DEFAULT += dg535Main.cpp

dg535_LIBS += asyn

dg535_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
