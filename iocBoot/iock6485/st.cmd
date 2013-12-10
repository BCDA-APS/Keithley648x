#!../../bin/linux-x86_64/k6485

## You may have to change k6485 to something else
## everywhere it appears in this file

< envPaths



# save_restore.cmd needs the full path to the startup directory, which
# envPaths currently does not provide
#epicsEnvSet(STARTUP,$(TOP)/iocBoot/$(IOC))

# Increase size of buffer for error logging from default 1256
#errlogInit(20000)


cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/k6485.dbd"
k6485_registerRecordDeviceDriver pdbbase

## Load record instances

cd ${TOP}/iocBoot/${IOC}


# serial 1
#drvAsynSerialPortConfigure("portName","ttyName",priority,noAutoConnect,
#                            noProcessEos)
drvAsynSerialPortConfigure("serial1", "/dev/ttyS1", 0, 0, 0)
asynSetOption(serial1, 0, "baud",   "9600")
asynSetOption(serial1, 0, "bits",   "8")
asynSetOption(serial1, 0, "parity", "none")
asynSetOption(serial1, 0, "stop",   "1")
#asynOctetSetInputEos(const char *portName, int addr,
#                     const char *eosin,const char *drvInfo)
asynOctetSetInputEos("serial1",0,"\r")
# asynOctetSetOutputEos(const char *portName, int addr,
#                       const char *eosin,const char *drvInfo)
asynOctetSetOutputEos("serial1",0,"\r")
# Make port available from the iocsh command line
#asynOctetConnect(const char *entry, const char *port, int addr,
#                 int timeout, int buffer_len, const char *drvInfo)
asynOctetConnect("serial1", "serial1")



drvAsynKeithley6485("CA1","serial1",-1);
dbLoadRecords("$(TOP)/k6485App/Db/drvAsynKeithley6485.db","P=k6485:,CA=CA1:,PORT=CA1"



dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=k6485:,R=asyn_k6485,PORT=serial1,ADDR=0,OMAX=256,IMAX=2048")

## Run this to trace the stages of iocInit
#traceIocInit

iocInit

## Start any sequence programs
