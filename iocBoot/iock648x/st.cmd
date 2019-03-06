#!../../bin/linux-x86_64/k648x

< envPaths

cd ${TOP}


## Register all support components
dbLoadDatabase "dbd/k648x.dbd"
k648x_registerRecordDeviceDriver pdbbase


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



#drvAsynKeithley648x("6487", "CA1","serial1",-1);
#dbLoadRecords("$(TOP)/k648xApp/Db/Keithley6487.db","P=k648x:,CA=CA1:,PORT=CA1"

drvAsynKeithley648x("6485", "CA1","serial1",-1);
dbLoadRecords("$(TOP)/k648xApp/Db/Keithley6485.db","P=k648x:,CA=CA1:,PORT=CA1"



dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=k648x:,R=asyn_k648x,PORT=serial1,ADDR=0,OMAX=256,IMAX=2048")

## Run this to trace the stages of iocInit
#traceIocInit

iocInit

