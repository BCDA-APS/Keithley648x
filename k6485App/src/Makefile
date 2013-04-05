TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library

LIBRARY_IOC += k6485Support

# xxxRecord.h will be created from xxxRecord.dbd
#####DBDINC += xxxRecord
# Install devXxxSoft.dbd into <top>/dbd
#####DBD += xxxSupport.dbd

# Compile and add the code to the support library
#####k6485Support_SRCS += xxxRecord.c
#####k6485Support_SRCS += devXxxSoft.c

# Link locally-provided code into the support library,
# rather than directly into the IOC application.
#####k6485Support_SRCS += dbSubExample.c
#####k6485Support_SRCS += k6485Hello.c
#####k6485Support_SRCS += initTrace.c


k6485Support_SRCS += drvAsynKeithley6485.cpp


k6485Support_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = k6485
# k6485.dbd will be created and installed
DBD += k6485.dbd

# k6485.dbd will be made up from these files:
k6485_DBD += base.dbd

# asyn
k6485_DBD += asyn.dbd
k6485_DBD += drvAsynSerialPort.dbd
#k6485_DBD += drvAsynIPPort.dbd
#k6485_DBD += drvVxi11.dbd
#k6485_DBD += devGpib.dbd



###k6485_DBD += xxxSupport.dbd
###k6485_DBD += dbSubExample.dbd
###k6485_DBD += k6485Hello.dbd
###k6485_DBD += initTrace.dbd

k6485_DBD += drvAsynKeithley6485.dbd


# k6485_registerRecordDeviceDriver.cpp derives from k6485.dbd
k6485_SRCS += k6485_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
k6485_SRCS_DEFAULT += k6485Main.cpp
k6485_SRCS_vxWorks += -nil-



# Add support from base/src/vxWorks if needed
#k6485_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Link in the code from the support library
k6485_LIBS += k6485Support

k6485_LIBS += asyn


# Finally link to the EPICS Base libraries
k6485_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

