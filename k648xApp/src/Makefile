TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library

LIBRARY_IOC += k648xSupport

# xxxRecord.h will be created from xxxRecord.dbd
#####DBDINC += xxxRecord
# Install devXxxSoft.dbd into <top>/dbd
#####DBD += xxxSupport.dbd

# Compile and add the code to the support library
#####k648xSupport_SRCS += xxxRecord.c
#####k648xSupport_SRCS += devXxxSoft.c

# Link locally-provided code into the support library,
# rather than directly into the IOC application.
#####k648xSupport_SRCS += dbSubExample.c
#####k648xSupport_SRCS += k648xHello.c
#####k648xSupport_SRCS += initTrace.c


k648xSupport_SRCS += drvAsynKeithley648x.cpp


k648xSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = k648x
# k648x.dbd will be created and installed
DBD += k648x.dbd

# k648x.dbd will be made up from these files:
k648x_DBD += base.dbd

# asyn
k648x_DBD += asyn.dbd
k648x_DBD += drvAsynSerialPort.dbd
#k648x_DBD += drvAsynIPPort.dbd
#k648x_DBD += drvVxi11.dbd
#k648x_DBD += devGpib.dbd



###k648x_DBD += xxxSupport.dbd
###k648x_DBD += dbSubExample.dbd
###k648x_DBD += k648xHello.dbd
###k648x_DBD += initTrace.dbd

k648x_DBD += drvAsynKeithley648x.dbd


# k648x_registerRecordDeviceDriver.cpp derives from k648x.dbd
k648x_SRCS += k648x_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
k648x_SRCS_DEFAULT += k648xMain.cpp
k648x_SRCS_vxWorks += -nil-



# Add support from base/src/vxWorks if needed
#k648x_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Link in the code from the support library
k648x_LIBS += k648xSupport

k648x_LIBS += asyn


# Finally link to the EPICS Base libraries
k648x_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

