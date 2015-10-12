/*
 Description
    This module provides support for a multiple device port driver. To
    initialize the driver, the method drvAsynKeithley6485() is called from the
    startup script with the following calling sequence.

        drvAsynKeithley6485(myport,ioport,ioaddr)

        Where:
            myport - Keithley6485 Asyn interface port driver name (i.e. "EP0" )
            ioport - Communication port driver name (i.e. "S0" )
            ioaddr - Communication port device addr

    The method dbior can be called from the IOC shell to display the current
    status of the driver.
*/


/* System related include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* EPICS system related include files */
#include <iocsh.h>
#include <epicsStdio.h>
#include <cantProceed.h>
#include <epicsString.h>
#include <epicsExport.h>
#include <errlog.h>

/* EPICS synApps/Asyn related include files */
#include <asynDriver.h>
#include <asynDrvUser.h>
#include <asynInt32.h>
#include <asynFloat64.h>
#include <asynOctet.h>
#include <asynOctetSyncIO.h>
#include <asynStandardInterfaces.h>

/* Define symbolic constants */
#define TIMEOUT         (5.0)
#define BUFFER_SIZE     (100)


typedef enum {Octet=1, Float64=2, Int32=3} Type;

static const char *driver = "drvAsynKeithley648x";      /* String for asynPrint */


/* Declare port driver structure */
struct Port
{
  int devtype; // 1:6485, 2:6487
  char* myport;
  char* ioport;
  int ioaddr;

  int init; // really needed??

  char model[BUFFER_SIZE+1], *serial, *dig_rev, *disp_rev, *brd_rev;

  struct
  {
    int ioErrors;
    int writeReads;
    int writeOnlys;
  } stats;

  struct
  {
    double reading;
    int timestamp;
    union 
    {
      int raw;
      struct 
      {
        unsigned int overflow             : 1;
        unsigned int filter_enabled       : 1;
        unsigned int math_enabled         : 1;
        unsigned int null_enabled         : 1;
        unsigned int limit_test           : 1;
        unsigned int limit_result         : 2;
        unsigned int overvoltage          : 1;
        unsigned int padding              : 1;
        unsigned int zero_check_enabled   : 1;
        unsigned int zero_correct_enabled : 1;
      } bits;
    } status;
    /* 
       Bits:
       0 (OFLO)   — Set to 1 if measurement performed while in over-range 
                    (overflowed reading).
       1 (Filter) — Set to 1 when measurement performed with the averaging 
                    filter enabled.
       2 (Math)   — Set to 1 when measurement performed with CALC1 enabled.
       3 (Null)   — Set to 1 if null for CALC2 is enabled.
       4 (Limits) — Set to 1 if a limit test (CALC2) is enabled.
       5 & 6 (Limit Results) — Provides limit test results:
                Bit 6    Bit 5   
                  0        0      All limit tests passed
                  0        1      CALC2:LIM1 test failed
                  1        0      CALC2:LIM2 test failed
       7 (Overvoltage)  — Set to 1 if measurement performed with an 
                          overvoltage condition on the input.
       9 (Zero Check)   — Set to 1 when zero check is enabled.
      10 (Zero Correct) — Set to 1 when zero correct is enabled.
    */
    int eom;
  } data;

  /* Asyn info */
  asynUser *pasynUser;
  asynUser *pasynUserTrace;  /* asynUser for asynTrace on this port */
  asynStandardInterfaces asynStdInterfaces;
};


struct Command
{
  const char *tag;
  int dev;
  int type;
  int id;
};


/* Declare command structure */
struct GenCommand
{
  asynStatus (*readFunc)(int which, Port *pport, void* data, Type Iface, 
                         size_t *length, int *eom);
  asynStatus (*writeFunc)(int which, Port *pport, void* data, Type Iface);
};

struct SimpleCommand
{
  int type;
  const char *cmd_str;
};

/* Public interface forward references */
int drvAsynKeithley648x(const char* myport,const char* ioport, int ioaddr);


/* Forward references for asynCommon methods */
static void report(void* ppvt,FILE* fp,int details);
static asynStatus connect(void* ppvt,asynUser* pasynUser);
static asynStatus disconnect(void* ppvt,asynUser* pasynUser);
static asynCommon ifaceCommon = {report,connect,disconnect};

/* Forward references for asynDrvUser methods */
static asynStatus create(void* ppvt,asynUser* pasynUser,const char* drvInfo,
                         const char** pptypeName,size_t* psize);
static asynStatus destroy(void* ppvt,asynUser* pasynUser);
static asynStatus gettype(void* ppvt,asynUser* pasynUser,
                          const char** pptypeName,size_t* psize);
static asynDrvUser ifaceDrvUser = {create,gettype,destroy};

/* Forward references for asynFloat64 methods */
static asynStatus readFloat64(void* ppvt,asynUser* pasynUser,
                              epicsFloat64* value);
static asynStatus writeFloat64(void* ppvt,asynUser* pasynUser,
                               epicsFloat64 value);
static asynFloat64 ifaceFloat64 = {writeFloat64, readFloat64};

/* Forward references for asynInt32 methods */
static asynStatus readInt32(void* ppvt,asynUser* pasynUser,epicsInt32* value);
static asynStatus writeInt32(void* ppvt,asynUser* pasynUser,epicsInt32 value);
static asynInt32 ifaceInt32 =  {writeInt32, readInt32};

/* Forward references for asynOctet methods */
static asynStatus flushOctet( void* ppvt, asynUser* pasynUser);
static asynStatus writeOctet( void* ppvt, asynUser* pasynUser, const char *data,
                              size_t numchars, size_t* nbytes);
static asynStatus readOctet( void* ppvt, asynUser* pasynUser, char* data,
                             size_t maxchars, size_t *nbytes, int *eom);
static asynOctet ifaceOctet = { writeOctet, readOctet, flushOctet};


/* Forward references for external asynOctet interface */
static asynStatus writeOnly(Port* pport, const char* outBuf);
static asynStatus writeRead(Port* pport, const char* outBuf, char* inpBuf,
                            int inputSize, int *eomReason);


static asynStatus readDummy(int which, Port *pport, void *data, Type Iface, 
                            size_t *length, int *eom);
static asynStatus writeDummy(int which, Port *pport, void* data, Type Iface);

static asynStatus readSimpleData( int which, Port *pport, void *data, 
                                  Type Iface, size_t *length, int *eom);
static asynStatus writeSimpleData( int which, Port *pport, void *data, 
                                   Type Iface);

static asynStatus readCache(int which, Port *pport, void *data, 
                            Type Iface, size_t *length, int *eom);

static asynStatus readSensorReading(int which, Port *pport, void* data, 
                                    Type Iface, size_t *length, int *eom);
static asynStatus readRange(int which, Port *pport, void* data, 
                            Type Iface, size_t *length, int *eom);
static asynStatus writeRange(int which, Port *pport, void* data, Type Iface);
static asynStatus readRate(int which, Port *pport, void* data, 
                           Type Iface, size_t *length, int *eom);
static asynStatus writeRate(int which, Port *pport, void* data, Type Iface);
static asynStatus readCommon(int which, Port *pport, void* data, 
                             Type Iface, size_t *length, int *eom);
static asynStatus writeCommon(int which, Port *pport, void* data, Type Iface);




// General commands that need special attention go here
enum { VOID_CMD, READ_CMD, RANGE_CMD, RANGE_AUTO_ULIMIT_CMD, 
       RANGE_AUTO_LLIMIT_CMD, RATE_CMD, DIGITAL_FILTER_CONTROL_CMD,
       VOLT_RANGE_CMD, 
       GEN_CMD_NUMBER };
static GenCommand genCommandTable[GEN_CMD_NUMBER] = 
  {
    { readDummy,          writeDummy},  // VOID
    { readSensorReading,  writeDummy},  // READ
    { readRange,          writeRange},  // RANGE
    { readRange,          writeRange},  // RANGE_AUTO_ULIMIT
    { readRange,          writeRange},  // RANGE_AUTO_LLIMIT
    { readRate,           writeRate},   // RATE
    { readCommon,         writeCommon}, // DIGITAL_FILTER_CONTROL
    { readCommon,         writeCommon}, // VOLT_RANGE_COMMAND
  };

// commands that are very simple-minded go here
enum { RESET_CMD, RANGE_AUTO_CMD,
       ZERO_CHECK_CMD, ZERO_CORRECT_CMD, ZERO_CORRECT_ACQUIRE_CMD, 
       MEDIAN_FILTER_CMD, MEDIAN_FILTER_RANK_CMD, 
       DIGITAL_FILTER_CMD, DIGITAL_FILTER_COUNT_CMD,
       SIMPLE_CMD_NUMBER};

enum { SIMPLE_TRIGGER=0, SIMPLE_OCTET=Octet, SIMPLE_FLOAT64=Float64, 
       SIMPLE_INT32=Int32 };
static SimpleCommand simpleCommandTable[SIMPLE_CMD_NUMBER] = 
  {
    { SIMPLE_TRIGGER, "*RST"},           // RESET DEVICE

    { SIMPLE_INT32,   ":RANGE:AUTO"},    // RANGE_AUTO

    { SIMPLE_INT32,   "SYST:ZCH"},       // ZERO CHECK
    { SIMPLE_INT32,   "SYST:ZCOR"},      // ZERO CORRECT
    { SIMPLE_TRIGGER, "SYST:ZCOR:ACQ"},  // ZERO CORRECT ACQUIRE

    { SIMPLE_INT32,   "MED"},            // MEDIAN FILTER
    { SIMPLE_INT32,   "MED:RANK"},       // MEDIAN FILTER RANK

    { SIMPLE_INT32,   "AVER"},           // DIGITAL FILTER
    { SIMPLE_INT32,   "AVER:COUN"},      // DIGITAL FILTER COUNT

    // { SIMPLE_INT32,   "AVER:"},      // 
    // { SIMPLE_INT32,   "AVER:"},      // 
  };


enum { TIMESTAMP_CMD, STATUS_RAW_CMD, STATUS_OVERFLOW_CMD, STATUS_FILTER_CMD, 
       STATUS_MATH_CMD, STATUS_NULL_CMD, STATUS_LIMITS_CMD, 
       STATUS_OVERVOLTAGE_CMD, STATUS_ZERO_CHECK_CMD, STATUS_ZERO_CORRECT_CMD,
       MODEL_CMD, SERIAL_CMD, DIG_REV_CMD, DISP_REV_CMD, BRD_REV_CMD, 
       CACHE_CMD_NUMBER };

#define COMMAND_NUMBER (GEN_CMD_NUMBER + SIMPLE_CMD_NUMBER + CACHE_CMD_NUMBER)

enum { CMD_GEN, CMD_SIMPLE, CMD_CACHE };
enum { DEV_ALL, DEV_6485, DEV_6487};
static Command commandTable[ COMMAND_NUMBER ] = 
  {
    { "VOID",                   DEV_ALL,  CMD_GEN,    VOID_CMD                   },
    { "READ",                   DEV_ALL,  CMD_GEN,    READ_CMD                   },
    { "RANGE",                  DEV_ALL,  CMD_GEN,    RANGE_CMD                  },
    { "RANGE_AUTO_ULIMIT",      DEV_ALL,  CMD_GEN,    RANGE_AUTO_ULIMIT_CMD      },
    { "RANGE_AUTO_LLIMIT",      DEV_ALL,  CMD_GEN,    RANGE_AUTO_LLIMIT_CMD      },
    { "RATE",                   DEV_ALL,  CMD_GEN,    RATE_CMD                   },
    { "DIGITAL_FILTER_CONTROL", DEV_ALL,  CMD_GEN,    DIGITAL_FILTER_CONTROL_CMD },
    { "VOLT_RANGE",             DEV_6487, CMD_GEN,    VOLT_RANGE_CMD             },
    { "RESET",                  DEV_ALL,  CMD_SIMPLE, RESET_CMD                  },
    { "RANGE_AUTO",             DEV_ALL,  CMD_SIMPLE, RANGE_AUTO_CMD             },
    { "ZERO_CHECK",             DEV_ALL,  CMD_SIMPLE, ZERO_CHECK_CMD             },
    { "ZERO_CORRECT",           DEV_ALL,  CMD_SIMPLE, ZERO_CORRECT_CMD           },
    { "ZERO_CORRECT_ACQUIRE",   DEV_ALL,  CMD_SIMPLE, ZERO_CORRECT_ACQUIRE_CMD   },
    { "MEDIAN_FILTER",          DEV_ALL,  CMD_SIMPLE, MEDIAN_FILTER_CMD          },
    { "MEDIAN_FILTER_RANK",     DEV_ALL,  CMD_SIMPLE, MEDIAN_FILTER_RANK_CMD     },
    { "DIGITAL_FILTER",         DEV_ALL,  CMD_SIMPLE, DIGITAL_FILTER_CMD         },
    { "DIGITAL_FILTER_COUNT",   DEV_ALL,  CMD_SIMPLE, DIGITAL_FILTER_COUNT_CMD   },
    { "MODEL",                  DEV_ALL,  CMD_CACHE,  MODEL_CMD                  },
    { "SERIAL",                 DEV_ALL,  CMD_CACHE,  SERIAL_CMD                 },
    { "DIG_REV",                DEV_ALL,  CMD_CACHE,  DIG_REV_CMD                },
    { "DISP_REV",               DEV_ALL,  CMD_CACHE,  DISP_REV_CMD               },
    { "BRD_REV",                DEV_ALL,  CMD_CACHE,  BRD_REV_CMD                },
    { "TIMESTAMP",              DEV_ALL,  CMD_CACHE,  TIMESTAMP_CMD              },
    { "STATUS_RAW",             DEV_ALL,  CMD_CACHE,  STATUS_RAW_CMD             },
    { "STATUS_OVERFLOW",        DEV_ALL,  CMD_CACHE,  STATUS_OVERFLOW_CMD        },
    { "STATUS_FILTER",          DEV_ALL,  CMD_CACHE,  STATUS_FILTER_CMD          },
    { "STATUS_MATH",            DEV_ALL,  CMD_CACHE,  STATUS_MATH_CMD            },
    { "STATUS_NULL",            DEV_ALL,  CMD_CACHE,  STATUS_NULL_CMD            },
    { "STATUS_LIMITS",          DEV_ALL,  CMD_CACHE,  STATUS_LIMITS_CMD          },
    { "STATUS_OVERVOLTAGE",     DEV_ALL,  CMD_CACHE,  STATUS_OVERVOLTAGE_CMD     },
    { "STATUS_ZERO_CHECK",      DEV_ALL,  CMD_CACHE,  STATUS_ZERO_CHECK_CMD      },
    { "STATUS_ZERO_CORRECT",    DEV_ALL,  CMD_CACHE,  STATUS_ZERO_CORRECT_CMD    },
  };



/****************************************************************************
 * Define public interface methods
 ****************************************************************************/
int drvAsynKeithley648x(const char *type, const char *myport,
                        const char *ioport, int ioaddr)
{
  int status = asynSuccess;
  Port* pport;
  //  int i;
  asynStandardInterfaces *pInterfaces;

  char inpBuf[BUFFER_SIZE];
  int eomReason;

  pport = (Port*)callocMustSucceed(1,sizeof(Port),"drvAsynKeithley6485");
  pport->myport = epicsStrDup(myport);
  pport->ioport = epicsStrDup(ioport);
  pport->ioaddr = ioaddr;

  pport->devtype = DEV_ALL;
  if( !strcmp("6485", type))
    pport->devtype = DEV_6485;
  if( !strcmp("6487", type))
    pport->devtype = DEV_6487;
  if(pport->devtype == DEV_ALL) // DEV_ALL is not a real device
    {
      errlogPrintf("%s::drvAsynKeithley6485 type has to be "
                   "either \'6485\' or \'6487\'.\n",
                   driver, myport, ioport, ioaddr);
      return asynError;
    }


  status = pasynOctetSyncIO->connect(ioport,ioaddr,&pport->pasynUser,NULL);
  if (status != asynSuccess)
    {
      errlogPrintf("%s::drvAsynKeithley6485 port %s can't connect "
                   "to asynCommon on Octet server %s address %d.\n",
                   driver, myport, ioport, ioaddr);
      return asynError;
    }

  /* Create asynUser for asynTrace */
  pport->pasynUserTrace = pasynManager->createAsynUser(0, 0);
  pport->pasynUserTrace->userPvt = pport;

  status = pasynManager->registerPort(myport,ASYN_CANBLOCK,1,0,0);
  if( status != asynSuccess) 
    {
      errlogPrintf("%s::drvAsynKeithley6485 port %s can't register port\n",
                   driver, myport);
      return asynError;
    }

  pInterfaces = &pport->asynStdInterfaces;
    
  /* Initialize interface pointers */
  pInterfaces->common.pinterface    = (void *)&ifaceCommon;
  pInterfaces->drvUser.pinterface   = (void *)&ifaceDrvUser;
  pInterfaces->octet.pinterface     = (void *)&ifaceOctet;
  pInterfaces->int32.pinterface     = (void *)&ifaceInt32;
  pInterfaces->float64.pinterface   = (void *)&ifaceFloat64;

  status = pasynStandardInterfacesBase->initialize(myport, pInterfaces,
                                                   pport->pasynUserTrace, 
                                                   pport);
  if (status != asynSuccess) 
    {
      errlogPrintf("%s::drvAsynKeithley6485 port %s"
                   " can't register standard interfaces: %s\n",
                   driver, myport, pport->pasynUserTrace->errorMessage);
      return asynError;
    }

#ifdef vxWorks
  /* Send a sacrificial clear status to vxworks device (i.e. VME)*/
  /* This fixes a problem with *IDN? call when starting from a cold boot */
  /* with the SBS IP-Octal hardware. */
  if( writeOnly(pport,"") )
    {
      errlogPrintf("%s::drvAsynKeithley6485 port %s failed to write\n",
                   driver, myport);
      return asynError;
    }
#endif

  /* Reset device */
  if( writeOnly(pport,"*CLS") )
    {
      errlogPrintf("%s::drvAsynKeithley6485 port %s failed to clear\n",
                   driver, myport);
      return asynError;
    }
  

  /* Identification query */
  if( writeRead(pport,"*IDN?",inpBuf,sizeof(inpBuf),&eomReason) )
    {
      errlogPrintf("%s::drvAsynKeithley6485 port %s failed to "
                   "acquire identification\n", driver, myport);
      return asynError;
    }
  strcpy(pport->model,inpBuf);
  // char *model, *serial, *dig_rev, *disp_rev, *brd_rev;
  pport->serial = strchr( pport->model, ',');
  pport->serial = strchr( pport->serial + 1, ',');
  *(pport->serial) = '\0';
  pport->serial++;
  pport->dig_rev = strchr( pport->serial, ',');
  *(pport->dig_rev) = '\0';
  pport->dig_rev++;
  pport->disp_rev = strchr( pport->dig_rev, '/');
  *(pport->disp_rev) = '\0';
  pport->disp_rev++;
  pport->brd_rev = strchr( pport->disp_rev, '/');
  *(pport->brd_rev) = '\0';
  pport->brd_rev++;
  
  /* Complete initialization */
  pport->init=1;

  pport->data.reading = 0.0;
  pport->data.timestamp = 0;
  pport->data.status.raw = 0;

  return asynSuccess;
}




/****************************************************************************
 * Define private read and write parameter methods
 ****************************************************************************/

static asynStatus readDummy(int which, Port *pport, void *data, Type Iface, 
                            size_t *length, int *eom)
{
  return asynSuccess;
}

static asynStatus writeDummy(int which, Port *pport, void *data, Type Iface)
{
  return asynSuccess;
}

///

static asynStatus readSimpleData( int which, Port *pport, void *data, 
                                  Type Iface, size_t *length, int *eom)
{
  asynStatus status;
  char outBuf[BUFFER_SIZE];
  char inpBuf[BUFFER_SIZE];

  int len;

  // Trigger will automatically not work
  if( simpleCommandTable[which].type != Iface)
    return asynSuccess;

  sprintf( outBuf, "%s?", simpleCommandTable[which].cmd_str);
    
  status = writeRead( pport, outBuf, inpBuf, BUFFER_SIZE, &pport->data.eom);
  if( status != asynSuccess)
    return status;

  switch( Iface)
    {
    case Float64:
      *((epicsFloat64 *) data) = atof(inpBuf);
      break;
    case Int32:
      *((epicsInt32 *) data) = atoi(inpBuf);
      break;
    case Octet:
      len = strlen( inpBuf);
      if( len > 39)
        inpBuf[39] = '\0';
      strcpy( (char *) data, inpBuf);
      break;
    }

  return asynSuccess;
}

static asynStatus writeSimpleData( int which, Port *pport, void *data, 
                                    Type Iface)
{
  char outBuf[BUFFER_SIZE];
  
  if( simpleCommandTable[which].type == SIMPLE_TRIGGER )
    sprintf( outBuf, "%s", simpleCommandTable[which].cmd_str);
  else
    {
      if( simpleCommandTable[which].type != Iface )
        return asynSuccess;
      
      switch( simpleCommandTable[which].type & Iface)
        {
        case Float64:
          sprintf( outBuf, "%s %g", simpleCommandTable[which].cmd_str, 
                   *((epicsFloat64*) data) );
          break;
        case Int32:
          sprintf( outBuf, "%s %d", simpleCommandTable[which].cmd_str, 
                   *((epicsInt32*) data) );
          break;
        case Octet:
          sprintf( outBuf, "%s %s", simpleCommandTable[which].cmd_str, 
                   ((char *) data) );
          break;
        }
    }

  return writeOnly( pport, outBuf);
}

////

static asynStatus readCache(int which, Port *pport, void *data, 
                            Type Iface, size_t *length, int *eom)
{
  char *char_cache = NULL;
  int len;

  switch( Iface)
    {
    case Octet:
      switch(which)
        {
        case MODEL_CMD:
          char_cache = pport->model;
          break;
        case SERIAL_CMD:
          char_cache = pport->serial;
          break;
        case DIG_REV_CMD:
          char_cache = pport->dig_rev;
          break;
        case DISP_REV_CMD:
          char_cache = pport->disp_rev;
          break;
        case BRD_REV_CMD:
          char_cache = pport->brd_rev;
          break;
        }
      len = strlen(char_cache);
      if( len < 40)
        strcpy( (char *) data, char_cache);
      else // just in case string will overflow the EPICS string size of 40
        {
          len = 39;
          memcpy( (char *) data, char_cache, len);
          ((char *) data)[len] = '\0';
        }
      *length = len;
      *eom = 0;
      break;
    case Float64:
    //   switch( which)
    //     {
    //     }
      break;
    case Int32:
      switch( which)
        {
        case TIMESTAMP_CMD:
          *(epicsInt32*)data = pport->data.timestamp;
          break;  
        case STATUS_RAW_CMD:
          *(epicsInt32*) data = pport->data.status.raw;
          break;
        case STATUS_OVERFLOW_CMD:
          *(epicsInt32*) data = pport->data.status.bits.overflow;
          break;
        case STATUS_FILTER_CMD:
          *(epicsInt32*) data = pport->data.status.bits.filter_enabled;
          break;
        case STATUS_MATH_CMD:
          *(epicsInt32*) data = pport->data.status.bits.math_enabled;
          break;
        case STATUS_NULL_CMD:
          *(epicsInt32*) data = pport->data.status.bits.null_enabled;
          break;
        case STATUS_LIMITS_CMD:
          if( pport->data.status.bits.limit_test)
            *(epicsInt32*) data = pport->data.status.bits.limit_result;
          else
            *(epicsInt32*) data = 3;
          break;
        case STATUS_OVERVOLTAGE_CMD:
          *(epicsInt32*) data = pport->data.status.bits.overvoltage;
          break;
        case STATUS_ZERO_CHECK_CMD:
          *(epicsInt32*) data = pport->data.status.bits.zero_check_enabled;
          break;
        case STATUS_ZERO_CORRECT_CMD:
          *(epicsInt32*) data = pport->data.status.bits.zero_correct_enabled;
          break;
        }
      break;
    }

  return asynSuccess;
}


static asynStatus readSensorReading(int which, Port *pport, void *data, 
                                    Type Iface, size_t *length, int *eom)
{
  asynStatus status;
  char inpBuf[BUFFER_SIZE];

  char *str, *token[3], *saveptr;
  int pass;

  status = writeRead( pport, "READ?", inpBuf, BUFFER_SIZE, &pport->data.eom);
  if( status != asynSuccess)
    return status;

  str = inpBuf;
  for( pass = 0; pass < 3; pass++, str = NULL)
    {
      token[pass] = epicsStrtok_r(str, ",", &saveptr);
      if (token == NULL)
        break;
    }
  if( pass != 3)
    return asynError;

  pport->data.reading = atof( token[0]);
  pport->data.timestamp = (int) atof( token[1]);
  pport->data.status.raw = (int) atof( token[2]);

  switch( Iface )
    {
    case Octet:
      // only print current value
      *length = sprintf( (char *) data, "%s", token[0]);
      *eom = pport->data.eom;
      break;
    case Float64:
      *(epicsFloat64*)data = pport->data.reading;
      break;
    case Int32:
      break;
    }

  return asynSuccess;
}

static asynStatus readRange(int which, Port *pport, void *data, 
                            Type Iface, size_t *length, int *eom)
{
  asynStatus status;
  char inpBuf[BUFFER_SIZE];

  if( Iface == Octet)
    return asynSuccess;

  switch( which)
    {
    case RANGE_CMD:
      status = writeRead( pport, ":RANGE?", inpBuf, BUFFER_SIZE, 
                          &pport->data.eom);
      break;
    case RANGE_AUTO_ULIMIT_CMD:
      status = writeRead( pport, ":RANGE:AUTO:ULIM?", inpBuf, BUFFER_SIZE, 
                          &pport->data.eom);
      break;
    case RANGE_AUTO_LLIMIT_CMD:
      status = writeRead( pport, ":RANGE:AUTO:LLIM?", inpBuf, BUFFER_SIZE, 
                          &pport->data.eom);
      break;
    default:
      return asynError;
    }
  if( status != asynSuccess)
    return status;

  if( Iface == Float64)
    {
      double val;

      val = atof( inpBuf);
      if( val == 0.0)
        return asynError;

      *(epicsFloat64*) data = val;
    }
  else if( Iface == Int32)
    {
      char *p;

      p = strchr( inpBuf, 'E');
      if(p == NULL)
        return asynError;
      p++;
      *(epicsInt32*) data = 9 + atoi(p) ;
    }

  return asynSuccess;
}


static asynStatus writeRange( int which, Port *pport, void *data, Type Iface)
{
  char outBuf[BUFFER_SIZE];
  int value;

  if( Iface != Int32)
    return asynSuccess;

  value = *((epicsInt32*) data);
  if( (value < 0) || (value > 7) )
    return asynError;
    
  switch( which)
    {
    case RANGE_CMD:
      sprintf( outBuf, ":RANGE 2.0e%d", -9 + value );
      break;
    case RANGE_AUTO_ULIMIT_CMD:
      sprintf( outBuf, ":RANGE:AUTO:ULIM 2.0e%d", -9 + value );
      break;
    case RANGE_AUTO_LLIMIT_CMD:
      sprintf( outBuf, ":RANGE:AUTO:LLIM 2.0e%d", -9 + value );
      break;
    default:
      return asynError;
    }

  return writeOnly( pport, outBuf);
}


static asynStatus readRate(int which, Port *pport, void *data, 
                           Type Iface, size_t *length, int *eom)
{
  asynStatus status;
  char inpBuf[BUFFER_SIZE];

  double val;
  int rate;

  if( Iface != Int32)
    return asynSuccess;

  status = writeRead( pport, ":NPLC?", inpBuf, BUFFER_SIZE, &pport->data.eom);
  if( status != asynSuccess)
    return status;

  val = atof( inpBuf);
  if( val > 1.0)
    rate = 0; // SLOW
  else if( val > 0.1)
    rate = 1; // MEDIUM
  else
    rate = 2; // FAST
  
  *((epicsInt32*) data) = rate;

  return asynSuccess;
}


static asynStatus writeRate( int which, Port *pport, void *data, Type Iface)
{
  char outBuf[BUFFER_SIZE];
  int rate;
  double val;

  if( Iface != Int32)
    return asynSuccess;

  rate = *((epicsInt32*) data);
  if( (rate < 0) || (rate > 2) )
    return asynError;

  switch( rate)
    {
    case 0:
      val = 6.0;
      break;
    case 1:
      val = 1.0;
      break;
    case 2:
      val = 0.1;
      break;
    default:
      val = 1.0;
      break;
    }

  sprintf( outBuf, ":NPLC %g", val );
  return writeOnly( pport, outBuf);
}


static asynStatus readCommon(int which, Port *pport, void *data, 
                             Type Iface, size_t *length, int *eom)
{
  asynStatus status;
  char inpBuf[BUFFER_SIZE];

  int val;

  if( Iface != Int32)
    return asynSuccess;

  switch(which)
    {
    case DIGITAL_FILTER_CONTROL_CMD:
      status = writeRead( pport, "AVER:TCON?", inpBuf, BUFFER_SIZE, 
                          &pport->data.eom);

      if( status != asynSuccess)
        return status;
      if( !strcmp( "MOV", inpBuf) )
        val = 0;
      else if( !strcmp( "REP", inpBuf) )
        val = 1;
      else
        return asynError;
      break;
    }
  
  *((epicsInt32*) data) = val;

  return asynSuccess;
}


static asynStatus writeCommon( int which, Port *pport, void *data, Type Iface)
{
  char outBuf[BUFFER_SIZE];
  int val;

  if( Iface != Int32)
    return asynSuccess;

  val = *((epicsInt32*) data);
  switch( which)
    {
    case DIGITAL_FILTER_CONTROL_CMD:
      if( !val)
        sprintf( outBuf, "AVER:TCON MOV" );
      else if(val == 1)
        sprintf( outBuf, "AVER:TCON REP" );
      else
        return asynError;
      break;
    }

  return writeOnly( pport, outBuf);
}


/****************************************************************************
 * Define private interface asynCommon methods
 ****************************************************************************/
static void report(void* ppvt,FILE* fp,int details)
{
  //  int i;
  Port* pport = (Port*)ppvt;

  fprintf( fp, "Keithley648x port: %s\n", pport->myport);
  if( details)
    {
      fprintf( fp, "    server:     %s\n", pport->ioport);
      fprintf( fp, "    address:    %d\n", pport->ioaddr);
      fprintf( fp, "    ioErrors:   %d\n", pport->stats.ioErrors);
      fprintf( fp, "    writeReads: %d\n", pport->stats.writeReads);
      fprintf( fp, "    writeOnlys: %d\n", pport->stats.writeOnlys);
      fprintf( fp, "    support %s initialized\n",(pport->init)?"IS":"IS NOT");
    }

}

static asynStatus connect(void* ppvt,asynUser* pasynUser)
{
  pasynManager->exceptionConnect(pasynUser);
  return asynSuccess;
}

static asynStatus disconnect(void* ppvt,asynUser* pasynUser)
{
  pasynManager->exceptionDisconnect(pasynUser);
  return asynSuccess;
}


/****************************************************************************
 * Define private interface asynDrvUser methods
 ****************************************************************************/
static asynStatus create(void* ppvt, asynUser *pasynUser, const char *drvInfo, 
                         const char **pptypeName, size_t *psize)
{
  Port* pport=(Port*)ppvt;
  
  int i;
  
  for(i = 0; i < COMMAND_NUMBER; i++) 
    if( !epicsStrCaseCmp( drvInfo, commandTable[i].tag) ) 
      {
        if( (commandTable[i].dev != DEV_ALL) && 
            (commandTable[i].dev != pport->devtype) )
          {
            errlogPrintf("%s::create port %s failed as tag %s is for different "
                         "device\n", driver, pport->myport, drvInfo);
            pasynUser->reason = 0;
            return asynError;
          }
        pasynUser->reason = i;
        break;
      }
  if( i == COMMAND_NUMBER ) 
    {
      errlogPrintf("%s::create port %s failed to find tag %s\n",
                   driver, pport->myport, drvInfo);
      pasynUser->reason = 0;
      return asynError;
    }
  
  return asynSuccess;
}

static asynStatus gettype(void* ppvt,asynUser* pasynUser,
                          const char** pptypeName,size_t* psize)
{
  if( pptypeName ) 
    *pptypeName = NULL;
  if( psize ) 
    *psize = 0;

  return asynSuccess;
}

static asynStatus destroy(void* ppvt,asynUser* pasynUser)
{
  return asynSuccess;
}


/****************************************************************************
 * Define private interface asynFloat64 methods
 ****************************************************************************/
static asynStatus writeFloat64(void* ppvt,asynUser* pasynUser,
                               epicsFloat64 value)
{
  Port* pport=(Port*)ppvt;
  int which = pasynUser->reason;

  int id;
  id = commandTable[which].id;

  if( pport->init == 0) 
    return asynError;

   switch( commandTable[which].type )
    {
    case CMD_GEN:
      return genCommandTable[id].writeFunc(id, pport, &value, Float64);
      break;
    case CMD_SIMPLE:
      return writeSimpleData( id, pport, &value, Float64);
      break;
    }

  return asynSuccess;
}

static asynStatus readFloat64(void* ppvt,asynUser* pasynUser,
                              epicsFloat64* value)
{
  Port* pport=(Port*)ppvt;
  int which = pasynUser->reason;

  int id;
  id = commandTable[which].id;

  if( pport->init == 0) 
    return asynError;

  switch( commandTable[which].type )
    {
    case CMD_GEN:
      return genCommandTable[id].readFunc(id, pport, value, Float64, 
                                          NULL, NULL);
      break;
    case CMD_SIMPLE:
      return readSimpleData( id, pport, value, Float64, NULL, NULL);
      break;
    case CMD_CACHE:
      return readCache(id, pport, value, Float64, NULL, NULL);
      break;
    }

  return asynSuccess;
}


/****************************************************************************
 * Define private interface asynInt32 methods
 ****************************************************************************/
static asynStatus writeInt32(void *ppvt, asynUser *pasynUser, epicsInt32 value)
{
  Port* pport=(Port*)ppvt;
  int which = pasynUser->reason;

  int id;
  id = commandTable[which].id;

  if( pport->init == 0) 
    return asynError;

  switch( commandTable[which].type )
    {
    case CMD_GEN:
      return genCommandTable[id].writeFunc(id, pport, &value, Int32);
      break;
    case CMD_SIMPLE:
      return writeSimpleData( id, pport, (void *) &value, Int32);
      break;
    }
  
  return asynSuccess;
}

static asynStatus readInt32(void *ppvt, asynUser *pasynUser, epicsInt32 *value)
{
  Port* pport=(Port*)ppvt;
  int which = pasynUser->reason;

  int id;
  id = commandTable[which].id;

  if( pport->init == 0) 
    return asynError;

  switch( commandTable[which].type )
    {
    case CMD_GEN:
      return genCommandTable[id].readFunc(id, pport, value, Int32, 
                                          NULL, NULL);
      break;
    case CMD_SIMPLE:
      return readSimpleData( id, pport, value, Int32, NULL, NULL);
      break;
    case CMD_CACHE:
      return readCache(id, pport, value, Int32, NULL, NULL);
      break;
    }

  return asynSuccess;
}


/****************************************************************************
 * Define private interface asynOctet methods
 ****************************************************************************/
static asynStatus flushOctet(void *ppvt, asynUser* pasynUser)
{
  return asynSuccess;
}

static asynStatus writeOctet(void *ppvt, asynUser *pasynUser, const char *data,
                             size_t numchars, size_t *nbytes)
{
  Port* pport=(Port*)ppvt;
  int which = pasynUser->reason;

  int id;
  id = commandTable[which].id;

  if( pport->init == 0) 
    return asynError;

  switch( commandTable[which].type )
    {
    case CMD_GEN:
      *nbytes=strlen(data);
      return genCommandTable[id].writeFunc(id, pport, (void *) data, Octet);
      break;
    case CMD_SIMPLE:
      *nbytes=strlen(data);
      return writeSimpleData( id, pport, &data, Octet);
      break;
    }
  
  return asynSuccess;
}

static asynStatus readOctet(void* ppvt, asynUser* pasynUser, char* data,
                            size_t maxchars,size_t* nbytes,int* eom)
{
  Port* pport=(Port*)ppvt;
  int which = pasynUser->reason;

  int id;
  id = commandTable[which].id;

  if( pport->init == 0) 
    return asynError;

  switch( commandTable[which].type )
    {
    case CMD_GEN:
      return genCommandTable[id].readFunc(id, pport, (void *) data, Octet, 
                                          nbytes, eom);
      break;
    case CMD_SIMPLE:
      return readSimpleData( id, pport, data, Octet, nbytes, eom);
      break;
    case CMD_CACHE:
      return readCache(id, pport, (void *) data, Octet, nbytes, eom);
      break;
    }

  return asynSuccess;
}


/****************************************************************************
 * Define private Keithley648x external interface asynOctet methods
 ****************************************************************************/
static asynStatus writeOnly(Port *pport, const char *outBuf)
{
  asynStatus status;
  size_t nActual, nRequested;

  nRequested=strlen(outBuf);
  status = 
    pasynOctetSyncIO->write(pport->pasynUser,outBuf,nRequested,TIMEOUT,&nActual);
  if( nActual!=nRequested ) 
    status = asynError;

  if( status!=asynSuccess )
    {
      pport->stats.ioErrors++;
      asynPrint(pport->pasynUserTrace,ASYN_TRACE_ERROR, 
                "%s writeOnly: error %d wrote \"%s\"\n",
                pport->myport,status,outBuf);
    }
  else
    pport->stats.writeOnlys++;

  asynPrint(pport->pasynUserTrace, ASYN_TRACEIO_FILTER,
            "%s writeOnly: wrote \"%s\"\n",
            pport->myport,outBuf);

  return status;
}

static asynStatus writeRead(Port *pport, const char *outBuf, char *inpBuf,
                            int inputSize, int *eomReason)
{
  asynStatus status;
  size_t nWrite, nRead, nWriteRequested;

  nWriteRequested=strlen(outBuf);
  status = pasynOctetSyncIO->writeRead(pport->pasynUser,outBuf,
                                       nWriteRequested,inpBuf,inputSize-1,
                                       TIMEOUT,&nWrite,&nRead,eomReason);
  if( nWrite!=nWriteRequested ) 
    status = asynError;

  if( status!=asynSuccess )
    {
      pport->stats.ioErrors++;
      asynPrint(pport->pasynUserTrace,ASYN_TRACE_ERROR,
                "%s writeRead: error %d wrote \"%s\"\n",
                pport->myport,status,outBuf);
    }
  else
    {
      inpBuf[nRead]='\0';
      pport->stats.writeReads++;
    }

  asynPrint(pport->pasynUserTrace,ASYN_TRACEIO_FILTER,
            "%s writeRead: wrote \"%s\" read \"%s\"\n",
            pport->myport,outBuf,inpBuf);
  
  return status;
}


/****************************************************************************
 * Register public methods
 ****************************************************************************/
 
/* Initialization method definitions */
static const iocshArg arg0 = {"type",iocshArgString};
static const iocshArg arg1 = {"myport",iocshArgString};
static const iocshArg arg2 = {"ioport",iocshArgString};
static const iocshArg arg3 = {"ioaddr",iocshArgInt};
static const iocshArg* args[]= {&arg0,&arg1,&arg2,&arg3};
static const iocshFuncDef drvAsynKeithley648xFuncDef = 
  {"drvAsynKeithley648x",4,args};
static void drvAsynKeithley648xCallFunc(const iocshArgBuf* args)
{
  drvAsynKeithley648x(args[0].sval,args[1].sval,args[2].sval,args[3].ival);
}

/* Registration method */
static void drvAsynKeithley648xRegister(void)
{
  static int firstTime = 1;

  if( firstTime )
    {
      firstTime = 0;
      iocshRegister( &drvAsynKeithley648xFuncDef,drvAsynKeithley648xCallFunc );
    }
}
epicsExportRegistrar( drvAsynKeithley648xRegister );
