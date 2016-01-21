// Standard includes
#include <vector>
#include <sys/stat.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <locale>
#include <map>

// EPICS includes
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

// areaDetector includes
#include <ADDriver.h>

// Asyn driver includes
#include "asynOctetSyncIO.h"

// String length
#define SPECS_MAX_STRING 4096
// Asyn timeout
#define SPECS_TIMEOUT 1000
// SPECS Update rate
#define SPECS_UPDATE_RATE 0.1
// SPECS Response OK/ERROR
#define SPECS_OK_STRING    "OK"
#define SPECS_ERROR_STRING "ERROR"

// SPECS Run Modes
#define SPECS_RUN_FAT  0
#define SPECS_RUN_SFAT 1
#define SPECS_RUN_FRR  2
#define SPECS_RUN_FE   3

// SPECS Types
#define SPECS_TYPE_DOUBLE  "double"
#define SPECS_TYPE_INTEGER "integer"
#define SPECS_TYPE_STRING  "string"
#define SPECS_TYPE_BOOL    "bool"

// SPECS Command Strings
#define SPECS_CMD_CONNECT      "Connect"
#define SPECS_CMD_DISCONNECT   "Disconnect"
#define SPECS_CMD_DEFINE_FAT   "DefineSpectrumFAT"
#define SPECS_CMD_DEFINE_SFAT  "DefineSpectrumSFAT"
#define SPECS_CMD_DEFINE_FRR   "DefineSpectrumFRR"
#define SPECS_CMD_DEFINE_FE    "DefineSpectrumFE"
#define SPECS_CMD_VALIDATE     "ValidateSpectrum"
#define SPECS_CMD_START        "Start"
#define SPECS_CMD_PAUSE        "Pause"
#define SPECS_CMD_RESUME       "Resume"
#define SPECS_CMD_ABORT        "Abort"
#define SPECS_CMD_GET_STATUS   "GetAcquisitionStatus"
#define SPECS_CMD_GET_DATA     "GetAcquisitionData"
#define SPECS_CMD_CLEAR        "ClearSpectrum"
#define SPECS_CMD_GET_NAMES    "GetAllAnalyzerParameterNames"
#define SPECS_CMD_GET_INFO     "GetAnalyzerParameterInfo"
#define SPECS_CMD_GET_VISNAME  "GetAnalyzerVisibleName"
#define SPECS_CMD_GET_VALUE    "GetAnalyzerParameterValue"
#define SPECS_CMD_SET_VALUE    "SetAnalyzerParameterValue"

// Pre-defined EPICS Parameter Names
#define SPECSConnectString                   "SPECS_CONNECT"
#define SPECSConnectedString                 "SPECS_CONNECTED"
#define SPECSPauseAcqString                  "SPECS_PAUSE_ACQ"
#define SPECSMsgCounterString                "SPECS_MSG_COUNTER"
#define SPECSServerNameString                "SPECS_SERVER_NAME"
#define SPECSProtocolVersionString           "SPECS_PROTOCOL_VERSION"
#define SPECSStartEnergyString               "SPECS_START_ENERGY"
#define SPECSEndEnergyString                 "SPECS_END_ENERGY"
#define SPECSRetardingRatioString            "SPECS_RETARDING_RATIO"
#define SPECSKineticEnergyString             "SPECS_KINETIC_ENERGY"
#define SPECSStepWidthString                 "SPECS_STEP_WIDTH"
#define SPECSSamplesString                   "SPECS_SAMPLES"
#define SPECSSamplesIterationString          "SPECS_SAMPLES_ITERATION"
#define SPECSSnapshotValuesString            "SPECS_SNAPSHOT_VALUES"
#define SPECSPassEnergyString                "SPECS_PASS_ENERGY"
#define SPECSLensModeString                  "SPECS_LENS_MODE"
#define SPECSScanRangeString                 "SPECS_SCAN_RANGE"
#define SPECSCurrentSampleString             "SPECS_CURRENT_SAMPLE"
#define SPECSPercentCompleteString           "SPECS_PERCENT_COMPLETE"
#define SPECSRemainingTimeString             "SPECS_REMAINING_TIME"
#define SPECSCurrentSampleIterationString    "SPECS_CRT_SAMPLE_ITER"
#define SPECSPercentCompleteIterationString  "SPECS_PCT_COMPLETE_ITER"
#define SPECSRemainingTimeIterationString    "SPECS_RMG_TIME_ITER"
#define SPECSAcqSpectrumString               "SPECS_ACQ_SPECTRUM"
#define SPECSAcqImageString                  "SPECS_ACQ_IMAGE"

#define SPECSRunModeString                   "SPECS_RUN_MODE"
#define SPECSDefineString                    "SPECS_DEFINE"
#define SPECSValidateString                  "SPECS_VALIDATE"

#define SPECSNonEnergyChannelsString   "SPECS_NON_ENERGY_CHANNELS"

typedef enum
{
  SPECSTypeDouble,
  SPECSTypeInteger,
  SPECSTypeString,
  SPECSTypeBool
} SPECSValueType_t;

class SpecsAnalyser: public ADDriver
{
  public:
    SpecsAnalyser(const char *portName, const char *driverPort, int maxBuffers, size_t maxMemory, int priority, int stackSize);
    virtual ~SpecsAnalyser();
    void specsAnalyserTask();
    asynStatus makeConnection();
    asynStatus connect();
    asynStatus disconnect();
    asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    asynStatus validateSpectrum();
    asynStatus defineSpectrumFAT();
    asynStatus defineSpectrumSFAT();
    asynStatus defineSpectrumFRR();
    asynStatus defineSpectrumFE();
    asynStatus readAcquisitionData(int startIndex, int endIndex, std::vector<double> &values);
    asynStatus sendSimpleCommand(const std::string& command, std::map<std::string, std::string> *data = NULL);
    asynStatus readDeviceVisibleName();
    asynStatus setupEPICSParameters();
    asynStatus getAnalyserParameterType(const std::string& name, SPECSValueType_t &value);
    asynStatus getAnalyserParameter(const std::string& name, int &value);
    asynStatus getAnalyserParameter(const std::string& name, double &value);
    asynStatus getAnalyserParameter(const std::string& name, std::string &value);
    asynStatus getAnalyserParameter(const std::string& name, bool &value);
    asynStatus setAnalyserParameter(const std::string& name, int value);
    asynStatus setAnalyserParameter(const std::string& name, double value);
    asynStatus setAnalyserParameter(const std::string& name, std::string value);
    asynStatus readIntegerData(std::map<std::string, std::string> data, const std::string& name, int &value);
    asynStatus readDoubleData(std::map<std::string, std::string> data, const std::string& name, double &value);
    asynStatus readLensModes();
    asynStatus readScanRanges();
    asynStatus readRunModes();
    asynStatus asynPortConnect(const char *port, int addr, asynUser **ppasynUser, const char *inputEos, const char *outputEos);
    asynStatus asynPortDisconnect(asynUser *pasynUser);
    asynStatus commandResponse(const std::string &command, std::string &response, std::map<std::string, std::string> &data);
    asynStatus asynWriteRead(const char *command, char *response);

    // String cleanup routines
    asynStatus cleanString(std::string &str, const std::string &search = ": \n", int where = 0);

    // Debugging routines
    asynStatus initDebugger(int initDebug);
    asynStatus debugLevel(const std::string& method, int onOff);
    asynStatus debug(const std::string& method, const std::string& msg);
    asynStatus debug(const std::string& method, const std::string& msg, int value);
    asynStatus debug(const std::string& method, const std::string& msg, double value);
    asynStatus debug(const std::string& method, const std::string& msg, const std::string& value);
    asynStatus debug(const std::string& method, const std::string& msg, std::map<std::string, std::string> value);

  protected:
    int SPECSConnect_;
    #define FIRST_SPECS_PARAM SPECSConnect_
    int SPECSConnected_;
    int SPECSPauseAcq_;
    int SPECSMsgCounter_;
    int SPECSServerName_;
    int SPECSProtocolVersion_;
    int SPECSStartEnergy_;
    int SPECSEndEnergy_;
    int SPECSRetardingRatio_;
    int SPECSKineticEnergy_;
    int SPECSStepWidth_;
    int SPECSSamples_;
    int SPECSSamplesIteration_;
    int SPECSSnapshotValues_;
    int SPECSPassEnergy_;
    int SPECSLensMode_;
    int SPECSScanRange_;
    int SPECSCurrentSample_;
    int SPECSPercentComplete_;
    int SPECSRemainingTime_;
    int SPECSCurrentSampleIteration_;
    int SPECSPercentCompleteIteration_;
    int SPECSRemainingTimeIteration_;
    int SPECSAcqSpectrum_;
    int SPECSAcqImage_;

    int SPECSRunMode_;
    int SPECSDefine_;
    int SPECSValidate_;

    int SPECSNonEnergyChannels_;
    #define LAST_SPECS_PARAM SPECSNonEnergyChannels_

  private:
    asynUser                           *portUser_;
    char                               driverPort_[SPECS_MAX_STRING];
    std::map<std::string, int>         debugMap_;
    epicsEventId                       startEventId_;
    epicsEventId                       stopEventId_;
    std::vector<std::string>           lensModes_;
    std::vector<std::string>           scanRanges_;
    std::vector<std::string>           runModes_;
    std::map<std::string, std::string> paramMap_;
    std::map<int, std::string>         paramIndexes_;
    bool                               firstConnect_;
};

// Number of asyn parameters (asyn commands) this driver supports
// Currently hardcoding 100 additional device specific parameters
#define NUM_SPECS_PARAMS ((int)(&LAST_SPECS_PARAM - &FIRST_SPECS_PARAM + 100))


