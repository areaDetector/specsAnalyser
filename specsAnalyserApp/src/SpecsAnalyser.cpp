#include "SpecsAnalyser.h"

/**
 * A bit of C glue to make the config function available in the startup script (ioc shell) 
 */
extern "C"
{
  int specsAnalyserConfig(const char *portName, const char *driverPort, int maxBuffers, size_t maxMemory, int priority,	int stackSize)
  {
    new SpecsAnalyser(portName, driverPort, maxBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
  }

  int specsSetDebugLevel(const char *driver, const char *method, int debug)
  {
    SpecsAnalyser *sA;
    static const char *functionName = "specsSetDebugLevel";

    sA = (SpecsAnalyser*)findAsynPortDriver(driver);
    if (!sA){
      printf("%s: Error: port %s not found.", functionName, driver);
      return -1;
    }
    return sA->debugLevel(method, debug);
  }
}

/**
 * Function to run the analyser task within a separate thread in C++
 */
static void specsAnalyserTaskC(void *drvPvt)
{
	SpecsAnalyser *pPvt = (SpecsAnalyser *)drvPvt;
	pPvt->specsAnalyserTask();
}

/**
 * specsAnalyser destructor
 */
SpecsAnalyser::~SpecsAnalyser()
{
}

SpecsAnalyser::SpecsAnalyser(const char *portName, const char *driverPort, int maxBuffers, size_t maxMemory, int priority, int stackSize) :
  ADDriver(portName,
           1,
           NUM_SPECS_PARAMS,
           maxBuffers,
           maxMemory,
           asynEnumMask | asynFloat64ArrayMask,
           asynEnumMask | asynFloat64ArrayMask, /* No interfaces beyond those set in ADDriver.cpp */
           ASYN_CANBLOCK,
           1, //asynflags (CANBLOCK means separate thread for this driver)
           priority,
           stackSize) // thread priority and stack size (0=default)
{
  static const char *functionName = "SpecsAnalyser::SpecsAnalyser";
  int status = 0;

  // Setup flag to state this is our first connection
  firstConnect_ = true;

  // Initialise the debugger
  initDebugger(1);

  //Initialize non static data members
  portUser_  = NULL;
  strcpy(driverPort_, driverPort);
  
  // Create the epicsEvents for signalling to the SPECS Analyser task when acquisition starts
  this->startEventId_ = epicsEventCreate(epicsEventEmpty);
  if (!this->startEventId_){
    debug(functionName, "epicsEventCreate failure for start event");
    status = asynError;
  }

  // Create the epicsEvents for signalling to the SPECS Analyser task when acquisition stops
  this->stopEventId_ = epicsEventCreate(epicsEventEmpty);
  if (!this->stopEventId_){
    debug(functionName, "epicsEventCreate failure for stop event");
    status = asynError;
  }

  // Create all SPECS parameters
  createParam(SPECSConnectString,                   asynParamInt32,         &SPECSConnect_);
  createParam(SPECSConnectedString,                 asynParamInt32,         &SPECSConnected_);
  createParam(SPECSPauseAcqString,                  asynParamInt32,         &SPECSPauseAcq_);
  createParam(SPECSMsgCounterString,                asynParamInt32,         &SPECSMsgCounter_);
  createParam(SPECSServerNameString,                asynParamOctet,         &SPECSServerName_);
  createParam(SPECSProtocolVersionString,           asynParamInt32,         &SPECSProtocolVersion_);
  createParam(SPECSProtocolVersionMinorString,      asynParamInt32,         &SPECSProtocolVersionMinor_);
  createParam(SPECSProtocolVersionMajorString,      asynParamInt32,         &SPECSProtocolVersionMajor_);
  createParam(SPECSStartEnergyString,               asynParamFloat64,       &SPECSStartEnergy_);
  createParam(SPECSEndEnergyString,                 asynParamFloat64,       &SPECSEndEnergy_);
  createParam(SPECSRetardingRatioString,            asynParamFloat64,       &SPECSRetardingRatio_);
  createParam(SPECSKineticEnergyString,             asynParamFloat64,       &SPECSKineticEnergy_);
  createParam(SPECSStepWidthString,                 asynParamFloat64,       &SPECSStepWidth_);
  createParam(SPECSSamplesString,                   asynParamInt32,         &SPECSSamples_);
  createParam(SPECSSamplesIterationString,          asynParamInt32,         &SPECSSamplesIteration_);
  createParam(SPECSSnapshotValuesString,            asynParamInt32,         &SPECSSnapshotValues_);
  createParam(SPECSPassEnergyString,                asynParamFloat64,       &SPECSPassEnergy_);
  createParam(SPECSLensModeString,                  asynParamInt32,         &SPECSLensMode_);
  createParam(SPECSScanRangeString,                 asynParamInt32,         &SPECSScanRange_);
  createParam(SPECSCurrentSampleString,             asynParamInt32,         &SPECSCurrentSample_);
  createParam(SPECSPercentCompleteString,           asynParamInt32,         &SPECSPercentComplete_);
  createParam(SPECSRemainingTimeString,             asynParamFloat64,       &SPECSRemainingTime_);
  createParam(SPECSCurrentSampleIterationString,    asynParamInt32,         &SPECSCurrentSampleIteration_);
  createParam(SPECSPercentCompleteIterationString,  asynParamInt32,         &SPECSPercentCompleteIteration_);
  createParam(SPECSRemainingTimeIterationString,    asynParamFloat64,       &SPECSRemainingTimeIteration_);
  createParam(SPECSAcqSpectrumString,               asynParamFloat64Array,  &SPECSAcqSpectrum_);
  createParam(SPECSAcqImageString,                  asynParamFloat64Array,  &SPECSAcqImage_);

  createParam(SPECSRunModeString,                   asynParamInt32,         &SPECSRunMode_);
  createParam(SPECSDefineString,                    asynParamInt32,         &SPECSDefine_);
  createParam(SPECSValidateString,                  asynParamInt32,         &SPECSValidate_);

  createParam(SPECSNonEnergyChannelsString,         asynParamInt32,         &SPECSNonEnergyChannels_);
  createParam(SPECSNonEnergyUnitsString,            asynParamOctet,         &SPECSNonEnergyUnits_);
  createParam(SPECSNonEnergyMaxString,              asynParamFloat64,         &SPECSNonEnergyMax_);
  createParam(SPECSNonEnergyMinString,              asynParamFloat64,         &SPECSNonEnergyMin_);

  // Initialise SPECS parameters
  setIntegerParam(SPECSConnected_,                 0);
  setIntegerParam(SPECSPauseAcq_,                  0);
  setIntegerParam(SPECSMsgCounter_,                0);
  setIntegerParam(SPECSPercentComplete_,           0);
  setIntegerParam(SPECSCurrentSample_,             0);
  setIntegerParam(SPECSSnapshotValues_,            1);
  setIntegerParam(SPECSSamplesIteration_,          0);
  setIntegerParam(SPECSPercentCompleteIteration_,  0);
  setIntegerParam(SPECSCurrentSampleIteration_,    0);
  setDoubleParam(SPECSRemainingTime_,              0.0);
	setStringParam(ADManufacturer,                   "SPECS");

  if (status == asynSuccess){
    debug(functionName, "Starting up polling task....");
    // Create the thread that runs the acquisition
    status = (epicsThreadCreate("SpecsAnalyserTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)specsAnalyserTaskC,
                                this) == NULL);
    if (status){
      debug(functionName, "epicsTheadCreate failure for image task");
    }
  }

  if (status == asynSuccess){
    // Attempt connection
    status |= makeConnection();
    // Failed connection is not terminal here
    //if (status != asynSuccess){
    //  setIntegerParam(ADStatus, ADStatusError);
    //  setStringParam(ADStatusMessage, "Not connected to SPECS");
    //  callParamCallbacks();
    //  status = asynSuccess;
    //}

    // Read in the lens modes
    status |= readSpectrumParameter(SPECSLensMode_);
    // Read in the scan ranges
    status |= readSpectrumParameter(SPECSScanRange_);
    // Setup the run modes
    status |= readRunModes();
  }
  
  // Check if the status is bad.  If it is do our best to set the status record and message
  if (status != asynSuccess){
    setIntegerParam(ADStatus, ADStatusError);
    setStringParam(ADStatusMessage, "Failed to initialise - check connection");
    callParamCallbacks();
  }
}

asynStatus SpecsAnalyser::makeConnection()
{
  int status = asynSuccess;

  status = connect();

  if (status == asynSuccess){
    if (firstConnect_ == true){
      // This is our first connection attempt
      // Read in the device name
      if (status == asynSuccess){
        status = readDeviceVisibleName();
      }

      // Setup all of the available parameters obtained from the hardware
      if (status == asynSuccess){
        status = setupEPICSParameters();
      }

      // Read in the lists of enumerated types ready for use by records
      if (status == asynSuccess){
        // Read the number of non-energy channels (slices)
        int nonEnergyChannels = 0;
        getAnalyserParameter("NumNonEnergyChannels", nonEnergyChannels);
        setIntegerParam(SPECSNonEnergyChannels_, nonEnergyChannels);
      }

      // Publish any updates necessary
      callParamCallbacks();

      if (status == asynSuccess){
        // First connection established, set flag
        firstConnect_ = false;
      }
    }
  }

  return (asynStatus)status;
}

asynStatus SpecsAnalyser::connect()
{
  asynStatus status = asynSuccess;
  const char *functionName = "SpecsAnalyser::connect";

  // Connect our Asyn user to the low level port that is a parameter to this constructor
  status = asynPortConnect(driverPort_, 0, &portUser_, "\n", "\n");
  if (status != asynSuccess){
    debug(functionName, "Failed to connect to low level asynOctetSyncIO port", driverPort_);
    // Failure, set the connected status to 0
    setIntegerParam(SPECSConnected_,     0);
    setIntegerParam(ADStatus, ADStatusError);
    callParamCallbacks();
  } else {
    // Success, set the connected status to 1
    setIntegerParam(SPECSConnected_,     1);
    setStringParam(ADStatusMessage, "Connected to SPECS");
    setIntegerParam(ADStatus, ADStatusIdle);
    callParamCallbacks();
  }
  return status;
}

asynStatus SpecsAnalyser::disconnect()
{
  asynStatus status = asynSuccess;
  const char *functionName = "SpecsAnalyser::disconnect";
  int connected = 0;

  getIntegerParam(SPECSConnected_, &connected);
  // We can only force a disconnect if we are connected
  if (connected == 1){
    // Connect our Asyn user to the low level port that is a parameter to this constructor
    status = asynPortDisconnect(portUser_);
    if (status != asynSuccess){
      debug(functionName, "Failed to disconnect to low level asynOctetSyncIO port", driverPort_);
    }
  }
  return status;
}

/** 
 *  Task to listen to the SPECS prodigy application and update the higher level software.
 *
 *  This function runs the polling thread.
 *  It is started in the class constructor and must not return until the IOC stops.
 *
 */
void SpecsAnalyser::specsAnalyserTask()
{
  int status = asynSuccess;
  int acquire;
  int nbytes;
  int numImages        = 0;
  int numImagesCounter = 0;
  int imageCounter     = 0;
  int imageMode        = 0;
  int arrayCallbacks   = 0;
  int iterations       = 1;
  double acquireTime, acquirePeriod, delay;
  epicsTimeStamp startTime, endTime;
  double elapsedTime;
  std::map<std::string, std::string> data;
  NDArray *pImage;
  size_t dims[2];
  NDDataType_t dataType;
  epicsFloat64 *pNDImage = 0;
  epicsFloat64 *image = 0;
  epicsFloat64 *spectrum = 0;
  int nonEnergyChannels = 0;
  int energyChannels = 0;
  int currentDataPoint = 0;
  int numDataPoints = 0;
  int runMode = 0;
  const char *functionName = "SpecsAnalyser::specsAnalyserTask";

  debug(functionName, "Polling thread started");

  this->lock();
  while (1){
    getIntegerParam(ADAcquire, &acquire);
    // If we are not acquiring or encountered a problem then wait for a semaphore that is given when acquisition is started
    if (!acquire){
      // Reset the paused state in case the previous acquisition had been paused
      setIntegerParam(SPECSPauseAcq_, 0);
      // Only set the status message if we didn't encounter a problem last time, so we don't overwrite the error mesage
      if(!status){
        debug(functionName, "Waiting for the acquire command");
        setStringParam(ADStatusMessage, "Waiting for the acquire command");
        setIntegerParam(ADStatus, ADStatusIdle);
      }
      // Reset the counters
      setIntegerParam(ADNumExposuresCounter, 0);
      setIntegerParam(ADNumImagesCounter, 0);
      callParamCallbacks();

      // Release the lock while we wait for an event that says acquire has started, then lock again
      this->unlock();
      debug(functionName, "Waiting for acquire to start");
      status = epicsEventWait(this->startEventId_);
      this->lock();
      getIntegerParam(ADAcquire, &acquire);
      // Reset the paused state in case the previous acquisition had been paused
      setIntegerParam(SPECSPauseAcq_, 0);

      // Read the number of exposures (iterations)
      getIntegerParam(ADNumExposures, &iterations);

      // Read the number of non-energy channels ready to store the data
      status = getAnalyserParameter("NumNonEnergyChannels", nonEnergyChannels);
      if (status == asynSuccess){
        setIntegerParam(SPECSNonEnergyChannels_, nonEnergyChannels);
      }
      
      // Clear any stale data from previous acquisitions
      if (status == asynSuccess){
        sendSimpleCommand(SPECS_CMD_CLEAR);

        // Check the desired run mode
        getIntegerParam(SPECSRunMode_, &runMode);
        // Define the spectrum according to the run mode
        switch(runMode){
          case SPECS_RUN_FAT:
            // Define the fixed analyser transmission spectrum
            status = defineSpectrumFAT();
            break;
          case SPECS_RUN_SFAT:
            // Define the snapshot fixed analyser transmission spectrum
            status = defineSpectrumSFAT();
            break;
          case SPECS_RUN_FRR:
            // Define the fixed retarding ratio spectrum
            status = defineSpectrumFRR();
            break;
          case SPECS_RUN_FE:
            // Define the fixed energy spectrum
            status = defineSpectrumFE();
            break;
          default:
            // This is bad news, invalid mode of operation abort the scan
            debug(functionName, "Invalid mode of operation specified, aborting", runMode);
            break;
        }
      }

      // Now Validate the spectrum
      if (status == asynSuccess){
        status = validateSpectrum();
      }

      if (status == asynSuccess){
        // Read out the number of energy channels (samples)
        getIntegerParam(SPECSSamplesIteration_, &energyChannels);
        // Set the total number of samples for all of the iterations
        setIntegerParam(SPECSSamples_, (energyChannels*iterations));

        // If the mode is snapshot then we need to calculate the number of energy channels
        if (runMode == SPECS_RUN_SFAT){
          double start, end, width;
          getDoubleParam(SPECSStartEnergy_, &start);
          getDoubleParam(SPECSEndEnergy_, &end);
          getDoubleParam(SPECSStepWidth_, &width);

          // The number of channels is simply (end-start)/width + 1
          energyChannels = (int)floor(((end-start)/width)+0.5) + 1;
          setIntegerParam(SPECSSamplesIteration_, energyChannels);
          setIntegerParam(SPECSSamples_, (energyChannels*iterations));
        }

        // Free the image buffer if it has already been allocated
        if (image){
          free(image);
        }
        // Free the spectrum buffer if it has already been allocated
        if (spectrum){
          free(spectrum);
        }
        // Allocate the required memory to store the image
        image = (double *)malloc(nonEnergyChannels * energyChannels * sizeof(epicsFloat64));
        // Allocate the required memory to store the spectrum
        spectrum = (double *)malloc(energyChannels * sizeof(epicsFloat64));
        // Set the size parameters
        setIntegerParam(NDArraySizeX, energyChannels);
        dims[0] = energyChannels;
        setIntegerParam(NDArraySizeY, nonEnergyChannels);
        dims[1] = nonEnergyChannels;
        nbytes = (dims[0] * dims[1]) * sizeof(double);
        setIntegerParam(NDArraySize, nbytes);
        callParamCallbacks();
        // Get data type
        getIntegerParam(NDDataType, (int *) &dataType);
      }
    }

    // Check here for any bad status, if there is a problem abort the acquisition
    if (status != asynSuccess){
      acquire = 0;
      setIntegerParam(ADAcquire, acquire);
    } else {
      // No problems, continue with the acquisition

      // Initialise the image data points
      for (int x = 0; x < energyChannels*nonEnergyChannels; x++){
        image[x] = 0.0;
      }
      // Initialise the spectrum data points
      for (int x = 0; x < energyChannels; x++){
        spectrum[x] = 0.0;
      }

      // Allocate NDArray memory
      pImage = this->pNDArrayPool->alloc(2, dims, dataType, 0, NULL);

      // Reset the percent complete and current sample
      setIntegerParam(SPECSPercentCompleteIteration_, 0);
      setIntegerParam(SPECSCurrentSampleIteration_,   0);
      setIntegerParam(SPECSPercentComplete_,          0);
      setIntegerParam(SPECSCurrentSample_,            0);

      // We are acquiring.
      debug(functionName, "We are acquiring");

      epicsTimeGetCurrent(&startTime);

      // Get the exposure parameters
      getDoubleParam(ADAcquireTime, &acquireTime);
      getDoubleParam(ADAcquirePeriod, &acquirePeriod);

      // Get the acquisition parameters
      getIntegerParam(ADNumImages, &numImages);
      getIntegerParam(ADImageMode, &imageMode);

      // Set the status and message to notify user we are acquiring
      setIntegerParam(ADStatus, ADStatusInitializing);
      setStringParam(ADStatusMessage, "Executing pre-scan...");

      // Loop over the number of exposures (iterations)
      int iteration = 0;
      while((iteration < iterations) && (acquire == 1)){
        // Clear any stale data from previous acquisitions
        sendSimpleCommand(SPECS_CMD_CLEAR);
        // Send the start command
        sendSimpleCommand(SPECS_CMD_START);

        std::vector<double> values;
        currentDataPoint = 0;
        numDataPoints = 0;
        sendSimpleCommand(SPECS_CMD_GET_STATUS, &data);
        pNDImage = (double *)(pImage->pData);



        while (status == asynSuccess && (((data["ControllerState"] != "finished") && (data["ControllerState"] != "aborted") && (data["ControllerState"] != "error")) || (numDataPoints != currentDataPoint))){
          this->unlock();
          epicsThreadSleep(SPECS_UPDATE_RATE);
          this->lock();

          status = sendSimpleCommand(SPECS_CMD_GET_STATUS, &data);
          if (data.count("Code") > 0){
            data["ControllerState"] = "error";
          }
          debug(functionName, "Status", data);

          // Check the number of available data points, and retrieve any that are not already in memory
          readIntegerData(data, "NumberOfAcquiredPoints", numDataPoints);

          if (numDataPoints > currentDataPoint){
            setIntegerParam(ADStatus, ADStatusAcquire);
            setStringParam(ADStatusMessage, "Acquiring data...");

            // On the first data point read out the ordinate range and units (not available until now).
            if (currentDataPoint == 0) {
              // Wait for the dwell time to elapse to guarantee data will be ready to read out.
              epicsThreadSleep(acquireTime);
              readSpectrumDataInfo(SPECSOrdinateRange);
            }

            // If numDataPoints is greater than currentDataPoint then request the extra points
            readAcquisitionData(currentDataPoint, (numDataPoints-1), values);

            // Loop over the vector of newly acquired points and store in the correct image location
            int index = 0;
            debug(functionName, "Number of samples read", (numDataPoints-currentDataPoint));
            debug(functionName, "Received data points", (int)values.size());

            for (int y = 0; y < nonEnergyChannels; y++){

              // Quick and dirty fix to work around an issues with snapshot mode - if samples is set too high SPECS will misreport
              // the end energy, causing us to allocate too small a buffer and overflow it. This clause should detect this case and
              // abort before we overflow and crash.
              // Note the data we get will be nonsense if iterations > 1.
              // EW
              if (numDataPoints > energyChannels) {
        	  debug(functionName, "Data overflow: More data than number of allocated energyChannels received");
        	      // Sent the message to the analyser to stop
        	      sendSimpleCommand(SPECS_CMD_ABORT);
        	      status = asynError;
        	      setIntegerParam(ADAcquire, 0);
        	      setIntegerParam(ADStatus, ADStatusError);
        	      setStringParam(ADStatusMessage, "SPECS Controller Error, see log");
        	      break;
              }

              for (int x = currentDataPoint; x < numDataPoints; x++){
                // If this is the first iteration set the image values, otherwise add them to the current values
                if (iteration == 0){
                  pNDImage[(y * energyChannels) + x] = values[index];
                  image[(y * energyChannels) + x] = values[index];
                } else {
                  pNDImage[(y * energyChannels) + x] += values[index];
                  image[(y * energyChannels) + x] += values[index];
                }
                // Integrate for the spectrum
                spectrum[x] += values[index];
                index++;
              }
            }
            // Set the current point equal to the number of data points retrieved so far
            currentDataPoint = numDataPoints;

            // Notify listeners of the update to the spectrum data
            if (iteration == 0){
              doCallbacksFloat64Array(spectrum, currentDataPoint, SPECSAcqSpectrum_, 0);
            } else {
              // After the first iteration the spectrum is already full og data, so always post the full array
              doCallbacksFloat64Array(spectrum, energyChannels, SPECSAcqSpectrum_, 0);
            }
            // Notify listeners of the update to the image data
            doCallbacksFloat64Array(image, (energyChannels*nonEnergyChannels), SPECSAcqImage_, 0);
        
            // Set the percent complete and current sample number
            setIntegerParam(SPECSPercentCompleteIteration_, (int)(((currentDataPoint)*100)/energyChannels));
            setIntegerParam(SPECSPercentComplete_, (int)(((currentDataPoint+(iteration*energyChannels))*100)/(iterations*energyChannels)));
            setIntegerParam(SPECSCurrentSampleIteration_,   currentDataPoint);
            setIntegerParam(SPECSCurrentSample_,   currentDataPoint + (iteration*energyChannels));
            // Calculate the remaining time
            double dvalue = 0.0;
            getDoubleParam(ADAcquireTime, &dvalue);
            double time = (double)(energyChannels - (currentDataPoint)) * dvalue;
            setDoubleParam(SPECSRemainingTimeIteration_, time);
            double totalTime = (double)(energyChannels - currentDataPoint + ((iterations-(iteration+1)) * energyChannels)) * dvalue;
            setDoubleParam(SPECSRemainingTime_, totalTime);

            this->unlock();
            callParamCallbacks();
            this->lock();
          }
        }
        if (data["ControllerState"] == "error"){
          status = asynError;
          setIntegerParam(ADAcquire, 0);
          setIntegerParam(ADStatus, ADStatusError);
          setStringParam(ADStatusMessage, "SPECS Controller Error, see log");
        }

        // Check the acquisition status
        getIntegerParam(ADAcquire, &acquire);
        // Increase the iteration
        iteration++;
        // End of the iteration loop
      }


      pImage->dims[0].size = dims[0];
      pImage->dims[1].size = dims[1];

      // Set a bit of areadetector image/frame statistics...
      getIntegerParam(ADNumImages, &numImages);
      getIntegerParam(ADImageMode, &imageMode);
      getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
      getIntegerParam(NDArrayCounter, &imageCounter);
      getIntegerParam(ADNumImagesCounter, &numImagesCounter);
      imageCounter++;
      numImagesCounter++;
      setIntegerParam(NDArrayCounter, imageCounter);
      setIntegerParam(ADNumImagesCounter, numImagesCounter);

      pImage->uniqueId = imageCounter;
      pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

      // Get any attributes that have been defined for this driver
      this->getAttributes(pImage->pAttributeList);

      if (arrayCallbacks){
        // Must release the lock here, or we can get into a deadlock, because we can
        // block on the plugin lock, and the plugin can be calling us
        this->unlock();
        debug(functionName, "Calling NDArray callback");
        doCallbacksGenericPointer(pImage, NDArrayData, 0);
        this->lock();
      }

      // Free the image buffer
      pImage->release();

      if (status != asynError){
        // Check to see if acquisition is complete
        if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))){
          setIntegerParam(ADAcquire, 0);
          debug(functionName, "Acquisition completed");
        }
        // Call the callbacks to update any changes
        callParamCallbacks();
        getIntegerParam(ADAcquire, &acquire);

        // If we are acquiring then sleep for the acquire period minus elapsed time.
        if (acquire){
          epicsTimeGetCurrent(&endTime);
          elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
          delay = acquirePeriod - elapsedTime;
          if (delay >= 0.0){
            debug(functionName, "Delay", delay);
            // We set the status to indicate we are in the period delay
            setIntegerParam(ADStatus, ADStatusWaiting);
            callParamCallbacks();
            this->unlock();
            status = epicsEventWaitWithTimeout(this->stopEventId_, delay);
            this->lock();
          }
        }
      }
    }
  }
}

/** Called when asyn clients call pasynEnum->read().
  * The base class implementation simply prints an error message.
  * @param pasynUser - pasynUser structure that encodes the reason and address.
  * @param strings - Array of string pointers.
  * @param values - Array of values
  * @param severities - Array of severities
  * @param nElements - Size of value array
  * @param nIn - Number of elements actually returned */
asynStatus SpecsAnalyser::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn)
{
  const char *functionName = "SpecsAnalyser::readEnum";
  asynStatus status = asynSuccess;
  int function = pasynUser->reason;
  size_t index;

  if (function == SPECSLensMode_){
    for (index = 0; ((index < (size_t)lensModes_.size()) && (index < nElements)); index++){
      if (strings[index]){
        free(strings[index]);
      }
      strings[index] = epicsStrDup(lensModes_[index].c_str());
      debug(functionName, "Reading lens mode", strings[index]);
      values[index] = index;
      severities[index] = 0;
    }
    *nIn = index;
  } else if (function == SPECSScanRange_){
    for (index = 0; ((index < (size_t)scanRanges_.size()) && (index < nElements)); index++){
      if (strings[index]){
        free(strings[index]);
      }
      strings[index] = epicsStrDup(scanRanges_[index].c_str());
      debug(functionName, "Reading scan range", strings[index]);
      values[index] = index;
      severities[index] = 0;
    }
    *nIn = index;
  } else if (function == SPECSRunMode_){
    for (index = 0; ((index < (size_t)runModes_.size()) && (index < nElements)); index++){
      if (strings[index]){
        free(strings[index]);
      }
      strings[index] = epicsStrDup(runModes_[index].c_str());
      debug(functionName, "Reading run mode", strings[index]);
      values[index] = index;
      severities[index] = 0;
    }
    *nIn = index;
  } else {
    *nIn = 0;
    status = asynError;
  }
  return status;
}

/**
 * Called when asyn clients call pasynInt32->write().
 * Write integer value to the drivers parameter table.
 *
 * @param pasynUser - Pointer to the asyn user structure.
 * @param value - The new value to write
 */
asynStatus SpecsAnalyser::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  const char *functionName = "SpecsAnalyser::writeInt32";
  int status = asynSuccess;
  int function = pasynUser->reason;
  int oldValue;

  // parameters for functions
  int adstatus;

  getIntegerParam(function, &oldValue);
  setIntegerParam(function, value);
  getIntegerParam(ADStatus, &adstatus);

  if (function == ADAcquire){
    if (value && (adstatus == ADStatusIdle || adstatus == ADStatusError || adstatus == ADStatusAborted)){
      // Send an event to wake up the electronAnalyser task.
      epicsEventSignal(this->startEventId_);
    }
    if (!value && (adstatus != ADStatusIdle)){
      // Reset the paused state
      setIntegerParam(SPECSPauseAcq_, 0);
      // Stop acquiring ( abort any hardware processings )
      epicsEventSignal(this->stopEventId_);
      // Sent the message to the analyser to stop
      sendSimpleCommand(SPECS_CMD_ABORT);
    }
  } else if (function == SPECSConnect_){
    int connected = 0;
    // Get the connected status
    getIntegerParam(SPECSConnected_, &connected);
    // Only proceed if we are not connected
    if (connected == 0){
      status = makeConnection();

      // Read in the lens modes
      status |= readSpectrumParameter(SPECSLensMode_);

      // Read in the scan ranges
      status |= readSpectrumParameter(SPECSScanRange_);

      // Set up the run modes
      status |= readRunModes();
    }
  } else if (function == SPECSDefine_){
    // A new definition is requested so check the value of the mode and send the required command
    int runMode = 0;
    getIntegerParam(SPECSRunMode_, &runMode);
    switch (runMode){
      case SPECS_RUN_FAT:
        status = defineSpectrumFAT();
        break;
      case SPECS_RUN_SFAT:
        status = defineSpectrumSFAT();
        break;
      case SPECS_RUN_FRR:
        status = defineSpectrumFRR();
        break;
      case SPECS_RUN_FE:
        status = defineSpectrumFE();
        break;
    }
  } else if (function == SPECSValidate_){
    // Validate the currently defined spectrum
    status = validateSpectrum();
  } else if (function == SPECSPauseAcq_){
    // Check for a pause request
    int acquire = 0;
    getIntegerParam(ADAcquire, &acquire);
    if (value == 1){
      // Check we are not already paused and are acquiring
      if (oldValue == 0 && acquire == 1){
        // Send the pause command
        sendSimpleCommand(SPECS_CMD_PAUSE);
        debug(functionName, "Pausing the current acquisition");
        setStringParam(ADStatusMessage, "Acquisition paused");
      } else {
        // Do not act on the pause, we are not in the correct state
        setIntegerParam(SPECSPauseAcq_, oldValue);
        debug(functionName, "Unable to pause in the current state");
      }
    } else if (value == 0){
      // Check we are pause and acquiring
      if (oldValue == 1 && acquire == 1){
        // Send the resume command
        sendSimpleCommand(SPECS_CMD_RESUME);
        debug(functionName, "Resuming the paused acquisition");
        setStringParam(ADStatusMessage, "Acquiring data...");
      } else {
        // Do not act on the resume, we are not in the correct state
        setIntegerParam(SPECSPauseAcq_, oldValue);
        debug(functionName, "Unable to resume in the current state");
      }
    }
  } else {
    // Check if the function is one of our stored parameter index values
    if (paramIndexes_.count(function) == 1){
      // This means the parameter was read out from the SPECS hardware at startup
      debug(functionName, "Update request of parameter", paramIndexes_[function]);
      debug(functionName, "New Value", value);
      status = setAnalyserParameter(paramMap_[paramIndexes_[function]], value);
      int newValue = 0;
      if (status == asynSuccess){
        status = getAnalyserParameter(paramMap_[paramIndexes_[function]], newValue);
      }
      if (status == asynSuccess){
        setIntegerParam(function, newValue);
      } else {
        setIntegerParam(function, oldValue);
      }
    }
  }

  this->lock();
  // Do callbacks so higher layers see any changes
  callParamCallbacks();
  this->unlock();
  if (status){
    debug(functionName, "Error, status", status);
    debug(functionName, "Error, function", function);
    debug(functionName, "Error, value", value);
  } else {
    debug(functionName, "Function", function);
    debug(functionName, "Value", value);
  }
  return (asynStatus)status;
}

/**
 * Called when asyn clients call pasynFloat64->write().
 * Write integer value to the drivers parameter table.
 *
 * @param pasynUser - Pointer to the asyn user structure.
 * @param value - The new value to write
 */
asynStatus SpecsAnalyser::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  const char *functionName = "SpecsAnalyser::writeFloat64";
  asynStatus status = asynSuccess;
  int function = pasynUser->reason;
  double oldValue;

  // parameters for functions
  int adstatus;

  getDoubleParam(function, &oldValue);
  setDoubleParam(function, value);
  getIntegerParam(ADStatus, &adstatus);

  // Check if the function is one of our stored parameter index values
  if (paramIndexes_.count(function) == 1){
    // This means the parameter was read out from the SPECS hardware at startup
    debug(functionName, "Update request of parameter", paramIndexes_[function]);
    debug(functionName, "New Value", value);
    status = setAnalyserParameter(paramMap_[paramIndexes_[function]], value);
    double newValue = 0.0;
    if (status == asynSuccess){
      status = getAnalyserParameter(paramMap_[paramIndexes_[function]], newValue);
    }
    if (status == asynSuccess){
      setDoubleParam(function, newValue);
    } else {
      setDoubleParam(function, oldValue);
    }
  }

  this->lock();
  // Do callbacks so higher layers see any changes
  callParamCallbacks();
  this->unlock();
  if (status){
    debug(functionName, "Error, status", status);
    debug(functionName, "Error, function", function);
    debug(functionName, "Error, value", value);
  } else {
    debug(functionName, "Function", function);
    debug(functionName, "Value", (double)value);
  }
  return status;
}

/**
 * Validate the currently defined spectrum.  This validation must take
 * place before a new acquisition can begin.  The values defined for the
 * acquisition can actually be altered by the validation so the read back
 * values are written back into the parameters.
 */
asynStatus SpecsAnalyser::validateSpectrum()
{
  static const char *functionName = "SpecsAnalyser::validateSpectrum";

  asynStatus status = asynSuccess;
  std::stringstream command;
  std::map<std::string, std::string> data;
  std::stringstream decode;
  std::string lookup;
  double dvalue = 0.0;
  int ivalue = 0;
  unsigned int index = 0;

  debug(functionName, "Sending command", SPECS_CMD_VALIDATE);
  // Send the command and get the reply
  status = sendSimpleCommand(SPECS_CMD_VALIDATE, &data);
  // Check the command sent OK
  if (status == asynSuccess){
    debug(functionName, "Spectrum validated", data);
    // Use the returned values to set the EPICS parameters accordingly
    // Check the StartEnergy
    decode.str(data["StartEnergy"]);
    decode >> dvalue;
    decode.clear();
    setDoubleParam(SPECSStartEnergy_, dvalue);
    // Check the EndEnergy
    decode.str(data["EndEnergy"]);
    decode >> dvalue;
    decode.clear();
    setDoubleParam(SPECSEndEnergy_, dvalue);
    // Check the StepWidth
    decode.str(data["StepWidth"]);
    decode >> dvalue;
    decode.clear();
    setDoubleParam(SPECSStepWidth_, dvalue);
    // Check the Samples
    decode.str(data["Samples"]);
    decode >> ivalue;
    decode.clear();
    setIntegerParam(SPECSSamplesIteration_, ivalue);
    // Check the DwellTime
    decode.str(data["DwellTime"]);
    decode >> dvalue;
    decode.clear();
    setDoubleParam(ADAcquireTime, dvalue);
    // Check the PassEnergy
    decode.str(data["PassEnergy"]);
    decode >> dvalue;
    decode.clear();
    setDoubleParam(SPECSPassEnergy_, dvalue);
    // Check the LensMode
    lookup = data["LensMode"];
    cleanString(lookup, "\"");
    ivalue = -1;
    index = 0;
    while ((index < lensModes_.size()) && (ivalue == -1)){
      if (lensModes_[index] == lookup){
        ivalue = index;
      }
      index++;
    }
    setIntegerParam(SPECSLensMode_, ivalue);
    // Check the ScanRange
    lookup = data["ScanRange"];
    cleanString(lookup, "\"");
    ivalue = -1;
    index = 0;
    while ((index < scanRanges_.size()) && (ivalue == -1)){
      if (scanRanges_[index] == lookup){
        ivalue = index;
      }
      index++;
    }
    setIntegerParam(SPECSScanRange_, ivalue);

    // ***** WORKAROUND FOR FIXED ENERGY START AND END *******
    int runMode = 0;
    getIntegerParam(SPECSRunMode_, &runMode);
    if (runMode == SPECS_RUN_FE){
      double ke = 0.0;
      double pe = 0.0;
      getDoubleParam(SPECSKineticEnergy_ , &ke);
      getDoubleParam(SPECSPassEnergy_, &pe);
      setDoubleParam(SPECSStartEnergy_, ke-(0.1*pe));
      setDoubleParam(SPECSEndEnergy_, ke+(0.1*pe));
    }
    // ***** END OF WORKAROUND *******

    this->lock();
    // Do callbacks so higher layers see any changes
    callParamCallbacks();
    this->unlock();
  }
  return status;
}

/**
 * Define a Fixed Analyser Transmission spectrum.  All of the desired parameters should
 * have already been set, this method constructs the required command string from those
 * parameters and sends it to the device.
 */
asynStatus SpecsAnalyser::defineSpectrumFAT()
{
  static const char *functionName = "SpecsAnalyser::defineSpectrumFAT";

  asynStatus status = asynSuccess;
  std::stringstream command;
  std::string response = "";
  std::map<std::string, std::string> data;
  double dvalue = 0.0;
  int ivalue = 0;

  // Construct the correct command string format
  command << SPECS_CMD_DEFINE_FAT << " ";
  // Add the StartEnergy
  getDoubleParam(SPECSStartEnergy_ , &dvalue);
  command << "StartEnergy:" << dvalue << " ";
  // Add the EndEnergy
  getDoubleParam(SPECSEndEnergy_ , &dvalue);
  command << "EndEnergy:" << dvalue << " ";
  // Add the StepWidth
  getDoubleParam(SPECSStepWidth_ , &dvalue);
  command << "StepWidth:" << dvalue << " ";
  // Add the DwellTime
  getDoubleParam(ADAcquireTime , &dvalue);
  command << "DwellTime:" << dvalue << " ";
  // Add the PassEnergy
  getDoubleParam(SPECSPassEnergy_ , &dvalue);
  command << "PassEnergy:" << dvalue << " ";
  // Add the LensMode
  getIntegerParam(SPECSLensMode_ , &ivalue);
  command << "LensMode:\"" << lensModes_[ivalue] << "\" ";
  // Add the ScanRange
  getIntegerParam(SPECSScanRange_ , &ivalue);
  command << "ScanRange:\"" << scanRanges_[ivalue] << "\"";

  debug(functionName, "Sending command", command.str());
  // Send the command and get the reply
  status = sendSimpleCommand(command.str(), &data);
  return status;
}

/**
 * Define a Snapshot Fixed Analyser Transmission spectrum.  All of the desired parameters should
 * have already been set, this method constructs the required command string from those
 * parameters and sends it to the device.
 */
asynStatus SpecsAnalyser::defineSpectrumSFAT()
{
  static const char *functionName = "SpecsAnalyser::defineSpectrumSFAT";

  asynStatus status = asynSuccess;
  std::stringstream command;
  std::string response = "";
  std::map<std::string, std::string> data;
  double dvalue = 0.0;
  int ivalue = 0;

  // Construct the correct command string format
  command << SPECS_CMD_DEFINE_SFAT << " ";
  // Add the StartEnergy
  getDoubleParam(SPECSStartEnergy_ , &dvalue);
  command << "StartEnergy:" << dvalue << " ";
  // Add the EndEnergy
  getDoubleParam(SPECSEndEnergy_ , &dvalue);
  command << "EndEnergy:" << dvalue << " ";
  // Add the Samples
  getIntegerParam(SPECSSnapshotValues_ , &ivalue);
  command << "Samples:" << ivalue << " ";
  // Add the DwellTime
  getDoubleParam(ADAcquireTime , &dvalue);
  command << "DwellTime:" << dvalue << " ";
  // Add the LensMode
  getIntegerParam(SPECSLensMode_ , &ivalue);
  command << "LensMode:\"" << lensModes_[ivalue] << "\" ";
  // Add the ScanRange
  getIntegerParam(SPECSScanRange_ , &ivalue);
  command << "ScanRange:\"" << scanRanges_[ivalue] << "\"";

  debug(functionName, "Sending command", command.str());
  // Send the command and get the reply
  status = sendSimpleCommand(command.str(), &data);
  return status;
}

asynStatus SpecsAnalyser::defineSpectrumFRR()
{
  static const char *functionName = "SpecsAnalyser::defineSpectrumFRR";

  asynStatus status = asynSuccess;
  std::stringstream command;
  std::string response = "";
  std::map<std::string, std::string> data;
  double dvalue = 0.0;
  int ivalue = 0;
  
  // Construct the correct command string format
  command << SPECS_CMD_DEFINE_FRR << " ";
  // Add the StartEnergy
  getDoubleParam(SPECSStartEnergy_ , &dvalue);
  command << "StartEnergy:" << dvalue << " ";
  // Add the EndEnergy
  getDoubleParam(SPECSEndEnergy_ , &dvalue);
  command << "EndEnergy:" << dvalue << " ";
  // Add the StepWidth
  getDoubleParam(SPECSStepWidth_ , &dvalue);
  command << "StepWidth:" << dvalue << " ";
  // Add the DwellTime
  getDoubleParam(ADAcquireTime , &dvalue);
  command << "DwellTime:" << dvalue << " ";
  // Add the RetardingRatio
  getDoubleParam(SPECSRetardingRatio_ , &dvalue);
  command << "RetardingRatio:" << dvalue << " ";
  // Add the LensMode
  getIntegerParam(SPECSLensMode_ , &ivalue);
  command << "LensMode:\"" << lensModes_[ivalue] << "\" ";
  // Add the ScanRange
  getIntegerParam(SPECSScanRange_ , &ivalue);
  command << "ScanRange:\"" << scanRanges_[ivalue] << "\"";

  debug(functionName, "Sending command", command.str());
  // Send the command and get the reply
  status = sendSimpleCommand(command.str(), &data);
  return status;
}

asynStatus SpecsAnalyser::defineSpectrumFE()
{
  static const char *functionName = "SpecsAnalyser::defineSpectrumFE";

  asynStatus status = asynSuccess;
  std::stringstream command;
  std::string response = "";
  std::map<std::string, std::string> data;
  double dvalue = 0.0;
  int ivalue = 0;
  
  // Construct the correct command string format
  command << SPECS_CMD_DEFINE_FE << " ";
  // Add the KineticEnergy
  getDoubleParam(SPECSKineticEnergy_ , &dvalue);
  command << "KinEnergy:" << dvalue << " ";
  // Add the Samples
  getIntegerParam(SPECSSnapshotValues_ , &ivalue);
  command << "Samples:" << ivalue << " ";
  // Add the DwellTime
  getDoubleParam(ADAcquireTime , &dvalue);
  command << "DwellTime:" << dvalue << " ";
  // Add the PassEnergy
  getDoubleParam(SPECSPassEnergy_ , &dvalue);
  command << "PassEnergy:" << dvalue << " ";
  // Add the LensMode
  getIntegerParam(SPECSLensMode_ , &ivalue);
  command << "LensMode:\"" << lensModes_[ivalue] << "\" ";
  // Add the ScanRange
  getIntegerParam(SPECSScanRange_ , &ivalue);
  command << "ScanRange:\"" << scanRanges_[ivalue] << "\"";

  debug(functionName, "Sending command", command.str());
  // Send the command and get the reply
  status = sendSimpleCommand(command.str(), &data);
  return status;
}

asynStatus SpecsAnalyser::readAcquisitionData(int startIndex, int endIndex, std::vector<double> &values)
{
  static const char *functionName = "SpecsAnalyser::readAcquisitionData";

  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;

  debug(functionName, "Reading data from", startIndex);
  debug(functionName, "Reading data to", endIndex);
  // First construct the command string
  std::stringstream cmd;
  cmd << SPECS_CMD_GET_DATA << " FromIndex:" << startIndex << " ToIndex:" << endIndex;
  // Now send the command and retrieve the data
  status = sendSimpleCommand(cmd.str(), &data);

  if (status == asynSuccess){
    // Decode the data into the individual values
    values.clear();
    std::string dataString = data["Data"];
    cleanString(dataString, "[]");
    std::string dataItem;
    size_t loc = 0;
    double dval = 0.0;
    // Loop over the string searching for commas
    while ((loc = dataString.find_first_of(",")) != std::string::npos){
      // At each occurrance of a comma, the text leading up to it is the value
      std::stringstream dataItem(dataString.substr(0, loc));
      dataItem >> dval;
      values.push_back(dval);
      // Shorten the string by removing the processed item
      dataString = dataString.substr(loc+1);
    }
    // Don't forget the final data point
    std::stringstream finalItem(dataString);
    finalItem >> dval;
    values.push_back(dval);
  }
  return status;
}

/**
 * Method for sending a simple command (no parameters) that will check the
 * error status and do the right thing.
 */
asynStatus SpecsAnalyser::sendSimpleCommand(const std::string& command, std::map<std::string, std::string> *data)
{
  static const char *functionName = "SpecsAnalyser::sendSimpleCommand";

  asynStatus status = asynSuccess;
  std::string response = "";
  std::map<std::string, std::string> ldata;

  debug(functionName, "Sending command", command);
  // Send the command and get the reply
  status = commandResponse(command, response, ldata);
  // Check the command sent OK
  if (status == asynSuccess){
    // Check the response for any errors
    if (response == SPECS_ERROR_STRING){
      debug(functionName, "Unable to successfully issue the command", command);
      std::string msg = ldata["Code"] + ": " + ldata["Message"];
      debug(functionName, "Returned error", msg);
      setStringParam(ADStatusMessage, msg.c_str());
      status = asynError;
    }
  }
  if (data != NULL){
    *data = ldata;
  }
  return status;
}

/**
 * This method queries the analyser for its device name to populate
 * the ADModel parameter
 */
asynStatus SpecsAnalyser::readDeviceVisibleName()
{
  const char * functionName = "SpecsAnalyser::readDeviceVisibleName";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  
  // alyzerParameterNames to get the list of parameters
  status = sendSimpleCommand(SPECS_CMD_GET_VISNAME, &data);
  debug(functionName, "Returned parameter map", data);
  if (status == asynSuccess){
    std::string nameString = data["AnalyzerVisibleName"];
    cleanString(nameString, "\"");
    setStringParam(ADModel, nameString.c_str());
    callParamCallbacks();
  }
  return status;
}

/**
 * This method sets up the analyser parameters that are not known prior to
 * connecting to the device.
 */
asynStatus SpecsAnalyser::setupEPICSParameters()
{
  const char * functionName = "SpecsAnalyser::setupEPICSParameters";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::vector<std::string> names;
  std::locale loc;

  // First call getAllAnalyzerParameterNames to get the list of parameters
  status = sendSimpleCommand(SPECS_CMD_GET_NAMES, &data);
  debug(functionName, "Returned parameter map", data);
  if (status == asynSuccess){
    std::string nameString = data["ParameterNames"];
    // Clean any [] and whitespace from the ends of the string
    cleanString(nameString, "[]");
    // Now loop over the remainder of the string and search for names
    // Create a small state machine for this
    int insideQuotes = 0;
    std::string name;
    std::string rawname;
    for (unsigned int index = 0; index < nameString.length(); index++){
      // Get the current character
      std::string cc = nameString.substr(index, 1);
      // Are we inside quotes (reading out a name)
      if (insideQuotes == 1){
        // Is the character a space
        if (cc == " " || cc == "[" || cc == "]" || cc == "/"){
          // We do not specials in the parameter name but we do need them in the SPECS name
          rawname += cc;
        } else if (cc == "\""){
          debug(functionName, "Parameter found", name);
          // The parameter name is complete, record it
          paramMap_[name] = rawname;
          // Reset the name variable
          name = "";
          // Reset the rawname variable
          rawname = "";
          // Notify we are no longer inside quotes
          insideQuotes = 0;
        } else {
          // This is part of the parameter name, record it
          name += std::toupper(cc[0], loc);
          rawname += cc;
        }
      } else {
        // Here we are only really looking for opening quotation marks
        std::string cc = nameString.substr(index, 1);
        if (cc == "\""){
          insideQuotes = 1;
        }
      }
    }
    debug(functionName, "Constructed parameter map", paramMap_);
    // Now loop over paramMap_, find the type and value for each, and create the asyn parameter
    std::map<std::string, std::string>::iterator iter;
    SPECSValueType_t valType;
    int ivalue = 0;
    double dvalue = 0.0;
    std::string svalue = "";
    bool bvalue = false;
    for (iter = paramMap_.begin(); iter != paramMap_.end(); iter++){
      if (status == asynSuccess){
        status = getAnalyserParameterType(iter->second, valType);
        if (status == asynSuccess){
          int index = 0;
          // Create the parameter and record the index for future reference
          switch (valType)
          {
            case SPECSTypeInteger:
              // Create an integer parameter
              createParam(iter->first.c_str(), asynParamInt32, &index);
              // Read the value and set it
              status = getAnalyserParameter(iter->second, ivalue);
              if (status == asynSuccess){
                setIntegerParam(index, ivalue);
              }
              // Store the index to parameter name
              paramIndexes_[index] = iter->first;
              break;
            case SPECSTypeDouble:
              // Create a double parameter
              createParam(iter->first.c_str(), asynParamFloat64, &index);
              // Read the value and set it
              status = getAnalyserParameter(iter->second, dvalue);
              if (status == asynSuccess){
                setDoubleParam(index, dvalue);
              }
              // Store the index to parameter name
              paramIndexes_[index] = iter->first;
              break;
            case SPECSTypeString:
              // Create a string parameter
              createParam(iter->first.c_str(), asynParamOctet, &index);
              // Read the value and set it
              status = getAnalyserParameter(iter->second, svalue);
              if (status == asynSuccess){
                setStringParam(index, svalue.c_str());
              }
              // Store the index to parameter name
              paramIndexes_[index] = iter->first;
              break;
            case SPECSTypeBool:
              debug(functionName, "Got here, ", status);
              // Create an integer parameter
              createParam(iter->first.c_str(), asynParamInt32, &index);
              // Read the value and set it
              status = getAnalyserParameter(iter->second, bvalue);
              if (status == asynSuccess) {
        	  debug(functionName, "Success");
        	  setIntegerParam(index, bvalue);
              }
              // Store the index to parameter name
              paramIndexes_[index] = iter->first;
              break;
            default:
              // This is bad, all cases should have been covered
              // TODO: Raise this as an error, print debug
              break;
          }
        }
      }
    }
  }
  return status;
}

asynStatus SpecsAnalyser::getAnalyserParameterType(const std::string& name, SPECSValueType_t &type)
{
  const char * functionName = "SpecsAnalyser::getAnalyserParameterType";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_GET_INFO;
  
  debug(functionName, "Parameter", name);
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"", &data);
  if (status == asynSuccess){
    debug(functionName, "Parameter value type", data["ValueType"]);
    if (data["ValueType"] == SPECS_TYPE_DOUBLE){
      type = SPECSTypeDouble;
    } else if (data["ValueType"] == SPECS_TYPE_INTEGER){
      type = SPECSTypeInteger;
    } else if (data["ValueType"] == SPECS_TYPE_STRING){
      type = SPECSTypeString;
    } else if (data["ValueType"] == SPECS_TYPE_BOOL)
      type = SPECSTypeBool;
  }
  return status;
}

asynStatus SpecsAnalyser::getAnalyserParameter(const std::string& name, int &value)
{
  const char * functionName = "SpecsAnalyser::getAnalyserParameter";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_GET_VALUE;
  
  debug(functionName, "Parameter", name);
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"", &data);
  if (status == asynSuccess){
    if (data["Value"] == "\"false\""){
      value = 0;
    } else if (data["Value"] == "\"true\""){
      value = 1;
    } else {
      status = readIntegerData(data, "Value", value);
    }
  }
  return status;
}

asynStatus SpecsAnalyser::getAnalyserParameter(const std::string& name, double &value)
{
  const char * functionName = "SpecsAnalyser::getAnalyserParameter";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_GET_VALUE;

  debug(functionName, "Parameter", name);
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"", &data);
  if (status == asynSuccess){
    status = readDoubleData(data, "Value", value);
  }
  return status;
}

asynStatus SpecsAnalyser::getAnalyserParameter(const std::string& name, std::string &value)
{
  const char * functionName = "SpecsAnalyser::getAnalyserParameter";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_GET_VALUE;

  debug(functionName, "Parameter", name);
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"", &data);
  if (status == asynSuccess){
    value = data["Value"];
    cleanString(value, "\"");
  }
  return status;
}

asynStatus SpecsAnalyser::getAnalyserParameter(const std::string& name, bool &value)
{
  const char * functionName = "SpecsAnalyser::getAnalyserParameter (bool)";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_GET_VALUE;

  debug(functionName, "Parameter", name);
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"", &data);
  if (status == asynSuccess){
      if (data["Value"] == "\"false\""){
        value = true;
      } else if (data["Value"] == "\"true\""){
        value = false;
      } else {
	debug(functionName, "Invalid value returned for bool parameter: ", data["Value"]);
	status = asynError;
      }
  }
  return status;
}

asynStatus SpecsAnalyser::setAnalyserParameter(const std::string& name, int value)
{
  const char * functionName = "SpecsAnalyser::setAnalyserParameter";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_SET_VALUE;
  std::stringstream svalue;
  
  debug(functionName, "Parameter", name);
  debug(functionName, "Value", value);
  svalue << value;
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"" + " Value:" + svalue.str(), &data);
  return status;
}

asynStatus SpecsAnalyser::setAnalyserParameter(const std::string& name, double value)
{
  const char * functionName = "SpecsAnalyser::setAnalyserParameter";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_SET_VALUE;
  std::stringstream svalue;
  
  debug(functionName, "Parameter", name);
  debug(functionName, "Value", value);
  svalue << value;
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"" + " Value:" + svalue.str(), &data);
  return status;
}

asynStatus SpecsAnalyser::setAnalyserParameter(const std::string& name, std::string value)
{
  const char * functionName = "SpecsAnalyser::setAnalyserParameter";
  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_SET_VALUE;
  
  debug(functionName, "Parameter", name);
  debug(functionName, "Value", value);
  status = sendSimpleCommand(cmd + " ParameterName:\"" + name + "\"" + " Value:" + value, &data);
  return status;
}

asynStatus SpecsAnalyser::readIntegerData(std::map<std::string, std::string> data, const std::string& name, int &value)
{
  const char * functionName = "SpecsAnalyser::readIntegerData";
  asynStatus status = asynSuccess;

  std::stringstream integerValueStream(data[name]);
  integerValueStream >> value;
  if (integerValueStream.fail()){
    status = asynError;
    debug(functionName, "Failed to decode stream into integer", integerValueStream.str());
  }
  return status;
}

asynStatus SpecsAnalyser::readDoubleData(std::map<std::string, std::string> data, const std::string& name, double &value)
{
  const char * functionName = "SpecsAnalyser::readDoubleData";
  asynStatus status = asynSuccess;

  std::stringstream doubleValueStream(data[name]);
  doubleValueStream >> value;
  if (doubleValueStream.fail()){
    status = asynError;
    debug(functionName, "Failed to decode stream into double", doubleValueStream.str());
  }
  return status;
}

asynStatus SpecsAnalyser::readSpectrumParameter(int param){
  const char * functionName = "SpecsAnalyser::readSpectrumParameter";
  asynStatus status = asynSuccess;

  std::vector<std::string>* store;
  std::string paramName;

  if (param == SPECSLensMode_) {
      store = &lensModes_;
      paramName = "\"LensMode\"";
      debug(functionName, "Reading Lens Modes...");
  } else if (param == SPECSScanRange_) {
      store = &scanRanges_;
      paramName = "\"ScanRange\"";
      debug(functionName, "Reading Scan Ranges...");
  } else {
      debug(functionName, "Unrecognised spectrum parameter", param);
      return asynError;
  }

  // Clear out any existing values
  store->clear();

  // Get the string containing the lens modes.
  std::map<std::string, std::string> data;
  std::string cmd = SPECS_CMD_GET_SPECTRUM;
  status = sendSimpleCommand(cmd + " ParameterName:" + paramName, &data);

  if (status == asynSuccess) {
      debug(functionName, "Got params:", data["Values"]);

      if (data.find("Values") != data.end()) {
	  std::stringstream stringStream(data["Values"]);
	  std::string token;

	  // Pull out the individual tokens and clean them up
	  while(!stringStream.eof()) {
	      std::getline(stringStream, token, ',');
	      debug(functionName, token);
	      // std::remove moves all instances of character to end of string and returns position of
	      // first removed character in new string. erase then truncates the string at that point.
	      token.erase(std::remove(token.begin(), token.end(), '\"'), token.end());
	      token.erase(std::remove(token.begin(), token.end(), '['), token.end());
	      token.erase(std::remove(token.begin(), token.end(), ']'), token.end());
	      debug(functionName, token);
	      store->push_back(token);
	  }
      } else {
	  debug(functionName, "No values returned for param ", paramName);
	  status = asynError;
      }
  }

  // We need to make sure that enum field always contains at least one entry to avoid EDM problems
  // caused by the SDEF field not being set in the mbbi/mbbo records.
  if (status != asynSuccess) {
      debug(functionName, "Failed to read ", paramName);
      store->push_back("Not connected");
  }

  // Push the discovered changes back up to EPICS
  // Do this regardless of the value of status
  char *strings[store->size()];
  int values[store->size()];
  int severities[store->size()];
  for (size_t index = 0; ((index < (size_t)store->size())); index++){
    strings[index] = epicsStrDup((*store)[index].c_str());
    debug(functionName, "Reading lens mode", strings[index]);
    values[index] = index;
    severities[index] = 0;
  }
  // Don't set status so as not to mask faults with the remote access (since this should always succeed)
  this->doCallbacksEnum(strings, values, severities, (size_t)store->size(), param, 0);

  return status;
}

asynStatus SpecsAnalyser::readRunModes()
{
  const char * functionName = "SpecsAnalyser::readRunModes";
  asynStatus status = asynSuccess;

  debug(functionName, "Reading Run Modes...");

  // These values are defined by the SPECS interface software and will not change
  runModes_.push_back("Fixed Transmission");
  runModes_.push_back("Snapshot");
  runModes_.push_back("Fixed Retarding Ratio");
  runModes_.push_back("Fixed Energy");

  return status;
}

/**
 * Send the GetSpectrumDataInfo method to query the spectrum metadata.
 * Currently only OrdinateRange is supported, which gives us the units and min/max of the
 * non-energy axis.
 * This method is new in protocol version 1.11 so in earlier versions we expect to receive an error message.
 *
 * @param param - Data Info parameter to request. Currently (v1.11) only OrdinateRange is supported.
 */
asynStatus SpecsAnalyser::readSpectrumDataInfo(SPECSDataInfoParam_t param)
{
   static const char *functionName = "SpecsAnalyser::asynPortConnect";

   asynStatus status = asynSuccess;
   std::string parameterName;
   std::map<std::string, std::string> data;
   std::string cmd = SPECS_CMD_GET_DATA_INFO;

   std::string unit;
   double min, max;

   switch(param) {
     case SPECSOrdinateRange:
       parameterName = "\"OrdinateRange\"";
       break;

     default:
       debug(functionName, "Unknown parameter", param);
       status = asynError;
       break;

   }

   debug(functionName, "Reading Spectrum Data Info ", parameterName);

   if (status == asynSuccess) {
       status = sendSimpleCommand(cmd + " ParameterName:" + parameterName, &data);
   }

   if (status == asynSuccess) {
       switch(param) {
	 case SPECSOrdinateRange:
	   unit = data["Unit"];
	   cleanString(unit, "\"");
	   setStringParam(SPECSNonEnergyUnits_, unit.c_str());
	   if (readDoubleData(data, "Min", min) == asynSuccess)
	     setDoubleParam(SPECSNonEnergyMin_, min);
	   if (readDoubleData(data, "Max", max) == asynSuccess)
	     setDoubleParam(SPECSNonEnergyMax_, max);
	   break;

	 default:
	   debug(functionName, "Unknown parameter", param);
	   status = asynError;
	   break;

       }
   }
   callParamCallbacks();
   return status;
}

/**
 * Connect to the underlying low level Asyn port that is used for comms.
 * This uses the asynOctetSyncIO interface, and also sets the input and output terminators.
 *
 * @param port - Name of the port to connect to.
 * @param addr - Address to connect to.
 * @param ppasynUser - Pointer to the asyn user structure.
 * @param inputEos - String input EOS.
 * @param outputEos - String output EOS.
 */
asynStatus SpecsAnalyser::asynPortConnect(const char *port, int addr, asynUser **ppasynUser, const char *inputEos, const char *outputEos)
{
  static const char *functionName = "SpecsAnalyser::asynPortConnect";

  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;

  status = pasynOctetSyncIO->connect( port, addr, ppasynUser, NULL);
  if (status) {
    debug(functionName, "Unable to connect to port", port);
    return status;
  }

  status = pasynOctetSyncIO->setInputEos(*ppasynUser, inputEos, strlen(inputEos));
  status = pasynOctetSyncIO->setOutputEos(*ppasynUser, outputEos, strlen(outputEos));

  debug(functionName, "Connected to low level asynOctetSyncIO port", port);

  // We have connected successfully to the port so reset the message counter
  setIntegerParam(SPECSMsgCounter_, 0);
  // Record that we are connected
  setIntegerParam(SPECSConnected_, 1);

  // Issue the connect command to complete connection
  status = sendSimpleCommand(SPECS_CMD_CONNECT, &data);
  if (status != asynSuccess){
    debug(functionName, "Failed to send command", SPECS_CMD_CONNECT);
  } else {
    debug(functionName, "Connect command return data", data);
  }
  if (status == asynSuccess){
    // The connection was successful, record the server name and the protocol version
    std::string server = data["ServerName"];
    cleanString(server, "\"");
    setStringParam(SPECSServerName_, server.c_str());
    std::string protocol = data["ProtocolVersion"];
    cleanString(protocol, "\"");
    setStringParam(SPECSProtocolVersion_, protocol.c_str());
  }
  return status;
}

asynStatus SpecsAnalyser::asynPortDisconnect(asynUser *pasynUser)
{
  static const char *functionName = "SpecsAnalyser::asynPortDisconnect";

  asynStatus status = asynSuccess;
  std::map<std::string, std::string> data;

  status = pasynOctetSyncIO->disconnect(pasynUser);
  if (status) {
    debug(functionName, "Unable to disconnect from asyn port");
  }
  // Record that we are not connected
  setIntegerParam(SPECSConnected_, 0);

  return status;
}

/**
 * This sends a command to the device and parses the response.  Data is returned
 * in a std::map that is indexed by the parameter name
 * @param command - String command to send.
 * @param response - String response back (OK or ERROR)
 * @param data - Map of data items indexed by name
 */
asynStatus SpecsAnalyser::commandResponse(const std::string &command, std::string &response, std::map<std::string, std::string> &data)
{
  static const char *functionName = "SpecsAnalyser::commandResponse";

  asynStatus status = asynSuccess;
  std::string errorCode = "";
  std::string replyString = "";
  std::string nameValueString = "";
  char replyArray[SPECS_MAX_STRING];

  debug(functionName, "Command to send", command);
  status = asynWriteRead(command.c_str(), replyArray);
  debug(functionName, "Response received", replyArray);
  // Only continue if the status is good...
  if (status == asynSuccess){
    // OK we need to first find out if the command was accepted or not
    replyString.assign(replyArray);
    // Search for OK or ERROR at the beginning of the string
    if (replyString.substr(0, 2) == "OK"){
      response = "OK";
      // Only parse the response if there is something to parse!
      if (replyString.length() > 3){

      replyString = replyString.substr(2);
      // Clean any : and whitespace from the ends of the string
      cleanString(replyString);
      // Now loop over the remainder of the string and search for name value pairs
      // Create a small state machine for this
      int         insideQuotes = 0;
      int         nameCheck    = 1;
      std::string name         = "";
      int         valueCheck   = 0;
      std::string value        = "";
      int         arrayCheck   = 0;
      for (unsigned int index = 0; index < replyString.length(); index++){
        // Get the current character
        std::string cc = replyString.substr(index, 1);
        // Are we name checking or value checking
        if (nameCheck == 1){
          // Is the character a :
          if (cc == ":"){
            // We have the name, now start the value check
            nameCheck = 0;
            valueCheck = 1;
          } else {
            name += cc;
          }
        } else if (valueCheck == 1){
          value += cc;
          // If this is the last character then we must be at the end of this value
          if (index == replyString.length()-1){
            cleanString(name);
            cleanString(value);
            data[name] = value;
          } else {
            // If we are inside a quote we only care about the end quote
            if (insideQuotes == 1){
              // Test check previous character for '\'
              std::string tcc = replyString.substr(index-1, 1);
              if (tcc != "\\"){
                if (cc == "\""){
                  insideQuotes = 0;
                }
              }
            } else {
              // If we see a [ then we are in an array
              if (cc == "["){
                arrayCheck = 1;
              // If we see a ] then we are exiting the array
              } else if (cc == "]"){
                arrayCheck = 0;
              // If we see a " then we are entering quotes
              } else if (cc == "\""){
                insideQuotes = 1;
              // If we see a ' ' then we might be at the end of the value
              } else if (cc == " "){
                // Check we are not inside quotes and not inside an array
                if ((insideQuotes == 0) && (arrayCheck == 0)){
                  // Here the value is complete so store the pair
                  cleanString(name);
                  cleanString(value);
                  data[name] = value;
                  name = "";
                  value = "";
                  nameCheck = 1;
                  valueCheck = 0;
                }
              }
            }
          }
          }
        }
      }

    } else {
      response = "ERROR";
      replyString = replyString.substr(5);
      // Clean any : and whitespace from the front of the string
      cleanString(replyString);
      // Read out the next number, as this is the error code
      while (replyString.substr(0, 1).find_first_of(" ") == std::string::npos){
        errorCode += replyString.substr(0, 1);
        replyString = replyString.substr(1);
      }
      if (errorCode == "3"){
        // The error is that we are not connected, force full disconnect
        disconnect();
      }
      data["Code"] = errorCode;
      // Clean any : and whitespace from the ends of the string
      cleanString(replyString);
      // Place the remainder of the message into the map with index Message
      data["Message"] = replyString;
    }
  }
  

  return status;
}

/**
 * Wrapper for asynOctetSyncIO write/read functions.
 * @param command - String command to send.
 * @param response - String response back.
 */
asynStatus SpecsAnalyser::asynWriteRead(const char *command, char *response)
{
  static const char *functionName = "SpecsAnalyser::asynWriteRead";

  asynStatus status = asynSuccess;
  int eomReason;
  size_t nwrite = 0;
  size_t nread = 0;
  int connected = 0;
  char sendString[SPECS_MAX_STRING];
  char replyArray[SPECS_MAX_STRING];
  int msgCounter;
   
  // Read the message counter, and increment it
  getIntegerParam(SPECSMsgCounter_, &msgCounter);
  msgCounter++;
  if (msgCounter > 9999){
    msgCounter = 1;
  }
  // Store the value once more
  setIntegerParam(SPECSMsgCounter_, msgCounter);
  callParamCallbacks();

  //Setup the string to send to the detector
  // String starts with query:    ?
  // Followed by 4 digit number:  nnnn
  // Then a space and finally the command
  sprintf(sendString, "?%04d %s\n", msgCounter, command);

  // If there is no asyn port user then something higher up has failed
  // Make sure to set connected to 0 and then bail out of this call with an error
  if (!portUser_) {
    setIntegerParam(SPECSConnected_, 0);
    return asynError;
  }

  // Get the connected status
  getIntegerParam(SPECSConnected_, &connected);

  // Only proceed if we are connected
  if (connected == 1){
    status = pasynOctetSyncIO->writeRead(portUser_ ,
                                         sendString, strlen(sendString),
                                         replyArray, SPECS_MAX_STRING,
                                         SPECS_TIMEOUT,
                                         &nwrite, &nread, &eomReason );

    debug(functionName, "Raw response", replyArray);
    // Extract out the response, message counter, and error code
    if (!status && (replyArray[0] != '!')){
      // Problem with response string.  Record the error and return error response
      status = asynError;
    }
    if (!status){
      std::string replyString(replyArray);
      int counter = 0;
      // Scan the 4 digits (1..5) for the message counter
      sscanf(replyString.substr(1,4).c_str(), "%04d", &counter);
      // Verify the message counter is the same as the sent message
      debug(functionName, "Command message counter", msgCounter);
      debug(functionName, "Response message counter", counter);
      if (counter != msgCounter){
        // Problem with message ID.  Record the error and return error response
        status = asynError;
      } else {
        // Copy the response to the return string
        strcpy(response, replyString.substr(6).c_str());
      }
    }
  } else {
    status = asynError;
  }

  return status;
}

/**
 * Utility function, returns strings with unwanted characters stripped.
 * @param str - string parameter, the string to clean up, returned by reference
 * @param search - string parameter containing the set of characters to strip from str
 * @param where - integer parameter to control where characters are stripped - 1 to strip from start, 2 to strip from end, 0 to do both.
 */
asynStatus SpecsAnalyser::cleanString(std::string &str, const std::string &search, int where)
{
  if (where == 0 || where == 1){
    // Clean up from the front of the string
    while (str.substr(0, 1).find_first_of(search) != std::string::npos){
      str = str.substr(1);
    }
  }
  if (where == 0 || where == 2){
    // Clean up from the back of the string
    while (str.substr(str.length()-1, 1).find_first_of(search) != std::string::npos){
      str = str.substr(0, str.length()-1);
    }
  }
  return asynSuccess;
}

asynStatus SpecsAnalyser::initDebugger(int initDebug)
{
  // Set all debugging levels to initialised value
  debugMap_["SpecsAnalyser::SpecsAnalyser"]            = initDebug;
  debugMap_["SpecsAnalyser::specsAnalyserTask"]        = initDebug;
  debugMap_["SpecsAnalyser::readEnum"]                 = initDebug;
  debugMap_["SpecsAnalyser::writeInt32"]               = initDebug;
  debugMap_["SpecsAnalyser::writeFloat64"]             = initDebug;
  debugMap_["SpecsAnalyser::validateSpectrum"]         = initDebug;
  debugMap_["SpecsAnalyser::defineSpectrumFAT"]        = initDebug;
  debugMap_["SpecsAnalyser::defineSpectrumSFAT"]       = initDebug;
  debugMap_["SpecsAnalyser::defineSpectrumFRR"]        = initDebug;
  debugMap_["SpecsAnalyser::defineSpectrumFE"]         = initDebug;
  debugMap_["SpecsAnalyser::readAcquisitionData"]      = initDebug;
  debugMap_["SpecsAnalyser::sendSimpleCommand"]        = initDebug;
  debugMap_["SpecsAnalyser::setupEPICSParameters"]     = initDebug;
  debugMap_["SpecsAnalyser::getAnalyserParameterType"] = initDebug;
  debugMap_["SpecsAnalyser::getAnalyserParameter"]     = initDebug;
  debugMap_["SpecsAnalyser::readIntegerData"]          = initDebug;
  debugMap_["SpecsAnalyser::readDoubleData"]           = initDebug;
  debugMap_["SpecsAnalyser::readSpectrumParameter"]    = initDebug;
  debugMap_["SpecsAnalyser::readRunModes"]             = initDebug;
  debugMap_["SpecsAnalyser::readSpectrumDataInfo"]     = initDebug;
  debugMap_["SpecsAnalyser::asynPortConnect"]          = initDebug;
  debugMap_["SpecsAnalyser::commandResponse"]          = initDebug;
  debugMap_["SpecsAnalyser::asynWriteRead"]            = initDebug;
  return asynSuccess;
}

asynStatus SpecsAnalyser::debugLevel(const std::string& method, int onOff)
{
  if (method == "all"){
    debugMap_["SpecsAnalyser::SpecsAnalyser"]            = onOff;
    debugMap_["SpecsAnalyser::specsAnalyserTask"]        = onOff;
    debugMap_["SpecsAnalyser::readEnum"]                 = onOff;
    debugMap_["SpecsAnalyser::writeInt32"]               = onOff;
    debugMap_["SpecsAnalyser::writeFloat64"]             = onOff;
    debugMap_["SpecsAnalyser::validateSpectrum"]         = onOff;
    debugMap_["SpecsAnalyser::defineSpectrumFAT"]        = onOff;
    debugMap_["SpecsAnalyser::defineSpectrumSFAT"]       = onOff;
    debugMap_["SpecsAnalyser::defineSpectrumFRR"]        = onOff;
    debugMap_["SpecsAnalyser::defineSpectrumFE"]         = onOff;
    debugMap_["SpecsAnalyser::readAcquisitionData"]      = onOff;
    debugMap_["SpecsAnalyser::sendSimpleCommand"]        = onOff;
    debugMap_["SpecsAnalyser::setupEPICSParameters"]     = onOff;
    debugMap_["SpecsAnalyser::getAnalyserParameterType"] = onOff;
    debugMap_["SpecsAnalyser::getAnalyserParameter"]     = onOff;
    debugMap_["SpecsAnalyser::readIntegerData"]          = onOff;
    debugMap_["SpecsAnalyser::readDoubleData"]           = onOff;
    debugMap_["SpecsAnalyser::readSpectrumParameter"]    = onOff;
    debugMap_["SpecsAnalyser::readRunModes"]             = onOff;
    debugMap_["SpecsAnalyser::readSpectrumDataInfo"]     = onOff;
    debugMap_["SpecsAnalyser::asynPortConnect"]          = onOff;
    debugMap_["SpecsAnalyser::commandResponse"]          = onOff;
    debugMap_["SpecsAnalyser::asynWriteRead"]            = onOff;
  } else {
    debugMap_[method] = onOff;
  }
  return asynSuccess;
}

asynStatus SpecsAnalyser::debug(const std::string& method, const std::string& msg)
{
  // First check for the debug entry in the debug map
  if (debugMap_.count(method) == 1){
    // Now check if debug is turned on
    if (debugMap_[method] == 1){
      // Print out the debug message
      std::cout << method << ": " << msg << std::endl;
    }
  }
  return asynSuccess;
}

asynStatus SpecsAnalyser::debug(const std::string& method, const std::string& msg, int value)
{
  // First check for the debug entry in the debug map
  if (debugMap_.count(method) == 1){
    // Now check if debug is turned on
    if (debugMap_[method] == 1){
      // Print out the debug message
      std::cout << method << ": " << msg << " [" << value << "]" << std::endl;
    }
  }
  return asynSuccess;
}

asynStatus SpecsAnalyser::debug(const std::string& method, const std::string& msg, double value)
{
  // First check for the debug entry in the debug map
  if (debugMap_.count(method) == 1){
    // Now check if debug is turned on
    if (debugMap_[method] == 1){
      // Print out the debug message
      std::cout << method << ": " << msg << " [" << value << "]" << std::endl;
    }
  }
  return asynSuccess;
}

asynStatus SpecsAnalyser::debug(const std::string& method, const std::string& msg, const std::string& value)
{
  // First check for the debug entry in the debug map
  if (debugMap_.count(method) == 1){
    // Now check if debug is turned on
    if (debugMap_[method] == 1){
      // Copy the string
      std::string val = value;
      // Trim the output
      val.erase(val.find_last_not_of("\n")+1);
      // Print out the debug message
      std::cout << method << ": " << msg << " [" << val << "]" << std::endl;
    }
  }
  return asynSuccess;
}

asynStatus SpecsAnalyser::debug(const std::string& method, const std::string& msg, std::map<std::string, std::string> value)
{
  std::map<std::string, std::string>::iterator iter;

  // First check for the debug entry in the debug map
  if (debugMap_.count(method) == 1){
    // Now check if debug is turned on
    if (debugMap_[method] == 1){
      // Print out the debug message
      std::cout << method << ": " << msg << " [std::map" << std::endl;
      // This is a map of data, so log the entire map
      for (iter = value.begin(); iter != value.end(); ++iter) {
        std::cout << "     " << iter->first << " => " << iter->second << std::endl;
      }
      std::cout << "]" << std::endl;
    }
  }
  return asynSuccess;
}

// Code required for iocsh registration of the SPECS analyser
static const iocshArg specsAnalyserConfigArg0 = {"portName", iocshArgString};
static const iocshArg specsAnalyserConfigArg1 = {"driverPortName", iocshArgString};
static const iocshArg specsAnalyserConfigArg2 = {"Max number of NDArray buffers", iocshArgInt};
static const iocshArg specsAnalyserConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg specsAnalyserConfigArg4 = {"priority", iocshArgInt};
static const iocshArg specsAnalyserConfigArg5 = {"stackSize", iocshArgInt};

static const iocshArg * const specsAnalyserConfigArgs[] =  {&specsAnalyserConfigArg0,
                                                            &specsAnalyserConfigArg1,
                                                            &specsAnalyserConfigArg2,
                                                            &specsAnalyserConfigArg3,
                                                            &specsAnalyserConfigArg4,
                                                            &specsAnalyserConfigArg5};

static const iocshFuncDef configSpecsAnalyser = {"specsAnalyserConfig", 6, specsAnalyserConfigArgs};

static void configSpecsAnalyserCallFunc(const iocshArgBuf *args)
{
    specsAnalyserConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

// Code required for setting the debug level of the SPECS analyser
static const iocshArg specsAnalyserDebugArg0 = {"port name", iocshArgString};
static const iocshArg specsAnalyserDebugArg1 = {"method name", iocshArgString};
static const iocshArg specsAnalyserDebugArg2 = {"debug on or off", iocshArgInt};

static const iocshArg * const specsAnalyserDebugArgs[] =  {&specsAnalyserDebugArg0,
                                                           &specsAnalyserDebugArg1,
                                                           &specsAnalyserDebugArg2};

static const iocshFuncDef debugSpecsAnalyser = {"specsAnalyserDebug", 3, specsAnalyserDebugArgs};

static void debugSpecsAnalyserCallFunc(const iocshArgBuf *args)
{
    specsSetDebugLevel(args[0].sval, args[1].sval, args[2].ival);
}

static void specsAnalyserRegister(void)
{
    iocshRegister(&configSpecsAnalyser, configSpecsAnalyserCallFunc);
    iocshRegister(&debugSpecsAnalyser, debugSpecsAnalyserCallFunc);
}

epicsExportRegistrar(specsAnalyserRegister);

