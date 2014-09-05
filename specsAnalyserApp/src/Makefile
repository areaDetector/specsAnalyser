TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += specs

DBD += specsAnalyserSupport.dbd

# The following are compiled and added to the support library
specs_SRCS += SpecsAnalyser.cpp

specs_LIBS += asyn
specs_LIBS += ADBase
specs_LIBS += busy

# We need to link against the EPICS Base libraries
specs_LIBS += $(EPICS_BASE_IOC_LIBS)


include $(TOP)/configure/RULES
