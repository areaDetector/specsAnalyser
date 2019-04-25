TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += specs
#
# The following are compiled and added to the support library
LIB_SRCS += specsAnalyser.cpp
INC += specsAnalyser.h
DBD += specsAnalyserSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
