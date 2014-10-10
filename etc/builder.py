from iocbuilder import Device, AutoSubstitution, Architecture, SetSimulation
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import Asyn, AsynPort, AsynIP

from iocbuilder.modules.areaDetector import AreaDetector, _NDPluginBase, _ADBase, _ADBaseTemplate, simDetector

class _SpecsAnalyser(AutoSubstitution):
    TemplateFile= "specsAnalyser.template"
    SubstitutionOverwrites = [_ADBaseTemplate]

class SpecsAnalyserDLD(AutoSubstitution):
    TemplateFile= "specsAnalyserDLD.template"

class SpecsAnalyser(_ADBase):
    """Create a SPECS Analyser detector"""
    _BaseTemplate = _ADBaseTemplate
    _SpecificTemplate = _SpecsAnalyser
    AutoInstantiate = True
    def __init__(self, DRIVER="DRV.1", BUFFERS=50, MEMORY=-1, **args):
        self.__super.__init__(**args)
        self.__dict__.update(locals())
        
    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _BaseTemplate.ArgInfo + makeArgInfo(__init__,
        DRIVER = Simple('Asyn driver port to connect to for communication to the device', str),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for plugin callbacks', int),
        MEMORY  = Simple('Max memory to allocate, should be maxw*maxh*nbuffer for driver and all attached plugins', int))
    LibFileList = ['specs']
    DbdFileList = ['specsAnalyserSupport']
    SysLibFileList = []
    MakefileStringList = []

    def Initialise(self):
        print '# specsAnalyserConfig(portName, driverName, maxBuffers, maxMemory )'
        print 'specsAnalyserConfig( %(PORT)10s, %(DRIVER)12s, %(BUFFERS)10d, %(MEMORY)9d )' % self.__dict__


