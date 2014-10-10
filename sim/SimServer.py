#!/usr/bin/python

import sys
import SocketServer

class MyTCPHandler(SocketServer.StreamRequestHandler):
  """
  The RequestHandler class for our server.

  It is instantiated once per connection to the server, and must
  override the handle() method to implement communication to the
  client.
  """
  def loadData(self, filename):
    # Init the parameters dictionary
    self.parameters = {}
    # Load in the file data
    lines = tuple(open(filename, 'r'))
    # Loop over lines and create data structure
    for line in lines:
      data = line.rstrip("\n").split(",")
      if len(data) > 2:
        self.parameters[data[0]] = {'type':data[1], 'value':data[2]}

    print str(self.parameters)

  def handle(self):
    # Initialise some connection data
    self.loadData("parameters.dat")
    self.client_connected = False
    self.cmd_id = ""

    while (1):
      try:
        # Listen for incoming commands
        self.command = self.rfile.readline().strip()

        resp = self.parseCommand()
        if resp == True:
          # Good format command, continue
          self.wfile.write(self.reply + "\n")

      except KeyboardInterrupt:
        # Quit
        sys.exit(0)

  def parseCommand(self):
    # Check the length
    if len(self.command) == 0:
      return False

    # Strip off the incoming 1st byte, should be a ?
    # If it isn't return a failure
    if self.command[0] == '?':
      # Place the number into a store for the reply
      self.cmd_id = self.command[1:5]


      # Check for a connection
      if "Connect" in self.command[6:]:
        if self.client_connected == False:
          self.client_connected = True
          self.reply = "!" + self.cmd_id + " OK: ServerName:\"Test\" ProtocolVersion:1.2"
        else:
          self.reply = "!" + self.cmd_id + " Error: 2 Already connected to a TCP client."

      elif "Disconnect" in self.command[6:]:
        if self.client_connected == True:
          self.client_connected = False
          self.reply = "!" + self.cmd_id + " OK"
        else:
          self.reply = "!" + self.cmd_id + " Error: 3 You are not connected."

      elif "DefineSpectrumFAT" in self.command[6:]:
        # Split up the incoming parameters
        params = self.command[24:].split()
        for param in params:
          print "Param: " + param
          if "StartEnergy:" in param:
            print "Start Energy => " + param[12:]
            self.startEnergy = float(param[12:])
          if "EndEnergy:" in param:
            print "End Energy => " + param[10:]
            self.endEnergy = float(param[10:])
          if "StepWidth:" in param:
            print "Step Width => " + param[10:]
            self.stepWidth = float(param[10:])
          if "DwellTime:" in param:
            print "Dwell Time => " + param[10:]
            self.dwellTime = float(param[10:])
          if "PassEnergy:" in param:
            print "Pass Energy => " + param[11:]
            self.passEnergy = float(param[11:])
          if "LensMode:" in param:
            print "LensMode => " + param[9:]
            self.lensMode = param[9:]
          if "ScanRange:" in param:
            print "Scan Range => " + param[10:]
            self.scanRange = param[10:]
        self.samples = int((self.endEnergy - self.startEnergy) / self.stepWidth + 1)
        print "Samples => " + str(self.samples)
        self.reply = "!" + self.cmd_id + " OK"
      elif "DefineSpectrumFFR" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " DefineSpectrumFFR called"
      elif "DefineSpectrumFE" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " DefineSpectrumFE called"
      elif "ValidateSpectrum" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " OK: StartEnergy:" + str(self.startEnergy) + " EndEnergy:" + str(self.endEnergy) + " StepWidth:" + str(self.stepWidth) + " Samples:" + str(self.samples) + " DwellTime:" + str(self.dwellTime) + " PassEnergy:" + str(self.passEnergy) + " LensMode:" + self.lensMode + " ScanRange:" + self.scanRange
      elif "Start" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " OK"
      elif "Pause" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " Pause called"
      elif "Resume" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " Resume called"
      elif "Abort" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " Abort called"
      elif "GetAcquisitionStatus" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " GetAcquisitionStatus called"
      elif "GetAcquisitionData" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " GetAcquisitionData called"
      elif "ClearSpectrum" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " OK"
      elif "GetAllAnalyzerParameterNames" in self.command[6:]:
        self.reply = "!" + self.cmd_id + " OK: ParameterNames:["
        for key in self.parameters:
          self.reply += "\"" + key + "\","
        self.reply = self.reply.rstrip(',')
        self.reply += "]"

      elif "GetAnalyzerParameterInfo" in self.command[6:]:
        # Search for the parameter name
        if "ParameterName" in self.command[31:]:
          name = self.command[46:].rstrip('"')
          #print name + "\n"
          self.reply = "!" + self.cmd_id + " OK:  ValueType:" + self.parameters[name]['type']
        #self.reply = "!" + self.cmd_id + " GetAnalyzerParameterInfo called"
      elif "GetAnalyzerParameterValue" in self.command[6:]:
        # Search for the parameter name
        if "ParameterName" in self.command[31:]:
          name = self.command[47:].rstrip('"')
          #print name
          self.reply = "!" + self.cmd_id + " OK: Name:\"" + name + "\" Value:" + self.parameters[name]['value']
        #self.reply = "!" + self.cmd_id + " GetAnalyzerParameterValue called"
      else:
        # Anything else is an unknown command
        self.reply = "!" + self.cmd_id + " Error: 101 Unknown command: " + self.command[6:]

      return True

    else:
      if self.client_connected == False:
        self.reply = "!FFFF Error: 4 Unknown message format."
        return True

      return False


if __name__ == "__main__":
  HOST, PORT = "localhost", 7010
  #HOST, PORT = "10.2.2.55", 7010

  # Create the server, binding to localhost on port 9999
  server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

  # Activate the server; this will keep running until you
  # interrupt the program with Ctrl-C
  server.serve_forever()

