############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2014
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
#=============================================================================
#
# file :        Andor3.py
#
# description : Python source for the Andor3 and its commands. 
#                The class is derived from Device. It represents the
#                CORBA servant object which will be accessed from the
#                network. All commands which can be executed on the
#                Pilatus are implemented in this file.
#
# project :     TANGO Device Server
#
# copyleft :    European Synchrotron Radiation Facility
#               BP 220, Grenoble 38043
#               FRANCE
#
#=============================================================================
#         (c) - Bliss - ESRF
#=============================================================================
#
import PyTango
import sys, types, os, time

from Lima import Core
from Lima import Andor3 as Andor3Module
# import some useful helpers to create direct mapping between tango attributes
# and Lima interfaces.
from Lima.Server import AttrHelper

class Andor3(PyTango.Device_4Impl):

    Core.DEB_CLASS(Core.DebModApplication, 'LimaCCDs')
    
#==================================================================
#   Andor3 Class Description:
#
#
#==================================================================

class Andor3(PyTango.Device_4Impl):

#--------- Add you global variables here --------------------------
    Core.DEB_CLASS(Core.DebModApplication, 'LimaCCDs')

#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def __init__(self,cl, name):
        PyTango.Device_4Impl.__init__(self,cl,name)
        # dictionnaries to be used with AttrHelper.get_attr_4u
        self.__AdcGain = {'B11_HI_GAIN':  _Andor3Camera.b11_hi_gain,
                             'B11_LOW_GAIN': _Andor3Camera.b11_low_gain,
                             'B16_LH_GAIN':  _Andor3Camera.b16_lh_gain,
                          }
        self.__AdcRate = {'MHZ10':  _Andor3Camera.MHz10,
                          'MHZ100': _Andor3Camera.MHz100,
                          'MHZ200': _Andor3Camera.MHz200,
                          'MHZ280': _Andor3Camera.MHz280,
                          }
        self.__Cooler = {'ON':  True,
                         'OFF': False}
        self.__FanSpeed = {'OFF':  _Andor3Camera.Off,
                           'LOW':  _Andor3Camera.Low,
                           'HIGH': _Andor3Camera.On,
                           }
        self.__ElectronicShutterMode = {'ROLLING': _Andor3Camera.Rolling,
                                        'GLOBAL': _Andor3Camera.Global,
                                        }
        self.__Overlap = {'ON':  True,
                         'OFF': False}
        self.__SpuriousNoiseFilter = {'ON':  True,
                         'OFF': False}
        self.__Attribute2FunctionBase = {'adc_gain': 'SimpleGain',
                                         'adc_rate': 'AdcRate',
                                         'temperature': 'Temperature',
                                         'temperature_sp': 'TemperatureSP',
                                         'cooler': 'Cooler',
                                         'cooling_status': 'CoolingStatus',
                                         'fan_speed': 'FanSpeed',
                                         'electronic_shutter_mode': 'ElectronicShutterMode',
                                         'frame_rate': 'FrameRate',
                                         'max_frame_rate_transfer': 'MaxFrameRateTransfer',
                                         'readout_time': 'ReadoutTime',
                                         'overlap': 'Overlap',
                                         'spurious_noise_filter': 'SpuriousNoiseFilter',
                                         }
        self.init_device()
                                               
#------------------------------------------------------------------
#    Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        pass


#------------------------------------------------------------------
#    Device initialization
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def init_device(self):
        self.set_state(PyTango.DevState.ON)

        # Load the properties
        self.get_device_properties(self.get_device_class())

        # Apply properties if any
        if self.adc_gain:
            _Andor3Interface.setAdcGain(self.__AdcGain[self.adc_gain])
            
        if self.adc_rate:
            _Andor3Interface.setAdcRate(self.__AdcRate[self.adc_rate])
            
        if self.temperature_sp:            
            _Andor3Camera.setTemperatureSP(self.temperature_sp)
            
        if self.cooler:
            _Andor3Camera.setCooler(self.__Cooler[self.cooler])
            

#==================================================================
#
#    Andor3 read/write attribute methods
#
#==================================================================


    def __getattr__(self,name) :
        try:
            return AttrHelper.get_attr_4u(self, name, _Andor3Interface)
        except:
            return AttrHelper.get_attr_4u(self, name, _Andor3Camera)


#==================================================================
#
#    Andor3 command methods
#
#==================================================================

#------------------------------------------------------------------
#    getAttrStringValueList command:
#
#    Description: return a list of authorized values if any
#    argout: DevVarStringArray   
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return AttrHelper.get_attr_string_value_list(self, attr_name)
    

#==================================================================
#
#    Andor3 class definition
#
#==================================================================
class Andor3Class(PyTango.DeviceClass):

    #    Class Properties
    class_property_list = {
        }

    #    Device Properties
    device_property_list = {
        'config_path':
        [PyTango.DevString,
         'configuration path directory', []],
        'camera_number':
        [PyTango.DevShort,
         'Camera number', []],
        'adc_gain':
        [PyTango.DevString,
         'Adc Gain', []],
        'adc_rate':
        [PyTango.DevString,
         'Adc readout rate', []],
        'temperature_sp':
        [PyTango.DevShort,
         'Temperature set point in Celsius', []],
        'cooler':
        [PyTango.DevString,
         'Start or stop the cooler ("ON"/"OFF")', []],
        }


    #    Command definitions
    cmd_list = {
        'getAttrStringValueList':
        [[PyTango.DevString, "Attribute name"],
         [PyTango.DevVarStringArray, "Authorized String value list"]]
        }


    #    Attribute definitions
    attr_list = {
       'temperature_sp':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'label':'Set/get the temperature set-point',
             'unit': 'C',
             'format': '%f',
             'description': 'in Celsius',
             }],
        'temperature':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'label':'get the current temperature sensor',
             'unit': 'C',
             'format': '%f',
             'description': 'in Celsius',
             }],
        'cooler':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'label':'Start/stop the cooler',
             'unit': 'N/A',
             'format': '',
             'description': 'OFF or ON',
             }],
        'cooling_status':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'label':'Fast trigger mode, see manual for usage',
             'unit': 'N/A',
             'format': '',
             'description': '0-OFF / 1-ON',
             }],
        'adc_gain':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'label':'ADC Gain',
             'unit': 'N/A',
             'format': '',
             'description': 'ADC Gain which can be apply to the preamplifier',
             }],
        'adc_rate':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'label': 'ADC Rate',
             'unit': 'N/A',
             'format': '',
             'description': 'ADC Readout Rate',
             }],
        'electronic_shutter_mode':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'label':'Electronic Shutter Mode',
             'unit': 'N/A',
             'format': '',
             'description': 'Electronic shutter mode, Rolling or Global',
             }],
       'fan_speed':
       [[PyTango.DevString,
         PyTango.SCALAR,
         PyTango.READ_WRITE],
        {
            'label':'Fan speed',
            'unit': 'N/A',
            'format': '',
            'description': 'Fan speed, off, low or High',
            }],
        'frame_rate':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'label':'Frame rate',
             'unit': 'Hz',
             'format': '%f',
             'description': 'the rate at which frames are delivered to the use',
             }],
        'max_frame_rate_transfer':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'label':'Maximum frame rate transfer',
             'unit': 'byte per sec.',
             'format': '%f',
             'description': 'Returns the maximum sustainable transfer rate of the interface for the current shutter mode and ROI',
             }],
        'readout_time':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'label':'Readout time',
             'unit': 'sec',
             'format': '%f',
             'description': 'return the time to readout data from the sensor',
             }],
        'overlap':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'label':' Enable/Disable overlap mode',
             'unit': 'N/A',
             'format': '',
             'description': 'OFF or ON',
             }],
        'spurious_noise_filter':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'label':'Enable/Disable spurious noise filter',
             'unit': 'N/A',
             'format': '',
             'description': 'OFF or ON',
             }],
        }

#------------------------------------------------------------------
#    Andor3Class Constructor
#------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name)

            
#----------------------------------------------------------------------------
#                              Plugins
#----------------------------------------------------------------------------
from Lima  import Andor3 as Andor3Acq

_Andor3Camera = None
_Andor3Interface = None

def get_control(config_path='/users/blissadm/local/Andor3/andor/bitflow', camera_number = '0', **keys) :
    #properties are passed here as string
    global _Andor3Camera
    global _Andor3Interface
    if _Andor3Camera is None:
        print ('\n\nStarting and configuring the Andor3 camera ...')
        _Andor3Camera = Andor3Acq.Camera(config_path, int(camera_number))
        _Andor3Interface = Andor3Acq.Interface(_Andor3Camera)
        print ('\n\nAndor3 Camera #%s (%s:%s) is started'%(camera_number,_Andor3Camera.getDetectorType(),_Andor3Camera.getDetectorModel()))
    return Core.CtControl(_Andor3Interface)

    
def get_tango_specific_class_n_device():
    return Andor3Class,Andor3

