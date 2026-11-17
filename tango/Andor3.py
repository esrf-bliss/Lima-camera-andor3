############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2026
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
import tango
from tango import AttrWriteType, DevState
from tango.server import Device, attribute, command, device_property

from lima import core
from lima import andor3 as Andor3Acq
from lima.server import AttrHelper


def andor_list2dict(alist):
     adict = dict()
     for name in alist:
         newname = name.upper().strip()
         newname = newname.replace("(", "").replace(")", "").replace("-", "_").replace(" ", "_")
         adict[newname] = name
     return adict


def _make_dynamic_enum_fget_fset(attr_name, dict_attr_name):
    """Like AttrHelper.make_fget_fset, but the enum dict isn't a fixed
    class-level constant: adc_gain/adc_rate/electronic_shutter_mode/
    output_signal/fan_speed's valid values are queried from the actual
    connected camera at init_device time (andor_list2dict()) and stored
    per-instance, since they can genuinely differ between Andor3 units.
    """
    camel = AttrHelper._title_case(attr_name)
    get_name, set_name = "get" + camel, "set" + camel

    def _resolve_target(self):
        for target in (self._Andor3Interface, self._Andor3Camera):
            if hasattr(target, get_name) or hasattr(target, set_name):
                return target
        raise AttributeError("No %s/%s found for attribute %r" % (get_name, set_name, attr_name))

    def fget(self):
        value = getattr(_resolve_target(self), get_name)()
        return AttrHelper.getDictKey(getattr(self, dict_attr_name), value)

    def fset(self, value):
        d = getattr(self, dict_attr_name)
        resolved = AttrHelper.getDictValue(d, value.upper())
        if resolved is None:
            tango.Except.throw_exception(
                "WrongData", "Wrong value %s: %s" % (attr_name, value.upper()), "LimaCCDs Class"
            )
        getattr(_resolve_target(self), set_name)(resolved)

    return fget, fset


class Andor3(Device):

    core.DEB_CLASS(core.DebModule.DebModApplication, 'LimaCCDs')

    # ------------------------------------------------------------------
    #    Static properties
    # ------------------------------------------------------------------
    _Andor3Camera = None
    """Reference to Lima andor3 camera binding"""
    _Andor3Interface = None
    """Reference to Lima andor3 interface binding"""

    _Cooler = {'ON': True, 'OFF': False}
    _Overlap = {'ON': True, 'OFF': False}
    _SpuriousNoiseFilter = {'ON': True, 'OFF': False}
    _GateInverted = {'YES': True, 'NO': False}
    _TriggerInverted = {'YES': True, 'NO': False}

    # ------------------------------------------------------------------
    #    Device properties (initial hardware configuration only - the
    #    live attributes of the same name below read/write the camera
    #    directly, they don't share storage with these)
    # ------------------------------------------------------------------
    config_path = device_property(dtype=str, default_value="")
    serial_number = device_property(dtype=str, default_value="")
    adc_gain = device_property(dtype=str, default_value="")
    adc_rate = device_property(dtype=str, default_value="")
    temperature_sp = device_property(dtype=int, default_value=0)
    cooler = device_property(dtype=str, default_value="")
    overlap = device_property(dtype=str, default_value="")

    # ------------------------------------------------------------------
    #    Device initialization
    # ------------------------------------------------------------------
    @core.DEB_MEMBER_FUNCT
    def init_device(self):
        Device.init_device(self)

        # dictionnaries to be used with the dynamic-enum fget/fset above
        self._adc_gain_dict = andor_list2dict(self._Andor3Camera.getSimpleGainList())
        self._adc_rate_dict = andor_list2dict(self._Andor3Camera.getAdcRateList())
        self._electronic_shutter_mode_dict = andor_list2dict(
            self._Andor3Camera.getElectronicShutterModeList()
        )
        self._output_signal_dict = andor_list2dict(self._Andor3Camera.getOutputSignalList())
        self._fan_speed_dict = andor_list2dict(self._Andor3Camera.getFanSpeedList())

        # Leagcy code used by AttrHelper.get_attr_string_value_list
        self.__AdcGain = self._adc_gain_dict
        self.__AdcRate = self._adc_rate_dict
        self.__ElectronicShutterMode = self._electronic_shutter_mode_dict
        self.__OutputSignal = self._output_signal_dict
        self.__FanSpeed = self._fan_speed_dict
        self.__Cooler = self._Cooler
        self.__Overlap = self._Overlap
        self.__SpuriousNoiseFilter = self._SpuriousNoiseFilter
        self.__GateInverted = self._GateInverted
        self.__TriggerInverted = self._TriggerInverted

        # Apply properties if any
        if self.adc_gain:
            self._Andor3Camera.setSimpleGain(self._adc_gain_dict[self.adc_gain])

        if self.adc_rate:
            self._Andor3Camera.setAdcRate(self._adc_rate_dict[self.adc_rate])

        if self.temperature_sp:
            self._Andor3Camera.setTemperatureSP(self.temperature_sp)

        if self.cooler:
            self._Andor3Camera.setCooler(self._Cooler[self.cooler])

        if self.overlap:
            self._Andor3Camera.setOverlap(self._Overlap[self.overlap])

        self.set_state(DevState.ON)

    # ------------------------------------------------------------------
    #    Commands
    # ------------------------------------------------------------------
    @command(dtype_in=str, dtype_out=(str,))
    @core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return AttrHelper.get_attr_string_value_list(self, attr_name)

    # ------------------------------------------------------------------
    #    Attributes with a per-instance (hardware-queried) enum
    # ------------------------------------------------------------------
    # naming-convention would compute getAdcGain; the underlying hw method
    # is really named getSimpleGain (see the original Attribute2FunctionBase
    # override) - "simple_gain" (not "adc_gain") drives the get/set names.
    _adc_gain_fget, _adc_gain_fset = _make_dynamic_enum_fget_fset(
        "simple_gain", "_adc_gain_dict"
    )
    adc_gain_attr = attribute(
        name="adc_gain",
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="ADC Gain",
        doc="ADC Gain which can be apply to the preamplifier",
        fget=_adc_gain_fget,
        fset=_adc_gain_fset,
    )

    _adc_rate_fget, _adc_rate_fset = _make_dynamic_enum_fget_fset("adc_rate", "_adc_rate_dict")
    adc_rate_attr = attribute(
        name="adc_rate",
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="ADC Rate",
        doc="ADC Readout Rate",
        fget=_adc_rate_fget,
        fset=_adc_rate_fset,
    )

    _electronic_shutter_mode_fget, _electronic_shutter_mode_fset = _make_dynamic_enum_fget_fset(
        "electronic_shutter_mode", "_electronic_shutter_mode_dict"
    )
    electronic_shutter_mode = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="Electronic Shutter Mode",
        doc="Electronic shutter mode, Rolling or Global",
        fget=_electronic_shutter_mode_fget,
        fset=_electronic_shutter_mode_fset,
    )

    _output_signal_fget, _output_signal_fset = _make_dynamic_enum_fget_fset(
        "output_signal", "_output_signal_dict"
    )
    output_signal = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="Output signal selection",
        doc="FireRow1, FireRowN, FireAny, FireAll",
        fget=_output_signal_fget,
        fset=_output_signal_fset,
    )

    _fan_speed_fget, _fan_speed_fset = _make_dynamic_enum_fget_fset("fan_speed", "_fan_speed_dict")
    fan_speed = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="Fan speed",
        doc="Fan speed setting",
        fget=_fan_speed_fget,
        fset=_fan_speed_fset,
    )

    # ------------------------------------------------------------------
    #    Attributes fully dispatched by naming convention (fixed enums,
    #    or no enum at all)
    # ------------------------------------------------------------------
    # naming-convention would compute getTemperatureSp; the underlying hw
    # method is really named getTemperatureSP (see the original
    # Attribute2FunctionBase override - "SP" doesn't title-case correctly).
    def _temperature_sp_fget(self):
        for target in (self._Andor3Interface, self._Andor3Camera):
            if hasattr(target, "getTemperatureSP"):
                return target.getTemperatureSP()
        raise AttributeError("No getTemperatureSP/setTemperatureSP found")

    def _temperature_sp_fset(self, value):
        for target in (self._Andor3Interface, self._Andor3Camera):
            if hasattr(target, "setTemperatureSP"):
                target.setTemperatureSP(value)
                return
        raise AttributeError("No getTemperatureSP/setTemperatureSP found")
    temperature_sp_attr = attribute(
        name="temperature_sp",
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        label="Set/get the temperature set-point",
        unit="C",
        format="%f",
        doc="in Celsius",
        fget=_temperature_sp_fget,
        fset=_temperature_sp_fset,
    )

    _temperature_fget, _ = AttrHelper.make_fget_fset(
        "temperature", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera
    )
    temperature = attribute(
        dtype=float,
        access=AttrWriteType.READ,
        label="get the current temperature sensor",
        unit="C",
        format="%f",
        doc="in Celsius",
        fget=_temperature_fget,
    )

    _cooler_fget, _cooler_fset = AttrHelper.make_fget_fset(
        "cooler", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera,
        enum=_Cooler,
    )
    cooler_attr = attribute(
        name="cooler",
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="Start/stop the cooler",
        doc="OFF or ON",
        fget=_cooler_fget,
        fset=_cooler_fset,
    )

    _cooling_status_fget, _ = AttrHelper.make_fget_fset(
        "cooling_status", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera
    )
    cooling_status = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        label="Cooling status",
        doc="Tells if the setpoint temperature is reached",
        fget=_cooling_status_fget,
    )

    _frame_rate_fget, _ = AttrHelper.make_fget_fset(
        "frame_rate", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera
    )
    frame_rate = attribute(
        dtype=float,
        access=AttrWriteType.READ,
        label="Frame rate",
        unit="Hz",
        format="%f",
        doc="the rate at which frames are delivered to the user",
        fget=_frame_rate_fget,
    )

    _max_frame_rate_transfer_fget, _ = AttrHelper.make_fget_fset(
        "max_frame_rate_transfer", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera
    )
    max_frame_rate_transfer = attribute(
        dtype=float,
        access=AttrWriteType.READ,
        label="Maximum frame rate transfer",
        unit="byte per sec.",
        format="%f",
        doc="Returns the maximum sustainable transfer rate of the interface for the current shutter mode and ROI",
        fget=_max_frame_rate_transfer_fget,
    )

    _readout_time_fget, _ = AttrHelper.make_fget_fset(
        "readout_time", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera
    )
    readout_time = attribute(
        dtype=float,
        access=AttrWriteType.READ,
        label="Readout time",
        unit="sec",
        format="%f",
        doc="return the time to readout data from the sensor",
        fget=_readout_time_fget,
    )

    _overlap_fget, _overlap_fset = AttrHelper.make_fget_fset(
        "overlap", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera,
        enum=_Overlap,
    )
    overlap_attr = attribute(
        name="overlap",
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label=" Enable/Disable overlap mode",
        doc="OFF or ON",
        fget=_overlap_fget,
        fset=_overlap_fset,
    )

    _spurious_noise_filter_fget, _spurious_noise_filter_fset = AttrHelper.make_fget_fset(
        "spurious_noise_filter", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera,
        enum=_SpuriousNoiseFilter,
    )
    spurious_noise_filter = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="Enable/Disable spurious noise filter",
        doc="OFF or ON",
        fget=_spurious_noise_filter_fget,
        fset=_spurious_noise_filter_fset,
    )

    _serial_number_fget, _ = AttrHelper.make_fget_fset(
        "serial_number", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera
    )
    serial_number_attr = attribute(
        name="serial_number",
        dtype=str,
        access=AttrWriteType.READ,
        label="Read camera serial number",
        doc="camera serial number",
        fget=_serial_number_fget,
    )

    _trigger_inverted_fget, _trigger_inverted_fset = AttrHelper.make_fget_fset(
        "trigger_inverted", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera,
        enum=_TriggerInverted,
    )
    trigger_inverted = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="trigger signal inverted",
        doc="YES or NO",
        fget=_trigger_inverted_fget,
        fset=_trigger_inverted_fset,
    )

    _gate_inverted_fget, _gate_inverted_fset = AttrHelper.make_fget_fset(
        "gate_inverted", lambda self: self._Andor3Interface, lambda self: self._Andor3Camera,
        enum=_GateInverted,
    )
    gate_inverted = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        label="gate signal inverted",
        doc="YES or NO",
        fget=_gate_inverted_fget,
        fset=_gate_inverted_fset,
    )


#----------------------------------------------------------------------------
# Plugins
#----------------------------------------------------------------------------

def get_control(config_path='/users/blissadm/local/Andor3/andor/bitflow', serial_number='',
                 _Andor3=Andor3, _Camera=Andor3Acq.Camera, _Interface=Andor3Acq.Interface,
                 **keys):
    camera = _Andor3._Andor3Camera
    interface = _Andor3._Andor3Interface

    if camera is None:
        print('\n\nStarting and configuring the Andor3 camera ...')
        camera = _Camera(config_path, serial_number)
        interface = _Interface(camera)
        print('\n\nAndor3 Camera (%s:%s) is started' % (camera.getDetectorType(), camera.getDetectorModel()))

    _Andor3._Andor3Camera = camera
    _Andor3._Andor3Interface = interface

    return core.CtControl(interface)

def get_tango_specific_class_n_device():
    return Andor3.TangoClassClass, Andor3
