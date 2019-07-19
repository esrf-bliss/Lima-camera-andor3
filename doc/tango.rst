Andor3 Tango device
=====================

This is the reference documentation of the Andor3 Tango device.

you can also find some useful information about prerequisite/installation/configuration/compilation in the :ref:`Andor3 camera plugin <camera-andor3>` section.

Properties
----------

================= =============== =============== =========================================================================
Property name	  Mandatory	  Default value	  Description
================= =============== =============== =========================================================================
config_path	  Yes		  N/A		  The configuration path, for linux default is /usr/local/etc/andor/bitflow
camera_number	  Yes		  N/A		  The camera number range is [0-N]
adc_gain	  No		  N/A		  The ADC gain setting, see attribute for possible values
adc_rate	  No		  N/A		  The ADC readout rate, see the attribute for possible values
temperature_sp	  No		  N/A		  The sensor temperature set-point
cooler		  No		  N/A		  To start/stop the cooler, values ON or OFF
================= =============== =============== =========================================================================



Attributes
----------
======================= ======= ======================= ======================================================================
Attribute name		RW	Type			Description
======================= ======= ======================= ======================================================================
adc_gain		rw	DevString		ADC/Gain pair settings  :
							 - **B11_HI-GAIN**
							 - **B11_LOW_GAIN**
							 - **B16_LH_GAIN**
adc_rate		rw	DevString		The ADC rate:
							 - **MHZ10**
							 - **MHZ100**
							 - **MHZ200**
							 - **MHZ280**
electronic_shutter_mode	rw	DevString		The electronic shutter:
							 - **GLOBAL**
							 - **ROLLING**
cooler			rw	DevString		Start/stop the cooling system of the camera mode:							
							 - **ON**, the cooler is started
							 - **OFF**, the cooler is stopped 	
cooling_status		ro	DevString		The status of the cooling system, tell if the setpoint 
							temperature is reached
fan_mode		rw	DevString		The FAN mode for extra-cooling:
							 - **OFF**  
							 - **LOW**
							 - **HIGH**
frame_rate		ro	DevDouble		The current frame rate, depends of exposure and latency time
max_frame_rate_transfer ro	DevSouble		The rate transfer of the camera interface card, can be lower
							than the camera frame rate
readout_time		ro	DevDouble		The readout time of the camera sensor
overlap			rw	DevString		To enable or disable the overlap mode:
							 - **ON**
							 - **OFF**
spurious_noise_filter	rw	DevString		To enable or disable the spurious noise filter mode:
							 - **ON**
							 - **OFF**			
temperature		ro	DevShort	 	The current sensor temperature in Celsius	
temperature_sp		rw	DevShort		The temperature setpoint in Celsius
======================= ======= ======================= ======================================================================


Commands
--------

=======================	=============== =======================	===========================================
Command name		Arg. in		Arg. out		Description
=======================	=============== =======================	===========================================
Init			DevVoid 	DevVoid			Do not use
State			DevVoid		DevLong			Return the device state
Status			DevVoid		DevString		Return the device state as a string
getAttrStringValueList	DevString:	DevVarStringArray:	Return the authorized string value list for
			Attribute name	String value list	a given attribute name
=======================	=============== =======================	===========================================
