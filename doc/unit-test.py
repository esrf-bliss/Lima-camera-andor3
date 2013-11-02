#!/usr/bin/python 

### This script should be run after :
### export LD_LIBRARY_PATH="/usr/local/Lima/Lima/lib" 



### The aim is to test a version installed by
###   INSTALL_DIR=/usr/local/Lima make install

import os,sys,time
## os.environ['LD_LIBRARY_PATH']="/usr/local/Lima/Lima/lib"

print("* The general setup :")
print("\n** The environement of the pocess is now : ")
print(os.environ)

print("\n** Now, importing Lima (Core and Andor3 :")
sys.path.insert(1, "/usr/local/Lima")
print("\twith sys.path = " + ":".join(sys.path))
from Lima import Core,Andor3

print("\n** Constructing the camera and the other related object :")
cam = Andor3.Camera("/usr/local/andor/bitflow", 0)
cam_int = Andor3.Interface(cam)
cam_ctr = Core.CtControl(cam_int)

print("\n** Removing all files that could interfer with acquisition :")
os.system("rm -fv /mnt/local-spool/testing_A_*.tiff /mnt/local-spool/testing_A_*.nxs")

print("\n** Taking care of the saving format : 1st serie is wihtOUT saving")
cam_sav = cam_ctr.saving()
cam_sav.setDirectory("/mnt/local-spool")
cam_sav.setPrefix("testing_A_")
cam_sav.setSavingMode(Core.CtSaving.Manual)

print(cam_sav.getParameters())

print("** Testing the ROI : 100, 100, 2360, 1960 (in HardOnly mode)")
cam_ctr.image().setMode(Core.CtImage.HardOnly)
cam_ctr.image().setRoi(Core.Roi(100, 100, 2360, 1960))  ### left, top, width, height
the_roi = cam_ctr.image().getRoi()
print("\tThe retrieved roi is (%d, %d, %d, %d)" % (the_roi.getTopLeft().x, the_roi.getTopLeft().y, the_roi.getSize().getWidth(), the_roi.getSize().getHeight()))


print("\n**Testing the acquisition exposure and latency :")
## ##### Faster frame-rates, lower latency :
## cam.setElectronicShutterMode(Andor3.Camera.Rolling)
## cam.setAdcRate(cam.MHz280)
## cam.getLatTimeRange() ## 0.0103
## ##### Slower frame-rate, higher latency :
## cam.setElectronicShutterMode(Andor3.Camera.Global)
## cam.setAdcRate(cam.MHz100)
## cam.getLatTimeRange() ## 0.0286

cam_acq= cam_ctr.acquisition()
print("Setting the trigger mode to internal :")
cam_acq.setTriggerMode(Core.IntTrig)
print("setting the exposition time to 0.001 ...")
cam_acq.setAcqExpoTime(.001)
print("*** Getting in slow acquisition mode :")
print("setting the true camera speed to Global shutter and 100MHz")
cam.setElectronicShutterMode(Andor3.Camera.Global)
cam.setAdcRate(cam.MHz100)
print("the range of latency time for the hardware is now %.3f to %.3f s" % cam.getLatTimeRange())
print("setting the latency time to 0.015 (below the limit of the hardware")
cam_acq.setLatencyTime(.015)
print("retrieving the values : ")
print("\texpo time = %f" % cam_acq.getAcqExpoTime())
print("\tlat time = %f" % cam_acq.getLatencyTime())
if ( 0.015 != cam_acq.getLatencyTime() ) :
    print("\t The retrieved latency is different from the set one (expected)")
if ( cam.getLatTimeRange()[0] != cam_acq.getLatencyTime() ) :
    print("\t The retrieved latency time is DIFFERENT from the min latency of the hardware, %.3f vs. %.3f respectively" % (cam_acq.getLatencyTime(), cam.getLatTimeRange()[0]))

print("\nTesting the ROI : 100, 100, 2360, 1960")
cam_ctr.image().setRoi(Core.Roi(100, 100, 2360, 1960))  ### left, top, width, height
the_roi = cam_ctr.image().getRoi()
print("\tThe retrieved roi is (%d, %d, %d, %d)" % (the_roi.getTopLeft().x, the_roi.getTopLeft().y, the_roi.getSize().getWidth(), the_roi.getSize().getHeight()))

print("*** Getting in fast acquisition mode :")
print("setting the true camera speed to Global shutter and 280MHz")
cam.setElectronicShutterMode(Andor3.Camera.Global)
cam.setAdcRate(cam.MHz280)
print("the range of latency time for the hardware is now %.3f to %.3f s" % cam.getLatTimeRange())
print("setting the latency time to 0.015 (below the limit of the hardware")
cam_acq.setLatencyTime(.015)
print("retrieving the values : ")
print("\texpo time = %f" % cam_acq.getAcqExpoTime())
print("\tlat time = %f" % cam_acq.getLatencyTime())
if ( 0.015 != cam_acq.getLatencyTime() ) :
    print("\t The retrieved latency is DIFFERENT from the set one (unexpected in fast mode)")
if ( cam.getLatTimeRange()[0] != cam_acq.getLatencyTime() ) :
    print("\t The retrieved latency time is DIFFERENT from the min latency of the hardware, %.3f vs. %.3f respectively" % (cam_acq.getLatencyTime(), cam.getLatTimeRange()[0]))

print("* Testing some frame-number based acquisitions")
# print("  Setting the debug to tracing on camera:")
# Core.DebParams.setTypeFlagsNameList(['Fatal', 'Error', 'Warning', 'Trace'])
# Core.DebParams.setModuleFlagsNameList(['Camera'])

print("  Acquiring 50 image using the current settings.")
cam_ctr.acquisition().setAcqNbFrames(50)

print("  Setting the output sink :")
cam_sav = cam_ctr.saving()
cam_sav.setDirectory("/mnt/local-spool")
cam_sav.setPrefix("testing_A_")
print("  The saving parameters are :")
print(cam_sav.getParameters())

print("   Launching acquisition of %d frames whith exposure time of %.3fs and latency time %.3fs." % (cam_ctr.acquisition().getAcqNbFrames(),  cam_acq.getAcqExpoTime(), cam_acq.getLatencyTime()))

print("** Setting the output to none")
cam_sav.setSavingMode(Core.CtSaving.Manual)
print("  The saving parameters are :")
print(cam_sav.getParameters())
cam_ctr.prepareAcq()
cam_ctr.startAcq()
the_wait=0
while ( Core.AcqReady != cam_ctr.getStatus().AcquisitionStatus ) :
    time.sleep(1)
    the_wait += 1
print("Acquisition done with sleep of %ds" % (the_wait))

print("And again...")
cam_ctr.prepareAcq()
cam_ctr.startAcq()
the_wait=0
while ( Core.AcqReady != cam_ctr.getStatus().AcquisitionStatus ) :
    time.sleep(1)
    the_wait += 1
print("Acquisition done with sleep of %ds" % (the_wait))

print("** Setting the output to tiff")
cam_sav.setSavingMode(Core.CtSaving.AutoFrame)
cam_sav.setFormat(Core.CtSaving.TIFFFormat)
cam_sav.setSuffix(".tiff")
print("  The saving parameters are :")
print(cam_sav.getParameters())
cam_ctr.prepareAcq()
cam_ctr.startAcq()
the_wait=0
while ( Core.AcqReady != cam_ctr.getStatus().AcquisitionStatus ) :
    time.sleep(1)
    the_wait += 1
print("Acquisition done with sleep of %ds"% (the_wait))

print("And again...")
cam_ctr.prepareAcq()
cam_ctr.startAcq()
the_wait=0
while ( Core.AcqReady != cam_ctr.getStatus().AcquisitionStatus ) :
    time.sleep(1)
    the_wait += 1
print("Acquisition done with sleep of %ds"% (the_wait))


print("** Setting the output to nexus")
cam_sav.setSavingMode(Core.CtSaving.AutoFrame)
cam_sav.setFormat(Core.CtSaving.NXS)
cam_sav.setSuffix(".nxs")
print("  The saving parameters are :")
print(cam_sav.getParameters())
cam_ctr.prepareAcq()
cam_ctr.startAcq()
the_wait=0
while ( Core.AcqReady != cam_ctr.getStatus().AcquisitionStatus ) :
    time.sleep(1)
    the_wait += 1
print("Acquisition done with sleep of %ds"% (the_wait))

print("And again...")
cam_ctr.prepareAcq()
cam_ctr.startAcq()
the_wait=0
while ( Core.AcqReady != cam_ctr.getStatus().AcquisitionStatus ) :
    time.sleep(1)
    the_wait += 1
print("Acquisition done with sleep of %ds"% (the_wait))


print("* Testing some free-running acquisitions, during approx 2s each test :")
print("  Setting the output sink :")
cam_sav = cam_ctr.saving()
cam_sav.setDirectory("/mnt/local-spool")
cam_sav.setPrefix("testing_A_")
print("  The saving parameters are :")
print(cam_sav.getParameters())

print("** Using the null output sink :")
cam_sav.setSavingMode(Core.CtSaving.Manual)
print("  The saving parameters are :")
print(cam_sav.getParameters())
cam_ctr.video().startLive()
time.sleep(2)
cam_ctr.video().stopLive()
print("Acquisition done in free running while the main thread slept 2s")

print("And again...")
cam_ctr.video().startLive()
time.sleep(2)
cam_ctr.video().stopLive()
print("Acquisition done in free running while the main thread slept 2s")

print("** Setting the output to tiff")
cam_sav.setSavingMode(Core.CtSaving.AutoFrame)
cam_sav.setFormat(Core.CtSaving.TIFFFormat)
cam_sav.setSuffix(".tiff")
print("  The saving parameters are :")
print(cam_sav.getParameters())
cam_ctr.video().startLive()
time.sleep(2)
cam_ctr.video().stopLive()
print("Acquisition done in free running while the main thread slept 2s")

print("And again...")
cam_ctr.video().startLive()
time.sleep(2)
cam_ctr.video().stopLive()
print("Acquisition done in free running while the main thread slept 2s")

print("** Setting the output to nexus")
cam_sav.setSavingMode(Core.CtSaving.AutoFrame)
cam_sav.setFormat(Core.CtSaving.NXS)
cam_sav.setSuffix(".nxs")
print("  The saving parameters are :")
print(cam_sav.getParameters())
cam_ctr.video().startLive()
time.sleep(2)
cam_ctr.video().stopLive()
print("Acquisition done in free running while the main thread slept 2s")

print("And again...")
cam_ctr.video().startLive()
time.sleep(2)
cam_ctr.video().stopLive()
print("Acquisition done in free running while the main thread slept 2s")

