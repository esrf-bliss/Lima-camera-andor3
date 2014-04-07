
### This [i]python script should be run after making sure that the LD_LIBRARY_PATH contains the 
###    INSTALL_DIR/Lima/lib (or any other directory containing the *.so from lima).
### Here this gives (since the configuration is well done and no normal library requires the use of the fallback LD_LIBRARY_PATH setting) :
### export LD_LIBRARY_PATH=/usr/local/Lima/lib 

### Note also : there is a bug with Sps : if it is not compiled in (COMPILE_SPS=0 in config.inc) then it is still required by limacore !!!

LD_LIBRARY_PATH=/usr/local/Lima/Lima/lib ipython

import sys,time,os
## sys.path.insert(1, "/usr/local/Lima")
sys.path.insert(1, os.environ['LD_LIBRARY_PATH']+"/../..")
from Lima import Core,Andor3
import time

cam = Andor3.Camera("/usr/local/andor/bitflow", 0)
cam_int = Andor3.Interface(cam)
cam_ctr = Core.CtControl(cam_int)

# cam.setNbFrames(3)
## Taking care of the saving :
cam_sav = cam_ctr.saving()
cam_sav.setDirectory("/mnt/local-spool")
cam_sav.setPrefix("testing_A_")
cam_sav.setSavingMode(Core.CtSaving.AutoFrame)

## cam_sav.setSavingMode(Core.CtSaving.Manual)

## Nexus :
cam_sav.setFormat(Core.CtSaving.NXS)
cam_sav.setSuffix(".nxs")

## TIFF :
cam_sav.setFormat(Core.CtSaving.TIFFFormat)
cam_sav.setSuffix(".tiff")

cam_sav.getParameters()


## Proper way of doing that is :
cam_ctr.acquisition().setAcqExpoTime(.001)
cam_ctr.acquisition().setLatencyTime(.001)
cam_ctr.acquisition().setAcqNbFrames(10)
cam_ctr.image().setMode(Core.CtImage.HardOnly)
##cam_ctr.image().setMode(Core.CtImage.HardAndSoft)
cam_ctr.image().setRoi(Core.Roi(409, 201, 1776, 1760))  ### left, top, width, height 

## Core.DebParams.setTypeFlags( Core.DebParams.getTypeFlags() | Core.DebTypeTrace )
Core.DebParams.setTypeFlagsNameList(['Fatal', 'Error', 'Warning', 'Trace', 'Funct'])

cam_ctr.video().startLive()
time.sleep(3)
cam_ctr.video().stopLive()

cam_ctr.prepareAcq()
cam_ctr.startAcq()
time.sleep(1)
cam_ctr.stopAcq()


im1 = cam_ctr.ReadImage()
im2 = cam_ctr.ReadImage()
im3 = cam_ctr.ReadImage()

import time
import numpy
import numpy.fft
import matplotlib

matplotlib.use('TkAgg') # do this before importing pylab

import matplotlib.pyplot as plt

def get_image():
    cam.setNbFrames(1)
    cam_ctr.prepareAcq()
    cam_ctr.startAcq()
    while cam_ctr.getStatus().AcquisitionStatus: pass
    return cam_ctr.ReadImage().buffer

def animate():
    tstart = time.time()                   # for profiling
    data=get_image()
    im=plt.imshow(data)

    while 1:
        data=get_image()
        im.set_data(data)
        fig.canvas.draw()                         # redraw the canvas
    print 'FPS:' , 200/(time.time()-tstart)

fig = plt.figure()
ax = fig.add_subplot(111)

win = fig.canvas.manager.window
fig.canvas.manager.window.after(100, animate)
plt.show()

if __name__ == "__main__":
    animate()


## Using the debugging :
## % python
## Python 2.6.4 (r264:75706, Dec  3 2009, 02:54:59)
## [GCC 4.1.2 20070626 (Red Hat 4.1.2-14)] on linux2
## Type "help", "copyright", "credits" or "license" for more information.
## >>> from Lima import Core
## >>> '%x' % Core.DebParams.getTypeFlags()
## '87'
## >>> Core.DebParams.getTypeFlagsNameList()
## ['Fatal', 'Error', 'Warning', 'Always']
## >>> '%x' % Core.DebParams.AllFlags
## 'ffffffff'
## >>> Core.DebParams.setTypeFlags(Core.DebParams.AllFlags)
## >>> Core.DebParams.getTypeFlagsNameList()
## ['Fatal', 'Error', 'Warning', 'Trace', 'Funct', 'Param', 'Return', 'Always']
## >>> Core.DebParams.setTypeFlagsNameList(['Fatal', 'Error', 'Warning',
## 'Trace', 'Funct'])
## >>> Core.DebParams.getTypeFlagsNameList()['Fatal', 'Error', 'Warning',
## 'Trace', 'Funct']
## >>> '%x' % Core.DebParams.getTypeFlags()
## '1f'


* Debug de andor3 :
** Question prepareAcq/startAcq :
   Peut-on faire plusieurs startAcq à la suite d\'un seul prepareAcq ?
** Blocage à la 3ème startAcq.
** Vitesse de lecture des frame très lente !!! Vérifier le «FrameRate» au lancement de la caméra
*** Problème sur le réglage de Latency… accepter de la mettre à zero, pour au niveau de startAcq mettre la valeur minimale.
*** Problème de la lecture des frame -> c\'est très lent, même avec un court temps de pause.
** Roi : le checkRoi cause un bug lors de l\'utilisation de HardSoft roi dans CtImage.

