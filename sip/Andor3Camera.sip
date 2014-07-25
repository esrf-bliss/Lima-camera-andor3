/* andor3 plugin camera class interface generation
 * Copyright (C) 2013 IPANEMA USR3461, CNRS/MCC.
 * Written by Serge Cohen <serge.cohen@synchrotron-soleil.fr>
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3 of
 * the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this file. If not, see <http://www.gnu.org/licenses/>.
 */

namespace Andor3
{
  class Camera /NoDefaultCtors/
  {
%TypeHeaderCode
#include <Andor3Camera.h>
%End

  public:

// SIP is not working with the namespace, we have to give the enums and use them directly in here, even the one coming from other parts of LIMA :

    enum Status { Ready, Exposure, Readout, Latency, Fault };
    enum A3_TypeInfo { Unknown, Int, Float, Bool, Enum, String };

    enum A3_Gain { Gain1 = 0, Gain2 = 1, Gain3 = 2, Gain4 = 3, Gain1_Gain3 = 4, Gain1_Gain4 = 5, Gain2_Gain3 = 6, Gain2_Gain4 = 7 };
    enum A3_SimpleGain { b11_hi_gain=0, b11_low_gain=1, b16_lh_gain=2, none=31};
    enum A3_ShutterMode { Rolling = 0, Global = 1 };
    enum A3_ReadOutRate { MHz10 = 0, MHz100 = 1, MHz200 = 2, MHz280 = 3 };
    enum A3_BitDepth { b11 = 0, b16= 1 };
    enum A3_TriggerMode { Internal = 0, ExternalLevelTransition = 1, ExternalStart = 2, ExternalExposure = 3, Software = 4, Advanced = 5, External = 6 };
    enum A3_Binning { B1x1=0, B2x2=1, B4x4=2, B8x8=3};
    enum A3_FanSpeed { Off=0, Low=1, On=2};

    Camera(const std::string& bitflow_path, int camera_number=0);
    ~Camera();

    void prepareAcq();
    void startAcq();
    void stopAcq();

    void getImageType(ImageType& type /Out/ );
    void setImageType(ImageType type /In/ );

    void getDetectorType(std::string& type /Out/ );
    void getDetectorModel(std::string& model /Out/ );
    void getDetectorImageSize(Size& size /Out/ );

    HwBufferCtrlObj* getBufferCtrlObj();

    bool checkTrigMode(TrigMode mode /In/ );
    void setTrigMode(TrigMode  mode /In/ );
    void getTrigMode(TrigMode& mode /Out/ );

    void setExpTime(double  exp_time /In/ );
    void getExpTime(double& exp_time /Out/ );

    void setLatTime(double  lat_time /In/ );
    void getLatTime(double& lat_time /Out/ );

    void getExposureTimeRange(double& min_expo /Out/, double& max_expo /Out/) const;
    void getLatTimeRange(double& min_lat /Out/, double& max_lat /Out/) const;

    void setNbFrames(int  nb_frames /In/ );
    void getNbFrames(int& nb_frames /Out/ );
    void getNbHwAcquiredFrames(int &nb_acq_frames /Out/ );

    void checkRoi(const Roi& set_roi /In/ , Roi& hw_roi /Out/);
    void setRoi(const Roi& set_roi /In/ );
    void getRoi(Roi& hw_roi /Out/);

    bool isBinningAvailable();
    void checkBin(Bin& ioBin /In,Out/ );
    void setBin(const Bin& iBin /In/ );
    void getBin(Bin& oBin /Out/ );

    void setShutterMode(ShutterMode mode /In/ );
    void getShutterMode(ShutterMode& mode /Out/ );

    void getPixelSize(double& sizex /Out/, double& sizey /Out/);

    void getStatus(Status& status /Out/);

    void reset();
// For python (and hence SIP) this one has same signature as void getNbHwAcquiredFrames(int &nb_acq_frames /Out/ );
//  -> Keeping the more "traditional" void getNbHwAcquiredFrames(int &nb_acq_frames /Out/ );
//   int getNbHwAcquiredFrames();

// -- andor3 specific, LIMA don't worry about it !
   void initialiseController();

   void setAdcGain(A3_Gain iGain /In/ );
   void getAdcGain(A3_Gain &oGain /Out/ ) const;
   void getAdcGainString(std::string &oGainString /Out/ ) const;
   void setAdcRate(A3_ReadOutRate iRate /In/ );
   void getAdcRate(A3_ReadOutRate &oRate /Out/ ) const;
   void getAdcRateString(std::string &oRateString /Out/ ) const;
   void setElectronicShutterMode(A3_ShutterMode iMode /In/ );
   void getElectronicShutterMode(A3_ShutterMode &oMode /Out/ ) const;
   void getElectronicShutterModeString(std::string &oModeString /Out/) const;
   void setBitDepth(A3_BitDepth iMode /In/ );
   void getBitDepth(A3_BitDepth &oMode /Out/ ) const;
   void getBitDepthString(std::string &oDepthString /Out/ ) const;
   void getPxEncoding(std::string &oPxEncoding /Out/ ) const;
   void setTriggerMode(A3_TriggerMode iMode /In/ );
   void getTriggerMode(A3_TriggerMode &oMode /Out/ ) const;
   void getTriggerModeString(std::string &oModeString /Out/ ) const;
   void setTemperatureSP(double temp /In/ );
   void getTemperatureSP(double& temp /Out/ ) const;
   void getTemperature(double& temp /Out/ ) const;
   void setCooler(bool flag /In/ );
   void getCooler(bool& flag /Out/ ) const;
   void getCoolingStatus(std::string& status /Out/ ) const;

   void setBufferOverflow(bool i_overflow /In/ );
   void getBufferOverflow(bool &o_overflow /Out/ ) const;
   void setFanSpeed(A3_FanSpeed iFS /In/ );
   void getFanSpeed(A3_FanSpeed &oFS /Out/ ) const;
   void getFanSpeedString(std::string &oFSString /Out/ ) const;
   void setOverlap(bool i_overlap /In/ );
   void getOverlap(bool &o_overlap /Out/ ) const;
   void setSimpleGain(A3_SimpleGain i_gain /In/ );
   void getSimpleGain(A3_SimpleGain &o_gain /Out/ ) const;
   void getSimpleGainString(std::string &o_gainString /Out/ ) const;
   void setSpuriousNoiseFilter(bool i_filter /In/ );
   void getSpuriousNoiseFilter(bool &o_filter /Out/ ) const;
   void setSyncTriggering(bool i_sync /In/ );
   void getSyncTriggering(bool &o_sync /Out/ );
      
      // -- some readonly attributes :
   void getBytesPerPixel(double &o_value /Out/ ) const;
   void getFirmwareVersion(std::string &o_fwv /Out/ ) const;
   void getFrameRate(double &o_frame_rate /Out/ ) const;
   void getFrameRateRange(double& o_min_lat /Out/, double& o_max_lat /Out/) const;
   void getFullRoiControl(bool &o_fullROIcontrol /Out/ ) const;
   void getImageSize(int &o_frame_size /Out/ ) const;
   void getMaxFrameRateTransfer(double &o_max_transfer_rate /Out/ ) const;
   void getReadoutTime(double &o_time /Out/ ) const;
   void getSerialNumber(std::string &o_sn /Out/ ) const;
   int  getPixelStride() const;
  };

};

