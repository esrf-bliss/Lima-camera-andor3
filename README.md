[![License](https://img.shields.io/github/license/esrf-bliss/lima.svg?style=flat)](https://opensource.org/licenses/GPL-3.0)
[![Gitter](https://img.shields.io/gitter/room/esrf-bliss/lima.svg?style=flat)](https://gitter.im/esrf-bliss/LImA)
[![Conda](https://img.shields.io/conda/dn/esrf-bcu/lima-camera-andor3.svg?style=flat)](https://anaconda.org/esrf-bcu)
[![Version](https://img.shields.io/conda/vn/esrf-bcu/lima-camera-andor3.svg?style=flat)](https://anaconda.org/esrf-bcu)
[![Platform](https://img.shields.io/conda/pn/esrf-bcu/lima-camera-andor3.svg?style=flat)](https://anaconda.org/esrf-bcu)

# LImA Andor SDK3 Camera Plugin

This is the LImA plugin for Andor SDK3 cameras (e.g sCMOS detector, Neo and Zyla families).

## Install

### Camera python

conda install -c esrf-bcu lima-camera-andor3

### Camera tango device server

conda install -c tango-controls -c esrf-bcu lima-camera-andor3-tango

# LImA

Lima ( **L** ibrary for **Im** age **A** cquisition) is a project for the unified control of 2D detectors. The aim is to clearly separate hardware specific code from common software configuration and features, like setting standard acquisition parameters (exposure time, external trigger), file saving and image processing.

Lima is a C++ library which can be used with many different cameras. The library also comes with a [Python](http://python.org) binding and provides a [PyTango](http://pytango.readthedocs.io/en/stable/) device server for remote control.

## Documentation

The documentation is available [here](https://lima.blissgarden.org)


