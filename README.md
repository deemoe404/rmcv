# rmcv

## Overview

This project is a library for RoboMaster computer vision based on OpenCV. The functions that currently implemented
includes:

* DaHeng industrial camera driver
* Image preprocessing
* Pattern matching for light blob and armour
* Serial port communication (Linux only temporarily)
* Gravity compensation

***

## Code Specification

This part generalized the over-all architecture and coding style. It's meant to be help in understanding and contribute
to the project.

### 1. Basic architecture

#### Core
Core contains multiple modules that essential to the project.
* core.h defines the basic classes 
