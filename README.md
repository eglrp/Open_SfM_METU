## Open_SfM_METU
Self Calibration Technique Integration to Structure from Motion Pipeline
Two calibration including radial distortion removal and focal length estimation
N view focal length estimation

Open_SfM_METU is structure from motion (SfM) pipeline in which both self calibration from two views for sequential SfM and calibration for all the cameras in global SfM are provided. 

## Code Example

Show what the library does as concisely as possible, developers should be able to figure out **how** your project solves their problem by looking at the code example. Make sure the API you are showing off is obvious, and that your code is short and concise.

## Motivation

In the scope this project, two views calibration algorithm is developed. Also, the two views calibration idea is applied to the global SfM pipeline containing N different cameras. 

## Installation

Use CMake build system :
=> Install the following libraries : openCV, Ceres, 
 $ mkdir Build
 $ cd Build
 $ cmake . ..
 $ make

## Tests

Unit test will be added. 


