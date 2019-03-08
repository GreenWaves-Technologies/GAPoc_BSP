# GAPoc_BSP

A repository to use in conjunction with GAPPoc boards.

(NB - throughout the code and doc you may find references to GAPOC, GAPPoc or GAPoc. These acronyms all refer to the same thing: the "GAP8 Proof of concept" platform, outlined below.)


This repo contains Board Support Package (BSP) files with examples of code to run on the GAPPoc Platform.

The GAPPoc (GAP8 Proof of concept) platform integrates on a single board a GAP8 chip along with fairly invariant components (memory, crystal osc., etc.) plus a number of peripherals (sensors and radio mainly).
Different versions of GAPPoc address different classes of applications:
- GAPPoc A targets image analysis applications and includes a monochrome VGA sensor and BLE radio
- GAPPoc B targets thermal IR analysis applications and include a connector dedicated to a specific IR Sensor satellite board and the same BLE radio
- other versions will follow.
GAPPoc boards are architectured in a modular way, so there will be common blocks between one version of GAPPoc and another one.

This repo is structured to reflect the different flavors of GAPPoc :

- sub-directory GAPOC_BSP: contains source and header files useful to any version of GAPPoc,

- sub-directory GAPOC_A contains projects running on GAPOC_A. 
   For example, in directory GAPOC_A/test_GAPOC_BlinkLED, you will find source files to run a basic LED blinking test along with a Makefile to build and
 run the project
   ("make clean all" in this directory to clean and build, "make run" to execute.
   
- Similarly, sub-directory GAPOC_B contains projects running on GAPOC_B, etc.

- etc.

Licensing :
GAPPoc-related software code is provided under the terms of the Apache license, see LICENSE file.
GAPPoc hardware information is provided under the terms of the SolderPAd 2.0 harwdare license, see license file in hardware related directories.
