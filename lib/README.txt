This directory contains copies of the shared libraries used on the VersaLogic EPU-4562 to access its hardware modules via the API:
  - Digital I/O
  - I2C
  - SPI

The VersaLogic EPU-4562 runs Ubuntu 20.04 and so these shared libraries will link on a similar host, note however that the application will not work properly on a standard PC as it does not share the EPU-4562 hardware being accessed via these libraries.

