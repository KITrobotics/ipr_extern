^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ipr_extern
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Ati rs485 interface (#1)
  * Integrated the libmodbus package (https://libmodbus.org/).
  The package was build on the official v3.1.4. release of libmodbus and was slightly modified.
  The modifications enable us to use custom baudrates not supported in the termios.h by using 'baudrate aliasing'.
  This will redefine an existing baudrate to the baudrate requested by the user and then it for modbus communication.
  * Renamed repository and added travis build
* Contributors: Denis Štogl
