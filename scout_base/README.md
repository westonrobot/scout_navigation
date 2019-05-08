# Prerequisites

* Make sure your user account has access to
  * serial port (to communicate with robot base)
  * keyboard input (to use keyboard control)

* Grant access to serial port
```
# Note: you can use "ls /dev/ttyUSB*" to check the serial port number on your computer
$ sudo chmod 666 /dev/ttyUSB0
```

* Add user to "input" group
```
$ sudo adduser <your-user-name> input
```