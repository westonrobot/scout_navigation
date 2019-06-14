# SDK for AgileX Scout Mobile Base

## Introduction

This software packages provides a C++ interface to communicate with the Scout mobile base, for sending commands to the robot and acquiring latest robot state.

Generally, you only need to instantiate an object of "class ScoutBase", then use the object to programmatically control the robot. Conceptually, class ScoutBase manages two background threads, one to process CAN messages of the robot state and accordingly update state variables in the ScoutState data structre, and the other to maintain a 50Hz loop and send latest command to the robot base. User can check the robot state or set control commands, as well as other tasks, in the main thread. 

See "src/demo/demo_scout_can.cpp" for an example.

## Package Structure

* scout: interface to send command to robot and receive robot state
* driver: manages raw data communication with robot
* third_party
    - asio: asynchronous IO management (serial and CAN)
    - stopwatch: timing control
    - googletest: for unit tests only (not required otherwise)

## Hardware Interface

* CAN: full support
* RS-232: under development

A easy and low-cost option to use the CAN interface would be using a Raspberry Pi or Beaglebone board with CAN Hat/Cape. The SDK can compile on both x86 and ARM platforms. Then you can use whatever interface you prefer (serial, USB, Ethernet etc.) for the communication between the single-board computer and your main PC.

## Build SDK

Install compile tools

```
$ sudo apt install build-essential cmake
```

Configure and build

```
$ cd scout_sdk 
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Known Limitations

1. The CAN interface requires the hardware to appear as a CAN device in the system. You can use the command "ifconfig" to check the interface status. For example, you may see something like

    ```
    can1: flags=193<UP,RUNNING,NOARP>  mtu 16
            unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
            RX packets 4751634  bytes 38013072 (36.2 MiB)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 126269  bytes 1010152 (986.4 KiB)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
            device interrupt 43
    ```
    
    If you use a CAN-to-USB adapter, make sure it supports slcan or can be brought up as a native CAN device (for example, CANable https://www.canable.io/). Some adapters may use a custom-defined protocol and appear as a serial device in Linux. In such case, you will have to translate the bytes between CAN and UART by yourself. It would be difficult for us to provide support for them since not all manufactures define this protocol in the same way.

2. Release v0.1 of this SDK provided a serial interface to talk with the robot. Front/rear light on the robot cannot be controlled and only a small subset of all robot states can be acquired throught that interface. Full support of the serial interface is still under development and requires additional work in both the SDK and firmware.
