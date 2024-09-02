# JoeScan Pinchot C API
The JoeScan Pinchot C API is the C/C++ language interface to JoeScan JS-50
scan heads. This API allows users to develop software to run on a desktop
computer and control scan heads that are connected to the same network.

If writing software to control classic JoeScan products such as the JS-20 or
JS-25, please download the supporting software
[here](help.joescan.com/display/ds/downloads).

## Examples
Example use of this API can be found in the `examples` directory. These
examples are ordered numerically and are meant to provide instruction as to how
the API is to be used. For new users, it is recommended to begin with
`00-configure-and-connect` and work upwards to the remaining examples.

## Build Dependencies
For the examples, solution files targeting Visual Studio 2019 are provided.
For those using Visual Studio 2017 or 2015, these files should be able to be
retargeted to support those older versions. If these solution files are not
suitable, CMake files are provided for each of the examples and can be used
to build the code on Windows 10.
