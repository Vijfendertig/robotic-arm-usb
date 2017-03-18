# Interface to the Velleman/OWI robotic arm's USB interface


## Hardware

The kit is available under a few different names:

* Velleman Robotic Arm KSR10:
    * http://www.velleman.eu/products/view/?id=375310
    * http://www.velleman.eu/products/view/?id=379738
* OWI Robotic Arm Edge or OWI-535
    * http://www.owirobot.com/robotic-arm-edge-1/
    * http://www.owirobot.com/products/USB-Interface-for-Robotic-Arm-Edge.html

Officially, the USB interface is only compatible with 32-bit Windows versions (which don't require
signed drivers), but [Vadim Zaliva](http://www.crocodile.org/lord/) reverse-engineered the protocol
and published his findings [on his blog](http://notbrainsurgery.livejournal.com/38622.html), making
it easy to use the robotic arm with all platforms supported by libusb.

The robotic arm has five degrees of freedom (a rotating base, a shoulder, elbow and wrist and a
gripper) and a LED in the gripper, but it lacks any kind of position feedback.


## Software

I got a Velleman Robotic Arm KSR10 together with its USB interface from my colleague as a birthday
present (yes, I know I have the best colleague(s) in the world ;-)). Since I was looking for a
little application to write some concurrent C++11 code (instead of my usual pthreads or Erlang/OTP
approach), I decided to write a multithreaded library for it (although that wasn't required, given
its current functionality). So, when connecting, the library spawns a new thread which does all USB
communication. This thread is terminated when it detects an I/O error or when the application
calls the disconnect function. All calls to the library should be thread-safe.

### Some ideas...

On the internet, I found some people who managed to implement a closed loop controller using an
Arduino with a motor shield and potentiometers as position sensors. That's the most straightforward
approach (and probably the most performant and accurate approach too), but I would like to try
locating the position and orientation of the gripper by placing a small AR marker (or marker board)
on the gripper and using a Raspberry Pi with a camera module to locate it.

I would also like to test whether it's possible to use software PWM (pulse width modulation) to
have some sort of speed control. A hardware PWM controller (an Arduino board with a motor shield,
for example) is of course a much better solution, but it's nice to test how far I can go with the
standard USB interface and some custom software.


## Included examples

### test_library

This small example shows how to connect to and disconnect from the robotic arm and how to send
commands to it. Nothing fancy at all.


## Building the library and the examples

To build the library and the examples, go to the build directory and run CMake and Make:

```
$ cd build
$ cmake ..
$ make
```


## License

MIT License

Copyright (c) 2017 Maarten De Munck

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
