# Teleplot for ROS

> Send ROS telemetry and data to [_Teleplot_](https://github.com/nesnes/teleplot) for realtime and rosbag plots that feels like a breeze.

![Example of a browser window with teleplot](https://github.com/nesnes/teleplot/raw/main/images/preview.jpg)

## Usage

1. Install _Teleplot_:
   * As a [VS Code extension](https://github.com/nesnes/teleplot-vscode) (Recommended)
   * As a [standalone application](https://github.com/nesnes/teleplot)
2. Launch / Open _Teleplot_
3. Customize and launch the appropriate examples in order to [send relevant data](#basics-of-teleplot) to Teleplot :
   * [Subscriber example](scripts/teleplot_subscriber.py), for real-time plots of robot telemetry
   * [Rosbag example](scripts/), using the [`rosbag`](http://wiki.ros.org/rosbag/Code%20API) API

All the examples were developed by following the names of the topics in [The Canadian Planetary Emulation Terrain Energy-Aware Rover Navigation Dataset](https://github.com/utiasSTARS/enav-planetary-dataset). This dataset is interesting, as its rosbags contains well-defined topics for GPS, IMU, power consumption, and odometry data.

## Basics of _Teleplot_

_Teleplot_ plots values that are sent with UDP packets on port `47269`. These packets must comply to the following formats :

- `varName:1234|g` adds or update the `varName` variable value on _Teleplot_ *plots*.
- `varName:1627551892437:1234|g` does the same, but specifies the value's timestamp in milliseconds for more accurate plotting.
- `varName:1627551892444:1;1627551892555:2;1627551892666:3|g` does the same as above, but publishes multiple values in a single packet.
- `gps:48.636044,-1.511441|xy` publishes two values to display an YX line plot

> More specifications on the format are available in the [_Teleplot_ repository](https://github.com/nesnes/teleplot)
