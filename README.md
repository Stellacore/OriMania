
# OriMania - Resolve convention for sensor-to-vehicle mounting orientation

Keywords: Sensor Calibration, Photogrammetry, Computer Vision, Remote Sensing

Given six parameters (three angles and three distances), determine the
conventions for combining these parameters that is most consistent with
available exterior orientation data.


## Application

Consider the situation:

* A software application processes data from multiple sensors mounted onto
a rigid assembly (e.g. a vehicle, a robot, a spacecraft, etc).

* The orientation (offset and attitude) of each sensor is expressed
in terms of 6 parameters comprising 3 angles, and 3 distances.

* However the interpretation of the 6 parameters is not well known (e.g.
they may be the results of a black box calibration process.

* It is desired to utilize these 6 parameters in another application,
or to provide other values for these parameters to the original
application.

* External orientation data are available for each sensor via
independent means. (E.g for cameras, such information might obtained from
independent space resection or relative orientation operations). Here
sensor exterior orientation is expressed with respect to some arbitrary
"world" frame.

In this situation, OriMania can be used to determine the interior
orientation transformations - i.e. the offset and attitude with which
each sensor is mounted to the payload structure inside the black box
system). E.g. to resolve the payload configuration that is consistent
with both the 6 unknown numeric values and also the known external
sensor configuration.

## Operation

For each sensor of interest, provide a data file that contains the
sensor orientation with respect to an arbitrary external world frame -
the "World" system, along with the three unknown angle and 3 unknown
distance parameters (here it is assumed the two groups can be
separated).

This information is provided via simple ASCII files of the form:

	# File format: Sensor Wrt World (passive convention)
	<xlocation ylocation zlocation angle23 angle31 angle12> # Note: exact
	<offset offset offset>  # Note: order arbitrary
	<angle angle angle> # Note: order arbitrary

Given such files for two or more sensors, OriMania performs a brute
for exploration of all common conventions for using the 6 parameters
to specify a rigid body orientation (e.g. translate then rotate, rotate
then translate, roll pitch then yaw, yaw roll then pitch, yaw pitch yaw,
and so on).

OriMania uses combinations of the 6 unknown parameters to evaluate several
thousand permutations of common orientation conventions in order to
determine the specific permutation(s) that is most consistent with the
known external relationships.

## Software

The OriMania software:

* Is written in standard C++ (standard 17 or later).

OriMania project:

* The project repository incorporates the cross platform build system
generator environment, [CMake](https://cmake.org).

* Developer API documentation is genreated using the documentation
generation utility, [Doxygen](https://www.doxygen.nl/index.html).

OriMania requirements include:

* The [Engabra](https://github.com/Stellacore/engabra) geometric algebra
package.

* The [Rigibra](https://github.com/Stellacore/Rigibra) rigid body
transformation package.

## Build Process

Use the CMake system to create a build generation environment. E.g. in a
\*nix environment,

	$ # -- Compile everything (including doxygen documentation)
	$ mkdir <someBuildDir> && cd <someBuildDir>
	$ cmake  \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_INSTALL_PREFIX=/tmpLocal/ \
		-DCMAKE_PREFIX_PATH=/tmpLocal/ \
		/repos/OriMania
	$ cmake --build . --target all -j `nproc`
	$ # -- Run library unit tests
	$ ctest -j `nproc`

## Usage

### Data File Setup

Create sensor information files as described above. E.g.,

	.../data
		SensorInfoFile1.txt
		SensorInfoFile2.txt
		...

### Data File analysis

Run OriMania main program, OriAnalyse with these files as arguments.
E.g., in \*nix environments,

	OriAnalyse .../data/Sensor*.txt

### Output

Output is of the form...

	TODO/TBD

