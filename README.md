
# OriMania - Resolve sensor mounting orientations

Keywords: Sensor Calibration, Photogrammetry, Computer Vision, Remote Sensing

Given six parameters comprising three angles and three distances, determine
the conventions for combining these parameters that is most consistent with
available independent exterior orientation data.


## Application

Consider the situation:

* A software application processes data from multiple sensors mounted onto
a rigid assembly (e.g. a vehicle, a robot, a spacecraft, etc).

* The orientation (offset and attitude) of each sensor is expressed in
terms of numeric values for 3 angles and for 3 distances.

* However the interpretation of the two groups of three parameters is
not well known (e.g.  they may be the results of a black box calibration
process.

* It is desired to utilize 6 orientation parameters in another
application, or to provide other numeric values for these parameters
for use by the original application.

* Independent external orientation data are available for each sensor via
an arbitrary but independent means. (E.g for cameras, such information
might obtained from independent space resection or relative orientation
operations). Here the independent sensor exterior orientation is expressed
with respect to some arbitrary "world" frame.

OriMania can be used in this situation to determine the conventions used
within the black box for representing the sensor-to-payload mounting
transformations - i.e. the offset and attitude with which each sensor
is attached to the rigid payload structure within the black box system.

I.e., OriMania allows determining the payload orientation convention
that is consistent with the group of 3 attitude parameters, the group
of 3 distance parameters and the known independent sensor exterior
orientations.

## Operation

For each sensor of interest, OriMania expects a data file containing the
sensor orientation with respect to an arbitrary external world frame -
the "World" system, along with the 3 unknown angle and 3 unknown
distance parameters.

This information is provided via simple ASCII files of the form:

	# File format: Sensor Wrt World (passive convention)
	xlocation ylocation zlocation angle23 angle31 angle12 # Note: exact
	offset offset offset  # Note: order arbitrary
	angle angle angle # Note: order arbitrary

Given a file like this for two or more sensors, OriMania performs a brute
for exploration of all common conventions of the 3 angle and 3 distance
values if all combinations for specifying a rigid body orientation
(e.g. translate then rotate, rotate then translate, roll pitch then yaw,
yaw roll then pitch, yaw pitch yaw, and so on, and so on, etc.).

OriMania uses combinations of the 6 unknown parameters to evaluate several
thousand permutations of common orientation conventions and determines
the specific permutation(s) that is(are) most consistent with the known
external relationships.

## Software

The OriMania software:

* Is written in C++ (conforming with standard 17 or later).

OriMania project:

* The project repository incorporates the [CMake](https://cmake.org)
cross platform build environment generator system.

* Developer API documentation is generated using the
[Doxygen](https://www.doxygen.nl/index.html) documentation generation
utility.

OriMania requirements include:

* The [Engabra](https://github.com/Stellacore/engabra) geometric algebra
package.

* The [Rigibra](https://github.com/Stellacore/Rigibra) rigid body
transformation package.

## Build Process

Use the CMake system to create a build generation environment. E.g. in a
\*nix environment, build and test commands are similar to,

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

