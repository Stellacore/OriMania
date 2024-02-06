
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

The math model is described
[further below](#Theoretical-Framework) and in this
[theory document](./theory/OriManiaTheory.lyx).

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
force exploration of all common conventions of the 3 angle and 3 distance
values in all combinations that specify a rigid body orientation
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

* The [Engabra](https://github.com/Stellacore/Engabra) geometric algebra
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

## Theoretical Framework

### Coordinate frames

* BlackBox ("Box") frame - this is a coordinate frame that is implemented
but inaccessible for inspection (e.g. such as implemented within a
proprietary data processing system).

* Sensor ("Sen") frame - each sensor is assumed to have its own well
defined 3D coordinate frame. (E.g. for cameras, this would be the exterior
coordinate frame often attached to the optical system's entrance pupil).

* Independent Reference ("Ind") frame - some arbitrary frame in which Sensor
orientations can be established by a independent means.

### Available data

* Black box parameters - 6 values that specify (in some unknown manner)
the offset and attitude of the sensors with respect to the box frame. E.g.
these parameters accurately specify the SenWrtBox orientation. However, the
convention by which they do so is not known.

	* Three angles: These (in some combination or another) represent the
	attitude of a sensor with respect to the (arbitrary and unknown) Box
	frame.

	* Three distances: These (again in some combination or another)
	represent the offset of a sensor with respect to the origin of the
	(unspecified) Box coordinate frame.

	* Ideally the numeric values for each of these six parameters are
	all notably distinct from each other.

* Independent sensor reference orientations: These data comprise six
degrees of measurement representing the position and attitude of each
sensor with respect to a common and unique (but otherwise arbitrary and
unknown) independent reference frame. I.e.,

	* Six parameter values that specify (in a known manner) the location
	and attitude of each and all sensors with respect to a single common
	"Ind" frame. I.e. one SenWrtInd transform for each sensor.

### Methodology

The analysis proceeds along the lines of the following.

* Use the independent sensor orientations orientation SenWrtInd for each
sensor to compute relative orientations between all pairwise combinations 
of sensors.

	* Given 'N' sensors, this produces (N(N-1)/2) pairwise relative
	orientations of the form Ro2w1, Ro3w1, ... Ro3w2, ..., and so on.
	In general, the relative orientation of sensor id 'I' with respect
	to sensor id 'J' that is computed from the independent orientation
	data may be denoted IndRoIwJ.

* Generate convention candidate combinations of the 6 input numbers
(more on this below). For each convention candidate

	* Compute candidate box orientation transforms for each pair
	of sensors - i.e., compute SenIwBox and SenJwBox using the current
	candidate convention.

	* Use these box orientation candidates to compute relative orientation
	in the box frame. E.g.

		BoxRoIwJ = SenIwBox * inverse(SenJwBox)

	* Compare the candidate BoxRoIwJ with independent reference IndRoIwJ
	and assign a similarity score to this pair. E.g.,

		DiffScore = functionOf{ Identity - BoxRoIwJ * inverse(IndRoIwJ) };

	* Combine similarity scores for all pairs to assign an overall
	fitness metric to the current convention candidate being considered.

* Select the convention candidates with the best overall fitness metric(s)
and report details.

