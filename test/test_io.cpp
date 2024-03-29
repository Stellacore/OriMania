//
// MIT License
//
// Copyright (c) 2024 Stellacore Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


/*! \file
\brief Unit tests (and example) code for OriMania om::io functions
*/


#include "io.hpp"

#include <fstream>
#include <iostream>
#include <sstream>


namespace
{
	//! Load independent exterior orientation data.
	void
	testIndEO
		( std::ostream & oss
		)
	{
		using namespace om;

		// [DoxyExampleLoadIndEOs]

		// Example independent EO 3-angle-sequence data file contents.
		std::ostringstream eoFile;
		eoFile <<
			"# Independent orientation by sequential angles and offsets\n"
			"# Comments start with hash until end of line\n"
			"\n"
			"# NOTE: All three records are required per each sensor.\n"
			"# Convention: <senKey> <convention> # interptation convention\n"
			"# Locations: <senKey> <dist1> <dist2> <dist3>  # 3 distance[m]\n"
			"# Angles: <senKey> <angle1> <angle2> <angle3>  # 3 angles[rad]\n"
			"\n"
			"  Convention: TestSen1 +++ 012 +++ 012 012 0  # convention\n"
			"  Locations:  TestSen1 10.7 -60.7  31.1  # distances[m]\n"
			"  Angles:     TestSen1 -.127  .619 -.317 # physical angles[rad]\n"
			"\n"
			"garbage lines for testing\n"
			"AB C\n"
			"A B C\n"
			;

		// Load indendent exterior body orientations from stream.
		std::istringstream iss(eoFile.str());
		using namespace om;
		std::map<SenKey, SenOri> const indKeyOris{ loadIndEOs(iss) };

		// [DoxyExampleLoadIndEOs]

		bool showFileContents{ false };
		constexpr std::size_t expCount{ 1u };
		if (! (expCount == indKeyOris.size()))
		{
			oss << "Failure of load count test\n";
			oss << "exp: " << expCount << '\n';
			oss << "got: " << indKeyOris.size() << '\n';
			showFileContents = true;
		}
		else
		{
			// check first (and only) orientation
			std::map<SenKey, SenOri>::const_iterator
				const itBeg{ indKeyOris.begin() };

			SenKey const expKey{ "TestSen1" };
			SenKey const & gotKey = itBeg->first;

			using namespace rigibra;
			using namespace engabra::g3; // eij
			// "  Convention: FakeSen1 +++ 012 +++ 012 012 0
			// "  Locations:  FakeSen1 10.7 -60.7  31.1
			Location const loc{ 10.7, -60.7, 31.1 };
			// "  Convention: FakeSen1 +++ 012 +++ 012 012 0
			// "  Angles:     FakeSen1 -.127  .619 -.317
			// NOTE the 3 angle product convention
			//      (ref om::attitudeFrom3AngleSequence())
			Attitude const attA{ PhysAngle{ -.127 * e23 } };
			Attitude const attB{ PhysAngle{  .619 * e31 } };
			Attitude const attC{ PhysAngle{ -.317 * e12 } };
			Attitude const att{ attC * attB * attA };
			// form expected orientation
			SenOri const expOri{ loc, att };

			SenOri const & gotOri = itBeg->second;

			if (! (gotKey == expKey))
			{
				oss << "Failure of load key value test\n";
				oss << "exp: " << expKey << '\n';
				oss << "got: " << gotKey << '\n';
			}

			if (! nearlyEquals(gotOri, expOri))
			{
				oss << "Failure of load orientation test\n";
				oss << "exp: " << expOri << '\n';
				oss << "got: " << gotOri << '\n';
				showFileContents = true;
			}

			if (showFileContents)
			{
				oss << "-----------\n";
				oss << eoFile.str() << std::endl;
				oss << "-----------\n";
			}

		} // bad test

	} // testIndEO

	//! Load ParmGroup data.
	void
	testParmGroup
		( std::ostream & oss
		)
	{
		// [DoxyExampleLoadPG]

		// Example independent EO 3-angle-sequence data file contents.
		std::ostringstream pgFile;
		pgFile <<
			"# Black Box Orientation by 3 angle sizes and 3 offset values\n"
			"# Comments start with hash until end of line\n"
			"\n"
			"# NOTE: All two records are required per each sensor.\n"
			"# Distances: <senKey> <dist1> <dist2> <dist3>  # 3 distance[m]\n"
			"# Angles: <senKey> <angle1> <angle2> <angle3>  # 3 angles[rad]\n"
			"\n"
			"  Distances: TestSen1 10.7 -60.7  31.1 \n"
			"  Angles:    TestSen1 -.127  .619 -.317 \n"
			"\n"
			"garbage lines for testing\n"
			"AB C\n"
			"A B C\n"
			;

		// Load indendent exterior body orientations from stream.
		std::istringstream iss(pgFile.str());
		using namespace om;
		std::map<SenKey, ParmGroup> const pgs{ loadParmGroups(iss) };

		// [DoxyExampleLoadPG]

		constexpr std::size_t expCount{ 1u };
		bool showFileContents{ false };
		if (! (pgs.size() == expCount))
		{
			oss << "Failure of load ParmGroup count test\n";
			oss << "exp: " << expCount << '\n';
			oss << "got: " << pgs.size() << '\n';
			showFileContents = true;
		}
		else
		{
			// have pg group
			std::map<SenKey, ParmGroup>::const_iterator
				const itBeg{ pgs.cbegin() };

			// check key value
			SenKey const expKey{ "TestSen1" };
			SenKey const & gotKey = itBeg->first;
			if (! (gotKey == expKey))
			{
				oss << "Failure of ParmGroup key test\n";
				oss << "exp: " << expKey << '\n';
				oss << "got: " << gotKey << '\n';
				showFileContents = true;
			}

			// check group parameter values
			ParmGroup const gotGroup = itBeg->second;

			ThreeDistances const expDists{ 10.7, -60.7,  31.1 };
			ThreeAngles const expAngles{ -.127,  .619, -.317 };

			// TODO - should be part of test_ParmGroup
		//	ParmGroup const expGroup{ expDists, expAngles };
			ThreeDistances const & gotDists = gotGroup.theDistances;
			ThreeAngles const & gotAngles = gotGroup.theAngles;
			using namespace engabra::g3;
			if (! nearlyEquals(gotDists, expDists))
			{
				oss << "Failure of ParmGroup distances test\n";
				oss << "exp: " << expDists << '\n';
				oss << "got: " << gotDists << '\n';
				showFileContents = true;
			}
			if (! nearlyEquals(gotAngles, expAngles))
			{
				oss << "Failure of ParmGroup angles test\n";
				oss << "exp: " << expAngles << '\n';
				oss << "got: " << gotAngles << '\n';
				showFileContents = true;
			}

			if (showFileContents)
			{
				oss << "-----------\n";
				oss << pgFile.str() << std::endl;
				oss << "-----------\n";
			}
		}


	} // testParmGroup

}

//! Check behavior of NS
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	testIndEO(oss);
	testParmGroup(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}

