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
\brief Unit tests (and example) code for OriMania::NS::CN
*/


#include "Orientation.hpp"
#include "Convention.hpp"
//#include "OriMania.hpp"

#include <Rigibra>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>


namespace sim
{
	//! A diverse selection of angle and distance parameters
	static std::vector<om::ParmGroup> const sParmGroups
		{ om::ParmGroup{ {   .0,   .0,   .0 }, { .000, .000, .000,} }
		, om::ParmGroup{ { 60.1, 10.3, 21.1 }, { .617, .113, .229 } }
		, om::ParmGroup{ { 10.7, 60.7, 31.1 }, { .127, .619, .317 } }
		, om::ParmGroup{ { 30.7, 22.7, 61.3 }, { .331, .631, .239 } }
		, om::ParmGroup{ { 10.1, 40.9, 50.3 }, { .109, .421, .523 } }
		, om::ParmGroup{ { 41.9, 22.3, 52.1 }, { .431, .233, .541 } }
		, om::ParmGroup{ { 40.1, 50.9, 31.3 }, { .433, .547, .337 } }
		};
		/* Angle-Distance order
		{ om::ParmGroup{ { .000, .000, .000,}, {   .0,   .0,   .0 } }
		, om::ParmGroup{ { .617, .113, .229 }, { 60.1, 10.3, 21.1 } }
		, om::ParmGroup{ { .127, .619, .317 }, { 10.7, 60.7, 31.1 } }
		, om::ParmGroup{ { .331, .631, .239 }, { 30.7, 22.7, 61.3 } }
		, om::ParmGroup{ { .109, .421, .523 }, { 10.1, 40.9, 50.3 } }
		, om::ParmGroup{ { .431, .233, .541 }, { 41.9, 22.3, 52.1 } }
		, om::ParmGroup{ { .433, .547, .337 }, { 40.1, 50.9, 31.3 } }
		};
		*/

	//! An arbitrarily set convention
	static om::Convention const sConventionA
		{ { -1, 1, -1 }
		, { 1, 0, 2 }
		, { -1,-1,  1 }
		, { 2, 1, 0 }
		, { 1, 2, 1 }
		, om::RotTran
		};

	std::map<om::SenKey, om::SenOri>
	senExCals
		()
	{
		std::map<om::SenKey, om::SenOri> oris;
		oris.emplace_hint
			( oris.end()
			, std::make_pair
				(om::SenKey("fake1"), rigibra::identity<om::SenOri>())
			);
		oris.emplace_hint
			( oris.end()
			, std::make_pair
				(om::SenKey("fake2"), rigibra::identity<om::SenOri>())
			);
		oris.emplace_hint
			( oris.end()
			, std::make_pair
				(om::SenKey("fake3"), rigibra::identity<om::SenOri>())
			);
		return oris;
	};

} // [sim]

namespace
{

	//! Check convention extraction from simulated data
	void
	testSim
		( std::ostream & oss
		)
	{
		using namespace om;

std::cout << '\n';
		for (om::ParmGroup const & parmGroup : sim::sParmGroups)
		{
			rigibra::Transform const xfm
				{ sim::sConventionA.transformFor(parmGroup) };
std::cout << "ParmGroup xfm: " << xfm << '\n';
		}


		//
		// Simulate sensor data
		//   1) Individual ori of each sensor in system 'Box' frame
		//   2) Relative orienation between objects in some arbitrary frame.
		//

		// simulate arbitrary ExCal orientations
		std::map<om::SenKey, om::SenOri> const senWrtBoxs{ sim::senExCals() };

		// use the ExCal data to generate Independent EO data
		std::map<om::SenKey, rigibra::Transform> const senWrtInds{};


		// generate RO pairs
		std::map<om::KeyPair, rigibra::Transform>
			const roXforms{ relativeOrientationBetweens(senWrtInds) };

		// consider common conventions
		std::vector<om::Convention> conventions{};


		// [DoxyExample01]

		// [DoxyExample01]

		// TODO replace this with real test code
		constexpr bool successful{ false };
		if (! successful)
		{
			oss << "Failure of testSim implementation test\n";
		}
	}

}

//! Check convention recovery with simulated data
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	testSim(oss);

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

