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
	std::map<om::SenKey, om::SenOri>
	simEOs
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

		//
		// Simulate sensor data
		//   1) Individual ori of each sensor in system 'Box' frame
		//   2) Relative orienation between objects in some arbitrary frame.
		//

		// simulate arbitrary ExCal orientations
		std::vector<rigibra::Transform> const senWrtBoxs{};

		// use the ExCal data to generate Independent EO data
		std::map<om::SenKey, rigibra::Transform> const senWrtInds{};


		// generate RO pairs
		std::map<om::KeyPair, rigibra::Transform>
			const roXforms{ relativeOrientationBetweens(senWrtInds) };

		// consider common conventions
		std::vector<om::Convention> conventions{};

		// compute BoxRo
			// candidate for Sen1wBox
			// candidate for Sen2wBox
//			BoxRo = Sen2wBox * inverse(Sen1wBox);
		// IndRo
//			IndRo = Sen2wInd * inverse(Sen1wInd);

		// compare and assign metric
//		std::map<CHash, Metric>


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

