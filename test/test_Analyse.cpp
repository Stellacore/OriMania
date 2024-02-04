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


namespace
{
	using SenKey = std::string;

	using Ori = rigibra::Transform;
	std::map<SenKey, Ori>
	simEOs
		()
	{
		std::map<SenKey, Ori> oris;
		oris.emplace_hint
			( oris.end()
			, std::make_pair(SenKey("fake1"), rigibra::identity<Ori>())
			);
		oris.emplace_hint
			( oris.end()
			, std::make_pair(SenKey("fake2"), rigibra::identity<Ori>())
			);
		oris.emplace_hint
			( oris.end()
			, std::make_pair(SenKey("fake3"), rigibra::identity<Ori>())
			);
		return oris;
	};

	/*! \brief All combinations of (non trivial) relative orientations.
	 *
	 *
	 */
	std::map<std::pair<SenKey, SenKey>, Ori>
	relativeOrientationBetweens
		( std::map<SenKey, Ori> const & senWrtInds
		)
	{
		std::map<std::pair<SenKey, SenKey>, Ori> roInds;
		for (std::pair<SenKey, Ori> const senWrtInd1 : senWrtInds)
		{
			for (std::pair<SenKey, Ori> const senWrtInd2 : senWrtInds)
			{
				std::pair<SenKey, SenKey> const keyPair
					{ senWrtInd1.first, senWrtInd2.first };
std::cout << "keyPair: " << keyPair.first << ", " << keyPair.second << '\n';
				Ori const & ori1wR = senWrtInd1.second;
				Ori const & ori2wR = senWrtInd2.second;
				Ori const & oriRw1{ inverse(ori1wR) };
				Ori const & ori2w1{ ori2wR * oriRw1 };

std::cout << "ori2w1: " << ori2w1 << '\n';
			}
		}
		return roInds;
	}

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
		std::map<SenKey, rigibra::Transform> const senWrtInds{};


		// generate RO pairs
		std::map<std::pair<SenKey, SenKey>, rigibra::Transform>
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

