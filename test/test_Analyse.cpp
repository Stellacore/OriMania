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


#include "Analysis.hpp"
#include "Convention.hpp"
#include "io.hpp"
#include "Orientation.hpp"

#include "Simulation.hpp"

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
	//! Check convention extraction from simulated data
	void
	testSim
		( std::ostream & oss
		)
	{
		using namespace om;

		// simulate configuration of a payload system
		// in which sensor ExCal data are using some unknown
		// arbitrary convention (here sConventionA is assumed unknown)
std::cout << '\n';
std::cout << "using convention: " << om::sim::sConventionA.asNumber() << '\n';
		std::map<SenKey, SenOri> const boxKeyOris
			{ om::sim::boxKeyOris(om::sim::sKeyGroups, om::sim::sConventionA) };

		// simulated exported indendent exterior body orientations
		std::map<SenKey, SenOri> const indKeyOris
			{ om::sim::independentKeyOris(boxKeyOris) };

		// generate ROs from indepent exterior body orientations
		std::map<KeyPair, SenOri> const relKeyOris
			{ om::relativeOrientationBetweens(indKeyOris) };

		//
		// report simulated data results
		//

		// report simulated independent orientations
		std::ostringstream msgIndEOs;
		msgIndEOs << "\nSimulated independent orientations:\n";
		msgIndEOs << indKeyOris << '\n';
		std::cout << msgIndEOs.str() << '\n';

		// report relative orientations computed from indpendent EOs
		std::ostringstream msgROs;
		msgROs << "\nRelative Orientations (in independent frame)\n";
		msgROs << relKeyOris << '\n';
		std::cout << msgROs.str() << '\n';

		std::vector<Convention> const allCons{ Convention::allConventions() };
		std::vector<om::FitNdxPair> const fitConPairs
			{ fitConventionPairs(om::sim::sKeyGroups, relKeyOris, allCons) };

		// check if correct number are computed
		if (! (allCons.size() == fitConPairs.size()))
		{
			oss << "Failure of fitConPairs size test\n";
			oss << "exp: " << allCons.size() << '\n';
			oss << "got: " << fitConPairs.size() << '\n';
		}
		else
		{
			// report a few results
			std::size_t aFew{ 2u };
			std::cout << '\n';
			for (std::size_t nn{0u} ; nn < aFew ; ++nn)
			{
				using namespace engabra::g3;
				double const & fitError = fitConPairs[nn].first;
				Convention const & convention = allCons[fitConPairs[nn].second];
				std::cout
					<< " fitError: " << io::fixed(fitError)
					<< "  convention: " << convention.asNumber()
					<< '\n';
			}

		}


		// TODO replace this with real test code
		std::string const fname(__FILE__);
		bool const isTemplate{ (std::string::npos != fname.find("/_.cpp")) };
		if (! isTemplate)
		{
			oss << "Failure to implement real test\n";
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

