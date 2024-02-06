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


namespace om
{

} // [om]

namespace
{
	//! \brief String containing first few and last few lines of fitConPairs
	inline
	std::string
	infoStringFitCons
		( std::vector<om::FitNdxPair> const & fitConPairs
		, std::vector<om::Convention> const & allCons
		, std::size_t const & numBeg = 8u
		, std::size_t const & numEnd = 2u
		)
	{
		std::ostringstream oss;

		// report a few results
		oss << '\n';
		for (std::size_t nn{0u} ; nn < numBeg ; ++nn)
		{
			oss
				<< om::infoString(fitConPairs[nn], allCons)
				<< '\n';
		}
		oss << " fitError: ..." << '\n';
		for (std::size_t nn{fitConPairs.size()-1u-numEnd}
			; nn < (fitConPairs.size() - 1u) ; ++nn)
		{
			oss
				<< om::infoString(fitConPairs[nn], allCons)
				<< '\n';
		}

		return oss.str();
	}

	//! Check convention extraction from simulated data
	void
	testSim
		( std::ostream & oss
		)
	{
		using namespace om;

		om::Convention const convention{ om::sim::sConventionA };

		// if true report various simulation data values
		constexpr bool show{ false };

		//
		// Simulation
		//

		// report configuration
		if (show)
		{
			std::cout << '\n';
			std::cout << "using convention: " << convention << '\n';
		}

	 	// [DoxyExample01]

		//
		// Simulation of input data
		//

		// Simulate: configuration of a payload system
		// in which sensor ExCal data are using some unknown
		// arbitrary convention (here sConventionA is assumed unknown)
		std::map<SenKey, SenOri> const boxKeyOris
			{ om::sim::boxKeyOris(om::sim::sKeyGroups, om::sim::sConventionA) };

		// Simulate: exported indendent exterior body orientations
		std::map<SenKey, SenOri> const indKeyOris
			{ om::sim::independentKeyOris(boxKeyOris) };

		// Get block box parameter groupings
		// (here from simulation data, but in general should load these)
		std::map<om::SenKey, om::ParmGroup>
			const & keyGroups = om::sim::sKeyGroups;

		//
		// Determine which parameter conventions best match input data
		//

		// Compute fit error for each convention index
		std::vector<Convention> const allCons{ Convention::allConventions() };
		std::vector<FitNdxPair> fitIndexPairs
			{ fitIndexPairsFor(keyGroups, indKeyOris, allCons) };

		// Find the convention with the smallest error
		// (for test here sort full collection to assess significance
		// of the best error in context of other values - e.g. prominence).
		std::sort(fitIndexPairs.begin(), fitIndexPairs.end());
		double prominenceFraction{ engabra::g3::nan };
		if (2u < fitIndexPairs.size())
		{
			// check prominence 
			double const & currBest = fitIndexPairs[0].first;
			double const & nextBest = fitIndexPairs[1].first;
			double const & lastBest = fitIndexPairs.back().first;
			if (std::numeric_limits<double>::epsilon() < lastBest)
			{
				prominenceFraction = (nextBest - currBest) / lastBest;
			}
		}

	 	// [DoxyExample01]

		// check if correct number are computed
		if (! (allCons.size() == fitIndexPairs.size()))
		{
			oss << "Failure of fitIndexPairs size test\n";
			oss << "exp: " << allCons.size() << '\n';
			oss << "got: " << fitIndexPairs.size() << '\n';
		}
		else
		{
			std::size_t const expConventionId{ convention.asNumber() };
			std::size_t const gotConventionId
				{ allCons[fitIndexPairs[0].second].asNumber() };
			if (! (gotConventionId == expConventionId))
			{
				oss << "Failure of find best convention test\n";
				oss << "exp: " << expConventionId << '\n';
				oss << "got: " << gotConventionId << '\n';
				oss << "\nSampling of Results\n";
				oss << infoStringFitCons(fitIndexPairs, allCons) << '\n';
			}

			constexpr double tolFrac{ .05 }; // inspection of simulation data
			if (! (tolFrac < prominenceFraction))
			{
				oss << "Failure of prominence fraction test\n";
				oss << "exp: more than fraction = " << tolFrac << '\n';
				oss << "got: fraction = " << prominenceFraction << '\n';
			}

		} // check found convention

	} // testSim

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

