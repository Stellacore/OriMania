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
	//! Check convention extraction from simulated data
	void
	testSim
		( std::ostream & oss
		)
	{
		using namespace om;

		om::Convention const expConvention{ om::sim::sConventionBox };

		// if true report various simulation data values
		constexpr bool showData{ true };

		//
		// Simulation
		//

		// report configuration
		if (showData)
		{
			std::cout << '\n';
			std::cout << "using expConvention: " << expConvention << '\n';
		}

	 	// [DoxyExample01]

		//
		// Simulation of external data
		//

		// Simulate: configuration of a payload system
		// in which sensor ExCal data are using some unknown
		// arbitrary convention (here sConventionBox is assumed unknown)
		// 
		// NOTE: the conventions used to generate these data are the
		//       unknown values to be determined by solution code below.
		//
		std::map<SenKey, SenOri> const boxKeyOris
			{ om::sim::boxKeyOris(om::sim::sKeyGroups, expConvention) };

		//
		// "Load" input data
		//

		// Simulate: exported indendent exterior body orientations
		std::map<SenKey, SenOri> const indKeyOris
			{ om::sim::independentKeyOris(boxKeyOris, om::sim::sXfmBoxWrtRef) };

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

		// show data values (e.g. for dev use)
		if (showData)
		{
			std::ostringstream msg;

			msg << '\n';
			msg << "\nParmGroups in use:\n";
			for (std::map<om::SenKey, om::ParmGroup>::value_type
				const & keyGroup : keyGroups)
			{
				msg << keyGroup.first
					<< " " << keyGroup.second
					<< '\n';
			}
			msg << "\nIndpendent EOs:\n";
			for (std::map<SenKey, SenOri>::value_type
				const & indKeyOri : indKeyOris)
			{
				msg
					<< indKeyOri.first
					<< " " << indKeyOri.second
					<< '\n';
			}
			msg << "\nSolution Sample:\n";
			msg << infoStringFitConventions(fitIndexPairs, allCons) << '\n';
			msg << '\n';

			std::cout << msg.str();
		}

		// check if correct number are computed
		if (! (allCons.size() == fitIndexPairs.size()))
		{
			oss << "Failure of fitIndexPairs size test\n";
			oss << "exp: " << allCons.size() << '\n';
			oss << "got: " << fitIndexPairs.size() << '\n';
		}
		else
		{
			std::int64_t const expConventionId
				{ expConvention.numberEncoding() };
			std::int64_t const gotConventionId
				{ allCons[fitIndexPairs[0].second].numberEncoding() };
			if (! (gotConventionId == expConventionId))
			{
				oss << "Failure of find best convention test\n";
				oss << "exp: " << expConventionId << '\n';
				oss << "got: " << gotConventionId << '\n';
				oss << "\nSampling of Results\n";
				oss << infoStringFitConventions(fitIndexPairs, allCons) << '\n';
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

