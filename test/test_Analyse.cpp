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
	//! Statistic representing error between ori{1,2}. 
	inline
	double
	differenceBetween
		( SenOri const & ori1
		, SenOri const & ori2
		)
	{
		// Location
		using namespace engabra::g3;
		Vector const & loc1 = ori1.theLoc;
		Vector const & loc2 = ori2.theLoc;

		// compute weight for locations
		double const aveMag{ .5 * (magnitude(loc1) + magnitude(loc2)) };
		double wLoc{ 1. };
		if (1. < aveMag)
		{
			wLoc = (1. / aveMag);
		}

		// weighted squared location residuals
		double const residSqLoc{ (wLoc / 3.) * magSq(loc2 - loc1) };

		// Attitude
		using namespace rigibra;
		Attitude const & att1 = ori1.theAtt;
		Attitude const & att2 = ori2.theAtt;

		// weight for attitudes
		constexpr double wAtt{ 1. };

		// weighted squared angle residuals
		BiVector const biv1{ att1.spinAngle().theBiv };
		BiVector const biv2{ att2.spinAngle().theBiv };
		double const residSqAtt{ (wAtt / 3.) * magSq(biv2 - biv1) };

		double const rase{ std::sqrt(.5 * (residSqLoc + residSqAtt)) };
		return rase;
	}

	//! Error between roInd and black box RO computed from pg{1,2}.
	inline
	double
	fitErrorFor
		( ParmGroup const & pg1
		, ParmGroup const & pg2
		, Convention const & convention
		, SenOri const & roInd
		)
	{
		// generate forward transforms internal to black box frame
		SenOri const ori1wB{ convention.transformFor(pg1) };
		SenOri const ori2wB{ convention.transformFor(pg2) };
		// compute relative orientation in black box frame
		SenOri const oriBw1{ inverse(ori1wB) };
		SenOri const roBox{ ori2wB * oriBw1 };
		// compare black box and independent relative orientations
		return differenceBetween(roBox, roInd);
	}

	//! Pair of (fitErrorValue, ConventionArrayIndex))
	using FitNdxPair = std::pair<double, std::size_t>;

	/*! \brief Fit error and allConventions index (ordered best[0] to worst).
	 *
	 * Uses every element of allConventions to transform each of the
	 * two ParmGroup values (both transformed with same convention). For
	 * each resulting pair of orientations (in black box frame) a relative
	 * orientation, roBox, is computed and compared with the provided
	 * independent relative orientation transform, relOri.
	 *
	 * For each convention case, the error between the roBox and roInd
	 * transformations is computed. This fitError value is stored in the
	 * first member of the returned pairs, and the index (of allConventions)
	 * used for the computation is stored in the second member of the pair.
	 *
	 * The resulting collection is then sorted by fitError. Such that
	 * the return structure:
	 * \arg (returnCollection)[0].first -- is the smallest fit error found
	 * \arg (returnCollection)[0].second -- is the index, ndx, to the member
	 * of allConventions[ndx] that was used to obtain the fit error.
	 */
	std::vector<FitNdxPair>
	bestFitConventions
		( ParmGroup const & pg1
		, ParmGroup const & pg2
		, std::vector<Convention> const & allConvetions
		, om::SenOri const & roInd
		)
	{
		std::vector<FitNdxPair> fitConventionPairs;
		for (std::size_t cNdx{0u} ; cNdx < allConvetions.size() ; ++cNdx)
		{
			Convention const & convention = allConvetions[cNdx];
			double const fitError
				{ om::fitErrorFor(pg1, pg2, convention, roInd) };
			fitConventionPairs.emplace_back(std::make_pair(fitError, cNdx));
		}
		std::sort(fitConventionPairs.begin(), fitConventionPairs.end());
		return fitConventionPairs;
	}

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


		// compute consistency score vector for each relative orientation
		std::vector<Convention> const allCons{ Convention::allConventions() };
		for (std::map<KeyPair, SenOri>::value_type
			const & relKeyOri : relKeyOris)
		{
			KeyPair const & keyPair = relKeyOri.first;
			SenOri const & relOri = relKeyOri.second;

			// get parameter groups for these two keys
			std::map<om::SenKey, om::ParmGroup>::const_iterator
				const itFind1{ om::sim::sKeyGroups.find(keyPair.key1()) };
			std::map<om::SenKey, om::ParmGroup>::const_iterator
				const itFind2{ om::sim::sKeyGroups.find(keyPair.key2()) };
			if ( (om::sim::sKeyGroups.end() != itFind1)
			  && (om::sim::sKeyGroups.end() != itFind1)
			   )
			{
				ParmGroup const & pg1 = itFind1->second;
				ParmGroup const & pg2 = itFind2->second;

				std::vector<om::FitNdxPair> const fitConPairs
					{ om::bestFitConventions(pg1, pg2, allCons, relOri) };


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

