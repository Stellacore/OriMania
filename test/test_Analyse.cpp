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
	using PG = om::ParmGroup;
	//! A diverse selection of angle and distance parameters
	static std::map<om::SenKey, om::ParmGroup> const sKeyGroups
		{ { "pg0", PG{ {    .0,    .0,    .0 }, {  .000,  .000,  .000,} } }
		, { "pg1", PG{ { -60.1,  10.3,  21.1 }, {  .617, -.113, -.229 } } }
		, { "pg2", PG{ {  10.7, -60.7,  31.1 }, { -.127,  .619, -.317 } } }
		, { "pg3", PG{ {  30.7,  22.7, -61.3 }, { -.331, -.631,  .239 } } }
		, { "pg4", PG{ {  10.1, -40.9, -50.3 }, { -.109,  .421,  .523 } } }
		, { "pg5", PG{ { -41.9,  22.3, -52.1 }, {  .431, -.233,  .541 } } }
		, { "pg6", PG{ { -40.1, -50.9,  31.3 }, {  .433,  .547, -.337 } } }
		};

	// TODO - run test over many (all?) different conventions.
	//! An arbitrarily set convention
	static om::Convention const sConventionA
		{ {  1,  1, -1 }
		, { 1, 0, 2 }
		, {  1, -1,  1 }
		, { 0, 1, 2 }
		, { 1, 2, 1 }
		, om::RotTran
		};

	//! An arbitrary orientation for Box frame w.r.t. arbitrary Ref frame.
	static rigibra::Transform const sXfmBoxWrtRef
		{ rigibra::Location{ 1000., 2000., 3000. }
		, rigibra::Attitude(rigibra::PhysAngle{ -.7, 1.5, 3. })
		};


} // [sim]


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

} // [om]

namespace sim
{
	/*! \brief Simulate orientation of sensors wrt black box frame.
	 *
	 * uses
	 */
	std::map<om::SenKey, om::SenOri>
	boxKeyOris
		( std::map<om::SenKey, om::ParmGroup> const & keyGroups
		, om::Convention const & convention
		)
	{
		using namespace om;
		std::map<SenKey, SenOri> keyOris;
		for (std::map<SenKey, ParmGroup>::value_type
			const & keyGroup : keyGroups)
		{
			rigibra::Transform const xSenWrtBox
				{ convention.transformFor(keyGroup.second) };
			keyOris[keyGroup.first] = xSenWrtBox;
		}
		return keyOris;
	}

	/*! \brief Simulate export of the body orientation data.
	 *
	 * Orientations are assumed relative to some arbitary black Box
	 * orienation (sXfmBoxWrtRef)
	 */
	std::map<om::SenKey, om::SenOri>
	independentKeyOris
		( std::map<om::SenKey, om::SenOri> const & boxKeyOris
		)
	{
		using namespace om;
		std::map<SenKey, SenOri> refKeyOris;
		for (std::map<SenKey, SenOri>::value_type
			const & boxKeyOri : boxKeyOris)
		{
			using namespace rigibra;
			SenKey const & key = boxKeyOri.first;
			SenOri const & oriSenWrtBox = boxKeyOri.second;
			SenOri const & oriBoxWrtRef = sim::sXfmBoxWrtRef;
			SenOri const oriSenWrtRef{ oriSenWrtBox * oriBoxWrtRef };
			refKeyOris[key] = oriSenWrtRef;
		}
		return refKeyOris;
	}

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

		// simulate configuration of a payload system
		// in which sensor ExCal data are using some unknown
		// arbitrary convention (here sConventionA is assumed unknown)
std::cout << '\n';
std::cout << "using convention: " << sim::sConventionA.asNumber() << '\n';
		std::map<SenKey, SenOri> const boxKeyOris
			{ sim::boxKeyOris(sim::sKeyGroups, sim::sConventionA) };

		// simulated exported indendent exterior body orientations
		std::map<SenKey, SenOri> const refKeyOris
			{ sim::independentKeyOris(boxKeyOris) };

		// generate ROs from indepent exterior body orientations
		std::map<KeyPair, SenOri> const relKeyOris
			{ om::relativeOrientationBetweens(refKeyOris) };



		//
		// report results
		//

		// report simulated independent orientations
		std::ostringstream msgIndEOs;
		msgIndEOs << "\nSimulated independent orientations:\n";
		for (std::map<SenKey, SenOri>::value_type
			const & refKeyOri : refKeyOris)
		{
			msgIndEOs
				<< "RO: " << refKeyOri.first
				<< "  oriSenWrtRef: " << refKeyOri.second
				<< '\n';
		}
		msgIndEOs << '\n';
		std::cout << msgIndEOs.str() << '\n';

		// report relative orientations computed from indpendent EOs
		std::ostringstream msgROs;
		msgROs << "\nRelative Orientations (in independent frame)\n";
		for (std::map<KeyPair, SenOri>::value_type
			const & relKeyOri : relKeyOris)
		{
			msgROs
				<< "keyPair: " << relKeyOri.first
				<< "  ro: " << relKeyOri.second
				<< '\n';
		}
		std::cout << msgROs.str() << '\n';

		// compute consistency score vector for each relative orientation
std::cout << '\n';
		std::vector<Convention> const allCons{ Convention::allConventions() };
		for (std::map<KeyPair, SenOri>::value_type
			const & relKeyOri : relKeyOris)
		{
			KeyPair const & keyPair = relKeyOri.first;
			SenOri const & relOri = relKeyOri.second;

			// get parameter groups for these two keys
			std::map<om::SenKey, om::ParmGroup>::const_iterator
				const itFind1{ sim::sKeyGroups.find(keyPair.key1()) };
			std::map<om::SenKey, om::ParmGroup>::const_iterator
				const itFind2{ sim::sKeyGroups.find(keyPair.key2()) };
			if ( (sim::sKeyGroups.end() != itFind1)
			  && (sim::sKeyGroups.end() != itFind1)
			   )
			{
				ParmGroup const & pg1 = itFind1->second;
				ParmGroup const & pg2 = itFind2->second;

std::cout << "\nscore RO: "
	<< "keyPair: " << keyPair
	<< "\n " << keyPair.key1() << " " << pg1
	<< "\n " << keyPair.key2() << " " << pg2
	<< '\n';

				//! Fit error, convention array index
				using FitNdxPair = std::pair<double, std::size_t>;
				std::vector<FitNdxPair> fitConPairs;
				for (std::size_t cNdx{0u} ; cNdx < allCons.size() ; ++cNdx)
				{
					Convention const & convention = allCons[cNdx];
					double const fitErr
						{ om::fitErrorFor(pg1, pg2, convention, relOri) };
					fitConPairs.emplace_back(std::make_pair(fitErr, cNdx));
				}
				std::sort(fitConPairs.begin(), fitConPairs.end());

std::size_t aFew{ 8u };
for (std::size_t nn{0u} ; nn < aFew ; ++nn)
{
	using namespace engabra::g3;
	double const & fitErr = fitConPairs[nn].first;
	Convention const & convention = allCons[fitConPairs[nn].second];
	std::cout
		<< " fitErr: " << io::fixed(fitErr)
		<< "  convention: " << convention.asNumber()
		<< '\n';
}
std::cout << " fitErr: ..." << '\n';
std::size_t nMax{ fitConPairs.size() };
for (std::size_t nn{nMax-aFew} ; nn < nMax-1u ; ++nn)
{
	using namespace engabra::g3;
	double const & fitErr = fitConPairs[nn].first;
	Convention const & convention = allCons[fitConPairs[nn].second];
	std::cout
		<< " fitErr: " << io::fixed(fitErr)
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

