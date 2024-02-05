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
		{ { "pg0", PG{ {   .0,   .0,   .0 }, { .000, .000, .000,} } }
		, { "pg1", PG{ { 60.1, 10.3, 21.1 }, { .617, .113, .229 } } }
		, { "pg2", PG{ { 10.7, 60.7, 31.1 }, { .127, .619, .317 } } }
		, { "pg3", PG{ { 30.7, 22.7, 61.3 }, { .331, .631, .239 } } }
		, { "pg4", PG{ { 10.1, 40.9, 50.3 }, { .109, .421, .523 } } }
		, { "pg5", PG{ { 41.9, 22.3, 52.1 }, { .431, .233, .541 } } }
		, { "pg6", PG{ { 40.1, 50.9, 31.3 }, { .433, .547, .337 } } }
		};
		/* Angle-Distance order
		{ PG{ { .000, .000, .000,}, {   .0,   .0,   .0 } }
		, PG{ { .617, .113, .229 }, { 60.1, 10.3, 21.1 } }
		, PG{ { .127, .619, .317 }, { 10.7, 60.7, 31.1 } }
		, PG{ { .331, .631, .239 }, { 30.7, 22.7, 61.3 } }
		, PG{ { .109, .421, .523 }, { 10.1, 40.9, 50.3 } }
		, PG{ { .431, .233, .541 }, { 41.9, 22.3, 52.1 } }
		, PG{ { .433, .547, .337 }, { 40.1, 50.9, 31.3 } }
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

	//! An arbitrary orientation for Box frame w.r.t. arbitrary Ref frame.
	static rigibra::Transform const sXfmBoxWrtRef
		{ rigibra::Location{ 1000., 2000., 3000. }
		, rigibra::Attitude(rigibra::PhysAngle{ -.7, 1.5, 3. })
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

		// simulate configuration of a payload system
		// in which sensor ExCal data are using some unknown
		// arbitrary convention (here sConventionA is assumed unknown)
std::cout << '\n';
std::cout << "using convention: " << sim::sConventionA.asNumber() << '\n';
		std::map<SenKey, SenOri> boxKeyOris;
		for (std::map<SenKey, om::ParmGroup>::value_type
			const & keyGroup : sim::sKeyGroups)
		{
			rigibra::Transform const xSenWrtBox
				{ sim::sConventionA.transformFor(keyGroup.second) };
			boxKeyOris[keyGroup.first] = xSenWrtBox;
		}

		// simulate export of the camera orientation data relative
		// to some arbitary Box orienation (sXfmBoxWrtRef)
		using namespace rigibra;
		std::map<SenKey, SenOri> refKeyOris;
		for (std::map<SenKey, SenOri>::value_type
			const & boxKeyOri : boxKeyOris)
		{
			SenKey const & key = boxKeyOri.first;
			SenOri const & oriSenWrtBox = boxKeyOri.second;
			SenOri const & oriBoxWrtRef = sim::sXfmBoxWrtRef;
			SenOri const oriSenWrtRef{ oriSenWrtBox * oriBoxWrtRef };
			refKeyOris[key] = oriSenWrtRef;
		}

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

		// generate ROs from this
		std::map<KeyPair, SenOri> const relKeyOris
			{ om::relativeOrientationBetweens(refKeyOris) };

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

