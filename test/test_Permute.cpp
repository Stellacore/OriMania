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


#include "io.hpp"
#include "Permute.hpp"
#include "Rotation.hpp"
#include "Simulation.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Simulate contents of an indKeyPGs output file
	std::string
	indPGsFileTextOPK
		( std::map<om::SenKey, om::SenOri> const & indKeyPGs
		, om::Convention const & expIndConv
		)
	{
		std::ostringstream oss;

		// Only OPK attitude convention is supported for this simulation
		static om::Convention const xyzopkConv
			{ om::ConventionOffset{ {  1,  1,  1 }, { 0, 1, 2 } }
			, om::ConventionAngle{ {  1,  1,  1 }, { 0, 1, 2 }, { 0, 1, 2 } }
			, om::RotTran
			};
		if (! (xyzopkConv == expIndConv))
		{
			std::cerr << "FATAL: indPGsFileTextOPK - only supports xyzOPK\n";
			std::cerr << std::flush;
			exit(8);
		}

		// save independent Key orientations with Ind conventions.
		for (std::map<om::SenKey, om::SenOri>::value_type
			const & indKeyPG : indKeyPGs)
		{
			om::SenOri const & ori = indKeyPG.second;
			using namespace engabra::g3;
			Spinor const spin{ ori.theAtt.spinor() };
			std::array<double, 3u> const opk{ om::opkFrom(spin) };
			std::cout
				<< "sen: " << indKeyPG.first
				<< " att: " << indKeyPG.second.theAtt
				<< " opk:"
				<< ' ' << io::fixed(opk[0])
				<< ' ' << io::fixed(opk[1])
				<< ' ' << io::fixed(opk[2])
				<< '\n';
			/*
			*/
			oss
				<< "Distances:"
					<< ' ' << indKeyPG.first
					<< io::fixed(indKeyPG.second.theLoc, 8u, 3u)
				<< '\n'
				<< "Angles:   "
					<< ' ' << indKeyPG.first
					<< ' ' << io::fixed(opk[0], 1u, 9u)
					<< ' ' << io::fixed(opk[1], 1u, 9u)
					<< ' ' << io::fixed(opk[2], 1u, 9u)
				<< '\n';
		}

		return oss.str();
	}

	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace om;

		//
		// Conventions: can be used for both box and ind data
		//              (with different iterators in use).
		//

		/*
		// offset conventions to utilize
		std::vector<ConventionOffset>
			const allOffCons{ ConventionOffset::allConventions() };

		// attitude conventions to utilize
		std::vector<ConventionAngle>
			const allAngCons{ ConventionAngle::allConventions() };
		*/

		//! interpetation alternatives for offset/rotate convention
//		std::array<OrderTR, 2u>
//			const allOrders{ om::allOrderTRs() };

		std::vector<om::Convention> const allBoxCons
			{ Convention::allConventions() };

		//
		// Box frame (simulated data values
		//

		// fetch parameter group values that are known in Box frame.
		std::map<SenKey, ParmGroup> const & boxKeyPGs = sim::sKeyGroups;

		//
		// Ind frame (simulated data values
		//
		Convention const & expBoxConv = sim::sConventionBox;
		Convention const & expIndConv = sim::sConventionInd;

		// simulate independent exterior orientations
		std::map<SenKey, SenOri> const boxKeyOris
			{ sim::boxKeyOris(boxKeyPGs, expBoxConv) };
		std::map<SenKey, SenOri> const indKeyOris
			{ sim::independentKeyOris(boxKeyOris, sim::sXfmBoxWrtRef) };

		// create data file using "OPK convention"
		std::string const indPGsFileText
			{ indPGsFileTextOPK(indKeyOris, expIndConv) };
		std::istringstream issIndPG(indPGsFileText);
		std::map<SenKey, ParmGroup> const indKeyPGs
			{ loadParmGroups(issIndPG) };

		ConventionOffset const indConvOffset{ { 1, 1, 1 }, { 0u, 1u, 2u } };
		std::vector<om::Convention> const allIndCons
			{ Convention::allConventionsFor(indConvOffset) };

/*
		std::vector<om::OneTrialResult> trialResults
			{ om::allTrialResults
				(boxKeyPGs, allBoxCons, indKeyPGs, allIndCons)
			};
*/

		// generate overall trial results



std::cout << "boxKeyOris.size: " << boxKeyOris.size() << '\n';
std::cout << "indKeyOris.size: " << indKeyOris.size() << '\n';
std::cout << "indKeyPGs.size: " << indKeyPGs.size() << '\n';

oss << "Failure: implement (optimized) test\n";

		// [DoxyExample01]

	}

}

//! Check behavior of NS
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);

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

