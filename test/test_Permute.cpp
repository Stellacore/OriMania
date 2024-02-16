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


#include "Permute.hpp"
#include "Simulation.hpp"

#include <iostream>
#include <sstream>


namespace
{
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

		// offset conventions to utilize
		std::vector<ConventionOffset>
			const allOffCons{ om::ConventionOffset::allConventions() };

		// attitude conventions to utilize
		std::vector<ConventionAngle>
			const allAngCons{ om::ConventionAngle::allConventions() };

		//! interpetation alternatives for offset/rotate convention
//		std::array<OrderTR, 2u>
//			const allOrders{ om::allOrderTRs() };

		//
		// Box frame (simulated data values
		//

		// fetch parameter group values that are known in Box frame.
		std::map<SenKey, ParmGroup> const & boxKeyPGs = sim::sKeyGroups;

		//
		// Ind frame (simulated data values
		//
		om::Convention const & expBoxConv = sim::sConventionBox;
//		om::Convention const expIndConv = sim::sConventionInd;

		// simulate inependent exterior orientations
		std::map<om::SenKey, om::SenOri> const boxKeyOris
			{ sim::boxKeyOris(boxKeyPGs, expBoxConv) };
		std::map<SenKey, SenOri> const indKeyPGs
			{ sim::independentKeyOris(boxKeyOris, om::sim::sXfmBoxWrtRef) };

		// save independent Key orientations with Ind conventions.

		oss << "Failure: TODO/Need to decompose/save 3 angle conventions\n";

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

