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
\brief Unit tests (and example) code for OriMania Rotation.hpp
*/


#include "Rotation.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace
{
	//! Example of "omega,phi,kappa" photogrammetric convention
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace engabra::g3;

		// specify sequential angles
		constexpr double expO{  .3 * pi };
		constexpr double expP{ -.2 * pi };
		constexpr double expK{  .4 * pi };
		BiVector const angO{ expO * e23 };
		BiVector const angP{ expP * e31 };
		BiVector const angK{ expK * e12 };

		// create corresponding spinors
		Spinor const spinO{ exp(.5 * angO) };
		Spinor const spinP{ exp(.5 * angP) };
		Spinor const spinK{ exp(.5 * angK) };

		// create net rotation spinor (sequence from right to left)
		Spinor const spinR{ spinK * spinP * spinO };

		// recover Omega,Phi,Kappa sequence from spinR
		std::array<double, 3u> const opk{ om::opkFrom(spinR) };

		// values returned in "opk" order
		double const & gotO = opk[0]; // 'omega'
		double const & gotP = opk[1]; // 'phi'
		double const & gotK = opk[2]; // 'kappa'

		// [DoxyExample01]

		constexpr double tol{ 2.*std::numeric_limits<double>::epsilon() };
		if ( (! nearlyEquals(gotO, expO, tol))
		  || (! nearlyEquals(gotP, expP, tol))
		  || (! nearlyEquals(gotK, expK, tol))
		   )
		{
			double const difO{ gotO - expO };
			double const difP{ gotP - expP };
			double const difK{ gotK - expK };
			oss << "Failure of opk extraction test\n";
			oss << "expOPK:"
				<< " " << io::fixed(expO)
				<< " " << io::fixed(expP)
				<< " " << io::fixed(expK)
				<< '\n';
			oss << "gotOPK:"
				<< " " << io::fixed(gotO)
				<< " " << io::fixed(gotP)
				<< " " << io::fixed(gotK)
				<< '\n';
			oss << "difOPK:"
				<< " " << io::enote(difO)
				<< " " << io::enote(difP)
				<< " " << io::enote(difK)
				<< '\n';
		}

	}

}

//! Check behavior of Rotation functions
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

