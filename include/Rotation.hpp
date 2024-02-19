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


#ifndef OriMania_Rotation_INCL_
#define OriMania_Rotation_INCL_

/*! \file
\brief Contains (limited) functions and utilities for manipulating rotations.

*/


#include <Engabra>

#include <cmath>


namespace om
{

	/*! Classic photogrammetric convention from smaller rotations.
	 *
	 * Algorithm is simplification that assumes rotations are small enough
	 * to stay away from gimbals lock. (i.e., that phi rotation magnitude
	 * is (numerically meaningfully) less than pi/2.
	 *
	 * Example:
	 * \snippet test_Rotation.cpp DoxyExample01
	 */
	std::array<double, 3u>
	opkFrom
		( engabra::g3::Spinor const & spin
		)
	{
		using namespace engabra::g3;
		std::array<double, 3u> opk
			{ null<double>(), null<double>(), null<double>() };

		// access spinor components
		double const & r0 = spin[0];
		double const & r1 = spin[1];
		double const & r2 = spin[2];
		double const & r3 = spin[3];

		// reconstitute preferred rotation matrix elements
		double const r31{ 2.*(r1*r3 + r0*r2) };

		constexpr double tooNear{ 1./262144. }; // arbitrary (2^-18)
		if (! nearlyEquals(std::abs(r31), 1., tooNear))
		{
			double const r11{ r0*r0 + r1*r1 - r2*r2 - r3*r3 };
			double const r21{ 2.*(r1*r2 - r0*r3) };
			double const r32{ 2.*(r2*r3 - r0*r1) };
			double const r33{ r0*r0 - r1*r1 - r2*r2 + r3*r3 };

			// Algorithm formulae (Eberly 1999) are associated
			// with negative angles directions so negate here
			opk[0] = -( std::atan2(r32, r33) );
			opk[1] = -( std::asin(-r31) );
			opk[2] = -( std::atan2(r21, r11) );
		}
		/* Cases near gimbals lock - todo if needed
		else
		if (0. < r31) // (nearlyEquals(r31, -1.))
		{
			// any solution for which: omega - kappa = atan2(-r23,r22)
			phi =  piHalf;
			kap = -atan2(-r23, r22);
			ome = 0.;
		}
		else
		if (r31 < 0.) // (nearlyEquals(r31,  1.))
		{
			// any solution for which: omega + kappa = atan2(-r23,r22)
			phi = -piHalf;
			kap =  atan2(-r23, r22);
			ome = 0.;
		}
		*/

		return opk;
	}

} // [om]


#endif // OriMania_Rotation_INCL_
