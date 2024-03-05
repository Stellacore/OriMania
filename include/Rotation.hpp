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

	/*! \brief Classic photogrammetric convention from smaller rotations.
	 *
	 * Algorithm is simplification that assumes rotations are small enough
	 * to stay away from gimbals lock. (i.e., that phi rotation magnitude
	 * is (numerically meaningfully) less than pi/2.
	 *
	 * Photogrammetric "Omega-Phi-Kappa (OPK)" convention:
	 * \snippet test_Rotation.cpp DoxyExampleSpin
	 *
	 * Example OPK extraction:
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

		/*
		double const sinPhi{ -r31 };
		double const cosPhi{ std::cos(std::asin(sinPhi)) };
		std::cout
			<< " sinPhi: " << io::fixed(sinPhi)
			<< " cosPhi: " << io::fixed(cosPhi)
			<< '\n';
		*/

		constexpr double tooNear{ 1./1024./1024. }; // arbitrary
		if ( nearlyEquals(r31,  1., tooNear)
		  || nearlyEquals(r31, -1., tooNear)
		   )
		{
			std::cerr << "### opkFrom() near gimbals lock - no solution\n";
		}
		else
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

		return opk;
/*
double const rm11{ r0*r0 + r1*r1 - r2*r2 - r3*r3 };
double const rm12{ 2.*(r1*r2 + r0*r3) }; //
double const rm13{ 2.*(r1*r3 - r0*r2) }; //
double const rm21{ 2.*(r1*r2 - r0*r3) };
double const rm22{ r0*r0 - r1*r1 + r2*r2 - r3*r3 };
double const rm23{ 2.*(r2*r3 + r0*r1) }; //
double const rm31{ 2.*(r1*r3 + r0*r2) };
double const rm32{ 2.*(r2*r3 - r0*r1) };
double const rm33{ r0*r0 - r1*r1 - r2*r2 + r3*r3 };

using engabra::g3::io::fixed;
std::cout
	<< "rmat\n"
	<< ' ' << fixed(rm11) << ' ' << fixed(rm12) << ' ' << fixed(rm13) << '\n'
	<< ' ' << fixed(rm21) << ' ' << fixed(rm22) << ' ' << fixed(rm23) << '\n'
	<< ' ' << fixed(rm31) << ' ' << fixed(rm32) << ' ' << fixed(rm33) << '\n'
	;
std::cout << "opk:"
	<< ' ' << io::fixed(opk[0])
	<< ' ' << io::fixed(opk[1])
	<< ' ' << io::fixed(opk[2])
	<< '\n';
*/

		/* Cases near gimbals lock - todo if needed
		else
		if (0. < r31) // (nearlyEquals(r31, -1.))
		{
			// any solution for which: omega - kappa = atan2(-r23,r22)
			opk[0] = 0.;
			opk[1] =  piHalf;
			opk[2] = -atan2(-r23, r22);
		}
		else
		if (r31 < 0.) // (nearlyEquals(r31,  1.))
		{
			// any solution for which: omega + kappa = atan2(-r23,r22)
			opk[0] = 0.;
			opk[1] = -piHalf;
			opk[2] =  atan2(-r23, r22);
		}
		*/
	}

} // [om]


#endif // OriMania_Rotation_INCL_
