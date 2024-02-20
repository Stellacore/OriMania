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
#include <vector>


namespace
{
	//! Construct spinor from omega,phi,kappa (physical) angle sizes
	inline
	engabra::g3::Spinor
	spinorFrom
		( std::array<double, 3u> const & opks
		)
	{
		// [DoxyExampleSpin]

		// *physical* angle sizes
		double const & expO = opks[0]; // 'omega'
		double const & expP = opks[1]; // 'phi'
		double const & expK = opks[2]; // 'kappa'

		// create corresponding spinors
		using namespace engabra::g3;
		Spinor const spinO{ exp(.5 * expO * e23) }; // x axis
		Spinor const spinP{ exp(.5 * expP * e31) }; // rotated y' axis
		Spinor const spinK{ exp(.5 * expK * e12) }; // double rotated z'' axis

		// create net rotation spinor (sequence from right to left)
		Spinor const spinR{ spinK * spinP * spinO };

		// [DoxyExampleSpin]

		return spinR;
	}

	//! Test om::opkFrom() extraction in terms of OPK angle size values
	inline
	void
	checkOPK
		( std::ostringstream & oss
		, std::array<double, 3u> const & expOPK
		, std::string const & tname
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		using namespace engabra::g3;

		// physical angle sizes
		double const & expO = expOPK[0]; // 'omega'
		double const & expP = expOPK[1]; // 'phi'
		double const & expK = expOPK[2]; // 'kappa'

		// create net rotation spinor (sequence from right to left)
		Spinor const spinR{ spinorFrom(expOPK) };

		// [DoxyExample01]

		// extract omega,phi,kappa values
		std::array<double, 3u> const gotOPK{ om::opkFrom(spinR) };

		// values returned in "opk" order
		double const & gotO = gotOPK[0]; // 'omega'
		double const & gotP = gotOPK[1]; // 'phi'
		double const & gotK = gotOPK[2]; // 'kappa'

		// [DoxyExample01]

		// check if results are consistent
		if ( (! nearlyEquals(gotO, expO, tol))
		  || (! nearlyEquals(gotP, expP, tol))
		  || (! nearlyEquals(gotK, expK, tol))
		   )
		{
			double const difO{ gotO - expO };
			double const difP{ gotP - expP };
			double const difK{ gotK - expK };
			oss << "Failure of opk extraction test " << tname << '\n';
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
				<< " " << io::enote(difO, 5u)
				<< " " << io::enote(difP, 5u)
				<< " " << io::enote(difK, 5u)
				<< '\n';
		}
	}

	//! Positive scalar grade spinor (if anySpin[0]<0, then return -anySpin)
	inline
	engabra::g3::Spinor
	posSpin
		( engabra::g3::Spinor const anySpin
		)
	{
		engabra::g3::Spinor spin;
		if (anySpin[0] < 0.)
		{
			spin = -anySpin;
		}
		else
		{
			spin =  anySpin;
		}
		return spin;
	}

	inline
	bool
	sameSpin
		( engabra::g3::Spinor const & spinAwX
		, engabra::g3::Spinor const & spinBwX
		, double const & tol
		)
	{
		using namespace engabra::g3;
		Spinor const spinXwB{ reverse(spinBwX) };
		static Spinor const expNet{ one<Spinor>() };
		Spinor const gotNet{ spinAwX * spinXwB };
		Spinor const gotPos{ posSpin(gotNet) };
		return nearlyEquals(gotPos, expNet, tol);
	}

	//! Test om::opkFrom() extraction in terms of reconstituted spinor
	inline
	void
	checkSpin
		( std::ostringstream & oss
		, std::array<double, 3u> const & expOPK
		, std::string const & tname
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		using namespace engabra::g3;

		Spinor const expSpin{ spinorFrom(expOPK) };

		// extract omega,phi,kappa values
		/*
		std::cout << "#opk: "
			<< ' ' << io::fixed(expOPK[0])
			<< ' ' << io::fixed(expOPK[1])
			<< ' ' << io::fixed(expOPK[2])
			<< '\n';
		*/

		std::array<double, 3u> const gotOPK{ om::opkFrom(expSpin) };

		Spinor const gotSpin{ spinorFrom(gotOPK) };

		if (! sameSpin(gotSpin, expSpin, tol))
		{
			Spinor const difSpin{ gotSpin - expSpin };
			double const & expO = expOPK[0];
			double const & expP = expOPK[1];
			double const & expK = expOPK[2];
			double const & gotO = gotOPK[0];
			double const & gotP = gotOPK[1];
			double const & gotK = gotOPK[2];
			double const difO{ gotO - expO };
			double const difP{ gotP - expP };
			double const difK{ gotK - expK };
			oss << '\n';
			oss << "Failure of opk/spin reconstruction test " << tname << '\n';
			oss << "expSpin:" << io::fixed(expSpin) << '\n';
			oss << "gotSpin:" << io::fixed(gotSpin) << '\n';
			oss << "difSpin:" << io::enote(difSpin) << '\n';
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
				<< " " << io::enote(difO, 5u)
				<< " " << io::enote(difP, 5u)
				<< " " << io::enote(difK, 5u)
				<< '\n';
		}
	}

	//! Generate a collection of angle sizes
	std::vector<double>
	angleSizes
		( double const & max
		, double const & min
		, std::size_t const & num = 2u
		)
	{
		std::vector<double> sizes;
		sizes.reserve(num);
		double const del{ (max-min) / (double)(num-1u) };
		for (std::size_t nn{0u} ; nn < num ; ++nn)
		{
			double const sz{ min + (double)nn * del };
			sizes.emplace_back(sz);
		}
		return sizes;
	}

	// combine all combinations of sizes into triplets 
	std::vector<std::array<double, 3u> >
	triosFrom
		( std::vector<double> const & sizes
		)
	{
		using OPK = std::array<double, 3u>;
		std::vector<OPK> opks;
		std::size_t const num(sizes.size());
		opks.reserve(num*num*num);
		for (double const & omega : sizes)
		{
			for (double const & phi : sizes)
			{
				for (double const & kappa : sizes)
				{
					opks.emplace_back(OPK{ omega, phi, kappa });
				}
			}
		}
		return opks;
	}

	//! Test one case (useful for development exploration)
	void
	testOne
		( std::ostringstream & oss
		)
	{
		// specify sequential angles
		using engabra::g3::pi;
		/*
		constexpr std::array<double, 3u> const opk
			{ (  .7 * pi) // omega about 'x' (e23)
			, ( -.8 * pi) // phi about 'y' (e31)
			, (  .9 * pi) // kappa about 'z' (e12)
			};
		*/
		constexpr std::array<double, 3u> const opk{ 3., 1.9, 3. };
		constexpr double tol{ 16.*std::numeric_limits<double>::epsilon() };
		checkSpin(oss, opk, "testOne", tol);
	}

	//! Test a multitude of opk extractions
	void
	testMany
		( std::ostringstream & oss
		)
	{
		using namespace engabra::g3;
		using OPK = std::array<double, 3u>;

		double const min{ -pi };
		double const max{  pi };
		std::size_t const num{ 31u }; // odd value avoids pi/2 gimbals lock
		std::vector<double> const angSizes{ angleSizes(min, max, num) };
		std::vector<OPK> const opks{ triosFrom(angSizes) };
		for (OPK const & opk : opks)
		{
			// algorithm is sensitive
			constexpr double tol{ 128.*std::numeric_limits<double>::epsilon() };
			checkSpin(oss, opk, "testMany", tol);
		}
	}

/*
	//! Test gimbals lock configurations - TODO needs implementation
	void
	testLock
		( std::ostringstream & oss
		)
	{
		using namespace engabra::g3;
		using OPK = std::array<double, 3u>;

		double const min{ -1.0 * pi };
		double const max{  1.0 * pi };
		std::size_t const num{ 3u };
		std::vector<double> const angSizes{ angleSizes(min, max, num) };
		std::vector<OPK> const opks{ triosFrom(angSizes) };
		for (OPK const & opk : opks)
		{

			std::cout << "opk:"
				<< ' ' << io::fixed(opk[0])
				<< ' ' << io::fixed(opk[1])
				<< ' ' << io::fixed(opk[2])
				<< '\n';

			constexpr double tol{ 16.*std::numeric_limits<double>::epsilon() };
			checkSpin(oss, opk, "testLock", tol);
			if (! oss.str().empty())
			{
				break;
			}
		}
	}
*/
}

//! Check behavior of Rotation functions
int
main
	()
{
	int status{ 1 };
	std::ostringstream oss;

	testOne(oss);
	testMany(oss);
///	testLock(oss);

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

