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
\brief Unit tests (and example) code for OriMania om::Orientation
*/


#include "io.hpp"
#include "Orientation.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Check simple things
	void
	test0
		( std::ostream & oss
		)
	{
		using namespace om;

		SenKey const expKey1{ "key1" };
		SenKey const expKey2{ "key2" };

		KeyPair const aPair{ expKey1, expKey2 };

		SenKey const gotKey1{ aPair.key1() };
		SenKey const gotKey2{ aPair.key2() };

		if (! (gotKey1 == expKey1))
		{
			oss << "Failure of key1 order test\n";
			oss << "exp: " << expKey1 << '\n';
			oss << "got: " << gotKey1 << '\n';
		}

		if (! (gotKey2 == expKey2))
		{
			oss << "Failure of key2 order test\n";
			oss << "exp: " << expKey2 << '\n';
			oss << "got: " << gotKey2 << '\n';
		}
	}

	//! Examples for documentation
	void
	testRelOrientations
		( std::ostream & oss
		)
	{
		using namespace om;

		// alias to clarify code
		using RelOri = SenOri;

		using namespace engabra::g3;
		Vector const locDel{ .2, .3, .4 };
		Vector loc{ -4., 3., -5. }; // update to each loop step
		using namespace rigibra;
		Attitude const attDel(PhysAngle{ -.5, .3, -.2 });
		Attitude att(PhysAngle{ .3, -.2, .5 }); // update to each loop
		std::map<SenKey, SenOri> senKeyOris;
		std::map<KeyPair, RelOri> expRos;
		for (std::size_t nn{0u} ; nn < 4u ; ++nn)
		{
			loc = loc + locDel;
			att = attDel * att;
			SenOri const nextOri{ loc, att };;
			SenKey const nextKey{ keyFrom(nn) };;
			senKeyOris.emplace_hint
				(senKeyOris.end(), std::make_pair(nextKey, nextOri));

			// compute expected relative orientations wrt prior orientations
			for (std::map<SenKey, SenOri>::value_type
				const & senKeyOri : senKeyOris)
			{
				SenKey const & pastKey = senKeyOri.first;
				if (pastKey < nextKey) // skip self
				{
					SenOri const & ori1 = senKeyOri.second;
					SenOri const & ori2 = nextOri;
					RelOri const ro2w1{ ori2 * inverse(ori1) };
					expRos.emplace_hint
						( expRos.end()
						, std::make_pair(KeyPair{ pastKey, nextKey }, ro2w1)
						);
				}
			}
		}

		// [DoxyExample01]

		// get collection of relative orientations
		std::map<KeyPair, RelOri> const gotKeyRos
			{ om::relativeOrientationBetweens(senKeyOris) };

		// [DoxyExample01]

		// check each got relative orientation against expected RO
		for (std::map<KeyPair, RelOri>::value_type
			const & gotKeyRo : gotKeyRos)
		{
			KeyPair const & gotKeyPair = gotKeyRo.first;
			RelOri const & gotRo = gotKeyRo.second;

			// find corresponding expected one 
			std::map<KeyPair, RelOri>::const_iterator
				const itFind(expRos.find(gotKeyPair));
			if (gotKeyRos.end() != itFind)
			{
				std::cout << "Found keyPair: " << itFind->first << '\n';
				RelOri const & expRo = itFind->second;

				if (! nearlyEquals(gotRo, expRo))
				{
					oss << "Failure of RO compare test\n";
					oss << "exp: " << expRo << '\n';
					oss << "got: " << gotRo << '\n';
				}
			}
			else
			{
				oss << "Failure to find expected RO test\n";
				oss << "seeking key: " << gotKeyRo.first << '\n';
			}
		}
	}

}

//! Check behavior of Orientation operations
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);
	testRelOrientations(oss);

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

