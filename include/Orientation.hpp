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


#ifndef OriMania_Orientation_INCL_
#define OriMania_Orientation_INCL_

/*! \file
\brief Contains ######

Example:
\snippet test_Orientation.cpp DoxyExample01

*/


#include <Rigibra>

#include <map>
#include <string>


namespace om
{
	//! Assume individual sensors are identified by arbitrary string values.
	using SenKey = std::string;

	//! encode numeric value into sensor key
	inline
	std::string
	keyFrom
		( std::size_t const & num
		)
	{
		std::ostringstream oss;
		oss << "Key_" << num;
		return oss.str();
	}

	//! Pair of SenKey representing two members in pairwise relationship.
	struct KeyPair
	{
		SenKey theKeyFrom;
		SenKey theKeyInto;

		//! Same as from() to emphasize first in order for 2w1 notation
		inline SenKey const & key1() const { return theKeyFrom; }

		//! Same as into() to emphasize second in order for 2w1 notation
		inline SenKey const & key2() const { return theKeyInto; }

		//! Shorthand name access
		inline
		SenKey const &
		from
			() const
		{
			return theKeyFrom;
		}

		//! Shorthand name access
		inline
		SenKey const &
		into
			() const
		{
			return theKeyInto;
		}

		//! Descriptive information about this instance
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << " ";
			}
			oss
				<< "(Into): " << into()
				<< "  "
				<< "Wrt(From): " << from()
				;
			return oss.str();
		}

	}; // KeyPair

	//! Lexicographic comparision of keys
	inline
	bool
	operator<
		( KeyPair const & pairA
		, KeyPair const & pairB
		)
	{
		// use logic from std::pair
		std::pair const sPairA{ pairA.theKeyFrom, pairA.theKeyInto };
		std::pair const sPairB{ pairB.theKeyFrom, pairB.theKeyInto };
		return (sPairA < sPairB);
	}


	//! Shorthand for classic 3D rigid body orientation transform.
	using SenOri = rigibra::Transform;

	/*! \brief Generate all (non trivial) combinations of relative orientation.
	 *
	 * Generates relative orientations for all combinations of KeyPair
	 * for which:
	 * \arg KeyPair.from() < KeyPair.into()
	 *
	 * Relative Orientation (RO) is defined for input orientations
	 * ori1wX and ori2wX as:
	 * \arg Using input orientations for which (key1 < key2)
	 * \arg oriXw1 = inverse(ori1wX)
	 * \arg Ro2w1 = ori2wX * oriXw1
	 */
	std::map<KeyPair, SenOri>
	relativeOrientationBetweens
		( std::map<SenKey, SenOri> const & keyOris
		)
	{
		std::map<KeyPair, SenOri> ros;

		for (std::map<SenKey, SenOri>::const_iterator
			it1{keyOris.begin()} ; keyOris.end() != it1 ; ++it1)
		{
			std::map<SenKey, SenOri>::const_iterator it2{it1};
			++it2;
			for ( ; keyOris.end() != it2 ; ++it2)
			{
				SenKey const & key1 = it1->first;
				SenKey const & key2 = it2->first;

				SenOri const & ori1wR = it1->second;
				SenOri const & ori2wR = it2->second;

				KeyPair const keyPair{ key1, key2 };
				SenOri const oriRw1{ inverse(ori1wR) };
				SenOri const ori2w1{ ori2wR * oriRw1 };

/*
std::cout
	<< "key(1,2): " << keyPair.key1() << ", " << keyPair.key2()
	<< "  "
	<< "ori2w1: " << ori2w1
	<< '\n';
*/

				ros.emplace_hint(ros.end(), std::make_pair(keyPair, ori2w1));
			}
		}
		return ros;
	}


} // [om]

namespace
{

	//! Put KeyPair.infoString() to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, om::KeyPair const & keyPair
		)
	{
		ostrm << keyPair.infoString();
		return ostrm;
	}

} // [anon]

#endif // OriMania_Orientation_INCL_
