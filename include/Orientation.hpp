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
\brief Types and functions for handling rigid body orientation data.

Example:
\snippet test_Orientation.cpp DoxyExample01

*/


#include "Key.hpp"
#include "ParmGroup.hpp"
#include "Convention.hpp"

#include <Rigibra>

#include <map>


namespace om
{
	//! Shorthand for classic 3D rigid body orientation transform.
	using SenOri = rigibra::Transform;

	//! Orientations from collection of ParmGroup using useConvention.
	inline
	std::map<om::SenKey, om::SenOri>
	keyOrisFor
		( std::map<om::SenKey, om::ParmGroup> const & keyPGs
		, om::Convention const & useConvention
		)
	{
		using namespace om;
		std::map<SenKey, SenOri> keyOris;
		for (std::map<SenKey, ParmGroup>::value_type const & keyPG : keyPGs)
		{
			SenKey const & senKey = keyPG.first;
			ParmGroup const & pg = keyPG.second;
			keyOris[senKey] = useConvention.transformFor(pg);
		}
		return keyOris;
	}

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
	inline
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
				SenOri const ro2w1{ ori2wR * oriRw1 };

				ros.emplace_hint(ros.end(), std::make_pair(keyPair, ro2w1));
			}
		}
		return ros;
	}


} // [om]

#endif // OriMania_Orientation_INCL_
