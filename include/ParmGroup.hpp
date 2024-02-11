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


#ifndef OriMania_ParmGroup_INCL_
#define OriMania_ParmGroup_INCL_

/*! \file
\brief Contains ######

Example:
\snippet test_ParmGroup.cpp DoxyExample01

*/


#include <Engabra>
#include <Rigibra>

#include <array>
#include <cstdint>
#include <sstream>
#include <string>


namespace om
{

//
// Supporting Types
//

	//! Transformation convention: translate then rotate or v.v.
	enum OrderTR
	{
		  TranRot // both expressed in domain
		, RotTran // rotation expressed in domain, translation in range
		, Unknown // not specified
	};

	//! Alias for tracking values of three +/- signs
	using ThreeSigns = std::array<std::int8_t, 3u>;

	//! Alias for tracking permutation of three (small) index values
	using ThreeIndices = std::array<std::uint8_t, 3u>;

	//! Alias for tracking two different transformation orders
	using TwoOrders = std::array<OrderTR, 2u>;

	//! Alias for three distinct offset values (with unknown order and sign)
	using ThreeDistances = std::array<double, 3u>;

	//! Alias for three distinct angle values (with unknown order and sign)
	using ThreeAngles = std::array<double, 3u>;

	//! Alias for three distinct planes (e.g. basis for sequential rotation)
	using ThreePlanes = std::array<engabra::g3::BiVector, 3u>;

	//! String of +/- characters for signed integer values
	std::string
	stringFrom
		( ThreeSigns const & signInts
		);

	//! String of [012] characters for unsigned integer values
	std::string
	stringFrom
		( ThreeIndices const & ndxInts
		);

	//! String of [0...] characters for enum OrderTR type.
	std::string
	stringFrom
		( OrderTR const & order
		);

	//! Convert string to three numeric index values
	ThreeSigns
	threeSignsFrom
		( std::string const & str
		);

	//! Convert string to three numeric index values
	ThreeIndices
	threeIndicesFrom
		( std::string const & str
		);

	//! Decode string character [01] to [TR,RT]
	OrderTR
	orderTRFrom
		( std::string const & str
		);

	//! All combinations of signs for three elements
	std::array<ThreeSigns, 8u>
	allThreeSigns
		();

	//! All 6 combinations of unique indices for three element array.
	std::array<ThreeIndices, 6u>
	allThreeIndices
		();

	//! All 12 combinations of unique bivector rotation indices.
	std::array<ThreeIndices, 12u>
	allBivIndices
		();

	//! All transformation translate/rotate conventions
	std::array<OrderTR, 2u>
	allOrderTRs
		();

//
// Info/formatting
//

	//! String representation of three signs
	std::string
	infoStringOrders
		( om::OrderTR const & order
		);

	//! String representation of three signs
	std::string
	infoStringSigns
		( om::ThreeSigns const & signs
		);

	//! String representation of three indices
	std::string
	infoStringIndices
		( om::ThreeIndices const & indices
		);

//
// ParmGroup
//

	//! Grouping of parameters by angle and distance values
	struct ParmGroup
	{
		//! Numeric distace values (meters) for which order/sign are unknown
		ThreeDistances theDistances
			{ engabra::g3::nan
			, engabra::g3::nan
			, engabra::g3::nan
			};

		//! Numeric angle values (radians) for which order/sign are unknown
		ThreeAngles theAngles
			{ engabra::g3::nan
			, engabra::g3::nan
			, engabra::g3::nan
			};

		//! True if this instance contains plausible data values.
		bool
		isValid
			() const;

		//! Descriptive information about this instance
		std::string
		infoString
			( std::string const & title = {}
			) const;

	}; // ParmGroup

} // [om]


#endif // OriMania_ParmGroup_INCL_
