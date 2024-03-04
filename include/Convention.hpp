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


#ifndef OriMania_Convention_INCL_
#define OriMania_Convention_INCL_

/*! \file
\brief Structures and functions related to orientation parameter conventions.
*/

/*
Example:
\snippet test_Convention.cpp DoxyExample01
*/


#include "ParmGroup.hpp"

#include <Rigibra>

#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>


namespace om
{

//
// Convention for transformation parameters
//

	/*! \brief Conventions for creating offset vector from 3 distance values.
	 *
	 */
	struct ConventionOffset
	{
		//! \brief Permutations: ---, --+, -+-, -++, +--, +-+, ++-, +++
		ThreeSigns theOffSigns;

		//! \brief Permutations: 012, 021, 120, 102, 201, 210
		ThreeIndices theOffIndices;

		//! Collection of unique conventions that are supported overall
		static
		std::vector<ConventionOffset>
		allConventions
			();

		//! Vector offset for parmGroup using this current convention
		rigibra::Location
		offsetFor
			( ParmGroup const & parmGroup
			) const;

		/*! \brief Assign a number to each convention (for easy tracking))
		 *
		 * Number of index values is less than
		 * \arg 216 = 8(signs) * 27(offNdxOrder)
		 *
		 * Actual max index value is 210. (i.e. 211 element storage needed).
		 */
		std::size_t
		indexValue
			() const;

		//! Descriptive information about this instance
		std::string
		infoString
			( std::string const & title = {}
			) const;

	}; // ConventionOffset

	/*! \brief Conventions for 3-angle sequences from 3 angle size values.
	 *
	 */
	struct ConventionAngle
	{
		//! \brief Permutations: ---, --+, -+-, -++, +--, +-+, ++-, +++
		ThreeSigns theAngSigns;

		//! \brief Permutations: 012, 021, 120, 102, 201, 210
		ThreeIndices theAngIndices;

		/*! \brief Permutes: 010,012,020,021, 101,102,120,121, 201,202,210,212.
		 *
		 * \verbatim
		 * ---, 001, 002  |  ---, ..., ...
		 * 010, 011, 012  |  010, ..., 012
		 * 020, 021, 022  |  020, 021, ...
		 * 100, 101, 102  |  ..., 101, 102
		 * 110, ---, 112  |  ..., ---, ...
		 * 120, 121, 122  |  120, 121, ...
		 * 200, 201, 202  |  ..., 201, 202
		 * 210, 211, 212  |  210, ..., 212
		 * 220, 221, ---  |  ..., ..., ---
		 * \endverbatim
		 */
		ThreeIndices theBivIndices;

		//! Collection of unique conventions that are supported overall
		static
		std::vector<ConventionAngle>
		allConventions
			();

		/*! \brief Assign a number to each convention (for easy tracking))
		 *
		 * Number of index values is less than
		 * \arg 5832 = 8(signs) * 27(angNdxOrder) * 27(bivNdxOrder)
		 *
		 * Actual max index value is 5693. (i.e. 5694 element storage needed).
		 */
		std::size_t
		indexValue
			() const;

		//! Attitude associated with parmGroup given this convention.
		rigibra::Attitude
		attitudeFor
			( ParmGroup const & parmGroup
			) const;

		//! Descriptive information about this instance
		std::string
		infoString
			( std::string const & title = {}
			) const;

	}; // ConventionAngle

	//! Candidate convention associated with 6 orientation values
	struct Convention
	{
		//! \brief Conventions for interpreting 3 offset distances
		ConventionOffset theConvOff{};

		//! \brief Conventions for interpreting 3 angle sizes
		ConventionAngle theConvAng{};

		//! \brief Permutations: TranRot, RotTran
		OrderTR theOrder{ Unknown };

		//! Collection of unique conventions that are supported overall
		static
		std::vector<Convention>
		allConventionsFor
			( ConventionOffset const & offConv
			);

		//! Collection of unique conventions that are supported overall
		static
		std::vector<Convention>
		allConventions
			();

		//! Construct an instance from numeric encoding.
		static
		Convention
		fromNumberEncoding
			( std::int64_t const & numId
			);

		//! Assign a number to each convention (for easy tracking))
		std::int64_t
		numberEncoding
			() const;

		//! True if this instance has valid data (uses theOrder as flag).
		bool
		isValid
			() const;

		//! Offset vector (expressed in OrderTR domain - invert if OrderRT)
		rigibra::Location
		offsetFor
			( ParmGroup const & parmGroup
			) const;

		//! Attitude associated with parmGroup given this convention.
		rigibra::Attitude
		attitudeFor
			( ParmGroup const & parmGroup
			) const;

		//! Transform with ParmGroup values consistent with this convention.
		rigibra::Transform
		transformFor
			( ParmGroup const & parmGroup
			) const;

		//! Descriptive information about this instance
		std::string
		infoString
			( std::string const & title = {}
			) const;

	}; // Convention

//
// ConventionString en/de-coder
//

	/*! \brief Represent Convention as string with to/from-string ablities.
	 *
	 * The various individual conventions of Convention class members
	 * are represented by strings. E.g. strings of '+' and '-' characters
	 * for sign conventions, and strings of digits [0,1,2] for index
	 * conventions. The enumeration OrderTR is represented as the
	 * enumeration item value (integer cast of enum type).
	 */
	struct ConventionString
	{
		//! Three offset vector sign conventions - e.g. "---", "++-", etc.
		std::string theStrOffSigns;
		//! Three offset vector indices {0,1,2} - e.g. "012", "201", etc
		std::string theStrOffNdxs;
		//! Three angle size sign conventions - e.g. "---", "++-", etc.
		std::string theStrAngSigns;
		//! Three angle size indices {0,1,2} - e.g. "012", "201", etc
		std::string theStrAngNdxs;
		//! Three angle direction indices {0,1,2} - e.g. "012", "201", etc
		std::string theStrBivNdxs;
		//! Integer value representing OrderTR enum item - e.g. "0", "1"
		std::string theStrOrder;

		//! Construct from canonical encoding.
		static
		ConventionString
		from
			( Convention const & convention
			);

		//! Construct from canonical encoding.
		static
		ConventionString
		from
			( std::string const & encoding
			);

		//! Canonical string encoding for a convention
		std::string
		stringEncoding
			(
			) const;

		//! True if all strings components are valid
		bool
		isValid
			() const;

		//! Offset convention associated with current string
		ConventionOffset
		conventionOffset
			() const;

		//! Attitude convention associated with current string
		ConventionAngle
		conventionAngle
			() const;

		//! Convention associated with current string values
		Convention
		convention
			() const;

	}; // ConventionString

//
// Comparision operators
//

	//! True if all corresponding items of A are less than those of B
	inline
	bool
	operator<
		( Convention const & convA
		, Convention const & convB
		)
	{
		return (convA.numberEncoding() < convB.numberEncoding());
	}

	//! True if ((!(A<B)) && (!(B<A)))
	inline
	bool
	operator==
		( Convention const & convA
		, Convention const & convB
		)
	{
		return
			(  (! (convA < convB))
			&& (! (convB < convA))
			);
	}

} // [om]


#endif // OriMania_Convention_INCL_

