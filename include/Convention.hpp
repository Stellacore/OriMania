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


#ifndef OriMania_Convention_INCL_
#define OriMania_Convention_INCL_

/*! \file
\brief Structures and functions related to orientation parameter conventions.
*/

/*
Example:
\snippet test_Convention.cpp DoxyExample01
*/



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

		//! A '+' or '-' character depending on the sign of aByte
		inline
		std::string::value_type
		pmCharFor
			( int8_t const & aByte
			)
		{
			std::string::value_type aChar{ '+' };
			if (aByte < 0)
			{
				aChar = '-';
			}
			return aChar;
		}

		//! String of +/- characters for signed integer values
		inline
		std::string
		stringFrom
			( ThreeSigns const & signInts
			)
		{
			std::ostringstream oss;
			//	using ThreeSigns = std::array<std::int8_t, 3u>;
			oss
				<< pmCharFor(signInts[0])
				<< pmCharFor(signInts[1])
				<< pmCharFor(signInts[2])
				;
			return oss.str();
		}

		//! String of [012] characters for unsigned integer values
		inline
		std::string
		stringFrom
			( ThreeIndices const & ndxInts
			)
		{
			std::ostringstream oss;
			//	using ThreeIndices = std::array<std::uint8_t, 3u>;
			oss
				<< static_cast<int>(ndxInts[0])
				<< static_cast<int>(ndxInts[1])
				<< static_cast<int>(ndxInts[2])
				;
			return oss.str();
		}

		//! String of [0...] characters for enum OrderTR type.
		inline
		std::string
		stringFrom
			( OrderTR const & order
			)
		{
			std::ostringstream oss;
			oss << static_cast<int>(order);
			return oss.str();
		}

		//! Convert string characters [-,+] into {-1.,+1.}
		inline
		double
		signFrom
			( std::string::value_type const & aChar
			)
		{
			double value{ engabra::g3::null<double>() };
			if ('-' == aChar)
			{
				value = -1.;
			}
			else
			if ('+' == aChar)
			{
				value = 1.;
			}
			return value;
		}

		//! Convert string characters [012] int size_t types
		inline
		std::uint8_t
		indexFrom
			( std::string::value_type const & aChar
			)
		{
			std::uint8_t ndx{ 255u };
			if ('0' == aChar)
			{
				ndx = 0;
			}
			else
			if ('1' == aChar)
			{
				ndx = 1;
			}
			else
			if ('2' == aChar)
			{
				ndx = 2;
			}
			return ndx;
		}

		//! Convert string to three numeric index values
		inline
		ThreeSigns
		threeSignsFrom
			( std::string const & str
			)
		{
			ThreeSigns signs{ -128, -128, -128 };
			if (3u == str.size())
			{
				signs[0] = signFrom(str[0]);
				signs[1] = signFrom(str[1]);
				signs[2] = signFrom(str[2]);
			}
			return signs;
		}

		//! Convert string to three numeric index values
		inline
		ThreeIndices
		threeIndicesFrom
			( std::string const & str
			)
		{
			ThreeIndices ndxs{ 255u, 255u, 255u };
			if (3u == str.size())
			{
				ndxs[0] = indexFrom(str[0]);
				ndxs[1] = indexFrom(str[1]);
				ndxs[2] = indexFrom(str[2]);
			}
			return ndxs;
		}

		//! Decode string character [01] to [TR,RT]
		inline
		OrderTR
		orderTRFrom
			( std::string const & str
			)
		{
			OrderTR order{ Unknown };
			if (1u == str.size())
			{
				if ('0' == str[0])
				{
					order = TranRot;
				}
				else
				if ('1' == str[0])
				{
					order = RotTran;
				}
			}
			return order;
		}


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
				oss << title << ' ';
			}
			using engabra::g3::io::fixed;
			oss
				<< "  Distances: "
					<< fixed(theDistances[0], 1u, 6u)
					<< fixed(theDistances[1], 1u, 6u)
					<< fixed(theDistances[2], 1u, 6u)
				<< "  Angles: "
					<< fixed(theAngles[0], 1u, 9u)
					<< fixed(theAngles[1], 1u, 9u)
					<< fixed(theAngles[2], 1u, 9u)
				;
			return oss.str();
		}


	}; // ParmGroup

//
// Math utilities
//

	/*! \brief Generate rotation as sequence of three rotations.
	 *
	 * The return Attitude is computed as the sequence of rotations
	 * as follows:
	 * \arg spinC = exp(angleSize[0]*angleDir[0])
	 * \arg spinB = exp(angleSize[1]*angleDir[1])
	 * \arg spinA = exp(angleSize[2]*angleDir[2])
	 * \arg spinNet = spinC * spinB * spinA
	 *
	 * The attitude associated with spinNet is returned.
	 */
	inline
	rigibra::Attitude
	attitudeFrom3AngleSequence
		( ThreeAngles const & angleSizes
		, ThreePlanes const & angleDirs
		)
	{
		using namespace rigibra;
		PhysAngle const physAngleA{ angleSizes[0] * angleDirs[0] };
		PhysAngle const physAngleB{ angleSizes[1] * angleDirs[1] };
		PhysAngle const physAngleC{ angleSizes[2] * angleDirs[2] };
		Attitude const attA(physAngleA);
		Attitude const attB(physAngleB);
		Attitude const attC(physAngleC);
		return (attC * attB * attA);
	}

//
// Conventions for transformation parameters
//

	//! Candidate convention associated with 6 orientation values
	struct Convention
	{
		//! \brief Permutations: ---, --+, -+-, -++, +--, +-+, ++-, +++
		ThreeSigns theAngSigns;

		//! \brief Permutations: 012, 021, 120, 102, 201, 210
		ThreeIndices theAngIndices;

		//! \brief Permutations: ---, --+, -+-, -++, +--, +-+, ++-, +++
		ThreeSigns theLocSigns;

		//! \brief Permutations: 012, 021, 120, 102, 201, 210
		ThreeIndices theLocIndices;

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

		//! \brief Permutations: TranRot, RotTran
		OrderTR theOrder;

		//! Assign a number to each convention (for easy tracking))
		std::size_t
		asNumber
			() const;

		/*TODO
		//! Create Convention from number - inverse of asNumber() method.
		inline
		static
		Convention
		fromNumber
			( std::size_t const & number
			)
		*/

		//! All combinations of signs for three elements
		static
		std::array<ThreeSigns, 8u>
		allThreeSigns
			();

		//! All combinations of unique indices for three element array
		static
		std::array<ThreeIndices, 6u>
		allThreeIndices
			();

		//! All combinations of unique indices for three element array
		static
		std::array<ThreeIndices, 12u>
		allBivIndices
			();

		//! All transformation translate/rotate conventions
		static
		std::array<OrderTR, 2u>
		allOrderTRs
			();

		//! Collection of unique conventions that are supported overall
		static
		std::vector<Convention>
		allConventions
			();

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
		std::string theStrLocSigns;
		std::string theStrLocNdxs;
		std::string theStrAngSigns;
		std::string theStrAngNdxs;
		std::string theStrBivNdxs;
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
		return (convA.asNumber() < convB.asNumber());
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

