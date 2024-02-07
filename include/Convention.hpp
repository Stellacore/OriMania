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
// Numeric encodings
//

	//! Convert transform order to numeric values (e.g. for sorting) [0,1]
	inline
	std::size_t
	numberFor
		( OrderTR const & order
		)
	{
		return static_cast<std::size_t>(order);
	}

	//! Convert sign collection to numeric values (e.g. for sorting) [0,7]
	inline
	std::size_t
	numberFor
		( ThreeSigns const & signs
		)
	{
		return
			( 4u * (static_cast<std::size_t>(1u + signs[0]) / 2u)
			+ 2u * (static_cast<std::size_t>(1u + signs[1]) / 2u)
			+ 1u * (static_cast<std::size_t>(1u + signs[2]) / 2u)
			);
	}

	//! Convert index collection to numeric values (e.g. for sorting) [0,26]
	inline
	std::size_t
	numberFor
		( ThreeIndices const & indices
		)
	{
		return
			( 9u * static_cast<std::size_t>(indices[0])
			+ 3u * static_cast<std::size_t>(indices[1])
			+ 1u * static_cast<std::size_t>(indices[2])
			);
	}

//
// Info/formatting
//

	//! String representation of three signs
	inline
	std::string
	infoStringOrders
		( OrderTR const & order
		)
	{
		std::ostringstream oss;
		switch (order)
		{
			case TranRot:
				oss << "TR";
				break;
			case RotTran:
				oss << "RT";
				break;
			case Unknown:
			default:
				oss << "??";
				break;
		}
		return oss.str();
	}

	//! String representation of three signs
	inline
	std::string
	infoStringSigns
		( ThreeSigns const & signs
		)
	{
		std::ostringstream oss;
		for (std::size_t nn{0u} ; nn < 3u ; ++nn)
		{
			oss << ' ' << std::setw(2u) << +signs[nn];
		}
		return oss.str();
	}

	//! String representation of three indices
	inline
	std::string
	infoStringIndices
		( ThreeIndices const & indices
		)
	{
		std::ostringstream oss;
		for (std::size_t nn{0u} ; nn < 3u ; ++nn)
		{
			oss << ' ' << +indices[nn];
		}
		return oss.str();
	}

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
		inline
		std::size_t
		asNumber
			() const
		{
			return
				( 1000000000000u
				+   10000000000u * numberFor(theAngSigns) // 8
				+     100000000u * numberFor(theAngIndices) // <32
				+       1000000u * numberFor(theLocSigns) // 8
				+         10000u * numberFor(theLocIndices) // <32
				+           100u * numberFor(theBivIndices) // <32
				+             1u * numberFor(theOrder) // 2
				);
		}

		/*TODO
		//! Create Convention from number - inverse of asNumber() method.
		inline
		static
		Convention
		fromNumber
			( std::size_t const & number
			)
		{
			return {}; // TODO - needs supporting decoding functions
		}
		*/

		//! All combinations of signs for three elements
		inline
		static
		std::array<ThreeSigns, 8u>
		allThreeSigns
			()
		{
			return
				{ ThreeSigns{ -1, -1, -1 }
				, ThreeSigns{ -1, -1,  1 }
				, ThreeSigns{ -1,  1, -1 }
				, ThreeSigns{ -1,  1,  1 }
				, ThreeSigns{  1, -1, -1 }
				, ThreeSigns{  1, -1,  1 }
				, ThreeSigns{  1,  1, -1 }
				, ThreeSigns{  1,  1,  1 }
				};
		}

		//! All combinations of unique indices for three element array
		inline
		static
		std::array<ThreeIndices, 6u>
		allThreeIndices
			()
		{
			return
				{ ThreeIndices{ 0u, 1u, 2u }
				, ThreeIndices{ 0u, 2u, 1u }
				, ThreeIndices{ 1u, 0u, 2u }
				, ThreeIndices{ 1u, 2u, 0u }
				, ThreeIndices{ 2u, 1u, 0u }
				, ThreeIndices{ 2u, 0u, 1u }
				};
		}

		//! All combinations of unique indices for three element array
		inline
		static
		std::array<ThreeIndices, 12u>
		allBivIndices
			()
		{
			return
				{ ThreeIndices{ 0, 1, 0 }
				, ThreeIndices{ 0, 1, 2 }
				, ThreeIndices{ 0, 2, 0 }
				, ThreeIndices{ 0, 2, 1 }
				, ThreeIndices{ 1, 0, 1 }
				, ThreeIndices{ 1, 0, 2 }
				, ThreeIndices{ 1, 2, 0 }
				, ThreeIndices{ 1, 2, 1 }
				, ThreeIndices{ 2, 0, 1 }
				, ThreeIndices{ 2, 0, 2 }
				, ThreeIndices{ 2, 1, 0 }
				, ThreeIndices{ 2, 1, 2 }
				};
		}

		//! All transformation translate/rotate conventions
		inline
		static
		std::array<OrderTR, 2u>
		allOrderTRs
			()
		{
			return
				{ TranRot
				, RotTran
				};
		}

		//! Collection of unique conventions that are supported overall
		inline
		static
		std::vector<Convention>
		allConventions
			()
		{
			std::vector<Convention> conventions;
			conventions.reserve(55296);

			// all combinations of each characteristic
			std::array<ThreeSigns, 8u> const attSigns
				{ Convention::allThreeSigns() };
			std::array<ThreeIndices, 6u> const attNdxs
				{ Convention::allThreeIndices() };
			std::array<ThreeSigns, 8u> const locSigns
				{ Convention::allThreeSigns() };
			std::array<ThreeIndices, 6u> const locNdxs
				{ Convention::allThreeIndices() };
			std::array<ThreeIndices, 12u> const bivNdxs
				{ Convention::allBivIndices() };
			std::array<OrderTR, 2u> const orders
				{ Convention::allOrderTRs() };

			// brute force generation of all possible combinations
			for (ThreeSigns const & attSign : attSigns)
			{
				for (ThreeIndices const & attNdx : attNdxs)
				{
					for (ThreeSigns const & locSign : locSigns)
					{
						for (ThreeIndices const & locNdx : locNdxs)
						{
							for (ThreeIndices const & bivNdx : bivNdxs)
							{
								for (OrderTR const & order : orders)
								{
									Convention const convention
										{ attSign
										, attNdx
										, locSign
										, locNdx
										, bivNdx
										, order
										};
									conventions.emplace_back(convention);
								}
							}
						}
					}
				}
			}
			return conventions;
		}

		//! Attitude associated with parmGroup given this convention.
		inline
		rigibra::Attitude
		attitudeFor
			( ParmGroup const & parmGroup
			) const
		{
			std::array<double, 3u> const & aVals = parmGroup.theAngles;

			// gather angle sizes together
			ThreeAngles const angleSizes
				{ theAngSigns[0] * aVals[theAngIndices[0]]
				, theAngSigns[1] * aVals[theAngIndices[1]]
				, theAngSigns[2] * aVals[theAngIndices[2]]
				};

			// fixed set of cardinal planes (direction carried by angle sign)
			using namespace engabra::g3;
			static ThreePlanes
				const & eVals{ e23, e31, e12 };

			// gather angle directions together
			ThreePlanes const angleDirs
				{ eVals[theBivIndices[0]]
				, eVals[theBivIndices[1]]
				, eVals[theBivIndices[2]]
				};

			return attitudeFrom3AngleSequence(angleSizes, angleDirs);
		}


		//! Transform with ParmGroup values consistent with this convention.
		inline
		rigibra::Transform
		transformFor
			( ParmGroup const & parmGroup
			) const
		{
			std::array<double, 3u> const & dVals = parmGroup.theDistances;

			using namespace engabra::g3;

			// gather signed distance values together
			ThreeDistances const offset
				{ theLocSigns[0] * dVals[theLocIndices[0]]
				, theLocSigns[1] * dVals[theLocIndices[1]]
				, theLocSigns[2] * dVals[theLocIndices[2]]
				};

			// determine attitude associated with parmGroup
			rigibra::Attitude const attR(attitudeFor(parmGroup));

			// to compute translation
			// first, assume TranRot convention...
			Vector tVec{ offset };
			// ... unless inverse convention is needed
			if (RotTran == theOrder)
			{
				// compute forward translation from inverse offset convention
				Vector const ty{ offset };
				tVec = attR(ty);
			}
			return rigibra::Transform{ tVec, attR };
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
				oss << title << ' ';
			}
			oss
				<< "  Ang+/-: " << infoStringSigns(theAngSigns)
				<< "  AngNdx: " << infoStringIndices(theAngIndices)
				<< "  Loc+/-: " << infoStringSigns(theLocSigns)
				<< "  LocNdx: " << infoStringIndices(theLocIndices)
				<< "  BivNdx: " << infoStringIndices(theBivIndices)
				<< "   Order: " << infoStringOrders(theOrder)
				<< "  Number: " << asNumber()
				;
			return oss.str();
		}

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

		//! Convert string characters [-,+] into {-1.,+1.}
		inline
		static
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
		static
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
		static
		ThreeSigns
		threeSigns
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
		static
		ThreeIndices
		threeIndices
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
		static
		OrderTR
		orderFrom
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

		//! A '+' or '-' character depending on the sign of aByte
		inline
		static
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
		static
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
		static
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
		static
		std::string
		stringFrom
			( OrderTR const & order
			)
		{
			std::ostringstream oss;
			oss << static_cast<int>(order);
			return oss.str();
		}

		//! Construct from canonical encoding.
		inline
		static
		ConventionString
		from
			( Convention const & convention
			)
		{
			std::string const strLocSigns
				{ stringFrom(convention.theAngSigns) };
			std::string const strLocNdxs
				{ stringFrom(convention.theAngIndices) };
			std::string const strAngSigns
				{ stringFrom(convention.theLocSigns) };
			std::string const strAngNdxs
				{ stringFrom(convention.theLocIndices) };
			std::string const strBivNdxs
				{ stringFrom(convention.theBivIndices) };
			std::string const strOrder
				{ stringFrom(convention.theOrder) };
			return ConventionString
				{ strLocSigns
				, strLocNdxs
				, strAngSigns
				, strAngNdxs
				, strBivNdxs
				, strOrder
				};
			/*
			ThreeSigns theAngSigns;
			ThreeIndices theAngIndices;
			ThreeSigns theLocSigns;
			ThreeIndices theLocIndices;
			ThreeIndices theBivIndices;
			OrderTR theOrder;
			*/
		}

		//! Construct from canonical encoding.
		inline
		static
		ConventionString
		from
			( std::string const & encoding
			)
		{
			std::istringstream iss(encoding);
			ConventionString cs;
			iss
				>> cs.theStrLocSigns >> cs.theStrLocNdxs
				>> cs.theStrAngSigns >> cs.theStrAngNdxs
				>> cs.theStrBivNdxs
				>> cs.theStrOrder
				;
			return cs;
		}

		//! Canonical string encoding for a convention
		inline
		std::string
		stringEncoding
			(
			) const
		{
			std::ostringstream oss;
			oss
				<< theStrLocSigns
				<< ' ' << theStrLocNdxs
				<< ' ' << theStrAngSigns
				<< ' ' << theStrAngNdxs
				<< ' ' << theStrBivNdxs
				<< ' ' << theStrOrder
				;
			return oss.str();
		}

		//! True if all strings components are valid
		inline
		bool
		isValid
			() const
		{
			// quick check on length - could/should inspect contents as well
			return
				(  (3u == theStrLocSigns.size())
				&& (3u == theStrLocNdxs.size())
				&& (3u == theStrAngSigns.size())
				&& (3u == theStrAngNdxs.size())
				&& (3u == theStrBivNdxs.size())
				&& (1u == theStrOrder.size())
				);
		}

		//! Convention associated with current string values
		inline
		Convention
		convention
			() const
		{
			ThreeSigns const locSigns{ threeSigns(theStrLocSigns) };
			ThreeIndices const locNdxs{ threeIndices(theStrLocNdxs) };
			ThreeSigns const angSigns{ threeSigns(theStrAngSigns) };
			ThreeIndices const angNdxs{ threeIndices(theStrAngNdxs) };
			ThreeIndices const bivNdxs{ threeIndices(theStrBivNdxs) };
			OrderTR const order{ orderFrom(theStrOrder) };
			return Convention
				{ locSigns, locNdxs, angSigns, angNdxs, bivNdxs, order };
		}

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

