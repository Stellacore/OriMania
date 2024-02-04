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
\brief Contains ######

Example:
\snippet test_Convention.cpp DoxyExample01

*/


#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>


namespace om
{

	//! Transformation convention: translate then rotate or v.v.
	enum OrderTR
	{
		  TranRot // both expressed in domain
		, RotTran // rotation expressed in domain, translation in range
	};

	//! Convert transform order to numeric values (e.g. for sorting)
	inline
	std::size_t
	numberFor
		( OrderTR const & order
		)
	{
		return static_cast<std::size_t>(order);
	}

	//! String representation of three signs
	inline
	std::string
	orderOf
		( OrderTR const & order
		)
	{
		std::ostringstream oss;
		oss << +order;
		/*
		switch (order)
		{
			case TranRot:
				oss << "TR";
				break;
			case RotTran:
				oss << "RT";
				break;
		}
		*/
		return oss.str();
	}


	using ThreeSigns = std::array<std::int8_t, 3u>;
	using ThreeIndices = std::array<std::uint8_t, 3u>;
	using TwoOrders = std::array<OrderTR, 2u>;

	//! Convert sign collection to numeric values (e.g. for sorting)
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

	//! Convert index collection to numeric values (e.g. for sorting)
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

	//! String representation of three signs
	inline
	std::string
	signsOf
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
	indicesOf
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

	//! Candidate convention associated with 6 orientation values
	struct Convention
	{
		// ---, --+, -+-, -++, +--, +-+, ++-, +++
		ThreeSigns theAngSigns;
		// 012, 021, 120, 102, 201, 210
		ThreeIndices theAngIndices;
		// ---, --+, -+-, -++, +--, +-+, ++-, +++
		ThreeSigns theLocSigns;
		// 012, 021, 120, 102, 201, 210
		ThreeIndices theLocIndices;
		/*
		 * ---, 001, 002  |  ---, ..., ...
		 * 010, 011, 012  |  010, ..., 012
		 * 020, 021, 022  |  020, 021, ...
		 * 100, 101, 102  |  ..., 101, 102
		 * 110, ---, 112  |  ..., ---, ...
		 * 120, 121, 122  |  120, 121, ...
		 * 200, 201, 202  |  ..., 201, 202
		 * 210, 211, 212  |  210, ..., 212
		 * 220, 221, ---  |  ..., ..., ---
		 */
		ThreeIndices theBivIndices;
		// TranRot, RotTran
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

		//! Descriptive information about this instance
		inline
		std::string
		infoString
			( std::string const & title
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "  Ang+/-: " << signsOf(theAngSigns)
				<< "  AngNdx: " << indicesOf(theAngIndices)
				<< "  Loc+/-: " << signsOf(theLocSigns)
				<< "  LocNdx: " << indicesOf(theLocIndices)
				<< "  BivNdx: " << indicesOf(theBivIndices)
				<< "  Order: " << orderOf(theOrder)
				<< "  Number: " << asNumber()
				;
			return oss.str();
		}

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

	}; // Convention

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
