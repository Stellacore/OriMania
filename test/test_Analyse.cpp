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
\brief Unit tests (and example) code for OriMania::NS::CN
*/


// TODO #include "_.hpp" // template for header files

#include "OriMania.hpp"

#include <Rigibra>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <vector>


namespace
{
	using SenKey = std::string;

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

	using Ori = rigibra::Transform;
	std::map<SenKey, Ori>
	simEOs
		()
	{
		std::map<SenKey, Ori> oris;
		oris.emplace_hint
			( oris.end()
			, std::make_pair(SenKey("fake1"), rigibra::identity<Ori>())
			);
		oris.emplace_hint
			( oris.end()
			, std::make_pair(SenKey("fake2"), rigibra::identity<Ori>())
			);
		oris.emplace_hint
			( oris.end()
			, std::make_pair(SenKey("fake3"), rigibra::identity<Ori>())
			);
		return oris;
	};

	/*! \brief All combinations of (non trivial) relative orientations.
	 *
	 *
	 */
	std::map<std::pair<SenKey, SenKey>, Ori>
	relativeOrientationBetweens
		( std::map<SenKey, Ori> const & senWrtInds
		)
	{
		std::map<std::pair<SenKey, SenKey>, Ori> roInds;
		for (std::pair<SenKey, Ori> const senWrtInd1 : senWrtInds)
		{
			for (std::pair<SenKey, Ori> const senWrtInd2 : senWrtInds)
			{
				std::pair<SenKey, SenKey> const keyPair
					{ senWrtInd1.first, senWrtInd2.first };
std::cout << "keyPair: " << keyPair.first << ", " << keyPair.second << '\n';
				Ori const & ori1wR = senWrtInd1.second;
				Ori const & ori2wR = senWrtInd2.second;
				Ori const & oriRw1{ inverse(ori1wR) };
				Ori const & ori2w1{ ori2wR * oriRw1 };
			}
		}
		return roInds;
	}

	//! Check that number and uniqueness of conventions
	void
	testConventions
		( std::ostream & oss
		)
	{
		constexpr std::size_t expNumConventions{ 55296u };

		// check for small data storage size
		constexpr std::size_t expDataSize{ 3u + 3u + 3u + 3u + 6u + 2u };
		std::size_t const gotDataSize{ sizeof(Convention) };
		if (! (expDataSize == gotDataSize))
		{
			oss << "Failure of per convention data size test\n";
			oss << "exp: " << expDataSize << '\n';
			oss << "got: " << gotDataSize << '\n';
		}

		// generate all combinations of data sets
		std::vector<Convention> const conventions
			{ Convention::allConventions() };

		// check number of conventions supported
		if (! (expNumConventions == conventions.size()))
		{
			oss << "Failure to testConventions count test\n";
			oss << "exp: " << expNumConventions << '\n';
			oss << "got: " << conventions.size() << '\n';
		}

		// check if all are unique
		std::set<Convention> const uniques
			(conventions.cbegin(), conventions.cend());
		if (! (uniques.size() == conventions.size()))
		{
			oss << "Failure of testConventions uniqueness test\n";
			oss << "exp: " << conventions.size() << '\n';
			oss << "got: " << uniques.size() << '\n';
		}
	}

	//! Check convention extraction from simulated data
	void
	testSim
		( std::ostream & oss
		)
	{
		using namespace om;

		//
		// Simulate sensor data
		//   1) Individual ori of each sensor in system 'Box' frame
		//   2) Relative orienation between objects in some arbitrary frame.
		//

		// simulate arbitrary ExCal orientations
		std::vector<rigibra::Transform> const senWrtBoxs{};

		// use the ExCal data to generate Independent EO data
		std::map<SenKey, rigibra::Transform> const senWrtInds{};


		// generate RO pairs
		std::map<std::pair<SenKey, SenKey>, rigibra::Transform>
			const roXforms{ relativeOrientationBetweens(senWrtInds) };

		// consider common conventions
		std::vector<Convention> conventions{};

		// compute BoxRo
			// candidate for Sen1wBox
			// candidate for Sen2wBox
//			BoxRo = Sen2wBox * inverse(Sen1wBox);
		// IndRo
//			IndRo = Sen2wInd * inverse(Sen1wInd);

		// compare and assign metric
//		std::map<CHash, Metric>


		// [DoxyExample01]

		// [DoxyExample01]

		// TODO replace this with real test code
		constexpr bool successful{ false };
		if (! successful)
		{
			oss << "Failure of testSim implementation test\n";
		}
	}

}

//! Check convention recovery with simulated data
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	testConventions(oss);
	testSim(oss);

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
