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
\brief Implementation code for OriMania Convention.
*/


#include "Convention.hpp"


namespace
{

//
// Numeric encodings
//

	//! Convert transform order to numeric values (e.g. for sorting) [0,1]
	inline
	std::size_t
	numberFor
		( om::OrderTR const & order
		)
	{
		return static_cast<std::size_t>(order);
	}

	//! Convert sign collection to numeric values (e.g. for sorting) [0,7]
	inline
	std::size_t
	numberFor
		( om::ThreeSigns const & signs
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
		( om::ThreeIndices const & indices
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
		( om::OrderTR const & order
		)
	{
		std::ostringstream oss;
		using namespace om;
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
		( om::ThreeSigns const & signs
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
		( om::ThreeIndices const & indices
		)
	{
		std::ostringstream oss;
		for (std::size_t nn{0u} ; nn < 3u ; ++nn)
		{
			oss << ' ' << +indices[nn];
		}
		return oss.str();
	}


} // [anon]


namespace om
{

namespace priv
{

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

} // [priv]



//
//==========================================================================
// ConventionString encoding support
//==========================================================================
//

std::string
stringFrom
	( om::ThreeSigns const & signInts
	)
{
	std::ostringstream oss;
	//	using ThreeSigns = std::array<std::int8_t, 3u>;
	oss
		<< priv::pmCharFor(signInts[0])
		<< priv::pmCharFor(signInts[1])
		<< priv::pmCharFor(signInts[2])
		;
	return oss.str();
}

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

std::string
stringFrom
	( OrderTR const & order
	)
{
	std::ostringstream oss;
	oss << static_cast<int>(order);
	return oss.str();
}

ThreeSigns
threeSignsFrom
	( std::string const & str
	)
{
	ThreeSigns signs{ -128, -128, -128 };
	if (3u == str.size())
	{
		signs[0] = priv::signFrom(str[0]);
		signs[1] = priv::signFrom(str[1]);
		signs[2] = priv::signFrom(str[2]);
	}
	return signs;
}

ThreeIndices
threeIndicesFrom
	( std::string const & str
	)
{
	ThreeIndices ndxs{ 255u, 255u, 255u };
	if (3u == str.size())
	{
		ndxs[0] = priv::indexFrom(str[0]);
		ndxs[1] = priv::indexFrom(str[1]);
		ndxs[2] = priv::indexFrom(str[2]);
	}
	return ndxs;
}

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

std::array<OrderTR, 2u>
allOrderTRs
	()
{
	return
		{ TranRot
		, RotTran
		};
}


//
//==========================================================================
// Convention
//==========================================================================
//

std::size_t
Convention :: asNumber
	() const
{
	return
		( 1000000000000u
		+   10000000000u * numberFor(theConvOff.theOffSigns) // 8
		+     100000000u * numberFor(theConvOff.theOffIndices) // <32
		+       1000000u * numberFor(theConvAng.theAngSigns) // 8
		+         10000u * numberFor(theConvAng.theAngIndices) // <32
		+           100u * numberFor(theConvAng.theBivIndices) // <32
		+             1u * numberFor(theOrder) // 2
		);
}

/*
// static
std::array<ThreeSigns, 8u>
Convention :: allThreeSigns
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

// static
std::array<ThreeIndices, 6u>
Convention :: allThreeIndices
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

// static
std::array<ThreeIndices, 12u>
Convention :: allBivIndices
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

// static
std::array<OrderTR, 2u>
Convention :: allOrderTRs
	()
{
	return
		{ TranRot
		, RotTran
		};
}
*/

// static
std::vector<Convention>
Convention :: allConventions
	()
{
	std::vector<Convention> conventions;
	conventions.reserve(55296);

	// all combinations of each characteristic
	std::array<ThreeSigns, 8u> const attSigns{ allThreeSigns() };
	std::array<ThreeIndices, 6u> const attNdxs{ allThreeIndices() };
	std::array<ThreeSigns, 8u> const locSigns{ allThreeSigns() };
	std::array<ThreeIndices, 6u> const locNdxs{ allThreeIndices() };
	std::array<ThreeIndices, 12u> const bivNdxs{ allBivIndices() };
	std::array<OrderTR, 2u> const orders{ allOrderTRs() };

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

rigibra::Attitude
Convention :: attitudeFor
	( ParmGroup const & parmGroup
	) const
{
	std::array<double, 3u> const & aVals = parmGroup.theAngles;

	// gather angle sizes together
	ThreeAngles const angleSizes
		{ theConvAng.theAngSigns[0] * aVals[theConvAng.theAngIndices[0]]
		, theConvAng.theAngSigns[1] * aVals[theConvAng.theAngIndices[1]]
		, theConvAng.theAngSigns[2] * aVals[theConvAng.theAngIndices[2]]
		};

	// fixed set of cardinal planes (direction carried by angle sign)
	using namespace engabra::g3;
	static ThreePlanes
		const & eVals{ e23, e31, e12 };

	// gather angle directions together
	ThreePlanes const angleDirs
		{ eVals[theConvAng.theBivIndices[0]]
		, eVals[theConvAng.theBivIndices[1]]
		, eVals[theConvAng.theBivIndices[2]]
		};

	return attitudeFrom3AngleSequence(angleSizes, angleDirs);
}


rigibra::Transform
Convention :: transformFor
	( ParmGroup const & parmGroup
	) const
{
	std::array<double, 3u> const & dVals = parmGroup.theDistances;

	using namespace engabra::g3;

	// gather signed distance values together
	ThreeDistances const offset
		{ theConvOff.theOffSigns[0] * dVals[theConvOff.theOffIndices[0]]
		, theConvOff.theOffSigns[1] * dVals[theConvOff.theOffIndices[1]]
		, theConvOff.theOffSigns[2] * dVals[theConvOff.theOffIndices[2]]
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

std::string
Convention :: infoString
	( std::string const & title
	) const
{
	std::ostringstream oss;
	if (! title.empty())
	{
		oss << title << ' ';
	}
	oss
		<< "  Ang+/-: " << infoStringSigns(theConvAng.theAngSigns)
		<< "  AngNdx: " << infoStringIndices(theConvAng.theAngIndices)
		<< "  Off+/-: " << infoStringSigns(theConvOff.theOffSigns)
		<< "  OffNdx: " << infoStringIndices(theConvOff.theOffIndices)
		<< "  BivNdx: " << infoStringIndices(theConvAng.theBivIndices)
		<< "   Order: " << infoStringOrders(theOrder)
		<< "  Number: " << asNumber()
		;
	return oss.str();
}

//
//==========================================================================
// ConventionString
//==========================================================================
//

// static
ConventionString
ConventionString :: from
	( Convention const & convention
	)
{
	std::string const strOffSigns
		{ stringFrom(convention.theConvAng.theAngSigns) };
	std::string const strOffNdxs
		{ stringFrom(convention.theConvAng.theAngIndices) };
	std::string const strAngSigns
		{ stringFrom(convention.theConvOff.theOffSigns) };
	std::string const strAngNdxs
		{ stringFrom(convention.theConvOff.theOffIndices) };
	std::string const strBivNdxs
		{ stringFrom(convention.theConvAng.theBivIndices) };
	std::string const strOrder
		{ stringFrom(convention.theOrder) };
	return ConventionString
		{ strOffSigns
		, strOffNdxs
		, strAngSigns
		, strAngNdxs
		, strBivNdxs
		, strOrder
		};
}

// static
ConventionString
ConventionString :: from
	( std::string const & encoding
	)
{
	std::istringstream iss(encoding);
	ConventionString cs;
	iss
		>> cs.theStrOffSigns >> cs.theStrOffNdxs
		>> cs.theStrAngSigns >> cs.theStrAngNdxs
		>> cs.theStrBivNdxs
		>> cs.theStrOrder
		;
	return cs;
}

std::string
ConventionString :: stringEncoding
	(
	) const
{
	std::ostringstream oss;
	oss
		<< theStrOffSigns
		<< ' ' << theStrOffNdxs
		<< ' ' << theStrAngSigns
		<< ' ' << theStrAngNdxs
		<< ' ' << theStrBivNdxs
		<< ' ' << theStrOrder
		;
	return oss.str();
}

bool
ConventionString :: isValid
	() const
{
	// quick check on length - could/should inspect contents as well
	return
		(  (3u == theStrOffSigns.size())
		&& (3u == theStrOffNdxs.size())
		&& (3u == theStrAngSigns.size())
		&& (3u == theStrAngNdxs.size())
		&& (3u == theStrBivNdxs.size())
		&& (1u == theStrOrder.size())
		);
}

Convention
ConventionString :: convention
	() const
{
	ThreeSigns const locSigns{ threeSignsFrom(theStrOffSigns) };
	ThreeIndices const locNdxs{ threeIndicesFrom(theStrOffNdxs) };
	ThreeSigns const angSigns{ threeSignsFrom(theStrAngSigns) };
	ThreeIndices const angNdxs{ threeIndicesFrom(theStrAngNdxs) };
	ThreeIndices const bivNdxs{ threeIndicesFrom(theStrBivNdxs) };
	OrderTR const order{ orderTRFrom(theStrOrder) };
	return Convention
		{ locSigns, locNdxs, angSigns, angNdxs, bivNdxs, order };
}


} // [om]

