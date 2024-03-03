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

#include <iomanip>


namespace
{

namespace num
{
	/*
	For Convention numeric ID encoding,

		Att: +++ 012  (6 char)
		Loc: +++ 012  (6 char)
		Biv: 012      (3 char)
		oTR: 0        (1 char )
					  === 16 char
		(64-bit signed number has max value of about 18 decimal digits)

		Bits
		1,1,1,2,2,2 = 9
		1,1,1,2,2,2 = 9
		2,2,2       = 6
		1           = 1
					=== 27
		(more compactly)
		3,5 = 8
		3,5 = 8
		5   = 6
		2   = 2
		  === 24
	*/

	constexpr std::int64_t sIdBase{ 100 }; // for easy human interpretation
	constexpr std::int64_t sPad   { 1000000000000 };
	constexpr std::int64_t sOffSgn{   10000000000 };
	constexpr std::int64_t sOffNdx{     100000000 };
	constexpr std::int64_t sAngSgn{       1000000 };
	constexpr std::int64_t sAngNdx{         10000 };
	constexpr std::int64_t sBivNdx{           100 };
	constexpr std::int64_t sOrder {             1 };

} // [num]


//
// Numeric encodings
//

	//! Convert transform order to numeric values (e.g. for sorting) [0,1]
	inline
	std::int64_t
	numberFor
		( om::OrderTR const & order
		)
	{
		return static_cast<std::int64_t>(order);
	}

	//! Convert sign collection to numeric values (e.g. for sorting) [0,7]
	inline
	std::int64_t
	numberFor
		( om::ThreeSigns const & signs
		)
	{
		return
			( 4u * (static_cast<std::int64_t>(1u + signs[0]) / 2u)
			+ 2u * (static_cast<std::int64_t>(1u + signs[1]) / 2u)
			+ 1u * (static_cast<std::int64_t>(1u + signs[2]) / 2u)
			);
	}

	//! Convert index collection to numeric values (e.g. for sorting) [0,26]
	inline
	std::int64_t
	numberFor
		( om::ThreeIndices const & indices
		)
	{
		return
			( 9u * static_cast<std::int64_t>(indices[0])
			+ 3u * static_cast<std::int64_t>(indices[1])
			+ 1u * static_cast<std::int64_t>(indices[2])
			);
	}

	//! ThreeSign (int8_t) values for numeric Id
	inline
	om::ThreeSigns
	threeSignsFor
		( std::int64_t const numId
		)
	{
		constexpr std::array<om::ThreeSigns, 8u> asInts
			{ om::ThreeSigns{ -1, -1, -1 }
			, om::ThreeSigns{ -1, -1,  1 }
			, om::ThreeSigns{ -1,  1, -1 }
			, om::ThreeSigns{ -1,  1,  1 }
			, om::ThreeSigns{  1, -1, -1 }
			, om::ThreeSigns{  1, -1,  1 }
			, om::ThreeSigns{  1,  1, -1 }
			, om::ThreeSigns{  1,  1,  1 }
			};
		return asInts[numId];
	}

	//! ThreeIndices (int8_t) values for numeric Id
	inline
	om::ThreeIndices
	threeIndicesFor
		( std::int64_t const numId
		)
	{
		using TI = om::ThreeIndices;
		constexpr std::array<om::ThreeIndices, 27u> asInts
			{ TI{  0,  0,  0 }, TI{  0,  0,  1 }, TI{  0,  0,  2 }
			, TI{  0,  1,  0 }, TI{  0,  1,  1 }, TI{  0,  1,  2 }
			, TI{  0,  2,  0 }, TI{  0,  2,  1 }, TI{  0,  2,  2 }
			, TI{  1,  0,  0 }, TI{  1,  0,  1 }, TI{  1,  0,  2 }
			, TI{  1,  1,  0 }, TI{  1,  1,  1 }, TI{  1,  1,  2 }
			, TI{  1,  2,  0 }, TI{  1,  2,  1 }, TI{  1,  2,  2 }
			, TI{  2,  0,  0 }, TI{  2,  0,  1 }, TI{  2,  0,  2 }
			, TI{  2,  1,  0 }, TI{  2,  1,  1 }, TI{  2,  1,  2 }
			, TI{  2,  2,  0 }, TI{  2,  2,  1 }, TI{  2,  2,  2 }
			};
		return asInts[numId];
	}

	//! OrderTR (enum) values for numeric Id
	inline
	om::OrderTR
	orderFor
		( std::int64_t const numId
		)
	{
		constexpr std::array<om::OrderTR, 3u> orders
			{ om::TranRot
			, om::RotTran
			, om::Unknown
			};
		return orders[numId];
	}


} // [anon]


namespace om
{

//
//==========================================================================
// ConventionOffset
//==========================================================================
//

// static
std::vector<ConventionOffset>
ConventionOffset :: allConventions
	()
{
	std::vector<ConventionOffset> conventions;
	conventions.reserve(48u);

	// all combinations of each characteristic
	std::array<ThreeSigns, 8u> const locSigns{ allThreeSigns() };
	std::array<ThreeIndices, 6u> const locNdxs{ allThreeIndices() };

	// brute force generation of all possible combinations
	for (ThreeSigns const & locSign : locSigns)
	{
		for (ThreeIndices const & locNdx : locNdxs)
		{
			ConventionOffset const convention
				{ locSign
				, locNdx
				};
			conventions.emplace_back(convention);
		}
	}
	return conventions;
}

rigibra::Location
ConventionOffset :: offsetFor
	( ParmGroup const & parmGroup
	) const
{
	std::array<double, 3u> const & dVals = parmGroup.theDistances;

	// gather signed distance values together
	ThreeDistances const offset
		{ theOffSigns[0] * dVals[theOffIndices[0]]
		, theOffSigns[1] * dVals[theOffIndices[1]]
		, theOffSigns[2] * dVals[theOffIndices[2]]
		};

	return engabra::g3::Vector{ offset };
}


std::size_t
ConventionOffset :: indexValue
	() const
{
	// 216 elements:  8 (signs) * 27 (indices)
	std::size_t const numSgn{ (std::size_t)numberFor(theOffSigns) };
	std::size_t const numNdx{ (std::size_t)numberFor(theOffIndices) };
	std::size_t const ndxVal( (numSgn * 27u) + numNdx );
	return ndxVal;
}

std::string
ConventionOffset :: infoString
	( std::string const & title
	) const
{
	std::ostringstream oss;
	if (! title.empty())
	{
		oss << title << ' ';
	}
	oss
		<< " Off+/-: " << infoStringSigns(theOffSigns)
		<< " OffNdx: " << infoStringIndices(theOffIndices)
		<< " ndxVal: " << std::setw(3u) << indexValue()
		;
	return oss.str();
}

//
//==========================================================================
// ConventionAngle
//==========================================================================
//

// static
std::vector<ConventionAngle>
ConventionAngle :: allConventions
	()
{
	std::vector<ConventionAngle> conventions;
	conventions.reserve(576u);

	// all combinations of each characteristic
	std::array<ThreeSigns, 8u> const attSigns{ allThreeSigns() };
	std::array<ThreeIndices, 6u> const attNdxs{ allThreeIndices() };
	std::array<ThreeIndices, 12u> const bivNdxs{ allBivIndices() };

	// brute force generation of all possible combinations
	for (ThreeSigns const & attSign : attSigns)
	{
		for (ThreeIndices const & attNdx : attNdxs)
		{
			for (ThreeIndices const & bivNdx : bivNdxs)
			{
				ConventionAngle const convention
					{ attSign
					, attNdx
					, bivNdx
					};
				conventions.emplace_back(convention);
			}
		}
	}
	return conventions;
}

std::size_t
ConventionAngle :: indexValue
	() const
{
	// 5832 possible: 8(signs) * 27 (angSizeIndices) * 27 (bivIndices)
	std::size_t const numAngSgn{ (std::size_t)numberFor(theAngSigns) };
	std::size_t const numAngNdx{ (std::size_t)numberFor(theAngIndices) };
	std::size_t const numBivNdx{ (std::size_t)numberFor(theBivIndices) };
	std::size_t const ndxVal
		{ (numAngSgn * 27u * 27u)
		+ (numAngNdx * 27u)
		+  numBivNdx
		};
	return ndxVal;
}

rigibra::Attitude
ConventionAngle :: attitudeFor
	( ParmGroup const & parmGroup
	) const
{
	using namespace rigibra;
	Attitude attNet{ null<Attitude>() };

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

	// form physical angles
	PhysAngle const physAngleA{ angleSizes[0] * angleDirs[0] };
	PhysAngle const physAngleB{ angleSizes[1] * angleDirs[1] };
	PhysAngle const physAngleC{ angleSizes[2] * angleDirs[2] };

	// generate attitude from 3-angle-sequence
	Attitude const attA(physAngleA);
	Attitude const attB(physAngleB);
	Attitude const attC(physAngleC);
	attNet = attC * attB * attA;

	return attNet;
}

std::string
ConventionAngle :: infoString
	( std::string const & title
	) const
{
	std::ostringstream oss;
	if (! title.empty())
	{
		oss << title << ' ';
	}
	oss
		<< " Ang+/-: " << infoStringSigns(theAngSigns)
		<< " AngNdx: " << infoStringIndices(theAngIndices)
		<< " BivNdx: " << infoStringIndices(theBivIndices)
		<< " ndxVal: " << std::setw(3u) << indexValue()
		;
	return oss.str();
}


//
//==========================================================================
// Convention
//==========================================================================
//

// static
std::vector<Convention>
Convention :: allConventionsFor
	( ConventionOffset const & offConv
	)
{
	std::vector<Convention> conventions;
	std::vector<ConventionAngle>
		const angConvs{ ConventionAngle::allConventions() };
	std::array<OrderTR, 2u>
		const orders{ allOrderTRs() };

	for (ConventionAngle const & angConv : angConvs)
	{
		for (OrderTR const & order : orders)
		{
			Convention const convention{ offConv, angConv, order };
			conventions.emplace_back(convention);
		}
	}
	return conventions;
}

// static
std::vector<Convention>
Convention :: allConventions
	()
{
	std::vector<Convention> conventions;
	conventions.reserve(55296);

	std::vector<ConventionOffset>
		const offConvs{ ConventionOffset::allConventions() };

	for (ConventionOffset const & offConv : offConvs)
	{
		std::vector<Convention> const offConvs
			{ allConventionsFor(offConv) };
		conventions.insert
			( conventions.end()
			, offConvs.begin(), offConvs.end()
			);
	}

	return conventions;
}

// static
Convention
Convention :: fromNumberEncoding
	( std::int64_t const & numId
	)
{
	std::int64_t curr{ numId };

	std::int64_t digOrder{ curr % num::sIdBase };
	curr = curr / num::sIdBase;

	std::int64_t digBivNdx{ curr % num::sIdBase };
	curr = curr / num::sIdBase;

	std::int64_t digAngNdx{ curr % num::sIdBase };
	curr = curr / num::sIdBase;

	std::int64_t digAngSgn{ curr % num::sIdBase };
	curr = curr / num::sIdBase;

	std::int64_t digOffNdx{ curr % num::sIdBase };
	curr = curr / num::sIdBase;

	std::int64_t digOffSgn{ curr % num::sIdBase };
	curr = curr / num::sIdBase;

	// should be 1
	// std::int64_t digPad{ curr % num::sIdBase };
	// curr = curr / num::sIdBase;

	return Convention
		{ ThreeSigns{ threeSignsFor(digOffSgn) }
		, ThreeIndices{ threeIndicesFor(digOffNdx) }
		, ThreeSigns{ threeSignsFor(digAngSgn) }
		, ThreeIndices{ threeIndicesFor(digAngNdx) }
		, ThreeIndices{ threeIndicesFor(digBivNdx) }
		, OrderTR{ orderFor(digOrder) }
		};
}

std::int64_t
Convention :: numberEncoding
	() const
{
	std::int64_t numId{ -1 };;
	if (isValid())
	{
		numId = 
			( num::sPad       // so that all values show same width
			+ num::sOffSgn  * numberFor(theConvOff.theOffSigns)
			+ num::sOffNdx  * numberFor(theConvOff.theOffIndices)
			+ num::sAngSgn  * numberFor(theConvAng.theAngSigns)
			+ num::sAngNdx  * numberFor(theConvAng.theAngIndices)
			+ num::sBivNdx  * numberFor(theConvAng.theBivIndices)
			+ num::sOrder   * numberFor(theOrder)
			);
	}
	return numId;
}

bool
Convention :: isValid
	() const
{
	return (Unknown != theOrder);
}

rigibra::Location
Convention :: offsetFor
	( ParmGroup const & parmGroup
	) const
{
	return theConvOff.offsetFor(parmGroup);
}

rigibra::Attitude
Convention :: attitudeFor
	( ParmGroup const & parmGroup
	) const
{
	return theConvAng.attitudeFor(parmGroup);
}

rigibra::Transform
Convention :: transformFor
	( ParmGroup const & parmGroup
	) const
{
	using engabra::g3::Vector;

	// determine attitude associated with parmGroup
	rigibra::Attitude const attR(attitudeFor(parmGroup));

	// to compute translation
	// first, assume TranRot convention...
	Vector tVec{ offsetFor(parmGroup) };
	// ... unless inverse convention is needed
	if (RotTran == theOrder)
	{
		// compute forward translation from inverse offset convention
		Vector const & ty = tVec; // alias to indicate interpretation change
		tVec = inverse(attR)(ty);
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
		<< " " << theConvOff.infoString()
		<< " " << theConvAng.infoString()
		<< "   Order: " << infoStringOrders(theOrder)
		<< "   NumId: " << numberEncoding()
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

ConventionOffset
ConventionString :: conventionOffset
	() const
{
	ThreeSigns const locSigns{ threeSignsFrom(theStrOffSigns) };
	ThreeIndices const locNdxs{ threeIndicesFrom(theStrOffNdxs) };
	ConventionOffset const conOff{ locSigns, locNdxs };
	return conOff;
}

ConventionAngle
ConventionString :: conventionAngle
	() const
{
	ThreeSigns const angSigns{ threeSignsFrom(theStrAngSigns) };
	ThreeIndices const angNdxs{ threeIndicesFrom(theStrAngNdxs) };
	ThreeIndices const bivNdxs{ threeIndicesFrom(theStrBivNdxs) };
	ConventionAngle const conAng{ angSigns, angNdxs, bivNdxs };
	return conAng;
}

Convention
ConventionString :: convention
	() const
{
	OrderTR const order{ orderTRFrom(theStrOrder) };
	return Convention
		{ conventionOffset()
		, conventionAngle()
		, order
		};
	/*
	ThreeSigns const locSigns{ threeSignsFrom(theStrOffSigns) };
	ThreeIndices const locNdxs{ threeIndicesFrom(theStrOffNdxs) };
	ThreeSigns const angSigns{ threeSignsFrom(theStrAngSigns) };
	ThreeIndices const angNdxs{ threeIndicesFrom(theStrAngNdxs) };
	ThreeIndices const bivNdxs{ threeIndicesFrom(theStrBivNdxs) };
	OrderTR const order{ orderTRFrom(theStrOrder) };

	ConventionOffset const conOff{ locSigns, locNdxs };
	ConventionAngle const conAng{ angSigns, angNdxs, bivNdxs };

	return Convention{ conOff, conAng, order };
	*/
}


} // [om]

