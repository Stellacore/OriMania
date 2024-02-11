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

	// form physical angles
	using namespace rigibra;
	PhysAngle const physAngleA{ angleSizes[0] * angleDirs[0] };
	PhysAngle const physAngleB{ angleSizes[1] * angleDirs[1] };
	PhysAngle const physAngleC{ angleSizes[2] * angleDirs[2] };

	// generate attitude from 3-angle-sequence
	Attitude const attA(physAngleA);
	Attitude const attB(physAngleB);
	Attitude const attC(physAngleC);
	Attitude const attNet(attC * attB * attA);

	return attNet;
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

