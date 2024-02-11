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
\brief Implementation code for ParmGroup.hpp
*/


#include "ParmGroup.hpp"


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
// String encoding support
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

} // [om]

