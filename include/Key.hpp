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


#ifndef OriMania_Key_INCL_
#define OriMania_Key_INCL_

/*! \file
\brief Contains Functions for management of data keys.
*/

/*
Example:
\snippet test_Key.cpp DoxyExample01

*/

#include <string>
#include <sstream>


namespace om
{

	//! Assume individual sensors are identified by arbitrary string values.
	using SenKey = std::string;

	//! encode numeric value into sensor key
	inline
	std::string
	keyFrom
		( std::size_t const & num
		)
	{
		std::ostringstream oss;
		oss << "Key_" << num;
		return oss.str();
	}

	//! Pair of SenKey representing two members in pairwise relationship.
	struct KeyPair
	{
		//! Key value associated relationship domain.
		SenKey theKeyFrom;

		//! Key value associated relationship range.
		SenKey theKeyInto;

		//! Same as from() to emphasize first in order for 2w1 notation
		inline SenKey const & key1() const { return theKeyFrom; }

		//! Same as into() to emphasize second in order for 2w1 notation
		inline SenKey const & key2() const { return theKeyInto; }

		//! Shorthand name access to theKeyFrom.
		inline
		SenKey const &
		from
			() const
		{
			return theKeyFrom;
		}

		//! Shorthand name access to theKeyInto.
		inline
		SenKey const &
		into
			() const
		{
			return theKeyInto;
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
				oss << title << " ";
			}
			oss
				<< "(Into): " << into()
				<< "  "
				<< "Wrt(From): " << from()
				;
			return oss.str();
		}

	}; // KeyPair

	//! Lexicographic comparision of keys
	inline
	bool
	operator<
		( KeyPair const & pairA
		, KeyPair const & pairB
		)
	{
		// use logic from std::pair
		std::pair const sPairA{ pairA.theKeyFrom, pairA.theKeyInto };
		std::pair const sPairB{ pairB.theKeyFrom, pairB.theKeyInto };
		return (sPairA < sPairB);
	}

} // [om]


#endif // OriMania_Key_INCL_
