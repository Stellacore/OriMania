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


#ifndef OriMania_mapkey_INCL_
#define OriMania_mapkey_INCL_

/*! \file
\brief Functions useful for working with keys in std::maps.
*/

/*
Example:
\snippet test_mapkey.cpp DoxyExample01
*/


#include <algorithm>
#include <map>
#include <set>


namespace om
{

namespace mapkey
{
	//! Return keys from map (same sorted order as in map).
	template< typename Key, typename Value >
	inline
	std::set<Key>
	from
		( std::map<Key, Value> const & aMap
		)
	{
		std::set<Key> keys;
		std::transform
			( aMap.cbegin(), aMap.cend()
			, std::inserter(keys, keys.end())
			, [] (typename std::map<Key, Value>::value_type const & pair)
				{ return pair.first; }
			);
		return keys;
	}

	//! Keys in common between both maps
	template< typename Key, typename Value >
	inline
	std::set<Key>
	commonBetween
		( std::map<Key, Value> const & map1
		, std::map<Key, Value> const & map2
		)
	{
		std::set<Key> const keys1{ from(map1) };
		std::set<Key> const keys2{ from(map2) };
		std::set<Key> keysBoth;
		std::set_intersection
			( keys1.cbegin(), keys1.cend()
			, keys2.cbegin(), keys2.cend()
			, std::inserter(keysBoth, keysBoth.end())
			);
		return keysBoth;
	}

	//! Check if both maps have identical keys
	template< typename Key, typename Value >
	inline
	bool
	allMatch
		( std::map<Key, Value> const & map1
		, std::map<Key, Value> const & map2
		)
	{
		bool same{ map1.size() == map2.size() };
		if (same)
		{
			std::set<Key> const keys1{ from(map1) };
			std::set<Key> const keys2{ from(map2) };
			std::set<Key> keysBoth;
			std::set_intersection
				( keys1.cbegin(), keys1.cend()
				, keys2.cbegin(), keys2.cend()
				, std::inserter(keysBoth, keysBoth.end())
				);
			same = (map1.size() == keysBoth.size());
		}
		return same;
	}

} // [mapkey]


} // [om]


#endif // OriMania_mapkey_INCL_
