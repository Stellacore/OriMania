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
\brief Unit tests (and example) code for om:::Convention
*/


#include "Convention.hpp"
#include "io.hpp"

#include <iostream>
#include <map>
#include <set>
#include <sstream>


namespace
{
	//! Check that number and uniqueness of conventions
	void
	testPermuations
		( std::ostream & oss
		)
	{
		constexpr std::size_t expNumConventions{ 55296u };

		// check for small data storage size
		constexpr std::size_t expDataSize{ 3u + 3u + 3u + 3u + 6u + 2u };
		std::size_t const gotDataSize{ sizeof(om::Convention) };
		if (! (expDataSize == gotDataSize))
		{
			oss << "Failure of per convention data size test\n";
			oss << "exp: " << expDataSize << '\n';
			oss << "got: " << gotDataSize << '\n';
		}

		// generate all combinations of data sets
		std::vector<om::Convention> const conventions
			{ om::Convention::allConventions() };

		// check number of conventions supported
		if (! (expNumConventions == conventions.size()))
		{
			oss << "Failure to testConventions count test\n";
			oss << "exp: " << expNumConventions << '\n';
			oss << "got: " << conventions.size() << '\n';
		}

		// check if all are unique
		std::set<om::Convention> const uniques
			(conventions.cbegin(), conventions.cend());
		if (! (uniques.size() == conventions.size()))
		{
			oss << "Failure of testConventions uniqueness test\n";
			oss << "exp: " << conventions.size() << '\n';
			oss << "got: " << uniques.size() << '\n';
		}
	}

	//! Check numeric encoding
	void
	testNumId
		( std::ostream & oss
		)
	{
		om::ConventionString const expCS
			{ "+-+", "210", "++-", "201", "102", "1"};
		om::Convention const expCon{ expCS.convention() };
		std::int64_t const gotNum{ expCon.numberEncoding() };

		om::Convention const gotCon
			{ om::Convention::fromNumberEncoding(gotNum) };
		if (! (gotCon == expCon))
		{
			oss << "Failure of numeric en/de-code test\n";
			oss << "exp: " << expCon << '\n';
			oss << "got: " << gotCon << '\n';
			oss << "gotNum: " << gotNum << '\n';
		}

	}

	//! Check key generation for conventions
	void
	testKeys
		( std::ostream & oss
		)
	{
		// generate all combinations of data sets
		std::vector<om::Convention> const conventions
			{ om::Convention::allConventions() };

		// store in map using numberEncoding() as key
		using NumId = std::int64_t;
		std::map<NumId, om::Convention> numCons;
		for (om::Convention const & convention : conventions)
		{
			numCons.emplace_hint
				( numCons.end()
				, std::make_pair(convention.numberEncoding(), convention)
				);
		}

		// check number of unique keys matches number of conventions
		if (! (conventions.size() == numCons.size()))
		{
			oss << "Failure of convention/numCon size test\n";
			oss << "conventions.size(): " << conventions.size() << '\n';
			oss << "    numCons.size(): " << numCons.size() << '\n';
		}

		// retrieve numeric values and reconstruct conventions
		for (std::map<NumId, om::Convention>::value_type
			const & numCon : numCons)
		{
			NumId const & numId = numCon.first;
			om::Convention const & expCon = numCon.second;
			om::Convention const gotCon
				{ om::Convention::fromNumberEncoding(numId) };
			if (! (gotCon == expCon))
			{
				oss << "Failure of numeric en/de-code test\n";
				oss << "exp: " << expCon << '\n';
				oss << "got: " << gotCon << '\n';
			}
		}
	}

	using Hash = std::array<double, 9u>;

	//! Create a hash that represents result of a transformation.
	inline
	Hash
	hashfor
		( rigibra::Transform const & xfm
		)
	{
		Hash hVals;
		using namespace engabra::g3;
		Vector const y1{ xfm(e1) };
		Vector const y2{ xfm(e2) };
		Vector const y3{ xfm(e3) };
		hVals[0] = y1[0];
		hVals[1] = y1[1];
		hVals[2] = y1[2];
		hVals[3] = y2[0];
		hVals[4] = y2[1];
		hVals[5] = y2[2];
		hVals[6] = y3[0];
		hVals[7] = y3[1];
		hVals[8] = y3[2];
		return hVals;
	}

	//! Check for transformation function availability
	void
	testTransforms
		( std::ostream & oss
		)
	{
		std::vector<om::Convention> const conventions
			{ om::Convention::allConventions() };
		om::ParmGroup const parmGroup
			{ om::ThreeAngles{ -.7, .3, -.5 }
			, om::ThreeAngles{ 10., -30., 20. }
			};

		std::set<Hash> hashes; // use to check uniqueness
		for (om::Convention const & convention : conventions)
		{
			using namespace rigibra; // for Transform related capabilities
			Transform const xfm{ convention.transformFor(parmGroup) };

			// check if transform is valid
			if (! isValid(xfm))
			{
				oss << "Failure to construct valid transformation test\n";
				oss << "convention: " << convention << '\n';
				oss << " parmGroup: " << parmGroup << '\n';
				oss << "       xfm: " << xfm << '\n';
				break;
			}

			// transform 3 basis vectors and use result as indicator of
			// transformation result (in order to check if they are all
			// different from each other.
			Hash const hash{ hashfor(xfm) };
			hashes.emplace_hint(hashes.end(), hash);
		}

		// check how many of the transformations are unique (should be all)
		std::size_t const expUnique{ conventions.size() };
		std::size_t const gotUnique{ hashes.size() };
		if (! (gotUnique == expUnique))
		{
			oss << "Failure of unique transform result test\n";
			oss << "expUnique: " << expUnique << '\n';
			oss << "gotUnique: " << gotUnique << '\n';
		}
	}

	//! Check string en/de-coding of conventions
	void
	testEncode
		( std::ostream & oss
		)
	{
		std::string const expStr("+++ 012 +++ 012 012 0");
		om::ConventionString const cs1{ om::ConventionString::from(expStr) };
		std::string const gotStr{ cs1.stringEncoding() };
		if (! (gotStr == expStr))
		{
			oss << "Failure of string encoding test\n";
			oss << "exp: '" << expStr << "'\n";
			oss << "got: '" << gotStr << "'\n";
		}

		om::Convention const convention{ cs1.convention() };
		om::ConventionString const cs2
			{ om::ConventionString::from(convention) };

		if (! (cs2.stringEncoding() == cs1.stringEncoding()))
		{
			oss << "Failure of Convention reconstruction test\n";
			oss << "cs1: " << cs1.stringEncoding() << '\n';
			oss << "convention: " << convention << '\n';
			oss << "cs2: " << cs2.stringEncoding() << '\n';
		}
	}

	//! Check Convention Offset index generation
	void
	testIndicesOffset
		( std::ostream & oss
		)
	{
		using namespace om;

		std::vector<ConventionOffset> const allCons
			{ ConventionOffset::allConventions() };
		// Note that only 6 of the 27 index permuations use all 3 indices
		constexpr std::size_t expSize{ 8u * 6u };
		if (! (expSize == allCons.size()))
		{
			oss << "Failure offset convention size test\n";
			oss << "exp: " << expSize << '\n';
			oss << "got: " << allCons.size() << '\n';
		}

		std::set<std::size_t> uniqNdxs;
		for (ConventionOffset const & allCon : allCons)
		{
			uniqNdxs.insert(allCon.indexValue());
		}

		constexpr std::size_t expUniq{ 48u };
		if (! (expUniq == uniqNdxs.size()))
		{
			oss << "Failure of unique offset index test\n";
			oss << "exp: " << expUniq << '\n';
			oss << "got: " << uniqNdxs.size() << '\n';
		}
	}

	//! Check Convention Angle index generation
	void
	testIndicesAngle
		( std::ostream & oss
		)
	{
		using namespace om;

		std::vector<ConventionAngle> const allCons
			{ ConventionAngle::allConventions() };
		// Note that only 6 of the 27 index permuations are relevant
		//      for each of the angle size but 12 of 27 for biv dir
		constexpr std::size_t expSize{ 8u * 6u * 12u };
		if (! (expSize == allCons.size()))
		{
			oss << "Failure angle convention size test\n";
			oss << "exp: " << expSize << '\n';
			oss << "got: " << allCons.size() << '\n';
		}

		std::set<std::size_t> uniqNdxs;
		for (ConventionAngle const & allCon : allCons)
		{
			uniqNdxs.insert(allCon.indexValue());
		}

		constexpr std::size_t expUniq{ 8u * 6u * 12u };
		if (! (expUniq == uniqNdxs.size()))
		{
			oss << "Failure of unique angle index test\n";
			oss << "exp: " << expUniq << '\n';
			oss << "got: " << uniqNdxs.size() << '\n';
		}
	}
}

//! Check behavior of Convention handling
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	testPermuations(oss);
	testNumId(oss);
	testKeys(oss);
	testTransforms(oss);
	testEncode(oss);
	testIndicesOffset(oss);
	testIndicesAngle(oss);

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

