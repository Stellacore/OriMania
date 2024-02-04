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

#include "OriMania.hpp"

#include <iostream>
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


}

//! Check behavior of Convention handling
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	testPermuations(oss);

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

