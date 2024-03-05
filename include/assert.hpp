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


#ifndef OriMania_assert_INCL_
#define OriMania_assert_INCL_

/*! \file
\brief Utility functions for hard "assert" operations.

A key property of these functions is that they are *IN*depen of code
compilation flags - e.g. asserts remain in place even for release
optimzied builds.

*/

/*
Example:
\snippet test_assert.cpp DoxyExample01

*/


#include <iostream>
#include <string>


namespace om
{

	//! If mustBeTrue is not true, put msg to std::cerr and exit().
	inline
	void
	assertExit
		( bool const & mustBeTrue
		, std::string const & msg = {}
		, int const & exitCode = 1
		)
	{
		if (! mustBeTrue)
		{
			std::cerr << "Assert Error!\n";
			std::cerr << msg << std::endl;
			exit(exitCode);
		}
	}

} // [om]


#endif // OriMania_assert_INCL_
