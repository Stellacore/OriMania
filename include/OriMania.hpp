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


#ifndef OriMania__INCL_
#define OriMania__INCL_

/*! \file
 * \brief Top level include file for OriMania.
 *
 * Provides project level functions and utilities.
 */


// Include the key files

#include "Analysis.hpp"
#include "Convention.hpp"
#include "io.hpp"
#include "Orientation.hpp"

#include <string>


/*! \brief Main namespace for OriMania project software.
 *
 * Example:
 * \snippet test_cmake.cpp DoxyExample01
 *
 */
namespace om
{
		//! Project version description
		std::string
		projectVersion
			();

		//! Project source code identification description
		std::string
		sourceIdentity
			();

} // [om]


#endif // OriMania__INCL_
