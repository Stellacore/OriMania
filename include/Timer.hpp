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


#ifndef OriMania_Timer_INCL_
#define OriMania_Timer_INCL_

/*! \file
\brief Contains code timing utilities.

Example:
\snippet test_Timer.cpp DoxyExampleTime

*/


#include <chrono>
#include <iomanip>
#include <string>


namespace om
{

	//! \brief High precision (nanosecond) timer
	struct Timer
	{
		//! Instantate with a string value here if desired
		std::string theName{};

		//! Start of interval for elapsed() computation.
		std::chrono::time_point<std::chrono::high_resolution_clock>
			theBeg{ std::chrono::high_resolution_clock::now() };

		//! End of interval for elapsed() computation.
		std::chrono::time_point<std::chrono::high_resolution_clock>
			theEnd{ std::chrono::high_resolution_clock::now() };

		//! Set theBeg time value (used for elapsed time evaluation)
		inline
		void
		restart
			()
		{
			theBeg = std::chrono::high_resolution_clock::now();
		}

		//! Set theEnd time value (used for elapsed time evaluation)
		inline
		void
		stop
			()
		{
			theEnd = std::chrono::high_resolution_clock::now();
		}

		//! Elapsed time from restart() to stop() (theEnd - theBeg) in [sec]
		inline
		double
		elapsed
			() const
		{
			using namespace std::chrono;
			duration const duro{ duration_cast<nanoseconds>(theEnd - theBeg) };
			double const delta{ 1.e-9 * static_cast<double>(duro.count()) };
			return delta;
		}

	}; // Timer

} // [om]


namespace
{
	//! Put timer information to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, om::Timer const & tmr
		)
	{
		ostrm
			<< std::setw(15u) << std::fixed << tmr.elapsed()
			<< ' '
			<< tmr.theName
			;
		return ostrm;
	}

} // [anon]


#endif // OriMania_Timer_INCL_
