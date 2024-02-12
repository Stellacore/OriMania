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


#ifndef OriMania_io_INCL_
#define OriMania_io_INCL_

/*! \file
\brief Functions for supporting basic input/output operations.
*/

/*
Example:
\snippet test_io.cpp DoxyExample01

*/


#include "Analysis.hpp"
#include "Convention.hpp"
#include "Orientation.hpp"
#include "ParmGroup.hpp"

#include <iostream>
#include <map>
#include <set>
#include <sstream>


namespace om
{

//
// String processing utilities
//

	//! Leading portion of string before endChar
	std::string
	withoutComment
		( std::string const & line
		, std::string::value_type const & endChar = '#'
		);

	//! Portion of string with leading and trailing white space removed
	std::string
	trimmed
		( std::string const & full
		, std::string const & white = " \t"
		);

//
// Data values loaders
//

	/*! \brief Orientation results from EO ascii data stream.
	 *
	 * Example file content and use:
	 * \snippet test_io.cpp DoxyExampleLoadIndEOs
	 */
	std::map<SenKey, SenOri>
	loadIndEOs
		( std::istream & istrm
		);
	
	/*! \brief ParmGroup data values loaded from ascii data stream.
	 *
	 * Example file content and use:
	 * \snippet test_io.cpp DoxyExampleLoadPG
	 */
	std::map<SenKey, ParmGroup>
	loadParmGroups
		( std::istream & istrm
		);

//
// Descriptive strings for various items
//

	//! String with of FitNdxPair data with associated Convention
	std::string
	infoString
		( FitNdxPair const & fitConPair
		, std::vector<Convention> const & allConventions
		);

	//! \brief String containing range of fitIndexPairs
	std::string
	infoStringFitConventions
		( std::vector<om::FitNdxPair>::const_iterator const & fitNdxBeg
		, std::vector<om::FitNdxPair>::const_iterator const & fitNdxEnd
		, std::vector<om::Convention> const & allConventions
		);

	//! \brief String containing first few and last few lines of fitIndexPairs
	std::string
	infoStringFitConventions
		( std::vector<om::FitNdxPair> const & fitIndexPairs
		, std::vector<om::Convention> const & allConventions
		, std::size_t const & showNumBeg = 8u
		, std::size_t const & showNumEnd = 2u
		);

} // [om]


// Place operator<<() overloads in global namespace

namespace
{

//
// For ParmGroup.hpp
//

	//! Put ParmGroup infoString() output to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, om::ParmGroup const & parmGroup
		)
	{
		ostrm << parmGroup.infoString();
		return ostrm;
	}

//
// For Convention.hpp
//

	//! Put std::array data to stream
	template <std::size_t Dim = 3u>
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, std::array<double, Dim> const & values
		)
	{
		for (double const & value : values)
		{
			ostrm << ' ' << engabra::g3::io::fixed(value);
		}
		return ostrm;
	}

	//! Put convention infoString() output to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, om::Convention const & convention
		)
	{
		ostrm << convention.infoString();
		return ostrm;
	}

//
// For Orientation.hpp
//

	//! Put KeyPair.infoString() to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, om::KeyPair const & keyPair
		)
	{
		ostrm << keyPair.infoString();
		return ostrm;
	}

	//! \brief Put collection of exterior orientations to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, std::map<om::SenKey, om::SenOri> const & keyOris
		)
	{
		using namespace om;
		std::size_t recCount{ 0u };
		for (std::map<SenKey, SenOri>::value_type const & keyOri : keyOris)
		{
			if (0u < (recCount++))
			{
				ostrm << '\n';
			}
			ostrm
				<< " EO: " << keyOri.first
				<< "  oriSenWrtRef: " << keyOri.second
				;
		}
		return ostrm;
	}

	//! \brief Put collection of relative orientations to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, std::map<om::KeyPair, om::SenOri> const & keyRos
		)
	{
		using namespace om;
		std::size_t recCount{ 0u };
		for (std::map<KeyPair, SenOri>::value_type const & keyRo : keyRos)
		{
			if (0u < (recCount++))
			{
				ostrm << '\n';
			}
			ostrm
				<< " RO: " << keyRo.first
				<< " " << keyRo.second
				;
		}
		return ostrm;
	}

//
// For Analysis.hpp
//

	//! Put OneTrialResult.infoString() to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, om::OneTrialResult const & trialResult
		)
	{
		ostrm << trialResult.infoString();
		return ostrm;
	}


} // [anon]


#endif // OriMania_io_INCL_

