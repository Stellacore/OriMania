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

Example:
\snippet test_io.cpp DoxyExample01

*/


#include "Analysis.hpp"
#include "Convention.hpp"
#include "Orientation.hpp"

#include <iostream>
#include <map>
#include <set>
#include <sstream>


namespace om
{
	namespace priv
	{
		//! Leading portion of string before endChar
		inline
		std::string
		activePartOf
			( std::string const & line
			, std::string::value_type const & endChar = '#'
			)
		{
			std::string active;
			std::string::size_type const end{ line.find(endChar) };
			if (std::string::npos != end)
			{
				active = std::string(line.begin(), line.begin()+end);
			}
			else
			{
				active = line;
			}
			return active;
		}

		//! Portion of string with leading and trailing white space removed
		inline
		std::string
		trimmed
			( std::string const & full
			, std::string const & white = " \t"
			)
		{
			std::string trim;
			if (! full.empty())
			{
				using Ndx = std::string::size_type;
				Ndx posBeg{ 0u };
				Ndx posEnd{ full.size() };
				Ndx const gotBeg{ full.find_first_not_of(white) };
				if (std::string::npos != gotBeg)
				{
					posBeg = gotBeg;
				}
				Ndx const gotLast{ full.find_last_not_of(white) };
				if (std::string::npos != gotLast)
				{
					posEnd = gotLast + 1u;
				}
				trim = std::string(full.begin()+posBeg, full.begin()+posEnd);
			}
			return trim;
		}

	} // [priv]


	/*! \brief Orientation results from EO ascii file.
	 *
	 * Example file content and use:
	 * \snippet test_io.cpp DoxyExample01
	 */
	inline
	std::map<SenKey, SenOri>
	loadIndEOs
		( std::istream & istrm
		)
	{
		std::map<SenKey, SenOri> indOris;

		std::string line;
		std::string keyword;
		std::string senKey;
		std::set<SenKey> senKeys;
		std::map<SenKey, Convention> keyConventions;
		std::map<SenKey, ThreeDistances> keyDistances;
		std::map<SenKey, ThreeAngles> keyAngles;
		while (istrm.good() && (! istrm.eof()))
		{
			line.clear();
			std::getline(istrm, line);
			std::string const record
				{ priv::trimmed(priv::activePartOf(line)) };
			if (! record.empty())
			{
				std::istringstream iss(record);
				iss >> keyword >> senKey;
				if ("Convention:" == keyword)
				{
					ConventionString cs;
					iss
						>> cs.theStrLocSigns >> cs.theStrLocNdxs
						>> cs.theStrAngSigns >> cs.theStrAngNdxs
						>> cs.theStrBivNdxs
						>> cs.theStrOrder
						;
					if (cs.isValid())
					{
						Convention const convention{ cs.convention() };
						keyConventions[senKey] = convention;
						senKeys.insert(senKey);
					}
				}
				else
				if ("Distances:" == keyword)
				{
					ThreeDistances dists
						{ engabra::g3::null<double>()
						, engabra::g3::null<double>()
						, engabra::g3::null<double>()
						};
					iss >> dists[0] >> dists[1] >> dists[2];
					using namespace engabra::g3;
					if (isValid(dists))
					{
						keyDistances[senKey] = dists;
						senKeys.insert(senKey);
					}
				}
				else
				if ("Angles:" == keyword)
				{
					ThreeAngles angles
						{ engabra::g3::null<double>()
						, engabra::g3::null<double>()
						, engabra::g3::null<double>()
						};
					iss >> angles[0] >> angles[1] >> angles[2];
					using namespace engabra::g3;
					if (isValid(angles))
					{
						keyAngles[senKey] = angles;
						senKeys.insert(senKey);
					}
				}
			} // record parsing
		} // stream reading

		for (SenKey const & senKey : senKeys)
		{
			std::map<SenKey, Convention>::const_iterator
				const itConvention{ keyConventions.find(senKey) };
			std::map<SenKey, ThreeDistances>::const_iterator
				const itDistance{ keyDistances.find(senKey) };
			std::map<SenKey, ThreeAngles>::const_iterator
				const itAngle{ keyAngles.find(senKey) };
			if ( (keyConventions.end() != itConvention)
			  && (keyDistances.end() != itDistance)
			  && (keyAngles.end() != itAngle)
			   )
			{
				Convention const & convention = itConvention->second;
				ParmGroup const pg{ itDistance->second, itAngle->second };
				SenOri const indOri{ convention.transformFor(pg) };
				indOris[senKey] = indOri;
			}
		}

		return indOris;
	}

	//! \brief String containing first few and last few lines of fitIndexPairs
	inline
	std::string
	infoStringFitConventions
		( std::vector<om::FitNdxPair> const & fitIndexPairs
		, std::vector<om::Convention> const & allConventions
		, std::size_t const & showNumBeg = 8u
		, std::size_t const & showNumEnd = 2u
		)
	{
		std::ostringstream oss;

		// report a few results
		oss << '\n';
		for (std::size_t nn{0u} ; nn < showNumBeg ; ++nn)
		{
			oss
				<< om::infoString(fitIndexPairs[nn], allConventions)
				<< '\n';
		}
		oss << " fitError: ..." << '\n';
		for (std::size_t nn{fitIndexPairs.size()-1u-showNumEnd}
			; nn < (fitIndexPairs.size() - 1u) ; ++nn)
		{
			oss
				<< om::infoString(fitIndexPairs[nn], allConventions)
				<< '\n';
		}

		return oss.str();
	}


} // [om]

// Place operator<<() overloads in global namespace

namespace
{

//
// For Convention.hpp
//

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

} // [anon]


#endif // OriMania_io_INCL_

