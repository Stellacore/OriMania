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
\brief Implementation code for OriMania io.hpp
*/


#include "io.hpp"

#include <locale>


namespace om
{

std::string
withoutComment
	( std::string const & line
	, std::string::value_type const & endChar
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

std::string
trimmed
	( std::string const & full
	, std::string const & white
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

std::map<SenKey, SenOri>
loadOrientations
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
			{ trimmed(withoutComment(line)) };
		if (! record.empty())
		{
			std::istringstream iss(record);
			iss >> keyword >> senKey;
			if ("Convention:" == keyword)
			{
				std::string encoding;
				std::getline(iss, encoding);
				ConventionString const cs
					{ ConventionString::from(encoding) };
				if (cs.isValid())
				{
					Convention const convention{ cs.convention() };
					keyConventions[senKey] = convention;
					senKeys.insert(senKey);
				}
			}
			else
			if ("Locations:" == keyword)
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

std::map<SenKey, ParmGroup>
loadParmGroups
	( std::istream & istrm
	)
{
	std::map<SenKey, ParmGroup> pgs;
	std::string line;
	std::string keyword;
	std::string senKey;
	std::set<SenKey> senKeys;
	std::map<SenKey, ThreeDistances> keyDistances;
	std::map<SenKey, ThreeAngles> keyAngles;
	while (istrm.good() && (! istrm.eof()))
	{
		line.clear();
		std::getline(istrm, line);
		std::string const record
			{ trimmed(withoutComment(line)) };
		if (! record.empty())
		{
			std::istringstream iss(record);
			iss >> keyword >> senKey;
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
		std::map<SenKey, ThreeDistances>::const_iterator
			const itDistance{ keyDistances.find(senKey) };
		std::map<SenKey, ThreeAngles>::const_iterator
			const itAngle{ keyAngles.find(senKey) };
		if ( (keyDistances.end() != itDistance)
		  && (keyAngles.end() != itAngle)
		   )
		{
			ParmGroup const pg{ itDistance->second, itAngle->second };
			if (pg.isValid())
			{
				pgs[senKey] = pg;
			}
		}
	}

	return pgs;
}

std::string
commaNumber
	( std::size_t const & num
	)
{
	std::stringstream ss;
	ss.imbue(std::locale(""));
	ss << std::fixed << num ;
	return  ss.str();
}

std::string
infoString
	( FitNdxPair const & fitConPair
	, std::vector<Convention> const & allConventions
	)
{
	std::ostringstream oss;
	double const & fitError = fitConPair.first;
	Convention const & convention = allConventions[fitConPair.second];
	ConventionString const cs{ ConventionString::from(convention) };
	using engabra::g3::io::fixed;
	oss
		<< " fitError: " << fixed(fitError)
		<< "  convention: " << convention.numberEncoding()
		<< " '" << cs.stringEncoding() << "'"
		;
	return oss.str();
}

std::string
infoStringFitConventions
	( std::vector<om::FitNdxPair>::const_iterator const & fitNdxBeg
	, std::vector<om::FitNdxPair>::const_iterator const & fitNdxEnd
	, std::vector<om::Convention> const & allConventions
	)
{
	std::ostringstream oss;
	std::size_t count{0u};
	for (std::vector<om::FitNdxPair>::const_iterator
		iter{ fitNdxBeg } ; fitNdxEnd != iter ; ++iter)
	{
		if (0 < count++)
		{
			oss << '\n';
		}
		oss << om::infoString(*iter, allConventions);
	}
	return oss.str();
}

std::string
infoStringFitConventions
	( std::vector<om::FitNdxPair> const & fitIndexPairs
	, std::vector<om::Convention> const & allConventions
	, std::size_t const & showNumBeg
	, std::size_t const & showNumEnd
	)
{
	std::ostringstream oss;

	std::size_t const ndxBegAll{ 0u };
	std::size_t const ndxEndAll{ fitIndexPairs.size() };
	if (! ((showNumBeg + showNumEnd) < ndxEndAll))
	{
		// not enough data values, just show entire collection
		oss << infoStringFitConventions
			(fitIndexPairs.begin(), fitIndexPairs.end(), allConventions);
	}
	else
	{
		// show begining and end in separate sections
		std::size_t const ndxBeg1{ ndxBegAll };
		std::size_t const ndxEnd1{ ndxBeg1 + showNumBeg };
		std::size_t const ndxEnd2{ ndxEndAll };
		std::size_t const ndxBeg2{ ndxEnd2 - showNumEnd };
		oss << infoStringFitConventions
				( fitIndexPairs.begin() + ndxBeg1
				, fitIndexPairs.begin() + ndxEnd1
				, allConventions
				)
			<< '\n';
		oss << " : ..." << '\n';
		oss << infoStringFitConventions
				( fitIndexPairs.begin() + ndxBeg2
				, fitIndexPairs.begin() + ndxEnd2
				, allConventions
				);
	}

	return oss.str();
}

} // [om]

