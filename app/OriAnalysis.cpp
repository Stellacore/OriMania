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
\brief Application for estimating payload ExCal parameter conventions.
*/


#include "OriMania.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>


namespace
{
	//! Check basic application usage
	struct Usage
	{
		std::filesystem::path theIndEoPath{};
		std::filesystem::path theParmGroupPath{};

		//! True if verboase output has been requested
		inline
		bool
		isVerbose
			() const
		{
			return true;
		}

		//! Check invocation arguments.
		explicit
		Usage
			( int argc
			, char * argv[]
			)
		{
			int narg{ 1 };
			if (2 < argc)
			{
				theIndEoPath = argv[narg++];
				theParmGroupPath = argv[narg++];
			}
			else
			{
				std::cerr << '\n' << argv[0] << " Bad invocation:"
					"\nUsage:"
					"\n  <ProgName> <IndEoPath> <ParmGroupPath>"
					"\n\n"
					;
			}
		}

		//! True if input file path is set to existing file.
		inline
		bool
		isValid
			() const
		{
			return std::filesystem::exists(theIndEoPath);
		}

	}; // Usage

} // [anon]


/*! \brief Estimate payload sensor ExCal tranforms by analysing exported data.
 *
 * \arg Load independent EO's via om::loadIndEOs().
 *
 */
int
main
	( int argc
	, char * argv[]
	)
{
	Usage const use(argc, argv);
	if (! use.isValid())
	{
		return 1;
	}

	using namespace om;

	// load indepenent EOs from specified file
	std::ifstream ifsEO(use.theIndEoPath);
	std::map<SenKey, SenOri> const indKeyOris{ loadIndEOs(ifsEO) };

	if (! (1u < indKeyOris.size()))
	{
		std::cerr << "\nFatal Error:\n";
		std::cerr
			<< "  Only loaded " << indKeyOris.size() << " indpendent EOs.\n"
			<< "  Need at least two in order to form relative orientations!\n"
			<< '\n';
		return 2;
	}

	// load ParmGroups from specified file
	std::ifstream ifsPG(use.theParmGroupPath);
	std::map<om::SenKey, om::ParmGroup>
		const keyPGs{ om::loadParmGroups(ifsPG) };

	std::vector<om::Convention> const allCons{ Convention::allConventions() };
	std::vector<om::FitNdxPair> fitIndexPairs
		{ fitIndexPairsFor(keyPGs, indKeyOris, allCons) };

	// sort from best and worst
	std::sort(fitIndexPairs.begin(), fitIndexPairs.end());

	// report data encountered
	if (use.isVerbose())
	{
		std::ostringstream msg;

		// report ParmGroup values
		msg << '\n';
		for (std::map<om::SenKey, om::ParmGroup>::value_type
			const & keyPG : keyPGs)
		{
			msg << "PG: " << keyPG.first
				<< " " << keyPG.second
				<< '\n';
		}

		// report independent EO values
		msg << '\n';
		msg << "Independent EO count: " << indKeyOris.size() << '\n';
		for (std::map<SenKey, SenOri>::const_iterator
			iter{indKeyOris.begin()} ; indKeyOris.end() != iter ; ++iter)
		{
			msg
				<< std::setw(12u) << iter->first
				<< ' ' << iter->second
				<< '\n';
		}

		// report ind relative orientations
		std::map<KeyPair, SenOri> const indKeyROs
			{ relativeOrientationBetweens(indKeyOris) };
		msg << '\n';
		for (std::map<KeyPair, SenOri>::value_type
			const & indKeyRO : indKeyROs)
		{
			msg << indKeyRO.first
				<< "  " << indKeyRO.second
				<< '\n';
		}

		// display first and last several lines
		msg << '\n';
		msg << om::infoStringFitConventions(fitIndexPairs, allCons) << '\n';
		msg << "===\n";

		std::cout << msg.str();
	}

	// report results
	if (! fitIndexPairs.empty())
	{
		std::size_t const numShow
			{ std::min((std::size_t)5u, (std::size_t)fitIndexPairs.size()) };
		std::cout << '\n';
		std::cout << "Best fitting Conventions\n";
		std::cout << om::infoStringFitConventions
			(fitIndexPairs.begin(), fitIndexPairs.begin()+numShow, allCons)
			<< '\n';
		std::cout << '\n';
	}
	else
	{
		std::cerr << "Error: No results to report\n" << std::endl;
	}

	return 0;
}


