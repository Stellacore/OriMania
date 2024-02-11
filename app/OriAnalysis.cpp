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
		std::filesystem::path theBoxPGPath{};
		std::filesystem::path theIndPGPath{};

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
				theBoxPGPath = argv[narg++];
				theIndPGPath = argv[narg++];
			}
			else
			{
				std::cerr << '\n' << argv[0] << " Bad invocation:"
					"\nUsage:"
					"\n  <ProgName> <BoxPGPath> <IndPGPath>"
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
			return
				(  std::filesystem::exists(theBoxPGPath)
				&& std::filesystem::exists(theBoxPGPath)
				);
		}

	}; // Usage

} // [anon]


namespace rpt
{

	inline
	std::string
	stringInputs
		( std::map<om::SenKey, om::ParmGroup> const & keyBoxPGs
		, std::map<om::SenKey, om::SenOri> const & indKeyOris
		)
	{
		std::ostringstream msg;

		using namespace om;

		// report ParmGroup values
		msg << '\n';
		msg << "Box ParmGroup count: " << keyBoxPGs.size() << '\n';
		for (std::map<SenKey, ParmGroup>::value_type
			const & keyPG : keyBoxPGs)
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

		return msg.str();
	}

	//! String sampling first several and last few solutions
	inline
	std::string
	stringSolution
		( std::vector<om::FitNdxPair> const & fitIndexPairs
		, std::vector<om::Convention> const & allBoxCons
		, std::size_t const & numBeg = 8u
		, std::size_t const & numEnd = 2u
		)
	{
		std::ostringstream msg;

		using namespace om;

		// display first and last several lines
		msg << '\n';
		msg << infoStringFitConventions
			(fitIndexPairs, allBoxCons, numBeg, numEnd) << '\n';
		msg << "===\n";

		return msg.str();
	}

} // [rpt]



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

	// load interior Box ParmGroups from specified file
	std::ifstream ifsBoxPG(use.theBoxPGPath);
	std::map<om::SenKey, om::ParmGroup>
		const keyBoxPGs{ om::loadParmGroups(ifsBoxPG) };
std::cout << "# keyBoxPGs count: " << keyBoxPGs.size() << '\n';

	// try all internal conventions
	std::vector<om::Convention> const allBoxCons
		{ Convention::allConventions() };
std::cout << "# allBoxCons count: " << allBoxCons.size() << std::endl;


	// load exterior Ind parameter group from specified file
	std::ifstream ifsIndPG(use.theIndPGPath);
	std::map<om::SenKey, om::ParmGroup>
		const keyIndPGs{ om::loadParmGroups(ifsIndPG) };
std::cout << "# keyIndPGs count: " << keyIndPGs.size() << '\n';

	//! Conventions for Ind EO interpretations
	om::ConventionOffset const indConvOffset{ { 1, 1, 1 }, { 0u, 1u, 2u } };
	std::vector<om::Convention> const allIndCons
		{ Convention::allConventionsFor(indConvOffset) };
std::cout << "# allIndCons.size() : " << allIndCons.size() << "\n";

	std::vector<om::OneTrialResult> trialResults;
	std::vector<om::OneSolutionFit> solns;
std::cout << "# Number indEO conventions: " << allIndCons.size() << '\n';
	for (om::Convention const & currIndCon : allIndCons)
	{
		//! Get independent station grouping for current Ind convention
		std::map<SenKey, SenOri> const indKeyStas
			{ om::keyOrisFor(keyIndPGs, currIndCon) };
// std::cout << "got indKeyStas count: " << indKeyStas.size() << '\n';

		std::vector<om::FitNdxPair> fitIndexPairs
			{ fitIndexPairsFor(keyBoxPGs, indKeyStas, allBoxCons) };
// std::cout << "got fitIndexPairs count: " << fitIndexPairs.size() << '\n';

		if (! fitIndexPairs.empty())
		{
			om::OneTrialResult const trialResult
				{ om::trialResultFrom(fitIndexPairs, allBoxCons, currIndCon) };
			trialResults.emplace_back(trialResult);

			double const & fit1st = trialResult.the1st.theFitError;
			double const & fit2nd = trialResult.the2nd.theFitError;
			double const & fitEnd = trialResult.theEnd.theFitError;
			double const promFrac{ (fit2nd - fit1st) / fitEnd };

			if (use.isVerbose())
			{
				using engabra::g3::io::fixed;
				std::cout
					<< "fitError: " << fixed(fit1st, 8u, 6u)
					<< "  boxPGs: " << trialResult.the1st.theBoxCS
					<< "  indPGs: " << trialResult.the1st.theIndCS
					<< "  2ndFit: " << fixed(fit2nd, 8u, 6u)
					<< "  EndFit: " << fixed(fitEnd, 8u, 6u)
					<< "  promFrac: " << fixed(promFrac)
					<< '\n';
				std::cout << std::flush;
			}

if (5u < trialResults.size())
{
	break;
}

		}
		else
		{
			std::cerr << "Error: No results to report\n" << std::endl;
		}
	}

std::cout << "trialResults count: " << trialResults.size() << '\n';

/*

	// report data encountered
	if (use.isVerbose())
	{
		std::cout << rpt::stringInputs(keyBoxPGs, indKeyStas);
		std::cout << rpt::stringSolution(fitIndexPairs, allBoxCons);
	}

	//
	// Results reporting
	//

	// report results
	if (! fitIndexPairs.empty())
	{
		std::size_t const numShow
			{ std::min((std::size_t)5u, (std::size_t)fitIndexPairs.size()) };
		std::cout << '\n';
		std::cout << "Best fitting Conventions\n";
		std::cout << om::infoStringFitConventions
			(fitIndexPairs.begin(), fitIndexPairs.begin()+numShow, allBoxCons)
			<< '\n';
		std::cout << '\n';
	}
	else
	{
		std::cerr << "Error: No results to report\n" << std::endl;
	}
*/

	return 0;
}


