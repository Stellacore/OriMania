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
		std::filesystem::path theOutPath{};

		//! True if verboase output has been requested
		inline
		bool
		isVerbose
			() const
		{
			return true; // TODO could control with command line option
		}

		//! Check invocation arguments.
		explicit
		Usage
			( int argc
			, char * argv[]
			)
		{
			int narg{ 1 };
			if (! (4 == argc))
			{
				std::cerr << '\n' << argv[0] << " Bad invocation:"
					"\nUsage:"
					"\n  <ProgName> <BoxPGPath> <IndPGPath> <OutPath>"
					"\n\n"
					;
			}
			else
			{
				theBoxPGPath = argv[narg++];
				theIndPGPath = argv[narg++];
				theOutPath = argv[narg++];
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

	// try all internal conventions
	std::vector<om::Convention> const allBoxCons
		{ Convention::allConventions() };


	// load exterior Ind parameter group from specified file
	std::ifstream ifsIndPG(use.theIndPGPath);
	std::map<om::SenKey, om::ParmGroup>
		const keyIndPGs{ om::loadParmGroups(ifsIndPG) };

	//! Conventions for Ind EO interpretations
	om::ConventionOffset const indConvOffset{ { 1, 1, 1 }, { 0u, 1u, 2u } };
	std::vector<om::Convention> const allIndCons
		{ Convention::allConventionsFor(indConvOffset) };

	std::vector<om::OneTrialResult> trialResults;
	std::vector<om::OneSolutionFit> solns;

	if (use.isVerbose())
	{
		std::cout << "# keyBoxPGs count: " << keyBoxPGs.size() << '\n';
		std::cout << "# allBoxCons count: " << allBoxCons.size() << std::endl;
		std::cout << "# keyIndPGs count: " << keyIndPGs.size() << '\n';
		std::cout << "# allIndCons.size() : " << allIndCons.size() << "\n";
		std::cout << "# indEO count: " << allIndCons.size() << '\n';
	}
	for (om::Convention const & currIndCon : allIndCons)
	{
		//! Get independent station grouping for current Ind convention
		std::map<SenKey, SenOri> const indKeyStas
			{ om::keyOrisFor(keyIndPGs, currIndCon) };

		std::vector<om::FitNdxPair> fitIndexPairs
			{ fitIndexPairsFor(keyBoxPGs, indKeyStas, allBoxCons) };

		// report data encountered - for debugging
		constexpr bool showIntermediateData{ false };
		if (showIntermediateData)
		{
			std::cout << rpt::stringInputs(keyBoxPGs, indKeyStas);
			std::cout << rpt::stringSolution(fitIndexPairs, allBoxCons);
		}

		// find the best solution for this trial
		if (! fitIndexPairs.empty())
		{
			om::OneTrialResult const trialResult
				{ om::trialResultFrom(fitIndexPairs, allBoxCons, currIndCon) };
			trialResults.emplace_back(trialResult);

			if (use.isVerbose())
			{
				using engabra::g3::io::fixed;
				std::cout << std::setw(4u) << trialResults.size()
					<< ' ' << trialResult.infoString() << '\n';
				std::cout << std::flush; // for watching progress if piped
			}
		}
		else
		{
			std::cerr << "Error: No results to report\n" << std::endl;
		}

		// for dev/testing
		/*
		if (5 < trialResults.size())
		{
			break;
		}
		*/
	}

	//
	// Report results
	//

	// sort overall trial results for reporting
	std::sort(trialResults.begin(), trialResults.end());

	// show results
	std::ofstream ofsOut(use.theOutPath);
	ofsOut << "#\n";
	ofsOut << "# KeyBoxPGs count: " << keyBoxPGs.size() << '\n';
	ofsOut << "# KeyIndPGs count: " << keyIndPGs.size() << '\n';
	ofsOut << "# AllBoxCons count: " << allBoxCons.size() << std::endl;
	ofsOut << "# AllIndCons.size() : " << allIndCons.size() << "\n";
	ofsOut << "# TrialResults count: " << trialResults.size() << '\n';
	ofsOut << "#\n";
	for (om::OneTrialResult const & trialResult : trialResults)
	{
		ofsOut << trialResult << '\n';
	}
	ofsOut << "#\n";

	return 0;
}


