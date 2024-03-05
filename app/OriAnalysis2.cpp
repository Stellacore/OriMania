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


#include "assert.hpp"
#include "io.hpp"
#include "mapkey.hpp"
#include "OriMania.hpp"
#include "Timer.hpp"

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
					"\n  " << argv[0] << " <BoxPGPath> <IndPGPath> <OutPath>"
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

	//! Display information about ErrPairCon
	inline
	std::string
	infoString
		( om::ErrPairCon const & anEPC
		, std::string const & name = {}
		, std::string const & head = "# "
		)
	{
		std::ostringstream oss;
		using namespace om;

		// access Convention::numberEncoding() values
		ConNumId const & boxConNumId = anEPC.second.first;
		ConNumId const & indConNumId = anEPC.second.second;

		// use to construct new Conventions
		Convention const boxCon
			{ Convention::fromNumberEncoding(boxConNumId) };
		Convention const indCon
			{ Convention::fromNumberEncoding(indConNumId) };

		// generate string representations for Conventions
		ConventionString const boxConStr{ ConventionString::from(boxCon) };
		ConventionString const indConStr{ ConventionString::from(indCon) };

		oss << head << name
			<< " " << engabra::g3::io::fixed(anEPC.first)
			<< "  boxPGs: " << boxConStr.stringEncoding()
			<< "  indPGs: " << indConStr.stringEncoding()
			;

		return oss.str();
	}

} // [rpt]


/*! \brief Estimate payload sensor ExCal tranforms by analysing exported data.
 *
 * \arg Load parameter group values associated with Box frame
 * \arg Synthesize all possible Box frame conventions
 * \arg Load parameter group values associated with Ind frame
 * \arg Synthesize possible Box frame conventions (attitude changes only)
 * \arg find combination of Box and Ind conventions that produce best fit
 *
 * Note: This approach is rather inefficience since the same attitude 
 * object instances need to be recomputed over and over (e.g. many thousands
 * of times).
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
	std::map<SenKey, ParmGroup> const boxPGs{ loadParmGroups(ifsBoxPG) };

	// load exterior Ind parameter group from specified file
	std::ifstream ifsIndPG(use.theIndPGPath);
	std::map<SenKey, ParmGroup> const indPGs{ loadParmGroups(ifsIndPG) };

	assertExit(boxPGs.size() == indPGs.size(), "{box,ind}PGs.size() error");
	assertExit(! boxPGs.empty(), "!{box,ind}PGs.empty() error");

	// Use first sensor as reference for Relative Orientations
	SenKey const useSenKey{ boxPGs.begin()->first };

	// Conventions to try for Box frame
	std::vector<ConventionOffset> const boxConOffs
		{ ConventionOffset::allConventions() };
	std::vector<ConventionAngle> const boxConAngs
		{ ConventionAngle::allConventions() };
	std::map<SenKey, std::vector<ConOri> > const boxConOris
		{ conventionOrientationsFor(boxConOffs, boxConAngs, boxPGs) };

	// Conventions to try for Ind frame
	std::vector<ConventionOffset> const indConOffs
		{ ConventionOffset{ThreeSigns{ 1, 1, 1 }, ThreeIndices{ 0, 1, 2 }}
		};
	std::vector<ConventionAngle> const indConAngs
		{ ConventionAngle::allConventions() };
	std::map<SenKey, std::vector<ConOri> > const indConOris
		{ conventionOrientationsFor(indConOffs, indConAngs, indPGs) };

		// Compute relative orientations in both the Box and Ind frames
		std::map<SenKey, std::vector<ConOri> > const boxConROs
			{ conventionROsWrtUseKey(boxConOris, useSenKey) };
		std::map<SenKey, std::vector<ConOri> > const indConROs
			{ conventionROsWrtUseKey(indConOris, useSenKey) };

		// Compare ROs between Box and Ind frames for each sensor

		// list of sensor ROs to compare between Box and Ind frames
		std::set<SenKey> const senKeys
			{ mapkey::commonBetween(boxConROs, indConROs) };

		// for processing remove the sensor used to form the ROs
		// since it will always have identity relative orientation.
		std::set<SenKey> useSenKeys{ senKeys };
		useSenKeys.erase(useSenKey);

		// Comparisons: per sensor {pair(boxCID,indCID), rmse}
		// per sensor: 55296(box) * 1152(ind) = 64M cases
		std::size_t const boxNumCons
			{ 2u * boxConOffs.size() * boxConAngs.size() };
		std::size_t const indNumCons
			{ 2u * indConOffs.size() * indConAngs.size() };
		std::size_t const pairNumCons{ boxNumCons * indNumCons };

		constexpr bool showInfo{ true };
		if (showInfo)
		{
			constexpr char nl{ '\n' };
			std::cout << " boxNumCons: " << boxNumCons << nl;
			std::cout << " indNumCons: " << indNumCons << nl;
			std::cout << "pairNumCons: " << om::commaNumber(pairNumCons) << nl;
			std::size_t const elemSize{ sizeof(ErrPairCon) };
			std::size_t const vecSize{ elemSize * pairNumCons };
			std::cout << nl;
			std::cout << "ErrPairCon:\n";
			std::cout << "   elemSize: " << elemSize << nl;
			std::cout << "    vecSize: " << om::commaNumber(vecSize) << nl;
			std::cout << nl;
		}

		// using PairConId = std::pair<ConNumId, ConNumId>;
		// using ErrPairCon = std::pair<double, PairConId>;
		std::vector<ErrPairCon> maxErrPairCons;
		maxErrPairCons.resize(pairNumCons);

		om::Timer timeRMSEs{ "Time for RMSE computations" };
		om::computeMaxErrors
			(useSenKeys, boxConROs, indConROs, &maxErrPairCons);
		timeRMSEs.stop();

		om::Timer timeSort{ "Time for sorting results" };
		// sort to put smallest errors at front
		std::sort(maxErrPairCons.begin(), maxErrPairCons.end());
		timeSort.stop();

		ErrPairCon const & maxEPCBest = maxErrPairCons.front();
		ErrPairCon const & maxEPCLast = maxErrPairCons.back();

	std::ofstream ofsOut(use.theOutPath);
	{
		constexpr char nl{ '\n' };
		ofsOut << "# " << nl;
		ofsOut << "# " << "Box:\n";
		ofsOut << "# " << infoStringSizes(boxConOris, "boxConOris") << nl;
		ofsOut << "# " << infoStringSizes(boxConROs, " boxConROs") << nl;
		ofsOut << "# " << "Ind:\n";
		ofsOut << "# " << infoStringSizes(indConOris, "indConOris") << nl;
		ofsOut << "# " << infoStringSizes(indConROs, " indConROs") << nl;
		ofsOut << "# " << "Out:\n";
		ofsOut << "# " << "maxErrPairCons: " << maxErrPairCons.size() << nl;

		std::size_t const boxNumOff{ boxConOffs.size() };
		std::size_t const boxNumAng{ boxConAngs.size() };
		std::size_t const boxNumCon{ boxNumOff * boxNumAng };
		std::size_t const boxNumTot{ 2u * boxNumCon };
		std::size_t const indNumOff{ indConOffs.size() };
		std::size_t const indNumAng{ indConAngs.size() };
		std::size_t const indNumCon{ indNumOff * indNumAng };
		std::size_t const indNumTot{ 2u * indNumCon };
		std::size_t const allNumTot{ boxNumTot * indNumTot };

		ofsOut << "# " << nl;
		ofsOut << "# " << "Conventions:\n";
		ofsOut << "# " << "  No. boxOffs: " << boxNumOff << nl;
		ofsOut << "# " << "  No. boxAngs: " << boxNumAng << nl;
		ofsOut << "# " << "  No.     box: " << boxNumCon << nl;
		ofsOut << "# " << "  No.   2xbox: " << boxNumTot << nl;
		ofsOut << "# " << "  No. indOffs: " << indNumOff << nl;
		ofsOut << "# " << "  No. indAngs: " << indNumAng << nl;
		ofsOut << "# " << "  No.     ind: " << indNumCon << nl;
		ofsOut << "# " << "  No.   2xind: " << indNumTot << nl;
		ofsOut << "# " << "  No. all tot: " << om::commaNumber(allNumTot) << nl;

		ofsOut << "# " << nl;
		ofsOut << "# " << timeRMSEs << nl;
		ofsOut << "# " << timeSort << nl;
		ofsOut << "# " << nl;

		ConNumId const & boxBestConNumId = maxEPCBest.second.first;
		ConNumId const & indBestConNumId = maxEPCBest.second.second;
		ofsOut << "# " << nl;
		ofsOut << rpt::infoString(maxEPCBest, "maxEPCBest") << nl;
		ofsOut << rpt::infoString(maxEPCLast, "maxEPCLast") << nl;

		ofsOut << nl;
constexpr std::size_t maxShowSort{ 1000u };
		ofsOut << "# Results - showing only first(best)"
			<< ' ' << maxShowSort
			<< " of " << maxErrPairCons.size()
			<< nl;
		for (std::size_t nn{0u} ; nn < maxShowSort ; ++nn)
		{
			ErrPairCon const & maxErrPairCon = maxErrPairCons[nn];
		//	ofsOut << "# " << maxErrPairCon << '\n';
			ofsOut
				<< rpt::infoString(maxErrPairCon, "", "")
				<< "  "
				<< ' ' << maxErrPairCon.second.first
				<< ' ' << maxErrPairCon.second.second
				<< '\n';
		}
		ofsOut << "# " << std::endl;

	}


	return 0;
}


