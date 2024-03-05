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
\brief Unit tests (and example) speed for OriMania analysis
*/


#include "Analysis.hpp"
#include "Combo.hpp"
#include "Convention.hpp"
#include "io.hpp"
#include "ParmGroup.hpp"
#include "Timer.hpp"

#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>


namespace keys
{
	//! Return keys from map (same sorted order as in map).
	template< typename Key, typename Value >
	inline
	std::set<Key>
	from
		( std::map<Key, Value> const & aMap
		)
	{
		std::set<Key> keys;
		std::transform
			( aMap.cbegin(), aMap.cend()
			, std::inserter(keys, keys.end())
			, [] (typename std::map<Key, Value>::value_type const & pair)
				{ return pair.first; }
			);
		return keys;
	}

	//! Keys in common between both maps
	template< typename Key, typename Value >
	inline
	std::set<Key>
	commonBetween
		( std::map<Key, Value> const & map1
		, std::map<Key, Value> const & map2
		)
	{
		std::set<Key> const keys1{ from(map1) };
		std::set<Key> const keys2{ from(map2) };
		std::set<Key> keysBoth;
		std::set_intersection
			( keys1.cbegin(), keys1.cend()
			, keys2.cbegin(), keys2.cend()
			, std::inserter(keysBoth, keysBoth.end())
			);
		return keysBoth;
	}

	//! Check if both maps have identical keys
	template< typename Key, typename Value >
	inline
	bool
	allMatch
		( std::map<Key, Value> const & map1
		, std::map<Key, Value> const & map2
		)
	{
		bool same{ map1.size() == map2.size() };
		if (same)
		{
			std::set<Key> const keys1{ from(map1) };
			std::set<Key> const keys2{ from(map2) };
			std::set<Key> keysBoth;
			std::set_intersection
				( keys1.cbegin(), keys1.cend()
				, keys2.cbegin(), keys2.cend()
				, std::inserter(keysBoth, keysBoth.end())
				);
			same = (map1.size() == keysBoth.size());
		}
		return same;
	}

} // [keys]

namespace
{
	//! String containing info on map and member vector sizes
	template <typename Key, typename PairType>
	inline
	std::string
	infoStringSizes
		( std::map<Key, std::vector<PairType> > const keyPairs
		, std::string const & name
		)
	{
		std::ostringstream oss;
		oss
			<< name
			<< " NumKeys: " << keyPairs.size()
			<< " VectorSizes: "
			;
		for (typename std::map<Key, std::vector<PairType> >::value_type
			const & keyPair : keyPairs)
		{
			oss << ' ' << keyPair.second.size();
		}
		return oss.str();
	}

} // [anon]

namespace sim
{
	using namespace om;

	static std::string const sFileContentBoxPG
		( "# Simulated ParmGroups in Box Frame"
		  "\n Distances: SimSen1 0. 0. 0."
		  "\n Angles:    SimSen1 .0 .0 .0"
		  "\n Distances: SimSen2 3. 5. 7."
		  "\n Angles:    SimSen2 .25 .5 .75"
		  "\n Distances: SimSen3 .1 .2 .3"
		  "\n Angles:    SimSen3 .7 .6 .5"
		/*
		  "\n Distances: SimSen4 1.1 1.2 1.3"
		  "\n Angles:    SimSen4 .7 .6 .5"
		  "\n Distances: SimSen5 1.1 1.2 1.3"
		  "\n Angles:    SimSen5 .7 .6 .5"
		  "\n Distances: SimSen6 1.1 1.2 1.3"
		  "\n Angles:    SimSen6 .7 .6 .5"
		  "\n Distances: SimSen7 1.1 1.2 1.3"
		  "\n Angles:    SimSen7 .7 .6 .5"
		*/
		);
	/*
	static std::string const sFileContentIndPG
		( "# Simulated ParmGroups in Ind Frame"
		  "\n Distances: SimSen1 0. 0. 0."
		  "\n Angles:    SimSen1 .0 .0 .0"
		  "\n Distances: SimSen2 3. 5. 7."
		  "\n Angles:    SimSen2 .25 .5 .75"
		);
	*/

	//! Simulate parameter group values in Box frame
	inline
	std::map<SenKey, ParmGroup>
	boxPGs
		()
	{
		std::istringstream iss(sFileContentBoxPG);
		return loadParmGroups(iss);
	}

	//! Simulate parameter group values in Box frame
	inline
	std::map<SenKey, ParmGroup>
	indPGs
		()
	{
		// NOTE -- use same box contents for identity transform
		std::string const & sFileContentIndPG = sFileContentBoxPG;
		std::istringstream iss(sFileContentIndPG);
		return loadParmGroups(iss);
	}

} // [sim]


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostringstream & oss
		)
	{
		using namespace om;

		// Load Parameter Groups for two sensors in Box frame
		std::map<SenKey, ParmGroup> const & boxPGs{ sim::boxPGs() };

		// Load Parameter Groups for two sensors in Ind frame
		std::map<SenKey, ParmGroup> const & indPGs{ sim::indPGs() };

		assertExit(keys::allMatch(boxPGs, indPGs));

		// Use this sensor as reference for Relative Orientations
		// SenKey const useSenKey("SimSen2");
		SenKey const useSenKey{ boxPGs.begin()->first };


		// Conventions to try for Box frame
		om::Timer timeBoxEOs{ "Time for Box orientation construction" };
		std::vector<ConventionOffset> const boxConOffs
			{ ConventionOffset::allConventions() };
		std::vector<ConventionAngle> const boxConAngs
			{ ConventionAngle::allConventions() };
		std::map<SenKey, std::vector<ConOri> > const boxConOris
			{ conventionOrientationsFor(boxConOffs, boxConAngs, boxPGs) };
		timeBoxEOs.stop();

		// Conventions to try for Ind frame
		om::Timer timeIndEOs{ "Time for Ind orientation construction" };
//		#define UseAll
		#ifdef UseAll
		// Requires about 74 GB for maxErrPairCons array below
		std::vector<ConventionOffset> const indConOffs
			{ ConventionOffset::allConventions() };
		#else
		std::vector<ConventionOffset> const indConOffs
			{ ConventionOffset{ThreeSigns{ 1, 1, 1 }, ThreeIndices{ 0, 1, 2 }}
			};
		#endif
		std::vector<ConventionAngle> const indConAngs
			{ ConventionAngle::allConventions() };
		std::map<SenKey, std::vector<ConOri> > const indConOris
			{ conventionOrientationsFor(indConOffs, indConAngs, indPGs) };
		timeIndEOs.stop();

		// Compute relative orientations in both the Box and Ind frames
		om::Timer timeROs{ "Time for relative orientations" };
		om::Timer timeBoxROs{ "Time for Box relative orientations" };
		std::map<SenKey, std::vector<ConOri> > const boxConROs
			{ conventionROsWrtUseKey(boxConOris, useSenKey) };
		timeBoxROs.stop();
		om::Timer timeIndROs{ "Time for Ind relative orientations" };
		std::map<SenKey, std::vector<ConOri> > const indConROs
			{ conventionROsWrtUseKey(indConOris, useSenKey) };
		timeIndROs.stop();
		timeROs.stop();

		// Compare ROs between Box and Ind frames for each sensor

		// list of sensor ROs to compare between Box and Ind frames
		std::set<SenKey> const senKeys
			{ keys::commonBetween(boxConROs, indConROs) };

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

		// loop over results: expect:
		// -- near zero err should have same conventions while
		// -- larger errors should have different conventions
		for (ErrPairCon const & maxErrPairCon : maxErrPairCons)
		{
			double const & gotErr = maxErrPairCon.first;
			ConNumId const & cid1 = maxErrPairCon.second.first;
			ConNumId const & cid2 = maxErrPairCon.second.second;
			using namespace engabra::g3;
			constexpr double expErr{ 0. };
			if (cid1 == cid2)
			{
				if (! nearlyEqualsAbs(gotErr, expErr))
				{
					oss << "Failure of cid1==cid2 (zero)error test\n";
					oss << "exp: " << expErr << '\n';
					oss << "got: " << gotErr << '\n';
					oss << "cid1: " << cid1 << '\n';
					oss << "cid2: " << cid2 << '\n';
					break;
				}
			}
			else
			{
				if (  nearlyEqualsAbs(gotErr, expErr))
				{
					oss << "Failure of cid1!=cid2 (large)error test\n";
					oss << "exp: " << expErr << '\n';
					oss << "got: " << gotErr << '\n';
					oss << "cid1: " << cid1 << '\n';
					oss << "cid2: " << cid2 << '\n';
					break;
				}
			}
		}

		// [DoxyExampleTime]

		om::Timer timeSort{ "Time for sorting results" };
		// sort to put smallest errors at front
		std::sort(maxErrPairCons.begin(), maxErrPairCons.end());
		timeSort.stop();

		// [DoxyExampleTime]

		ErrPairCon const & maxEPCBest = maxErrPairCons.front();
		ErrPairCon const & maxEPCLast = maxErrPairCons.back();
		double const & gotErrMin = maxEPCBest.first;
		double const & gotErrMax = maxEPCLast.first;
		if (gotErrMax < gotErrMin)
		{
			using engabra::g3::io::fixed;
			oss << "Failure of (errMin <= errMax) test\n";
			oss << "gotErrMin: " << fixed(gotErrMin) << '\n';
			oss << "gotErrMax: " << fixed(gotErrMax) << '\n';
		}

		constexpr double expErrMin{ 0. }; // for simulated data
		if (! engabra::g3::nearlyEqualsAbs(gotErrMin, expErrMin))
		{
			using engabra::g3::io::fixed;
			oss << "Failure of errMin value test\n";
			oss << "exp: " << fixed(expErrMin) << '\n';
			oss << "got: " << fixed(gotErrMin) << '\n';
		}

		if (showInfo)
		{
			constexpr char nl{ '\n' };
			std::cout << nl;
			std::cout << "Box:\n";
			std::cout << infoStringSizes(boxConOris, "boxConOris") << nl;
			std::cout << infoStringSizes(boxConROs, " boxConROs") << nl;
			std::cout << "Ind:\n";
			std::cout << infoStringSizes(indConOris, "indConOris") << nl;
			std::cout << infoStringSizes(indConROs, " indConROs") << nl;
			std::cout << "Out:\n";
			std::cout << "maxErrPairCons: " << maxErrPairCons.size() << nl;

			std::size_t const boxNumOff{ boxConOffs.size() };
			std::size_t const boxNumAng{ boxConAngs.size() };
			std::size_t const boxNumCon{ boxNumOff * boxNumAng };
			std::size_t const boxNumTot{ 2u * boxNumCon };
			std::size_t const indNumOff{ indConOffs.size() };
			std::size_t const indNumAng{ indConAngs.size() };
			std::size_t const indNumCon{ indNumOff * indNumAng };
			std::size_t const indNumTot{ 2u * indNumCon };
			std::size_t const allNumTot{ boxNumTot * indNumTot };

			std::cout << nl;
			std::cout << "Conventions:\n";
			std::cout << "  No. boxOffs: " << boxNumOff << nl;
			std::cout << "  No. boxAngs: " << boxNumAng << nl;
			std::cout << "  No.     box: " << boxNumCon << nl;
			std::cout << "  No.   2xbox: " << boxNumTot << nl;
			std::cout << "  No. indOffs: " << indNumOff << nl;
			std::cout << "  No. indAngs: " << indNumAng << nl;
			std::cout << "  No.     ind: " << indNumCon << nl;
			std::cout << "  No.   2xind: " << indNumTot << nl;
			std::cout << "  No. all tot: " << om::commaNumber(allNumTot) << nl;

			std::cout << nl;
			std::cout << "maxEPCBest:"
				<< ' ' << engabra::g3::io::fixed(maxEPCBest.first)
				<< ' ' << maxEPCBest.second
				<< nl;
			std::cout << "maxEPCLast:"
				<< ' ' << engabra::g3::io::fixed(maxEPCLast.first)
				<< ' ' << maxEPCLast.second
				<< nl;

			std::cout << nl;
			for (std::size_t nn{0u} ; nn < 7u ; ++nn)
			{
				ErrPairCon const & epc = maxErrPairCons[nn];
				std::cout << "ErrPairCon[" << std::setw(6) << nn << "]:"
					<< ' ' << epc << nl;
			}

			std::cout << nl;
			std::cout << timeBoxEOs << nl;
			std::cout << timeIndEOs << nl;
			std::cout << timeBoxROs << nl;
			std::cout << timeIndROs << nl;
			std::cout << timeROs << nl;
			std::cout << timeRMSEs << nl;
			std::cout << timeSort << nl;
			std::cout << std::endl;
		}


	}

}

//! Check speed of processing
int
main
	()
{
	int status{ 1 };
	std::ostringstream oss;

	test0(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}

