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


#include "Convention.hpp"
#include "io.hpp"
#include "ParmGroup.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <locale>
#include <set>
#include <sstream>


namespace
{
	//! High precision timer
	struct Timer
	{
		std::string theName{};
		std::chrono::time_point<std::chrono::high_resolution_clock>
			theBeg{ std::chrono::high_resolution_clock::now() };
		std::chrono::time_point<std::chrono::high_resolution_clock>
			theEnd{ std::chrono::high_resolution_clock::now() };

		inline
		void
		restart
			()
		{
			theBeg = std::chrono::high_resolution_clock::now();
		}

		inline
		void
		stop
			()
		{
			theEnd = std::chrono::high_resolution_clock::now();
		}

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

	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, Timer const & tmr
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

namespace om
{

	using ConOri = std::pair<Convention, SenOri>;

	using ConRMSE = std::pair<Convention, double>;

	using ErrCon = std::pair<double, Convention>;


	/*! \brief All orientations associated with offset and angle conventions.
	 *
	 * Creates orientations that:
	 * \arg use ParmGroup values
	 * \arg combinatorially combine offsets and angles
	 * \arg include "false" translations associated with order differences
	 *
	 * The OrderTR is addressed by computating an equivalent domain expression
	 * for a translation vector computed from range expression and (inverse)
	 * attitude. E.g. orientations are produced as
	 * \arg <offsetTR, angle> -- for TranRot order where offsetRT is the
	 * offset formed by the ParmGroup and boxConOff convention.
	 * \arg <offsetRT, angle> -- for RotTran order where offsetRT is a
	 * transformed version of offsetTR.
	 */
	inline
	std::vector<ConOri>
	conventionOrientationPairsFor
		( std::vector<ConventionOffset> const & boxConOffs
		, std::vector<ConventionAngle> const & boxConAngs
		, ParmGroup const & parmGroup
		)
	{
		std::vector<ConOri> conOris;
		// combinations of offsets and angles for each of two OrderTR's
		std::size_t const numOris
			{ boxConAngs.size() * boxConOffs.size() * 2u };
		conOris.reserve(numOris);

		// combinatorially evaluate all convention/orientations
		for (ConventionAngle const & boxConAng : boxConAngs)
		{
			// compute the attitude for this angle convention
			// domain: X
			//  range: Y
			rigibra::Attitude const attYwX{ boxConAng.attitudeFor(parmGroup) };
			rigibra::Attitude const attXwY{ inverse(attYwX) };

			for (ConventionOffset const & boxConOff : boxConOffs)
			{
				// compute offset for each order convention
				using namespace engabra::g3;
				Vector const trans{ boxConOff.offsetFor(parmGroup) };

				// Translate then Rotate (offset is in domain, 'X')
				Vector const & tTR = trans;
				SenOri const oriTR{ tTR, attYwX };
				Convention const convTR{ boxConOff, boxConAng, TranRot };

				// Rotate then Translate (offset is in range, 'Y')
				Vector const tRT{ attXwY(trans) };
				SenOri const oriRT{ tRT, attYwX };
				Convention const convRT{ boxConOff, boxConAng, RotTran };

				// Append convention/orientations
				conOris.emplace_back(std::make_pair(convTR, oriTR));
				conOris.emplace_back(std::make_pair(convRT, oriRT));
			}
		}

		return conOris;
	}

	//! Collection of conventionOrientationPairsFor() by sensor key.
	inline
	std::map<SenKey, std::vector<ConOri> >
	conventionOrientationsFor
		( std::vector<ConventionOffset> const & boxConOffs
		, std::vector<ConventionAngle> const & boxConAngs
		, std::map<SenKey, ParmGroup> const & parmGroups
		)
	{
		std::map<SenKey, std::vector<ConOri> > conOris;
		for (std::map<SenKey, ParmGroup>::value_type
			const & parmGroup : parmGroups)
		{
			conOris[parmGroup.first] = conventionOrientationPairsFor
				(boxConOffs, boxConAngs, parmGroup.second);
		}
		return conOris;
	}

	//! Convention and orientations *RELATIVE TO FIRST Sensor* 
	std::map<SenKey, std::vector<ConOri> >
	conventionROsWrtFirst
		( std::map<SenKey, std::vector<ConOri> > const & eoConOris
		, SenKey const & useKey
		)
	{
		std::map<SenKey, std::vector<ConOri> > roSenConOris;
int cnt{ 0 };
		if (! eoConOris.empty())
		{
			// Use first sensor as reference
			std::map<SenKey, std::vector<ConOri> >::const_iterator
				const itUse{ eoConOris.find(useKey) };
std::cout << "### Using key: " << useKey << std::endl;

			std::vector<ConOri> const & oriUses = itUse->second;
			for (std::map<SenKey, std::vector<ConOri> >::const_iterator
				itAny{eoConOris.cbegin()} ; eoConOris.cend() != itAny ; ++itAny)
			{
				SenKey const & senKey = itAny->first;
				std::vector<ConOri> const & oriAnys = itAny->second;
				std::size_t const numOri{ oriUses.size() };
				if (! (numOri == oriAnys.size()))
				{
					std::cerr << "Error in array sizes!" << std::endl;
					exit(10);
				}

std::cout << "### computing key: " << senKey << std::endl;
				std::vector<ConOri> conROs;
				conROs.reserve(numOri);
				for (std::size_t nn{0u} ; nn < numOri ; ++nn)
				{
					Convention const & conv = itUse->second[nn].first;
					SenOri const & oriUseWrtRef = itUse->second[nn].second;
					SenOri const & oriAnyWrtRef = itAny->second[nn].second;
					SenOri const oriRefWrtUse{ inverse(oriUseWrtRef) };
					SenOri const oriAnyWrtUse{ oriAnyWrtRef * oriRefWrtUse };
					ConOri const conRO{ conv, oriAnyWrtUse };
++cnt;
					conROs.emplace_back(conRO);
				}
				roSenConOris.emplace_hint
					( roSenConOris.end()
					, std::make_pair(senKey, conROs)
					);
			}
		}
std::cout << "=== ro cnt: " << cnt << '\n';
		return roSenConOris;
	}

	//! Return keys from map (same sorted order as in map).
	template< typename Key, typename Value >
	inline
	std::set<Key>
	keysFrom
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
	keysInCommon
		( std::map<Key, Value> const & map1
		, std::map<Key, Value> const & map2
		)
	{
		std::set<Key> const keys1{ keysFrom(map1) };
		std::set<Key> const keys2{ keysFrom(map2) };
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
	allKeysMatch
		( std::map<Key, Value> const & map1
		, std::map<Key, Value> const & map2
		)
	{
		bool same{ map1.size() == map2.size() };
		if (same)
		{
			std::set<Key> const keys1{ keysFrom(map1) };
			std::set<Key> const keys2{ keysFrom(map2) };
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

	inline
	void
	assertExit
		( bool const & mustBeTrue
		)
	{
		if (! mustBeTrue)
		{
			exit(18);
		}
	}

	template< typename Key, typename Value >
	inline
	void
	assertMatchingKeys
		( std::map<Key, Value> const & map1
		, std::map<Key, Value> const & map2
		)
	{
		assertExit(allKeysMatch(map1, map2));
	}

static std::size_t rmseCount{ 0u };

	inline
	std::map<SenKey, std::vector<ConRMSE> >
	rmseDifferencesBetween
		( std::map<SenKey, std::vector<ConOri> > const & keyConOri1s
		, std::map<SenKey, std::vector<ConOri> > const & keyConOri2s
		)
	{
		std::map<SenKey, std::vector<ConRMSE> > keyConRMSEs;

		// keys common to both maps
		std::set<SenKey> const senKeys
			{ keysInCommon(keyConOri1s, keyConOri2s) };
		for (SenKey const & senKey : senKeys)
		{
			using Iter = std::map<SenKey, std::vector<ConOri> >::const_iterator;
			Iter const it1{ keyConOri1s.find(senKey) };
			Iter const it2{ keyConOri2s.find(senKey) };
			std::vector<ConOri> const & conOri1s = it1->second;
			std::vector<ConOri> const & conOri2s = it2->second;
			std::size_t const numCons{ conOri1s.size() };
			assertExit(numCons == conOri2s.size());
			std::vector<ConRMSE> conRMSEs;
			conRMSEs.reserve(numCons);
			for (std::size_t nn{0u} ; nn < numCons ; ++nn)
			{
				// access convention data
				Convention const & con1 = conOri1s[nn].first;
				SenOri const & ori1 = conOri1s[nn].second;
				Convention const & con2 = conOri2s[nn].first;
				SenOri const & ori2 = conOri2s[nn].second;
				assertExit(con1 == con2);

				double const rmse{ rmseBasisErrorBetween(ori1, ori2) };
++rmseCount;
				ConRMSE const conRMSE{ con1, rmse };
				conRMSEs.emplace_back(conRMSE);
			}
			keyConRMSEs.emplace_hint
				(keyConRMSEs.end(), std::make_pair(senKey, conRMSEs));
		}

		return keyConRMSEs;
	}

	//! For each convention, find the (sensor with) largest RMSE value.
	inline
	std::vector<ErrCon>
	conventionMaxRMSEs
		( std::map<SenKey, std::vector<ConRMSE> > const & keyConRMSEs
		)
	{
		std::vector<ErrCon> errCons;

		// organize pointers into data structures
		std::vector<SenKey> senKeys;
		std::vector<std::vector<ConRMSE> const *> ptConRMSEs;

		senKeys.reserve(keyConRMSEs.size());
		ptConRMSEs.reserve(keyConRMSEs.size());

		for (std::map<SenKey, std::vector<ConRMSE> >::value_type
			const & keyConRMSE : keyConRMSEs)
		{
			senKeys.emplace_back(keyConRMSE.first);
			ptConRMSEs.emplace_back( &(keyConRMSE.second) );
		}

		std::size_t const numSen{ ptConRMSEs.size() };

		// safety check that all conventions data have same sizes
		assertExit(! ptConRMSEs.empty());
		std::size_t minNumCons{ ptConRMSEs.front()->size() };
		for (std::size_t nSen{1u} ; nSen < numSen ; ++nSen)
		{
			std::size_t const conSize{ ptConRMSEs[nSen]->size() };
			if (conSize < minNumCons)
			{
				minNumCons = conSize;
			}
		}

		errCons.reserve(minNumCons);

		for (std::size_t nCon{0u} ; nCon < minNumCons ; ++nCon)
		{
			Convention const & conv0 = (*(ptConRMSEs[0]))[nCon].first;
			double maxRMSE{ 0. }; // should match ROs with self
			for (std::size_t nSen{0u} ; nSen < numSen ; ++nSen)
			{
				Convention const & conv2 = (*(ptConRMSEs[nSen]))[nCon].first;
				assertExit(conv2 == conv0);
				double const & rmse = (*(ptConRMSEs[nSen]))[nCon].second;
				if (maxRMSE < rmse)
				{
					maxRMSE = rmse;
				}
			}
			errCons.emplace_back( std::make_pair(maxRMSE, conv0));
		}

		return errCons;
	}

	//! String for number using local for separating 1000's grouping
	inline
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

	template <typename PairType>
	inline
	void
	showSizes
		( std::map<SenKey, std::vector<PairType> > const senConOris
		, std::string const & name
		)
	{
		std::cout << name << "<SenKey,vector<ConOri>:sizes:"
			<< " NumSen: " << senConOris.size()
			;
		for (typename std::map<SenKey, std::vector<PairType> >::value_type
			const & senConOri : senConOris)
		{
			std::cout << ' ' << senConOri.second.size();
		}
		std::cout << std::endl;
	}

} // [om]


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

		assertMatchingKeys(boxPGs, indPGs);

		// Use this sensor as reference for Relative Orientations
		// SenKey const useSenKey("SimSen2");
		SenKey const useSenKey{ boxPGs.begin()->first };


		// Conventions to try for Box frame
		Timer timeBoxEOs{ "Time for Box orientation construction" };
		std::vector<ConventionOffset> const boxConOffs
			{ ConventionOffset::allConventions() };
		std::vector<ConventionAngle> const boxConAngs
			{ ConventionAngle::allConventions() };
		std::map<SenKey, std::vector<ConOri> > const boxConOris
			{ conventionOrientationsFor(boxConOffs, boxConAngs, boxPGs) };
		timeBoxEOs.stop();

		// Conventions to try for Ind frame
		Timer timeIndEOs{ "Time for Ind orientation construction" };
		std::vector<ConventionOffset> const indConOffs
			{ ConventionOffset::allConventions() };
		std::vector<ConventionAngle> const indConAngs
			{ ConventionAngle::allConventions() };
		std::map<SenKey, std::vector<ConOri> > const indConOris
			{ conventionOrientationsFor(indConOffs, indConAngs, indPGs) };
		timeIndEOs.stop();

		// Compute relative orientations in both the Box and Ind frames
		Timer timeROs{ "Time for relative orientations" };
		Timer timeBoxROs{ "Time for Box relative orientations" };
		std::map<SenKey, std::vector<ConOri> > const boxConROs
			{ conventionROsWrtFirst(boxConOris, useSenKey) };
		timeBoxROs.stop();
		Timer timeIndROs{ "Time for Ind relative orientations" };
		std::map<SenKey, std::vector<ConOri> > const indConROs
			{ conventionROsWrtFirst(indConOris, useSenKey) };
		timeIndROs.stop();
		timeROs.stop();

showSizes(boxConOris, "boxConOris");
showSizes(indConOris, "indConOris");
showSizes(boxConROs, " boxConROs");
showSizes(indConROs, " indConROs");

//
// TODO - need to use combination of boxCon and indCon (not assume they match)
//

		// Compute RMSE difference statistic between Relative orientations
		Timer timeRMSEs{ "Time for RMSE computations" };
		std::map<SenKey, std::vector<ConRMSE> > const keyConRMSEs
			{ rmseDifferencesBetween(boxConROs, indConROs) };
		timeRMSEs.stop();

		// Determine maximum RMSE errors per convention
		std::vector<ErrCon> errCons
			{ conventionMaxRMSEs(keyConRMSEs) };

		// put smallest errors at front
		std::sort(errCons.begin(), errCons.end());

		double const & gotErrMin = errCons.front().first;
		double const & gotErrMax = errCons.back().first;
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


std::cout << "boxConOris.size: " << boxConOris.size() << '\n';
std::cout << "indConOris.size: " << indConOris.size() << '\n';
std::cout << "rmseCount: " << rmseCount << '\n';

std::size_t const boxNumOff{ boxConOffs.size() };
std::size_t const boxNumAng{ boxConAngs.size() };
std::size_t const boxNumCon{ boxNumOff * boxNumAng };
std::size_t const boxNumTot{ 2u * boxNumCon };
std::size_t const indNumOff{ indConOffs.size() };
std::size_t const indNumAng{ indConAngs.size() };
std::size_t const indNumCon{ indNumOff * indNumAng };
std::size_t const indNumTot{ 2u * indNumCon };
std::size_t const allNumTot{ boxNumTot * indNumTot };

std::cout << "Conventions:\n";
std::cout << "  No. boxOffs: " << boxNumOff << '\n';
std::cout << "  No. boxAngs: " << boxNumAng << '\n';
std::cout << "  No.     box: " << boxNumCon << '\n';
std::cout << "  No.   2xbox: " << boxNumTot << '\n';
std::cout << "  No. indOffs: " << indNumOff << '\n';
std::cout << "  No. indAngs: " << indNumAng << '\n';
std::cout << "  No.     ind: " << indNumCon << '\n';
std::cout << "  No.   2xind: " << indNumTot << '\n';
std::cout << "  No. all tot: " << commaNumber(allNumTot) << '\n';

/*
std::set<SenKey> const senKeys{ keysInCommon(boxConROs, indConROs) };
std::cout << "Common Keys: ";
for (SenKey const & senKey : senKeys)
{
	std::cout << ' ' << senKey;
}
std::cout << std::endl;

for (std::size_t nn{0u} ; nn < 9u ; ++nn)
{
	ErrCon const & errCon = errCons[nn];
	using namespace engabra::g3;
	std::cout
		<< " err/conv:"
		<< ' ' << io::fixed(errCon.first)
		<< ' ' << errCon.second
		<< '\n';
}

std::cout << "Box PGs:\n";
for (std::map<SenKey, ParmGroup>::value_type const & boxPG : boxPGs)
{
	std::cout << boxPG.first << " " << boxPG.second << '\n';
}

std::cout << "Ind PGs:\n";
for (std::map<SenKey, ParmGroup>::value_type const & indPG : indPGs)
{
	std::cout << indPG.first << " " << indPG.second << '\n';
}

std::cout << '\n';
std::cout << "boxConOris: " << boxConOris.size() << '\n';
std::cout << "indConOris: " << indConOris.size() << '\n';
for (std::map<SenKey, std::vector<ConOri> >::value_type
	const & boxConOri : boxConOris)
{
	std::cout << "...:"
		<< " " << boxConOri.first
		<< " " << boxConOri.second.size() << '\n';
}
for (std::map<SenKey, std::vector<ConOri> >::value_type
	const & indConOri : indConOris)
{
	std::cout << "...:"
		<< " " << indConOri.first
		<< " " << indConOri.second.size() << '\n';
	for (std::size_t nn{0u} ; nn < 3u ; ++nn)
	{
		std::cout << "ori:"
			<< "    " << indConOri.second[nn].first << '\n'
			<< "    " << indConOri.second[nn].second << '\n'
			;
	}
}

std::cout << '\n';
std::cout << "boxConROs: " << boxConROs.size() << '\n';
for (std::map<SenKey, std::vector<ConOri> >::value_type
	const & boxConOri : boxConROs)
{
	if (useSenKey == boxConOri.first)
		{ std::cout << "###:"; }
	else
		{ std::cout << "...:"; }
	std::cout
		<< " " << boxConOri.first
		<< " " << boxConOri.second.size() << '\n';
	for (std::size_t nn{0u} ; nn < 3u ; ++nn)
	{
		std::cout << " RO:"
			<< "    " << boxConOri.second[nn].first.numberEncoding()
			<< "    " << boxConOri.second[nn].second << '\n'
			;
	}
}
std::cout << "indConROs: " << indConROs.size() << '\n';
for (std::map<SenKey, std::vector<ConOri> >::value_type
	const & indConOri : indConROs)
{
	if (useSenKey == indConOri.first)
		{ std::cout << "###:"; }
	else
		{ std::cout << "...:"; }
	std::cout
		<< " " << indConOri.first
		<< " " << indConOri.second.size() << '\n';
	for (std::size_t nn{0u} ; nn < 3u ; ++nn)
	{
		std::cout << " RO:"
			<< "    " << indConOri.second[nn].first.numberEncoding()
			<< "    " << indConOri.second[nn].second << '\n'
			;
	}
}


std::cout << "sizeof(ConOri): " << sizeof(ConOri) << '\n';

std::cout << "\nRMSEs:\n";
for (std::map<SenKey, std::vector<ConRMSE> >::value_type
	const & keyConRMSE : keyConRMSEs)
{
	std::vector<ConRMSE> const & conRMSEs = keyConRMSE.second;
	std::cout << keyConRMSE.first << '\n';
	for (std::size_t nn{0u} ; nn < 7u ; ++nn)
	{
		using engabra::g3::io::fixed;
		std::cout
			<< "   rmse:"
			<< " " << fixed(conRMSEs[nn].second)
			<< " " << conRMSEs[nn].first
			<< '\n';
	}
	std::cout << "   rmse: ...\n";
}
*/

std::cout << timeBoxEOs << std::endl;
std::cout << timeIndEOs << std::endl;
std::cout << timeBoxROs << std::endl;
std::cout << timeIndROs << std::endl;
std::cout << timeROs << std::endl;
std::cout << timeRMSEs << std::endl;

std::cout << std::endl;


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

