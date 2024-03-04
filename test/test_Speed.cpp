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
	//! If mustBeTrue is not true, put msg to std::cerr and exit().
	inline
	void
	assertExit
		( bool const & mustBeTrue
		, std::string const & msg = {}
		)
	{
		if (! mustBeTrue)
		{
			std::cerr << "Assert Error!\n";
			std::cerr << msg << std::endl;
			exit(18);
		}
	}

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

namespace om
{
	using ConNumId = std::size_t;

	using ConOri = std::pair<ConNumId, SenOri>;

	using PairConId = std::pair<ConNumId, ConNumId>;

	using ErrPairCon = std::pair<double, PairConId>;

} // [om]

namespace
{
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, std::pair<std::size_t, std::size_t> const & pair
		)
	{
		ostrm << pair.first << ' ' << pair.second;
		return ostrm;
	}

	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, om::ErrPairCon const & epc
		)
	{
		using engabra::g3::io::fixed;
		double const & err = epc.first;
		om::PairConId const & pairConId = epc.second;
		ostrm << fixed(err) << "  " << pairConId;
		return ostrm;
	}

} // [anon}

namespace om
{

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
				conOris.emplace_back
					(std::make_pair(convTR.numberEncoding(), oriTR));
				conOris.emplace_back
					(std::make_pair(convRT.numberEncoding(), oriRT));
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
		if (! eoConOris.empty())
		{
			// Use first sensor as reference
			std::map<SenKey, std::vector<ConOri> >::const_iterator
				const itUse{ eoConOris.find(useKey) };

			std::vector<ConOri> const & oriUses = itUse->second;
			for (std::map<SenKey, std::vector<ConOri> >::const_iterator
				itAny{eoConOris.cbegin()} ; eoConOris.cend() != itAny ; ++itAny)
			{
				SenKey const & senKey = itAny->first;
				std::vector<ConOri> const & oriAnys = itAny->second;
				std::size_t const numOri{ oriUses.size() };
				assertExit((numOri == oriAnys.size())
					, "Error in eoConOris array sizes");

				std::vector<ConOri> conROs;
				conROs.reserve(numOri);
				for (std::size_t nn{0u} ; nn < numOri ; ++nn)
				{
					ConNumId const & convNumId = itUse->second[nn].first;
					SenOri const & oriUseWrtRef = itUse->second[nn].second;
					SenOri const & oriAnyWrtRef = itAny->second[nn].second;
					SenOri const oriRefWrtUse{ inverse(oriUseWrtRef) };
					SenOri const oriAnyWrtUse{ oriAnyWrtRef * oriRefWrtUse };
					ConOri const conRO{ convNumId, oriAnyWrtUse };
					conROs.emplace_back(conRO);
				}
				roSenConOris.emplace_hint
					( roSenConOris.end()
					, std::make_pair(senKey, conROs)
					);
			}
		}
		return roSenConOris;
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
//		std::vector<ConventionOffset> const indConOffs
//			{ ConventionOffset::allConventions() };
std::vector<ConventionOffset> const indConOffs
	{ ConventionOffset{ ThreeSigns{ 1, 1, 1 }, ThreeIndices{ 0, 1, 2 } } };
		std::vector<ConventionAngle> const indConAngs
			{ ConventionAngle::allConventions() };
		std::map<SenKey, std::vector<ConOri> > const indConOris
			{ conventionOrientationsFor(indConOffs, indConAngs, indPGs) };
		timeIndEOs.stop();

		// Compute relative orientations in both the Box and Ind frames
		om::Timer timeROs{ "Time for relative orientations" };
		om::Timer timeBoxROs{ "Time for Box relative orientations" };
		std::map<SenKey, std::vector<ConOri> > const boxConROs
			{ conventionROsWrtFirst(boxConOris, useSenKey) };
		timeBoxROs.stop();
		om::Timer timeIndROs{ "Time for Ind relative orientations" };
		std::map<SenKey, std::vector<ConOri> > const indConROs
			{ conventionROsWrtFirst(indConOris, useSenKey) };
		timeIndROs.stop();
		timeROs.stop();

		// Compare ROs between Box and Ind frames for each sensor

		std::set<SenKey> const senKeys
			{ keys::commonBetween(boxConROs, indConROs) };

		std::size_t const boxNumCons
			{ 2u * boxConOffs.size() * boxConAngs.size() };
		std::size_t const indNumCons
			{ 2u * indConOffs.size() * indConAngs.size() };
		std::size_t const pairNumCons{ boxNumCons * indNumCons };
std::cout << " boxNumCons: " << boxNumCons << '\n';
std::cout << " indNumCons: " << indNumCons << '\n';
std::cout << "pairNumCons: " << om::commaNumber(pairNumCons) << '\n';
		
		// using PairConId = std::pair<ConNumId, ConNumId>;
		// using ErrPairCon = std::pair<double, PairConId>;
		std::vector<ErrPairCon> maxErrPairCons;
		maxErrPairCons.resize(pairNumCons);
		ErrPairCon const zeroErrPairCon{ 0., { 0u, 0u } };
		std::fill(maxErrPairCons.begin(), maxErrPairCons.end(), zeroErrPairCon);

		// Comparisons: per sensor {pair(boxCID,indCID), rmse}
		// per sensor: 55296(box) * 1152(ind) = 64M cases

std::size_t rmseCount{ 0u };
		om::Timer timeRMSEs{ "Time for RMSE computations" };
		bool firstPass{ true };
		for (std::set<SenKey>::const_iterator itKey{senKeys.cbegin()}
			; senKeys.cend() != itKey ; ++itKey)
		{
			std::size_t ndxEPCs{ 0u };
			SenKey const & senKey = *itKey;

			// skip the sensor used to define RO base since all these
			// RO values should be equal to identity (within numeric tolerance)
			if (senKey == useSenKey)
			{
				continue;
			}

			using ItRO = std::map<SenKey, std::vector<ConOri> >::const_iterator;
			ItRO const itBoxRO{ boxConROs.find(senKey) };
			std::vector<ConOri> const & boxConOris = itBoxRO->second;

std::cout << "Key:" << ' ' << senKey << ' ' << senKey << std::endl;

			ItRO const itIndRO{ indConROs.find(senKey) };
			std::vector<ConOri> const & indConOris = itIndRO->second;

			// Loop over all box conventions (e.g. up to 55296)
			// E.g. 55296 cases for full orientation convention coverage
std::cout << "boxConOris.size: " << boxConOris.size() << '\n';
std::cout << "indConOris.size: " << indConOris.size() << '\n';
			for (ConOri const & boxConOri : boxConOris)
			{
				ConNumId const & boxConId = boxConOri.first;
				SenOri const & boxRO = boxConOri.second;

				// Loop over all ind conventions
				// Up to 55296 for full convention, or 1152 if for angle only
				for (ConOri const & indConOri : indConOris)
				{
					ConNumId const & indConId = indConOri.first;
					SenOri const & indRO = indConOri.second;

					// compute goodness of git for this sensor
					double const rmse
						{ rmseBasisErrorBetween2(boxRO, indRO) };
++rmseCount;
					PairConId const currPairConId{ boxConId, indConId };

					// ErrPairCon = std::pair<double, PairConId>;
					ErrPairCon & maxErrPairCon = maxErrPairCons[ndxEPCs++];
					double & maxRMSE = maxErrPairCon.first;
					PairConId & savePairConId = maxErrPairCon.second;
					if (firstPass)
					{
						maxRMSE = rmse;
						savePairConId = currPairConId;
					}
					else
					{
						maxRMSE = std::max(rmse, maxRMSE);
						assertExit
							((savePairConId == currPairConId), "pairConId");
					}
				}
			}

			firstPass = false;

		} // senKey
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


std::cout << "sorting" << std::endl;

		// sort to put smallest errors at front

		// [DoxyExampleTime]

		om::Timer timeSort{ "Time for sorting results" };
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

std::cout << '\n';
std::cout << "Box:\n";
std::cout << infoStringSizes(boxConOris, "boxConOris") << '\n';
std::cout << infoStringSizes(boxConROs, " boxConROs") << '\n';
std::cout << "Ind:\n";
std::cout << infoStringSizes(indConOris, "indConOris") << '\n';
std::cout << infoStringSizes(indConROs, " indConROs") << '\n';

std::size_t const boxNumOff{ boxConOffs.size() };
std::size_t const boxNumAng{ boxConAngs.size() };
std::size_t const boxNumCon{ boxNumOff * boxNumAng };
std::size_t const boxNumTot{ 2u * boxNumCon };
std::size_t const indNumOff{ indConOffs.size() };
std::size_t const indNumAng{ indConAngs.size() };
std::size_t const indNumCon{ indNumOff * indNumAng };
std::size_t const indNumTot{ 2u * indNumCon };
std::size_t const allNumTot{ boxNumTot * indNumTot };

std::cout << '\n';
std::cout << "Conventions:\n";
std::cout << "  No. boxOffs: " << boxNumOff << '\n';
std::cout << "  No. boxAngs: " << boxNumAng << '\n';
std::cout << "  No.     box: " << boxNumCon << '\n';
std::cout << "  No.   2xbox: " << boxNumTot << '\n';
std::cout << "  No. indOffs: " << indNumOff << '\n';
std::cout << "  No. indAngs: " << indNumAng << '\n';
std::cout << "  No.     ind: " << indNumCon << '\n';
std::cout << "  No.   2xind: " << indNumTot << '\n';
std::cout << "  No. all tot: " << om::commaNumber(allNumTot) << '\n';
std::cout << "    rmseCount: " << om::commaNumber(rmseCount) << '\n';

std::cout << '\n';
std::cout << "maxEPCBest:"
	<< ' ' << engabra::g3::io::fixed(maxEPCBest.first)
	<< ' ' << maxEPCBest.second
	<< '\n';
std::cout << "maxEPCLast:"
	<< ' ' << engabra::g3::io::fixed(maxEPCLast.first)
	<< ' ' << maxEPCLast.second
	<< '\n';

		std::cout << '\n';
		for (std::size_t nn{0u} ; nn < 7u ; ++nn)
		{
			ErrPairCon const & epc = maxErrPairCons[nn];
			std::cout << "ErrPairCon[" << std::setw(6) << nn << "]:"
				<< ' ' << epc << '\n';
		}


std::cout << timeBoxEOs << std::endl;
std::cout << timeIndEOs << std::endl;
std::cout << timeBoxROs << std::endl;
std::cout << timeIndROs << std::endl;
std::cout << timeROs << std::endl;
std::cout << timeRMSEs << std::endl;
std::cout << timeSort << std::endl;
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

