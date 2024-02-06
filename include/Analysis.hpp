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


#ifndef OriMania_Analysis_INCL_
#define OriMania_Analysis_INCL_

/*! \file
\brief Contains Functions for analyzing orentation relationships.

Example:
\snippet test_Analysis.cpp DoxyExample01

*/


#include "Convention.hpp"
#include "Orientation.hpp"

#include <Engabra>
#include <Rigibra>

#include <map>
#include <vector>



namespace om
{

	//! Statistic representing error between ori{1,2}. 
	inline
	double
	differenceBetween
		( SenOri const & ori1
		, SenOri const & ori2
		)
	{
		// Location
		using namespace engabra::g3;
		Vector const & loc1 = ori1.theLoc;
		Vector const & loc2 = ori2.theLoc;

		// compute weight for locations
		double const aveMag{ .5 * (magnitude(loc1) + magnitude(loc2)) };
		double wLoc{ 1. };
		if (1. < aveMag)
		{
			wLoc = (1. / aveMag);
		}

		// weighted squared location residuals
		double const residSqLoc{ (wLoc / 3.) * magSq(loc2 - loc1) };

		// Attitude
		using namespace rigibra;
		Attitude const & att1 = ori1.theAtt;
		Attitude const & att2 = ori2.theAtt;

		// weight for attitudes
		constexpr double wAtt{ 1. };

		// weighted squared angle residuals
		BiVector const biv1{ att1.spinAngle().theBiv };
		BiVector const biv2{ att2.spinAngle().theBiv };
		double const residSqAtt{ (wAtt / 3.) * magSq(biv2 - biv1) };

		double const rase{ std::sqrt(.5 * (residSqLoc + residSqAtt)) };
		return rase;
	}

	//! Error between roInd and black box RO computed from pg{1,2}.
	inline
	double
	fitErrorFor
		( ParmGroup const & pg1
		, ParmGroup const & pg2
		, Convention const & convention
		, SenOri const & roInd
		)
	{
		// generate forward transforms internal to black box frame
		SenOri const ori1wB{ convention.transformFor(pg1) };
		SenOri const ori2wB{ convention.transformFor(pg2) };
		// compute relative orientation in black box frame
		SenOri const oriBw1{ inverse(ori1wB) };
		SenOri const roBox{ ori2wB * oriBw1 };
		// compare black box and independent relative orientations
		return differenceBetween(roBox, roInd);
	}

	//! Pair of (fitErrorValue, ConventionArrayIndex))
	using FitNdxPair = std::pair<double, std::size_t>;

	/*! \brief Fit error and allConventions index (ordered best[0] to worst).
	 *
	 * Uses every element of allConventions to transform each of the
	 * two ParmGroup values (both transformed with same convention). For
	 * each resulting pair of orientations (in black box frame) a relative
	 * orientation, roBox, is computed and compared with the provided
	 * independent relative orientation transform, relOri.
	 *
	 * For each convention case, the error between the roBox and roInd
	 * transformations is computed. This fitError value is stored in the
	 * first member of the returned pairs, and the index (of allConventions)
	 * used for the computation is stored in the second member of the pair.
	 * 
	 * Note that the second member of the pair is the same as offset
	 * into the array. This is done to faciliate subsequent sorting
	 * (e.g. simply sort the return array, in order to obtain the best
	 * fit values via:
	 * \arg (returnCollection)[0].first -- is the smallest fit error found
	 * \arg (returnCollection)[0].second -- is the index, ndx, to the member
	 * of allConventions[ndx] that was used to obtain the fit error.
	 */
	std::vector<FitNdxPair>
	fitConventions
		( ParmGroup const & pg1
		, ParmGroup const & pg2
		, std::vector<Convention> const & allConventions
		, SenOri const & roInd
		)
	{
		std::vector<FitNdxPair> fitConventionPairs;
		for (std::size_t cNdx{0u} ; cNdx < allConventions.size() ; ++cNdx)
		{
			Convention const & convention = allConventions[cNdx];
			double const fitError
				{ fitErrorFor(pg1, pg2, convention, roInd) };
			fitConventionPairs.emplace_back(std::make_pair(fitError, cNdx));
		}
		return fitConventionPairs;
	}


	/*! \brief TODO
	 *
	 *
	 *
	 */
	inline
	std::vector<FitNdxPair>
	fitConventionPairs
		( std::map<SenKey, ParmGroup> const & keyGroups
		, std::map<KeyPair, SenOri> const & relKeyOris
		, std::vector<Convention> const & allCons
		)
	{
		std::vector<FitNdxPair> allFitConPairs;

		// compute consistency score vector for each relative orientation
		for (std::map<KeyPair, SenOri>::value_type
			const & relKeyOri : relKeyOris)
		{
			KeyPair const & keyPair = relKeyOri.first;
			SenOri const & relOri = relKeyOri.second;

			// locate parameter groups for these two keys
			std::map<SenKey, ParmGroup>::const_iterator
				const itFind1{ keyGroups.find(keyPair.key1()) };
			std::map<SenKey, ParmGroup>::const_iterator
				const itFind2{ keyGroups.find(keyPair.key2()) };
			if ( (keyGroups.end() != itFind1)
			  && (keyGroups.end() != itFind1)
			   )
			{
				ParmGroup const & pg1 = itFind1->second;
				ParmGroup const & pg2 = itFind2->second;
				std::vector<FitNdxPair> fitConPairs
					{ fitConventions(pg1, pg2, allCons, relOri) };
// TODO
std::sort(fitConPairs.begin(), fitConPairs.end());
allFitConPairs = fitConPairs;
break;
// TODO
			}

//TODO			allFitConPairs.emplace_back();
		}
		return allFitConPairs;
	}


} // [om]


#endif // OriMania_Analysis_INCL_
