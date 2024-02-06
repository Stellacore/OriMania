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

	/*! \brief Sum-squared-errors (SSE) (across all ROs) by each convention.
	 *
	 * For each Convention (from allCons), compute the root average
	 * squared error (RASE) associated with each RO. Sum these per-RO
	 * RASE values into the SSE values for that Convention.
	 *
	 * The return collection contains SSE values in 1:1 correspondence
	 * with the convention cases in allCons.
	 */
	inline
	std::vector<double>
	sseSumByConvention
		( std::map<SenKey, ParmGroup> const & keyGroups
		, std::map<KeyPair, SenOri> const & relKeyOris
		, std::vector<Convention> const & allCons
		)
	{
		// accumulation of fit errors, one for each convention in allCons
		std::vector<double> sumFitErrors(allCons.size(), 0.);

		// compute consistency score vector for each relative orientation
		for (std::map<KeyPair, SenOri>::value_type
			const & relKeyOri : relKeyOris)
		{
			// access data for this RO
			KeyPair const & keyPair = relKeyOri.first;
			SenOri const & relOri = relKeyOri.second;

			// locate parameter groups for the two RO keys
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

				// compute fit scores for all conventions
				for (std::size_t cNdx{0u} ; cNdx < allCons.size() ; ++cNdx)
				{
					Convention const & convention = allCons[cNdx];
					double const fitError
						{ fitErrorFor(pg1, pg2, convention, relOri) };
					sumFitErrors[cNdx] += fitError;
				}
			}
		}

		return sumFitErrors;
	}


	//! Pair of (fitErrorValue, ConventionArrayIndex))
	using FitNdxPair = std::pair<double, std::size_t>;

	//! String with of FitNdxPair data with associated Convention
	std::string
	infoString
		( om::FitNdxPair const & fitConPair
		, std::vector<Convention> const & allConventions
		)
	{
		std::ostringstream oss;
		double const & fitError = fitConPair.first;
		Convention const & convention = allConventions[fitConPair.second];
		using engabra::g3::io::fixed;
		oss
			<< " fitError: " << fixed(fitError)
			<< "  convention: " << convention.asNumber()
			;
		return oss.str();
	}

	/*! \brief Pairs of (fit error, allConventions index) (best at [0]).
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
	inline
	std::vector<FitNdxPair>
	bestFitConventionPairs
		( std::map<SenKey, ParmGroup> const & keyGroups
		, std::map<KeyPair, SenOri> const & relKeyOris
		, std::vector<Convention> const & allCons
		)
	{
		std::vector<FitNdxPair> allFitConPairs;

		// accumulation of fit errors, one for each convention in allCons
		std::vector<double> const sumFitErrors
			{ sseSumByConvention(keyGroups, relKeyOris, allCons) };

		// normalize the scores by number of ROs
		std::size_t const numROs{ relKeyOris.size() };
		double const scale{ 1./static_cast<double>(numROs) };

		// sort results to return best values
		std::size_t const numFit{ sumFitErrors.size() };
		allFitConPairs.reserve(numFit);
		for (std::size_t nn{0u} ; nn < numFit ; ++nn)
		{
			allFitConPairs.emplace_back
				(std::make_pair(scale*sumFitErrors[nn], nn));
		}

		// sort by fit error
		std::sort(allFitConPairs.begin(), allFitConPairs.end());

		return allFitConPairs;
	}


} // [om]


#endif // OriMania_Analysis_INCL_