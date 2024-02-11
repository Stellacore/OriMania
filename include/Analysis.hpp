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
\brief Functions for analyzing orentation relationships.

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
	diffFromIdentity
		( SenOri const & ori
		)
	{
		using namespace engabra::g3;
		double rmse{ null<double>() };
		if (isValid(ori))
		{
			// transform orthogonal basis and sum square resulting differences
			Vector const got1{ ori(e1) };
			Vector const got2{ ori(e1) };
			Vector const got3{ ori(e1) };
			double const eSq1{ magSq(got1 - e1) };
			double const eSq2{ magSq(got2 - e2) };
			double const eSq3{ magSq(got3 - e3) };

			// Statistical degrees of freedom
			constexpr double numComp{ 9. }; // 9 components being compared
			constexpr double numParm{ 6. }; // 3 offsets and three translations
			constexpr double statDof{ numComp - numParm };

			double sse{ (1./statDof) * (eSq1 + eSq2 + eSq3) };
			rmse = std::sqrt(sse);
		}
		return rmse;
	}

	//! Statistic representing error between ori{1,2}. 
	inline
	double
	differenceBetween
		( SenOri const & ori1wX
		, SenOri const & ori2wX
		)
	{
		double rase{ engabra::g3::null<double>() };
		using namespace engabra::g3;
		using namespace rigibra;
		if (isValid(ori1wX) && isValid(ori2wX))
		{
			SenOri const & oriXw1{ inverse(ori1wX) };
			SenOri const ori2w1{ ori2wX * oriXw1 };
			rase = diffFromIdentity(ori2w1);
		}
		return rase;
	}

	/*! \brief Relative orientation between two ParmGroups.
	 *
	 * Each ParmGroup argument is converted to a SenOri using the
	 * same Convention. The two individual orientations are then
	 * combined into a relative orientation of "2" with respect to "1".
	 */
	inline
	SenOri
	relativeOrientationFor
		( ParmGroup const & pg1
		, ParmGroup const & pg2
		, Convention const & convention
		)
	{
		// generate forward transforms internal to black box frame
		SenOri const ori1wB{ convention.transformFor(pg1) };
		SenOri const ori2wB{ convention.transformFor(pg2) };
		// compute relative orientation in black box frame
		SenOri const oriBw1{ inverse(ori1wB) };
		SenOri const roBox{ ori2wB * oriBw1 };
		return roBox;
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
					SenOri const roBox
						{ relativeOrientationFor(pg1, pg2, convention) };
					double const fitError
						{ differenceBetween(roBox, relOri) };
					sumFitErrors[cNdx] += fitError;
				}
			}
		}

		return sumFitErrors;
	}


	//! Pair of (fitErrorValue, ConventionArrayIndex))
	using FitNdxPair = std::pair<double, std::size_t>;

	/*! \brief Convention error values and associated convention index.
	 *
	 * Uses every element of allBoxConventions to transform each of the
	 * two ParmGroup values (both transformed with same convention). For
	 * each resulting pair of orientations (in black box frame) a relative
	 * orientation, roBox, is computed and compared with the provided
	 * independent relative orientation transform, relOri.
	 *
	 * For each convention case, the error between the roBox and roInd
	 * transformations is computed. This fitError value is stored in the
	 * first member of the returned pairs, and the index (of allBoxConventions)
	 * used for the computation is stored in the second member of the pair.
	 * 
	 * Note that the second member of the pair is the same as offset
	 * into the array. This is done to faciliate subsequent sorting
	 * (e.g. simply sort the return array, in order to obtain the best
	 * fit values via:
	 * \arg (returnCollection)[0].first -- is the smallest fit error found
	 * \arg (returnCollection)[0].second -- is the index, ndx, to the member
	 * of allBoxConventions[ndx] that was used to obtain the fit error.
	 */
	inline
	std::vector<FitNdxPair>
	fitIndexPairsFor
		( std::map<SenKey, ParmGroup> const & keyGroups
		, std::map<KeyPair, SenOri> const & keyIndRelOris
		, std::vector<Convention> const & allBoxConventions
		)
	{
		std::vector<FitNdxPair> allFitConPairs;

		// accumulated fit errors, sum for each convention in allBoxConventions
		std::vector<double> const sumFitErrors
			{ sseSumByConvention(keyGroups, keyIndRelOris, allBoxConventions) };

		// normalize the scores by number of ROs
		std::size_t const numRelOris{ keyIndRelOris.size() };
		double const scale{ 1./static_cast<double>(numRelOris) };

		// associate the errors with allBoxConventions collection index
		std::size_t const numFit{ sumFitErrors.size() };
		allFitConPairs.reserve(numFit);
		for (std::size_t nn{0u} ; nn < numFit ; ++nn)
		{
			allFitConPairs.emplace_back
				(std::make_pair(scale*sumFitErrors[nn], nn));
		}

		return allFitConPairs;
	}

	/*! \brief Convention error values and associated convention index.
	 *
	 * The keyIndEOs argument provide exterior orientation (EO) values
	 * for each body of interest. These independent EO values are used
	 * to compute relative orientations, RoInd, for each possible
	 * (non-trivial) combination of the independent EOs.
	 *
	 * Each member of the allConventions collection, is utilized in
	 * conjunction with the keyGroup instances (with the keyGroup's
	 * SenKey matching the respective indKeyOri SenKey values). Together
	 * the ParmGroup and Convention instances are used to create
	 * candidate relative orientations in an assumed black box frame,
	 * i.e. RoBox transform. A goodness of fit metric (Sum-squared-error)
	 * is computed by comparing the RoInd and RoBox transformations.
	 *
	 * The fit error and the associated allConventions index (with
	 * which that fit error is computed) are placed into the return
	 * value collection.
	 *
	 * NOTE: There must be two or more individual EOs in order to
	 *       compare candidate ROs. If not, the return collection
	 *       will be empty.
	 *
	 * Example
	 * \snippet test_Analysis.cpp DoxyExample01
	 */
	inline
	std::vector<FitNdxPair>
	fitIndexPairsFor
		( std::map<SenKey, ParmGroup> const & keyGroups
		, std::map<SenKey, SenOri> const & keyIndEOs
		, std::vector<Convention> const & allBoxConventions
		)
	{
		std::vector<FitNdxPair> fitNdxPairs;

		if (1u < keyIndEOs.size())
		{
			// generate ROs from indepent exterior body orientations
			std::map<KeyPair, SenOri> const keyIndRelOris
				{ relativeOrientationBetweens(keyIndEOs) };

			// core algorithm operating on the relative orientations
			fitNdxPairs = fitIndexPairsFor
				(keyGroups, keyIndRelOris, allBoxConventions);
		}

		return fitNdxPairs;
	}

	/*! \brief TODO
	 */
	/*
	inline
	std::vector<FitNdxPair>
	fitIndexPairsFor
		( std::map<SenKey, ParmGroup> const & keyBoxPGs
		, std::vector<Convention> const & allBoxConventions
		, std::map<SenKey, ParmGroup> const & keyIndPGs
		, std::vector<Convention> const & allIndConventions
		)
	{
		return {};//TODO
	}
	*/

} // [om]


#endif // OriMania_Analysis_INCL_
