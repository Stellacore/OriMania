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


#ifndef OriMania_Combo_INCL_
#define OriMania_Combo_INCL_

/*! \file
\brief Functions associated with combinations of orientations and conventions.

*/
/*
Example:
\snippet test_Combo.cpp DoxyExample01
*/


#include "Convention.hpp"
#include "Key.hpp"
#include "Orientation.hpp"
#include "ParmGroup.hpp"

#include <map>
#include <vector>


namespace om
{
	//! Convention and associated 3D Orientation>
	using ConOri = std::pair<ConNumId, SenOri>;


	/*! \brief Convention and orientations *RELATIVE TO 'USE'* sensor.
	 *
	 * For each SenKey, the vector of input orientation data (e.g.
	 * Sensor orientations with respect to (wrt) 'X', is converted
	 * into a vector of relative orientations - orientation wrt
	 * frame 'U' where the 'U' frame is defined as the sensor frame
	 * associated with sensor \a useKey.
	 *
	 * For example If the input vector<ConOri> contains orientations,
	 * \arg Orientation Sen1 wrt 'X'
	 * \arg Orientation Sen2 wrt 'X'
	 * \arg Orientation Sen3 wrt 'X'
	 * \arg ...
	 *
	 * and the useKey is "Sen2", then the return orientations will be
	 * \arg Orientation Sen1wrt2
	 * \arg Orientation Sen2wrt2 == Identity
	 * \arg Orientation Sen3wrt2
	 * \arg ...
	 *
	 * The Convention values in the returned vector<ConOri> items
	 * are copied from input to output without change.
	 */
	std::map<SenKey, std::vector<ConOri> >
	conventionROsWrtUseKey
		( std::map<SenKey, std::vector<ConOri> > const & eoConOris
		, SenKey const & useKey
		)
	{
		std::map<SenKey, std::vector<ConOri> > roSenConOris;
		if (! eoConOris.empty())
		{
			// find 'use' sensor to use as base for all relative orientations.
			std::map<SenKey, std::vector<ConOri> >::const_iterator
				const itBase{ eoConOris.find(useKey) };

			// Process each relative orientation in turn
			// Includes processing of base sensor (into identity ROs)
			std::vector<ConOri> const & oriBases = itBase->second;
			for (std::map<SenKey, std::vector<ConOri> >::const_iterator
				  itFree{eoConOris.cbegin()}
				; eoConOris.cend() != itFree
				; ++itFree)
			{
				SenKey const & senKey = itFree->first;
				std::vector<ConOri> const & oriFrees = itFree->second;

				// enforce that data sizes are compatible
				std::size_t const numOri{ oriBases.size() };
				if (! (numOri == oriFrees.size()))
				{
					std::cerr << "\nFATAL! Error in eoConOris sizes!\n\n";
					std::cerr << "oriBases: " << oriBases.size() << '\n';
					std::cerr << "oriFrees: " << oriFrees.size() << '\n';
					roSenConOris.clear();
					break;
				}

				// Compute RO geometry for each item in return vector<ConOri>
				std::vector<ConOri> conROs;
				conROs.reserve(numOri);
				for (std::size_t nn{0u} ; nn < numOri ; ++nn)
				{
					ConNumId const & convNumId = itBase->second[nn].first;
					SenOri const & oriBaseWrtRef = itBase->second[nn].second;
					SenOri const & oriFreeWrtRef = itFree->second[nn].second;
					SenOri const oriRefWrtBase{ inverse(oriBaseWrtRef) };
					SenOri const oriFreeWrtBase{ oriFreeWrtRef*oriRefWrtBase };
					ConOri const conRO{ convNumId, oriFreeWrtBase };
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
	 * offset formed by the ParmGroup and conOff convention.
	 * \arg <offsetRT, angle> -- for RotTran order where offsetRT is a
	 * transformed version of offsetTR.
	 */
	inline
	std::vector<ConOri>
	conventionOrientationPairsFor
		( std::vector<ConventionOffset> const & conOffs
		, std::vector<ConventionAngle> const & conAngs
		, ParmGroup const & parmGroup
		)
	{
		std::vector<ConOri> conOris;
		// combinations of offsets and angles for each of two OrderTR's
		std::size_t const numOris
			{ conAngs.size() * conOffs.size() * 2u };
		conOris.reserve(numOris);

		// combinatorially evaluate all convention/orientations
		for (ConventionAngle const & conAng : conAngs)
		{
			// compute the attitude for this angle convention
			// domain: X
			//  range: Y
			rigibra::Attitude const attYwX{ conAng.attitudeFor(parmGroup) };
			rigibra::Attitude const attXwY{ inverse(attYwX) };

			for (ConventionOffset const & conOff : conOffs)
			{
				// compute offset for each order convention
				using namespace engabra::g3;
				Vector const trans{ conOff.offsetFor(parmGroup) };

				// Translate then Rotate (offset is in domain, 'X')
				Vector const & tTR = trans;
				SenOri const oriTR{ tTR, attYwX };
				Convention const convTR{ conOff, conAng, TranRot };

				// Rotate then Translate (offset is in range, 'Y')
				Vector const tRT{ attXwY(trans) };
				SenOri const oriRT{ tRT, attYwX };
				Convention const convRT{ conOff, conAng, RotTran };

				// Append convention/orientations
				conOris.emplace_back
					(std::make_pair(convTR.numberEncoding(), oriTR));
				conOris.emplace_back
					(std::make_pair(convRT.numberEncoding(), oriRT));
			}
		}

		return conOris;
	}

	/*! \brief Collection of conventionOrientationPairsFor() by sensor key.
	 *
	 * Return map includes:
	 * \arg Key: Sensor Key - to which sensor orientations pertain
	 * \arg Value: vector of ConOri - one element for each input convention.
	 *
	 * Note that ConOri is std::pair<ConNumId, SenOri> where ConNumId
	 * is the value from Convention::numberEncoding(), and SenOri is
	 * the 3D orientation (both location and attitude) that is generated
	 * from the ParmGroup for this Sensor Key (from the \a parmGroups
	 * input arguments.
	 * 
	 * The two convention arguments (conOffs, conAngs) are combined
	 * in all possible combined combintations. There is one data item
	 * in the returned vector<ConOri> (value of map) for each of these
	 * combinatorial conventions.
	 */
	inline
	std::map<SenKey, std::vector<ConOri> >
	conventionOrientationsFor
		( std::vector<ConventionOffset> const & conOffs
		, std::vector<ConventionAngle> const & conAngs
		, std::map<SenKey, ParmGroup> const & parmGroups
		)
	{
		std::map<SenKey, std::vector<ConOri> > conOris;
		for (std::map<SenKey, ParmGroup>::value_type
			const & parmGroup : parmGroups)
		{
			conOris[parmGroup.first] = conventionOrientationPairsFor
				(conOffs, conAngs, parmGroup.second);
		}
		return conOris;
	}


} // [om]


#endif // OriMania_Combo_INCL_
