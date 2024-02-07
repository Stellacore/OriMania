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


#ifndef OriMania_sim_INCL_
#define OriMania_sim_INCL_

/*! \file
\brief Contains ######

Example:
\snippet test_sim.cpp DoxyExample01

*/


#include "Orientation.hpp"
#include "Convention.hpp"

#include <Rigibra>

#include <map>



namespace om
{

/*! \brief Functions and utilities for simulating orientation data sets.
 *
 */
namespace sim
{

	using PG = om::ParmGroup;
	//! A diverse selection of angle and distance parameters
	static std::map<om::SenKey, om::ParmGroup> const sKeyGroups
		{ { "pg0", PG{ {    .0,    .0,    .0 }, {  .000,  .000,  .000,} } }
		, { "pg1", PG{ { -60.1,  10.3,  21.1 }, {  .617, -.113, -.229 } } }
		, { "pg2", PG{ {  10.7, -60.7,  31.1 }, { -.127,  .619, -.317 } } }
		, { "pg3", PG{ {  30.7,  22.7, -61.3 }, { -.331, -.631,  .239 } } }
		, { "pg4", PG{ {  10.1, -40.9, -50.3 }, { -.109,  .421,  .523 } } }
		, { "pg5", PG{ { -41.9,  22.3, -52.1 }, {  .431, -.233,  .541 } } }
		, { "pg6", PG{ { -40.1, -50.9,  31.3 }, {  .433,  .547, -.337 } } }
		};

	// TODO - run test over many (all?) different conventions.
	//! An arbitrarily set convention
	static om::Convention const sConventionA
		{ {  1,  1, -1 }
		, { 1, 0, 2 }
		, {  1, -1,  1 }
		, { 0, 1, 2 }
		, { 1, 2, 1 }
		, om::RotTran
		};

	//! An arbitrary orientation for Box frame w.r.t. arbitrary Ref frame.
	static rigibra::Transform const sXfmBoxWrtRef
		{ rigibra::Location{ 1000., 2000., 3000. }
		, rigibra::Attitude(rigibra::PhysAngle{ -.7, 1.5, 3. })
		};


	/*! \brief Simulate orientation of sensors wrt black box frame.
	 *
	 * uses
	 */
	std::map<om::SenKey, om::SenOri>
	boxKeyOris
		( std::map<om::SenKey, om::ParmGroup> const & keyGroups
		, om::Convention const & convention
		);

	/*! \brief Simulate export of the body orientation data in Ind frame.
	 *
	 * The boxKeyOris are assumed relative to some arbitary and unknown
	 * "black box" (Box) reference frame. This function applies the
	 * orienation oriBoxWrtRef to each of the input boxKeyOri orientations
	 * and returns the resulting sensor orientions with respect to the
	 * indepndent (Ind) frame (e.g. SenWrtInd).
	 */
	std::map<om::SenKey, om::SenOri>
	independentKeyOris
		( std::map<om::SenKey, om::SenOri> const & boxKeyOris
		, SenOri const & oriBoxWrtRef = om::sim::sXfmBoxWrtRef
		);

} // [sim]
} // [om]


#endif // OriMania_sim_INCL_
