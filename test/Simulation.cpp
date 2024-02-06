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
\brief Implementation code for OriMania simulation capabilities.
*/


#include "Simulation.hpp"


namespace om
{
namespace sim
{

std::map<om::SenKey, om::SenOri>
boxKeyOris
	( std::map<om::SenKey, om::ParmGroup> const & keyGroups
	, om::Convention const & convention
	)
{
	using namespace om;
	std::map<SenKey, SenOri> keyOris;
	for (std::map<SenKey, ParmGroup>::value_type
		const & keyGroup : keyGroups)
	{
		rigibra::Transform const xSenWrtBox
			{ convention.transformFor(keyGroup.second) };
		keyOris[keyGroup.first] = xSenWrtBox;
	}
	return keyOris;
}

std::map<om::SenKey, om::SenOri>
independentKeyOris
	( std::map<om::SenKey, om::SenOri> const & boxKeyOris
	)
{
	using namespace om;
	std::map<SenKey, SenOri> indKeyOris;
	for (std::map<SenKey, SenOri>::value_type
		const & boxKeyOri : boxKeyOris)
	{
		using namespace rigibra;
		SenKey const & key = boxKeyOri.first;
		SenOri const & oriSenWrtBox = boxKeyOri.second;
		SenOri const & oriBoxWrtRef = om::sim::sXfmBoxWrtRef;
		SenOri const oriSenWrtRef{ oriSenWrtBox * oriBoxWrtRef };
		indKeyOris[key] = oriSenWrtRef;
	}
	return indKeyOris;
}

} // [sim]
} // [om]

