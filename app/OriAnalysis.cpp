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


#include "OriMania.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <vector>


namespace
{
	//! Check basic application usage
	struct Usage
	{
		std::filesystem::path theIndEoPath{};

		//! Check invocation arguments.
		explicit
		Usage
			( int argc
			, char * argv[]
			)
		{
			int narg{ 1 };
			if (narg < argc)
			{
				theIndEoPath = argv[narg++];
			}
			else
			{
				std::cerr << '\n' << argv[0] << " Bad invocation:"
					"\nUsage:"
					"\n  <ProgName> <IndEoFile>"
					"\n\n"
					;
			}
		}

		//! True if input file path is set to existing file.
		inline
		bool
		isValid
			() const
		{
			return std::filesystem::exists(theIndEoPath);
		}

	}; // Usage

} // [anon]


/*! \brief Estimate payload sensor ExCal tranforms by analysing exported data.
 *
 * \arg Load independent EO's via om::loadIndEOs().
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

	std::ifstream ifs(use.theIndEoPath);
	std::map<SenKey, SenOri> const indKeyOris{ loadIndEOs(ifs) };

	std::cout << "num indEO sensors: " << indKeyOris.size() << '\n';

//TODO - need function to load these from file
	using PG = om::ParmGroup;
	//! A diverse selection of angle and distance parameters
	static std::map<om::SenKey, om::ParmGroup> const keyGroups
		{ { "pg0", PG{ {    .0,    .0,    .0 }, {  .000,  .000,  .000,} } }
		, { "pg1", PG{ { -60.1,  10.3,  21.1 }, {  .617, -.113, -.229 } } }
		, { "pg2", PG{ {  10.7, -60.7,  31.1 }, { -.127,  .619, -.317 } } }
		, { "pg3", PG{ {  30.7,  22.7, -61.3 }, { -.331, -.631,  .239 } } }
		, { "pg4", PG{ {  10.1, -40.9, -50.3 }, { -.109,  .421,  .523 } } }
		, { "pg5", PG{ { -41.9,  22.3, -52.1 }, {  .431, -.233,  .541 } } }
		, { "pg6", PG{ { -40.1, -50.9,  31.3 }, {  .433,  .547, -.337 } } }
		};

	std::vector<om::Convention> const allCons{ Convention::allConventions() };
	std::vector<om::FitNdxPair> const fitIndexPairs
		{ fitIndexPairsFor(keyGroups, indKeyOris, allCons) };

	return 0;
}


