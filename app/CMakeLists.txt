#
# MIT License
#
# Copyright (c) 2024 Stellacore Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

##
## -- CMake build system description
##

# ===
# === Application Programs
# ===

set(mainProgs

	OriAnalysis1 # brute force solution for 3 angle sequence conventions
	OriAnalysis2 # brute force solution for 3 angle sequence conventions

	)


foreach(mainProg ${mainProgs})

	add_executable(${mainProg} ${mainProg}.cpp)

	target_compile_options(
		${mainProg}
		PRIVATE
			$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CLANG}>
			$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_GCC}>
			$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_VISUAL}>
		)

	target_include_directories(
		${mainProg}
		PUBLIC
			${CMAKE_CURRENT_SOURCE_DIR}/../include # public interface
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}  # local test code includes
		)

	target_link_libraries(
		${mainProg}
		PRIVATE
			Engabra::Engabra
			Rigibra::Rigibra
			${aProjLib}
		)

endforeach(mainProg)

