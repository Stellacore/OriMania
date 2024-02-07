##
## -- CMake build system description
##

# ===
# === Library Components
# ===

set(srcFiles

	OriMania.cpp

	)

message("### aProjLib: ${aProjLib}")

add_library(
	${aProjLib}
	STATIC
		${srcFiles}
	)

target_compile_options(
	${aProjLib}
	PRIVATE
		$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CLANG}>
		$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_GCC}>
		$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_VISUAL}>
	)

target_include_directories(
	${aProjLib}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
		$<INSTALL_INTERFACE:include/${OriManiaProjName}>
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/ # local source headers
	)

target_link_libraries(
	${aProjLib}
	PRIVATE
		Engabra::Engabra
		Rigibra::Rigibra
	)

