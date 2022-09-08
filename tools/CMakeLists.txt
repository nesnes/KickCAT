set(EXAMPLE_SRCS debugExample.cc)

add_executable(debugExample ${EXAMPLE_SRCS} ${OS_SRCS})
target_link_libraries(debugExample kickcat)
set_target_properties(debugExample PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    POSITION_INDEPENDENT_CODE ON
)
