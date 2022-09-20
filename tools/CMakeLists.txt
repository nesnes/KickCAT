set(EXAMPLE_SRCS debugExample.cc)

add_executable(debugExample ${EXAMPLE_SRCS} ${OS_SRCS})
target_link_libraries(debugExample kickcat)
set_target_properties(debugExample PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    POSITION_INDEPENDENT_CODE ON
)

set(EXAMPLE_SRCS scanTopology.cc)

add_executable(scanTopology ${EXAMPLE_SRCS} ${OS_SRCS})
target_link_libraries(scanTopology kickcat)
set_target_properties(scanTopology PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    POSITION_INDEPENDENT_CODE ON
)

set(EXAMPLE_SRCS ./moveElmo/moveElmo.cc)

add_executable(moveElmo ${EXAMPLE_SRCS} ${OS_SRCS})
target_link_libraries(moveElmo kickcat)
set_target_properties(moveElmo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    POSITION_INDEPENDENT_CODE ON
)

set(EXAMPLE_SRCS ./moveElmo/detectCRC.cc)

add_executable(detectCRC ${EXAMPLE_SRCS} ${OS_SRCS})
target_link_libraries(detectCRC kickcat)
set_target_properties(detectCRC PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    POSITION_INDEPENDENT_CODE ON
)

set(EXAMPLE_SRCS ./moveElmo/detectEMG.cc)

add_executable(detectEMG ${EXAMPLE_SRCS} ${OS_SRCS})
target_link_libraries(detectEMG kickcat)
set_target_properties(detectEMG PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    POSITION_INDEPENDENT_CODE ON
)


