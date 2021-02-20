set(appNavikitInterface_SRCS
#    main.cpp
#    appSocket.cpp
    main_oldversion.cpp
)

IF(CMAKE_BUILD_TYPE MATCHES "release")

add_executable(appNavikitInterface  ${appNavikitInterface_SRCS})

if(WIN32)
target_link_libraries(appNavikitInterface  libxmlrpc libtransfer libcom libjsoncpp libwebinterface libxmlrpc librobot liblog4cpp ${Boost_LIBRARIES})
else()
target_link_libraries(appNavikitInterface  libxmlrpc libtransfer libcom libjsoncpp libwebinterface libxmlrpc librobot liblog4cpp rt ${Boost_LIBRARIES})
endif()

ELSE()

add_executable(appNavikitInterfaced  ${appNavikitInterface_SRCS})
if(WIN32)
target_link_libraries(appNavikitInterfaced  libxmlrpcd libtransferd libcomd libjsoncppd libwebinterfaced libxmlrpcd librobotd liblog4cppd ${Boost_LIBRARIES})
else()
target_link_libraries(appNavikitInterfaced  libxmlrpcd libtransferd libcomd libjsoncppd libwebinterfaced libxmlrpcd librobotd liblog4cppd rt ${Boost_LIBRARIES})
endif()

ENDIF()

