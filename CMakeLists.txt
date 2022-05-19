project(myTurnnel)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PUBLIC
        alphaBianjie.cpp
        getBoundaryEstimation.cpp
        getCentralAxis.cpp
		getCentroid.cpp
		getCylinder.cpp
		getDifferenceIn2Cloud.cpp
		RegionGrowAlgthom.cpp
		EuclideanClusteringAlgrothm.cpp
		myKMeans.cpp
		testMain.cpp
        )
find_package(PCL REQUIRED)
#include
target_include_directories(${PROJECT_NAME}
        PUBLIC
		myKMeans.h
        ${PCL_INCLUDE_DIRS} )
#link
target_link_directories(${PROJECT_NAME}
    PUBLIC
		myKMeans.h
        ${PCL_LIBRARY_DIRS}
        )
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        )
