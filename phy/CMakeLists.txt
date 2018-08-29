#################################################################
# Generate communicator_test executable.
#################################################################
include_directories("/usr/local/include")
include_directories("/usr/local/lib")
include_directories("../../communicator/cpp")

link_directories("/usr/local/lib")

#################################################################
# PHY
#################################################################

OPTION(ENABLE_PHY_TX_FILTERING "PHY Tx filtering" ON) # Enabled by default
IF(ENABLE_PHY_TX_FILTERING)
  ADD_DEFINITIONS(-DENABLE_PHY_TX_FILTERING)
ENDIF(ENABLE_PHY_TX_FILTERING)

IF(ENABLE_PHY_TX_FILTERING)
  message(STATUS "   PHY Tx filtering enabled")
  add_executable(phy phy.c helpers.c phy_reception.c phy_transmission.c plot.c trx_filter.c)
ELSE(ENABLE_PHY_TX_FILTERING)
  message(STATUS "   PHY Tx filtering disabled")
  add_executable(phy phy.c helpers.c phy_reception.c phy_transmission.c plot.c)
ENDIF(ENABLE_PHY_TX_FILTERING)

target_link_libraries(phy srslte pthread communicator protobuf zmq m boost_thread)
#target_link_libraries(phy srslte pthread communicator protobuf zmq m boost_thread profiler)

if(RF_FOUND)

else(RF_FOUND)
  add_definitions(-DDISABLE_RF)
endif(RF_FOUND)

find_package(SRSGUI)

if(SRSGUI_FOUND)
  include_directories(${SRSGUI_INCLUDE_DIRS})
  target_link_libraries(phy ${SRSGUI_LIBRARIES})
else(SRSGUI_FOUND)
  add_definitions(-DDISABLE_GRAPHICS)
endif(SRSGUI_FOUND)
