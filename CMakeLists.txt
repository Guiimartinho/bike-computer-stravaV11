cmake_minimum_required (VERSION 3.5.1)

project (stravaV10 LANGUAGES C CXX)

set(CMAKE_COLOR_MAKEFILE                        OFF)

add_definitions(-DTDD)
add_definitions(-D__HEAP_SIZE=150000)
add_definitions(-D_USE_MATH_DEFINES)
#add_definitions(-DLOC_SOURCE_GPS)
#add_definitions(-DTDD_RANDOMIZE)
add_definitions(-DUSB_ENABLED)
add_definitions(-DBLE_STACK_SUPPORT_REQD)
#add_definitions(-DLS027_GUI)

if(CMAKE_HOST_UNIX)


option(CODE_COVERAGE "Enable coverage reporting" ON)
if(CODE_COVERAGE)

  include(CodeCoverage.cmake)

  APPEND_COVERAGE_COMPILER_FLAGS()

endif(CODE_COVERAGE)
endif(CMAKE_HOST_UNIX)

if(CMAKE_HOST_UNIX)
set(CMAKE_C_FLAGS    "${CMAKE_C_FLAGS} -Og -rdynamic -fsanitize=undefined -fsanitize=float-divide-by-zero -fno-sanitize-recover")

set(CMAKE_C_FLAGS    "${CMAKE_C_FLAGS} -ggdb -fno-omit-frame-pointer --short-enums -MMD -ffunction-sections -fdata-sections -fno-strict-aliasing -fsingle-precision-constant -frounding-math -fsignaling-nans")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++0x -fno-rtti -fpermissive -fexceptions")
else()
set(CMAKE_C_FLAGS    "${CMAKE_C_FLAGS} -Og -ggdb -fno-omit-frame-pointer --short-enums -MMD -ffunction-sections -fdata-sections -fno-strict-aliasing -fsingle-precision-constant -frounding-math -fsignaling-nans")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++0x -fno-rtti -fpermissive -fexceptions")
add_compile_options(-msse2 -mno-ms-bitfields)
add_definitions(-DWIN32)
add_definitions(-D_WIN32_WINNT=0x0600)
add_definitions(-DWINVER=0x600)
include_directories(./tools/TDDW)
endif()


# On inclut notre bibliotheque dans le processus de CMake
# add_subdirectory(source/routes)
add_subdirectory(tools/TDD/sd/fatfs)


# add the binary tree to the search path for include files
include_directories(
    ./
    ./legacy
	tools/TDD
	tools/TDD/sd
	tools/TDD/sd/fatfs/src
	tools/TDD/drivers
    tools/TDD/logger
	tools/TDD/nordic
	tools/TDD/timer
	tools/TDD/sensors
	tools/TDD/scheduler
	tools/TDD/sys
	libraries/AdafruitGFX
    libraries/AdafruitGFX/Fonts
	libraries/AltiBaro
	libraries/GlobalTop
	libraries/utils
    libraries/filters
    libraries/komoot
    libraries/kalman
	libraries/TinyGPSPlus
	libraries/VParser
	legacy/rf
    legacy/source
    legacy/source/routes
	legacy/source/model
	legacy/source/usb
	legacy/source/scheduling
	legacy/source/vue
	legacy/source/parsers
	legacy/source/display
	legacy/source/sensors
    )


aux_source_directory(tools/TDD/nordic                   NORDIC)

# Cree des variables avec les fichiers a compiler
set(SRCS
    tools/TDD/main_tdd.cpp
    tools/TDD/scheduler/task_scheduler.c
    tools/TDD/unit_testing.cpp
	tools/TDD/Model_tdd.cpp
    tools/TDD/Simulator.cpp
	tools/TDD/millis_tdd.c
	tools/TDD/order1_filter_tdd.c
	tools/TDD/sd/sd_hal.c
	tools/TDD/timer/timer.c
    tools/TDD/sensors/STC3100.cpp
    tools/TDD/sensors/fxos.cpp
    tools/TDD/sensors/bme280.c
	tools/TDD/sensors/VEML6075.cpp
	tools/TDD/drivers/uart_tdd.c
	tools/TDD/drivers/ant_tdd.c
	tools/TDD/drivers/drv_ws2812.c
	tools/TDD/drivers/gpio_tdd.cpp
	tools/TDD/drivers/ls027_tdd.cpp
	tools/TDD/drivers/neopixel_tdd.c
    tools/TDD/logger/tdd_logger.c
    tools/TDD/sys/GUI_connector.cpp
    legacy/segger_wrapper.c
    legacy/rf/ant_device_manager.cpp
	libraries/utils/WString.cpp
	libraries/AdafruitGFX/Adafruit_GFX.cpp
	libraries/AdafruitGFX/Print.cpp
	libraries/AltiBaro/AltiBaro.cpp
	libraries/utils/utils.c
    libraries/komoot/komoot_nav.c
    libraries/kalman/kalman_ext.cpp
    libraries/kalman/UDMatrix.cpp
	libraries/GlobalTop/EPONmeaPacket.cpp
	libraries/TinyGPSPlus/TinyGPS++.cpp
	libraries/VParser/VParser.cpp
	legacy/source/display/SegmentManager.cpp
	legacy/source/display/Zoom.cpp
	legacy/source/display/notifications.c
    legacy/source/routes/ListePoints.cpp
	legacy/source/routes/Parcours.cpp
	legacy/source/routes/Points.cpp
	legacy/source/routes/Segment.cpp
	legacy/source/routes/Vecteur.cpp
    legacy/source/sensors/fram.c
    legacy/source/sensors/GPSMGMT.cpp
	legacy/source/parsers/file_parser.cpp
	legacy/source/model/Attitude.cpp
	legacy/source/model/Boucle.cpp
	legacy/source/model/BoucleCRS.cpp
	legacy/source/model/BoucleFEC.cpp
	legacy/source/model/Locator.cpp
	legacy/source/model/PowerZone.cpp
	legacy/source/model/RRZone.cpp
	legacy/source/model/SufferScore.cpp
	legacy/source/model/UserSettings.cpp
	legacy/source/scheduling/power_scheduler.cpp
	legacy/source/sd/sd_functions.cpp
	legacy/source/vue/Screenutils.cpp
	legacy/source/vue/Menuable.cpp
    legacy/source/vue/MenuObjects.cpp
	legacy/source/vue/Vue.cpp
	legacy/source/vue/VueCRS.cpp
	legacy/source/vue/VueDebug.cpp
	legacy/source/vue/VueGPS.cpp
	legacy/source/vue/VuePRC.cpp
	legacy/source/vue/VueFEC.cpp
	legacy/source/Model.cpp
	legacy/source/g_structs.c
    )

set(HEADERS
    main_tdd.h
    Simulator.h
    )

# add the executable
add_executable(StravaV10 ${SRCS} ${NORDIC})

# Et pour que l'executable fonctionne, il faut lui indiquer la bibliotheque dont il depend
target_link_libraries(StravaV10 fatfs)


if(CMAKE_HOST_UNIX)

  target_link_libraries(StravaV10 ubsan)

else()

  target_link_libraries(StravaV10 imagehlp)
  target_link_libraries(StravaV10 wsock32)

endif()


if(CMAKE_HOST_UNIX)
if(CODE_COVERAGE)

SETUP_TARGET_FOR_COVERAGE_LCOV(
   NAME cov						    # Name for custom target.
   EXECUTABLE StravaV10				# Name of the test driver executable that runs the tests.
   # NOTE! This should always have a ZERO as exit code
   # otherwise the coverage generation will not complete.
   DEPENDENCIES StravaV10			# Name of output directory.
   )

SETUP_TARGET_FOR_COVERAGE_GCOVR_XML(
   NAME cov_xml						# Name for custom target.
   EXECUTABLE StravaV10				# Name of the test driver executable that runs the tests.
   # NOTE! This should always have a ZERO as exit code
   # otherwise the coverage generation will not complete.
   DEPENDENCIES StravaV10			# Name of output directory.
   )

endif(CODE_COVERAGE)
endif(CMAKE_HOST_UNIX)


# Create the gcov-clean target. This cleans the build as well as generated
# .gcda and .gcno files.
add_custom_target(scrub
	COMMAND ${CMAKE_MAKE_PROGRAM} clean
	COMMAND rm -f ${OBJECT_DIR}/*.gcno
	COMMAND rm -f ${OBJECT_DIR}/*.gcda
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)
