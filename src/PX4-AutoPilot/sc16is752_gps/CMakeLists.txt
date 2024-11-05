############################################################################
#
# Copyright (c) 2024 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Define the module name
px4_add_module(
	MODULE drivers__gps__sc16is752
	MAIN sc16is752_gps
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		sc16is752.cpp             # Base UART driver
		sc16is752_gps.cpp         # GPS implementation
	DEPENDS
		px4_work_queue
		drivers_gps
		i2c_spi_buses
		matrix
		mathlib
)

# Additional module dependencies
target_link_libraries(drivers__gps__sc16is752 PRIVATE
	parameters
	perf
	px4_platform_common
)

# Include directories
target_include_directories(drivers__gps__sc16is752 PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
)

# Installation path (optional)
# install(TARGETS drivers__gps__sc16is752 DESTINATION ${PX4_BOARD_DRIVER_DIR})

# Optional: Add testing if needed
#if(CONFIG_TESTING)
#	add_subdirectory(test)
#endif()

