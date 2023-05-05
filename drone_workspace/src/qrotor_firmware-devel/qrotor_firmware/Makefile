
TARGET	?= qrotor

BOARD ?= NAVIO

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/tnt0

BOARD_LIB ?= libnavio.a


#################################
# Working directories
#################################
ROOT_DIR		= .
BOARD_DIR		= $(ROOT_DIR)/external/Navio2/C++/Navio
BUILD_DIR		= $(ROOT_DIR)/build
CXXSRC_DIR		= $(ROOT_DIR)/src
MATH_DIR	  	= $(ROOT_DIR)/external\
					$(ROOT_DIR)/external/Matrix \
					$(ROOT_DIR)/external/math

#################################
# Source Files
#################################
INCLUDE_DIRS += $(ROOT_DIR) \
				$(ROOT_DIR)/include \
				$(BOARD_DIR) \
				$(BOARD_DIR)/Common \
				$(BOARD_DIR)/Navio2 \
				$(BOARD_DIR)/Navio+ \
				$(BUILD_DIR) \
				$(CXXSRC_DIR) \
				$(ROOT_DIR)/lib 

# navio source files
VPATH		:= $(VPATH):$(BOARD_DIR):$(BOARD_DIR)/Common:$(BOARD_DIR)/Navio2:$(BOARD_DIR)/Navio+
NAVIO_CXX_SRC = $(notdir $(wildcard $(BOARD_DIR)/*/*.cpp))

# main source files
VPATH		:= $(VPATH):$(CXXSRC_DIR):
QROTOR_SRC =	qrotor_flight.cpp \
				board.cpp \
				mixer.cpp \
				sensors.cpp \
                                command_manager.cpp \
				state_machine.cpp \
                                estimation/ahrs_default.cpp \
				estimation/pose_estimator.cpp \
				estimation/pose_iekf.cpp \
                                estimation/pose_dummy.cpp \
				control/attitude_controller.cpp \
				control/euler_angle_ppid.cpp \
                                control/mueller_controller.cpp \
                                control/position_controller.cpp \
                                control/position_pid.cpp \
                                peripherals/rs_t265.cpp \
				external/mathlib/math/filter/LowPassFilter2pVector3f.cpp \
				main.cpp

# Math Source Files
VPATH := $(VPATH):$(MATH_DIR)

CXXSOURCES = $(NAVIO_CXX_SRC) \
			 $(QROTOR_SRC) 
#################################
# Build
#################################

CC=gcc
CXX=g++
CXXFLAGS = -std=c++11 -Wno-psabi -lpigpio -lrt -lpthread -lrealsense2 -llcm -lstdc++fs\
			$(addprefix -I ,$(INCLUDE_DIRS)) $(DEFS) #$(DEBUG_FLAGS) \
			$(CXX_FILE_SIZE_FLAGS) # $(CXX_STRICT_FLAGS)
# DEPS = $(notdir $(wildcard $(BOARD_SRC_DIR)/*/*.cpp))
OBJECTS = $(addsuffix .o,$(addprefix $(BUILD_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))


TARGET_EXE=$(BUILD_DIR)/$(TARGET)$(BOARD)


$(TARGET_EXE): $(OBJECTS)
	$(CXX) -o $@ $^ $(CXXFLAGS)

$(BUILD_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	$(CXX) -c -o $@ $< $(CXXFLAGS)


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
	rm -rf $(BUILD_DIR)

