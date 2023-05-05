###########################################################################
## Makefile generated for component 'qcqp'. 
## 
## Makefile     : qcqp_rtw.mk
## Generated on : Sun Apr 18 12:52:22 2021
## Final product: ./qcqp.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = qcqp
MAKEFILE                  = qcqp_rtw.mk
MATLAB_ROOT               = $(MATLAB_WORKSPACE)/usr/local/MATLAB/R2021a
MATLAB_BIN                = $(MATLAB_WORKSPACE)/usr/local/MATLAB/R2021a/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = $(MATLAB_WORKSPACE)/home/kotaru/Workspace/catkin_ws/qrotor_ws/src/qrotor_firmware/qrotor_firmware/scripts/control_m/codegen/lib/qcqp
TGT_FCN_LIB               = ISO_C++11
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
MODELLIB                  = qcqp.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU GCC Embedded Linux
# Supported Version(s):    
# ToolchainInfo Version:   2021a
# Specification Revision:  1.0
# 

#-----------
# MACROS
#-----------

CCOUTPUTFLAG = --output_file=
LDOUTPUTFLAG = --output_file=

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm -lm -lstdc++

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: GNU GCC Embedded Linux Assembler
AS = as

# C Compiler: GNU GCC Embedded Linux C Compiler
CC = gcc

# Linker: GNU GCC Embedded Linux Linker
LD = gcc

# C++ Compiler: GNU GCC Embedded Linux C++ Compiler
CPP = g++

# C++ Linker: GNU GCC Embedded Linux C++ Linker
CPP_LD = g++

# Archiver: GNU GCC Embedded Linux Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Make Tool
MAKE = make


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                = echo
MV                  =
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = -r
ASFLAGS              = -c \
                       $(ASFLAGS_ADDITIONAL) \
                       $(INCLUDES)
CFLAGS               = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -O2
CPPFLAGS             = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -fpermissive  \
                       -O2
CPP_LDFLAGS          = -lrt -lpthread -ldl
CPP_SHAREDLIB_LDFLAGS  = -shared  \
                         -lrt -lpthread -ldl
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -lrt -lpthread -ldl
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared  \
                       -lrt -lpthread -ldl



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./qcqp.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(MATLAB_WORKSPACE)/home/kotaru/Workspace/catkin_ws/qrotor_ws/src/qrotor_firmware/qrotor_firmware/scripts/control_m -I$(MATLAB_WORKSPACE)/home/kotaru/Documents/MATLAB/SupportPackages/R2021a/toolbox/realtime/targets/raspi/include -I$(MATLAB_ROOT)/toolbox/coder/rtiostream/src/utils -I$(MATLAB_ROOT)/extern/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
DEFINES_CUSTOM = 
DEFINES_SKIPFORSIL = -D__linux__ -DARM_PROJECT -D_USE_TARGET_UDP_ -D_RUNONTARGETHARDWARE_BUILD_ -DSTACK_SIZE=200000
DEFINES_STANDARD = -DMODEL=qcqp

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/qcqp_data.cpp $(START_DIR)/rt_nonfinite.cpp $(START_DIR)/rtGetNaN.cpp $(START_DIR)/rtGetInf.cpp $(START_DIR)/qcqp_initialize.cpp $(START_DIR)/qcqp_terminate.cpp $(START_DIR)/qcqp.cpp $(START_DIR)/computeComplError.cpp $(START_DIR)/xnrm2.cpp $(START_DIR)/evalObjAndConstrAndDerivatives.cpp $(START_DIR)/setProblemType.cpp $(START_DIR)/modifyOverheadPhaseOne_.cpp $(START_DIR)/driver.cpp $(START_DIR)/test_exit.cpp $(START_DIR)/computeGradLag.cpp $(START_DIR)/updateWorkingSetForNewQP.cpp $(START_DIR)/xgeqp3.cpp $(START_DIR)/xzgeqp3.cpp $(START_DIR)/xzlarfg.cpp $(START_DIR)/xzlarf.cpp $(START_DIR)/computeQ_.cpp $(START_DIR)/xgemv.cpp $(START_DIR)/sortLambdaQP.cpp $(START_DIR)/countsort.cpp $(START_DIR)/RemoveDependentIneq_.cpp $(START_DIR)/step.cpp $(START_DIR)/driver1.cpp $(START_DIR)/PresolveWorkingSet.cpp $(START_DIR)/feasibleX0ForWorkingSet.cpp $(START_DIR)/factorQR.cpp $(START_DIR)/maxConstraintViolation.cpp $(START_DIR)/computeFval.cpp $(START_DIR)/linearForm_.cpp $(START_DIR)/iterate.cpp $(START_DIR)/computeGrad_StoreHx.cpp $(START_DIR)/computeFval_ReuseHx.cpp $(START_DIR)/xrotg.cpp $(START_DIR)/deleteColMoveEnd.cpp $(START_DIR)/compute_deltax.cpp $(START_DIR)/xpotrf.cpp $(START_DIR)/fullColLDL2_.cpp $(START_DIR)/solve.cpp $(START_DIR)/xgemm.cpp $(START_DIR)/feasibleratiotest.cpp $(START_DIR)/addAineqConstr.cpp $(START_DIR)/addBoundToActiveSetMatrix_.cpp $(START_DIR)/relaxed.cpp $(START_DIR)/evalObjAndConstr.cpp $(START_DIR)/BFGSUpdate.cpp $(START_DIR)/qcqp_rtwutil.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = qcqp_data.cpp.o rt_nonfinite.cpp.o rtGetNaN.cpp.o rtGetInf.cpp.o qcqp_initialize.cpp.o qcqp_terminate.cpp.o qcqp.cpp.o computeComplError.cpp.o xnrm2.cpp.o evalObjAndConstrAndDerivatives.cpp.o setProblemType.cpp.o modifyOverheadPhaseOne_.cpp.o driver.cpp.o test_exit.cpp.o computeGradLag.cpp.o updateWorkingSetForNewQP.cpp.o xgeqp3.cpp.o xzgeqp3.cpp.o xzlarfg.cpp.o xzlarf.cpp.o computeQ_.cpp.o xgemv.cpp.o sortLambdaQP.cpp.o countsort.cpp.o RemoveDependentIneq_.cpp.o step.cpp.o driver1.cpp.o PresolveWorkingSet.cpp.o feasibleX0ForWorkingSet.cpp.o factorQR.cpp.o maxConstraintViolation.cpp.o computeFval.cpp.o linearForm_.cpp.o iterate.cpp.o computeGrad_StoreHx.cpp.o computeFval_ReuseHx.cpp.o xrotg.cpp.o deleteColMoveEnd.cpp.o compute_deltax.cpp.o xpotrf.cpp.o fullColLDL2_.cpp.o solve.cpp.o xgemm.cpp.o feasibleratiotest.cpp.o addAineqConstr.cpp.o addBoundToActiveSetMatrix_.cpp.o relaxed.cpp.o evalObjAndConstr.cpp.o BFGSUpdate.cpp.o qcqp_rtwutil.cpp.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


DERIVED_SRCS = $(subst .o,.dep,$(OBJS))

build:

%.dep:



-include codertarget_assembly_flags.mk
-include *.dep


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.c.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(MATLAB_WORKSPACE)/home/kotaru/Workspace/catkin_ws/qrotor_ws/src/qrotor_firmware/qrotor_firmware/scripts/control_m/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(MATLAB_WORKSPACE)/home/kotaru/Workspace/catkin_ws/qrotor_ws/src/qrotor_firmware/qrotor_firmware/scripts/control_m/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(MATLAB_WORKSPACE)/home/kotaru/Workspace/catkin_ws/qrotor_ws/src/qrotor_firmware/qrotor_firmware/scripts/control_m/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


qcqp_data.cpp.o : $(START_DIR)/qcqp_data.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_nonfinite.cpp.o : $(START_DIR)/rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetNaN.cpp.o : $(START_DIR)/rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetInf.cpp.o : $(START_DIR)/rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


qcqp_initialize.cpp.o : $(START_DIR)/qcqp_initialize.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


qcqp_terminate.cpp.o : $(START_DIR)/qcqp_terminate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


qcqp.cpp.o : $(START_DIR)/qcqp.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeComplError.cpp.o : $(START_DIR)/computeComplError.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xnrm2.cpp.o : $(START_DIR)/xnrm2.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


evalObjAndConstrAndDerivatives.cpp.o : $(START_DIR)/evalObjAndConstrAndDerivatives.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


setProblemType.cpp.o : $(START_DIR)/setProblemType.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


modifyOverheadPhaseOne_.cpp.o : $(START_DIR)/modifyOverheadPhaseOne_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


driver.cpp.o : $(START_DIR)/driver.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


test_exit.cpp.o : $(START_DIR)/test_exit.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeGradLag.cpp.o : $(START_DIR)/computeGradLag.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


updateWorkingSetForNewQP.cpp.o : $(START_DIR)/updateWorkingSetForNewQP.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgeqp3.cpp.o : $(START_DIR)/xgeqp3.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzgeqp3.cpp.o : $(START_DIR)/xzgeqp3.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlarfg.cpp.o : $(START_DIR)/xzlarfg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlarf.cpp.o : $(START_DIR)/xzlarf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeQ_.cpp.o : $(START_DIR)/computeQ_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgemv.cpp.o : $(START_DIR)/xgemv.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sortLambdaQP.cpp.o : $(START_DIR)/sortLambdaQP.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


countsort.cpp.o : $(START_DIR)/countsort.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


RemoveDependentIneq_.cpp.o : $(START_DIR)/RemoveDependentIneq_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


step.cpp.o : $(START_DIR)/step.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


driver1.cpp.o : $(START_DIR)/driver1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


PresolveWorkingSet.cpp.o : $(START_DIR)/PresolveWorkingSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


feasibleX0ForWorkingSet.cpp.o : $(START_DIR)/feasibleX0ForWorkingSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


factorQR.cpp.o : $(START_DIR)/factorQR.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


maxConstraintViolation.cpp.o : $(START_DIR)/maxConstraintViolation.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFval.cpp.o : $(START_DIR)/computeFval.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


linearForm_.cpp.o : $(START_DIR)/linearForm_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


iterate.cpp.o : $(START_DIR)/iterate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeGrad_StoreHx.cpp.o : $(START_DIR)/computeGrad_StoreHx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFval_ReuseHx.cpp.o : $(START_DIR)/computeFval_ReuseHx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xrotg.cpp.o : $(START_DIR)/xrotg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


deleteColMoveEnd.cpp.o : $(START_DIR)/deleteColMoveEnd.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


compute_deltax.cpp.o : $(START_DIR)/compute_deltax.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xpotrf.cpp.o : $(START_DIR)/xpotrf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


fullColLDL2_.cpp.o : $(START_DIR)/fullColLDL2_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


solve.cpp.o : $(START_DIR)/solve.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgemm.cpp.o : $(START_DIR)/xgemm.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


feasibleratiotest.cpp.o : $(START_DIR)/feasibleratiotest.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


addAineqConstr.cpp.o : $(START_DIR)/addAineqConstr.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


addBoundToActiveSetMatrix_.cpp.o : $(START_DIR)/addBoundToActiveSetMatrix_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


relaxed.cpp.o : $(START_DIR)/relaxed.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


evalObjAndConstr.cpp.o : $(START_DIR)/evalObjAndConstr.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


BFGSUpdate.cpp.o : $(START_DIR)/BFGSUpdate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


qcqp_rtwutil.cpp.o : $(START_DIR)/qcqp_rtwutil.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	echo "### PRODUCT = $(PRODUCT)"
	echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	echo "### BUILD_TYPE = $(BUILD_TYPE)"
	echo "### INCLUDES = $(INCLUDES)"
	echo "### DEFINES = $(DEFINES)"
	echo "### ALL_SRCS = $(ALL_SRCS)"
	echo "### ALL_OBJS = $(ALL_OBJS)"
	echo "### LIBS = $(LIBS)"
	echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	echo "### ASFLAGS = $(ASFLAGS)"
	echo "### CFLAGS = $(CFLAGS)"
	echo "### LDFLAGS = $(LDFLAGS)"
	echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	echo "### CPPFLAGS = $(CPPFLAGS)"
	echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	echo "### ARFLAGS = $(ARFLAGS)"
	echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(RM) *.c.dep
	$(RM) *.cpp.dep
	$(ECHO) "### Deleted all derived files."


