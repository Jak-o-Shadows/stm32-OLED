# To get cpputest working
# Use it as a submodule
# cd cpputest
# mkdir cpputest_build
# cmake -G"Unix Makefiles" ..
# sh
# make -j4
# Edit cmake_install.cmake. Change the install directory to ../install (ie. cpputest/install)
# make install




#.................................................
#    Path

CPPUTEST_DIR :=  ./cpputest/install

#.................................................
#    Gather of objects

TEST_C_SRCs   +=
TEST_CXX_SRCs += test/AllTests.cpp
TEST_CXX_SRCs += test/cheatsheet_test.cpp


TEST_OBJS      += $(sort $(TEST_C_SRCs:%.c=$(TEST_OUTDIR)%.to))
TEST_OBJS      += $(sort $(TEST_CXX_SRCs:%.cpp=$(TEST_OUTDIR)%.to))

#.................................................
#    Flags

TEST_CPPFLAGS   =
TEST_ASFLAGS    =
TEST_CFLAGS     =
TEST_CXXFLAGS   =
TEST_LDFLAGS    =

TEST_CXXFLAGS  += -I"$(CPPUTEST_DIR)/include/"

TEST_LDFLAGS   += -L"$(CPPUTEST_DIR)/lib/"\
                  -lCppUTest\
                  -lCppUTestExt\
				  -lstdc++

#TEST_CPPFLAGS   += $(CPPFLAGS)
#TEST_ASFLAGS    += $(ASFLAGS)
#TEST_CFLAGS     += $(CFLAGS)
#TEST_LDFLAGS    += $(LDFLAGS)

#.................................................
#    Toolchain

TEST_CC  := gcc
TEST_CXX := g++
TEST_LD  := g++

################################################################################
#    Rules
#


