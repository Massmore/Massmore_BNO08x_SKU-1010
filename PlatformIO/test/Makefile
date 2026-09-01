# Host-side test suite for Massmore_BNO08x.
# Runs on your PC or Mac — no ESP32, no sensor, no Arduino toolchain needed.
#
#   make        build and run
#   make clean
#
# It feeds synthetic SHTP cargoes through a mock I2C bus and checks that every
# report decodes to the value the SH-2 Reference Manual says it should, and
# that every command we send is byte-for-byte what the datasheet specifies.

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -O0 -g -Wall -Wextra -Wno-unused-parameter
# Works from either package layout: the Arduino library (../src) or the
# PlatformIO project (../lib/Massmore_BNO08x/src).
SRCDIR   := $(dir $(firstword $(wildcard ../src/Massmore_BNO08x.cpp \
                                        ../lib/Massmore_BNO08x/src/Massmore_BNO08x.cpp)))
INCLUDES := -Istub -I$(SRCDIR)
SOURCES  := test_parser.cpp $(SRCDIR)/Massmore_BNO08x.cpp

all: run

testrun: $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(SOURCES)

run: testrun
	./testrun

clean:
	rm -f testrun

.PHONY: all run clean
