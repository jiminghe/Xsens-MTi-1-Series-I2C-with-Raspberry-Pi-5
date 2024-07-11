# Makefile for mti_rp5 project

# Compiler
CXX = g++
CC = gcc

# Compiler flags
CXXFLAGS = -Wall -O2
CFLAGS = -Wall -O2

# Include directories
INCLUDES = -I. \
           -Ilib

# Library directories
LIB_DIRS =  -Llib/ -Llib/mtinterface -Llib/xbus -Llib/xstypes 

# Libraries
LIBS = -lwiringPi -lmtinterface -lxbus -lxstypes 

# Source files
CPP_SRC = $(wildcard src/*.cpp)
C_SRC = $(wildcard src/*.c)

# Object files
CPP_OBJ = $(CPP_SRC:.cpp=.o)
C_OBJ = $(C_SRC:.c=.o)
OBJ = $(CPP_OBJ) $(C_OBJ)

# Executable name
TARGET = mti_i2c_rp5

# Default target
all: libs $(TARGET)
#all: $(TARGET)


# Build libraries
libs:
	$(MAKE) -C lib/xstypes
	$(MAKE) -C lib/mtinterface
	$(MAKE) -C lib/xbus

# Link the target executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(LIB_DIRS) -o $@ $^ $(LIBS)

# Compile C++ source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile C source files into object files
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJ) $(TARGET)
	$(MAKE) -C lib/mtinterface clean
	$(MAKE) -C lib/xbus clean
	$(MAKE) -C lib/xstypes clean


# Phony targets
.PHONY: all clean libs
