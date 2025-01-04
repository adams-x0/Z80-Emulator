# Makefile for Z80 Emulator

# Compiler settings
CXX = g++
CC = gcc
CXXFLAGS = -c
TARGET = z80_emulator

# Object files
OBJS =  project-main.o z80.o z80_execute.o

# Target for building the executable
$(TARGET): $(OBJS)
	$(CXX) -o $(TARGET) $(OBJS)

# Rule for compiling project-main.c
project-main.o: project-main.c z80.h
	$(CC) $(CXXFLAGS) project-main.c -o project-main.o

# Rule for compiling z80.cpp
z80.o: z80.cpp z80.h
	$(CXX) $(CXXFLAGS) z80.cpp -o z80.o

# Rule for compiling z80_execute.cpp
z80_execute.o: z80_execute.cpp z80.h
	$(CXX) $(CXXFLAGS) z80_execute.cpp -o z80_execute.o


# Clean up object files and the executable
clean:
	rm -f $(OBJS) $(TARGET)
