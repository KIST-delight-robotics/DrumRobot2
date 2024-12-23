CC = g++
CFLAGS = -Wall -O2 -g -std=c++17 -fPIC
INCLUDE = -I./include -I./lib -I./include/USBIO_advantech -I./include/path_manager -I./include/motors -I./include/tasks
LDFLAGS = -lm -lpthread -lstdc++fs -L./lib -lUSBIO_64 -Wl,-rpath,/home/shy/DrumRobot_v1.0/lib -lbiodaq

SRCDIR = ./src
OBJDIR = ./obj
BINDIR = ./bin
EXECUTABLE = main.out

# Automatically include all .cpp files from the src directory
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
OBJFILES := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))

# Build target
all: $(BINDIR)/$(EXECUTABLE)

$(BINDIR)/$(EXECUTABLE): $(OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)

# Pattern rules
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Clean rule
clean:
	rm -f $(OBJDIR)/*.o $(BINDIR)/$(EXECUTABLE)

# Qt 관련 설정 (현재 사용하지 않으므로 주석 처리)
# MOC = moc
# MOC_HEADERS = $(wildcard ./include/*.hpp) # Adjust to include all headers in include directory
# MOC_SRC = $(patsubst %.hpp, %.moc.cpp, $(MOC_HEADERS))
# MOC_OBJ = $(patsubst %.moc.cpp, %.moc.o, $(MOC_SRC))

# $(BINDIR): $(OBJFILES) $(MOC_OBJ)
#	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)

# # MOC 규칙
# %.moc.cpp: %.hpp
#	$(MOC) $< -o $@

# %.moc.o: %.moc.cpp
#	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)
