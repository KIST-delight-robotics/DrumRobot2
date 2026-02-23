CC = g++
CFLAGS = -Wall -O2 -g -std=c++17 -fPIC

INCLUDE = -I./include -I./lib -I./include/USBIO_advantech -I./include/managers -I./include/motors -I./include/tasks \
		  -I/usr/include/opencv4 -I./include/eigen-3.4.0 -I/usr/local/include/pcl-1.13 -I/usr/include/eigen3 -I/usr/include/flann -I/usr/include/vtk-7.1 -I/usr/lib/x86_64-linux-gnu
LDFLAGS = -lm -lpthread -lstdc++fs -lbiodaq -L./lib -lUSBIO_64 -ldxl_x64_cpp -Wl,-rpath,'$$ORIGIN/../lib'
LDFLAGS += -lsfml-audio -lsfml-system
LDFLAGS += -lrealsense2
LDFLAGS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_aruco -lopencv_calib3d
# LDFLAGS = -lm -lpthread -lstdc++fs -L./lib -lUSBIO_64 -Wl,-rpath,/home/shy/DrumRobot_v1.0/lib -lbiodaq
LDFLAGS +=  -L/usr/local/lib -lpcl_common -lpcl_io -lpcl_filters -lpcl_segmentation \
           -lpcl_sample_consensus -lpcl_visualization -lpcl_search -lpcl_kdtree -lpcl_octree -lvtkRenderingLOD-7.1 -lvtkRenderingCore-7.1 -lvtkFiltersSources-7.1 -lvtkCommonExecutionModel-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonMath-7.1 -lvtkCommonCore-7.1

SRCDIR = ./src
OBJDIR = ./obj
BINDIR = ./bin

EXECUTABLE = main.out

# Automatically include all .cpp files from the src directory
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
OBJFILES := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))

# Build target
all: directories $(BINDIR)/$(EXECUTABLE)

# Ensure directories exist
directories:
	mkdir -p $(OBJDIR) $(BINDIR)

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
