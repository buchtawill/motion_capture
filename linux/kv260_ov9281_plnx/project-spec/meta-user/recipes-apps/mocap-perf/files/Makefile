APP = mocap-perf

# Add any other object files to this list below
APP_OBJS = mocap-perf.o

# <mocap/argparse.hpp> (staged from the mocap-common recipe sysroot) requires
# C++17. The sysroot includedir is already on the default search path.
CXXFLAGS += -std=c++17 -O2 -Wall -I.

all: build

build: $(APP)

$(APP): $(APP_OBJS)
	$(CXX) -o $@ $(APP_OBJS) $(LDFLAGS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(APP) *.o
