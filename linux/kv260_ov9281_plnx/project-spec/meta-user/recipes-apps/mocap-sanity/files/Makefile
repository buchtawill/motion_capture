APP = mocap-sanity

# Add any other object files to this list below
APP_OBJS = mocap-sanity.o

# argparse.hpp (vendored single header in this dir) requires C++17.
CXXFLAGS += -std=c++17 -Wall -I.

all: build

build: $(APP)

$(APP): $(APP_OBJS)
	$(CXX) -o $@ $(APP_OBJS) $(LDFLAGS) $(LDLIBS)

%.o: %.cpp argparse.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(APP) *.o
