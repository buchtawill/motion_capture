APP = mocap-hdmi-memcp

APP_OBJS = mocap-hdmi-memcp.o

CXXFLAGS += -std=c++17 -O2 -Wall -I.
LDLIBS += -lpthread

all: build

build: $(APP)

$(APP): $(APP_OBJS)
	$(CXX) -o $@ $(APP_OBJS) $(LDFLAGS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(APP) *.o
