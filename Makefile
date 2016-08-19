SRCS = main.en-US.cpp Interfaces.cpp Utilities.cpp DataConverter.cpp Finder.cpp
OBJS = $(SRCS:.cpp=.o)
PROG = bravo.out
CXX	= g++
CXXFLAGS = -O2
$(PROG) : $(OBJS)
		$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(OBJS)
