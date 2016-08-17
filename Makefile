SRCS = main.cpp Interfaces.cpp Utilities.cpp DataConverter.cpp Finder.cpp
OBJS = $(SRCS:.cpp=.o)
PROG = bravo.out
CXX	= clang++
CXXFLAGS = -O2
$(PROG) : $(OBJS)
		$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(OBJS)
