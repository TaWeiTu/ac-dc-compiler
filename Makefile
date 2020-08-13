CXXFLAGS = -std=c++17 -Wall -O3 -DNDEBUG

AcDc: main.cpp
	$(CXX) $(CXXFLAGS) main.cpp -o AcDc

clean:
	$(RM) AcDc
