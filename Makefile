CXXFLAGS = -std=c++17 -Wall -O3 -DNDEBUG

all: AcDc AcDc-ref

AcDc: sample.cpp
	$(CXX) $(CXXFLAGS) sample.cpp -o AcDc

AcDc-ref: ref.cpp
	$(CXX) $(CXXFLAGS) ref.cpp -o AcDc-ref

clean:
	$(RM) AcDc AcDc-ref
