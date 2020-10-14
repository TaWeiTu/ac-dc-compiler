CXXFLAGS = -std=c++17 -Wall -O3 -DNDEBUG

all: AcDc AcDc-ref fuzzer

AcDc: sample.cpp
	$(CXX) $(CXXFLAGS) sample.cpp -o AcDc

AcDc-ref: ref.cpp
	$(CXX) $(CXXFLAGS) ref.cpp -o AcDc-ref

fuzzer: fuzzer.cpp
	$(CXX) $(CXXFLAGS) fuzzer.cpp -o fuzzer

clean:
	$(RM) AcDc AcDc-ref fuzzer
