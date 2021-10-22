// #include "toy-car-tests.cxx"
#include "running-toy-car-tests.cxx"

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
