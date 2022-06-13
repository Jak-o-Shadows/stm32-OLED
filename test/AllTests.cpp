#include "CppUTest/CommandLineTestRunner.h"

int main(int argc, char** argv) {
  CHECK(true);
  LONGS_EQUAL(1, 1);
  return CommandLineTestRunner::RunAllTests(argc, argv);
}
