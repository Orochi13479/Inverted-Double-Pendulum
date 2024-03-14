// Driver code for Inverted Double Pendulum

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <optional>

#include "moteus.h"

int main(int argc, char** argv) {
  using namespace mjbots;

  moteus::Controller::DefaultArgProcess(argc, argv);

  return 0;
}
