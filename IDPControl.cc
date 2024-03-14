// Control code for Inverted Double Pendulum

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <optional>

#include "moteus.h"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp"

int main(int argc, char** argv) {
  using namespace mjbots;

  moteus::Controller::DefaultArgProcess(argc, argv);

  return 0;
}
