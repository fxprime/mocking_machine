#include <Arduino.h>

#include "app/MachineApplication.hpp"

void setup() { mm::MachineApplication::instance().begin(); }

void loop() { mm::MachineApplication::instance().runOnce(); }
