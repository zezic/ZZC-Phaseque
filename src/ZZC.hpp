#pragma once
#include <rack.hpp>

enum PolyphonyModes { MONOPHONIC, POLYPHONIC, UNISON, NUM_POLYPHONY_MODES };

using namespace rack;

// Forward-declare the Plugin, defined in Template.cpp
extern Plugin *pluginInstance;

// Forward-declare each Model, defined in each module source file
extern Model *modelPhaseque;
