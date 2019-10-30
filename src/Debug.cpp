#include "ZZC.hpp"

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../ZZC/src/widgets.hpp"
#endif

#include "Phaseque.hpp"

struct Debug : Module {
  enum ParamIds {
    NUM_PARAMS
  };
  enum InputIds {
    NUM_INPUTS
  };
  enum OutputIds {
    NUM_OUTPUTS
  };
  enum LightIds {
    NUM_LIGHTS
  };

  bool motherPresent = false;
  dsp::ClockDivider debugDivider;

  Debug() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    this->debugDivider.setDivision(128);
  }
  void process(const ProcessArgs &args) override;
};

void Debug::process(const ProcessArgs &args) {
  if (this->debugDivider.process()) {
    motherPresent = (leftExpander.module && leftExpander.module->model == modelPhaseque);
    if (motherPresent) {
      Phaseque *phaseque = reinterpret_cast<Phaseque*>(leftExpander.module);
      std::cout << phaseque->inputs[Phaseque::PHASE_INPUT].getVoltage() << std::endl;
    }
  }
}

struct DebugWidget : ModuleWidget {
  DebugWidget(Debug *module);
};

DebugWidget::DebugWidget(Debug *module) {
  setModule(module);
  setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/panels/Debug.svg")));
  addChild(createWidget<ZZC_Screw>(Vec(RACK_GRID_WIDTH, 0)));
  addChild(createWidget<ZZC_Screw>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
  addChild(createWidget<ZZC_Screw>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
  addChild(createWidget<ZZC_Screw>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
}

Model *modelDebug = createModel<Debug, DebugWidget>("Debug");
