#ifndef WIDGETS_H
#define WIDGETS_H
#include "../ZZC/src/widgets.hpp"
#endif

#include <ctime>

#include "ZZC.hpp"
#include "Phaseque.hpp"
#include "helpers.hpp"

struct PhasequePatternResoChange : history::ModuleAction {
  int paramId;
  unsigned int patternNum;
  float oldValue;
  float newValue;

  void undo() override {
    app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    assert(mw);
    Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    phaseq->patterns[patternNum].resolution = oldValue;
    if (phaseq->patternIdx == patternNum) {
      phaseq->pattern.resolution = oldValue;
      mw->module->params[paramId].value = oldValue;
    } else {
      phaseq->patternFlashNeg = patternNum;
    }
  }

  void redo() override {
    app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    assert(mw);
    Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    phaseq->patterns[patternNum].resolution = newValue;
    if (phaseq->patternIdx == patternNum) {
      phaseq->pattern.resolution = newValue;
      mw->module->params[paramId].value = newValue;
    } else {
      phaseq->patternFlashPos = patternNum;
    }
  }

  PhasequePatternResoChange() {
    name = "change pattern resolution";
  }
};

struct ZZC_PhasequePatternResoKnob : SvgKnob {
  ZZC_PhasequePatternResoKnob() {
    setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/knobs/ZZC-Knob-25-Encoder.svg")));
    shadow->box.size = Vec(29, 29);
    shadow->box.pos = Vec(-2, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
    // speed = 8.f;
    smooth = false;
    snap = true;
    maxAngle = M_PI * 4;
  }

  void onDragEnd(const event::DragEnd &e) override {
    if (e.button != GLFW_MOUSE_BUTTON_LEFT)
      return;

    APP->window->cursorUnlock();

    if (paramQuantity) {
      float newValue = paramQuantity->getValue();
      if (oldValue != newValue) {
        PhasequePatternResoChange *h = new PhasequePatternResoChange;
        h->moduleId = paramQuantity->module->id;
        h->paramId = paramQuantity->paramId;
        Phaseque* phaseq = static_cast<Phaseque*>(paramQuantity->module);
        h->patternNum = phaseq->patternIdx;
        h->oldValue = oldValue;
        h->newValue = newValue;
        h->name = "change pattern " + std::to_string(phaseq->patternIdx) + " resolution";
        APP->history->push(h);
      }
    }
  }
};

struct PhasequePatternShiftChange : history::ModuleAction {
  int paramId;
  unsigned int patternNum;
  float oldValue;
  float newValue;

  void undo() override {
    app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    assert(mw);
    Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    phaseq->patterns[patternNum].shift = oldValue;
    if (phaseq->patternIdx == patternNum) {
      phaseq->pattern.shift = oldValue;
      mw->module->params[paramId].value = oldValue;
    } else {
      phaseq->patternFlashNeg = patternNum;
    }
  }

  void redo() override {
    app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    assert(mw);
    Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    phaseq->patterns[patternNum].shift = newValue;
    if (phaseq->patternIdx == patternNum) {
      phaseq->pattern.shift = newValue;
      mw->module->params[paramId].value = newValue;
    } else {
      phaseq->patternFlashPos = patternNum;
    }
  }

  PhasequePatternShiftChange() {
    name = "change pattern shift";
  }
};

struct ZZC_DisplayKnob : SvgKnob {
  ZZC_DirectKnobDisplay *disp = nullptr;
  float strokeWidth = 1.5f;
  bool unipolar = false;

  ZZC_DisplayKnob() {
    smooth = false;
    disp = new ZZC_DirectKnobDisplay();
    fb->addChild(disp);
    disp->box.pos = math::Vec(0, 0);
    disp->strokeWidth = strokeWidth;
    disp->box.size = fb->box.size;
  }

  void recalcSizes() {
    float padding = strokeWidth + 2.f;
    sw->box.pos = math::Vec(padding, padding);
    tw->box.size = sw->box.size;
    math::Vec size = math::Vec(padding * 2, padding * 2).plus(sw->box.size);
    setSize(size);
    fb->box.size = size;
    shadow->box.size = sw->box.size;
    // Move shadow downward by 20% and take value display into account
    shadow->box.pos = math::Vec(padding, padding).plus(math::Vec(0, sw->box.size.y * 0.2));
    disp->strokeWidth = strokeWidth;
    disp->box.size = fb->box.size;
  }

  void onChange(const event::Change &e) override {
    if (paramQuantity) {
      disp->setLimits(paramQuantity->getMinValue(), paramQuantity->getMaxValue());
      disp->value = paramQuantity->getValue();
    }
    SvgKnob::onChange(e);
  }

  void randomize() override {};
};

struct ZZC_PhasequePatternShiftKnob : ZZC_DisplayKnob {
  ZZC_PhasequePatternShiftKnob() {
    setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/knobs/ZZC-Knob-25-Encoder.svg")));
    shadow->box.size = Vec(31, 31);
    shadow->box.pos = Vec(0, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
    maxAngle = M_PI * 1.5;
    recalcSizes();
  }

  void onDragEnd(const event::DragEnd &e) override {
    if (e.button != GLFW_MOUSE_BUTTON_LEFT)
      return;

    APP->window->cursorUnlock();

    if (paramQuantity) {
      float newValue = paramQuantity->getSmoothValue();
      if (oldValue != newValue) {
        PhasequePatternShiftChange *h = new PhasequePatternShiftChange;
        h->moduleId = paramQuantity->module->id;
        h->paramId = paramQuantity->paramId;
        Phaseque* phaseq = static_cast<Phaseque*>(paramQuantity->module);
        h->patternNum = phaseq->patternIdx;
        h->oldValue = oldValue;
        h->newValue = newValue;
        h->name = "change pattern " + std::to_string(phaseq->patternIdx) + " shift";
        APP->history->push(h);
      }
    }
  }
};

struct ZZC_PhasequeMutaKnob : SvgKnob {
  ZZC_PhasequeMutaKnob() {
    setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/knobs/ZZC-Knob-27-Encoder.svg")));
    shadow->box.size = Vec(33, 33);
    shadow->box.pos = Vec(-3, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
    smooth = false;
  }

  void onDragEnd(const event::DragEnd &e) override {
    if (e.button != GLFW_MOUSE_BUTTON_LEFT)
      return;
    APP->window->cursorUnlock();
  }
};

struct PhasequeStepAttrChange : history::ModuleAction {
  int paramId;
  unsigned int patternNum;
  int step;
  int attr;
  float oldValue;
  float newValue;

  void undo() override {
    app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    assert(mw);
    Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    phaseq->patterns[patternNum].steps[step].setAttrBase(attr, oldValue);
    if (phaseq->patternIdx == patternNum) {
      phaseq->pattern.steps[step].setAttrBase(attr, oldValue);
      mw->module->params[paramId].value = oldValue;
    } else {
      phaseq->patternFlashNeg = patternNum;
    }
  }

  void redo() override {
    app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    assert(mw);
    Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    phaseq->patterns[patternNum].steps[step].setAttrBase(attr, newValue);
    if (phaseq->patternIdx == patternNum) {
      phaseq->pattern.steps[step].setAttrBase(attr, newValue);
      mw->module->params[paramId].value = newValue;
    } else {
      phaseq->patternFlashPos = patternNum;
    }
  }

  PhasequeStepAttrChange() {
    name = "change step attribute";
  }
};

struct ZZC_PhasequeAttrKnob : ZZC_DisplayKnob {
  ZZC_PhasequeAttrKnob() {
  }

  void onDragEnd(const event::DragEnd &e) override {
    if (e.button != GLFW_MOUSE_BUTTON_LEFT)
      return;

    APP->window->cursorUnlock();

    if (paramQuantity) {
      float newValue = paramQuantity->getSmoothValue();
      if (oldValue != newValue) {
        PhasequeStepAttrChange *h = new PhasequeStepAttrChange;
        h->moduleId = paramQuantity->module->id;
        h->paramId = paramQuantity->paramId;
        Phaseque* phaseq = static_cast<Phaseque*>(paramQuantity->module);
        h->patternNum = phaseq->patternIdx;
        h->oldValue = oldValue;
        h->newValue = newValue;
        StepAttrParamQuantityBase* q = static_cast<StepAttrParamQuantityBase*>(paramQuantity);
        h->step = q->item;
        h->name = "change step " + StepAttrNames[q->attr];
        h->attr = q->attr;
        APP->history->push(h);
      }
    }
  }
};

struct ZZC_PhasequeXYDisplayWidget : XYDisplayWidget {
  ZZC_PhasequeXYDisplayWidget() {
  }
  void onDragEnd(const event::DragEnd &e) override {
    if (e.button != GLFW_MOUSE_BUTTON_LEFT)
      return;

    APP->window->cursorUnlock();

    if (paramQuantityX) {
      float newValueX = paramQuantityX->getValue();
      if (oldValueX != newValueX) {
        PhasequeStepAttrChange *h = new PhasequeStepAttrChange;
        h->moduleId = paramQuantityX->module->id;
        h->paramId = paramQuantityX->paramId;
        Phaseque* phaseq = static_cast<Phaseque*>(paramQuantityX->module);
        h->patternNum = phaseq->patternIdx;
        h->oldValue = oldValueX;
        h->newValue = newValueX;
        StepAttrParamQuantityBase* q = static_cast<StepAttrParamQuantityBase*>(paramQuantityX);
        h->step = q->item;
        h->name = "change step " + StepAttrNames[q->attr];
        h->attr = q->attr;
        APP->history->push(h);
      }
    }
    if (paramQuantityY) {
      float newValueY = paramQuantityY->getValue();
      if (oldValueY != newValueY) {
        PhasequeStepAttrChange *h = new PhasequeStepAttrChange;
        h->moduleId = paramQuantityY->module->id;
        h->paramId = paramQuantityY->paramId;
        Phaseque* phaseq = static_cast<Phaseque*>(paramQuantityY->module);
        h->patternNum = phaseq->patternIdx;
        h->oldValue = oldValueY;
        h->newValue = newValueY;
        StepAttrParamQuantityBase* q = static_cast<StepAttrParamQuantityBase*>(paramQuantityY);
        h->step = q->item;
        h->name = "change step " + StepAttrNames[q->attr];
        h->attr = q->attr;
        APP->history->push(h);
      }
    }
  }
};

struct ZZC_PhasequeStepAttrKnob23 : ZZC_PhasequeAttrKnob {
  ZZC_PhasequeStepAttrKnob23() {
    setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/knobs/ZZC-Knob-27-23-Encoder.svg")));
    shadow->box.size = Vec(33, 33);
    shadow->box.pos = Vec(-1.5, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
    recalcSizes();
  }
};

struct ZZC_PhasequeStepAttrKnob21 : ZZC_PhasequeAttrKnob {
  ZZC_PhasequeStepAttrKnob21() {
    setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/knobs/ZZC-Knob-27-21-Encoder.svg")));
    shadow->box.size = Vec(29, 29);
    shadow->box.pos = Vec(-0.5, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
    recalcSizes();
  }
};

struct ZZC_PhasequeStepAttrKnob21Uni : ZZC_PhasequeStepAttrKnob21 {
  ZZC_PhasequeStepAttrKnob21Uni() {
    unipolar = true;
  }
};

struct ZZC_PhasequeStepAttrKnob19 : ZZC_PhasequeAttrKnob {
  ZZC_PhasequeStepAttrKnob19() {
    setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/knobs/ZZC-Knob-27-19-Encoder.svg")));
    shadow->box.size = Vec(25, 25);
    shadow->box.pos = Vec(0.5, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
    recalcSizes();
    disp->enableColor();
  }
};

void Phaseque::process(const ProcessArgs &args) {
  processGlobalParams();
  processPatternNav();
  processButtons();
  processPatternButtons();

  // Phase param
  float phaseParamInput = params[PHASE_PARAM].getValue();
  bool phaseWasZeroed = false;
  if (phaseParamInput != phaseParam) {
    if (phaseParamInput == 0.0f || (lastPhaseParamInput < 0.1f && phaseParamInput > 0.9f) || (lastPhaseParamInput > 0.9f && phaseParamInput < 0.1f)) {
      if (phaseParamInput == 0.0f) {
        phaseWasZeroed = true;
      }
      phaseParam = phaseParamInput;
    } else {
      float delta = phaseParamInput - phaseParam;
      phaseParam = phaseParam + delta * args.sampleTime * 50.0f; // Smoothing
    }
  }

  if (absMode) {
    resolution = 1.0;
  } else {
    resolution = pattern.resolution;
  }

  processJumps();

  if (resetPulse) {
    phase = 0.0;
    phaseWasZeroed = true;
  }

  if (inputs[PHASE_INPUT].isConnected()) {
    bpmDisabled = true;
    double phaseIn = inputs[PHASE_INPUT].getVoltage() / 10.0;
    while (phaseIn >= 1.0) {
      phaseIn = phaseIn - 1.0;
    }
    while (phaseIn < 0.0) {
      phaseIn = phaseIn + 1.0;
    }
    double phaseInDelta = 0.0;
    if (resetPulse) {
      phaseInDelta = lastPhaseInDelta;
    } else {
      if (lastPhaseInState) {
        // Trust the state
        phaseInDelta = phaseIn - lastPhaseIn;
        if (fabs(phaseInDelta) > 0.01) {
          if (samplesSinceLastReset < 20) {
            // Nah! It's a cable-delay anomaly.
            phaseInDelta = lastPhaseInDelta;
          } else if (phaseIn < 0.01 || phaseIn > 0.99) {
            // Handle phase-flip
            if (phaseIn < lastPhaseIn) {
              phaseInDelta = 1.0f + phaseInDelta;
            }
            if (phaseIn > lastPhaseIn) {
              phaseInDelta = phaseInDelta - 1.0f;
            }
          }
        }
      }
    }
    if (clutch) {
      if (absMode) {
        phase = phaseIn;
      } else {
        if (inputs[CLOCK_INPUT].isConnected() && clockTrigger.process(inputs[CLOCK_INPUT].getVoltage())) {
          float targetPhase = phase + phaseInDelta / resolution;
          float delta = fmodf(targetPhase * resolution, 1.0f);
          if (delta < 0.01) {
            phase = roundf(targetPhase * resolution) / resolution;
          } else {
            phase = targetPhase;
          }
        } else {
          phase = phase + phaseInDelta / resolution;
        }
      }
    }
    lastPhaseIn = phaseIn;
    lastPhaseInDelta = phaseInDelta;
  } else if (inputs[CLOCK_INPUT].isConnected()) {
    bpmDisabled = false;
    if (!lastClockInputState || lastPhaseInState) {
      tempoTracker.reset();
    }
    float currentStep = floorf(phase * resolution);
    float sampleTime = args.sampleTime;
    if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage()) && samplesSinceLastReset > 19) {
      tempoTracker.tick(sampleTime);
      if (clutch) {
        if (bps < 0.0f) {
          float nextStep = eucMod(currentStep, resolution);
          float nextPhase = fastmod(nextStep / resolution, 1.0f);
          phase = nextPhase;
        } else {
          float nextStep = eucMod(currentStep + 1.0f, resolution);
          float nextPhase = fastmod(nextStep / resolution, 1.0f);
          phase = nextPhase;
        }
      }
      tickedAtLastSample = true;
    } else {
      tempoTracker.acc(sampleTime);
      if (inputs[VBPS_INPUT].isConnected()) {
        bps = inputs[VBPS_INPUT].getVoltage();
      } else if (tempoTrack) {
        if (tempoTracker.detected) {
          bps = tempoTracker.bps;
        }
      } else {
        bps = params[BPM_PARAM].getValue() / 60.0f;
      }
      float nextPhase = fastmod(phase + bps * args.sampleTime / resolution, 1.0f);
      float nextStep = floorf(nextPhase * resolution);
      if (clutch) {
        if (nextStep == currentStep || (bps < 0.0f && (tickedAtLastSample || resetPulse))) {
          phase = nextPhase;
        }
      }
      tickedAtLastSample = false;
    }
  } else if (inputs[VBPS_INPUT].isConnected()) {
    bpmDisabled = false;
    bps = inputs[VBPS_INPUT].getVoltage();
    if (clutch) {
      float nextPhase = fastmod(phase + bps * args.sampleTime / resolution, 1.0f);
      phase = nextPhase;
    }
  }
  float preciseBpm = bps * 60.0f;
  float roundedBpm = roundf(preciseBpm);
  if (isNear(preciseBpm, roundedBpm, 0.06f)) {
    bpm = roundedBpm;
  } else {
    bpm = preciseBpm;
  }
  if (std::isnan(phase)) {
    phase = 0.0;
  }
  while (phase >= 1.0) {
    phase = phase - 1.0;
  }
  while (phase < 0.0) {
    phase = phase + 1.0;
  }
  phaseShifted = phase + phaseParam;
  while (phaseShifted >= 1.0f) {
    phaseShifted = phaseShifted - 1.0f;
  }
  while (phaseShifted < 0.0f) {
    phaseShifted = phaseShifted + 1.0f;
  }

  float phaseDelta = phaseShifted - lastPhaseShifted;
  if (phaseDelta != 0.0f && fabsf(phaseDelta) < 0.95f && !phaseWasZeroed && !jump) {
    direction = phaseDelta > 0.0f ? 1 : -1;
  }
  if (phaseShifted < 0.05f && lastPhaseShifted > 0.95f) {
    ptrnEndPulseGenerator.trigger(1e-3f);
  } else if (phaseShifted > 0.95f && lastPhaseShifted < 0.05f) {
    ptrnStartPulseGenerator.trigger(1e-3f);
  }

  if (polyphonyMode == POLYPHONIC) {
    activeStep = nullptr;
    pattern.updateStepsStates(phaseShifted, globalGate, stepsStates, false);
  } else if (polyphonyMode == UNISON) {
    activeStep = nullptr;
    pattern.updateStepsStates(phaseShifted, globalGate, stepsStates, false);
    pattern.updateStepsStates(phaseShifted, globalGate, unisonStates, true);
  } else {
    activeStep = pattern.getStepForPhase(phaseShifted, globalGate);
    for (int i = 0; i < NUM_STEPS; i++) {
      stepsStates[i] = false;
      unisonStates[i] = false;
    }
  }
  if ((activeStep && lastActiveStep) && activeStep != lastActiveStep) {
    retrigGapGenerator.trigger(1e-4f);
  }
  lastActiveStep = activeStep;

  outputs[PTRN_START_OUTPUT].setVoltage(ptrnStartPulseGenerator.process(args.sampleTime) ? 10.0f : 0.0f);
  outputs[PTRN_END_OUTPUT].setVoltage(ptrnEndPulseGenerator.process(args.sampleTime) ? 10.0f : 0.0f);
  outputs[PTRN_WRAP_OUTPUT].setVoltage(std::max(outputs[PTRN_START_OUTPUT].value, outputs[PTRN_END_OUTPUT].value));
  outputs[PTRN_OUTPUT].setVoltage(patternToVolts(patternIdx));
  if (patternIdx != lastPatternIdx) {
    wentPulseGenerator.trigger(1e-3f);
    lastPatternIdx = patternIdx;
  }
  outputs[WENT_OUTPUT].setVoltage(wentPulseGenerator.process(args.sampleTime) ? 10.0f : 0.0f);

  if (resetPulse && !absMode) {
    retrigGapGenerator.trigger(1e-4f);
  }

  retrigGap = retrigGapGenerator.process(args.sampleTime);
  if (polyphonyMode == MONOPHONIC) {
    if (retrigGap || !clutch) {
      outputs[GATE_OUTPUT].setVoltage(0.0f);
    } else {
      outputs[GATE_OUTPUT].setVoltage(activeStep ? 10.0f : 0.0f);
    }
  }
  if (polyphonyMode == POLYPHONIC || polyphonyMode == UNISON) {
    for (int i = 0; i < NUM_STEPS; i++) {
      if (stepsStates[i]) {
        renderStep(
          &pattern.steps[i], i,
          outputs[V_OUTPUT],
          outputs[SHIFT_OUTPUT],
          outputs[LEN_OUTPUT],
          outputs[EXPR_OUTPUT],
          outputs[EXPR_CURVE_OUTPUT],
          outputs[PHASE_OUTPUT]
        );
        outputs[GATE_OUTPUT].setVoltage(clutch ? 10.f : 0.f, i);
        outputs[STEP_GATE_OUTPUT + i].setVoltage(10.f);
        lights[STEP_GATE_LIGHT + i].setBrightness(1.f);
      } else {
        outputs[GATE_OUTPUT].setVoltage(0.f, i);
        outputs[STEP_GATE_OUTPUT + i].setVoltage(0.f);
        lights[STEP_GATE_LIGHT + i].setBrightness(0.f);
      }
      if (polyphonyMode == UNISON) {
        if (unisonStates[i]) {
          renderUnison(
            &pattern.steps[i], i + NUM_STEPS,
            outputs[V_OUTPUT],
            outputs[SHIFT_OUTPUT],
            outputs[LEN_OUTPUT],
            outputs[EXPR_OUTPUT],
            outputs[EXPR_CURVE_OUTPUT],
            outputs[PHASE_OUTPUT]
          );
          outputs[GATE_OUTPUT].setVoltage(clutch ? 10.f : 0.f, i + NUM_STEPS);
          outputs[STEP_GATE_OUTPUT + i].setVoltage(10.f);
          lights[STEP_GATE_LIGHT + i].setBrightness(1.f);
        } else {
          outputs[GATE_OUTPUT].setVoltage(0.f, i + NUM_STEPS);
        }
      }
    }
    if (polyphonyMode == UNISON) {
      outputs[GATE_OUTPUT].setChannels(NUM_STEPS * 2);
      outputs[V_OUTPUT].setChannels(NUM_STEPS * 2);
      outputs[SHIFT_OUTPUT].setChannels(NUM_STEPS * 2);
      outputs[LEN_OUTPUT].setChannels(NUM_STEPS * 2);
      outputs[EXPR_OUTPUT].setChannels(NUM_STEPS * 2);
      outputs[EXPR_CURVE_OUTPUT].setChannels(NUM_STEPS * 2);
      outputs[PHASE_OUTPUT].setChannels(NUM_STEPS * 2);
    } else {
      outputs[GATE_OUTPUT].setChannels(NUM_STEPS);
      outputs[V_OUTPUT].setChannels(NUM_STEPS);
      outputs[SHIFT_OUTPUT].setChannels(NUM_STEPS);
      outputs[LEN_OUTPUT].setChannels(NUM_STEPS);
      outputs[EXPR_OUTPUT].setChannels(NUM_STEPS);
      outputs[EXPR_CURVE_OUTPUT].setChannels(NUM_STEPS);
      outputs[PHASE_OUTPUT].setChannels(NUM_STEPS);
    }

    processIndicators();
  } else {
    if (activeStep) {
      renderStep(
        activeStep, 0,
        outputs[V_OUTPUT],
        outputs[SHIFT_OUTPUT],
        outputs[LEN_OUTPUT],
        outputs[EXPR_OUTPUT],
        outputs[EXPR_CURVE_OUTPUT],
        outputs[PHASE_OUTPUT]
      );
      outputs[GATE_OUTPUT].setChannels(1);
      outputs[V_OUTPUT].setChannels(1);
      outputs[SHIFT_OUTPUT].setChannels(1);
      outputs[LEN_OUTPUT].setChannels(1);
      outputs[EXPR_OUTPUT].setChannels(1);
      outputs[EXPR_CURVE_OUTPUT].setChannels(1);
      outputs[PHASE_OUTPUT].setChannels(1);

      processIndicators();

      for (int i = 0; i < NUM_STEPS; i++) {
        outputs[STEP_GATE_OUTPUT + i].setVoltage(activeStep->idx == i ? 10.0f : 0.0f);
        lights[STEP_GATE_LIGHT + i].setBrightness(activeStep->idx == i ? 1.0f : 0.0f);
      }
    } else {
      for (int i = 0; i < NUM_STEPS; i++) {
        outputs[STEP_GATE_OUTPUT + i].setVoltage(0.0f);
        lights[STEP_GATE_LIGHT + i].setBrightness(0.0f);
      }
    }
  }

  outputs[PTRN_PHASE_OUTPUT].setVoltage(phaseShifted * 10.0f);

  for (int i = 0; i < NUM_STEPS; i++) {
    lights[GATE_SWITCH_LED + i].setBrightness(pattern.steps[i].gate ^ !globalGate);
  }

  lights[PHASE_LED].setBrightness(inputs[PHASE_INPUT].isConnected());
  lights[CLOCK_LED].setBrightness(inputs[CLOCK_INPUT].isConnected());
  lights[VBPS_LED].setBrightness(inputs[VBPS_INPUT].isConnected() && !inputs[PHASE_INPUT].isConnected());

  lastPhase = phase;
  lastPhaseInState = inputs[PHASE_INPUT].isConnected();
  lastPhaseShifted = phaseShifted;
  lastPhaseParamInput = params[PHASE_PARAM].getValue();
  lastClockInputState = inputs[CLOCK_INPUT].isConnected();

  if (this->patternsDisplayConsumer->consumed && this->patternsDisplayConsumer->currentPattern != this->patternIdx) {
    this->patternsDisplayConsumer->currentPattern = this->patternIdx;
    this->patternsDisplayConsumer->currentPatternGoTo = this->pattern.goTo;
    this->patternsDisplayConsumer->consumed = false;
    for (unsigned int i = 0; i < NUM_PATTERNS; i++) {
      this->patternsDisplayConsumer->dirtyMask.set(i, this->patterns[i].hasCustomSteps());
    }
  }
  if (this->patternDisplayConsumer->consumed) {
    this->patternDisplayConsumer->phase = this->phaseShifted;
    this->patternDisplayConsumer->direction = this->direction;
    this->patternDisplayConsumer->resolution = this->pattern.resolution;
    this->patternDisplayConsumer->consumed = false;
  }
}

struct PhasequeWidget : ModuleWidget {
  PhasequeWidget(Phaseque *module);
  void appendContextMenu(Menu *menu) override;
};

PhasequeWidget::PhasequeWidget(Phaseque *module) {
  setModule(module);
  setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/panels/Phaseque.svg")));

  addInput(createInput<ZZC_PJ_Port>(Vec(14, 50), module, Phaseque::CLOCK_INPUT));
  addChild(createLight<TinyLight<GreenLight>>(Vec(36, 50), module, Phaseque::CLOCK_LED));
  addParam(createParam<ZZC_LEDBezelDark>(Vec(15.3f, 82.3f), module, Phaseque::TEMPO_TRACK_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(17.1f, 84.0f), module, Phaseque::TEMPO_TRACK_LED));
  addInput(createInput<ZZC_PJ_Port>(Vec(49, 50), module, Phaseque::VBPS_INPUT));
  addChild(createLight<TinyLight<GreenLight>>(Vec(71, 50), module, Phaseque::VBPS_LED));
  addParam(createParam<ZZC_Knob25SnappyNoRand>(Vec(49, 80.8f), module, Phaseque::BPM_PARAM));
  addInput(createInput<ZZC_PJ_Port>(Vec(84, 50), module, Phaseque::PHASE_INPUT));
  addChild(createLight<TinyLight<GreenLight>>(Vec(106, 50), module, Phaseque::PHASE_LED));
  addParam(createParam<ZZC_EncoderKnob>(Vec(258.75f, 49), module, Phaseque::PHASE_PARAM));
  addParam(createParam<ZZC_LEDBezelDark>(Vec(120.3f, 51.3f), module, Phaseque::ABS_MODE_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(122.1f, 53.0f), module, Phaseque::ABS_MODE_LED));

  Display32Widget *bpmDisplay = new Display32Widget();
  bpmDisplay->box.pos = Vec(84.0f, 83.0f);
  bpmDisplay->box.size = Vec(58.0f, 21.0f);
  if (module) {
    bpmDisplay->value = &module->bpm;
    bpmDisplay->disabled = &module->bpmDisabled;
  }
  addChild(bpmDisplay);

  addInput(createInput<ZZC_PJ_Port>(Vec(189, 81), module, Phaseque::CLUTCH_INPUT));
  addParam(createParam<ZZC_LEDBezelDark>(Vec(190.3f, 51.3f), module, Phaseque::CLUTCH_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(192.1f, 53.0f), module, Phaseque::CLUTCH_LED));

  DisplayIntpartWidget *resolutionDisplay = new DisplayIntpartWidget();
  resolutionDisplay->box.pos = Vec(152, 52);
  resolutionDisplay->box.size = Vec(29, 21);
  if (module) {
    resolutionDisplay->value = &module->resolution;
  }
  addChild(resolutionDisplay);

  addParam(createParam<ZZC_PhasequePatternResoKnob>(Vec(154, 80.8f), module, Phaseque::PATTERN_RESO_PARAM));

  addInput(createInput<ZZC_PJ_Port>(Vec(224, 81), module, Phaseque::RESET_INPUT));
  addParam(createParam<ZZC_LEDBezelDark>(Vec(225.3f, 51.3f), module, Phaseque::RESET_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(227.1f, 53.0f), module, Phaseque::RESET_LED));

  addInput(createInput<ZZC_PJ_Port>(Vec(14, 126), module, Phaseque::GOTO_INPUT));
  addInput(createInput<ZZC_PJ_Port>(Vec(49, 126), module, Phaseque::PTRN_INPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(14, 174), module, Phaseque::WENT_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(49, 174), module, Phaseque::PTRN_OUTPUT));

  GridDisplayWidget *patternsDisplayDisplay = new GridDisplayWidget();
  patternsDisplayDisplay->box.pos = Vec(84.0f, 117.0f);
  patternsDisplayDisplay->setupSize(Vec(165.0f, 85.0f));
  if (module) {
    Phaseque *phaseque = dynamic_cast<Phaseque*>(module);
    assert(phaseque);
    phaseque->patternsDisplayConsumer = patternsDisplayDisplay->consumer;
    phaseque->patternsDisplayProducer = patternsDisplayDisplay->producer;
  }
  addChild(patternsDisplayDisplay);

  addOutput(createOutput<ZZC_PJ_Port>(Vec(259, 126), module, Phaseque::PTRN_PHASE_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(294, 126), module, Phaseque::PTRN_WRAP_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(259, 174), module, Phaseque::PTRN_START_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(294, 174), module, Phaseque::PTRN_END_OUTPUT));

  addInput(createInput<ZZC_PJ_Port>(Vec(14, 220), module, Phaseque::PREV_INPUT));
  addInput(createInput<ZZC_PJ_Port>(Vec(49, 220), module, Phaseque::NEXT_INPUT));

  addInput(createInput<ZZC_PJ_Port>(Vec(84, 220), module, Phaseque::LEFT_INPUT));
  addInput(createInput<ZZC_PJ_Port>(Vec(119, 220), module, Phaseque::DOWN_INPUT));
  addInput(createInput<ZZC_PJ_Port>(Vec(154, 220), module, Phaseque::UP_INPUT));
  addInput(createInput<ZZC_PJ_Port>(Vec(189, 220), module, Phaseque::RIGHT_INPUT));
  addInput(createInput<ZZC_PJ_Port>(Vec(224, 220), module, Phaseque::SEQ_INPUT));

  addInput(createInput<ZZC_PJ_Port>(Vec(259, 220), module, Phaseque::RND_INPUT));

  addParam(createParam<ZZC_LEDBezelDark>(Vec(295.3f, 221.3f), module, Phaseque::WAIT_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_RedLight>>(Vec(297.1f, 223.0f), module, Phaseque::WAIT_LED));

  MainDisplayWidget *patternDisplay = new MainDisplayWidget();
  patternDisplay->box.pos = Vec(333.0f, 50.0f);
  patternDisplay->box.size = Vec(233.0f, 233.0f);
  patternDisplay->setupSizes();
  if (module) {
    Phaseque *phaseque = dynamic_cast<Phaseque*>(module);
    assert(phaseque);
    phaseque->patternDisplayConsumer = patternDisplay->consumer;
    // patternDisplay->resolution = &module->resolution;
    // patternDisplay->phase = &module->phaseShifted;
    // patternDisplay->pattern = &module->pattern;
    // patternDisplay->activeStep = &module->activeStep;
    // patternDisplay->direction = &module->direction;
    // patternDisplay->globalGate = &module->globalGate;
    // patternDisplay->polyphonyMode = &module->polyphonyMode;
    // patternDisplay->stepsStates = module->stepsStates;
    // patternDisplay->unisonStates = module->unisonStates;
  }
  addChild(patternDisplay);

  addInput(createInput<ZZC_PJ_Port>(Vec(259, 319.75f), module, Phaseque::GLOBAL_GATE_INPUT));
  addParam(createParam<ZZC_LEDBezelDark>(Vec(260.3f, 278.3f), module, Phaseque::GLOBAL_GATE_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(262.1f, 280.0f), module, Phaseque::GLOBAL_GATE_LED));

  addInput(createInput<ZZC_PJ_Port>(Vec(14, 319.75f), module, Phaseque::GLOBAL_SHIFT_INPUT));
  addParam(createParam<ZZC_Knob25NoRand>(Vec(14.05f, 276.9f), module, Phaseque::GLOBAL_SHIFT_PARAM));

  addInput(createInput<ZZC_PJ_Port>(Vec(49, 319.75f), module, Phaseque::GLOBAL_LEN_INPUT));
  addParam(createParam<ZZC_Knob25NoRand>(Vec(49.05f, 276.9f), module, Phaseque::GLOBAL_LEN_PARAM));

  addInput(createInput<ZZC_PJ_Port>(Vec(294, 277), module, Phaseque::GLOBAL_EXPR_CURVE_INPUT));
  addInput(createInput<ZZC_PJ_Port>(Vec(294, 319.75f), module, Phaseque::GLOBAL_EXPR_POWER_INPUT));

  addParam(createParam<ZZC_LEDBezelDark>(Vec(85.3f, 278.3f), module, Phaseque::QNT_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(87.1f, 280.0f), module, Phaseque::QNT_LED));

  addParam(createParam<ZZC_LEDBezelDark>(Vec(120.3f, 278.3f), module, Phaseque::SHIFT_LEFT_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(122.1f, 280.0f), module, Phaseque::SHIFT_LEFT_LED));

  addParam(createParam<ZZC_PhasequePatternShiftKnob>(Vec(150.5f, 273.5f), module, Phaseque::PATTERN_SHIFT_PARAM));

  addParam(createParam<ZZC_LEDBezelDark>(Vec(190.3f, 278.3f), module, Phaseque::SHIFT_RIGHT_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(192.1f, 280.0f), module, Phaseque::SHIFT_RIGHT_LED));

  addParam(createParam<ZZC_LEDBezelDark>(Vec(225.3f, 278.3f), module, Phaseque::LEN_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(227.1f, 280.0f), module, Phaseque::LEN_LED));

  addParam(createParam<ZZC_LEDBezelDark>(Vec(85.3f, 320.3f), module, Phaseque::REV_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(87.1f, 322.0f), module, Phaseque::REV_LED));

  addInput(createInput<ZZC_PJ_Port>(Vec(119, 319.75f), module, Phaseque::MUTA_DEC_INPUT));

  addParam(createParam<ZZC_PhasequeMutaKnob>(Vec(153, 318), module, Phaseque::PATTERN_MUTA_PARAM));

  addInput(createInput<ZZC_PJ_Port>(Vec(189, 319.75f), module, Phaseque::MUTA_INC_INPUT));

  addParam(createParam<ZZC_LEDBezelDark>(Vec(225.3f, 320.3f), module, Phaseque::FLIP_SWITCH_PARAM));
  addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(227.1f, 322.0f), module, Phaseque::FLIP_LED));

  addInput(createInput<ZZC_PJ_Port>(Vec(579, 319.75f), module, Phaseque::RND_JUMP_INPUT));

  float stepPeriod = 35.0f;
  float stepsAreaX = 614.0;
  for (int i = 0; i < NUM_STEPS; i++) {
    addParam(createParam<ZZC_PhasequeStepAttrKnob23>(Vec(stepsAreaX + stepPeriod * i - 0.5f, 48), module, Phaseque::STEP_VALUE_PARAM + i));
    addParam(createParam<ZZC_LEDBezelDark>(Vec(stepsAreaX + 3.3f + stepPeriod * i, 82.3f), module, Phaseque::GATE_SWITCH_PARAM + i));
    addChild(createLight<LedLight<ZZC_YellowLight>>(Vec(stepsAreaX + 5.1f + stepPeriod * i, 84.0f), module, Phaseque::GATE_SWITCH_LED + i));
    addParam(createParam<ZZC_PhasequeStepAttrKnob21>(Vec(stepsAreaX + 0.5f + stepPeriod * i, 110.5f), module, Phaseque::STEP_SHIFT_PARAM + i));
    addParam(createParam<ZZC_PhasequeStepAttrKnob21Uni>(Vec(stepsAreaX + 0.5f + stepPeriod * i, 141.5f), module, Phaseque::STEP_LEN_PARAM + i));
    addParam(createParam<ZZC_PhasequeStepAttrKnob19>(Vec(stepsAreaX + 1.5f + stepPeriod * i, 176.f), module, Phaseque::STEP_EXPR_IN_PARAM + i));

    ZZC_PhasequeXYDisplayWidget *exprDisplay = new ZZC_PhasequeXYDisplayWidget();
    exprDisplay->box.pos = Vec(stepsAreaX + 1.0f + stepPeriod * i, 207);
    exprDisplay->box.size = Vec(27, 27);
    if (module) {
      exprDisplay->paramQuantityX = module->paramQuantities[Phaseque::STEP_EXPR_POWER_PARAM + i];
      exprDisplay->paramQuantityY = module->paramQuantities[Phaseque::STEP_EXPR_CURVE_PARAM + i];
    }
    exprDisplay->setupSize();
    addParam(exprDisplay);

    addParam(createParam<ZZC_PhasequeStepAttrKnob19>(Vec(stepsAreaX + 1.5f + stepPeriod * i, 240.5f), module, Phaseque::STEP_EXPR_OUT_PARAM + i));

    addInput(createInput<ZZC_PJ_Port>(Vec(stepsAreaX + 2.0f + stepPeriod * i, 271), module, Phaseque::STEP_JUMP_INPUT + i));
    addChild(createLight<SmallLight<GreenLight>>(Vec(stepsAreaX + 11.3f + stepPeriod * i, 301.5f), module, Phaseque::STEP_GATE_LIGHT + i));
    addOutput(createOutput<ZZC_PJ_Port>(Vec(stepsAreaX + 2.0f + stepPeriod * i, 319.75f), module, Phaseque::STEP_GATE_OUTPUT + i));
  }

  stepPeriod = 34.0f;
  float outputsAreaX = 335.0f;

  addOutput(createOutput<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 0, 319.75f), module, Phaseque::GATE_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 1, 319.75f), module, Phaseque::V_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 2, 319.75f), module, Phaseque::SHIFT_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 3, 319.75f), module, Phaseque::LEN_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 4, 319.75f), module, Phaseque::EXPR_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 5, 319.75f), module, Phaseque::EXPR_CURVE_OUTPUT));
  addOutput(createOutput<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 6, 319.75f), module, Phaseque::PHASE_OUTPUT));

  float outputLedsAreaX = 344.3f;

  addChild(createLight<SmallLight<GreenLight>>(Vec(outputLedsAreaX + stepPeriod * 0, 301.5f), module, Phaseque::GATE_LIGHT));
  addChild(createLight<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 1, 301.5f), module, Phaseque::V_POS_LIGHT));
  addChild(createLight<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 2, 301.5f), module, Phaseque::SHIFT_POS_LIGHT));
  addChild(createLight<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 3, 301.5f), module, Phaseque::LEN_POS_LIGHT));
  addChild(createLight<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 4, 301.5f), module, Phaseque::EXPR_POS_LIGHT));
  addChild(createLight<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 5, 301.5f), module, Phaseque::EXPR_CURVE_POS_LIGHT));
  addChild(createLight<SmallLight<GreenLight>>(Vec(outputLedsAreaX + stepPeriod * 6, 301.5f), module, Phaseque::PHASE_LIGHT));

  addChild(createWidget<ZZC_Screw>(Vec(RACK_GRID_WIDTH, 0)));
  addChild(createWidget<ZZC_Screw>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
  addChild(createWidget<ZZC_Screw>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
  addChild(createWidget<ZZC_Screw>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
}

struct PhasequeCopyToNextItem : MenuItem {
  Phaseque *phaseq;
  void onAction(const event::Action &e) override {
    phaseq->copyToNext();
  }
};

struct PhasequeCopyToPrevItem : MenuItem {
  Phaseque *phaseq;
  void onAction(const event::Action &e) override {
    phaseq->copyToPrev();
  }
};

struct PhasequeCopyResoItem : MenuItem {
  Phaseque *phaseq;
  void onAction(const event::Action &e) override {
    phaseq->copyResoToAll();
  }
};

struct PhasequeRndAllItem : MenuItem {
  Phaseque *phaseq;
  void onAction(const event::Action &e) override {
    phaseq->randomizeAll();
  }
};

struct PhasequeRndAllResoItem : MenuItem {
  Phaseque *phaseq;
  void onAction(const event::Action &e) override {
    phaseq->randomizeAllReso();
  }
};

struct PhasequeBakeMutationItem : MenuItem {
  Phaseque *phaseq;
  void onAction(const event::Action &e) override {
    phaseq->bakeMutation();
  }
};

struct PhasequeClearPatternItem : MenuItem {
  Phaseque *phaseq;
  void onAction(const event::Action &e) override {
    phaseq->pattern.init();
  }
};

struct PolyModeValueItem : MenuItem {
  Phaseque *module;
  PolyphonyModes polyMode;
  void onAction(const event::Action &e) override {
    module->setPolyMode(polyMode);
  }
};

struct PolyModeItem : MenuItem {
  Phaseque *module;
  Menu *createChildMenu() override {
    Menu *menu = new Menu;
    std::vector<std::string> polyModeNames = {
      "Monophonic",
      "Polyphonic 8",
      "Muta-Unison 16"
    };
    for (int i = 0; i < NUM_POLYPHONY_MODES; i++) {
      PolyphonyModes polyMode = (PolyphonyModes) i;
      PolyModeValueItem *item = new PolyModeValueItem;
      item->text = polyModeNames[i];
      item->rightText = CHECKMARK(module->polyphonyMode == polyMode);
      item->module = module;
      item->polyMode = polyMode;
      menu->addChild(item);
    }
    return menu;
  }
};

void PhasequeWidget::appendContextMenu(Menu *menu) {
  menu->addChild(new MenuSeparator());

  Phaseque *phaseq = dynamic_cast<Phaseque*>(module);
  assert(phaseq);

  PhasequeCopyToNextItem *phaseqCopyToNextItem = createMenuItem<PhasequeCopyToNextItem>("Copy Steps To Next Pattern");
  PhasequeCopyToPrevItem *phaseqCopyToPrevItem = createMenuItem<PhasequeCopyToPrevItem>("Copy Steps To Prev Pattern");
  PhasequeCopyResoItem *phaseqCopyResoItem = createMenuItem<PhasequeCopyResoItem>("Copy Reso To All Patterns");
  PhasequeRndAllItem *phaseqRndAllItem = createMenuItem<PhasequeRndAllItem>("Randomize All Patterns");
  PhasequeRndAllResoItem *phaseqRndAllResoItem = createMenuItem<PhasequeRndAllResoItem>("Randomize All Resolutions");
  PhasequeBakeMutationItem *phaseqBakeMutationItem = createMenuItem<PhasequeBakeMutationItem>("Bake Mutation");
  PhasequeClearPatternItem *phaseqClearPatternItem = createMenuItem<PhasequeClearPatternItem>("Clear Pattern");
  phaseqCopyToNextItem->phaseq = phaseq;
  phaseqCopyToPrevItem->phaseq = phaseq;
  phaseqCopyResoItem->phaseq = phaseq;
  phaseqRndAllItem->phaseq = phaseq;
  phaseqRndAllResoItem->phaseq = phaseq;
  phaseqBakeMutationItem->phaseq = phaseq;
  phaseqClearPatternItem->phaseq = phaseq;
  menu->addChild(phaseqCopyToNextItem);
  menu->addChild(phaseqCopyToPrevItem);
  menu->addChild(new MenuSeparator());
  menu->addChild(phaseqCopyResoItem);
  menu->addChild(new MenuSeparator());
  menu->addChild(phaseqRndAllItem);
  menu->addChild(phaseqRndAllResoItem);
  menu->addChild(new MenuSeparator());
  menu->addChild(phaseqBakeMutationItem);
  menu->addChild(new MenuSeparator());
  menu->addChild(phaseqClearPatternItem);
  menu->addChild(new MenuSeparator());
  PolyModeItem *polyModeItem = new PolyModeItem;
  polyModeItem->text = "Polyphony";
  polyModeItem->rightText = RIGHT_ARROW;
  polyModeItem->module = phaseq;
  menu->addChild(polyModeItem);
}

Model *modelPhaseque = createModel<Phaseque, PhasequeWidget>("Phaseque");
