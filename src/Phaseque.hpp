#pragma once
#include "rack0.hpp"
#include <algorithm> // for std:rotate

#include "ZZC.hpp"
#include "Pattern.hpp"
#include "MainDisplay.hpp"
#include "GridDisplay.hpp"

#include "helpers.hpp"

#define MAX_VOICES 16
#define NUM_PATTERNS 32

template<size_t c>
struct ForLoop {
  template<template <size_t> class Func>
  static void iterate(Module *module) {
    Func<c>()(module);
    if (c > 0) {}
    ForLoop<c-1>::template iterate<Func>(module);
  }
};

template<>
struct ForLoop<0> {
  template<template <size_t> class Func>
  static void iterate(Module *module) {
    Func<0>()(module);
  }
};

struct StepAttrParamQuantityBase : ParamQuantity {
  int item;
  int attr;
};

struct Phaseque : Module {
  enum ParamIds {
    PHASE_PARAM,
    TEMPO_TRACK_SWITCH_PARAM,
    BPM_PARAM,
    ABS_MODE_SWITCH_PARAM,
    CLUTCH_SWITCH_PARAM,
    RESET_SWITCH_PARAM,
    ENUMS(GATE_SWITCH_PARAM, NUM_STEPS),
    QNT_SWITCH_PARAM,
    SHIFT_LEFT_SWITCH_PARAM,
    SHIFT_RIGHT_SWITCH_PARAM,
    LEN_SWITCH_PARAM,
    MUT_SWITCH_PARAM,
    RND_SWITCH_PARAM,
    CLR_SWITCH_PARAM,
    REV_SWITCH_PARAM,
    FLIP_SWITCH_PARAM,
    GLOBAL_GATE_SWITCH_PARAM,
    GLOBAL_SHIFT_PARAM,
    GLOBAL_LEN_PARAM,
    WAIT_SWITCH_PARAM,
    PATTERN_RESO_PARAM,
    PATTERN_SHIFT_PARAM,
    PATTERN_MUTA_PARAM,
    ENUMS(STEP_VALUE_PARAM, NUM_STEPS),
    ENUMS(STEP_LEN_PARAM, NUM_STEPS),
    ENUMS(STEP_SHIFT_PARAM, NUM_STEPS),
    ENUMS(STEP_EXPR_IN_PARAM, NUM_STEPS),
    ENUMS(STEP_EXPR_CURVE_PARAM, NUM_STEPS),
    ENUMS(STEP_EXPR_POWER_PARAM, NUM_STEPS),
    ENUMS(STEP_EXPR_OUT_PARAM, NUM_STEPS),
    NUM_PARAMS
  };
  enum InputIds {
    CLOCK_INPUT,
    VBPS_INPUT,
    PHASE_INPUT,
    CLUTCH_INPUT,
    RESET_INPUT,
    GOTO_INPUT,
    PTRN_INPUT,
    FIRST_INPUT,
    RND_INPUT,
    LEFT_INPUT,
    DOWN_INPUT,
    UP_INPUT,
    RIGHT_INPUT,
    SEQ_INPUT,
    PREV_INPUT,
    NEXT_INPUT,
    GLOBAL_GATE_INPUT,
    GLOBAL_SHIFT_INPUT,
    GLOBAL_LEN_INPUT,
    GLOBAL_EXPR_CURVE_INPUT,
    GLOBAL_EXPR_POWER_INPUT,
    ENUMS(STEP_JUMP_INPUT, NUM_STEPS),
    RND_JUMP_INPUT,
    MUTA_DEC_INPUT,
    MUTA_INC_INPUT,
    NUM_INPUTS
  };
  enum OutputIds {
    GATE_OUTPUT,
    V_OUTPUT,
    SHIFT_OUTPUT,
    LEN_OUTPUT,
    EXPR_OUTPUT,
    EXPR_CURVE_OUTPUT,
    PHASE_OUTPUT,
    WENT_OUTPUT,
    PTRN_OUTPUT,
    ENUMS(STEP_GATE_OUTPUT, NUM_STEPS),
    PTRN_START_OUTPUT,
    PTRN_PHASE_OUTPUT,
    PTRN_END_OUTPUT,
    PTRN_WRAP_OUTPUT,
    NUM_OUTPUTS
  };
  enum LightIds {
    TEMPO_TRACK_LED,
    ABS_MODE_LED,
    CLUTCH_LED,
    RESET_LED,
    ENUMS(GATE_SWITCH_LED, NUM_STEPS),
    GLOBAL_GATE_LED,
    QNT_LED,
    SHIFT_LEFT_LED,
    SHIFT_RIGHT_LED,
    LEN_LED,
    MUT_LED,
    RND_LED,
    CLR_LED,
    REV_LED,
    FLIP_LED,
    GATE_LIGHT,
    V_POS_LIGHT,
    V_NEG_LIGHT,
    SHIFT_POS_LIGHT,
    SHIFT_NEG_LIGHT,
    LEN_POS_LIGHT,
    LEN_NEG_LIGHT,
    EXPR_POS_LIGHT,
    EXPR_NEG_LIGHT,
    EXPR_CURVE_POS_LIGHT,
    EXPR_CURVE_NEG_LIGHT,
    PHASE_LIGHT,
    ENUMS(STEP_GATE_LIGHT, NUM_STEPS),
    CLOCK_LED,
    VBPS_LED,
    PHASE_LED,
    WAIT_LED,
    NUM_LIGHTS
  };

  Pattern<8, 4> patterns[NUM_PATTERNS];
  unsigned int patternIdx = 0;
  unsigned int lastPatternIdx = 0;
  Pattern<8, 4> pattern = patterns[patternIdx];
  Step* activeStep = nullptr;
  Step* lastActiveStep = nullptr;
  int goToRequest = 0;

  dsp::SchmittTrigger clockTrigger;
  TempoTracker tempoTracker;
  bool lastClockInputState = false;
  bool tickedAtLastSample = false;

  dsp::SchmittTrigger absModeTrigger;
  bool absMode = false;

  dsp::SchmittTrigger tempoTrackButtonTrigger;
  bool tempoTrack = true;

  dsp::SchmittTrigger clutchButtonTrigger;
  dsp::SchmittTrigger clutchInputTrigger;
  bool clutch = true;

  dsp::SchmittTrigger resetButtonTrigger;
  dsp::SchmittTrigger resetInputTrigger;
  bool resetPulse = false;

  dsp::SchmittTrigger goToInputTrigger;
  dsp::SchmittTrigger seqInputTrigger;
  dsp::PulseGenerator wentPulseGenerator;
  dsp::SchmittTrigger firstInputTrigger;
  dsp::SchmittTrigger rndInputTrigger;

  dsp::SchmittTrigger leftInputTrigger;
  dsp::SchmittTrigger downInputTrigger;
  dsp::SchmittTrigger upInputTrigger;
  dsp::SchmittTrigger rightInputTrigger;

  dsp::PulseGenerator retrigGapGenerator;
  bool retrigGap = false;

  dsp::SchmittTrigger globalGateButtonTrigger;
  bool globalGate = true;
  bool globalGateInternal = true;
  float globalShift = 0.0f;
  float globalLen = 1.0f;

  dsp::SchmittTrigger gateButtonsTriggers[NUM_STEPS];
  dsp::SchmittTrigger jumpInputsTriggers[NUM_STEPS];
  dsp::SchmittTrigger rndJumpInputTrigger;

  dsp::SchmittTrigger prevPtrnInputTrigger;
  dsp::SchmittTrigger nextPtrnInputTrigger;

  dsp::SchmittTrigger qntTrigger;
  dsp::SchmittTrigger shiftLeftTrigger;
  dsp::SchmittTrigger shiftRightTrigger;
  dsp::SchmittTrigger lenTrigger;

  dsp::SchmittTrigger revTrigger;
  dsp::SchmittTrigger flipTrigger;

  NegSchmittTrigger mutRstTrigger[MAX_VOICES];
  dsp::SchmittTrigger mutDecTrigger[MAX_VOICES];
  dsp::SchmittTrigger mutIncTrigger[MAX_VOICES];

  dsp::SchmittTrigger waitButtonTrigger;
  bool wait = false;

  dsp::PulseGenerator ptrnStartPulseGenerator;
  dsp::PulseGenerator ptrnEndPulseGenerator;

  double phase = 0.0;
  double lastPhase = 0.0;
  double lastPhaseIn = 0.0;
  double lastPhaseInDelta = 0.0;
  bool lastPhaseInState = false;
  int samplesSinceLastReset = 0;
  float phaseShifted = 0.0f;
  float lastPhaseShifted = 0.0f;
  float phaseParam = 0.0f;
  float lastPhaseParamInput = 0.0f;
  int direction = 1;
  bool jump = false;
  float bps = 2.0f;
  float bpm = 120.0f;
  bool bpmDisabled = false;

  unsigned int resolution = pattern.resolution;
  float resolutionDisplay = pattern.resolution;
  unsigned int lastGoToRequest = 0;

  int polyphonyMode = MONOPHONIC;
  bool stepsStates[NUM_STEPS] = { false };
  bool unisonStates[NUM_STEPS] = { false };

  dsp::ClockDivider lightDivider;
  dsp::ClockDivider buttonsDivider;

  int patternFlashNeg = 0;
  int patternFlashPos = 0;

  /* CommunicationWithDisplays */
  std::shared_ptr<GridDisplayConsumer> gridDisplayConsumer;
  std::shared_ptr<GridDisplayProducer> gridDisplayProducer;
  std::shared_ptr<MainDisplayConsumer> mainDisplayConsumer;

  void setPolyMode(PolyphonyModes polyMode);
  void goToPattern(unsigned int targetIdx);
  void goToFirstNonEmpty();
  void jumpToStep(Step step);

  void processGlobalParams();
  void processPatternNav();
  void processButtons();
  void processClutchAndReset();
  void processPatternButtons();
  void processMutaInputs();
  void processJumpInputs();
  void processIndicators();
  bool processPhaseParam(float sampleTime);
  void findActiveSteps();
  void feedDisplays();

  void processTransport(bool phaseWasZeroed, float sampleTime);

  void triggerIfBetween(float from, float to);

  void renderStep(Step *step, int channel);
  void renderUnison(Step *step, int channel);

  struct PatternResoParamQuantity : ParamQuantity {
    void setValue(float value) override {
      if (!module)
        return;
      Phaseque* phaseq = static_cast<Phaseque*>(module);
      value = math::clampSafe(value, getMinValue(), getMaxValue());
      phaseq->setPatternReso(value);
      APP->engine->setParam(module, paramId, value);
    }
  };

  struct PatternShiftParamQuantity : ParamQuantity {
    void setValue(float value) override {
      if (!module)
        return;
      Phaseque* phaseq = static_cast<Phaseque*>(module);
      value = math::clampSafe(value, getMinValue(), getMaxValue());
      phaseq->setPatternShift(value);
      APP->engine->setParam(module, paramId, value);
    }
  };

  struct PatternMutaParamQuantity : ParamQuantity {
    void setValue(float value) override {
      if (!module)
        return;
      float delta = value - getValue();
      Phaseque* phaseq = static_cast<Phaseque*>(module);
      phaseq->mutate(delta);
      APP->engine->setParam(module, paramId, value);
    }
  };

  template <int ITEM, int ATTR>
  struct StepAttrParamQuantity : StepAttrParamQuantityBase {
    StepAttrParamQuantity() {
      item = ITEM;
      attr = ATTR;
    }

    void setValue(float value) override {
      if (!module)
        return;
      Phaseque* phaseq = static_cast<Phaseque*>(module);
      value = math::clampSafe(value, getMinValue(), getMaxValue());
      phaseq->setStepAttrBase(item, attr, value);
      APP->engine->setParam(module, paramId, value);
    }
  };

  template <size_t size>
  struct StepParamConfigurator {
    void operator()(Module *module) {
      module->configParam(GATE_SWITCH_PARAM + size, 0.0f, 1.0f, 0.0f, "Step Gate");
      module->configParam<StepAttrParamQuantity<size, STEP_VALUE>>(STEP_VALUE_PARAM + size, -2.0f, 2.0f, 0.0f, "Step Value");
      module->configParam<StepAttrParamQuantity<size, STEP_LEN>>(STEP_LEN_PARAM + size, 0.0f, 1.0f / NUM_STEPS * 2.0f, 1.0f / NUM_STEPS, "Step Length");
      module->configParam<StepAttrParamQuantity<size, STEP_SHIFT>>(STEP_SHIFT_PARAM + size, -1.0f / NUM_STEPS, 1.0f / NUM_STEPS, 0.0f, "Step Shift");
      module->configParam<StepAttrParamQuantity<size, STEP_EXPR_IN>>(STEP_EXPR_IN_PARAM + size, -1.0f, 1.0f, 0.0f, "Step Expression In");
      module->configParam<StepAttrParamQuantity<size, STEP_EXPR_CURVE>>(STEP_EXPR_CURVE_PARAM + size, -1.0f, 1.0f, 0.0f, "Step Expression Curve");
      module->configParam<StepAttrParamQuantity<size, STEP_EXPR_POWER>>(STEP_EXPR_POWER_PARAM + size, -1.0f, 1.0f, 0.0f, "Step Expression Power");
      module->configParam<StepAttrParamQuantity<size, STEP_EXPR_OUT>>(STEP_EXPR_OUT_PARAM + size, -1.0f, 1.0f, 0.0f, "Step Expression Out");
    }
  };

  Phaseque() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(TEMPO_TRACK_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Sync inter-beat phase with clock");
    configParam(BPM_PARAM, 0.0, 240.0, 120.0, "Inter-beat BPM override");
    configParam(PHASE_PARAM, 0.0, 1.0, 0.0, "Manual Phase Control");
    configParam(ABS_MODE_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Absolute Phase Input");
    configParam(CLUTCH_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Clutch Transport with Phase");
    configParam(RESET_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Reset Phase");
    configParam(WAIT_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Disable CV-driven Pattern Navigation");
    configParam(GLOBAL_GATE_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Global Gate");
    configParam(GLOBAL_SHIFT_PARAM, -baseStepLen, baseStepLen, 0.0, "Global Pattern Shift");
    configParam(GLOBAL_LEN_PARAM, 0.0, 2.0, 1.0, "Global Step Length Modifier");
    configParam(SHIFT_LEFT_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Shift Pattern to the Left by 1/8");
    configParam(SHIFT_RIGHT_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Shift Pattern to the Right by 1/8");
    configParam(LEN_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Quantize Steps Length");
    configParam(REV_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Reverse Pattern");
    configParam(FLIP_SWITCH_PARAM, 0.0f, 1.0f, 0.0f, "Flip Pattern");
    configParam<PatternResoParamQuantity>(PATTERN_RESO_PARAM, 1.0f, 99.0f, 8.0f, "Pattern Resolution");
    configParam<PatternShiftParamQuantity>(PATTERN_SHIFT_PARAM, -1.0f, 1.0f, 0.0f, "Pattern Shift");
    configParam<PatternMutaParamQuantity>(PATTERN_MUTA_PARAM, -INFINITY, INFINITY, 0.0f, "Pattern Mutation");
    ForLoop<NUM_STEPS-1>::iterate<StepParamConfigurator>(this);
    for (int i = 0; i < NUM_PATTERNS; i++) {
      patterns[i].init();
      patterns[i].goTo = eucMod(i + 1, NUM_PATTERNS);
    }
    this->patternIdx = 0;
    takeOutCurrentPattern();
    lights[TEMPO_TRACK_LED].value = tempoTrack ? 1.0f : 0.0f;
    lights[ABS_MODE_LED].value = absMode ? 1.0f : 0.0f;
    lights[CLUTCH_LED].value = clutch ? 1.0f : 0.0f;
    lightDivider.setDivision(256);
    buttonsDivider.setDivision(256);
  }

  void process(const ProcessArgs &args) override;

  void randomizeAll() {
    storeCurrentPattern();
    for (int i = 0; i < NUM_PATTERNS; i++) {
      patterns[i].randomize();
    }
    takeOutCurrentPattern();
    this->gridDisplayConsumer->dirtyMask.set();
    this->gridDisplayConsumer->consumed = false;
  }

  void randomizeAllReso() {
    storeCurrentPattern();
    for (int i = 0; i < NUM_PATTERNS; i++) {
      patterns[i].randomizeReso();
    }
    takeOutCurrentPattern();
  }

  void mutate(float factor) {
    this->pattern.mutate(factor);
  }
  void mutateStep(int stepIdx, float factor) {
    // TODO: implement this
    // this->pattern.steps[stepIdx].mutate(factor);
  }
  void scaleMutation(float factor) {
    // TODO: implement this
    // this->pattern.scaleMutation(factor);
  }
  void resetMutation() {
    // TODO: implement this
    // this->pattern.resetMutation();
  }
  void resetStepMutation(int stepIdx) {
    // TODO: implement this
    // this->pattern.steps[stepIdx].resetMutation();
  }
  void bakeMutation() {
    // TODO: implement this
    // this->pattern.bakeMutation();
  }

  void showCustomSteps() {
    // TODO: implement this
    // if (this->gridDisplayConsumer) {
    //   this->gridDisplayConsumer->dirtyMask.set(this->patternIdx, this->pattern.hasCustomSteps());
    // }
  }

  void setStepAttrBase(int stepNumber, int attr, float target) {
    // TODO: implement this
    unsigned int blockIdx = stepNumber / this->pattern.blockSize;
    unsigned int stepInBlockIdx = stepNumber % this->pattern.blockSize;
    this->pattern.stepBases[attr][blockIdx][stepInBlockIdx] = target;
    if (attr == StepAttr::STEP_SHIFT || attr == StepAttr::STEP_LEN) {
      this->pattern.recalcInOuts(blockIdx);
    }
    // this->pattern.steps[stepNumber].setAttrBase(attr, target);
    // this->showCustomSteps();
  }

  void resetStepAttr(int stepNumber, int attr) {
    // TODO: implement this
    // this->pattern.steps[stepNumber].resetAttr(attr);
    // this->showCustomSteps();
  }

  void setPatternReso(float target) {
    this->pattern.resolution = std::round(clamp(target, 1.0f, 99.0f));
  }

  void resetPatternReso() {
    this->pattern.resetResolution();
  }

  void adjPatternShift(float factor) {
    // TODO: implement this
    // this->pattern.shift = clamp(this->pattern.shift + factor, -baseStepLen, baseStepLen);
  }
  void setPatternShift(float value) {
    this->pattern.shift = clamp(value * this->pattern.baseStepLen, -this->pattern.baseStepLen, this->pattern.baseStepLen);
  }
  void resetPatternShift() {
    this->pattern.shift = 0.f;
  }

  void copyToNext() {
    unsigned int target = eucMod(this->patternIdx + 1, NUM_PATTERNS);
    this->copyTo(target);
  }

  void copyToPrev() {
    unsigned int target = eucMod(this->patternIdx - 1, NUM_PATTERNS);
    this->copyTo(target);
  }

  void copyTo(unsigned int target) {
    std::memcpy(
      this->patterns[target].stepBases,
      this->pattern.stepBases,
      sizeof(this->pattern.stepBases)
    );
    std::memcpy(
      this->patterns[target].stepMutas,
      this->pattern.stepMutas,
      sizeof(this->pattern.stepMutas)
    );
    std::memcpy(
      this->patterns[target].stepGates,
      this->pattern.stepGates,
      sizeof(this->pattern.stepGates)
    );
    patterns[target].resolution = pattern.resolution;
  }

  void copyResoToAll() {
    for (unsigned int i = 0; i < NUM_PATTERNS; i++) {
      patterns[i].resolution = pattern.resolution;
    }
  }

  void storeCurrentPattern() {
    this->patterns[this->patternIdx] = this->pattern;
  }

  void takeOutCurrentPattern() {
    this->pattern = this->patterns[patternIdx];
    this->renderParamQuantities();
  }

  void renderParamQuantities() {
    // TODO: implement this
    this->paramQuantities[PATTERN_RESO_PARAM]->setValue(this->pattern.resolution);
    this->paramQuantities[PATTERN_SHIFT_PARAM]->setValue(this->pattern.shift / this->pattern.baseStepLen);
    for (unsigned int stepIdx = 0; stepIdx < this->pattern.size; stepIdx++) {
      unsigned int blockIdx = stepIdx / this->pattern.blockSize;
      unsigned int stepInBlockIdx = stepIdx % this->pattern.blockSize;
      for (unsigned int attrIdx = 0; attrIdx < STEP_ATTRS_TOTAL; attrIdx++) {
        paramQuantities[STEP_VALUE_PARAM + stepIdx + attrIdx * this->pattern.size]->setValue(
          this->pattern.stepBases[attrIdx][blockIdx][stepInBlockIdx]
        );
      }
    }
    // for (int s = 0; s < NUM_STEPS; s++) {
    //   for (int a = 0; a < STEP_ATTRS_TOTAL; a++) {
    //     paramQuantities[STEP_VALUE_PARAM + a * NUM_STEPS + s]->setValue(pattern.steps[s].attrs[a].base);
    //   }
    // }
  }

  void onReset() override {
    for (int i = 0; i < NUM_PATTERNS; i++) {
      this->patterns[i].init();
      this->patterns[i].goTo = eucMod(i + 1, NUM_PATTERNS);
    }
    this->patternIdx = 0;
    this->takeOutCurrentPattern();
    this->polyphonyMode = PolyphonyModes::MONOPHONIC;
  }

  void onRandomize() override {
    this->pattern.randomize();
    this->renderParamQuantities();
  }

  json_t *dataToJson() override {
    this->storeCurrentPattern();
    json_t *rootJ = json_object();
    json_object_set_new(rootJ, "tempoTrack", json_boolean(tempoTrack));
    json_object_set_new(rootJ, "absMode", json_boolean(absMode));
    json_object_set_new(rootJ, "clutch", json_boolean(clutch));
    json_object_set_new(rootJ, "globalGateInternal", json_boolean(globalGateInternal));
    json_object_set_new(rootJ, "patternIdx", json_integer(patternIdx));
    json_object_set_new(rootJ, "wait", json_boolean(wait));
    json_object_set_new(rootJ, "polyphonyMode", json_integer(polyphonyMode));

    json_t *patternsJ = json_array();
    for (int i = 0; i < NUM_PATTERNS; i++) {
      if (patterns[i].isClean()) {
        json_array_append_new(patternsJ, json_null());
      } else {
        json_array_append(patternsJ, patterns[i].dataToJson());
      }
    }
    json_object_set_new(rootJ, "patterns", patternsJ);

    return rootJ;
  }

  void dataFromJson(json_t *rootJ) override {
    json_t *tempoTrackJ = json_object_get(rootJ, "tempoTrack");
    json_t *absModeJ = json_object_get(rootJ, "absMode");
    json_t *clutchJ = json_object_get(rootJ, "clutch");
    json_t *globalGateInternalJ = json_object_get(rootJ, "globalGateInternal");
    json_t *patternIdxJ = json_object_get(rootJ, "patternIdx");
    json_t *waitJ = json_object_get(rootJ, "wait");
    json_t *polyphonyModeJ = json_object_get(rootJ, "polyphonyMode");
    if (tempoTrackJ) {
      this->tempoTrack = json_boolean_value(tempoTrackJ);
    }
    lights[TEMPO_TRACK_LED].value = tempoTrack ? 1.0f : 0.0f;
    if (absModeJ) {
      this->absMode = json_boolean_value(absModeJ);
    }
    lights[ABS_MODE_LED].value = absMode ? 1.0f : 0.0f;
    if (clutchJ) {
      this->clutch = json_boolean_value(clutchJ);
    }
    lights[CLUTCH_LED].value = clutch ? 1.0f : 0.0f;
    if (globalGateInternalJ) {
      this->globalGateInternal = json_boolean_value(globalGateInternalJ);
    }
    if (json_integer_value(patternIdxJ) != 0) {
      this->patternIdx = json_integer_value(patternIdxJ);
    }
    if (waitJ) {
      this->wait = json_boolean_value(waitJ);
    }
    if (polyphonyModeJ) {
      this->polyphonyMode = json_integer_value(polyphonyModeJ);
    }

    json_t *patternsJ = json_object_get(rootJ, "patterns");
    if (patternsJ) {
      size_t arraySize = json_array_size(patternsJ);
      int offset = arraySize - NUM_PATTERNS;
      for (unsigned int i = offset; i < arraySize && (i - offset) < NUM_PATTERNS; i++) {
        json_t *patternJ = json_array_get(patternsJ, i);
        if (!json_is_null(patternJ)) {
          this->patterns[i - offset].dataFromJson(patternJ);
        }
      }
    }

    this->takeOutCurrentPattern();
  }
};
