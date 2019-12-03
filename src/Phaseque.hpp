#pragma once
#include "rack0.hpp"
#include <algorithm> // for std:rotate

#include "ZZC.hpp"
#include "Pattern.hpp"
#include "helpers.hpp"
#include "MainDisplay.hpp"
#include "GridDisplay.hpp"

#define MAX_VOICES 16
#define NUM_PATTERNS 32

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

  Pattern patterns[NUM_PATTERNS];
  unsigned int patternIdx = 0;
  unsigned int lastPatternIdx = 0;
  Pattern pattern = patterns[patternIdx];
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

  float resolution = pattern.resolution;
  unsigned int lastGoToRequest = 0;

  int polyphonyMode = MONOPHONIC;
  bool stepsStates[NUM_STEPS] = { false };
  bool unisonStates[NUM_STEPS] = { false };

  dsp::ClockDivider lightDivider;

  int patternFlashNeg = 0;
  int patternFlashPos = 0;

  /* CommunicationWithDisplays */

  std::shared_ptr<GridDisplayConsumer> gridDisplayConsumer;
  std::shared_ptr<GridDisplayProducer> gridDisplayProducer;

  std::shared_ptr<MainDisplayConsumer> mainDisplayConsumer;

  void setPolyMode(PolyphonyModes polyMode) {
    if (polyMode == this->polyphonyMode) {
      return;
    }
    this->polyphonyMode = polyMode;
  }

  void goToPattern(unsigned int targetIdx) {
    unsigned int targetIdxSafe = eucMod(targetIdx, NUM_PATTERNS);
    storeCurrentPattern();
    this->patternIdx = targetIdxSafe;
    takeOutCurrentPattern();
  }

  void goToFirstNonEmpty() {
    for (int i = 0; i < NUM_PATTERNS; i++) {
      if (patterns[i].hasCustomSteps()) {
        goToPattern(i);
        return;
      }
    }
  }

  inline void processGlobalParams() {
    // Gates
    if (globalGateButtonTrigger.process(params[GLOBAL_GATE_SWITCH_PARAM].getValue())) {
      globalGateInternal ^= true;
    }
    if (inputs[GLOBAL_GATE_INPUT].isConnected()) {
      globalGate = globalGateInternal ^ (inputs[GLOBAL_GATE_INPUT].getVoltage() > 1.0f);
    } else {
      globalGate = globalGateInternal;
    }
    lights[GLOBAL_GATE_LED].setBrightness(globalGate);

    // Shift
    if (inputs[GLOBAL_SHIFT_INPUT].isConnected()) {
      globalShift = params[GLOBAL_SHIFT_PARAM].getValue() + clamp(inputs[GLOBAL_SHIFT_INPUT].getVoltage() * 0.2f * baseStepLen, -baseStepLen, baseStepLen);
    } else {
      globalShift = params[GLOBAL_SHIFT_PARAM].getValue();
    }

    // Length
    if (inputs[GLOBAL_LEN_INPUT].isConnected()) {
      globalLen = params[GLOBAL_LEN_PARAM].getValue() * (clamp(inputs[GLOBAL_LEN_INPUT].getVoltage(), -5.0f, 4.999f) * 0.2f + 1.0f);
    } else {
      globalLen = params[GLOBAL_LEN_PARAM].getValue();
    }
  }

  inline void processPatternNav() {
    if (waitButtonTrigger.process(params[WAIT_SWITCH_PARAM].getValue())) {
      wait ^= true;
    }
    if (this->wait) {
      lights[WAIT_LED].value = 1.1f;
    }
    if (this->gridDisplayProducer->hasNextPatternRequest) {
      this->pattern.goTo = this->gridDisplayProducer->nextPatternRequest;
      this->gridDisplayProducer->hasNextPatternRequest = false;
    }
    if (this->gridDisplayProducer->hasGoToRequest) {
      unsigned int goToRequest = this->gridDisplayProducer->goToRequest;
      this->gridDisplayProducer->hasGoToRequest = false;
      if (goToRequest != this->patternIdx) {
        this->goToPattern(goToRequest);
      }
    }
    if (this->wait) {
      return;
    }
    if (inputs[SEQ_INPUT].isConnected() && seqInputTrigger.process(inputs[SEQ_INPUT].getVoltage())) {
      if (patternIdx != pattern.goTo) {
        goToPattern(pattern.goTo);
        return;
      }
    }
    if (inputs[GOTO_INPUT].isConnected()) {
      if (goToInputTrigger.process(inputs[GOTO_INPUT].getVoltage())) {
        if (inputs[PTRN_INPUT].isConnected()) {
          unsigned int target = voltsToPattern(inputs[PTRN_INPUT].getVoltage());
          if (target != patternIdx) {
            goToPattern(target);
            this->lastGoToRequest = target;
            return;
          }
        } else {
          this->goToFirstNonEmpty();
        }
      } else if (inputs[GOTO_INPUT].getVoltage() > 1.0f) {
        unsigned int target = voltsToPattern(inputs[PTRN_INPUT].getVoltage());
        if (target != this->lastGoToRequest && target != this->patternIdx) {
          this->goToPattern(target);
          this->lastGoToRequest = target;
          return;
        }
      }
    }
    if (inputs[PREV_INPUT].isConnected() && prevPtrnInputTrigger.process(inputs[PREV_INPUT].getVoltage())) {
      for (int i = this->patternIdx - 1; i >= 0; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (unsigned int i = NUM_PATTERNS - 1; i > patternIdx; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[NEXT_INPUT].isConnected() && nextPtrnInputTrigger.process(inputs[NEXT_INPUT].getVoltage())) {
      for (unsigned int i = this->patternIdx + 1; i < NUM_PATTERNS; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (unsigned int i = 0; i < this->patternIdx; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[RND_INPUT].isConnected() && firstInputTrigger.process(inputs[RND_INPUT].getVoltage())) {
      unsigned int nonEmpty[NUM_PATTERNS];
      unsigned int idx = 0;
      for (unsigned int i = 0; i < NUM_PATTERNS; i++) {
        if (i != this->patternIdx && patterns[i].hasCustomSteps()) {
          nonEmpty[idx] = i;
          idx++;
        }
      }
      if (idx != 0) {
        unsigned int randIdx = clamp((unsigned int) (random::uniform() * idx), 0, idx);
        goToPattern(nonEmpty[randIdx]);
        return;
      }
    }
    if (inputs[LEFT_INPUT].isConnected() && leftInputTrigger.process(inputs[LEFT_INPUT].getVoltage())) {
      Limits limits = getRowLimits(this->patternIdx);
      for (int i = ((int) patternIdx) - 1; i >= (int) limits.low; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (unsigned int i = limits.high - 1; i > patternIdx; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[RIGHT_INPUT].isConnected() && rightInputTrigger.process(inputs[RIGHT_INPUT].getVoltage())) {
      Limits limits = getRowLimits(this->patternIdx);
      for (unsigned int i = patternIdx + 1; i < limits.high; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (unsigned int i = limits.low; i < this->patternIdx; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[DOWN_INPUT].isConnected() && downInputTrigger.process(inputs[DOWN_INPUT].getVoltage())) {
      Limits limits = getColumnLimits(patternIdx);
      for (int i = patternIdx - 4; i >= (int) limits.low; i -= 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = limits.high - 4; i > (int) this->patternIdx; i -= 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[UP_INPUT].isConnected() && upInputTrigger.process(inputs[UP_INPUT].getVoltage())) {
      Limits limits = getColumnLimits(patternIdx);
      for (int i = patternIdx + 4; i <= (int) limits.high; i += 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (unsigned int i = limits.low; i < patternIdx; i += 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
  }

  inline void processButtons() {
    if (tempoTrackButtonTrigger.process(params[TEMPO_TRACK_SWITCH_PARAM].getValue())) {
      tempoTrack ^= true;
      lights[TEMPO_TRACK_LED].value = tempoTrack ? 1.0f : 0.0f;
    }
    if (absModeTrigger.process(params[ABS_MODE_SWITCH_PARAM].getValue())) {
      absMode ^= true;
      lights[ABS_MODE_LED].value = absMode ? 1.0f : 0.0f;
    }
    if (clutchButtonTrigger.process(params[CLUTCH_SWITCH_PARAM].getValue()) || (inputs[CLUTCH_INPUT].isConnected() && clutchInputTrigger.process(inputs[CLUTCH_INPUT].getVoltage()))) {
      clutch ^= true;
      lights[CLUTCH_LED].value = clutch ? 1.0f : 0.0f;
    }
    resetPulse = resetButtonTrigger.process(params[RESET_SWITCH_PARAM].getValue()) || (inputs[RESET_INPUT].isConnected() && resetInputTrigger.process(inputs[RESET_INPUT].getVoltage()));
    if (resetPulse) {
      samplesSinceLastReset = 0;
      tempoTracker.reset();
    } else {
      samplesSinceLastReset++;
      if (samplesSinceLastReset == __INT_MAX__) {
        samplesSinceLastReset = 384000;
      }
    }
    if (resetPulse) {
      lights[RESET_LED].value = 1.1f;
    }
    for (int i = 0; i < NUM_STEPS; i++) {
      if (gateButtonsTriggers[i].process(params[GATE_SWITCH_PARAM + i].getValue())) {
        pattern.steps[i].gate ^= true;
      }
    }
  }

  inline void processPatternButtons() {
    if (inputs[MUTA_DEC_INPUT].isConnected()) {
      int mutaDecChannels = inputs[MUTA_DEC_INPUT].getChannels();
      if (mutaDecChannels > 1) {
        for (int i = 0; i < mutaDecChannels; i++) {
          int targetStepIdx = i % NUM_STEPS;
          if (mutRstTrigger[i].process(inputs[MUTA_DEC_INPUT].getVoltage(i))) {
            this->resetStepMutation(targetStepIdx);
          } else if (mutDecTrigger[i].process(inputs[MUTA_DEC_INPUT].getVoltage(i))) {
            this->mutateStep(targetStepIdx, -0.05f);
          }
        }
      } else {
        if (mutRstTrigger[0].process(inputs[MUTA_DEC_INPUT].getVoltage())) {
          this->resetMutation();
        } else if (mutDecTrigger[0].process(inputs[MUTA_DEC_INPUT].getVoltage())) {
          this->mutate(-0.05f);
        }
      }
    }
    if (inputs[MUTA_INC_INPUT].isConnected()) {
      int mutaIncChannels = inputs[MUTA_INC_INPUT].getChannels();
      if (mutaIncChannels > 1) {
        for (int i = 0; i < mutaIncChannels; i++) {
          if (mutIncTrigger[i].process(inputs[MUTA_INC_INPUT].getVoltage(i))) {
            int targetStepIdx = i % NUM_STEPS;
            this->mutateStep(targetStepIdx, 0.1f);
          }
        }
      } else {
        if (mutIncTrigger[0].process(inputs[MUTA_INC_INPUT].getVoltage())) {
          this->mutate(0.1f);
        }
      }
    }
    if (qntTrigger.process(params[QNT_SWITCH_PARAM].getValue())) {
      lights[QNT_LED].value = 1.1f;
      pattern.quantize();
      renderParamQuantities();
      return;
    }
    if (shiftLeftTrigger.process(params[SHIFT_LEFT_SWITCH_PARAM].getValue())) {
      lights[SHIFT_LEFT_LED].value = 1.1f;
      pattern.shiftLeft();
      renderParamQuantities();
      return;
    }
    if (shiftRightTrigger.process(params[SHIFT_RIGHT_SWITCH_PARAM].getValue())) {
      lights[SHIFT_RIGHT_LED].value = 1.1f;
      pattern.shiftRight();
      renderParamQuantities();
      return;
    }
    if (lenTrigger.process(params[LEN_SWITCH_PARAM].getValue())) {
      lights[LEN_LED].value = 1.1f;
      pattern.resetLenghts();
      renderParamQuantities();
      return;
    }
    if (revTrigger.process(params[REV_SWITCH_PARAM].getValue())) {
      lights[REV_LED].value = 1.1f;
      pattern.reverse();
      renderParamQuantities();
      return;
    }
    if (flipTrigger.process(params[FLIP_SWITCH_PARAM].getValue())) {
      lights[FLIP_LED].value = 1.1f;
      pattern.flip();
      renderParamQuantities();
      return;
    }
  }

  void jumpToStep(Step step) {
    phase = eucMod((direction == 1 ? step.in() : step.out()) - phaseParam , 1.0f);
    jump = true;
  }

  inline void processJumps() {
    jump = false;
    if (absMode || samplesSinceLastReset < 20) {
      return;
    }
    for (int i = 0; i < NUM_STEPS; i++) {
      if (inputs[STEP_JUMP_INPUT + i].isConnected() && jumpInputsTriggers[i].process(inputs[STEP_JUMP_INPUT + i].getVoltage())) {
        jumpToStep(pattern.steps[i]);
        retrigGapGenerator.trigger(1e-4f);
        return;
      }
    }
    if (inputs[RND_JUMP_INPUT].isConnected() && rndJumpInputTrigger.process(inputs[RND_JUMP_INPUT].getVoltage())) {
      int nonMuted[NUM_STEPS];
      int idx = 0;
      for (int i = 0; i < NUM_STEPS; i++) {
        if (!pattern.steps[i].gate ^ !globalGate) { continue; }
        nonMuted[idx] = i;
        idx++;
      }
      if (idx > 0) {
        jumpToStep(pattern.steps[nonMuted[(int) (random::uniform() * idx)]]);
        retrigGapGenerator.trigger(1e-4f);
      }
    }
  }

  void triggerIfBetween(float from, float to) {
    if (!activeStep) {
      return;
    }
    for (int i = 0; i < NUM_STEPS; i++) {
      if (activeStep->idx != i) { continue; }
      if (!pattern.steps[i].gate ^ !globalGate) { continue; }
      float retrigWrapped = fastmod(direction == 1 ? pattern.steps[i].in() : pattern.steps[i].out(), 1.0);
      if (from <= retrigWrapped && retrigWrapped < to) {
        retrigGapGenerator.trigger(1e-4f);
        break;
      }
    }
  }

  void renderStep(
    Step *step, int channel,
    Output &vOutput,
    Output &shiftOutput,
    Output &lenOutput,
    Output &exprOutput,
    Output &exprCurveOutput,
    Output &phaseOutput
  ) {
    float v = step->attrs[STEP_VALUE].value;
    float shift = step->attrs[STEP_SHIFT].value / baseStepLen;
    float len = step->attrs[STEP_LEN].value / baseStepLen - 1.0f;
    float stepPhase = step->phase(phaseShifted);
    float expr = step->expr(stepPhase);
    float curve = step->attrs[STEP_EXPR_CURVE].value;

    outputs[V_OUTPUT].setVoltage(v, channel);
    outputs[SHIFT_OUTPUT].setVoltage(shift * 5.0f, channel);
    outputs[LEN_OUTPUT].setVoltage(len * 5.0f, channel);
    outputs[EXPR_OUTPUT].setVoltage(expr * 5.0f, channel);
    outputs[EXPR_CURVE_OUTPUT].setVoltage(curve * 5.0f, channel);
    outputs[PHASE_OUTPUT].setVoltage(stepPhase * 10.0f, channel);
  }

  void renderUnison(
    Step *step, int channel,
    Output &vOutput,
    Output &shiftOutput,
    Output &lenOutput,
    Output &exprOutput,
    Output &exprCurveOutput,
    Output &phaseOutput
  ) {
    float v = step->attrs[STEP_VALUE].base;
    float shift = step->attrs[STEP_SHIFT].base / baseStepLen;
    float len = step->attrs[STEP_LEN].base / baseStepLen - 1.0f;
    float stepPhase = step->phaseBase(phaseShifted);
    float expr = step->exprBase(stepPhase);
    float curve = step->attrs[STEP_EXPR_CURVE].base;

    outputs[V_OUTPUT].setVoltage(v, channel);
    outputs[SHIFT_OUTPUT].setVoltage(shift * 5.0f, channel);
    outputs[LEN_OUTPUT].setVoltage(len * 5.0f, channel);
    outputs[EXPR_OUTPUT].setVoltage(expr * 5.0f, channel);
    outputs[EXPR_CURVE_OUTPUT].setVoltage(curve * 5.0f, channel);
    outputs[PHASE_OUTPUT].setVoltage(stepPhase * 10.0f, channel);
  }

  void processIndicators() {
    if (!lightDivider.process()) {
      return;
    }

    lights[GATE_LIGHT].setBrightness(outputs[GATE_OUTPUT].getVoltageSum() / 10.f);

    float v = outputs[V_OUTPUT].getVoltageSum();
    lights[V_POS_LIGHT].setBrightness(std::max(v * 0.5f, 0.0f));
    lights[V_NEG_LIGHT].setBrightness(std::max(v * -0.5f, 0.0f));

    float shift = outputs[SHIFT_OUTPUT].getVoltageSum() / 5.f;
    lights[SHIFT_POS_LIGHT].setBrightness(std::max(shift, 0.0f));
    lights[SHIFT_NEG_LIGHT].setBrightness(std::max(-shift, 0.0f));

    float len = outputs[LEN_OUTPUT].getVoltageSum() / 5.f;
    lights[LEN_POS_LIGHT].setBrightness(std::max(len, 0.0f));
    lights[LEN_NEG_LIGHT].setBrightness(std::max(-len, 0.0f));

    float expr = outputs[EXPR_OUTPUT].getVoltageSum() / 5.f;
    lights[EXPR_POS_LIGHT].setBrightness(std::max(expr, 0.0f));
    lights[EXPR_NEG_LIGHT].setBrightness(std::max(-expr, 0.0f));

    float curve = outputs[EXPR_CURVE_OUTPUT].getVoltageSum() / 5.f;
    lights[EXPR_CURVE_POS_LIGHT].setBrightness(std::max(curve, 0.0f));
    lights[EXPR_CURVE_NEG_LIGHT].setBrightness(std::max(-curve, 0.0f));

    float stepPhase = outputs[PHASE_OUTPUT].getVoltageSum() / 5.f;
    lights[PHASE_LIGHT].setBrightness(stepPhase);
  }

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
    this->refreshPatternPointers();
    lights[TEMPO_TRACK_LED].value = tempoTrack ? 1.0f : 0.0f;
    lights[ABS_MODE_LED].value = absMode ? 1.0f : 0.0f;
    lights[CLUTCH_LED].value = clutch ? 1.0f : 0.0f;
    lightDivider.setDivision(256);
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
    this->pattern.steps[stepIdx].mutate(factor);
  }
  void scaleMutation(float factor) {
    this->pattern.scaleMutation(factor);
  }
  void resetMutation() {
    this->pattern.resetMutation();
  }
  void resetStepMutation(int stepIdx) {
    this->pattern.steps[stepIdx].resetMutation();
  }
  void bakeMutation() {
    this->pattern.bakeMutation();
  }

  void showCustomSteps() {
    if (this->gridDisplayConsumer) {
      this->gridDisplayConsumer->dirtyMask.set(this->patternIdx, this->pattern.hasCustomSteps());
    }
  }

  void setStepAttr(int stepNumber, int attr, float factor) {
    this->pattern.steps[stepNumber].setAttr(attr, factor);
    this->showCustomSteps();
  }

  void setStepAttrAbs(int stepNumber, int attr, float target) {
    this->pattern.steps[stepNumber].setAttrAbs(attr, target);
    this->showCustomSteps();
  }

  void setStepAttrBase(int stepNumber, int attr, float target) {
    this->pattern.steps[stepNumber].setAttrBase(attr, target);
    this->showCustomSteps();
  }

  void resetStepAttr(int stepNumber, int attr) {
    this->pattern.steps[stepNumber].resetAttr(attr);
    this->showCustomSteps();
  }

  void setPatternReso(float target) {
    this->pattern.resolution = roundf(clamp(target, 1.0f, 99.0f));
  }
  void resetPatternReso() {
    this->pattern.resolution = 8.0f;
  }

  void adjPatternShift(float factor) {
    this->pattern.shift = clamp(this->pattern.shift + factor, -baseStepLen, baseStepLen);
  }
  void setPatternShift(float value) {
    this->pattern.shift = clamp(value * baseStepLen, -baseStepLen, baseStepLen);
  }
  void resetPatternShift() {
    this->pattern.shift = 0.0f;
  }

  void copyToNext() {
    int target = eucMod(this->patternIdx + 1, NUM_PATTERNS);
    for (int i = 0; i < NUM_STEPS; i++) {
      patterns[target].steps[i] = pattern.steps[i];
    }
    patterns[target].resolution = pattern.resolution;
  }

  void copyToPrev() {
    int target = eucMod(((int) this->patternIdx) - 1, NUM_PATTERNS);
    for (int i = 0; i < NUM_STEPS; i++) {
      patterns[target].steps[i] = pattern.steps[i];
    }
    patterns[target].resolution = pattern.resolution;
  }

  void copyResoToAll() {
    for (int i = 0; i < NUM_PATTERNS; i++) {
      patterns[i].resolution = pattern.resolution;
    }
  }

  void refreshPatternPointers() {
    pattern.refreshPointers(
      &globalShift,
      &globalLen,
      &inputs[GLOBAL_EXPR_CURVE_INPUT],
      &inputs[GLOBAL_EXPR_POWER_INPUT]
    );
  }

  void storeCurrentPattern() {
    patterns[patternIdx] = pattern;
  }

  void takeOutCurrentPattern() {
    pattern = patterns[patternIdx];
    renderParamQuantities();
    refreshPatternPointers();
  }

  void renderParamQuantities() {
    paramQuantities[PATTERN_RESO_PARAM]->setValue(pattern.resolution);
    paramQuantities[PATTERN_SHIFT_PARAM]->setValue(pattern.shift / baseStepLen);
    for (int s = 0; s < NUM_STEPS; s++) {
      for (int a = 0; a < STEP_ATTRS_TOTAL; a++) {
        paramQuantities[STEP_VALUE_PARAM + a * NUM_STEPS + s]->setValue(pattern.steps[s].attrs[a].base);
      }
    }
  }

  void onReset() override {
    for (int i = 0; i < NUM_PATTERNS; i++) {
      patterns[i].init();
      patterns[i].goTo = eucMod(i + 1, NUM_PATTERNS);
    }
    patternIdx = 0;
    takeOutCurrentPattern();
    polyphonyMode = PolyphonyModes::MONOPHONIC;
  }

  void onRandomize() override {
    pattern.randomize();
    renderParamQuantities();
  }

  json_t *dataToJson() override {
    storeCurrentPattern();
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
      tempoTrack = json_boolean_value(tempoTrackJ);
    }
    lights[TEMPO_TRACK_LED].value = tempoTrack ? 1.0f : 0.0f;
    if (absModeJ) {
      absMode = json_boolean_value(absModeJ);
    }
    lights[ABS_MODE_LED].value = absMode ? 1.0f : 0.0f;
    if (clutchJ) {
      clutch = json_boolean_value(clutchJ);
    }
    lights[CLUTCH_LED].value = clutch ? 1.0f : 0.0f;
    if (globalGateInternalJ) {
      globalGateInternal = json_boolean_value(globalGateInternalJ);
    }
    if (json_integer_value(patternIdxJ) != 0) {
      patternIdx = json_integer_value(patternIdxJ);
    }
    if (waitJ) {
      wait = json_boolean_value(waitJ);
    }
    if (polyphonyModeJ) {
      polyphonyMode = json_integer_value(polyphonyModeJ);
    }

    json_t *patternsJ = json_object_get(rootJ, "patterns");
    if (patternsJ) {
      size_t arraySize = json_array_size(patternsJ);
      int offset = arraySize - NUM_PATTERNS;
      for (unsigned int i = offset; i < arraySize && (i - offset) < NUM_PATTERNS; i++) {
        json_t *patternJ = json_array_get(patternsJ, i);
        if (!json_is_null(patternJ)) {
          patterns[i - offset].dataFromJson(patternJ);
        }
      }
    }

    takeOutCurrentPattern();
  }
};
