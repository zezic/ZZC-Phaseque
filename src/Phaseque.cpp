#include "ZZC.hpp"

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../ZZC/src/widgets.hpp"
#endif

#include <ctime>

#ifndef PHASEQ_H
#define PHASEQ_H
#include "Phaseque.hpp"
#endif

#include "PhasequeDisplay.hpp"
#include "PhasequePatternsDisplay.hpp"

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

struct NegSchmittTrigger : dsp::SchmittTrigger {
  bool process(float in) {
    if (state) {
      if (in >= 0.f) {
        state = false;
      }
    }
    else {
      if (in <= -1.f) {
        state = true;
        return true;
      }
    }
    return false;
  }
};

inline float patternToVolts(int idx) {
  return (idx - 1) * 1.0f / 12.0f;
}

inline unsigned int voltsToPattern(float volts) {
  return clamp((int) std::floor((volts * 12.f)), 0, NUM_PATTERNS);
}

struct Limits {
  unsigned int low;
  unsigned int high;
};

Limits getRowLimits(int idx) {
  unsigned int rowIdx = (idx) / 4;
  Limits limits;
  limits.low = rowIdx * 4;
  limits.high = (rowIdx + 1) * 4;
  return limits;
}

Limits getColumnLimits(int idx) {
  unsigned int baseIdx = idx % 4;
  Limits limits;
  limits.low = baseIdx;
  limits.high = baseIdx + 8 * 4;
  return limits;
}

struct TempoTracker {
  int ticksTracked = 0;
  float delta = 0.0f;
  bool detected = false;
  float bps = 0.0f;

  void reset() {
    ticksTracked = 0;
    delta = 0.0f;
    detected = false;
    bps = 0.0f;
  }

  void tick(float sampleTime) {
    delta += sampleTime;
    ticksTracked++;
    if (ticksTracked > 1) {
      detected = true;
      bps = 1.0f / delta;
    }
    delta = 0.0f;
  }

  void acc(float sampleTime) {
    delta += sampleTime;
  }
};

struct StepAttrParamQuantityBase : ParamQuantity {
  int item;
  int attr;

  StepAttrParamQuantityBase() {
  }
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

  PatternsDisplayConsumer *patternsDisplayConsumer = nullptr;
  PatternsDisplayProducer *patternsDisplayProducer = nullptr;

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
    if (wait) {
      lights[WAIT_LED].value = 1.1f;
    }
    if (this->patternsDisplayProducer) {
      if (this->patternsDisplayProducer->hasRequest) {
        if (this->patternsDisplayProducer->goToRequest != this->patternIdx) {
          this->goToPattern(this->patternsDisplayProducer->goToRequest);
        }
        this->patternsDisplayProducer->hasRequest = false;
      }
    }
    // if (goToRequest != 0 && goToRequest != patternIdx) {
    //   goToPattern(goToRequest);
    //   goToRequest = 0;
    //   return;
    // }
    if (wait) {
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
      for (unsigned int i = patternIdx - 1; i >= limits.low; i--) {
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
      for (int i = limits.high - 4; i > (int) this->patternIdx; i -= 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = patternIdx - 4; i >= (int) limits.low; i -= 4) {
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
    lightDivider.setDivision(16);
  }
  void process(const ProcessArgs &args) override;

  void randomizeAll() {
    storeCurrentPattern();
    for (int i = 0; i < NUM_PATTERNS; i++) {
      patterns[i].randomize();
    }
    takeOutCurrentPattern();
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

  void setStepAttr(int stepNumber, int attr, float factor) {
    this->pattern.steps[stepNumber].setAttr(attr, factor);
  }

  void setStepAttrAbs(int stepNumber, int attr, float target) {
    this->pattern.steps[stepNumber].setAttrAbs(attr, target);
  }

  void setStepAttrBase(int stepNumber, int attr, float target) {
    this->pattern.steps[stepNumber].setAttrBase(attr, target);
  }

  void resetStepAttr(int stepNumber, int attr) {
    this->pattern.steps[stepNumber].resetAttr(attr);
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
    polyphonyMode = MONOPHONIC;
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

  if (this->patternsDisplayConsumer) {
    this->patternsDisplayConsumer->currentPattern = this->patternIdx;
  }
}

struct PhasequeWidget : ModuleWidget {
  PhasequeWidget(Phaseque *module);
  ~PhasequeWidget();
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

  PatternsDisplayWidget *patternsDisplayDisplay = new PatternsDisplayWidget();
  patternsDisplayDisplay->box.pos = Vec(84.0f, 117.0f);
  patternsDisplayDisplay->box.size = Vec(165.0f, 85.0f);
  if (module) {
    Phaseque *phaseque = dynamic_cast<Phaseque*>(module);
    assert(phaseque);
    phaseque->patternsDisplayConsumer = &patternsDisplayDisplay->consumer;
    phaseque->patternsDisplayProducer = &patternsDisplayDisplay->producer;
    // patternsDisplayDisplay->patterns = module->patterns;
    // patternsDisplayDisplay->currentPattern = &module->pattern;
    // patternsDisplayDisplay->currentIdx = &module->patternIdx;
    // patternsDisplayDisplay->goToRequest = &module->goToRequest;
    // patternsDisplayDisplay->patternFlashNeg = &module->patternFlashNeg;
    // patternsDisplayDisplay->patternFlashPos = &module->patternFlashPos;
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

  PatternDisplayWidget *patternDisplay = new PatternDisplayWidget();
  patternDisplay->box.pos = Vec(333.0f, 50.0f);
  patternDisplay->box.size = Vec(233.0f, 233.0f);
  patternDisplay->setupSizes();
  if (module) {
    patternDisplay->resolution = &module->resolution;
    patternDisplay->phase = &module->phaseShifted;
    patternDisplay->pattern = &module->pattern;
    patternDisplay->activeStep = &module->activeStep;
    patternDisplay->direction = &module->direction;
    patternDisplay->globalGate = &module->globalGate;
    patternDisplay->polyphonyMode = &module->polyphonyMode;
    patternDisplay->stepsStates = module->stepsStates;
    patternDisplay->unisonStates = module->unisonStates;
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

PhasequeWidget::~PhasequeWidget() {
	if (this->module) {
    Phaseque *phaseque = dynamic_cast<Phaseque*>(this->module);
    assert(phaseque);
    phaseque->patternsDisplayConsumer = nullptr;
    phaseque->patternsDisplayProducer = nullptr;
	}
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
