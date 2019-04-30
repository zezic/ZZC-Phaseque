#include "ZZC.hpp"

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../../ZZC/src/widgets.hpp"
#endif

#include <ctime>

#ifndef PHASEQ_H
#define PHASEQ_H
#include "Phaseque.hpp"
#endif

#include "PhasequeDisplay.hpp"
#include "PhasequePatternsDisplay.hpp"
#include "dsp/digital.hpp"


struct NegSchmittTrigger : SchmittTrigger {
	bool process(float in) {
		switch (state) {
			case LOW:
				if (in <= -1.f) {
					state = HIGH;
					return true;
				}
				break;
			case HIGH:
				if (in >= 0.f) {
					state = LOW;
				}
				break;
			default:
				if (in <= -1.f) {
					state = HIGH;
				}
				else if (in >= 0.f) {
					state = LOW;
				}
				break;
		}
		return false;
	}
};


inline float patternToVolts(int idx) {
  return (idx - 1) * 1.0f / 12.0f;
}

inline int voltsToPattern(float volts) {
  return 1 + (int) roundf(clamp(volts, 0.0, 3.1) * 12.0);
}

struct Limits {
  int low;
  int high;
};

Limits getRowLimits(int idx) {
  int rowIdx = (idx - 1) / 4;
  Limits limits;
  limits.low = rowIdx * 4 + 1;
  limits.high = limits.low + 3;
  return limits;
}

Limits getColumnLimits(int idx) {
  int baseIdx = (idx - 1) % 4;
  Limits limits;
  limits.low = (baseIdx + 1);
  limits.high = (baseIdx + 1) + 7 * 4;
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

  Pattern patterns[NUM_PATTERNS + 1]; // Because we don't want to use pattern #0
  int patternIdx = 1;
  int lastPatternIdx = 1;
  Pattern pattern = patterns[patternIdx];
  Step* activeStep = nullptr;
  Step* lastActiveStep = nullptr;
  int goToRequest = 0;

  SchmittTrigger clockTrigger;
  TempoTracker tempoTracker;
  bool lastClockInputState = false;
  bool tickedAtLastSample = false;

  SchmittTrigger absModeTrigger;
  bool absMode = false;

  SchmittTrigger tempoTrackButtonTrigger;
  bool tempoTrack = true;

  SchmittTrigger clutchButtonTrigger;
  SchmittTrigger clutchInputTrigger;
  bool clutch = true;

  SchmittTrigger resetButtonTrigger;
  SchmittTrigger resetInputTrigger;
  bool resetPulse = false;

  SchmittTrigger goToInputTrigger;
  SchmittTrigger seqInputTrigger;
  PulseGenerator wentPulseGenerator;
  SchmittTrigger firstInputTrigger;
  SchmittTrigger rndInputTrigger;

  SchmittTrigger leftInputTrigger;
  SchmittTrigger downInputTrigger;
  SchmittTrigger upInputTrigger;
  SchmittTrigger rightInputTrigger;

  PulseGenerator retrigGapGenerator;
  bool retrigGap = false;

  SchmittTrigger globalGateButtonTrigger;
  bool globalGate = true;
  bool globalGateInternal = true;
  float globalShift = 0.0f;
  float globalLen = 1.0f;

  SchmittTrigger gateButtonsTriggers[NUM_STEPS];
  SchmittTrigger jumpInputsTriggers[NUM_STEPS];
  SchmittTrigger rndJumpInputTrigger;

  SchmittTrigger prevPtrnInputTrigger;
  SchmittTrigger nextPtrnInputTrigger;

  SchmittTrigger qntTrigger;
  SchmittTrigger shiftLeftTrigger;
  SchmittTrigger shiftRightTrigger;
  SchmittTrigger lenTrigger;

  SchmittTrigger revTrigger;
  SchmittTrigger flipTrigger;

  NegSchmittTrigger mutRstTrigger;
  SchmittTrigger mutDecTrigger;
  SchmittTrigger mutIncTrigger;

  SchmittTrigger waitButtonTrigger;
  bool wait = false;

  PulseGenerator ptrnStartPulseGenerator;
  PulseGenerator ptrnEndPulseGenerator;

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
  int lastGoToRequest = 0;

  void goToPattern(int targetIdx) {
    if (targetIdx < 1) { targetIdx = NUM_PATTERNS; }
    if (targetIdx > NUM_PATTERNS) { targetIdx = 1; }
    storeCurrentPattern();
    patternIdx = targetIdx;
    takeOutCurrentPattern();
  }

  void goToFirstNonEmpty() {
    for (int i = 1; i <= NUM_PATTERNS; i++) {
      if (patterns[i].hasCustomSteps()) {
        goToPattern(i);
        return;
      }
    }
  }

  inline void processGlobalParams() {
    // Gates
    if (globalGateButtonTrigger.process(params[GLOBAL_GATE_SWITCH_PARAM].value)) {
      globalGateInternal ^= true;
    }
    globalGate = globalGateInternal ^ (inputs[GLOBAL_GATE_INPUT].value > 1.0f);
    lights[GLOBAL_GATE_LED].setBrightness(globalGate);
    // Shift
    globalShift = params[GLOBAL_SHIFT_PARAM].value + clamp(inputs[GLOBAL_SHIFT_INPUT].value * 0.2f * baseStepLen, -baseStepLen, baseStepLen);
    globalLen = params[GLOBAL_LEN_PARAM].value * (clamp(inputs[GLOBAL_LEN_INPUT].value, -5.0f, 5.0f) * 0.2f + 1.0f);
  }

  inline void processPatternNav() {
    if (waitButtonTrigger.process(params[WAIT_SWITCH_PARAM].value)) {
      wait ^= true;
    }
    if (wait) {
      lights[WAIT_LED].value = 1.1f;
    }
    if (goToRequest != 0 && goToRequest != patternIdx) {
      goToPattern(goToRequest);
      goToRequest = 0;
      return;
    }
    if (wait) {
      return;
    }
    if (inputs[SEQ_INPUT].active && seqInputTrigger.process(inputs[SEQ_INPUT].value)) {
      if (patternIdx != pattern.goTo) {
        goToPattern(pattern.goTo);
        return;
      }
    }
    if (inputs[GOTO_INPUT].active) {
      if (goToInputTrigger.process(inputs[GOTO_INPUT].value)) {
        if (inputs[PTRN_INPUT].active) {
          int target = voltsToPattern(inputs[PTRN_INPUT].value);
          if (target != patternIdx) {
            goToPattern(target);
            this->lastGoToRequest = target;
            return;
          }
        } else {
          this->goToFirstNonEmpty();
        }
      } else if (inputs[GOTO_INPUT].value > 1.0f) {
        int target = voltsToPattern(inputs[PTRN_INPUT].value);
        if (target != this->lastGoToRequest && target != this->patternIdx) {
          this->goToPattern(target);
          this->lastGoToRequest = target;
          return;
        }
      } else {
        this->lastGoToRequest = 0;
      }
    }
    if (inputs[PREV_INPUT].active && prevPtrnInputTrigger.process(inputs[PREV_INPUT].value)) {
      for (int i = patternIdx - 1; i >= 1; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = NUM_PATTERNS; i > patternIdx; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[NEXT_INPUT].active && nextPtrnInputTrigger.process(inputs[NEXT_INPUT].value)) {
      for (int i = patternIdx + 1; i <= NUM_PATTERNS; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = 1; i < patternIdx; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[RND_INPUT].active && firstInputTrigger.process(inputs[RND_INPUT].value)) {
      int nonEmpty[NUM_PATTERNS];
      int idx = 0;
      for (int i = 1; i <= NUM_PATTERNS; i++) {
        if (i != patternIdx && patterns[i].hasCustomSteps()) {
          nonEmpty[idx] = i;
          idx++;
        }
      }
      if (idx != 0) {
        int randIdx = (int) (randomUniform() * idx);
        goToPattern(nonEmpty[randIdx]);
        return;
      }
    }
    if (inputs[LEFT_INPUT].active && leftInputTrigger.process(inputs[LEFT_INPUT].value)) {
      Limits limits = getRowLimits(patternIdx);
      for (int i = patternIdx - 1; i >= limits.low; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = limits.high; i > patternIdx; i--) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[RIGHT_INPUT].active && rightInputTrigger.process(inputs[RIGHT_INPUT].value)) {
      Limits limits = getRowLimits(patternIdx);
      for (int i = patternIdx + 1; i <= limits.high; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = limits.low; i < patternIdx; i++) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[DOWN_INPUT].active && downInputTrigger.process(inputs[DOWN_INPUT].value)) {
      Limits limits = getColumnLimits(patternIdx);
      for (int i = patternIdx - 4; i >= limits.low; i -= 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = limits.high; i > patternIdx; i -= 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
    if (inputs[UP_INPUT].active && upInputTrigger.process(inputs[UP_INPUT].value)) {
      Limits limits = getColumnLimits(patternIdx);
      for (int i = patternIdx + 4; i <= limits.high; i += 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
      for (int i = limits.low; i < patternIdx; i += 4) {
        if (patterns[i].hasCustomSteps()) {
          goToPattern(i);
          return;
        }
      }
    }
  }

  inline void processButtons() {
    if (tempoTrackButtonTrigger.process(params[TEMPO_TRACK_SWITCH_PARAM].value)) {
      tempoTrack ^= true;
      lights[TEMPO_TRACK_LED].value = tempoTrack ? 1.0f : 0.0f;
    }
    if (absModeTrigger.process(params[ABS_MODE_SWITCH_PARAM].value)) {
      absMode ^= true;
      lights[ABS_MODE_LED].value = absMode ? 1.0f : 0.0f;
    }
    if (clutchButtonTrigger.process(params[CLUTCH_SWITCH_PARAM].value) || (inputs[CLUTCH_INPUT].active && clutchInputTrigger.process(inputs[CLUTCH_INPUT].value))) {
      clutch ^= true;
      lights[CLUTCH_LED].value = clutch ? 1.0f : 0.0f;
    }
    resetPulse = resetButtonTrigger.process(params[RESET_SWITCH_PARAM].value) || (inputs[RESET_INPUT].active && resetInputTrigger.process(inputs[RESET_INPUT].value));
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
      if (gateButtonsTriggers[i].process(params[GATE_SWITCH_PARAM + i].value)) {
        pattern.steps[i].gate ^= true;
      }
    }
  }

  inline void processPatternButtons() {
    if (inputs[MUTA_DEC_INPUT].active) {
      if (mutRstTrigger.process(inputs[MUTA_DEC_INPUT].value)) {
        this->resetMutation();
      } else if (mutDecTrigger.process(inputs[MUTA_DEC_INPUT].value)) {
        this->mutate(-0.05);
      }
    }
    if (inputs[MUTA_INC_INPUT].active && mutIncTrigger.process(inputs[MUTA_INC_INPUT].value)) {
      this->mutate(0.1);
    }
    if (qntTrigger.process(params[QNT_SWITCH_PARAM].value)) {
      lights[QNT_LED].value = 1.1f;
      pattern.quantize();
      return;
    }
    if (shiftLeftTrigger.process(params[SHIFT_LEFT_SWITCH_PARAM].value)) {
      lights[SHIFT_LEFT_LED].value = 1.1f;
      pattern.shiftLeft();
      return;
    }
    if (shiftRightTrigger.process(params[SHIFT_RIGHT_SWITCH_PARAM].value)) {
      lights[SHIFT_RIGHT_LED].value = 1.1f;
      pattern.shiftRight();
      return;
    }
    if (lenTrigger.process(params[LEN_SWITCH_PARAM].value)) {
      lights[LEN_LED].value = 1.1f;
      pattern.resetLenghts();
      return;
    }
    if (revTrigger.process(params[REV_SWITCH_PARAM].value)) {
      lights[REV_LED].value = 1.1f;
      pattern.reverse();
      return;
    }
    if (flipTrigger.process(params[FLIP_SWITCH_PARAM].value)) {
      lights[FLIP_LED].value = 1.1f;
      pattern.flip();
      return;
    }
  }

  void jumpToStep(Step step) {
    phase = eucmod((direction == 1 ? step.in() : step.out()) - phaseParam , 1.0f);
    jump = true;
  }

  inline void processJumps() {
    jump = false;
    if (absMode || samplesSinceLastReset < 20) {
      return;
    }
    for (int i = 0; i < NUM_STEPS; i++) {
      if (inputs[STEP_JUMP_INPUT + i].active && jumpInputsTriggers[i].process(inputs[STEP_JUMP_INPUT + i].value)) {
        jumpToStep(pattern.steps[i]);
        retrigGapGenerator.trigger(1e-4f);
        return;
      }
    }
    if (inputs[RND_JUMP_INPUT].active && rndJumpInputTrigger.process(inputs[RND_JUMP_INPUT].value)) {
      int nonMuted[NUM_STEPS];
      int idx = 0;
      for (int i = 0; i < NUM_STEPS; i++) {
        if (!pattern.steps[i].gate ^ !globalGate) { continue; }
        nonMuted[idx] = i;
        idx++;
      }
      if (idx > 0) {
        jumpToStep(pattern.steps[nonMuted[(int) (randomUniform() * idx)]]);
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

  Phaseque() : Module(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS) {
    for (int i = 0; i <= NUM_PATTERNS; i++) {
      patterns[i].init();
      patterns[i].goTo = i == NUM_PATTERNS ? 1.0f : ((float) (i + 1));
    }
    patternIdx = 1;
    takeOutCurrentPattern();
    this->refreshPatternPointers();
    lights[TEMPO_TRACK_LED].value = tempoTrack ? 1.0f : 0.0f;
    lights[ABS_MODE_LED].value = absMode ? 1.0f : 0.0f;
    lights[CLUTCH_LED].value = clutch ? 1.0f : 0.0f;
  }
  void step() override;

	void onRandomize() override {
    pattern.randomize();
	}

  void randomizeAll() {
    storeCurrentPattern();
    for (int i = 0; i < NUM_PATTERNS + 1; i++) {
      patterns[i].randomize();
    }
    takeOutCurrentPattern();
  }

  void randomizeAllReso() {
    storeCurrentPattern();
    for (int i = 0; i < NUM_PATTERNS + 1; i++) {
      patterns[i].randomizeReso();
    }
    takeOutCurrentPattern();
  }

  void mutate(float factor) {
    this->pattern.mutate(factor);
  }
  void resetMutation() {
    this->pattern.resetMutation();
  }
  void bakeMutation() {
    this->pattern.bakeMutation();
  }

  void setStepAttr(int stepNumber, int attr, float factor) {
    this->pattern.steps[stepNumber].setAttr(attr, factor);
  }

  void resetStepAttr(int stepNumber, int attr) {
    this->pattern.steps[stepNumber].resetAttr(attr);
  }

  void setPatternReso(float factor) {
    this->pattern.resolution = roundf(clamp(this->pattern.resolution + factor, 1.0f, 99.0f));
  }
  void resetPatternReso() {
    this->pattern.resolution = 8.0f;
  }

  void setPatternShift(float factor) {
    this->pattern.shift = clamp(this->pattern.shift + factor, -baseStepLen, baseStepLen);
  }
  void resetPatternShift() {
    this->pattern.shift = 0.0f;
  }

  void copyToNext() {
    int target = patternIdx + 1;
    if (target > NUM_PATTERNS) { target = 1; }
    for (int i = 0; i < NUM_STEPS; i++) {
      patterns[target].steps[i] = pattern.steps[i];
    }
    patterns[target].resolution = pattern.resolution;
  }

  void copyToPrev() {
    int target = patternIdx - 1;
    if (target < 1) { target = NUM_PATTERNS; }
    for (int i = 0; i < NUM_STEPS; i++) {
      patterns[target].steps[i] = pattern.steps[i];
    }
    patterns[target].resolution = pattern.resolution;
  }

  void copyResoToAll() {
    for (int i = 0; i < NUM_PATTERNS + 1; i++) {
      patterns[i].resolution = pattern.resolution;
    }
  }

  void refreshPatternPointers() {
    pattern.refreshPointers(
      &globalShift,
      &globalLen,
      &inputs[GLOBAL_EXPR_CURVE_INPUT].value,
      &inputs[GLOBAL_EXPR_POWER_INPUT].value
    );
  }

  void storeCurrentPattern() {
    patterns[patternIdx] = pattern;
  }

  void takeOutCurrentPattern() {
    pattern = patterns[patternIdx];
    refreshPatternPointers();
  }

	void onReset() override {
    for (int i = 0; i <= NUM_PATTERNS; i++) {
      patterns[i].init();
      patterns[i].goTo = i == NUM_PATTERNS ? 1.0f : ((float) (i + 1));
    }
    patternIdx = 1;
    takeOutCurrentPattern();
	}

  json_t *toJson() override {
    storeCurrentPattern();
    json_t *rootJ = json_object();
    json_object_set_new(rootJ, "tempoTrack", json_boolean(tempoTrack));
    json_object_set_new(rootJ, "absMode", json_boolean(absMode));
    json_object_set_new(rootJ, "clutch", json_boolean(clutch));
    json_object_set_new(rootJ, "globalGateInternal", json_boolean(globalGateInternal));
    json_object_set_new(rootJ, "patternIdx", json_integer(patternIdx));
    json_object_set_new(rootJ, "wait", json_boolean(wait));

    json_t *patternsJ = json_array();
    for (int i = 0; i <= NUM_PATTERNS; i++) {
      if (patterns[i].isClean()) {
        json_array_append_new(patternsJ, json_null());
      } else {
        json_array_append(patternsJ, patterns[i].toJson());
      }
    }
    json_object_set_new(rootJ, "patterns", patternsJ);

    return rootJ;
  }

  void fromJson(json_t *rootJ) override {
    json_t *tempoTrackJ = json_object_get(rootJ, "tempoTrack");
    json_t *absModeJ = json_object_get(rootJ, "absMode");
    json_t *clutchJ = json_object_get(rootJ, "clutch");
    json_t *globalGateInternalJ = json_object_get(rootJ, "globalGateInternal");
    json_t *patternIdxJ = json_object_get(rootJ, "patternIdx");
    json_t *waitJ = json_object_get(rootJ, "wait");
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

    json_t *patternsJ = json_object_get(rootJ, "patterns");
    if (patternsJ) {
      for (unsigned int i = 0; i < json_array_size(patternsJ) && i <= NUM_PATTERNS; i++) {
        json_t *patternJ = json_array_get(patternsJ, i);
        if (!json_is_null(patternJ)) {
          patterns[i].fromJson(patternJ);
        }
      }
    }

    takeOutCurrentPattern();
  }
};

struct ZZC_PhasequePatternResoKnob : ZZC_CallbackKnob {
  Phaseque* phaseq = nullptr;
  float acc = 0.0f;

  ZZC_PhasequePatternResoKnob() {
    setSVG(SVG::load(assetPlugin(plugin, "res/knobs/ZZC-Knob-25-Encoder.svg")), false);
    shadow->box.size = Vec(29, 29);
    shadow->box.pos = Vec(-2, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
  }

  void onInput(float factor) override {
    if (phaseq) {
      acc += factor * 8.0f;
      if (acc > 1.0f) {
        phaseq->setPatternReso(1.0f);
        acc = 0.0f;
      } else if (acc < -1.0f) {
        phaseq->setPatternReso(-1.0f);
        acc = 0.0f;
      }
    }
  }
  void onReset() override {
    if (phaseq) {
      phaseq->resetPatternReso();
    }
  }
};

struct ZZC_PhasequePatternShiftKnob : ZZC_CallbackKnob {
  Phaseque* phaseq = nullptr;

  ZZC_PhasequePatternShiftKnob() {
    strokeWidth = 1.5;
    setSVG(SVG::load(assetPlugin(plugin, "res/knobs/ZZC-Knob-25-Encoder.svg")), true);
    shadow->box.size = Vec(31, 31);
    shadow->box.pos = Vec(0, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
  }

  void onInput(float factor) override {
    if (phaseq) {
      phaseq->setPatternShift(factor);
    }
  }
  void onReset() override {
    if (phaseq) {
      phaseq->resetPatternShift();
    }
  }
};

struct ZZC_PhasequeMutaKnob : ZZC_CallbackKnob {
  Phaseque* phaseq = nullptr;

  ZZC_PhasequeMutaKnob() {
    setSVG(SVG::load(assetPlugin(plugin, "res/knobs/ZZC-Knob-27-Encoder.svg")), false);
    shadow->box.size = Vec(33, 33);
    shadow->box.pos = Vec(-3, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
  }

  void onInput(float factor) override {
    if (phaseq && factor != 0.0f) {
      phaseq->mutate(factor);
    }
  }
  void onReset() override {
    if (phaseq) {
      phaseq->resetMutation();
    }
  }
};

struct ZZC_PhasequeStepAttrKnob : ZZC_CallbackKnob {
  Phaseque* phaseq = nullptr;
  int item;
  int attr;

  void onInput(float factor) override {
    if (phaseq) {
      phaseq->setStepAttr(item, attr, factor);
    }
  }
  void onReset() override {
    if (phaseq) {
      phaseq->resetStepAttr(item, attr);
    }
  }
};

struct ZZC_PhasequeStepAttrKnob23 : ZZC_PhasequeStepAttrKnob {
  ZZC_PhasequeStepAttrKnob23() {
    strokeWidth = 1.5;
    setSVG(SVG::load(assetPlugin(plugin, "res/knobs/ZZC-Knob-27-23-Encoder.svg")), true);
    shadow->box.size = Vec(33, 33);
    shadow->box.pos = Vec(-1.5, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
  }
};

struct ZZC_PhasequeStepAttrKnob21 : ZZC_PhasequeStepAttrKnob {
  ZZC_PhasequeStepAttrKnob21() {
    strokeWidth = 1.5;
    setSVG(SVG::load(assetPlugin(plugin, "res/knobs/ZZC-Knob-27-21-Encoder.svg")), true);
    shadow->box.size = Vec(29, 29);
    shadow->box.pos = Vec(-0.5, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
  }
};

struct ZZC_PhasequeStepAttrKnob19 : ZZC_PhasequeStepAttrKnob {
  ZZC_PhasequeStepAttrKnob19() {
    strokeWidth = 1.5;
    setSVG(SVG::load(assetPlugin(plugin, "res/knobs/ZZC-Knob-27-19-Encoder.svg")), true);
    shadow->box.size = Vec(25, 25);
    shadow->box.pos = Vec(0.5, 2);
    shadow->blurRadius = 15.0f;
    shadow->opacity = 1.0f;
  }
};


struct PhasequeXYDisplayWidget : XYDisplayWidget {
  Phaseque* phaseq = nullptr;
  int item;
  int attrX;
  int attrY;

  PhasequeXYDisplayWidget() {
  }

  void onInput(float deltaX, float deltaY) override {
    if (phaseq) {
      phaseq->setStepAttr(item, attrX, deltaX);
      phaseq->setStepAttr(item, attrY, deltaY);
    }
  }
  void onReset() override {
    if (phaseq) {
      phaseq->resetStepAttr(item, attrX);
      phaseq->resetStepAttr(item, attrY);
    }
  }
};


void Phaseque::step() {
  processGlobalParams();
  processPatternNav();
  processButtons();
  processPatternButtons();

  // Phase param
  float phaseParamInput = params[PHASE_PARAM].value;
  bool phaseWasZeroed = false;
  if (phaseParamInput != phaseParam) {
    if (phaseParamInput == 0.0f || (lastPhaseParamInput < 0.1f && phaseParamInput > 0.9f) || (lastPhaseParamInput > 0.9f && phaseParamInput < 0.1f)) {
      if (phaseParamInput == 0.0f) {
        phaseWasZeroed = true;
      }
      phaseParam = phaseParamInput;
    } else {
      float delta = phaseParamInput - phaseParam;
      phaseParam = phaseParam + delta * engineGetSampleTime() * 50.0f; // Smoothing
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

  if (inputs[PHASE_INPUT].active) {
    bpmDisabled = true;
    double phaseIn = inputs[PHASE_INPUT].value / 10.0;
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
        if (inputs[CLOCK_INPUT].active && clockTrigger.process(inputs[CLOCK_INPUT].value)) {
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
  } else if (inputs[CLOCK_INPUT].active) {
    bpmDisabled = false;
    if (!lastClockInputState || lastPhaseInState) {
      tempoTracker.reset();
    }
    float currentStep = floorf(phase * resolution);
    float sampleTime = engineGetSampleTime();
    if (clockTrigger.process(inputs[CLOCK_INPUT].value) && samplesSinceLastReset > 19) {
      tempoTracker.tick(sampleTime);
      if (clutch) {
        if (bps < 0.0f) {
          float nextStep = eucmod(currentStep, resolution);
          float nextPhase = fastmod(nextStep / resolution, 1.0f);
          phase = nextPhase;
        } else {
          float nextStep = eucmod(currentStep + 1.0f, resolution);
          float nextPhase = fastmod(nextStep / resolution, 1.0f);
          phase = nextPhase;
        }
      }
      tickedAtLastSample = true;
    } else {
      tempoTracker.acc(sampleTime);
      if (inputs[VBPS_INPUT].active) {
        bps = inputs[VBPS_INPUT].value;
      } else if (tempoTrack) {
        if (tempoTracker.detected) {
          bps = tempoTracker.bps;
        }
      } else {
        bps = params[BPM_PARAM].value / 60.0f;
      }
      float nextPhase = fastmod(phase + bps * engineGetSampleTime() / resolution, 1.0f);
      float nextStep = floorf(nextPhase * resolution);
      if (clutch) {
        if (nextStep == currentStep || (bps < 0.0f && (tickedAtLastSample || resetPulse))) {
          phase = nextPhase;
        }
      }
      tickedAtLastSample = false;
    }
  } else if (inputs[VBPS_INPUT].active) {
    bpmDisabled = false;
    bps = inputs[VBPS_INPUT].value;
    if (clutch) {
      float nextPhase = fastmod(phase + bps * engineGetSampleTime() / resolution, 1.0f);
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
  if (isnan(phase)) {
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

  activeStep = pattern.getStepForPhase(phaseShifted, globalGate);
  if ((activeStep && lastActiveStep) && activeStep != lastActiveStep) {
    retrigGapGenerator.trigger(1e-4f);
  }
  lastActiveStep = activeStep;

  outputs[PTRN_START_OUTPUT].value = ptrnStartPulseGenerator.process(engineGetSampleTime()) ? 10.0f : 0.0f;
  outputs[PTRN_END_OUTPUT].value = ptrnEndPulseGenerator.process(engineGetSampleTime()) ? 10.0f : 0.0f;
  outputs[PTRN_WRAP_OUTPUT].value = fmaxf(outputs[PTRN_START_OUTPUT].value, outputs[PTRN_END_OUTPUT].value);
  outputs[PTRN_OUTPUT].value = patternToVolts(patternIdx);
  if (patternIdx != lastPatternIdx) {
    wentPulseGenerator.trigger(1e-3f);
    lastPatternIdx = patternIdx;
  }
  outputs[WENT_OUTPUT].value = wentPulseGenerator.process(engineGetSampleTime()) ? 10.0f : 0.0f;

  if (resetPulse && !absMode) {
    retrigGapGenerator.trigger(1e-4f);
  }

  retrigGap = retrigGapGenerator.process(engineGetSampleTime());
  if (retrigGap || !clutch) {
    outputs[GATE_OUTPUT].value = 0.0f;
    lights[GATE_LIGHT].setBrightness(0.0f);
  } else {
    outputs[GATE_OUTPUT].value = activeStep ? 10.0f : 0.0f;
    lights[GATE_LIGHT].setBrightness(activeStep ? 1.0f : 0.0f);
  }

  if (activeStep) {
    float v = activeStep->attrs[STEP_VALUE].value;
    float shift = activeStep->attrs[STEP_SHIFT].value / baseStepLen;
    float len = activeStep->attrs[STEP_LEN].value / baseStepLen - 1.0f;
    float stepPhase = activeStep->phase(phaseShifted);
    float expr = activeStep->expr(stepPhase);
    float curve = activeStep->attrs[STEP_EXPR_CURVE].value;

    outputs[V_OUTPUT].value = v;
    outputs[SHIFT_OUTPUT].value = shift * 5.0f;
    outputs[LEN_OUTPUT].value = len * 5.0f;
    outputs[EXPR_OUTPUT].value = expr * 5.0f;
    outputs[EXPR_CURVE_OUTPUT].value = curve * 5.0f;
    outputs[PHASE_OUTPUT].value = stepPhase * 10.0f;

    lights[V_POS_LIGHT].setBrightness(fmaxf(v * 0.5f, 0.0f));
    lights[V_NEG_LIGHT].setBrightness(fmaxf(v * -0.5f, 0.0f));
    lights[SHIFT_POS_LIGHT].setBrightness(fmaxf(shift, 0.0f));
    lights[SHIFT_NEG_LIGHT].setBrightness(fmaxf(-shift, 0.0f));
    lights[LEN_POS_LIGHT].setBrightness(fmaxf(len, 0.0f));
    lights[LEN_NEG_LIGHT].setBrightness(fmaxf(-len, 0.0f));
    lights[EXPR_POS_LIGHT].setBrightness(fmaxf(expr, 0.0f));
    lights[EXPR_NEG_LIGHT].setBrightness(fmaxf(-expr, 0.0f));
    lights[EXPR_CURVE_POS_LIGHT].setBrightness(fmaxf(curve, 0.0f));
    lights[EXPR_CURVE_NEG_LIGHT].setBrightness(fmaxf(-curve, 0.0f));
    lights[PHASE_LIGHT].setBrightness(stepPhase);

    for (int i = 0; i < NUM_STEPS; i++) {
      outputs[STEP_GATE_OUTPUT + i].value = activeStep->idx == i ? 10.0f : 0.0f;
      lights[STEP_GATE_LIGHT + i].setBrightness(activeStep->idx == i ? 1.0f : 0.0f);
    }
  } else {
    for (int i = 0; i < NUM_STEPS; i++) {
      outputs[STEP_GATE_OUTPUT + i].value = 0.0f;
      lights[STEP_GATE_LIGHT + i].setBrightness(0.0f);
    }
  }
  outputs[PTRN_PHASE_OUTPUT].value = phaseShifted * 10.0f;

  for (int i = 0; i < NUM_STEPS; i++) {
    lights[GATE_SWITCH_LED + i].setBrightness(pattern.steps[i].gate ^ !globalGate);
  }

  lights[PHASE_LED].setBrightness(inputs[PHASE_INPUT].active);
  lights[CLOCK_LED].setBrightness(inputs[CLOCK_INPUT].active);
  lights[VBPS_LED].setBrightness(inputs[VBPS_INPUT].active && !inputs[PHASE_INPUT].active);

  lastPhase = phase;
  lastPhaseInState = inputs[PHASE_INPUT].active;
  lastPhaseShifted = phaseShifted;
  lastPhaseParamInput = params[PHASE_PARAM].value;
  lastClockInputState = inputs[CLOCK_INPUT].active;
}

struct PhasequeWidget : ModuleWidget {
  PhasequeWidget(Phaseque *module);
  void appendContextMenu(Menu *menu) override;
};

PhasequeWidget::PhasequeWidget(Phaseque *module) : ModuleWidget(module) {
  setPanel(SVG::load(assetPlugin(plugin, "res/panels/Phaseque.svg")));

  addInput(Port::create<ZZC_PJ_Port>(Vec(14, 50), Port::INPUT, module, Phaseque::CLOCK_INPUT));
  addChild(ModuleLightWidget::create<TinyLight<GreenLight>>(Vec(36, 50), module, Phaseque::CLOCK_LED));
  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(15.3f, 82.3f), module, Phaseque::TEMPO_TRACK_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(17.1f, 84.0f), module, Phaseque::TEMPO_TRACK_LED));
  addInput(Port::create<ZZC_PJ_Port>(Vec(49, 50), Port::INPUT, module, Phaseque::VBPS_INPUT));
  addChild(ModuleLightWidget::create<TinyLight<GreenLight>>(Vec(71, 50), module, Phaseque::VBPS_LED));
  addParam(ParamWidget::create<ZZC_Knob25SnappyNoRand>(Vec(49, 80.8f), module, Phaseque::BPM_PARAM, 0.0, 240.0, 120.0));
  addInput(Port::create<ZZC_PJ_Port>(Vec(84, 50), Port::INPUT, module, Phaseque::PHASE_INPUT));
  addChild(ModuleLightWidget::create<TinyLight<GreenLight>>(Vec(106, 50), module, Phaseque::PHASE_LED));
  addParam(ParamWidget::create<ZZC_EncoderKnob>(Vec(258.75f, 49), module, Phaseque::PHASE_PARAM, 0.0, 1.0, 0.0));
  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(120.3f, 51.3f), module, Phaseque::ABS_MODE_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(122.1f, 53.0f), module, Phaseque::ABS_MODE_LED));

  Display32Widget *bpmDisplay = new Display32Widget();
  bpmDisplay->box.pos = Vec(84.0f, 83.0f);
  bpmDisplay->box.size = Vec(58.0f, 21.0f);
  bpmDisplay->value = &module->bpm;
  bpmDisplay->disabled = &module->bpmDisabled;
  addChild(bpmDisplay);

  addInput(Port::create<ZZC_PJ_Port>(Vec(189, 81), Port::INPUT, module, Phaseque::CLUTCH_INPUT));
  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(190.3f, 51.3f), module, Phaseque::CLUTCH_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(192.1f, 53.0f), module, Phaseque::CLUTCH_LED));

  DisplayIntpartWidget *resolutionDisplay = new DisplayIntpartWidget();
  resolutionDisplay->box.pos = Vec(152, 52);
  resolutionDisplay->box.size = Vec(29, 21);
  resolutionDisplay->value = &module->resolution;
  addChild(resolutionDisplay);

  ZZC_PhasequePatternResoKnob *patternResoKnob = new ZZC_PhasequePatternResoKnob();
  patternResoKnob->box.pos = Vec(154, 80.8f);
  if (module) {
    patternResoKnob->phaseq = module;
  }
  addChild(patternResoKnob);

  addInput(Port::create<ZZC_PJ_Port>(Vec(224, 81), Port::INPUT, module, Phaseque::RESET_INPUT));
  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(225.3f, 51.3f), module, Phaseque::RESET_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(227.1f, 53.0f), module, Phaseque::RESET_LED));

  addInput(Port::create<ZZC_PJ_Port>(Vec(14, 126), Port::INPUT, module, Phaseque::GOTO_INPUT));
  addInput(Port::create<ZZC_PJ_Port>(Vec(49, 126), Port::INPUT, module, Phaseque::PTRN_INPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(14, 174), Port::OUTPUT, module, Phaseque::WENT_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(49, 174), Port::OUTPUT, module, Phaseque::PTRN_OUTPUT));

  PatternsDisplayWidget *patternsDisplayDisplay = new PatternsDisplayWidget();
  patternsDisplayDisplay->box.pos = Vec(84.0f, 117.0f);
  patternsDisplayDisplay->box.size = Vec(165.0f, 85.0f);
  patternsDisplayDisplay->patterns = module->patterns;
  patternsDisplayDisplay->currentPattern = &module->pattern;
  patternsDisplayDisplay->currentIdx = &module->patternIdx;
  patternsDisplayDisplay->goToRequest = &module->goToRequest;
  addChild(patternsDisplayDisplay);

  addOutput(Port::create<ZZC_PJ_Port>(Vec(259, 126), Port::OUTPUT, module, Phaseque::PTRN_PHASE_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(294, 126), Port::OUTPUT, module, Phaseque::PTRN_WRAP_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(259, 174), Port::OUTPUT, module, Phaseque::PTRN_START_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(294, 174), Port::OUTPUT, module, Phaseque::PTRN_END_OUTPUT));

  addInput(Port::create<ZZC_PJ_Port>(Vec(14, 220), Port::INPUT, module, Phaseque::PREV_INPUT));
  addInput(Port::create<ZZC_PJ_Port>(Vec(49, 220), Port::INPUT, module, Phaseque::NEXT_INPUT));

  addInput(Port::create<ZZC_PJ_Port>(Vec(84, 220), Port::INPUT, module, Phaseque::LEFT_INPUT));
  addInput(Port::create<ZZC_PJ_Port>(Vec(119, 220), Port::INPUT, module, Phaseque::DOWN_INPUT));
  addInput(Port::create<ZZC_PJ_Port>(Vec(154, 220), Port::INPUT, module, Phaseque::UP_INPUT));
  addInput(Port::create<ZZC_PJ_Port>(Vec(189, 220), Port::INPUT, module, Phaseque::RIGHT_INPUT));
  addInput(Port::create<ZZC_PJ_Port>(Vec(224, 220), Port::INPUT, module, Phaseque::SEQ_INPUT));

  addInput(Port::create<ZZC_PJ_Port>(Vec(259, 220), Port::INPUT, module, Phaseque::RND_INPUT));

  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(295.3f, 221.3f), module, Phaseque::WAIT_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_RedLight>>(Vec(297.1f, 223.0f), module, Phaseque::WAIT_LED));

  PatternDisplayWidget *patternDisplay = new PatternDisplayWidget();
  patternDisplay->box.pos = Vec(333.0f, 50.0f);
  patternDisplay->box.size = Vec(233.0f, 233.0f);
  patternDisplay->setupSizes();
  patternDisplay->resolution = &module->resolution;
  patternDisplay->phase = &module->phaseShifted;
  patternDisplay->pattern = &module->pattern;
  patternDisplay->activeStep = &module->activeStep;
  patternDisplay->direction = &module->direction;
  patternDisplay->globalGate = &module->globalGate;
  addChild(patternDisplay);

  addInput(Port::create<ZZC_PJ_Port>(Vec(259, 319.75f), Port::INPUT, module, Phaseque::GLOBAL_GATE_INPUT));
  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(260.3f, 278.3f), module, Phaseque::GLOBAL_GATE_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(262.1f, 280.0f), module, Phaseque::GLOBAL_GATE_LED));

  addInput(Port::create<ZZC_PJ_Port>(Vec(14, 319.75f), Port::INPUT, module, Phaseque::GLOBAL_SHIFT_INPUT));
  addParam(ParamWidget::create<ZZC_Knob25NoRand>(Vec(14.05f, 276.9f), module, Phaseque::GLOBAL_SHIFT_PARAM, -baseStepLen, baseStepLen, 0.0));

  addInput(Port::create<ZZC_PJ_Port>(Vec(49, 319.75f), Port::INPUT, module, Phaseque::GLOBAL_LEN_INPUT));
  addParam(ParamWidget::create<ZZC_Knob25NoRand>(Vec(49.05f, 276.9f), module, Phaseque::GLOBAL_LEN_PARAM, 0.0, 2.0, 1.0));

  addInput(Port::create<ZZC_PJ_Port>(Vec(294, 277), Port::INPUT, module, Phaseque::GLOBAL_EXPR_CURVE_INPUT));
  addInput(Port::create<ZZC_PJ_Port>(Vec(294, 319.75f), Port::INPUT, module, Phaseque::GLOBAL_EXPR_POWER_INPUT));

  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(85.3f, 278.3f), module, Phaseque::QNT_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(87.1f, 280.0f), module, Phaseque::QNT_LED));

  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(120.3f, 278.3f), module, Phaseque::SHIFT_LEFT_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(122.1f, 280.0f), module, Phaseque::SHIFT_LEFT_LED));

  ZZC_PhasequePatternShiftKnob *patternShiftKnob = new ZZC_PhasequePatternShiftKnob();
  patternShiftKnob->box.pos = Vec(150.5f, 273.5f);
  if (module) {
    patternShiftKnob->phaseq = module;
    patternShiftKnob->attachValue(&module->pattern.shift, -baseStepLen, baseStepLen, 0.0f);
  }
  addChild(patternShiftKnob);

  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(190.3f, 278.3f), module, Phaseque::SHIFT_RIGHT_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(192.1f, 280.0f), module, Phaseque::SHIFT_RIGHT_LED));

  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(225.3f, 278.3f), module, Phaseque::LEN_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(227.1f, 280.0f), module, Phaseque::LEN_LED));

  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(85.3f, 320.3f), module, Phaseque::REV_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(87.1f, 322.0f), module, Phaseque::REV_LED));

  addInput(Port::create<ZZC_PJ_Port>(Vec(119, 319.75f), Port::INPUT, module, Phaseque::MUTA_DEC_INPUT));

  ZZC_PhasequeMutaKnob *mutaKnob = new ZZC_PhasequeMutaKnob();
  mutaKnob->box.pos = Vec(153, 318);
  if (module) {
    mutaKnob->phaseq = module;
  }
  addChild(mutaKnob);

  addInput(Port::create<ZZC_PJ_Port>(Vec(189, 319.75f), Port::INPUT, module, Phaseque::MUTA_INC_INPUT));

  addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(225.3f, 320.3f), module, Phaseque::FLIP_SWITCH_PARAM, 0.0f, 1.0f, 0.0f));
  addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(227.1f, 322.0f), module, Phaseque::FLIP_LED));

  addInput(Port::create<ZZC_PJ_Port>(Vec(579, 319.75f), Port::INPUT, module, Phaseque::RND_JUMP_INPUT));

  float stepPeriod = 35.0f;
  float stepsAreaX = 614.0;
  for (int i = 0; i < NUM_STEPS; i++) {
    ZZC_PhasequeStepAttrKnob23 *valueKnob = new ZZC_PhasequeStepAttrKnob23();
    valueKnob->box.pos = Vec(stepsAreaX + stepPeriod * i - 0.5f, 48);
    if (module) {
      valueKnob->phaseq = module;
      valueKnob->item = i;
      valueKnob->attr = STEP_VALUE;
      valueKnob->attachValue(&module->pattern.steps[i].attrs[STEP_VALUE].value, -2.0f, 2.0f, 0.0f);
    }
    addChild(valueKnob);

    addParam(ParamWidget::create<ZZC_LEDBezelDark>(Vec(stepsAreaX + 3.3f + stepPeriod * i, 82.3f), module, Phaseque::GATE_SWITCH_PARAM + i, 0.0f, 1.0f, 0.0f));
    addChild(ModuleLightWidget::create<LedLight<ZZC_YellowLight>>(Vec(stepsAreaX + 5.1f + stepPeriod * i, 84.0f), module, Phaseque::GATE_SWITCH_LED + i));

    ZZC_PhasequeStepAttrKnob21 *shiftKnob = new ZZC_PhasequeStepAttrKnob21();
    shiftKnob->box.pos = Vec(stepsAreaX + 0.5f + stepPeriod * i, 110.5f);
    if (module) {
      shiftKnob->phaseq = module;
      shiftKnob->item = i;
      shiftKnob->attr = STEP_SHIFT;
      shiftKnob->attachValue(&module->pattern.steps[i].attrs[STEP_SHIFT].value, -baseStepLen, baseStepLen, 0.0f);
    }
    addChild(shiftKnob);

    ZZC_PhasequeStepAttrKnob21 *lenKnob = new ZZC_PhasequeStepAttrKnob21();
    lenKnob->box.pos = Vec(stepsAreaX + 0.5f + stepPeriod * i, 141.5f);
    if (module) {
      lenKnob->phaseq = module;
      lenKnob->item = i;
      lenKnob->attr = STEP_LEN;
      lenKnob->attachValue(&module->pattern.steps[i].attrs[STEP_LEN].value, 0.0f, baseStepLen * 2.0f, baseStepLen);
    }
    addChild(lenKnob);

    ZZC_PhasequeStepAttrKnob19 *exprInKnob = new ZZC_PhasequeStepAttrKnob19();
    exprInKnob->box.pos = Vec(stepsAreaX + 1.5f + stepPeriod * i, 176);
    if (module) {
      exprInKnob->phaseq = module;
      exprInKnob->item = i;
      exprInKnob->attr = STEP_EXPR_IN;
      exprInKnob->attachValue(&module->pattern.steps[i].attrs[STEP_EXPR_IN].value, -1.0f, 1.0f, 0.0f);
      exprInKnob->enableColor();
    }
    addChild(exprInKnob);

    PhasequeXYDisplayWidget *exprDisplay = new PhasequeXYDisplayWidget();
    exprDisplay->box.pos = Vec(stepsAreaX + 1.0f + stepPeriod * i, 207);
    exprDisplay->box.size = Vec(27, 27);
    if (module) {
      exprDisplay->phaseq = module;
      exprDisplay->item = i;
      exprDisplay->attrX = STEP_EXPR_POWER;
      exprDisplay->attrY = STEP_EXPR_CURVE;
      exprDisplay->x = &module->pattern.steps[i].attrs[STEP_EXPR_POWER].value;
      exprDisplay->y = &module->pattern.steps[i].attrs[STEP_EXPR_CURVE].value;
    }
    exprDisplay->setupSize();
    exprDisplay->setupPtrs();
    addChild(exprDisplay);

    ZZC_PhasequeStepAttrKnob19 *exprOutKnob = new ZZC_PhasequeStepAttrKnob19();
    exprOutKnob->box.pos = Vec(stepsAreaX + 1.5f + stepPeriod * i, 240.5f);
    if (module) {
      exprOutKnob->phaseq = module;
      exprOutKnob->item = i;
      exprOutKnob->attr = STEP_EXPR_OUT;
      exprOutKnob->attachValue(&module->pattern.steps[i].attrs[STEP_EXPR_OUT].value, -1.0f, 1.0f, 0.0f);
      exprOutKnob->enableColor();
    }
    addChild(exprOutKnob);

    addInput(Port::create<ZZC_PJ_Port>(Vec(stepsAreaX + 2.0f + stepPeriod * i, 271), Port::INPUT, module, Phaseque::STEP_JUMP_INPUT + i));
    addChild(ModuleLightWidget::create<SmallLight<GreenLight>>(Vec(stepsAreaX + 11.3f + stepPeriod * i, 301.5f), module, Phaseque::STEP_GATE_LIGHT + i));
    addOutput(Port::create<ZZC_PJ_Port>(Vec(stepsAreaX + 2.0f + stepPeriod * i, 319.75f), Port::OUTPUT, module, Phaseque::STEP_GATE_OUTPUT + i));
  }

  stepPeriod = 34.0f;
  float outputsAreaX = 335.0f;

  addOutput(Port::create<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 0, 319.75f), Port::OUTPUT, module, Phaseque::GATE_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 1, 319.75f), Port::OUTPUT, module, Phaseque::V_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 2, 319.75f), Port::OUTPUT, module, Phaseque::SHIFT_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 3, 319.75f), Port::OUTPUT, module, Phaseque::LEN_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 4, 319.75f), Port::OUTPUT, module, Phaseque::EXPR_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 5, 319.75f), Port::OUTPUT, module, Phaseque::EXPR_CURVE_OUTPUT));
  addOutput(Port::create<ZZC_PJ_Port>(Vec(outputsAreaX + stepPeriod * 6, 319.75f), Port::OUTPUT, module, Phaseque::PHASE_OUTPUT));

  float outputLedsAreaX = 344.3f;

  addChild(ModuleLightWidget::create<SmallLight<GreenLight>>(Vec(outputLedsAreaX + stepPeriod * 0, 301.5f), module, Phaseque::GATE_LIGHT));
  addChild(ModuleLightWidget::create<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 1, 301.5f), module, Phaseque::V_POS_LIGHT));
  addChild(ModuleLightWidget::create<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 2, 301.5f), module, Phaseque::SHIFT_POS_LIGHT));
  addChild(ModuleLightWidget::create<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 3, 301.5f), module, Phaseque::LEN_POS_LIGHT));
  addChild(ModuleLightWidget::create<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 4, 301.5f), module, Phaseque::EXPR_POS_LIGHT));
  addChild(ModuleLightWidget::create<SmallLight<GreenRedLight>>(Vec(outputLedsAreaX + stepPeriod * 5, 301.5f), module, Phaseque::EXPR_CURVE_POS_LIGHT));
  addChild(ModuleLightWidget::create<SmallLight<GreenLight>>(Vec(outputLedsAreaX + stepPeriod * 6, 301.5f), module, Phaseque::PHASE_LIGHT));

  addChild(Widget::create<ZZC_Screw>(Vec(RACK_GRID_WIDTH, 0)));
  addChild(Widget::create<ZZC_Screw>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
  addChild(Widget::create<ZZC_Screw>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
  addChild(Widget::create<ZZC_Screw>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
}

struct PhasequeCopyToNextItem : MenuItem {
  Phaseque *phaseq;
  void onAction(EventAction &e) override {
    phaseq->copyToNext();
  }
};

struct PhasequeCopyToPrevItem : MenuItem {
  Phaseque *phaseq;
  void onAction(EventAction &e) override {
    phaseq->copyToPrev();
  }
};

struct PhasequeCopyResoItem : MenuItem {
  Phaseque *phaseq;
  void onAction(EventAction &e) override {
    phaseq->copyResoToAll();
  }
};

struct PhasequeRndAllItem : MenuItem {
  Phaseque *phaseq;
  void onAction(EventAction &e) override {
    phaseq->randomizeAll();
  }
};

struct PhasequeRndAllResoItem : MenuItem {
  Phaseque *phaseq;
  void onAction(EventAction &e) override {
    phaseq->randomizeAllReso();
  }
};

struct PhasequeBakeMutationItem : MenuItem {
  Phaseque *phaseq;
  void onAction(EventAction &e) override {
    phaseq->bakeMutation();
  }
};

struct PhasequeClearPatternItem : MenuItem {
  Phaseque *phaseq;
  void onAction(EventAction &e) override {
    phaseq->pattern.init();
  }
};

void PhasequeWidget::appendContextMenu(Menu *menu) {
	menu->addChild(new MenuSeparator());

  Phaseque *phaseq = dynamic_cast<Phaseque*>(module);
  assert(phaseq);

  PhasequeCopyToNextItem *phaseqCopyToNextItem = MenuItem::create<PhasequeCopyToNextItem>("Copy Steps To Next Pattern");
  PhasequeCopyToPrevItem *phaseqCopyToPrevItem = MenuItem::create<PhasequeCopyToPrevItem>("Copy Steps To Prev Pattern");
  PhasequeCopyResoItem *phaseqCopyResoItem = MenuItem::create<PhasequeCopyResoItem>("Copy Reso To All Patterns");
  PhasequeRndAllItem *phaseqRndAllItem = MenuItem::create<PhasequeRndAllItem>("Randomize All Patterns");
  PhasequeRndAllResoItem *phaseqRndAllResoItem = MenuItem::create<PhasequeRndAllResoItem>("Randomize All Resolutions");
  PhasequeBakeMutationItem *phaseqBakeMutationItem = MenuItem::create<PhasequeBakeMutationItem>("Bake Mutation");
  PhasequeClearPatternItem *phaseqClearPatternItem = MenuItem::create<PhasequeClearPatternItem>("Clear Pattern");
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
}

Model *modelPhaseque = Model::create<Phaseque, PhasequeWidget>("ZZC", "Phaseque", "Phaseque", SEQUENCER_TAG);
