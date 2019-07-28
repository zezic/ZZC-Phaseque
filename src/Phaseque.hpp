#include "rack0.hpp"
#include <algorithm> // for std:rotate

#include "ZZC.hpp"

const int NUM_STEPS = 8;
#define NUM_PATTERNS 32

using namespace rack;

enum PolyphonyModes {
  MONOPHONIC,
  POLYPHONIC,
  UNISON,
  NUM_POLYPHONY_MODES
};

inline float crossfadePow(float base, float exp) {
  float lowExp = std::floor(exp - 1.0f);
  float low = base;
  for (float i = 0.0f; i < lowExp; i = i + 1.0f) {
    low = low * base;
  }
  float high = low * base;
  float balance = exp - lowExp;
  return crossfade(low, high, balance);
}

inline float fastmod(float value, float base) {
  if (value >= 0.0f && value < base) { return value; }
  if (value < 0.0f) {
    return value + base;
  }
  return value - base;
}

float curve(float phase, float curvature, float power, float in, float out) {
  float range = out - in;
  if (curvature == 0.0f) {
    return in + range * phase;
  }
  float absCurvature = std::abs(curvature);
  bool invCurve = in > out;
  bool invSpline = (curvature > 0.0f) ^ invCurve;
  power = 5.0f + power * (invSpline ? -3.0f : 3.0f);
  // float spline = invSpline ? (1.0f - std::pow(1.0f - phase, power)) : std::pow(phase, power);
  float spline = invSpline ? (1.0f - crossfadePow(1.0f - phase, power)) : crossfadePow(phase, power);
  // float deformed = (phase * (1.0f - absCurvature)) + spline * absCurvature;
  // float deformed = phase + (spline - phase) * absCurvature;
  float deformed = crossfade(phase, spline, absCurvature);
  return in + deformed * range;
}

struct ChangeTrigger {
	bool lastState;
  bool hasLastState = false;

	ChangeTrigger() {
		reset();
	}
	void reset() {
		lastState = true;
	}
	bool process(bool state) {
    if (!hasLastState) {
      lastState = state;
      hasLastState = true;
      return false;
    }
		bool triggered = lastState != state;
		lastState = state;
		return triggered;
	}
};

const float baseStepLen = 1.0f / NUM_STEPS;
const float minStepLen = baseStepLen / 64.0f;

enum StepAttr {
  STEP_VALUE,
  STEP_LEN,
  STEP_SHIFT,
  STEP_EXPR_IN,
  STEP_EXPR_CURVE,
  STEP_EXPR_POWER,
  STEP_EXPR_OUT,
  STEP_ATTRS_TOTAL
};

struct AttrDefaults {
  float defaultValue;
  float minValue;
  float maxValue;
  float mutMult;
};

AttrDefaults getAttrDefaults(int idx) {
  switch(idx) {
    case STEP_VALUE: return AttrDefaults {0.0f, -2.0f, 2.0f, 0.2f};
    case STEP_LEN: return AttrDefaults {baseStepLen, 0.0f, baseStepLen * 2.0f, 1.0f};
    case STEP_SHIFT: return AttrDefaults {0.0f, -baseStepLen, baseStepLen, 1.0f};
    case STEP_EXPR_IN: return AttrDefaults {0.0f, -1.0f, 1.0f, 1.5f};
    case STEP_EXPR_CURVE: return AttrDefaults {0.0f, -1.0f, 1.0f, 2.0f};
    case STEP_EXPR_POWER: return AttrDefaults {0.0f, -1.0f, 1.0f, 2.0f};
    case STEP_EXPR_OUT: return AttrDefaults {0.0f, -1.0f, 1.0f, 1.5f};
    default: return AttrDefaults {0.0f, 0.0f, 1.0f, 1.0f};
  }
}

struct Mutator {
  float factor = 0.0f;

  void init() {
    this->factor = 0.0f;
  }

  float mutate(float value, float force, float low, float high) {
    float range = high - low;
    if ((isNear(value, high) && this->factor > 0.0f) ||
        (isNear(value, low) && this->factor < 0.0f)) {
      this->factor *= -1.0f;
    }
    this->factor = clamp(this->factor + (random::uniform() - 0.5f) * 0.05f, -1.0f, 1.0f);
    return clamp(value + this->factor * range * force, low, high);
  }
};

struct MutableValue {
  float base;
  float mutation;
  float value;
  float defaultValue = 0.0f;
  float minValue;
  float maxValue;
  Mutator mutator;
  bool isClean = true;
  float lastMutFactor = 0.0f;
  float unmutateFrom = 0.0f;
  float mutMult = 1.0f;

  json_t *dataToJson() {
    json_t *mutableValueJ = json_object();
    json_object_set_new(mutableValueJ, "base", json_real(this->base));
    json_object_set_new(mutableValueJ, "mutation", json_real(this->mutation));
    json_object_set_new(mutableValueJ, "isClean", json_boolean(this->isClean));
    return mutableValueJ;
  }

  void dataFromJson(json_t *mutableValueJ) {
    this->base = json_number_value(json_object_get(mutableValueJ, "base"));
    this->mutation = json_number_value(json_object_get(mutableValueJ, "mutation"));
    this->isClean = json_boolean_value(json_object_get(mutableValueJ, "isClean"));
    this->applyMutation();
  }

  void init() {
    this->base = this->defaultValue;
    this->mutation = 0.0f;
    this->mutator.init();
    this->applyMutation();
    this->isClean = true;
  }

  void setup(float defaultVal, float minVal, float maxVal, float mutMulti) {
    this->defaultValue = defaultVal;
    this->minValue = minVal;
    this->maxValue = maxVal;
    this->mutMult = mutMulti;
  }

  void mutate(float factor) {
    if (factor > 0.0f) {
      this->mutation = this->mutator.mutate(
        this->mutation,
        factor * this->mutMult,
        this->minValue - this->base,
        this->maxValue - this->base
      );
    } else if (factor < 0.0f) {
      if (sgn(factor) != sgn(this->lastMutFactor)) {
        unmutateFrom = fabsf(this->mutation);
      }
      float range = this->maxValue - this->minValue;
      float delta = fabsf(this->unmutateFrom * factor);
      if (this->mutation > 0.0f) {
        this->mutation = fmaxf(this->mutation - delta, 0.0f);
      } else if (this->mutation < 0.0f) {
        this->mutation = fminf(this->mutation + delta, 0.0f);
      }
      if (isNear(this->mutation, 0.0f, range * 0.001f)) {
        this->mutation = 0.0f;
        this->mutator.init();
      }
    }
    this->applyMutation();
    this->lastMutFactor = factor;
    this->isClean = this->mutation == 0.0f && this->base == this->defaultValue;
  }

  void applyMutation() {
    this->value = this->base + this->mutation;
  }

  void clampMutation() {
    if (this->base + this->mutation > this->maxValue) {
      this->mutation = this->maxValue - this->base;
    } else if (this->base + this->mutation < this->minValue) {
      this->mutation = this->minValue - this->base;
    }
  }

  void scaleMutation(float factor) {
    this->mutation *= factor;
    this->clampMutation();
    this->applyMutation();
  }

  void adjValue(float factor) {
    if ((factor > 0.0f && this->base == this->maxValue) ||
        (factor < 0.0f && this->base == this->minValue)) {
      // base is stuck to boundary, try to achieve target value by reducing mutation
      this->mutation += factor;
    } else {
      this->base = clamp(this->base + factor, this->minValue, this->maxValue);
    }
    this->clampMutation();
    this->applyMutation();
    this->isClean = false;
  }

  void setValue(float target) {
    this->base = target;
    this->mutation = 0.0f;
    this->applyMutation();
    this->isClean = false;
  }

  void setBase(float target) {
    this->base = target;
    if (target != this->defaultValue) {
      this->isClean = false;
    }
    this->applyMutation();
  }
  void setMutation(float target) {
    this->mutation = target;
    this->isClean = false;
  }

  void randomize() {
    float range = this->maxValue - this->minValue;
    this->base = this->minValue + random::uniform() * range;
    this->clampMutation();
    this->applyMutation();
    this->isClean = false;
  }

  void resetMutation() {
    this->mutation = 0.0f;
    this->mutator.init();
    this->applyMutation();
    this->isClean = this->mutation == 0.0f && this->base == this->defaultValue;
  }

  void bakeMutation() {
    this->base = this->base + this->mutation;
    this->mutation = 0.0f;
    this->isClean = this->base == this->defaultValue;
  }
};

struct Step {
  int idx;
  float in_;
  bool isClean = true;
  float mutationStrength = 0.f;

  MutableValue attrs[STEP_ATTRS_TOTAL];

  bool gate;
  float *patternShift = nullptr;
  float *globalShift = nullptr;
  float *globalLen = nullptr;
  rack::engine::Port *exprCurvePort = nullptr;
  rack::engine::Port *exprPowerPort = nullptr;

  Mutator mutator;

  json_t* dataToJson() {
    json_t *stepJ = json_object();
    json_object_set_new(stepJ, "idx", json_integer(idx));
    json_object_set_new(stepJ, "in_", json_real(in_));
    json_object_set_new(stepJ, "gate", json_boolean(gate));
    json_t *attrsJ = json_array();
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      json_array_append(attrsJ, this->attrs[i].dataToJson());
    }
    json_object_set_new(stepJ, "attrs", attrsJ);
    return stepJ;
  }

  void dataFromJson(json_t *stepJ) {
    idx = json_integer_value(json_object_get(stepJ, "idx"));
    in_ = json_number_value(json_object_get(stepJ, "in_"));
    gate = json_boolean_value(json_object_get(stepJ, "gate"));
    json_t *attrsJ = json_object_get(stepJ, "attrs");
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      json_t *attrJ = json_array_get(attrsJ, i);
      attrs[i].dataFromJson(attrJ);
    }
    this->updateCleanFlag();
    this->updateMutatedFlag();
  }

  float out() {
    return in() + minLen();
  }
  float outBase() {
    return inBase() + minLenBase();
  }
  float fastOut(float inCache, float globalLen) {
    return inCache + this->attrs[STEP_LEN].value * globalLen;
  }
  float in() {
    return in_ + attrs[STEP_SHIFT].value + (patternShift ? *patternShift : 0.0f) + (globalShift ? *globalShift : 0.0f);
  }
  float inBase() {
    return in_ + attrs[STEP_SHIFT].base + (patternShift ? *patternShift : 0.0f) + (globalShift ? *globalShift : 0.0f);
  }
  float fastIn(float shift) {
    return in_ + attrs[STEP_SHIFT].value + shift;
  }
  void init(int i) {
    this->idx = i;
    this->in_ = i * baseStepLen;
    this->gate = true;
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      AttrDefaults defaults = getAttrDefaults(i);
      this->attrs[i].setup(
        defaults.defaultValue,
        defaults.minValue,
        defaults.maxValue,
        defaults.mutMult
      );
      this->attrs[i].init();
    }
    this->isClean = true;
  }
  float minLen() {
    return std::max(minStepLen, this->attrs[STEP_LEN].value * (globalLen ? *globalLen : 1.0f));
  }
  float minLenBase() {
    return std::max(minStepLen, this->attrs[STEP_LEN].base * (globalLen ? *globalLen : 1.0f));
  }
  void randomize() {
    this->gate = random::uniform() > 0.2f; // Because it's too boring when there is only few notes
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      this->attrs[i].randomize();
    }
    this->isClean = false;
  }
  inline float expr(float phase) {
    return curve(phase,
      clamp(this->attrs[STEP_EXPR_CURVE].value + (exprCurvePort ? exprCurvePort->getVoltage() * 0.2f : 0.0f), -1.0f, 1.0f),
      clamp(this->attrs[STEP_EXPR_POWER].value + (exprPowerPort ? exprPowerPort->getVoltage() * 0.2f : 0.0f), -1.0f, 1.0f),
      this->attrs[STEP_EXPR_IN].value,
      this->attrs[STEP_EXPR_OUT].value
    );
  }
  inline float exprBase(float phase) {
    return curve(phase,
      clamp(this->attrs[STEP_EXPR_CURVE].base + (exprCurvePort ? exprCurvePort->getVoltage() * 0.2f : 0.0f), -1.0f, 1.0f),
      clamp(this->attrs[STEP_EXPR_POWER].base + (exprPowerPort ? exprPowerPort->getVoltage() * 0.2f : 0.0f), -1.0f, 1.0f),
      this->attrs[STEP_EXPR_IN].base,
      this->attrs[STEP_EXPR_OUT].base
    );
  }
  float phase(float transportPhase) {
    float eucIn = fastmod(this->in(), 1.0f);
    if (transportPhase < eucIn) {
      return (transportPhase + 1.0f - eucIn) / this->minLen();
    } else {
      return (transportPhase - eucIn) / this->minLen();
    }
  }
  float phaseBase(float transportPhase) {
    float eucIn = fastmod(this->inBase(), 1.0f);
    if (transportPhase < eucIn) {
      return (transportPhase + 1.0f - eucIn) / this->minLenBase();
    } else {
      return (transportPhase - eucIn) / this->minLenBase();
    }
  }
  void quantize() {
    this->attrs[STEP_SHIFT].setValue(roundf(this->attrs[STEP_SHIFT].value * (1.0f / baseStepLen) * 2.0f) * baseStepLen * 0.5f);
  }
  void resetLength() {
    this->attrs[STEP_LEN].init();
  }
  void mutate(float factor) {
    float mutationAcc = 0.f;
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      this->attrs[i].mutate(factor);
      mutationAcc += std::abs(this->attrs[i].mutation);
    }
    this->mutationStrength = mutationAcc;
    if (factor > 0.0f) {
      this->isClean = false;
    } else {
      this->updateCleanFlag();
    }
    this->updateMutatedFlag();
  }
  void scaleMutation(float factor) {
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      this->attrs[i].scaleMutation(factor);
    }
    this->updateCleanFlag();
    this->updateMutatedFlag();
  }
  void resetMutation() {
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      this->attrs[i].resetMutation();
    }
    this->updateCleanFlag();
    this->mutationStrength = 0.f;
  }
  void updateIn() {
    this->in_ = this->idx * baseStepLen;
  }
  void setAttr(int attr, float factor) {
    this->attrs[attr].adjValue(factor);
    this->isClean = false;
  }
  void setAttrAbs(int attr, float target) {
    this->attrs[attr].setValue(target);
    this->updateCleanFlag();
    this->updateMutatedFlag();
  }
  void setAttrBase(int attr, float target) {
    this->attrs[attr].setBase(target);
    this->updateCleanFlag();
  }
  void updateCleanFlag() {
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      if (!this->attrs[i].isClean) {
        this->isClean = false;
        return;
      }
    }
    this->isClean = true;
  }
  void updateMutatedFlag() {
    float mutationAcc = 0.f;
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      mutationAcc += std::abs(this->attrs[i].mutation);
    }
    this->mutationStrength = mutationAcc;
  }
  void resetAttr(int attr) {
    this->attrs[attr].init();
    this->updateCleanFlag();
    this->mutationStrength = 0.f;
  }
  void bakeMutation() {
    for (int i = 0; i < STEP_ATTRS_TOTAL; i++) {
      this->attrs[i].bakeMutation();
    }
    this->updateCleanFlag();
    this->mutationStrength = 0.f;
  }
};

struct Pattern {
  float resolution = 8.0f;
  float goTo = 1.0f;
  float shift = 0.0f;
  Step steps[NUM_STEPS];
  float *globalShiftPtr = nullptr;
  float *globalLenPtr = nullptr;

  json_t *dataToJson() {
    json_t *patternJ = json_object();
    json_object_set_new(patternJ, "resolution", json_real(resolution));
    json_object_set_new(patternJ, "goTo", json_real(goTo));
    json_object_set_new(patternJ, "shift", json_real(shift));
    json_t *stepsJ = json_array();
    for (int i = 0; i < NUM_STEPS; i++) {
      json_array_append(stepsJ, steps[i].dataToJson());
    }
    json_object_set_new(patternJ, "steps", stepsJ);
    return patternJ;
  }

  void dataFromJson(json_t *patternJ) {
    resolution = json_number_value(json_object_get(patternJ, "resolution"));
    goTo = json_number_value(json_object_get(patternJ, "goTo"));
    shift = json_number_value(json_object_get(patternJ, "shift"));
    json_t *stepsJ = json_object_get(patternJ, "steps");
    for (int i = 0; i < NUM_STEPS; i++) {
      json_t *stepJ = json_array_get(stepsJ, i);
      steps[i].dataFromJson(stepJ);
    }
  }

  bool isClean() {
    if (resolution != 8.0f || goTo != 1.0f || shift != 0.0f) {
      return false;
    }
    for (int i = 0; i < NUM_STEPS; i++) {
      if (!steps[i].isClean) {
        return false;
      }
    }
    return true;
  }

  bool hasCustomSteps() {
    for (int i = 0; i < NUM_STEPS; i++) {
      if (!steps[i].isClean) {
        return true;
      }
    }
    return false;
  }

  Pattern() {
    init();
    refreshPointers(nullptr, nullptr, nullptr, nullptr);
  }
  void init() {
    resolution = 8.0f;
    shift = 0.0f;
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].init(i);
    }
  }
  void refreshPointers(float *globalShiftPtr, float *globalLenPtr,
                       rack::engine::Port *exprCurveInputPtr, rack::engine::Port *exprPowerInputPtr) {
    this->globalShiftPtr = globalShiftPtr;
    this->globalLenPtr = globalLenPtr;
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].patternShift = &shift;
      if (globalShiftPtr) { this->steps[i].globalShift = globalShiftPtr; }
      if (globalLenPtr) { this->steps[i].globalLen = globalLenPtr; }
      if (exprCurveInputPtr) { this->steps[i].exprCurvePort = exprCurveInputPtr; }
      if (exprPowerInputPtr) { this->steps[i].exprPowerPort = exprPowerInputPtr; }
    }
  }

  void randomize() {
		for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].randomize();
		}
  }

  void randomizeReso() {
    this->resolution = roundf(1.0f + random::uniform() * 98.0f);
  }

  // // Slow implementation
  // Step* getStepForPhase(float phase, bool globalGate) {
  //   Step* step = nullptr;
  //   float bestDist = 10.0f;
  //   for (int i = 0; i < NUM_STEPS; i++) {
  //     Step* curStep = &steps[i];
  //     if (!curStep->gate ^ !globalGate) { continue; }
  //     float eucIn = fastmod(curStep->in(), 1.0f);
  //     float eucOut = fastmod(curStep->out(), 1.0f);
  //     if (((eucIn < eucOut) ^ !((phase >= eucIn) ^ (phase < eucOut)))) { continue; }
  //     float delta = phase - 0.5f;
  //     float in = fastmod(eucIn - delta, 1.0f);
  //     float dist = fabsf(0.5f - in);
  //     if (dist > bestDist) { continue; }
  //     step = curStep;
  //     bestDist = dist;
  //   }
  //   return step;
  // }

  Step* getStepForPhase(float phase, bool globalGate) {
    Step* step = nullptr;
    float localShift = shift + *globalShiftPtr;
    float localLen = *globalLenPtr;
    float prePhase = phase - 1.0f;
    float postPhase = phase + 1.0f;
    float bestRating = 10.0f;

    for (int i = 0; i < NUM_STEPS; i++) {
      Step* curStep = &steps[i];
      if (!curStep->gate ^ !globalGate) { continue; }
      float stepIn = curStep->in_ + curStep->attrs[STEP_SHIFT].value + localShift;
      float stepOut = stepIn + curStep->attrs[STEP_LEN].value * localLen;
      float rating = 10.0f;

      if (stepIn <= phase && phase < stepOut) {
        rating = phase - stepIn;
      } else if (stepIn <= prePhase && prePhase < stepOut) {
        rating = prePhase - stepIn;
      } else if (stepIn <= postPhase && postPhase < stepOut) {
        rating = postPhase - stepIn;
      } else {
        continue;
      }

      if (rating < bestRating) {
        bestRating = rating;
        step = curStep;
      }
    }

    return step;
  }

  void updateStepsStates(float phase, bool globalGate, bool *states, bool unison) {
    float localShift = shift + *globalShiftPtr;
    float localLen = *globalLenPtr;
    float prePhase = phase - 1.0f;
    float postPhase = phase + 1.0f;

    for (int i = 0; i < NUM_STEPS; i++) {
      Step* curStep = &steps[i];
      if (!curStep->gate ^ !globalGate) {
        states[i] = false;
        continue;
      }
      float stepIn = curStep->in_ + (unison ? curStep->attrs[STEP_SHIFT].base : curStep->attrs[STEP_SHIFT].value) + localShift;
      float stepOut = stepIn + (unison ? curStep->attrs[STEP_LEN].base : curStep->attrs[STEP_LEN].value) * localLen;

      if (stepIn <= phase && phase < stepOut) {
        states[i] = true;
      } else if (stepIn <= prePhase && prePhase < stepOut) {
        states[i] = true;
      } else if (stepIn <= postPhase && postPhase < stepOut) {
        states[i] = true;
      } else {
        states[i] = false;
      }
    }
  }

  void quantize() {
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].quantize();
    }
    this->shift = 0.0f;
  }
  void shiftLeft() {
    std::rotate(&this->steps[0], &this->steps[1], &this->steps[NUM_STEPS]);
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].in_ = fastmod(this->steps[i].in_ - baseStepLen, 1.0f);
      this->steps[i].idx = eucMod(this->steps[i].idx - 1, NUM_STEPS);
    }
  }
  void shiftRight() {
    std::rotate(&this->steps[0], &this->steps[NUM_STEPS - 1], &this->steps[NUM_STEPS]);
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].in_ = fastmod(this->steps[i].in_ + baseStepLen, 1.0f);
      this->steps[i].idx = eucMod(this->steps[i].idx + 1, NUM_STEPS);
    }
  }
  void resetLenghts() {
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].resetLength();
    }
  }
  void mutate(float factor) {
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].mutate(factor);
    }
  }
  void scaleMutation(float factor) {
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].scaleMutation(factor);
    }
  }
  void resetMutation() {
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].resetMutation();
    }
  }
  void reverse() {
    Step reversed[NUM_STEPS];
    for (int i = 0; i < NUM_STEPS; i++) {
      Step orig = this->steps[NUM_STEPS - i - 1];
      reversed[i] = orig;
      reversed[i].idx = i;
      reversed[i].updateIn();
      float newShiftBase = baseStepLen - orig.attrs[STEP_LEN].base - orig.attrs[STEP_SHIFT].base;
      float newShiftMut = -orig.attrs[STEP_LEN].mutation - orig.attrs[STEP_SHIFT].mutation;
      reversed[i].attrs[STEP_SHIFT].setBase(newShiftBase);
      reversed[i].attrs[STEP_SHIFT].setMutation(newShiftMut);
      reversed[i].attrs[STEP_SHIFT].applyMutation();
      reversed[i].attrs[STEP_EXPR_IN] = orig.attrs[STEP_EXPR_OUT];
      reversed[i].attrs[STEP_EXPR_OUT] = orig.attrs[STEP_EXPR_IN];
      reversed[i].attrs[STEP_EXPR_POWER].setBase(reversed[i].attrs[STEP_EXPR_POWER].base * -1.0f);
      reversed[i].attrs[STEP_EXPR_POWER].setMutation(reversed[i].attrs[STEP_EXPR_POWER].mutation * -1.0f);
      reversed[i].attrs[STEP_EXPR_POWER].applyMutation();
    }
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i] = reversed[i];
    }
  }
  void flip() {
    float low = 2.0f;
    float high = -2.0f;
    for (int i = 0; i < NUM_STEPS; i++) {
      low = fminf(low, this->steps[i].attrs[STEP_VALUE].base);
      high = fmaxf(high, this->steps[i].attrs[STEP_VALUE].base);
    }
    float range = high - low;
    for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].attrs[STEP_VALUE].setBase(range - (this->steps[i].attrs[STEP_VALUE].base - low) + low);
      this->steps[i].attrs[STEP_VALUE].setMutation(this->steps[i].attrs[STEP_VALUE].mutation * -1.0f);
      this->steps[i].attrs[STEP_VALUE].applyMutation();
    }
  }
  void bakeMutation() {
		for (int i = 0; i < NUM_STEPS; i++) {
      this->steps[i].bakeMutation();
		}
  }
};
