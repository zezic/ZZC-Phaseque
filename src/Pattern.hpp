#include "ZZC.hpp"
#include "Step.hpp"

struct Pattern {
  float resolution = 8.0f;
  unsigned int goTo = 0;
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
    this->resolution = 8.0f;
    this->shift = 0.0f;
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


