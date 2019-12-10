#pragma once
#include "ZZC.hpp"
#include "Step.hpp"


inline simd::float_4 eucMod(simd::float_4 a, simd::float_4 b) {
  simd::float_4 mod = simd::fmod(a, b);
  return simd::ifelse(mod < 0.f, mod + b, mod);
}

#define BLOCK_SIZE 4

template <unsigned int SIZE>
struct Pattern {
  /* Pattern settings */
  unsigned int resolution = SIZE;
  unsigned int goTo = 0;
  float shift = 0.f;

  unsigned int size = SIZE;
  unsigned int blockSize = BLOCK_SIZE;
  float baseStepLen = 1.f / SIZE;

  unsigned int activeStepIdx = 0;
  unsigned int activeBlockMask = 0;
  unsigned int activeBlockIdx = 0;
  bool hasActiveStep = false;

  simd::float_4 hitsTemp[SIZE / BLOCK_SIZE];

  /* Step attributes, mutations and gates */
  simd::float_4 stepBases[STEP_ATTRS_TOTAL][SIZE / BLOCK_SIZE];
  simd::float_4 stepMutas[STEP_ATTRS_TOTAL][SIZE / BLOCK_SIZE];
  simd::float_4 stepGates[SIZE / BLOCK_SIZE];
  simd::float_4 stepIns[SIZE / BLOCK_SIZE];

  simd::float_4 stepInsComputed[SIZE / BLOCK_SIZE];
  simd::float_4 stepOutsComputed[SIZE / BLOCK_SIZE];

  simd::float_4 stepAttrDefaults[STEP_ATTRS_TOTAL] = {
    simd::float_4(0.f),
    simd::float_4(1.f / SIZE),
    simd::float_4(0.f),
    simd::float_4(0.f),
    simd::float_4(0.f),
    simd::float_4(0.f),
    simd::float_4(0.f),
  };

  void findStepsForPhase(float phase) {
    /* Search for phase-step intersection */
    for (unsigned int blockIdx = 0; blockIdx < SIZE / BLOCK_SIZE; blockIdx++) {
      simd::float_4 inMod = this->stepInsComputed[blockIdx];
      simd::float_4 outMod = this->stepOutsComputed[blockIdx];
      this->hitsTemp[blockIdx] = (inMod <= phase) ^ (phase < outMod) ^ (inMod < outMod);
    }
  }

  void findMonoStep() {
    static int masks[4] = { 1, 2, 4, 8 };
    for (unsigned int blockIdx = SIZE / BLOCK_SIZE - 1; blockIdx >= 0; blockIdx--) {
      int blockMask = simd::movemask(this->hitsTemp[blockIdx]);
      if (blockMask == 0) { continue; } // No hits in this block
      for (unsigned int stepInBlockIdx = 3; stepInBlockIdx >= 0; stepInBlockIdx--) {
        if (masks[stepInBlockIdx] & blockMask) {
          this->hasActiveStep = true;
          this->activeStepIdx = blockIdx * BLOCK_SIZE + stepInBlockIdx;
          this->activeBlockMask = masks[stepInBlockIdx];
          this->activeBlockIdx = blockIdx;
          return;
        }
      }
    }
  }

  void recalcInOuts(unsigned int blockIdx) {
    this->stepInsComputed[blockIdx] = eucMod(this->stepIns[blockIdx] + this->stepBases[StepAttr::STEP_SHIFT][blockIdx], 1.f);
    this->stepOutsComputed[blockIdx] = eucMod(this->stepInsComputed[blockIdx] + this->stepBases[StepAttr::STEP_LEN][blockIdx], 1.f);
  }

  json_t *dataToJson() {
    json_t *patternJ = json_object();
    json_object_set_new(patternJ, "resolution", json_integer(resolution));
    json_object_set_new(patternJ, "goTo", json_integer(goTo));
    json_object_set_new(patternJ, "shift", json_real(shift));
    json_t *stepsJ = json_array();
    for (unsigned int stepIdx = 0; stepIdx < SIZE; stepIdx++) {
      unsigned int blockIdx = stepIdx / BLOCK_SIZE;
      unsigned int stepInBlockIdx = stepIdx % BLOCK_SIZE;

      json_t *stepJ = json_object();
      json_object_set_new(stepJ, "gate", json_boolean(
        simd::movemask(this->stepGates[blockIdx]) & 1 << stepInBlockIdx
      ));
      json_t *attrsJ = json_array();
      for (unsigned int attrIdx = 0; attrIdx < STEP_ATTRS_TOTAL; attrIdx++) {
        json_t *attrJ = json_object();
        json_object_set_new(attrJ, "base", json_real(
          this->stepBases[attrIdx][blockIdx][stepInBlockIdx]
        ));
        json_object_set_new(attrJ, "mutation", json_real(
          this->stepMutas[attrIdx][blockIdx][stepInBlockIdx]
        ));
        json_array_append(attrsJ, attrJ);
      }
      json_object_set_new(stepJ, "attrs", attrsJ);
      json_array_append(stepsJ, stepJ);
    }
    json_object_set_new(patternJ, "steps", stepsJ);
    return patternJ;
  }

  void dataFromJson(json_t *patternJ) {
    this->resolution = json_number_value(json_object_get(patternJ, "resolution"));
    this->goTo = json_number_value(json_object_get(patternJ, "goTo"));
    this->shift = json_number_value(json_object_get(patternJ, "shift"));
    json_t *stepsJ = json_object_get(patternJ, "steps");
    for (unsigned int stepIdx = 0; stepIdx < SIZE; stepIdx++) {
      unsigned int blockIdx = stepIdx / BLOCK_SIZE;
      unsigned int stepInBlockIdx = stepIdx % BLOCK_SIZE;
      json_t *stepJ = json_array_get(stepsJ, stepIdx);
      bool gate = json_boolean_value(json_object_get(stepJ, "gate"));
      if (!gate) {
        this->stepGates[blockIdx] ^= simd::movemaskInverse<simd::float_4>(1 << stepInBlockIdx);
      }
      json_t *attrsJ = json_object_get(stepJ, "attrs");
      for (unsigned int attrIdx = 0; attrIdx < STEP_ATTRS_TOTAL; attrIdx++) {
        json_t *attrJ = json_array_get(attrsJ, attrIdx);
        this->stepBases[attrIdx][blockIdx][stepInBlockIdx] = json_number_value(json_object_get(attrJ, "base"));
        this->stepMutas[attrIdx][blockIdx][stepInBlockIdx] = json_number_value(json_object_get(attrJ, "mutation"));
      }
    }
    for (unsigned int blockIdx = 0; blockIdx < SIZE / BLOCK_SIZE; blockIdx++) {
      this->recalcInOuts(blockIdx);
    }
  }

  bool isClean() {
    if (resolution != 8 || shift != 0.f) {
      return false;
    }
    for (unsigned int attrIdx = 0; attrIdx < STEP_ATTRS_TOTAL; attrIdx++) {
      for (unsigned int blockIdx = 0; blockIdx  < SIZE / BLOCK_SIZE; blockIdx++) {
        if (simd::movemask(this->stepBases[attrIdx][blockIdx] != this->stepAttrDefaults[attrIdx]) ||
            simd::movemask(this->stepMutas[attrIdx][blockIdx] != 0.f)) {
          return false;
        }
      }
    }
    return true;
  }

  bool hasCustomSteps() {
    for (unsigned int attrIdx = 0; attrIdx < STEP_ATTRS_TOTAL; attrIdx++) {
      for (unsigned int blockIdx = 0; blockIdx  < SIZE / BLOCK_SIZE; blockIdx++) {
        if (simd::movemask(this->stepBases[attrIdx][blockIdx] != this->stepAttrDefaults[attrIdx]) ||
            simd::movemask(this->stepMutas[attrIdx][blockIdx] != 0.f)) {
          return true;
        }
      }
    }
    return false;
  }

  Pattern() {
    this->init();
  }

  void init() {
    this->resolution = SIZE;
    this->shift = 0.f;
    for (unsigned int attrIdx = 0; attrIdx < STEP_ATTRS_TOTAL; attrIdx++) {
      for (unsigned int blockIdx = 0; blockIdx  < SIZE / BLOCK_SIZE; blockIdx++) {
        this->stepBases[attrIdx][blockIdx] = this->stepAttrDefaults[attrIdx];
        this->stepMutas[attrIdx][blockIdx] = 0.f;
      }
    }
    for (unsigned int stepIdx = 0; stepIdx < SIZE; stepIdx++) {
      unsigned int blockIdx = stepIdx / BLOCK_SIZE;
      unsigned int stepInBlockIdx = stepIdx % BLOCK_SIZE;
      this->stepIns[blockIdx][stepInBlockIdx] = 1.f / SIZE * stepIdx;
    }
    for (unsigned int blockIdx = 0; blockIdx < SIZE / BLOCK_SIZE; blockIdx++) {
      this->stepGates[blockIdx] = simd::float_4::mask();
      this->recalcInOuts(blockIdx);
    }
  }

  void randomize() {
    // TODO: Implement this
  }

  void randomizeReso() {
    this->resolution = 1 + std::round(99 * random::uniform());
  }

  void resetResolution() {
    this->resolution = SIZE;
  }

  void quantize() {
    // TODO: Implement this
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].quantize();
    // }
    // this->shift = 0.f;
  }

  void shiftLeft() {
    // TODO: Implement this
    // std::rotate(&this->steps[0], &this->steps[1], &this->steps[NUM_STEPS]);
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].in_ = fastmod(this->steps[i].in_ - baseStepLen, 1.0f);
    //   this->steps[i].idx = eucMod(this->steps[i].idx - 1, NUM_STEPS);
    // }
  }

  void shiftRight() {
    // TODO: Implement this
    // std::rotate(&this->steps[0], &this->steps[NUM_STEPS - 1], &this->steps[NUM_STEPS]);
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].in_ = fastmod(this->steps[i].in_ + baseStepLen, 1.0f);
    //   this->steps[i].idx = eucMod(this->steps[i].idx + 1, NUM_STEPS);
    // }
  }

  void resetLenghts() {
    for (unsigned int blockIdx = 0; blockIdx  < SIZE / BLOCK_SIZE; blockIdx++) {
      this->stepBases[StepAttr::STEP_LEN][blockIdx] = this->stepAttrDefaults[StepAttr::STEP_LEN];
    }
  }

  void mutate(float factor) {
    // TODO: Implement this
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].mutate(factor);
    // }
  }
  void scaleMutation(float factor) {
    // TODO: Implement this
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].scaleMutation(factor);
    // }
  }
  void resetMutation() {
    // TODO: Implement this
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].resetMutation();
    // }
  }
  void reverse() {
    // TODO: Implement this
    // Step reversed[NUM_STEPS];
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   Step orig = this->steps[NUM_STEPS - i - 1];
    //   reversed[i] = orig;
    //   reversed[i].idx = i;
    //   reversed[i].updateIn();
    //   float newShiftBase = baseStepLen - orig.attrs[STEP_LEN].base - orig.attrs[STEP_SHIFT].base;
    //   float newShiftMut = -orig.attrs[STEP_LEN].mutation - orig.attrs[STEP_SHIFT].mutation;
    //   reversed[i].attrs[STEP_SHIFT].setBase(newShiftBase);
    //   reversed[i].attrs[STEP_SHIFT].setMutation(newShiftMut);
    //   reversed[i].attrs[STEP_SHIFT].applyMutation();
    //   reversed[i].attrs[STEP_EXPR_IN] = orig.attrs[STEP_EXPR_OUT];
    //   reversed[i].attrs[STEP_EXPR_OUT] = orig.attrs[STEP_EXPR_IN];
    //   reversed[i].attrs[STEP_EXPR_POWER].setBase(reversed[i].attrs[STEP_EXPR_POWER].base * -1.0f);
    //   reversed[i].attrs[STEP_EXPR_POWER].setMutation(reversed[i].attrs[STEP_EXPR_POWER].mutation * -1.0f);
    //   reversed[i].attrs[STEP_EXPR_POWER].applyMutation();
    // }
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i] = reversed[i];
    // }
  }
  void flip() {
    // TODO: Implement this
    // float low = 2.0f;
    // float high = -2.0f;
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   low = fminf(low, this->steps[i].attrs[STEP_VALUE].base);
    //   high = fmaxf(high, this->steps[i].attrs[STEP_VALUE].base);
    // }
    // float range = high - low;
    // for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].attrs[STEP_VALUE].setBase(range - (this->steps[i].attrs[STEP_VALUE].base - low) + low);
    //   this->steps[i].attrs[STEP_VALUE].setMutation(this->steps[i].attrs[STEP_VALUE].mutation * -1.0f);
    //   this->steps[i].attrs[STEP_VALUE].applyMutation();
    // }
  }
  void bakeMutation() {
    // TODO: Implement this
		// for (int i = 0; i < NUM_STEPS; i++) {
    //   this->steps[i].bakeMutation();
		// }
  }
};


