#pragma once
#include "ZZC.hpp"
#include "Phaseque.hpp"

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

struct PhasequeStepAttrChange : history::ModuleAction {
  int paramId;
  unsigned int patternNum;
  int step;
  int attr;
  float oldValue;
  float newValue;

  void undo() override {
    // TODO: Implement this
    // app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    // assert(mw);
    // Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    // phaseq->patterns[patternNum].steps[step].setAttrBase(attr, oldValue);
    // if (phaseq->patternIdx == patternNum) {
    //   phaseq->pattern.steps[step].setAttrBase(attr, oldValue);
    //   mw->module->params[paramId].value = oldValue;
    // } else {
    //   phaseq->patternFlashNeg = patternNum;
    // }
  }

  void redo() override {
    // TODO: Implement this
    // app::ModuleWidget *mw = APP->scene->rack->getModule(moduleId);
    // assert(mw);
    // Phaseque* phaseq = static_cast<Phaseque*>(mw->module);
    // phaseq->patterns[patternNum].steps[step].setAttrBase(attr, newValue);
    // if (phaseq->patternIdx == patternNum) {
    //   phaseq->pattern.steps[step].setAttrBase(attr, newValue);
    //   mw->module->params[paramId].value = newValue;
    // } else {
    //   phaseq->patternFlashPos = patternNum;
    // }
  }

  PhasequeStepAttrChange() {
    name = "change step attribute";
  }
};
