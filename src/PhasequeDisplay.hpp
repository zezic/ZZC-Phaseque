#include "Phaseque.hpp"

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../ZZC/src/widgets.hpp"
#endif

struct PatternDisplayWidget : BaseDisplayWidget {
  Pattern *pattern = nullptr;
  Step **activeStep = nullptr;
  float *resolution = nullptr;
  float *phase = nullptr;
  int *direction = nullptr;
  float *globalValueInput = nullptr;
  float *globalValueParam = nullptr;
  bool *globalGate = nullptr;
  Pattern dummyPattern;
  int *polyphonyMode = nullptr;
  bool *stepsStates = nullptr;
  bool *unisonStates = nullptr;

  NVGcolor lcdGhostColor = nvgRGB(0x1e, 0x1f, 0x1d);

  NVGcolor lcdActiveColor = nvgRGB(0xff, 0xd4, 0x2a);
  NVGcolor lcdDimmedColor = nvgRGB(0xa0, 0x80, 0x00);
  NVGcolor lcdDisabledColor = nvgRGB(0x36, 0x2b, 0x00);

  NVGcolor lcdActiveMutColor = nvgRGB(0x2e, 0xc6, 0xff);
  NVGcolor lcdDimmedMutColor = nvgRGB(0x32, 0x95, 0xcc);
  NVGcolor lcdDisabledMutColor = nvgRGB(0x30, 0x4e, 0x5e);

  std::shared_ptr<Font> font;

  float padding = 6.0f;
  Vec area;
  float crossLine1;
  float dashLine1;
  float crossLine2;
  float dashLine2;
  float crossLine3;
  float exprHorizont;
  float stepX;
  float stepY;
  bool ready = false;

  PatternDisplayWidget() {
    font = APP->window->loadFont(asset::plugin(pluginInstance, "res/fonts/Nunito/Nunito-Black.ttf"));
  }

  void setupSizes() {
    area = Vec(box.size.x - padding * 2.0f, box.size.y - padding * 2.0f);
    stepX = (area.x - 1) / NUM_STEPS;
    stepY = (area.y) / 6;
    crossLine1 = stepY * 1.0f;
    dashLine1 = stepY * 2.0f;
    crossLine2 = stepY * 3.0f;
    dashLine2 = stepY * 4.0f;
    crossLine3 = stepY * 5.0f;
    exprHorizont = crossLine3 + stepY / 2.0f;
    ready = true;
  }

  void drawDash(const DrawArgs &args, Vec position) {
    nvgStrokeColor(args.vg, lcdGhostColor);
    nvgStrokeWidth(args.vg, 1.0f);

    nvgBeginPath(args.vg);
    nvgMoveTo(args.vg, position.x - 5.5f, position.y);
    nvgLineTo(args.vg, position.x + 5.5f, position.y);
    nvgStroke(args.vg);
  }

  void drawCross(const DrawArgs &args, Vec position) {
    drawDash(args, position);

    nvgBeginPath(args.vg);
    nvgMoveTo(args.vg, position.x, position.y - 5.5f);
    nvgLineTo(args.vg, position.x, position.y + 5.5f);
    nvgStroke(args.vg);
  }

  void drawResolution(const DrawArgs &args) {
    float resolutionVal = resolution ? *resolution : 8.0;
    float phaseVal = phase ? *phase : 0.0f;
    nvgStrokeWidth(args.vg, 1.0f);
    float periodSize = area.x / resolutionVal;
    for (float i = 0.0f; i < resolutionVal; i = i + 1.0f) {
      if (phaseVal * resolutionVal >= i && phaseVal * resolutionVal < i + 1) {
        nvgStrokeColor(args.vg, lcdActiveColor);
      } else {
        nvgStrokeColor(args.vg, lcdDisabledColor);
      }
      nvgBeginPath(args.vg);
      nvgMoveTo(args.vg, padding + periodSize * i, padding + crossLine1);
      nvgLineTo(args.vg, padding + periodSize * (i + 1), padding);
      nvgStroke(args.vg);
    }
  }

  void drawExprCurve(const DrawArgs &args, float x1, float workArea, Step step, bool mutated) {
    float len = workArea * (mutated ? step.minLen() : step.minLenBase());
    nvgBeginPath(args.vg);
    for (float phase = 0.0f; phase <= 1.000f; phase += 0.025) {
      float cx = x1 + len * phase;
      float cy = mutated ? step.expr(phase) : step.exprBase(phase);
      if (phase == 0.0f) {
        nvgMoveTo(args.vg, cx, exprHorizont + 6 + stepY * cy * -0.5);
        continue;
      }
      nvgLineTo(args.vg, cx, exprHorizont + 6 + stepY * cy * -0.5);
    }
    nvgStroke(args.vg);
  }

  void drawStep(const DrawArgs &args, Step step, bool mutated) {
    int dir = direction ? *direction : 1;
    float radius = 2.5f;
    float workArea = area.x - 1;
    float start = padding;
    float end = padding + workArea;
    float x1 = padding + workArea * eucMod(mutated ? step.in() : step.inBase(), 1.0f);
    float x2 = padding + workArea * eucMod(mutated ? step.out() : step.outBase(), 1.0f);
    float lineX1 = x1 + (dir == 1 ? radius * 1.8f : 0.f);
    float lineX2 = x2 - (dir == -1 ? radius * 1.8f : 0.f);
    float y = padding + crossLine2 + -stepY * (mutated ? step.attrs[STEP_VALUE].value : step.attrs[STEP_VALUE].base);

    // Step
    if (x2 >= x1) {
      nvgBeginPath(args.vg);
      nvgMoveTo(args.vg, lineX1, y);
      nvgLineTo(args.vg, lineX2, y);
      nvgStroke(args.vg);
    } else {
      nvgBeginPath(args.vg);
      nvgMoveTo(args.vg, lineX1, y);
      nvgLineTo(args.vg, end, y);
      nvgStroke(args.vg);
      nvgBeginPath(args.vg);
      nvgMoveTo(args.vg, start, y);
      nvgLineTo(args.vg, lineX2, y);
      nvgStroke(args.vg);
    }

    // Circle
    nvgBeginPath(args.vg);
    nvgCircle(args.vg, dir == 1 ? x1 + 2 : fastmod(x2 - 2 - padding, area.x) + padding, y, radius);
    nvgFill(args.vg);

    // Expression curve
    drawExprCurve(args, x1, workArea, step, mutated);
    if (x2 < x1) {
      drawExprCurve(args, x1 - workArea - 1, workArea, step, mutated);
    }
  }

  void drawSteps(const DrawArgs &args) {
    nvgStrokeWidth(args.vg, 1.0f);
    nvgScissor(args.vg, padding - 1, padding - 1, area.x + 2, area.y + 2);
    nvgGlobalCompositeOperation(args.vg, NVG_LIGHTER);

    if (pattern && globalGate && polyphonyMode && stepsStates && unisonStates) {
      for (int i = 0; i < NUM_STEPS; i++) {
        if (!pattern->steps[i].gate ^ !*globalGate) {
          nvgStrokeColor(args.vg, lcdDisabledColor);
          nvgFillColor(args.vg, lcdDisabledColor);
          nvgGlobalAlpha(args.vg, 1.f);
          drawStep(args, pattern->steps[i], false);
          if (pattern->steps[i].mutationStrength != 0.f) {
            nvgStrokeColor(args.vg, lcdDisabledMutColor);
            nvgFillColor(args.vg, lcdDisabledMutColor);
            nvgGlobalAlpha(args.vg, std::min(pattern->steps[i].mutationStrength, 1.f));
            drawStep(args, pattern->steps[i], true);
          }
        }
        if (pattern->steps[i].gate ^ !*globalGate) {
          nvgStrokeColor(args.vg, lcdDimmedColor);
          nvgFillColor(args.vg, lcdDimmedColor);
          nvgGlobalAlpha(args.vg, 1.f);
          drawStep(args, pattern->steps[i], false);
          if (pattern->steps[i].mutationStrength != 0.f) {
            nvgStrokeColor(args.vg, lcdDimmedMutColor);
            nvgFillColor(args.vg, lcdDimmedMutColor);
            nvgGlobalAlpha(args.vg, std::min(pattern->steps[i].mutationStrength, 1.f));
            drawStep(args, pattern->steps[i], true);
          }
        }
        nvgStrokeColor(args.vg, lcdActiveColor);
        nvgFillColor(args.vg, lcdActiveColor);
        nvgGlobalAlpha(args.vg, 1.f);
        if (*polyphonyMode == POLYPHONIC) {
          if (stepsStates[i]) {
            drawStep(args, pattern->steps[i], false);
            if (pattern->steps[i].mutationStrength != 0.f) {
              nvgStrokeColor(args.vg, lcdActiveMutColor);
              nvgFillColor(args.vg, lcdActiveMutColor);
              nvgGlobalAlpha(args.vg, std::min(pattern->steps[i].mutationStrength, 1.f));
              drawStep(args, pattern->steps[i], true);
            }
          }
        } else if (*polyphonyMode == UNISON) {
          if (unisonStates[i]) {
            drawStep(args, pattern->steps[i], false);
          }
          if (stepsStates[i]) {
            if (pattern->steps[i].mutationStrength != 0.f) {
              nvgStrokeColor(args.vg, lcdActiveMutColor);
              nvgFillColor(args.vg, lcdActiveMutColor);
              nvgGlobalAlpha(args.vg, std::min(pattern->steps[i].mutationStrength, 1.f));
              drawStep(args, pattern->steps[i], true);
            }
          }
        }
      }
      if (*polyphonyMode == MONOPHONIC && *activeStep) {
        Step *activeStepPtr = *activeStep;
        if (activeStepPtr) {
          Step activeStepCopy = *activeStepPtr;
          drawStep(args, activeStepCopy, false);
          if (activeStepCopy.mutationStrength != 0.f) {
            nvgStrokeColor(args.vg, lcdActiveMutColor);
            nvgFillColor(args.vg, lcdActiveMutColor);
            nvgGlobalAlpha(args.vg, std::min(activeStepCopy.mutationStrength, 1.f));
            drawStep(args, activeStepCopy, true);
          }
        }
      }
    } else {
      nvgStrokeColor(args.vg, lcdDimmedColor);
      nvgFillColor(args.vg, lcdDimmedColor);
      nvgGlobalAlpha(args.vg, 1.f);
      dummyPattern.init();
      for (int i = 0; i < NUM_STEPS; i++) {
        drawStep(args, dummyPattern.steps[i], false);
      }
      nvgStrokeColor(args.vg, lcdActiveColor);
      nvgFillColor(args.vg, lcdActiveColor);
      drawStep(args, dummyPattern.steps[0], false);
    }

    nvgResetScissor(args.vg);
    nvgGlobalCompositeOperation(args.vg, NVG_SOURCE_OVER);
  }

  void draw(const DrawArgs &args) override {
    drawBackground(args);
    if (!ready) { return; }

    for (int i = 1; i < NUM_STEPS; i++) {
      drawDash(args, Vec(padding + i * stepX, padding + dashLine1));
      drawCross(args, Vec(padding + i * stepX, padding + crossLine1));
      drawDash(args, Vec(padding + i * stepX, padding + dashLine2));
      drawCross(args, Vec(padding + i * stepX, padding + crossLine2));
      drawCross(args, Vec(padding + i * stepX, padding + crossLine3));
    }
    drawResolution(args);
    drawSteps(args);
  }
};
