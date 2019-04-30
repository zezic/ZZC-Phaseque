#ifndef PHASEQ_H
#define PHASEQ_H
#include "Phaseque.hpp"
#endif

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../../ZZC/src/widgets.hpp"
#endif

struct PatternDisplayWidget : BaseDisplayWidget {
  Pattern *pattern;
  Step **activeStep;
  float *resolution;
  float *phase;
  int *direction;
  float *globalValueInput;
  float *globalValueParam;
  bool *globalGate;

  NVGcolor lcdGhostColor = nvgRGB(0x1e, 0x1f, 0x1d);
  NVGcolor lcdActiveColor = nvgRGB(0xff, 0xd4, 0x2a);
  NVGcolor lcdDimmedColor = nvgRGB(0xa0, 0x80, 0x00);
  NVGcolor lcdDisabledColor = nvgRGB(0x36, 0x2b, 0x00);
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
    font = Font::load(assetPlugin(plugin, "res/fonts/Nunito/Nunito-Black.ttf"));
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

  void drawDash(NVGcontext *vg, Vec position) {
		nvgStrokeColor(vg, lcdGhostColor);
		nvgStrokeWidth(vg, 1.0f);

    nvgBeginPath(vg);
    nvgMoveTo(vg, position.x - 5.5f, position.y);
    nvgLineTo(vg, position.x + 5.5f, position.y);
		nvgStroke(vg);
  }

  void drawCross(NVGcontext *vg, Vec position) {
    drawDash(vg, position);

    nvgBeginPath(vg);
    nvgMoveTo(vg, position.x, position.y - 5.5f);
    nvgLineTo(vg, position.x, position.y + 5.5f);
		nvgStroke(vg);
  }

  void drawResolution(NVGcontext *vg) {
		nvgStrokeWidth(vg, 1.0f);
    float periodSize = area.x / *resolution;
    for (float i = 0.0f; i < *resolution; i = i + 1.0f) {
      if (*phase * *resolution >= i && *phase * *resolution < i + 1) {
        nvgStrokeColor(vg, lcdActiveColor);
      } else {
        nvgStrokeColor(vg, lcdDisabledColor);
      }
      nvgBeginPath(vg);
      nvgMoveTo(vg, padding + periodSize * i, padding + crossLine1);
      nvgLineTo(vg, padding + periodSize * (i + 1), padding);
      nvgStroke(vg);
    }
  }

  void drawExprCurve(NVGcontext *vg, float x1, float workArea, Step step) {
    float len = workArea * step.minLen();
    nvgBeginPath(vg);
    for (float phase = 0.0f; phase <= 1.000f; phase += 0.025) {
      float cx = x1 + len * phase;
      float cy = step.expr(phase);
      if (phase == 0.0f) {
        nvgMoveTo(vg, cx, exprHorizont + 6 + stepY * cy * -0.5);
        continue;
      }
      nvgLineTo(vg, cx, exprHorizont + 6 + stepY * cy * -0.5);
    }
    nvgStroke(vg);
  }

  void drawStep(NVGcontext *vg, Step step) {
    float workArea = area.x - 1;
    float start = padding;
    float end = padding + workArea;
    float x1 = padding + workArea * eucmod(step.in(), 1.0f);
    float x2 = padding + workArea * eucmod(step.out(), 1.0f);
    float y = padding + crossLine2 + -stepY * step.attrs[STEP_VALUE].value;

    // Step
    if (x2 >= x1) {
      nvgBeginPath(vg);
      nvgMoveTo(vg, x1, y);
      nvgLineTo(vg, x2, y);
      nvgStroke(vg);
    } else {
      nvgBeginPath(vg);
      nvgMoveTo(vg, x1, y);
      nvgLineTo(vg, end, y);
      nvgStroke(vg);
      nvgBeginPath(vg);
      nvgMoveTo(vg, start, y);
      nvgLineTo(vg, x2, y);
      nvgStroke(vg);
    }

    // Circle
    nvgBeginPath(vg);
    nvgCircle(vg, *direction == 1 ? x1 + 2 : fastmod(x2 - 2 - padding, area.x) + padding, y, 2.5f);
    nvgFill(vg);

    // Expression curve
    drawExprCurve(vg, x1, workArea, step);
    if (x2 < x1) {
      drawExprCurve(vg, x1 - workArea - 1, workArea, step);
    }
  }

  void drawSteps(NVGcontext *vg) {
		nvgStrokeWidth(vg, 1.0f);
    nvgScissor(vg, padding - 1, padding - 1, area.x + 2, area.y + 2);
    nvgStrokeColor(vg, lcdDisabledColor);
    nvgFillColor(vg, lcdDisabledColor);
    for (int i = 0; i < NUM_STEPS; i++) {
      if (!pattern->steps[i].gate ^ !*globalGate) {
        drawStep(vg, pattern->steps[i]);
      }
    }
    nvgStrokeColor(vg, lcdDimmedColor);
    nvgFillColor(vg, lcdDimmedColor);
    for (int i = 0; i < NUM_STEPS; i++) {
      if (pattern->steps[i].gate ^ !*globalGate) {
        drawStep(vg, pattern->steps[i]);
      }
    }
    if (*activeStep) {
      Step *activeStepPtr = *activeStep;
      if (activeStepPtr) {
        Step activeStepCopy = *activeStepPtr;
        nvgStrokeColor(vg, lcdActiveColor);
        nvgFillColor(vg, lcdActiveColor);
        drawStep(vg, activeStepCopy);
      }
    }
    nvgResetScissor(vg);
  }

  void draw(NVGcontext *vg) override {
    drawBackground(vg);
    if (!ready) { return; }

    for (int i = 1; i < NUM_STEPS; i++) {
      drawDash(vg, Vec(padding + i * stepX, padding + dashLine1));
      drawCross(vg, Vec(padding + i * stepX, padding + crossLine1));
      drawDash(vg, Vec(padding + i * stepX, padding + dashLine2));
      drawCross(vg, Vec(padding + i * stepX, padding + crossLine2));
      drawCross(vg, Vec(padding + i * stepX, padding + crossLine3));
    }
    drawResolution(vg);
    drawSteps(vg);
  }
};
