#ifndef PHASEQ_H
#define PHASEQ_H
#include "Phaseque.hpp"
#endif

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../../ZZC/src/widgets.hpp"
#endif

struct PatternsDisplayWidget : BaseDisplayWidget {
  Pattern *patterns = nullptr;
  Pattern *currentPattern = nullptr;
  int *currentIdx = nullptr;
  NVGcolor black = nvgRGB(0x00, 0x00, 0x00);
  NVGcolor lcdActiveColor = nvgRGB(0xff, 0xd4, 0x2a);
  NVGcolor lcdDimmedColor = nvgRGB(0xa0, 0x80, 0x00);
  NVGcolor lcdDisabledColor = nvgRGB(0x36, 0x2b, 0x00);
  std::shared_ptr<Font> font;
  float patSize = 17.0f;
  float gapSize = 3.0f;
  float padding = 4.0f;
  int *goToRequest = nullptr;

  PatternsDisplayWidget() {
    font = APP->window->loadFont(asset::plugin(pluginInstance, "res/fonts/Nunito/Nunito-Bold.ttf"));
  }

  void drawPattern(const DrawArgs &args, bool hasCustomSteps, int idx, int currentPatternGoTo, int currentIdxVal) {
    float x = padding + ((idx - 1) % 4) * (patSize + gapSize) + (idx > 16 ? (patSize + gapSize) * 4 : 0.0f);
    float y = box.size.y - ((idx - 1) % 16) / 4 * (patSize + gapSize) - (patSize) - padding;
    bool isClean = !hasCustomSteps;
    if (idx == currentIdxVal) {
      nvgBeginPath(args.vg);
      nvgRoundedRect(args.vg, x, y, patSize, patSize, 1.0);
      nvgFillColor(args.vg, lcdActiveColor);
      nvgFill(args.vg);
    } else if (idx == currentPatternGoTo) {
      nvgBeginPath(args.vg);
      nvgRoundedRect(args.vg, x + 0.5, y + 0.5, patSize - 1, patSize - 1, 1.0);
      nvgFillColor(args.vg, lcdDisabledColor);
      nvgStrokeColor(args.vg, lcdActiveColor);
      if (!isClean) {
        nvgFill(args.vg);
      }
      nvgStroke(args.vg);
    } else if (!isClean) {
      nvgBeginPath(args.vg);
      nvgRoundedRect(args.vg, x, y, patSize, patSize, 1.0);
      nvgFillColor(args.vg, lcdDisabledColor);
      nvgFill(args.vg);
    }

    Vec textPos = Vec(x + patSize / 2.0f, y + patSize / 2.0f + 2.5f);
    if (idx == currentIdxVal) {
      nvgFillColor(args.vg, black);
    } else if (idx == currentPatternGoTo) {
      nvgFillColor(args.vg, lcdActiveColor);
    } else {
      nvgFillColor(args.vg, isClean ? lcdDimmedColor : lcdActiveColor);
    }
    char string[3];
    sprintf(string, "%d", idx);
    nvgText(args.vg, textPos.x, textPos.y, string, NULL);
  }

  void draw(const DrawArgs &args) override {
    drawBackground(args);
    nvgFontSize(args.vg, 9);
    nvgFontFaceId(args.vg, font->handle);
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER);
    nvgTextLetterSpacing(args.vg, -1.0);
    if (patterns && currentPattern && currentIdx) {
      int currentPatternGoTo = currentPattern->goTo;
      int currentIdxVal = currentIdx ? *currentIdx : 1;
      for (int i = 1; i < NUM_PATTERNS + 1; i++) {
        drawPattern(args, patterns[i].hasCustomSteps(), i, currentPatternGoTo, currentIdxVal);
      }
    } else {
      for (int i = 1; i < NUM_PATTERNS + 1; i++) {
        drawPattern(args, false, i, 2, 1);
      }
    }
  }

  void onButton(const event::Button &e) override {
    int button = e.button;
    if ((button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT)
        && e.action == GLFW_PRESS) {
      e.consume(this);
    } else {
      return;
    }
    float x = e.pos.x;
    float y = e.pos.y;
    float ix = clamp(floorf((x - padding) / (patSize + gapSize)), 0.0f, 7.0f);
    float iy = clamp(floorf((box.size.y - y - padding) / (patSize + gapSize)), 0.0f, 3.0f);
    float targetIdx = (fmodf(ix, 4.0f) + 1.0f) + (iy * 4.0f) + (ix >= 4.0f ? 16.0f : 0.0f);
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      *goToRequest = (int) targetIdx;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
      currentPattern->goTo = targetIdx;
    }
  }
};
