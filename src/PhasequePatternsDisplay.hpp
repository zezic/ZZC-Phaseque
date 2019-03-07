#ifndef PHASEQ_H
#define PHASEQ_H
#include "Phaseque.hpp"
#endif

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../../ZZC/src/widgets.hpp"
#endif

struct PatternsDisplayWidget : BaseDisplayWidget {
  Pattern *patterns;
  Pattern *currentPattern;
  int *currentIdx;
  NVGcolor black = nvgRGB(0x00, 0x00, 0x00);
  NVGcolor lcdActiveColor = nvgRGB(0xff, 0xd4, 0x2a);
  NVGcolor lcdDimmedColor = nvgRGB(0xa0, 0x80, 0x00);
  NVGcolor lcdDisabledColor = nvgRGB(0x36, 0x2b, 0x00);
  std::shared_ptr<Font> font;
  float patSize = 17.0f;
  float gapSize = 3.0f;
  float padding = 4.0f;
  int *goToRequest;

  PatternsDisplayWidget() {
    font = Font::load(assetPlugin(plugin, "res/fonts/Nunito/Nunito-Bold.ttf"));
  }

  void drawPattern(NVGcontext *vg, Pattern pattern, int idx) {
    float x = padding + ((idx - 1) % 4) * (patSize + gapSize) + (idx > 16 ? (patSize + gapSize) * 4 : 0.0f);
    float y = box.size.y - ((idx - 1) % 16) / 4 * (patSize + gapSize) - (patSize) - padding;
    bool isClean = !pattern.hasCustomSteps();
    if (idx == *currentIdx) {
      nvgBeginPath(vg);
      nvgRoundedRect(vg, x, y, patSize, patSize, 1.0);
      nvgFillColor(vg, lcdActiveColor);
      nvgFill(vg);
    } else if (idx == currentPattern->goTo) {
      nvgBeginPath(vg);
      nvgRoundedRect(vg, x + 0.5, y + 0.5, patSize - 1, patSize - 1, 1.0);
      nvgFillColor(vg, lcdDisabledColor);
      nvgStrokeColor(vg, lcdActiveColor);
      if (!isClean) {
        nvgFill(vg);
      }
      nvgStroke(vg);
    } else if (!isClean) {
      nvgBeginPath(vg);
      nvgRoundedRect(vg, x, y, patSize, patSize, 1.0);
      nvgFillColor(vg, lcdDisabledColor);
      nvgFill(vg);
    }

    Vec textPos = Vec(x + patSize / 2.0f, y + patSize / 2.0f + 2.5f);
    if (idx == *currentIdx) {
      nvgFillColor(vg, black);
    } else if (idx == currentPattern->goTo) {
      nvgFillColor(vg, lcdActiveColor);
    } else {
      nvgFillColor(vg, isClean ? lcdDimmedColor : lcdActiveColor);
    }
    char string[3];
    sprintf(string, "%d", idx);
    nvgText(vg, textPos.x, textPos.y, string, NULL);
  }

  void draw(NVGcontext *vg) override {
    drawBackground(vg);
    nvgFontSize(vg, 9);
    nvgFontFaceId(vg, font->handle);
    nvgTextAlign(vg, NVG_ALIGN_CENTER);
    nvgTextLetterSpacing(vg, -1.0);
    for (int i = 1; i < NUM_PATTERNS + 1; i++) {
      drawPattern(vg, patterns[i], i);
    }
  }

  void onMouseDown(EventMouseDown &e) override {
    e.consumed = true;
    float x = e.pos.x;
    float y = e.pos.y;
    int button = e.button;
    float ix = clamp(floorf((x - padding) / (patSize + gapSize)), 0.0f, 7.0f);
    float iy = clamp(floorf((box.size.y - y - padding) / (patSize + gapSize)), 0.0f, 3.0f);
    float targetIdx = (fmodf(ix, 4.0f) + 1.0f) + (iy * 4.0f) + (ix >= 4.0f ? 16.0f : 0.0f);
    if (button == 0) {
      *goToRequest = (int) targetIdx;
    } else if (button == 1) {
      currentPattern->goTo = targetIdx;
    }
  }
};
