#include <bitset>

#ifndef PHASEQ_H
#define PHASEQ_H
#include "Phaseque.hpp"
#endif

#ifndef WIDGETS_H
#define WIDGETS_H
#include "../ZZC/src/widgets.hpp"
#endif

struct PatternsDisplayConsumer {
  unsigned int currentPattern = 0;
  unsigned int currentPatternGoTo = 1;
  std::bitset<NUM_PATTERNS> dirtyMask;
  bool consumed = false;
};

struct PatternsDisplayProducer {
  unsigned int goToRequest = 0;
  bool hasGoToRequest = false;

  unsigned int nextPatternRequest = 0;
  bool hasNextPatternRequest = false;
};

struct PatternsDisplay : BaseDisplayWidget {
  NVGcolor black = nvgRGB(0x00, 0x00, 0x00);
  NVGcolor white = nvgRGB(0xff, 0xff, 0xff);
  NVGcolor lcdActiveColor = nvgRGB(0xff, 0xd4, 0x2a);
  NVGcolor lcdDimmedColor = nvgRGB(0xa0, 0x80, 0x00);
  NVGcolor lcdDisabledColor = nvgRGB(0x36, 0x2b, 0x00);
  NVGcolor negColor = nvgRGB(0xe7, 0x34, 0x2d);
  NVGcolor posColor = nvgRGB(0x9c, 0xd7, 0x43);
  std::shared_ptr<Font> font;
  float patSize = 17.0f;
  float gapSize = 3.0f;
  float padding = 4.0f;
  float flashes[NUM_PATTERNS] = { 0.f };

  std::shared_ptr<PatternsDisplayConsumer> consumer;

  PatternsDisplay() {
    font = APP->window->loadFont(asset::plugin(pluginInstance, "res/fonts/Nunito/Nunito-Bold.ttf"));
  }

  void drawPattern(const DrawArgs &args, bool hasCustomSteps, int idx, int currentPatternGoTo, int currentIdxVal) {
    float x = padding + ((idx) % 4) * (patSize + gapSize) + (idx > 15 ? (patSize + gapSize) * 4 : 0.0f);
    float y = box.size.y - ((idx) % 16) / 4 * (patSize + gapSize) - (patSize) - padding;
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
    std::string humanString = std::to_string(idx + 1);
    nvgText(args.vg, textPos.x, textPos.y, humanString.c_str(), NULL);
  }

  void draw(const DrawArgs &args) override {
    this->drawBackground(args);
    nvgFontSize(args.vg, 9);
    nvgFontFaceId(args.vg, font->handle);
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER);
    nvgTextLetterSpacing(args.vg, -1.0);
    for (unsigned int i = 0; i < NUM_PATTERNS; i++) {
      drawPattern(args, this->consumer->dirtyMask.test(i), i, this->consumer->currentPatternGoTo, this->consumer->currentPattern);
    }
  }
};

struct PatternsDisplayWidget : widget::OpaqueWidget {
  float patSize = 17.0f;
  float gapSize = 3.0f;
  float padding = 4.0f;

  std::shared_ptr<PatternsDisplayConsumer> consumer;
  std::shared_ptr<PatternsDisplayProducer> producer;

	widget::FramebufferWidget* fb;
  PatternsDisplay* pd;

  PatternsDisplayWidget() {
    consumer = std::make_shared<PatternsDisplayConsumer>();
    producer = std::make_shared<PatternsDisplayProducer>();

    this->fb = new widget::FramebufferWidget;
    this->addChild(this->fb);

    this->pd = new PatternsDisplay;
    this->pd->consumer = this->consumer;
    this->fb->addChild(this->pd);
  }

  void setupSize(Vec size) {
    this->pd->setSize(size);
    this->fb->setSize(size);
    this->setSize(size);
  }

  void step() override {
    if (!this->consumer->consumed) {
      this->fb->dirty = true;
    }
    Widget::step();
    this->consumer->consumed = true;
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
    float targetIdx = fmodf(ix, 4.0f) + (iy * 4.0f) + (ix >= 4.0f ? 16.0f : 0.0f);
    unsigned int targetIdxInt = clamp((unsigned int) targetIdx, 0, NUM_PATTERNS - 1);
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      if (this->producer->hasGoToRequest) { return; }
      this->producer->goToRequest = targetIdxInt;
      this->producer->hasGoToRequest = true;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
      if (this->producer->hasNextPatternRequest) { return; }
      this->producer->nextPatternRequest = targetIdxInt;
      this->producer->hasNextPatternRequest = true;
    }
  }
};
