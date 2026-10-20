#include "GameInput.h"

#include <string.h>

GameIntent noGameIntent() {
  GameIntent g;
  g.forward = g.back = g.left = g.right = false;
  g.jump = g.use = false;
  return g;
}

GameInputParams defaultGameInputParams() {
  GameInputParams p;
  // Longer than one 50 ms game tick plus a frame, so a tap is never missed.
  // Provisional (docs/imu-decisions.md D16); tune in game.
  p.minPressUs = 60000;
  return p;
}

GameInput::GameInput() : params_(defaultGameInputParams()) {
  memset(sentKeys_, 0, sizeof(sentKeys_));
  sentButtons_ = 0;
  droppedEdges_ = 0;
  releaseAll();
}

void GameInput::releaseAll() {
  move_ = noGameIntent();
  Pulse idle;
  idle.state = P_IDLE;
  idle.queued = false;
  idle.prevLevel = true;  // a held bite/puff must be released before it counts
  idle.downUs = 0;
  jump_ = idle;
  use_ = idle;
  useLevel_ = false;
  forceKeyboard_ = true;
  forceMouse_ = true;
  lastSlot_ = HID_MOUSE;  // keyboard release goes first
}

void GameInput::edge(Pulse* p) {
  if (p->state == P_IDLE) {
    p->state = P_WANT_DOWN;
  } else if (!p->queued) {
    p->queued = true;
  } else {
    ++droppedEdges_;
  }
}

void GameInput::cancel(Pulse* p) {
  p->queued = false;
  p->prevLevel = true;
  if (p->state == P_WANT_DOWN) p->state = P_IDLE;  // never reached the host
  if (p->state == P_DOWN) p->state = P_WANT_UP;    // release now, no min hold
}

void GameInput::update(uint64_t nowUs, const GameIntent& in, bool allowActions) {
  if (!allowActions) {
    move_ = noGameIntent();
    useLevel_ = false;
    cancel(&jump_);
    cancel(&use_);
    return;
  }
  move_ = in;
  move_.jump = move_.use = false;

  if (in.jump && !jump_.prevLevel) edge(&jump_);
  jump_.prevLevel = in.jump;
  if (in.use && !use_.prevLevel) edge(&use_);
  use_.prevLevel = in.use;
  useLevel_ = in.use;

  // Space is a tap: up after the minimum hold even if the bite continues.
  if (jump_.state == P_DOWN && nowUs - jump_.downUs >= params_.minPressUs) {
    jump_.state = P_WANT_UP;
  }
  // Left button follows the puff, but never shorter than the minimum hold.
  if (use_.state == P_DOWN && !useLevel_ &&
      nowUs - use_.downUs >= params_.minPressUs) {
    use_.state = P_WANT_UP;
  }
}

void GameInput::desiredKeys(uint8_t keys[6]) const {
  memset(keys, 0, 6);
  int n = 0;
  if (move_.forward && !move_.back) keys[n++] = mc::kKeyForward;
  if (move_.back && !move_.forward) keys[n++] = mc::kKeyBack;
  if (move_.left && !move_.right) keys[n++] = mc::kKeyLeft;
  if (move_.right && !move_.left) keys[n++] = mc::kKeyRight;
  if (wantsDown(jump_)) keys[n++] = mc::kKeyJump;
}

uint8_t GameInput::desiredButtons() const {
  return wantsDown(use_) ? mc::kButtonPuff : 0;
}

bool GameInput::keyboardDirty() const {
  uint8_t keys[6];
  desiredKeys(keys);
  return forceKeyboard_ || memcmp(keys, sentKeys_, sizeof(keys)) != 0;
}

bool GameInput::mouseButtonsDirty() const {
  return forceMouse_ || desiredButtons() != sentButtons_;
}

HidPlan GameInput::plan(int8_t dx, int8_t dy) const {
  HidPlan p;
  memset(&p, 0, sizeof(p));
  const bool kb = keyboardDirty();
  const bool mouse = mouseButtonsDirty() || dx != 0 || dy != 0;
  if (kb && mouse) {
    p.slot = (lastSlot_ == HID_KEYBOARD) ? HID_MOUSE : HID_KEYBOARD;
  } else if (kb) {
    p.slot = HID_KEYBOARD;
  } else if (mouse) {
    p.slot = HID_MOUSE;
  } else {
    p.slot = HID_NONE;
  }
  if (p.slot == HID_KEYBOARD) {
    desiredKeys(p.keys);
  } else if (p.slot == HID_MOUSE) {
    p.buttons = desiredButtons();
    p.dx = dx;
    p.dy = dy;
  }
  return p;
}

void GameInput::onCommit(Pulse* p, bool reportHasIt, uint64_t nowUs) {
  if (p->state == P_WANT_DOWN && reportHasIt) {
    p->state = P_DOWN;
    p->downUs = nowUs;
  } else if (p->state == P_WANT_UP && !reportHasIt) {
    p->state = P_IDLE;
    if (p->queued) {
      p->queued = false;
      p->state = P_WANT_DOWN;
    }
  }
}

void GameInput::committed(const HidPlan& p, uint64_t nowUs) {
  if (p.slot == HID_KEYBOARD) {
    memcpy(sentKeys_, p.keys, sizeof(sentKeys_));
    forceKeyboard_ = false;
    bool hasJump = false;
    for (int i = 0; i < 6; ++i) hasJump = hasJump || p.keys[i] == mc::kKeyJump;
    onCommit(&jump_, hasJump, nowUs);
  } else if (p.slot == HID_MOUSE) {
    sentButtons_ = p.buttons;
    forceMouse_ = false;
    onCommit(&use_, (p.buttons & mc::kButtonPuff) != 0, nowUs);
  } else {
    return;
  }
  lastSlot_ = p.slot;
}
