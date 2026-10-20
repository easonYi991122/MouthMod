// Minecraft Education adapter: channel levels -> USB keyboard + mouse reports.
//
// Frozen mapping (guide §b/§c, [E] §4–5): joystick -> W/A/S/D held; bite ->
// one Space tap per rising edge; puff -> left button, held while puffing
// (attack / mine; press and release both delivered; D29); head tilt -> mouse motion. Sip/D6 disarm is the caller's job:
// it calls releaseAll() and stops passing allowActions.
//
// Delivery rules from the HID section of the guide:
// - one full desired keyboard state, so a jump never releases W;
// - every mouse report carries the current buttons, so motion never drops a
//   held left button;
// - a press is only "done" once its report was accepted, and it stays down
//   for at least minPressUs before its release (the game samples input per
//   frame/tick, so a 2 ms tap can be missed);
// - keyboard and mouse share one endpoint: one report per loop, alternating
//   when both have something to send.
//
// Pure C++: no Arduino headers. The caller passes time and acknowledges each
// report the endpoint accepted via committed().
#ifndef MOUTHMOD_GAME_INPUT_H
#define MOUTHMOD_GAME_INPUT_H

#include <stdint.h>

namespace mc {
// USB HID keyboard usage IDs (page 0x07) for the Minecraft Education default
// key bindings. Change here only if the in-game bindings are changed.
const uint8_t kKeyForward = 0x1A;  // W
const uint8_t kKeyLeft = 0x04;     // A
const uint8_t kKeyBack = 0x16;     // S
const uint8_t kKeyRight = 0x07;    // D
const uint8_t kKeyJump = 0x2C;     // Space
const uint8_t kButtonPuff = 0x01;  // left mouse button (D29)
}  // namespace mc

// Levels from the channel mappers, already filtered and hysteresis-confirmed.
struct GameIntent {
  bool forward;
  bool back;
  bool left;
  bool right;
  bool jump;  // bite active: one Space tap per rising edge, never repeats
  bool use;   // puff active: left button held while active
};

GameIntent noGameIntent();

struct GameInputParams {
  uint32_t minPressUs;  // shortest Space / left-button hold the host sees
};

GameInputParams defaultGameInputParams();

enum HidSlot { HID_NONE = 0, HID_KEYBOARD, HID_MOUSE };

struct HidPlan {
  HidSlot slot;
  uint8_t modifier;
  uint8_t keys[6];
  uint8_t buttons;
  int8_t dx;
  int8_t dy;
};

class GameInput {
 public:
  GameInput();

  void setParams(const GameInputParams& p) { params_ = p; }
  const GameInputParams& params() const { return params_; }

  // Clears every action and forces one neutral keyboard and mouse report
  // (host state unknown). Edges re-arm only after the input is seen released.
  void releaseAll();

  // Once per loop. allowActions = state is ACTIVE and output is permitted;
  // when false every action is released (Space/right go up at once).
  void update(uint64_t nowUs, const GameIntent& in, bool allowActions);

  // The one report to send this loop; dx/dy are the look deltas available.
  // A mouse plan carries the motion; a keyboard plan carries none.
  HidPlan plan(int8_t dx, int8_t dy) const;

  // Call only after the endpoint accepted the planned report.
  void committed(const HidPlan& p, uint64_t nowUs);

  // Committed (host-side) state, for logging.
  uint8_t committedButtons() const { return sentButtons_; }
  const uint8_t* committedKeys() const { return sentKeys_; }
  bool keyboardDirty() const;
  bool mouseButtonsDirty() const;
  uint32_t droppedEdges() const { return droppedEdges_; }

 private:
  enum PulseState { P_IDLE = 0, P_WANT_DOWN, P_DOWN, P_WANT_UP };
  struct Pulse {
    PulseState state;
    bool queued;      // one more edge arrived while busy
    bool prevLevel;   // last seen input level (true = must see release first)
    uint64_t downUs;  // when the press was accepted by the host
  };

  void edge(Pulse* p);
  void cancel(Pulse* p);
  void desiredKeys(uint8_t keys[6]) const;
  uint8_t desiredButtons() const;
  static bool wantsDown(const Pulse& p) {
    return p.state == P_WANT_DOWN || p.state == P_DOWN;
  }
  static void onCommit(Pulse* p, bool reportHasIt, uint64_t nowUs);

  GameInputParams params_;
  GameIntent move_;
  Pulse jump_;
  Pulse use_;
  bool useLevel_;
  uint8_t sentKeys_[6];
  uint8_t sentButtons_;
  bool forceKeyboard_;
  bool forceMouse_;
  HidSlot lastSlot_;
  uint32_t droppedEdges_;
};

#endif
