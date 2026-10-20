// Host tests for the Minecraft Education keyboard + mouse adapter.
#include <cassert>
#include <cstring>

#include "GameInput.h"

namespace {

const uint64_t kTick = 10000;  // 100 Hz loop

bool hasKey(const uint8_t* keys, uint8_t k) {
  for (int i = 0; i < 6; ++i) {
    if (keys[i] == k) return true;
  }
  return false;
}

// One loop: update, plan, and (if the endpoint accepts) commit.
struct Host {
  GameInput g;
  uint64_t t;
  Host() : t(0) {}
  HidPlan step(const GameIntent& in, bool allow = true, int8_t dx = 0,
               int8_t dy = 0, bool accept = true) {
    t += kTick;
    g.update(t, in, allow);
    const HidPlan p = g.plan(dx, dy);
    if (accept) g.committed(p, t);
    return p;
  }
  // Runs until nothing is pending; returns the number of loops used.
  int settle(const GameIntent& in, bool allow = true) {
    int n = 0;
    while (step(in, allow).slot != HID_NONE) {
      assert(++n < 100);
    }
    return n;
  }
};

GameIntent intent(bool w, bool jump, bool use) {
  GameIntent i = noGameIntent();
  i.forward = w;
  i.jump = jump;
  i.use = use;
  return i;
}

void test_powerup_sends_neutral_then_idles() {
  Host h;
  HidPlan p = h.step(noGameIntent());
  assert(p.slot == HID_KEYBOARD && p.keys[0] == 0);
  p = h.step(noGameIntent());
  assert(p.slot == HID_MOUSE && p.buttons == 0 && p.dx == 0);
  assert(h.step(noGameIntent()).slot == HID_NONE);
}

void test_jump_is_a_tap_and_never_releases_w() {
  Host h;
  h.settle(noGameIntent());
  HidPlan p = h.step(intent(true, false, false));
  assert(p.slot == HID_KEYBOARD && hasKey(p.keys, mc::kKeyForward));

  // Bite and keep biting: one Space tap, held >= minPress, W never dropped.
  uint64_t downAt = 0, upAt = 0;
  for (int i = 0; i < 40; ++i) {
    p = h.step(intent(true, true, false));
    if (p.slot != HID_KEYBOARD) continue;
    assert(hasKey(p.keys, mc::kKeyForward));
    if (hasKey(p.keys, mc::kKeyJump)) {
      assert(downAt == 0);
      downAt = h.t;
    } else if (downAt && !upAt) {
      upAt = h.t;
    } else {
      assert(false);  // no repeat while the bite is held
    }
  }
  assert(downAt && upAt);
  assert(upAt - downAt >= h.g.params().minPressUs);
  assert(upAt - downAt <= h.g.params().minPressUs + kTick);

  // Release the bite, bite again: a second tap.
  h.step(intent(true, false, false));
  p = h.step(intent(true, true, false));
  assert(p.slot == HID_KEYBOARD && hasKey(p.keys, mc::kKeyJump));
}

void test_one_loop_bite_still_taps() {
  Host h;
  h.settle(noGameIntent());
  HidPlan p = h.step(intent(false, true, false));
  assert(hasKey(p.keys, mc::kKeyJump));
  const uint64_t down = h.t;
  uint64_t up = 0;
  for (int i = 0; i < 20 && !up; ++i) {
    p = h.step(noGameIntent());
    if (p.slot == HID_KEYBOARD && !hasKey(p.keys, mc::kKeyJump)) up = h.t;
  }
  assert(up - down >= h.g.params().minPressUs);
}

void test_use_follows_puff_and_motion_keeps_button() {
  Host h;
  h.settle(noGameIntent());
  HidPlan p = h.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonUse);
  // Long puff with head motion: every mouse report keeps the right button.
  for (int i = 0; i < 30; ++i) {
    p = h.step(intent(false, false, true), true, 3, -2);
    assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonUse && p.dx == 3);
  }
  p = h.step(noGameIntent());
  assert(p.slot == HID_MOUSE && p.buttons == 0);

  // A one-loop puff still holds for the minimum time, then releases.
  p = h.step(intent(false, false, true));
  assert(p.buttons == mc::kButtonUse);
  const uint64_t down = h.t;
  uint64_t up = 0;
  for (int i = 0; i < 20 && !up; ++i) {
    p = h.step(noGameIntent());
    if (p.slot == HID_MOUSE && p.buttons == 0) up = h.t;
  }
  assert(up - down >= h.g.params().minPressUs);
}

void test_busy_endpoint_never_loses_a_press() {
  Host h;
  h.settle(noGameIntent());
  // Bite for one loop while the endpoint is busy for five loops.
  h.step(intent(false, true, false), true, 0, 0, false);
  for (int i = 0; i < 4; ++i) h.step(noGameIntent(), true, 0, 0, false);
  HidPlan p = h.step(noGameIntent());
  assert(p.slot == HID_KEYBOARD && hasKey(p.keys, mc::kKeyJump));
  const uint64_t down = h.t;
  // Release is not sent before the minimum hold measured from acceptance.
  p = h.step(noGameIntent());
  assert(p.slot == HID_NONE);
  uint64_t up = 0;
  for (int i = 0; i < 20 && !up; ++i) {
    p = h.step(noGameIntent());
    if (p.slot == HID_KEYBOARD) up = h.t;
  }
  assert(up - down >= h.g.params().minPressUs);
}

void test_keyboard_and_mouse_alternate() {
  Host h;
  h.settle(noGameIntent());
  // Keyboard changes on every loop it gets (worst case) while the head keeps
  // turning: the two strictly alternate, so the camera is never starved.
  bool left = true;
  int kb = 0, mouse = 0;
  HidSlot last = HID_NONE;
  for (int i = 0; i < 20; ++i) {
    GameIntent in = noGameIntent();
    in.left = left;
    in.right = !left;
    const HidPlan p = h.step(in, true, 2, 0);
    assert(p.slot != HID_NONE && p.slot != last);
    last = p.slot;
    if (p.slot == HID_KEYBOARD) {
      ++kb;
      left = !left;
    } else {
      ++mouse;
      assert(p.dx == 2);
    }
  }
  assert(kb == 10 && mouse == 10);
}

void test_opposite_directions_cancel() {
  Host h;
  h.settle(noGameIntent());
  GameIntent in = noGameIntent();
  in.forward = in.back = true;
  in.left = true;
  const HidPlan p = h.step(in);
  assert(p.slot == HID_KEYBOARD);
  assert(hasKey(p.keys, mc::kKeyLeft));
  assert(!hasKey(p.keys, mc::kKeyForward) && !hasKey(p.keys, mc::kKeyBack));
}

void test_disarm_releases_at_once_and_needs_release_to_retrigger() {
  Host h;
  h.settle(noGameIntent());
  h.step(intent(true, true, true));  // keyboard: W + Space
  h.step(intent(true, true, true));  // mouse: right
  // Disarm inside the minimum hold: everything goes up now.
  HidPlan p = h.step(intent(true, true, true), false);
  assert(p.slot == HID_KEYBOARD && p.keys[0] == 0);
  p = h.step(intent(true, true, true), false);
  assert(p.slot == HID_MOUSE && p.buttons == 0);
  assert(h.step(intent(true, true, true), false).slot == HID_NONE);

  // Re-armed with bite and puff still held: only W comes back.
  p = h.step(intent(true, true, true));
  assert(p.slot == HID_KEYBOARD && hasKey(p.keys, mc::kKeyForward) &&
         !hasKey(p.keys, mc::kKeyJump));
  assert(h.step(intent(true, true, true)).slot == HID_NONE);
  // Release, then press again: both fire.
  h.step(intent(true, false, false));
  bool jumped = false, used = false;
  for (int i = 0; i < 2; ++i) {
    p = h.step(intent(true, true, true));
    if (p.slot == HID_KEYBOARD) jumped = hasKey(p.keys, mc::kKeyJump);
    if (p.slot == HID_MOUSE) used = p.buttons == mc::kButtonUse;
  }
  assert(jumped && used);
}

void test_release_all_forces_neutral_reports() {
  Host h;
  h.settle(noGameIntent());
  h.step(intent(true, false, false));
  h.g.releaseAll();
  HidPlan p = h.g.plan(5, 5);
  assert(p.slot == HID_KEYBOARD && p.keys[0] == 0);
  h.g.committed(p, h.t);
  p = h.g.plan(0, 0);
  assert(p.slot == HID_MOUSE && p.buttons == 0);
  h.g.committed(p, h.t);
  assert(h.g.plan(0, 0).slot == HID_NONE);
}

}  // namespace

void run_game_input_tests() {
  test_powerup_sends_neutral_then_idles();
  test_jump_is_a_tap_and_never_releases_w();
  test_one_loop_bite_still_taps();
  test_use_follows_puff_and_motion_keeps_button();
  test_busy_endpoint_never_loses_a_press();
  test_keyboard_and_mouse_alternate();
  test_opposite_directions_cancel();
  test_disarm_releases_at_once_and_needs_release_to_retrigger();
  test_release_all_forces_neutral_reports();
}
