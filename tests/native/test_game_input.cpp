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
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  // Long puff with head motion: every mouse report keeps the right button.
  for (int i = 0; i < 30; ++i) {
    p = h.step(intent(false, false, true), true, 3, -2);
    assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff && p.dx == 3);
  }
  p = h.step(noGameIntent());
  assert(p.slot == HID_MOUSE && p.buttons == 0);

  // A one-loop puff still holds for the minimum time, then releases.
  p = h.step(intent(false, false, true));
  assert(p.buttons == mc::kButtonPuff);
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
    if (p.slot == HID_MOUSE) used = p.buttons == mc::kButtonPuff;
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

void setPuffMode(Host* h, uint8_t mode, uint32_t period = 600) {
  GameInputParams p = h->g.params();
  p.puffMode = mode; p.puffRepeatMs = period;
  h->g.setParams(p);
  h->settle(noGameIntent());
}

void test_puff_toggle_committed_and_cleared() {
  Host h;
  setPuffMode(&h, 1);
  HidPlan p = h.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  for (int i = 0; i < 30; ++i) {
    p = h.step(noGameIntent(), true, 2);
    assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  }
  p = h.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == 0);
  h.step(noGameIntent());
  p = h.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  p = h.step(intent(false, false, true), false);
  assert(p.slot == HID_MOUSE && p.buttons == 0);  // safety overrides min hold
  assert(h.step(intent(false, false, true)).slot == HID_NONE);
  h.step(noGameIntent());
  p = h.step(intent(false, false, true));
  assert(p.buttons == mc::kButtonPuff);
  h.g.releaseAll();
  h.settle(noGameIntent());
  assert(h.g.committedButtons() == 0);
  p = h.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);

  Host busy;
  setPuffMode(&busy, 1);
  busy.step(intent(false, false, true), true, 0, 0, false);
  busy.step(noGameIntent(), true, 0, 0, false);
  // Toggle off before the first press is accepted: still deliver a real click.
  p = busy.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  const uint64_t down = busy.t;
  uint64_t up = 0;
  for (int i = 0; i < 15; ++i) {
    p = busy.step(noGameIntent());
    if (p.slot == HID_MOUSE && p.buttons == 0) up = busy.t;
  }
  assert(up - down >= busy.g.params().minPressUs);
}

void test_puff_repeat_committed_period_and_release_order() {
  Host h;
  setPuffMode(&h, 2);
  uint64_t down = 0, lastDown = 0;
  int presses = 0, releases = 0;
  bool held = false;
  for (int i = 0; i < 160; ++i) {
    const HidPlan p = h.step(intent(false, false, true));
    if (p.slot != HID_MOUSE) continue;
    if (p.buttons) {
      assert(!held);
      if (lastDown) assert(h.t - lastDown == 600000);
      down = lastDown = h.t; held = true; ++presses;
    } else {
      assert(held && h.t - down >= h.g.params().minPressUs);
      held = false; ++releases;
    }
  }
  assert(presses == 3 && releases == 3);
  for (int i = 0; i < 100; ++i)
    assert(h.step(noGameIntent()).slot == HID_NONE);

  // prep < minPress: an accepted up report still separates every click.
  Host fast;
  setPuffMode(&fast, 2, 40);
  for (int i = 0; i < 5; ++i) {
    const HidPlan p = fast.step(intent(false, false, true), true, 0, 0, false);
    assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  }
  HidPlan p = fast.step(intent(false, false, true));
  assert(p.buttons == mc::kButtonPuff);
  down = fast.t;
  for (int i = 0; i < 5; ++i)
    assert(fast.step(intent(false, false, true)).slot == HID_NONE);
  // Release stays pending throughout a busy endpoint, never replaced by down.
  for (int i = 0; i < 10; ++i) {
    p = fast.step(intent(false, false, true), true, 0, 0, false);
    assert(p.slot == HID_MOUSE && p.buttons == 0);
  }
  p = fast.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == 0 && fast.t - down >= 60000);
  p = fast.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  p = fast.step(intent(false, false, true), false);
  assert(p.slot == HID_MOUSE && p.buttons == 0);
  for (int i = 0; i < 30; ++i)
    assert(fast.step(intent(false, false, true)).slot == HID_NONE);
  fast.step(noGameIntent());
  p = fast.step(intent(false, false, true));
  assert(p.slot == HID_MOUSE && p.buttons == mc::kButtonPuff);
  fast.g.releaseAll();
  fast.settle(intent(false, false, true));
  for (int i = 0; i < 30; ++i)
    assert(fast.step(intent(false, false, true)).slot == HID_NONE);
}

void test_sip_one_jump_per_press_and_bite_or() {
  Host h;
  h.settle(noGameIntent());
  GameIntent in = noGameIntent();
  in.sip = true;
  int presses = 0, releases = 0;
  for (int i = 0; i < 50; ++i) {
    HidPlan p = h.step(in);
    if (p.slot == HID_KEYBOARD) {
      if (hasKey(p.keys, mc::kKeyJump)) ++presses;
      else ++releases;
    }
  }
  assert(presses == 1 && releases == 1);
  h.step(noGameIntent());
  HidPlan p = h.step(in);
  assert(p.slot == HID_KEYBOARD && hasKey(p.keys, mc::kKeyJump));
  for (int i = 0; i < 20; ++i) h.step(noGameIntent());
  // Two independent rising edges, including a sip while bite remains held.
  in = noGameIntent(); in.jump = true;
  p = h.step(in);
  assert(hasKey(p.keys, mc::kKeyJump));
  in.sip = true;
  presses = 1; releases = 0;
  for (int i = 0; i < 50; ++i) {
    p = h.step(in);
    if (p.slot == HID_KEYBOARD) {
      if (hasKey(p.keys, mc::kKeyJump)) ++presses;
      else ++releases;
    }
  }
  assert(presses == 2 && releases == 2);
  h.step(noGameIntent());
  in = noGameIntent(); in.sip = in.jump = true;
  p = h.step(in);
  assert(hasKey(p.keys, mc::kKeyJump));  // simultaneous edges OR to one tap
  for (int i = 0; i < 50; ++i) {
    p = h.step(in);
    assert(p.slot != HID_KEYBOARD || !hasKey(p.keys, mc::kKeyJump));
  }
  h.step(in, false);
  assert(h.step(in).slot == HID_NONE);  // held sip cannot fire on re-enable
  GameInputParams params = h.g.params();
  params.sipJump = false; h.g.setParams(params);
  h.step(noGameIntent());
  in.jump = false;
  for (int i = 0; i < 10; ++i) assert(h.step(in).slot == HID_NONE);
  in.jump = true;
  assert(hasKey(h.step(in).keys, mc::kKeyJump));  // bite is still enabled
}

void test_puff_param_bounds() {
  GameInput g;
  GameInputParams p = g.params();
  p.puffMode = 255; p.puffRepeatMs = 0;
  g.setParams(p);
  assert(g.params().puffMode == 2 && g.params().puffRepeatMs == 40);
  p.puffRepeatMs = 5000;
  g.setParams(p);
  assert(g.params().puffRepeatMs == 2000);
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
  test_puff_toggle_committed_and_cleared();
  test_puff_repeat_committed_period_and_release_order();
  test_sip_one_jump_per_press_and_bite_or();
  test_puff_param_bounds();
}
