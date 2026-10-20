// USB composite keyboard + mouse (Adafruit TinyUSB, report IDs from the
// official hid_composite example). The descriptor matches the final MouthMod
// device so the host sees one layout.
//
// This class only puts reports on the wire. What to send (complete keyboard
// state, buttons on every mouse report, one report per loop) is decided by
// logic/GameInput. Its own release-all covers (re)enumeration and turning
// HID output off: neutral reports stay pending until each is accepted.
#ifndef MOUTHMOD_HID_OUTPUT_H
#define MOUTHMOD_HID_OUTPUT_H

#include <stdint.h>

class HidOutput {
 public:
  HidOutput();

  void begin();  // call early in setup(), before Serial.begin()

  bool mounted() const;
  bool ready() const;  // mounted and the shared IN endpoint is free

  // True once after each (re)enumeration or resume; host key state unknown.
  bool remounted();

  void requestReleaseAll();
  bool releasePending() const { return kbReleasePending_ || mouseReleasePending_; }

  // Sends one pending neutral report. Returns true if this loop's report
  // slot is used up (sent, or pending but the endpoint is not ready).
  bool flushRelease();

  // Each returns true only if the endpoint accepted the report.
  bool sendKeyboard(uint8_t modifier, const uint8_t keys[6]);
  bool sendMouse(uint8_t buttons, int8_t dx, int8_t dy);

  uint32_t sendFailures() const { return sendFailures_; }

 private:
  bool kbReleasePending_;
  bool mouseReleasePending_;
  bool wasMounted_;
  uint32_t sendFailures_;
};

#endif
