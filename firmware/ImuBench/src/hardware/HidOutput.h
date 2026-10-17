// USB composite keyboard + mouse (Adafruit TinyUSB, report IDs from the
// official hid_composite example). The bench only moves the mouse, but the
// descriptor matches the final MouthMod device so the host sees one layout.
//
// Release-all has priority: keyboard release and a zero-button mouse report
// stay pending until each is actually accepted by the endpoint.
#ifndef MOUTHMOD_HID_OUTPUT_H
#define MOUTHMOD_HID_OUTPUT_H

#include <stdint.h>

class HidOutput {
 public:
  HidOutput();

  void begin();  // call early in setup(), before Serial.begin()

  bool mounted() const;
  void requestReleaseAll();
  bool releasePending() const { return kbReleasePending_ || mouseReleasePending_; }

  // Called once per loop. Sends pending releases first; otherwise, if
  // allowMotion, one relative mouse report. Returns true if (dx, dy) was
  // accepted; *sent is then the committed delta (else 0,0).
  bool service(bool allowMotion, int8_t dx, int8_t dy, int8_t* sentDx,
               int8_t* sentDy);

  uint32_t sendFailures() const { return sendFailures_; }

 private:
  bool kbReleasePending_;
  bool mouseReleasePending_;
  bool wasMounted_;
  uint32_t sendFailures_;
};

#endif
