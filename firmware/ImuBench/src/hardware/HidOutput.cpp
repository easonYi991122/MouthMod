#include "HidOutput.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

namespace {
enum { RID_KEYBOARD = 1, RID_MOUSE = 2 };

uint8_t const kHidReportDesc[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(RID_MOUSE))};

Adafruit_USBD_HID gHid;
}  // namespace

HidOutput::HidOutput()
    : kbReleasePending_(true),
      mouseReleasePending_(true),
      wasMounted_(false),
      sendFailures_(0) {}

void HidOutput::begin() {
  gHid.setPollInterval(2);
  gHid.setReportDescriptor(kHidReportDesc, sizeof(kHidReportDesc));
  gHid.begin();
}

bool HidOutput::mounted() const {
  return TinyUSBDevice.mounted() && !TinyUSBDevice.suspended();
}

bool HidOutput::ready() const { return mounted() && gHid.ready(); }

bool HidOutput::remounted() {
  const bool isMounted = mounted();
  const bool rising = isMounted && !wasMounted_;
  wasMounted_ = isMounted;
  if (rising) requestReleaseAll();  // (re)connect: neutral reports first
  return rising;
}

void HidOutput::requestReleaseAll() {
  kbReleasePending_ = true;
  mouseReleasePending_ = true;
}

bool HidOutput::flushRelease() {
  if (!releasePending()) return false;
  if (!ready()) return true;
  // One report per loop: both report IDs share the IN endpoint.
  if (kbReleasePending_) {
    if (gHid.keyboardRelease(RID_KEYBOARD)) {
      kbReleasePending_ = false;
    } else {
      ++sendFailures_;
    }
    return true;
  }
  if (gHid.mouseReport(RID_MOUSE, 0, 0, 0, 0, 0)) {
    mouseReleasePending_ = false;
  } else {
    ++sendFailures_;
  }
  return true;
}

bool HidOutput::sendKeyboard(uint8_t modifier, const uint8_t keys[6]) {
  if (!ready()) return false;
  uint8_t k[6];
  memcpy(k, keys, sizeof(k));
  if (!gHid.keyboardReport(RID_KEYBOARD, modifier, k)) {
    ++sendFailures_;
    return false;
  }
  return true;
}

bool HidOutput::sendMouse(uint8_t buttons, int8_t dx, int8_t dy) {
  if (!ready()) return false;
  if (!gHid.mouseReport(RID_MOUSE, buttons, dx, dy, 0, 0)) {
    ++sendFailures_;
    return false;
  }
  return true;
}
