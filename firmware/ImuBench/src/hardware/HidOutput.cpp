#include "HidOutput.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

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

void HidOutput::requestReleaseAll() {
  kbReleasePending_ = true;
  mouseReleasePending_ = true;
}

bool HidOutput::service(bool allowMotion, int8_t dx, int8_t dy, int8_t* sentDx,
                        int8_t* sentDy) {
  *sentDx = 0;
  *sentDy = 0;
  const bool isMounted = mounted();
  if (isMounted && !wasMounted_) {
    requestReleaseAll();  // (re)connect: neutral reports first
  }
  wasMounted_ = isMounted;
  if (!isMounted || !gHid.ready()) {
    return false;
  }
  // One report per loop: both report IDs share the IN endpoint.
  if (kbReleasePending_) {
    if (gHid.keyboardRelease(RID_KEYBOARD)) {
      kbReleasePending_ = false;
    } else {
      ++sendFailures_;
    }
    return false;
  }
  if (mouseReleasePending_) {
    if (gHid.mouseReport(RID_MOUSE, 0, 0, 0, 0, 0)) {
      mouseReleasePending_ = false;
    } else {
      ++sendFailures_;
    }
    return false;
  }
  if (!allowMotion || (dx == 0 && dy == 0)) {
    return false;
  }
  if (!gHid.mouseReport(RID_MOUSE, 0, dx, dy, 0, 0)) {
    ++sendFailures_;
    return false;
  }
  *sentDx = dx;
  *sentDy = dy;
  return true;
}
