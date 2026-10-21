#!/usr/bin/env bash
# Builds "build/MouthMod Float.app", the floating debug window (option B).
# Needs only the Command Line Tools (swiftc). The app carries its own copy of
# tools/look_demo, so rebuild it after changing bridge.py or the pages. It can
# be moved anywhere, e.g. /Applications.
set -euo pipefail
cd "$(dirname "$0")/.."
APP="build/MouthMod Float.app"
SDK=$(xcrun --sdk macosx --show-sdk-path)
TMP=build/float_app
rm -rf "$APP" "$TMP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources/look_demo" "$TMP"

# Universal binary, so a copy also runs on Intel Macs.
for arch in arm64 x86_64; do
  xcrun swiftc -O -sdk "$SDK" -target "$arch-apple-macos12.0" \
    tools/float_app/MouthModFloat.swift -o "$TMP/MouthModFloat-$arch"
done
lipo -create "$TMP"/MouthModFloat-* -output "$APP/Contents/MacOS/MouthModFloat"
cp tools/look_demo/bridge.py tools/look_demo/float.html tools/look_demo/index.html \
  "$APP/Contents/Resources/look_demo/"
VERSION=$(cat VERSION)
cat > "$APP/Contents/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleIdentifier</key><string>edu.jhu.pdbmi.mouthmod.float</string>
  <key>CFBundleName</key><string>MouthMod Float</string>
  <key>CFBundleDisplayName</key><string>MouthMod 悬浮窗</string>
  <key>CFBundleExecutable</key><string>MouthModFloat</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>$VERSION</string>
  <key>CFBundleVersion</key><string>$VERSION</string>
  <key>LSMinimumSystemVersion</key><string>12.0</string>
  <key>LSUIElement</key><true/>
  <key>NSAppTransportSecurity</key><dict><key>NSAllowsLocalNetworking</key><true/></dict>
</dict></plist>
EOF
codesign --force --sign - "$APP" >/dev/null 2>&1  # ad-hoc; required on Apple Silicon
rm -rf "$TMP"
echo "OK $APP ($(du -sh "$APP" | cut -f1))"
