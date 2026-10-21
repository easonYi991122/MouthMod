// MouthMod floating window (option B): a small always-on-top macOS panel
// that starts bridge.py and shows float.html, without ever taking focus
// from Minecraft. Build with scripts/build-float-app.sh (Command Line Tools
// only). The browser version (bridge.py + float.html in Chrome) stays as the
// backup and uses the same page.
//
// - The panel is non-activating: clicks never make this app (or Chrome)
//   the front app, so the bridge's focus gate and Minecraft's own focus are
//   untouched. It floats above full-screen apps on every Space.
// - The keyboard stays with Minecraft. The page asks for it ("keyboard")
//   only while a text/number field is focused and hands it back ("release").
// - bridge.py runs as a child on a free port and stops with the app; on stop
//   it disarms the board. --exit-with-parent covers a crashed app.
import AppKit
import WebKit

final class FloatPanel: NSPanel {
  var allowKey = false
  override var canBecomeKey: Bool { allowKey }
  override var canBecomeMain: Bool { false }
}

final class FloatWebView: WKWebView {
  override func acceptsFirstMouse(for event: NSEvent?) -> Bool { true }
  override var needsPanelToBecomeKey: Bool { false }
}

// bridge.py as a child process. Reports the HTTP port once it is serving.
final class BridgeProcess {
  private let process = Process()
  private var output = ""
  var onReady: ((Int) -> Void)?
  var onExit: ((String) -> Void)?

  init(script: URL) {
    process.executableURL = URL(fileURLWithPath: "/usr/bin/python3")
    // MOUTHMOD_BRIDGE_ARGS adds bridge options, e.g. "--serial /dev/cu.usbmodem1101".
    let extra = ProcessInfo.processInfo.environment["MOUTHMOD_BRIDGE_ARGS"]?
      .split(separator: " ").map(String.init) ?? []
    process.arguments = ["-I", "-u", script.path, "--http", "0", "--exit-with-parent"] + extra
    let pipe = Pipe()
    process.standardOutput = pipe
    process.standardError = pipe
    pipe.fileHandleForReading.readabilityHandler = { [weak self] h in
      guard let text = String(data: h.availableData, encoding: .utf8), !text.isEmpty else { return }
      DispatchQueue.main.async { self?.consume(text) }
    }
    process.terminationHandler = { [weak self] _ in
      DispatchQueue.main.async { self?.onExit?(self?.output ?? "") }
    }
  }

  func start() throws { try process.run() }

  // SIGTERM makes the bridge send "disarm" before it exits.
  func stop() {
    guard process.isRunning else { return }
    process.terminate()
    let deadline = Date().addingTimeInterval(1.5)
    while process.isRunning && Date() < deadline { usleep(20_000) }
  }

  private func consume(_ text: String) {
    output = String((output + text).suffix(2000))
    guard onReady != nil,
          let r = output.range(of: #"open http://localhost:(\d+)"#, options: .regularExpression),
          let port = Int(output[r].split(separator: ":").last ?? "") else { return }
    let ready = onReady
    onReady = nil
    ready?(port)
  }
}

final class AppDelegate: NSObject, NSApplicationDelegate, NSWindowDelegate, WKScriptMessageHandler {
  private var panel: FloatPanel!
  private var web: FloatWebView!
  private var bridge: BridgeProcess?
  private var port = 0
  private var statusItem: NSStatusItem!

  func applicationDidFinishLaunching(_ note: Notification) {
    makePanel()
    makeStatusItem()
    startBridge()
  }

  func applicationWillTerminate(_ note: Notification) { bridge?.stop() }

  func windowWillClose(_ note: Notification) { NSApp.terminate(nil) }

  // MARK: window

  private func makePanel() {
    panel = FloatPanel(contentRect: NSRect(x: 40, y: 120, width: 360, height: 700),
                       styleMask: [.titled, .closable, .resizable, .utilityWindow, .nonactivatingPanel],
                       backing: .buffered, defer: false)
    panel.title = "MouthMod 悬浮窗"
    panel.level = .floating
    panel.isFloatingPanel = true
    panel.hidesOnDeactivate = false  // this app is never active
    panel.collectionBehavior = [.canJoinAllSpaces, .fullScreenAuxiliary]
    panel.delegate = self
    panel.setFrameAutosaveName("MouthModFloat")

    let config = WKWebViewConfiguration()
    config.userContentController.add(self, name: "mm")
    web = FloatWebView(frame: panel.contentView!.bounds, configuration: config)
    web.autoresizingMask = [.width, .height]
    web.setValue(false, forKey: "drawsBackground")  // no white flash before the page
    panel.contentView!.addSubview(web)
    showMessage("正在启动本地桥…")
    panel.orderFrontRegardless()
  }

  private func showMessage(_ text: String) {
    let html = """
      <body style="margin:0;padding:14px;background:#16181d;color:#e6e6e6;
      font:13px/1.5 -apple-system,'PingFang SC',sans-serif;white-space:pre-wrap">\(text)</body>
      """
    web.loadHTMLString(html, baseURL: nil)
  }

  // Keyboard only while the page has a field focused; then back to the game.
  func userContentController(_ c: WKUserContentController, didReceive message: WKScriptMessage) {
    guard let what = message.body as? String else { return }
    if what == "keyboard" {
      panel.allowKey = true
      panel.makeKey()
    } else if what == "release" {
      panel.allowKey = false
      if panel.isKeyWindow {
        panel.orderOut(nil)  // key returns to the front app's window
        panel.orderFrontRegardless()
      }
    }
  }

  // MARK: menu bar

  private func makeStatusItem() {
    statusItem = NSStatusBar.system.statusItem(withLength: NSStatusItem.variableLength)
    statusItem.button?.image = NSImage(systemSymbolName: "gamecontroller", accessibilityDescription: "MouthMod")
    let menu = NSMenu()
    menu.addItem(withTitle: "显示悬浮窗", action: #selector(showPanel), keyEquivalent: "")
    menu.addItem(withTitle: "打开完整测试页", action: #selector(openFullPage), keyEquivalent: "")
    menu.addItem(withTitle: "重新加载", action: #selector(reloadPage), keyEquivalent: "")
    menu.addItem(.separator())
    menu.addItem(withTitle: "退出（停用板子）", action: #selector(quit), keyEquivalent: "")
    for item in menu.items { item.target = self }
    statusItem.menu = menu
  }

  @objc private func showPanel() { panel.orderFrontRegardless() }
  @objc private func reloadPage() { if port > 0 { loadPage() } }
  @objc private func quit() { NSApp.terminate(nil) }
  @objc private func openFullPage() {
    if port > 0, let url = URL(string: "http://localhost:\(port)/") { NSWorkspace.shared.open(url) }
  }

  // MARK: bridge

  private func startBridge() {
    guard let script = Bundle.main.url(forResource: "bridge", withExtension: "py",
                                       subdirectory: "look_demo") else {
      showMessage("应用包里缺少 look_demo/bridge.py，请重新运行 scripts/build-float-app.sh")
      return
    }
    let b = BridgeProcess(script: script)
    b.onReady = { [weak self] port in
      self?.port = port
      self?.loadPage()
    }
    b.onExit = { [weak self] output in
      self?.port = 0
      self?.showMessage("本地桥已退出。\n\n最后输出：\n\(output)\n\n常见原因：没装 Command Line Tools（/usr/bin/python3），"
                        + "或者终端里已经有一个 bridge.py 占着串口。处理后从菜单栏图标退出再重新打开。")
    }
    do {
      try b.start()
      bridge = b
    } catch {
      showMessage("无法启动 /usr/bin/python3：\(error.localizedDescription)\n请先安装 Command Line Tools：xcode-select --install")
    }
  }

  private func loadPage() {
    guard let url = URL(string: "http://127.0.0.1:\(port)/float.html") else { return }
    web.load(URLRequest(url: url))
  }
}

let app = NSApplication.shared
let delegate = AppDelegate()
app.delegate = delegate
app.setActivationPolicy(.accessory)  // no Dock icon, never the front app
app.run()
