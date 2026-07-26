const supportedActions = new Set(["minimize", "toggle-maximize", "close"]);

export function windowChromeState(browserWindow, platform = process.platform) {
  return {
    customTitleBar: platform === "linux",
    maximized: platform === "linux" && Boolean(browserWindow?.isMaximized())
  };
}

export function performWindowAction(browserWindow, action, platform = process.platform) {
  if (platform !== "linux" || !supportedActions.has(action)) {
    throw new Error("Unsupported window action.");
  }

  if (action === "minimize") browserWindow.minimize();
  if (action === "toggle-maximize") {
    if (browserWindow.isMaximized()) browserWindow.unmaximize();
    else browserWindow.maximize();
  }
  if (action === "close") browserWindow.close();

  return windowChromeState(browserWindow, platform);
}
