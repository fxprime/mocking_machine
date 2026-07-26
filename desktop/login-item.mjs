export function getLaunchAtLoginState(electronApp, platform = process.platform) {
  const supported = platform === "darwin" && electronApp.isPackaged;
  return {
    supported,
    enabled: supported && electronApp.getLoginItemSettings().openAtLogin
  };
}

export function setLaunchAtLoginState(electronApp, enabled, platform = process.platform) {
  const current = getLaunchAtLoginState(electronApp, platform);
  if (!current.supported) {
    throw new Error("Launch at Login is available in the installed macOS application.");
  }
  electronApp.setLoginItemSettings({ openAtLogin: Boolean(enabled) });
  return getLaunchAtLoginState(electronApp, platform);
}
