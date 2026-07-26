export function getLaunchAtLoginState(electronApp, platform = process.platform) {
  const supported = (platform === "darwin" || platform === "win32") &&
    electronApp.isPackaged;
  return {
    supported,
    enabled: supported && electronApp.getLoginItemSettings().openAtLogin
  };
}

export function setLaunchAtLoginState(electronApp, enabled, platform = process.platform) {
  const current = getLaunchAtLoginState(electronApp, platform);
  if (!current.supported) {
    throw new Error(
      "Launch at Login is available in installed macOS and Windows applications."
    );
  }
  electronApp.setLoginItemSettings({ openAtLogin: Boolean(enabled) });
  return getLaunchAtLoginState(electronApp, platform);
}
