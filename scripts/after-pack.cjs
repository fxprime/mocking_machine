const { signAsync } = require("@electron/osx-sign");
const { execFile } = require("node:child_process");
const path = require("node:path");
const { promisify } = require("node:util");

const execFileAsync = promisify(execFile);

module.exports = async function signAndVerifyMacBundle(context) {
  if (context.electronPlatformName !== "darwin") return;

  const appPath = path.join(
    context.appOutDir,
    `${context.packager.appInfo.productFilename}.app`
  );
  const entitlementsPath = path.join(
    context.packager.projectDir,
    "build",
    "entitlements.mac.plist"
  );

  // An unsigned Electron bundle retains nested upstream signatures that become
  // inconsistent when Electron.app is renamed. Sign leaf-to-root so the preview
  // remains internally valid even when no Developer ID certificate is available.
  await signAsync({
    app: appPath,
    platform: "darwin",
    identity: "-",
    identityValidation: false,
    optionsForFile(filePath) {
      // Every Electron helper is a separate ad-hoc-signed process. Each one
      // needs library validation disabled so it can load the separately
      // ad-hoc-signed Electron Framework.
      return filePath.endsWith(".app")
        ? { entitlements: entitlementsPath }
        : {};
    },
    preAutoEntitlements: false,
    preEmbedProvisioningProfile: false
  });

  await execFileAsync("/usr/bin/codesign", [
    "--verify",
    "--deep",
    "--strict",
    "--verbose=4",
    appPath
  ]);
};
