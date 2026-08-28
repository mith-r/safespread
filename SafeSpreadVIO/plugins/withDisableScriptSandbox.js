// Expo config plugin: disable Xcode "User Script Sandboxing" for the app target.
//
// Why: React Native's build script phases (e.g. "Bundle React Native code and
// images") write files such as ip.txt into the .app bundle. With
// ENABLE_USER_SCRIPT_SANDBOXING = YES (Xcode's newer default, which
// `expo prebuild` regenerates), those writes are denied with
// "Operation not permitted" and device builds fail. Forcing it to NO keeps
// device builds working across clean prebuilds.
const { withXcodeProject } = require('@expo/config-plugins');

module.exports = function withDisableScriptSandbox(config) {
  return withXcodeProject(config, (config) => {
    const project = config.modResults;
    const configurations = project.pbxXCBuildConfigurationSection();
    for (const key in configurations) {
      const buildSettings = configurations[key] && configurations[key].buildSettings;
      if (buildSettings) {
        buildSettings.ENABLE_USER_SCRIPT_SANDBOXING = 'NO';
      }
    }
    return config;
  });
};
