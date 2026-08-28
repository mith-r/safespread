const fs = require('fs');
const path = require('path');
const {
  IOSConfig,
  withAppDelegate,
  withDangerousMod,
  withXcodeProject,
} = require('@expo/config-plugins');

const SCENE_DELEGATE = `internal import Expo
internal import ExpoModulesCore

@objc(SceneDelegate)
class SceneDelegate: UIResponder, UIWindowSceneDelegate {
  var window: UIWindow?

  func scene(
    _ scene: UIScene,
    willConnectTo session: UISceneSession,
    options connectionOptions: UIScene.ConnectionOptions
  ) {
    guard let windowScene = scene as? UIWindowScene,
          let appDelegate = UIApplication.shared.delegate as? AppDelegate,
          let factory = appDelegate.reactNativeFactory else {
      return
    }

    let window = UIWindow(windowScene: windowScene)
    self.window = window
    appDelegate.window = window
    factory.startReactNative(withModuleName: "main", in: window, launchOptions: nil)
  }

  func sceneDidBecomeActive(_ scene: UIScene) {
    ExpoAppDelegateSubscriberManager.applicationDidBecomeActive(UIApplication.shared)
  }

  func sceneWillResignActive(_ scene: UIScene) {
    ExpoAppDelegateSubscriberManager.applicationWillResignActive(UIApplication.shared)
  }

  func sceneWillEnterForeground(_ scene: UIScene) {
    ExpoAppDelegateSubscriberManager.applicationWillEnterForeground(UIApplication.shared)
  }

  func sceneDidEnterBackground(_ scene: UIScene) {
    ExpoAppDelegateSubscriberManager.applicationDidEnterBackground(UIApplication.shared)
  }
}
`;

function withIosSceneDelegate(config) {
  config = withAppDelegate(config, (appDelegateConfig) => {
    if (appDelegateConfig.modResults.language !== 'swift') {
      throw new Error('SafeSpreadVIO requires a Swift AppDelegate.');
    }
    const source = appDelegateConfig.modResults.contents;
    if (!source.includes('SceneDelegate owns the window')) {
      const launchBlock = /\n#if os\(iOS\) \|\| os\(tvOS\)\n    window = UIWindow\(frame: UIScreen\.main\.bounds\)\n    factory\.startReactNative\(\n      withModuleName: "main",\n      in: window,\n      launchOptions: launchOptions\)\n#endif\n/;
      if (!launchBlock.test(source)) {
        throw new Error('Could not locate Expo React Native launch block in AppDelegate.swift.');
      }
      appDelegateConfig.modResults.contents = source.replace(
        launchBlock,
        '\n    // SceneDelegate owns the window and starts React Native. Apps built with\n' +
        '    // the iOS 27 SDK are required to use the scene-based lifecycle.\n',
      );
    }
    return appDelegateConfig;
  });

  config = withDangerousMod(config, ['ios', async (dangerousConfig) => {
    const sourceRoot = IOSConfig.Paths.getSourceRoot(dangerousConfig.modRequest.projectRoot);
    fs.writeFileSync(path.join(sourceRoot, 'SceneDelegate.swift'), SCENE_DELEGATE);
    return dangerousConfig;
  }]);

  config = withXcodeProject(config, (projectConfig) => {
    const projectName = path.basename(
      IOSConfig.Paths.getSourceRoot(projectConfig.modRequest.projectRoot),
    );
    const filepath = `${projectName}/SceneDelegate.swift`;
    if (!projectConfig.modResults.hasFile(filepath)) {
      IOSConfig.XcodeUtils.addBuildSourceFileToGroup({
        filepath,
        groupName: projectName,
        project: projectConfig.modResults,
      });
    }
    return projectConfig;
  });

  return config;
}

module.exports = withIosSceneDelegate;
