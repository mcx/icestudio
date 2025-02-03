'use strict';

angular
  .module('icestudio')
  .filter('shortcut', function (shortcuts) {
    return function (action) {
      return shortcuts.label(action);
    };
  })
  .service('shortcuts', function (common) {
    this.method = function (action, method) {
      // Configure shortcut method
      if (action in shortcuts) {
        shortcuts[action]['method'] = method;
      }
    };

    this.execute = function (event, opt) {
      // Execute shortcut method
      // Options:
      // - opt.prompt: enable shortcut when a prompt is shown
      // - opt.disable: enable shortcut when the graph is disabled

      var action = '';
      var method = null;
      var system = common.DARWIN ? 'mac' : 'linux';
      var ret = { preventDefault: false };
      for (action in shortcuts) {
        var options = shortcuts[action].opt || {};
        var command = shortcuts[action][system];
        if (
          event.keyCode === command.key &&
          event.altKey === (command.alt || false) &&
          event.ctrlKey === (command.ctrl || false) &&
          event.metaKey === (command.meta || false) &&
          event.shiftKey === (command.shift || false) &&
          (!opt.prompt || options.prompt || false) &&
          (!opt.disabled || options.disabled || false)
        ) {
          method = shortcuts[action].method;
          ret.preventDefault = options.preventDefault || false;
          break;
        }
      }
      if (method) {
        method();
      }
      return ret;
    };

    this.label = function (action) {
      // Return shortcut label
      var label = '';
      if (action in shortcuts) {
        if (common.DARWIN) {
          label = shortcuts[action].mac.label;
        } else {
          label = shortcuts[action].linux.label;
        }
      }
      return label;
    };

    /* For easy keycode identification: https://keycode.info/ */

    var shortcuts = {
      showCollectionManager: {
        linux: { label: 'Ctrl+M', ctrl: true, key: 77 },
        mac: { label: '⌘+M', meta: true, key: 77 },
      },
      showToolBox: {
        linux: { label: 'Ctrl+T', ctrl: true, key: 84 },
        mac: { label: '⌘+T', meta: true, key: 84 },
      },
      showLabelFinder: {
        linux: { label: 'Ctrl+F', ctrl: true, key: 70 },
        mac: { label: '⌘+F', meta: true, key: 70 },
      },
      newProject: {
        linux: { label: 'Ctrl+N', ctrl: true, key: 78 },
        mac: { label: '⌘+N', meta: true, key: 78 },
      },
      openProject: {
        linux: { label: 'Ctrl+O', ctrl: true, key: 79 },
        mac: { label: '⌘+O', meta: true, key: 79 },
      },
      saveProject: {
        linux: { label: 'Ctrl+S', ctrl: true, key: 83 },
        mac: { label: '⌘+S', meta: true, key: 83 },
        opt: { prompt: true },
      },
      saveProjectAs: {
        linux: { label: 'Ctrl+Shift+S', ctrl: true, shift: true, key: 83 },
        mac: { label: '⌘+⇧+S', meta: true, shift: true, key: 83 },
        opt: { prompt: true },
      },
      quit: {
        linux: { label: 'Ctrl+Q', ctrl: true, key: 81 },
        mac: { label: '⌘+Q', meta: true, key: 81 },
      },
      undoGraph: {
        linux: { label: 'Ctrl+Z', ctrl: true, key: 90 },
        mac: { label: '⌘+Z', meta: true, key: 90 },
        opt: { preventDefault: true },
      },
      redoGraph: {
        linux: { label: 'Ctrl+Y', ctrl: true, key: 89 },
        mac: { label: '⌘+Y', meta: true, key: 89 },
        opt: { preventDefault: true },
      },
      redoGraph2: {
        linux: { label: 'Ctrl+Shift+Z', ctrl: true, shift: true, key: 90 },
        mac: { label: '⌘+⇧+Z', meta: true, shift: true, key: 90 },
        opt: { preventDefault: true },
      },
      cutSelected: {
        linux: { label: 'Ctrl+X', ctrl: true, key: 88 },
        mac: { label: '⌘+X', meta: true, key: 88 },
      },
      copySelected: {
        linux: { label: 'Ctrl+C', ctrl: true, key: 67 },
        mac: { label: '⌘+C', meta: true, key: 67 },
      },
      pasteSelected: {
        linux: { label: 'Ctrl+V', ctrl: true, key: 86 },
        mac: { label: '⌘+V', meta: true, key: 86 },
      },
      pasteAndCloneSelected: {
        linux: { label: 'Ctrl+Shift+V', ctrl: true, shift: true, key: 86 },
        mac: { label: '⌘+⇧+V', meta: true, shift: true, key: 86 },
      },
      duplicateSelected: {
        linux: { label: 'Ctrl+D', ctrl: true, key: 68 },
        mac: { label: '⌘+D', meta: true, key: 68 },
      },
      removeSelected: {
        linux: { label: 'Del', key: 46 },
        mac: { label: 'Fn+⌫', key: 46 },
      },
      selectAll: {
        linux: { label: 'Ctrl+A', ctrl: true, key: 65 },
        mac: { label: '⌘+A', meta: true, key: 65 },
      },
      fitContent: {
        linux: { label: 'Ctrl+1', ctrl: true, key: 49 },
        mac: { label: '⌘+1', meta: true, key: 49 },
        opt: { disabled: true },
      },
      verifyCode: {
        linux: { label: 'Ctrl+R', ctrl: true, key: 82 },
        mac: { label: '⌘+R', meta: true, key: 82 },
      },
      buildCode: {
        linux: { label: 'Ctrl+B', ctrl: true, key: 66 },
        mac: { label: '⌘+B', meta: true, key: 66 },
      },
      uploadCode: {
        linux: { label: 'Ctrl+Shift+U', ctrl: true, shift: true, key: 85 },
        mac: { label: '⌘+⇧+U', meta: true, shift: true, key: 85 },
      },
      takeSnapshotPNG: {
        linux: { label: 'Ctrl+Shift+P', ctrl: true, shift: true, key: 80 },
        mac: { label: '⌘+⇧+P', meta: true, shift: true, key: 80 },
      },
      takeSnapshotVideo: {
        linux: { label: 'Ctrl+Shift+R', ctrl: true, shift: true, key: 82 },
        mac: { label: '⌘+⇧+R', meta: true, shift: true, key: 82 },
      },

      stepUp: {
        linux: { label: 'Arrow up', key: 38 },
        mac: { label: 'Arrow up', key: 38 },
      },
      stepDown: {
        linux: { label: 'Arrow down', key: 40 },
        mac: { label: 'Arrow down', key: 40 },
      },
      stepLeft: {
        linux: { label: 'Arrow left', key: 37 },
        mac: { label: 'Arrow left', key: 37 },
      },
      stepRight: {
        linux: { label: 'Arrow right', key: 39 },
        mac: { label: 'Arrow right', key: 39 },
      },
      back: {
        linux: { label: 'Back', key: 8 },
        mac: { label: '⌫', key: 8 },
        opt: { disabled: true },
      },
      takeSnapshot: {
        linux: { label: 'Ctrl+P', ctrl: true, key: 80 },
        mac: { label: '⌘+P', meta: true, key: 80 },
        opt: { prompt: true, disabled: true, preventDefault: true },
      },

      //-- DEBUG: This shortcut is just for testing and debugging purposes
      testing: {
        linux: { label: 'Ctrl+2', ctrl: true, key: 50 },
        mac: { label: '⌘+2', meta: true, key: 50 },
        opt: { disabled: true },
      },
    };
  });
