//----------------------------------------------------------------------------
//-- This module defines the global data structures used in Icestudio
//-- The different Icestudio modules get this information and store it
//-- in the common module, so that it is always available to the rest
//-- of modules
//----------------------------------------------------------------------------
'use strict';

angular.module('icestudio').service(
  'common',
  function (
    //-- node Path module
    //-- More info: https://nodejs.org/docs/latest-v17.x/api/path.html
    nodePath,

    //-- Create temporary files and directories
    //-- More info: https://www.npmjs.com/package/tmp
    nodeTmp,
    nodeFs,
    _package
  ) {
    // Project version. It defines the current structure for the
    // icestudio projects (Both in memory and on the .ice files)
    this.VERSION = '1.2';

    // Project status: Has it change from the previous build or not?
    this.hasChangesSinceBuild = false;
    // Tri-state ports: Are they present in any opened designs or blocks, and is this approved?
    //                  (User profile "allowInoutPorts" is false)
    this.allowProjectInoutPorts = false;

    // All project dependencies
    this.allDependencies = {};

    // Selected board
    this.boards = []; //-- Array with the board objects. Initialized with the boards.loadBoards() function
    this.selectedBoard = null; //-- Board object. Current board used
    this.pinoutInputHTML = '';
    this.pinoutOutputHTML = '';

    // Selected collection
    this.defaultCollection = null;
    this.internalCollections = [];
    this.externalCollections = [];
    this.selectedCollection = null;

    // FPGA resources
    this.FPGAResources = {
      ffs: '-',
      luts: '-',
      pios: '-',
      plbs: '-',
      brams: '-',
    };

    // Debug mode (uncomment)
    this.DEBUGMODE = 1;

    // Command output
    this.commandOutput = '';

    // Apio develop URL
    // The apio develop is installed using this command:
    // pip install -U git+https://github.com/FPGAwars/apio.git@develop#egg=apio
    this.APIO_PIP_VCS =
      'git+https://github.com/FPGAwars/apio.git@develop#egg=apio';

    // Operating system: true/false
    this.LINUX = Boolean(process.platform.indexOf('linux') > -1);
    this.WIN32 = Boolean(process.platform.indexOf('win32') > -1);
    this.DARWIN = Boolean(process.platform.indexOf('darwin') > -1);

    //----------------------------------------
    //--  Icestudio app folder
    //--     |---> resources/locale: Translation files
    //--     |---> resources/sample: Folder for testing
    //--     |---> resources/collection: Default collection
    //--     |---> resources/plugin:  Plugins

    //-- Locale DIR: Translation files
    this.LOCALE_DIR = nodePath.join('resources', 'locale');

    //-- Sample DIR: just for testing apio
    this.SAMPLE_DIR = nodePath.join('resources', 'sample');

    //-- The default collection is stored in this Folder
    this.DEFAULT_COLLECTION_DIR = nodePath.resolve(
      nodePath.join('resources', 'collection')
    );

    //-- Folder for the system plugins
    this.DEFAULT_PLUGIN_DIR = nodePath.resolve(
      nodePath.join('resources', 'plugins')
    );

    //-- Path were the executale is run
    this.APP_DIR = nodePath.dirname(process.execPath);

    //-- Icestudio APP dir
    this.APP = process.cwd();

    //----------------------------------------------------
    //-- User/system Home folder (BASE_DIR)
    //--   |---> icestudio.log : Log file (debugging)
    //--   |---> .icestudio : Icestudio folder
    //--   |---> .icestudio/collections: Installed collections
    //--   |---> .icestudio/apio : Apio in installed in this folder
    //--   |---> .icestudio/profile.json
    //--   |---> .icestudio/venv: Python virtual environment
    //--   |---> .icestudio/venv/bin : Executables (Linux, mac)
    //--   |---> .icestudio/venv/Scripts: Executables (Windows)
    //--   |---> .icestudio/venv/bin/pip3
    //--   |---> .icestudio/venv/bin/apio
    //--
    this.ICESTUDIO_HOME =
      this.WIN32 && process.arch === 'ia32' ? 'icestudio_home' : '.icestudio';
    this.BASE_DIR = process.env.HOME || process.env.USERPROFILE;
    this.LOGFILE = nodePath.join(this.BASE_DIR, 'icestudio.log');
    this.ICESTUDIO_DIR = nodePath.join(this.BASE_DIR, this.ICESTUDIO_HOME);

    this.INTERNAL_COLLECTIONS_DIR = nodePath.join(
      this.ICESTUDIO_DIR,
      'collections'
    );
    this.APIO_HOME_DIR = nodePath.join(this.ICESTUDIO_DIR, 'apio');
    this.PROFILE_PATH = nodePath.join(this.ICESTUDIO_DIR, 'profile.json');

    //-- Python virtual environment
    this.ENV_DIR = nodePath.join(this.ICESTUDIO_DIR, 'venv');

    //-- Folder for the executables in the python virtual environment
    this.ENV_BIN_DIR = nodePath.join(
      this.ENV_DIR,
      this.WIN32 ? 'Scripts' : 'bin'
    );

    //-- pip3 executable in the virtual environment
    this.ENV_PIP = nodePath.join(this.ENV_BIN_DIR, 'pip3');

    //-- apio executable in the virtual environment
    this.ENV_APIO = nodePath.join(
      this.ENV_BIN_DIR,
      this.WIN32 ? 'apio.exe' : 'apio'
    );

    this.CACHE_DIR = nodePath.join(this.ICESTUDIO_DIR, '.cache');
    this.IMAGE_CACHE_DIR = nodePath.join(this.CACHE_DIR, 'images');
    this.OLD_BUILD_DIR = nodePath.join(this.ICESTUDIO_DIR, '.build');

    //-- Get the Icestudio Version
    this.ICESTUDIO_VERSION = _package.version;

    //-- Apio version values
    this.APIO_VERSION_STABLE = 0; //-- Use the stable version
    this.APIO_VERSION_LATEST_STABLE = 1; //-- Use the latest stable (available in the pypi repo)
    this.APIO_VERSION_DEV = 2; //-- Use the development apio version (from the github repo)
    this.APIO_VERSION = this.APIO_VERSION_STABLE; //-- Default apio version: STABLE

    //-- Apio PACKAGES VERSION to install for the Stable Version
    this.APIO_PKG_OSS_CAD_SUITE_VERSION = '0.0.9';

    //-- Get the System PATH
    this.PATH = process.env.PATH;

    //-- Set the apio command. It consist of two parts. The first is for defining two
    //-- environment variables: APIO_HOME_DIR and PATH
    //--    * APIO_HOME_DIR: It is used by apio to know where is located its installation.
    //--    * PATH: The PATH env variable (The virtuan env path is added in the begining)
    //--
    //-- The second part is the apio executable itself
    //--
    //-- EXAMPLE FOR Linux/MAC: (split in several lines)
    //-- APIO_CMD = APIO_HOME_DIR="/home/obijuan/.icestudio/apio"
    //              PATH="/home/obijuan/.icestudio/venv/bin:$PATH"
    //              "/home/obijuan/.icestudio/venv/bin/apio"
    //-- NOTICE THE paths are quoted! This is needed because there can be path with spaces in their folder names

    //-- EXAMPLE FOR Windows: (split in several lines)
    //-- APIO_CMD = set APIO_HOME_DIR="c:\Users\Obijuan\.icestudio\apio"&
    //              set PATH=c:\Users\Obijuan\.icestudio\venv\bin\apio;%PATH%&
    //              "C:\Users\Obijuan\.icestudio\venv\Scripts\apio.exe"

    if (this.WIN32) {
      //-- Apio execution command for Windows machines

      //-- APIO_HOME_DIR env variable
      this.APIO_CMD =
        'set APIO_HOME_DIR="' +
        this.APIO_HOME_DIR +
        '"& ' +
        'set PATH=' +
        this.ENV_BIN_DIR +
        ';' +
        this.PATH +
        '& ' +
        '"' +
        this.ENV_APIO +
        '"';

      //-- IMPORTANT!!! THERE SHOULD BE NO SPACE between APIO_HOME_DIR and the '&' operator in Windows!!
      //-- This a very difficult ERROR TO SPOT:
      //-- set APIO_HOME_DIR=c:\Users\Obijuan\.icestudio\apio &
      //-- It will set the APIO_HOME_DIR environment var to "c:\Users\Obijuan\.icestudio\apio "
      //-- (Notice the space in the end)
    } else {
      //-- Apio execution command for Linux/MAC machines

      this.APIO_CMD =
        //-- APIO_HOME_DIR env variable
        'APIO_HOME_DIR="' +
        this.APIO_HOME_DIR +
        '" ' +
        //-- Add the virtual env PATH in the begining of the PATH env. variable
        'PATH="' +
        this.ENV_BIN_DIR +
        ':' +
        this.PATH +
        '" ' +
        //-- Apio executable
        '"' +
        this.ENV_APIO +
        '"';
    }

    this.BUILD_DIR_OBJ = new nodeTmp.dirSync({
      prefix: 'icestudio-',
      unsafeCleanup: true,
    });
    this.BUILD_DIR = this.BUILD_DIR_OBJ.name;
    this.BUILD_DIR_TMP = this.BUILD_DIR_OBJ.name;

    this.PATTERN_PORT_LABEL =
      /^\s*(@*[A-Za-z_][A-Za-z_$0-9]*)?\s*(\[\s*([A-Za-z_$0-9+\-*/]+)\s*:\s*([A-Za-z_$0-9+\-*/]+)\s*\])?\s*$/;
    this.PATTERN_PARAM_LABEL = /^([A-Za-z_][A-Za-z_$0-9]*)?$/;

    //-- Check the port names. Ex. a[1:0], b
    this.PATTERN_GLOBAL_PORT_LABEL =
      /^([^\[\]]+)?(\[\s*([A-Za-z_$0-9+\-*/]+)\s*:\s*([A-Za-z_$0-9+\-*/]+)\s*\])?$/;
    this.PATTERN_GLOBAL_PARAM_LABEL = /^([^\[\]]+)?$/;

    this.setBuildDir = function (buildpath) {
      let fserror = false;
      if (!nodeFs.existsSync(buildpath)) {
        try {
          nodeFs.mkdirSync(buildpath, { recursive: true });
        } catch (e) {
          fserror = true;
        }
      }
      if (!fserror) {
        this.BUILD_DIR = buildpath;
      } else {
        this.BUILD_DIR = this.BUILD_DIR_TMP;
      }
    };

    this.showToolchain = function () {
      return (
        (this.selectedBoard && this.selectedBoard.info.interface !== 'GPIO') ||
        false
      );
    };

    this.showDrivers = function () {
      return (
        (this.selectedBoard &&
          (this.selectedBoard.info.interface === 'FTDI' ||
            this.selectedBoard.info.interface === 'Serial')) ||
        false
      );
    };

    this.isEditingSubmodule = false;

    let storage = new IceHD();
    if (!storage.isValidPath(this.ICESTUDIO_DIR)) {
      storage.mkDir(this.ICESTUDIO_DIR);
    }

    //-- Create the Cache dir
    //-- If it was not previously created
    storage.mkDir(this.CACHE_DIR);

    //-- Create the Image Cache dir
    //-- If it was not previously created
    storage.mkDir(this.IMAGE_CACHE_DIR);
  }
);
