'use strict';

class IceBlock {
  constructor(opts) {
    this.constants = {};
    this.config = opts || {};
    this.fs = new IceHD();
    this.content = false;
  }

  loadFromFile(path, onLoadOK, onLoadERROR) {
    let _this = this;
    this.content = this.fs.readFile(
      path,
      function (filepath, content) {
        _this.content = content;
        onLoadOK(_this.get());
      },
      onLoadERROR
    );
  }

  svgFile(hash, svg) {
    let path = this.fs.joinPath(this.config.cacheDirImg, `${hash}.svg`);
    if (!this.fs.isValidPath(path)) {
      this.fs.writeFile(path, svg);
    }

    return path;
  }

  busLoadFromFile(args) {
    this.fs.readFile(args.path, function (path, content) {
      args.obj = JSON.parse(content);
      //--  ICEpm.publishAt(args.id, "block.loadedFromFile", args);
      iceStudio.bus.events.publish('block.loadedFromFile', args);
    });
  }

  get() {
    return this.content;
  }
}
