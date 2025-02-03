'use strict';

class WafleUITree {
  constructor(opts) {
    this.config = opts || {};
    if (typeof this.config.dbVersion === 'undefined') this.config.dbVersion = 1;
    this.vtree = {};
    this.groupBlocks = [];
    this.renderer = new WafleTemplate();
    this.id = -1;
    this.dom = false;
    this.blocksRQ = []; //-- Blocks retrieved queue, store the pending blocks to put into design
    iceStudio.bus.events.subscribe(
      'localDatabase.retrieved',
      'blockRetrieved',
      this
    );
  }

  setDomRoot(root) {
    this.dom = root;
  }

  setId(id) {
    this.id = id;
  }

  setTree(tree) {
    this.vtree = tree;
  }

  render() {
    return this.vtree === false
      ? ''
      : this.renderer.render(this.tpl(), {
          tree: this.vtree,
        });
  }

  tpl() {
    return `<div id="tree-view-${this.id}" class="tree-view-root tree-view">
              {{#tree}}
                {{#isFolder}}
                  <div class="tree-view-main--folder tree-view--folder closed" data-nodeid="{{{id}}}">
                    <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                    {{#items}}
                      {{#isFolder}}
                          <div class="tree-view--folder closed" data-nodeid="{{{id}}}">
                            <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                            {{#items}}
                                {{#isFolder}}
                                  <div class="tree-view--folder closed" data-nodeid="{{{id}}}">
                                    <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                    {{#items}}
                                      {{#isFolder}}
                                        <div class="tree-view--folder closed" data-nodeid="{{{id}}}">
                                          <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                          {{#items}}
                                            {{#isFolder}}
                                              <div class="tree-view--folder closed" data-nodeid="{{{id}}}">
                                                  <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                                  {{#items}}
                                                    {{#isFolder}}
                                                      <div class="tree-view--folder closed" data-nodeid="{{{id}}}">
                                                        <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                                        {{#items}}
                                                          {{#isFolder}}
                                                            <div class="tree-view--folder closed" data-nodeid="{{{id}}}">
                                                              <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                                              {{#items}}
                                                                {{#isFolder}}
                                                                  <div class="tree-view--folder closed" data-nodeid="{{{id}}}">
                                                                    <span class="tree-view--folder-name {{#hasSubFolders}}tree-view--folder-subfolders{{/hasSubFolders}}"  data-guievt="click" data-handler="this.toggleFolder" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                                                  </div>
                                                                {{/isFolder}}
                                                                {{#isLeaf}}
                                                                <span class="tree-view--leaf" data-nodeid="{{{id}}}"  data-guievt="click" data-handler="this.getBlock" data-args='{"id":"{{{id}}}"}'><i class="block-icon"></i><input type="checkbox"  data-guievt="checkbox" data-handler="treeView.selectBlock" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                                              {{/isLeaf}}
                                                              {{/items}}
                                                            </div>
                                                          {{/isFolder}}
                                                          {{#isLeaf}}
                                                          <span class="tree-view--leaf" data-nodeid="{{{id}}}"  data-guievt="click" data-handler="this.getBlock" data-args='{"id":"{{{id}}}"}'><i class="block-icon"></i><input type="checkbox"  data-guievt="checkbox" data-handler="treeView.selectBlock" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                                        {{/isLeaf}}
                                                        {{/items}}
                                                      </div>
                                                    {{/isFolder}}
                                                    {{#isLeaf}}
                                                    <span class="tree-view--leaf" data-nodeid="{{{id}}}"  data-guievt="click" data-handler="this.getBlock" data-args='{"id":"{{{id}}}"}'><i class="block-icon"></i><input type="checkbox"  data-guievt="checkbox" data-handler="treeView.selectBlock" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                                  {{/isLeaf}}
                                                  {{/items}}
                                              </div>
                                            {{/isFolder}}
                                            {{#isLeaf}}
                                            <span class="tree-view--leaf" data-nodeid="{{{id}}}"  data-guievt="click" data-handler="this.getBlock" data-args='{"id":"{{{id}}}"}'><i class="block-icon"></i><input type="checkbox"  data-guievt="checkbox" data-handler="treeView.selectBlock" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                          {{/isLeaf}}
                                          {{/items}}
                                        </div>
                                      {{/isFolder}}
                                      {{#isLeaf}}
                                      <span class="tree-view--leaf" data-nodeid="{{{id}}}"  data-guievt="click" data-handler="this.getBlock" data-args='{"id":"{{{id}}}"}'><i class="block-icon"></i><input type="checkbox"  data-guievt="checkbox" data-handler="treeView.selectBlock" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                                    {{/isLeaf}}
                                    {{/items}}
                                  </div>
                                {{/isFolder}}
                                {{#isLeaf}}
                                <span class="tree-view--leaf" data-nodeid="{{{id}}}"  data-guievt="click" data-handler="this.getBlock" data-args='{"id":"{{{id}}}"}'><i class="block-icon"></i><input type="checkbox"  data-guievt="checkbox" data-handler="treeView.selectBlock" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                              {{/isLeaf}}
                                {{/items}}
                          </div>
                      {{/isFolder}}
                      {{#isLeaf}}
                        <span class="tree-view--leaf" data-nodeid="{{{id}}}"  data-guievt="click" data-handler="this.getBlock" data-args='{"id":"{{{id}}}"}'><i class="block-icon"></i><input type="checkbox"  data-guievt="checkbox" data-handler="treeView.selectBlock" data-args='{"id":"{{{id}}}"}'>{{{name}}}<i class="in-use"></i></span>
                      {{/isLeaf}}
                     {{/items}}
                  </div>
                {{/isFolder}}
              {{/tree}}
            </div>
            <div class="blocks-view">
              <div class="blocks-loader">
                <div class="blocks-loader--progress"></div>
              </div>
              <div class="blocks-db"></div>
              </div>`;
  }

  toggle(root, folder) {
    let search = this.toggleFolderState(this.vtree, folder);
    if (search) {
      let el = iceStudio.gui.el(
        `.tree-view--folder[data-nodeid="${folder}"]`,
        root
      );
      iceStudio.gui.elToggleClass(el[0], 'closed');
    }
  }

  hasSubFolders(tree) {
    if (typeof tree.items !== 'undefined') {
      for (let i = 0; i < tree.items.length; i++) {
        if (tree.items[i].isFolder === true) {
          return true;
        }
      }
    }
    return false;
  }

  toggleFolderState(tree, folder) {
    if (Array.isArray(tree)) {
      for (let i = 0; i < tree.length; i++) {
        if (tree[i].id === folder) {
          tree[i].opened = tree[i].opened ? false : true;
          return tree[i];
        }
        let tmp = this.toggleFolderState(tree[i], folder);
        if (tmp !== false) return tmp;
      }
    } else {
      if (tree.isFolder === true) {
        if (tree.id === folder) {
          tree.opened = tree.opened ? false : true;
          return tree;
        }
        for (let j = 0; j < tree.items.length; j++) {
          let tmp2 = this.toggleFolderState(tree.items[j], folder);
          if (tmp2 !== false) return tmp2;
        }
      }
    }
    return false;
  }

  blockRetrieved(item) {
    for (let i = 0; i < this.blocksRQ.length; i++) {
      if (this.blocksRQ[i].id === item.id) {
        iceStudio.bus.events.publish('block.addFromFile', item.path);
        this.blocksRQ.splice(i, 1);
      }
    }
  }

  getBlock(root, args) {
    let item = {
      id: args.id,
      store: 'blockAssets',
      block: false,
    };

    let transaction = {
      database: {
        dbId: 'Collections',
        storages: ['blockAssets'],
        version: 1,
      },
      data: item,
    };
    this.blocksRQ.push(item);
    iceStudio.bus.events.publish('localDatabase.retrieve', transaction);
    this.setInUse(item.id);
  }

  setInUse(blockId) {
    let leafs = iceStudio.gui.el(
      `.tree-view--leaf[data-nodeid="${blockId}"]`,
      this.dom
    );
    for (let i = 0; i < leafs.length; i++) {
      iceStudio.gui.elAddClass(leafs[i], 'is-in-use');
      this.setParentInUse(leafs[i]);
    }
  }

  setParentInUse(el) {
    if (iceStudio.gui.elHasClass(el.parentNode, 'tree-view--folder')) {
      iceStudio.gui.elAddClass(el.parentNode, 'is-in-use');
      this.setParentInUse(el.parentNode);
    }
  }
}
