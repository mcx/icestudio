'use strict';

angular
  .module('icestudio')
  .service('compiler', function (common, utils, blocks, _package) {
    let currentLibrary = false;
    this.generate = function (target, project, opt) {
      var content = '';
      var files = [];
      this.currentLibrary = false;
      switch (target) {
        case 'verilog':
          content += header('//', opt, true);
          content += '`default_nettype none\n\n';
          currentLibrary = project.dependencies;
          content += verilogCompiler('main', project, opt);
          files.push({
            name: 'main.v',
            content: content,
          });
          break;
        case 'pcf':
          content += header('#', opt);
          content += pcfCompiler(project, opt);
          files.push({
            name: 'main.pcf',
            content: content,
          });
          break;
        case 'lpf':
          content += header('#', opt);
          content += lpfCompiler(project, opt);
          files.push({
            name: 'main.lpf',
            content: content,
          });
          break;

        case 'list':
          files = listCompiler(project);
          break;
        case 'testbench':
          content += header('//', opt, true);
          content += testbenchCompiler(project);
          files.push({
            name: 'main_tb.v',
            content: content,
          });
          break;
        case 'gtkwave':
          content += header('[*]', opt);
          content += gtkwaveCompiler(project);
          files.push({
            name: 'main_tb.gtkw',
            content: content,
          });
          break;
        default:
          break;
      }
      return files;
    };

    function header(comment, opt, linterRules = false) {
      var header = '';
      var date = new Date();
      opt = opt || {};
      if (opt.header !== false) {
        header +=
          comment + ' Code generated by Icestudio ' + _package.version + '\n';
        if (linterRules) {
          header += comment + '  #START Verilator Linter rules:' + '\n';
          header += '/* verilator lint_off PINMISSING */' + '\n';
          header += '/* verilator lint_off WIDTHTRUNC */' + '\n';
          header += '/* verilator lint_off WIDTHEXPAND */' + '\n';
          header += comment + '  #END Verilator Linter rules' + '\n';
        }
        if (opt.datetime !== false) {
          header += comment + ' ' + date.toUTCString() + '\n';
        }
        header += '\n';
      }
      return header;
    }

    function module(data) {
      var code = '';
      if (data && data.name && data.ports) {
        // Header
        code += '\nmodule ' + data.name;

        //-- Parameters

        var params = [];
        let pname = '';
        for (var p in data.params) {
          if (data.params[p] instanceof Object) {
            pname =
              data.params[p].name.charAt(0) === '@'
                ? data.params[p].name.substr(1)
                : data.params[p].name;
            params.push(
              ' parameter ' +
                pname +
                ' = ' +
                (data.params[p].value ? data.params[p].value : '0')
            );
          }
        }

        if (params.length > 0) {
          code += ' #(\n';
          code += params.join(',\n');
          code += '\n)';
        }

        //-- Ports

        var ports = [];
        let i, o, _in, _out, _inout;

        let signed;
        for (i in data.ports.in) {
          _in = data.ports.in[i];
          pname = _in.name;
          signed = '';
          if (_in.name.charAt(0) === '@') {
            pname = _in.name.substr(1);
            signed = ' signed ';
          }
          ports.push(
            ' input ' + signed + (_in.range ? _in.range + ' ' : '') + pname
          );
        }

        for (o in data.ports.out) {
          _out = data.ports.out[o];
          pname = _out.name;
          signed = '';
          if (_out.name.charAt(0) === '@') {
            pname = _out.name.substr(1);
            signed = ' signed ';
          }
          ports.push(
            ' output ' + signed + (_out.range ? _out.range + ' ' : '') + pname
          );
        }
        for (i in data.ports.inout) {
          _inout = data.ports.inout[i];
          pname = _inout.name;
          signed = '';
          if (_inout.name.charAt(0) === '@') {
            pname = _inout.name.substr(1);
            signed = ' signed ';
          }
          ports.push(
            ' inout ' +
              signed +
              (_inout.range ? _inout.range + ' ' : '') +
              pname
          );
        }

        for (i in data.ports.inoutLeft) {
          _in = data.ports.inoutLeft[i];
          pname = _in.name;
          signed = '';
          if (_in.name.charAt(0) === '@') {
            pname = _in.name.substr(1);
            signed = ' signed ';
          }
          ports.push(
            ' inout ' + signed + (_in.range ? _in.range + ' ' : '') + pname
          );
        }
        for (o in data.ports.inoutRight) {
          _out = data.ports.inoutRight[o];
          pname = _out.name;
          signed = '';
          if (_out.name.charAt(0) === '@') {
            pname = _out.name.substr(1);
            signed = ' signed ';
          }
          ports.push(
            ' inout ' + signed + (_out.range ? _out.range + ' ' : '') + pname
          );
        }

        if (ports.length > 0) {
          code += ' (\n';
          code += ports.join(',\n');
          code += '\n)';
        }

        if (params.length === 0 && ports.length === 0) {
          code += '\n';
        }

        code += ';\n';

        // Content

        if (data.content) {
          var content = data.content.split('\n');

          content.forEach(function (element, index, array) {
            array[index] = ' ' + element;
          });

          code += content.join('\n');
        }

        // Footer

        code += '\nendmodule\n';
      }

      return code;
    }

    /* Deprecated *
    function digestNameBlock(block) {
      let id = block;

      if (typeof block.id !== 'undefined') {
        id = utils.digestId2(block.id);

        if (
          typeof block.data !== 'undefined' &&
          typeof block.data.name !== 'undefined' &&
          block.data.name.trim() !== ''
        ) {
          id = `${id}__${block.data.name}`;
        }
      }
      return id.replace(/(-)|(\s)/g, '');
    }
    */

    function getParams(project) {
      var params = [];
      var graph = project.design.graph;

      for (var i in graph.blocks) {
        var block = graph.blocks[i];

        // if no sysClkMhz is defined , no replacement is done, in this way the user is alerted because this empty parameter blocks
        // needs a value and the user should filled depending the board clock.

        let boardClkfreq =
          typeof common.selectedBoard.info.SysClkMhz !== 'undefined' &&
          common.selectedBoard.info.SysClkMhz.length !== false &&
          common.selectedBoard.info.SysClkMhz.length > 0
            ? common.selectedBoard.info.SysClkMhz.length
            : false;

        if (block.type === blocks.BASIC_CONSTANT) {
          if (boardClkfreq && block.data.value === 'SysClkMhz') {
            params.push({
              name: utils.digestId(block.id),
              value: boardClkfreq,
            });
          } else {
            params.push({
              name: utils.digestId(block.id),
              value: block.data.value,
            });
          }
        } else if (block.type === blocks.BASIC_MEMORY) {
          let name = utils.digestId(block.id);

          params.push({
            name: name,
            value: '"' + name + '.list"',
          });
        }
      }

      return params;
    }

    function getPorts(project) {
      var ports = {
        in: [],
        out: [],
        inout: [],
      };
      var graph = project.design.graph;

      for (var i in graph.blocks) {
        var block = graph.blocks[i];
        if (block.type === blocks.BASIC_INPUT) {
          if (
            typeof block.data.inout !== 'undefined' &&
            block.data.inout === true
          ) {
            ports.inout.push({
              name: utils.digestId(block.id),
              range: block.data.range ? block.data.range : '',
            });
          } else {
            ports.in.push({
              name: utils.digestId(block.id),
              range: block.data.range ? block.data.range : '',
            });
          }
        } else if (block.type === blocks.BASIC_OUTPUT) {
          if (
            typeof block.data.inout !== 'undefined' &&
            block.data.inout === true
          ) {
            ports.inout.push({
              name: utils.digestId(block.id),
              range: block.data.range ? block.data.range : '',
            });
          } else {
            ports.out.push({
              name: utils.digestId(block.id),
              range: block.data.range ? block.data.range : '',
            });
          }
        }
      }

      return ports;
    }

    function getContent(name, project) {
      var i, j, w;
      var content = [];
      var graph = project.design.graph;
      var connections = {
        localparam: [],
        wire: [],
        assign: [],
      };
      // We need to rearrange internal design specification to compile it
      // Convert virtual labels to wire and some stuff .

      var vwiresLut = {};
      var lidx, widx, lin, vw;
      var twire;

      //Create virtual wires

      //First identify sources and targets and create a look up table to work easy with it
      if (
        typeof graph !== 'undefined' &&
        graph.blocks.length > 0 &&
        graph.wires.length > 0
      ) {
        for (lidx in graph.blocks) {
          lin = graph.blocks[lidx];
          if (lin.type === blocks.BASIC_INPUT_LABEL) {
            for (widx in graph.wires) {
              vw = graph.wires[widx];
              if (vw.target.block === lin.id) {
                if (typeof vwiresLut[lin.data.name] === 'undefined') {
                  vwiresLut[lin.data.name] = {
                    source: [],
                    target: [],
                  };
                }
                twire = vw.source;
                twire.size = vw.size;
                vwiresLut[lin.data.name].source.push(twire);
              }
            }
          }
          if (lin.type === blocks.BASIC_OUTPUT_LABEL) {
            for (widx in graph.wires) {
              vw = graph.wires[widx];
              if (vw.source.block === lin.id) {
                if (typeof vwiresLut[lin.data.name] === 'undefined') {
                  vwiresLut[lin.data.name] = {
                    source: [],
                    target: [],
                  };
                }

                twire = vw.target;
                twire.size = vw.size;
                vwiresLut[lin.data.name].target.push(twire);
              }
            }
          }
        } //for lin
      } // if typeof....

      //Create virtual wires
      for (widx in vwiresLut) {
        vw = vwiresLut[widx];
        if (vw.source.length > 0 && vw.target.length > 0) {
          for (var vi = 0; vi < vw.source.length; vi++) {
            for (var vj = 0; vj < vw.target.length; vj++) {
              graph.wires.push({
                tcTodelete: true,
                size: vw.size,
                source: vw.source[vi],
                target: vw.target[vj],
                vertices: undefined,
              });
            }
          }
        }
      }

      // Remove virtual blocks
      // Save temporal wires and delete it

      graph.wiresVirtual = [];
      var wtemp = [];
      var iwtemp;
      var wi;
      for (wi = 0; wi < graph.wires.length; wi++) {
        if (
          graph.wires[wi].source.port === 'outlabel' ||
          graph.wires[wi].target.port === 'outlabel' ||
          graph.wires[wi].source.port === 'inlabel' ||
          graph.wires[wi].target.port === 'inlabel'
        ) {
          graph.wiresVirtual.push(graph.wires[wi]);
        } else {
          iwtemp = graph.wires[wi];
          if (typeof iwtemp.source.size !== 'undefined') {
            iwtemp.size = iwtemp.source.size;
          }
          wtemp.push(iwtemp);
        }
      }
      graph.wires = utils.clone(wtemp);
      // End of rearrange design connections for compilation

      for (w in graph.wires) {
        var wire = graph.wires[w];
        if (
          wire.source.port === 'constant-out' ||
          wire.source.port === 'memory-out'
        ) {
          // Local Parameters
          var constantBlock = findBlock(wire.source.block, graph);
          var paramValue = utils.digestId(constantBlock.id);
          if (paramValue) {
            connections.localparam.push(
              'localparam p' + w + ' = ' + paramValue + ';'
            );
          }
        } else {
          // Wires
          var range = wire.size ? ' [' + (wire.size - 1) + ':0] ' : ' ';
          connections.wire.push('wire' + range + 'w' + w + ';');
        }
        // Assign Statements
        for (i in graph.blocks) {
          var block = graph.blocks[i];
          if (block.type === blocks.BASIC_INPUT) {
            if (wire.source.block === block.id) {
              connections.assign.push(
                'assign w' + w + ' = ' + utils.digestId(block.id) + ';'
              );
            }
          } else if (block.type === blocks.BASIC_OUTPUT) {
            if (wire.target.block === block.id) {
              if (
                wire.source.port === 'constant-out' ||
                wire.source.port === 'memory-out'
              ) {
                // connections.assign.push('assign ' + digestId(block.id) + ' = p' + w + ';');
              } else {
                connections.assign.push(
                  'assign ' + utils.digestId(block.id) + ' = w' + w + ';'
                );
              }
            }
          }
        }
      }

      content = content.concat(connections.localparam);
      content = content.concat(connections.wire);
      content = content.concat(connections.assign);

      // Wires Connections

      var numWires = graph.wires.length;
      var gwi, gwj;
      for (i = 1; i < numWires; i++) {
        for (j = 0; j < i; j++) {
          gwi = graph.wires[i];
          gwj = graph.wires[j];
          if (
            gwi.source.block === gwj.source.block &&
            gwi.source.port === gwj.source.port &&
            gwi.source.port !== 'constant-out' &&
            gwi.source.port !== 'memory-out'
          ) {
            content.push('assign w' + i + ' = w' + j + ';');
          }
        }
      }

      // Block instances

      content = content.concat(getInstances(name, project.design.graph));

      // Restore original graph
      // delete temporal wires
      //

      wtemp = [];
      var wn = 0;
      for (wi = 0, wn = graph.wiresVirtual.length; wi < wn; wi++) {
        if (
          typeof graph.wiresVirtual[wi].tcTodelete !== 'undefined' &&
          graph.wiresVirtual[wi].tcTodelete === true
        ) {
          //Nothing for now, only remove
        } else {
          wtemp.push(graph.wiresVirtual[wi]);
        }
      }

      for (wi = 0, wn = graph.wires.length; wi < wn; wi++) {
        if (
          typeof graph.wires[wi].tcTodelete !== 'undefined' &&
          graph.wires[wi].tcTodelete === true
        ) {
          //Nothing for now, only remove
        } else {
          wtemp.push(graph.wires[wi]);
        }
      }

      graph.wires = wtemp;

      delete graph.wiresVirtual;
      //END ONWORK

      return content.join('\n');
    }

    function getInstances(name, graph) {
      var w, wire;
      var instances = [];
      var blockArray = graph.blocks;

      for (var b in blockArray) {
        var block = blockArray[b];

        if (
          block.type !== blocks.BASIC_INPUT &&
          block.type !== blocks.BASIC_OUTPUT &&
          block.type !== blocks.BASIC_CONSTANT &&
          block.type !== blocks.BASIC_MEMORY &&
          block.type !== blocks.BASIC_INFO &&
          block.type !== blocks.BASIC_INPUT_LABEL &&
          block.type !== blocks.BASIC_OUTPUT_LABEL
        ) {
          // Header
          var instance;
          if (block.type === blocks.BASIC_CODE) {
            instance = name + '_' + utils.digestId(block.id);
          } else {
            let prefix = currentLibrary[block.type].package.name ?? '';
            prefix = utils.normalizeVerilogName(prefix);
            if (prefix.length > 0) {
              prefix += '__';
            }
            instance = `${prefix}${utils.digestId(block.type)}`;

            // instance = utils.digestId(block.type);
          }

          //-- Parameters

          var params = [];
          for (w in graph.wires) {
            wire = graph.wires[w];
            if (
              block.id === wire.target.block &&
              (wire.source.port === 'constant-out' ||
                wire.source.port === 'memory-out')
            ) {
              var paramName = wire.target.port;
              if (block.type !== blocks.BASIC_CODE) {
                paramName = utils.digestId(paramName);
              }
              paramName =
                paramName.charAt(0) === '@' ? paramName.substr(1) : paramName;
              var param = '';
              param += ' .' + paramName;
              param += '(p' + w + ')';
              params.push(param);
            }
          }

          if (params.length > 0) {
            instance += ' #(\n' + params.join(',\n') + '\n)';
          }

          //-- Instance name

          instance += ' ' + utils.digestId(block.id);

          //-- Ports

          var ports = [];
          var portsNames = [];
          for (w in graph.wires) {
            wire = graph.wires[w];
            if (block.id === wire.source.block) {
              connectPort(wire.source.port, portsNames, ports, block);
            }
            if (block.id === wire.target.block) {
              if (
                wire.source.port !== 'constant-out' &&
                wire.source.port !== 'memory-out'
              ) {
                connectPort(wire.target.port, portsNames, ports, block);
              }
            }
          }

          instance += ' (\n' + ports.join(',\n') + '\n);';

          if (instance) {
            instances.push(instance);
          }
        }
      }

      function connectPort(portName, portsNames, ports, block) {
        if (portName) {
          if (block.type !== blocks.BASIC_CODE) {
            portName = utils.digestId(portName);
          }
          if (portsNames.indexOf(portName) === -1) {
            portsNames.push(portName);
            portName =
              portName.charAt(0) === '@' ? portName.substr(1) : portName;
            var port = '';
            port += ' .' + portName;
            port += '(w' + w + ')';
            ports.push(port);
          }
        }
      }

      return instances;
    }

    function findBlock(id, graph) {
      for (var b in graph.blocks) {
        if (graph.blocks[b].id === id) {
          return graph.blocks[b];
        }
      }
      return null;
    }

    this.getInitPorts = getInitPorts;

    function getInitPorts(project) {
      // Find not connected input wire ports to initialize

      var i, j;
      var initPorts = [];
      var blockArray = project.design.graph.blocks;

      // Find all not connected input ports:
      // - Code blocks
      // - Generic blocks
      for (i in blockArray) {
        var block = blockArray[i];
        if (block) {
          if (
            block.type === blocks.BASIC_CODE ||
            !block.type.startsWith('basic.')
          ) {
            // Code block or Generic block
            for (j in block.data.ports.in) {
              var inPort = block.data.ports.in[j];
              if (inPort.default && inPort.default.apply) {
                initPorts.push({
                  block: block.id,
                  port: inPort.name,
                  name: inPort.default.port,
                  pin: inPort.default.pin,
                });
              }
            }
          }
        }
      }

      return initPorts;
    }

    this.getInitPins = getInitPins;

    function getInitPins(project) {
      // Find not used output pins to initialize

      var i;
      var initPins = [];
      var usedPins = [];
      var blockArray = project.design.graph.blocks;

      // Find all set output pins
      for (i in blockArray) {
        var block = blockArray[i];
        if (block.type === blocks.BASIC_OUTPUT) {
          for (var p in block.data.pins) {
            usedPins.push(block.data.virtual ? '' : block.data.pins[p].value);
          }
        }
      }

      // Filter pins defined in rules
      var allInitPins = common.selectedBoard.rules.output;
      for (i in allInitPins) {
        if (usedPins.indexOf(allInitPins[i].pin) === -1) {
          initPins.push(allInitPins[i]);
        }
      }

      return initPins;
    }

    //-------------------------------------------------------------------
    //-- Generate the verilog code for the given circuit
    //-- name: String. Name of the current module (top module: 'main')
    //-- project: Project module
    //-- opt: Options
    //--------------------------------------------------------------------
    function verilogCompiler(name, project, opt) {
      var i,
        data,
        block,
        code = '';
      opt = opt || {};
      if (project && project.design && project.design.graph) {
        var blockArray = project.design.graph.blocks;
        var dependencies = project.dependencies;

        // Main module

        if (name) {
          // Initialize input ports

          if (name === 'main' && opt.boardRules) {
            var initPorts = opt.initPorts || getInitPorts(project);
            for (i in initPorts) {
              var initPort = initPorts[i];

              // Find existing input block with the initPort value
              var found = false;
              var source = {
                block: initPort.name,
                port: 'out',
              };
              for (i in blockArray) {
                block = blockArray[i];
                if (
                  block.type === blocks.BASIC_INPUT &&
                  !block.data.range &&
                  !block.data.virtual &&
                  initPort.pin === block.data.pins[0].value
                ) {
                  found = true;
                  source.block = block.id;
                  break;
                }
              }

              if (!found) {
                // Add imaginary input block with the initPort value
                project.design.graph.blocks.push({
                  id: initPort.name,
                  type: blocks.BASIC_INPUT,
                  data: {
                    name: initPort.name,
                    pins: [
                      {
                        index: '0',
                        value: initPort.pin,
                      },
                    ],
                    virtual: false,
                  },
                });
              }

              // Add imaginary wire between the input block and the initPort
              project.design.graph.wires.push({
                source: {
                  block: source.block,
                  port: source.port,
                },
                target: {
                  block: initPort.block,
                  port: initPort.port,
                },
              });
            }
          }

          var params = getParams(project);

          //-- Future improvement: After reading the ports, get the names defined in the .pcf or .lpf file
          //-- these are better names to use in the top module, instead of the current not-for-humans pin names
          var ports = getPorts(project);

          var content = getContent(name, project);

          // Initialize output pins

          if (name === 'main' && opt.boardRules) {
            var initPins = opt.initPins || getInitPins(project);
            var n = initPins.length;

            if (n > 0) {
              // Declare m port
              ports.out.push({
                name: 'vinit',
                //range: [0:' + (n - 1) + ']'
                range: `[${n - 1}:0 ]`,
              });
              // Generate port value
              var value = n.toString() + "'b";
              for (i in initPins) {
                value += initPins[i].bit;
              }
              // Assign m port
              content += '\nassign vinit = ' + value + ';';
            }
          }

          data = {
            name: name,
            params: params,
            ports: ports,
            content: content,
          };

          code += '//---- Top entity';
          code += module(data);
        }

        // Dependencies modules
        //-- Generate the comments header for the module
        if (typeof project.package !== 'undefined') {
          //-- Separation from the previous verilog block
          code += '\n';

          //-- It is only generate if the project/block has a name
          //-- Usually the top entity do not have a main
          if (project.package.name) {
            code += '//---------------------------------------------------\n';
            code += '//-- ' + project.package.name + '\n';
            code += '//-- - - - - - - - - - - - - - - - - - - - - - - - --\n';
            code += '//-- ' + project.package.description + '\n';
            code += '//---------------------------------------------------\n';
          }
        }

        let prefix = '';
        for (var d in dependencies) {
          prefix = dependencies[d].package.name ?? 'pkg';
          prefix = utils.normalizeVerilogName(prefix);
          code += verilogCompiler(
            `${prefix}__${utils.digestId(d)}`,
            dependencies[d]
          );
        }

        // Code modules

        for (i in blockArray) {
          block = blockArray[i];
          if (block) {
            if (block.type === blocks.BASIC_CODE) {
              data = {
                name: name + '_' + utils.digestId(block.id),
                params: block.data.params,
                ports: block.data.ports,
                content: block.data.code, //.replace(/\n+/g, '\n').replace(/\n$/g, '')
              };
              code += module(data);
            }
          }
        }
      }

      return code;
    }

    function pcfCompiler(project, opt) {
      var i,
        j,
        block,
        pin,
        value,
        code = '';
      var blockArray = project.design.graph.blocks;
      opt = opt || {};

      for (i in blockArray) {
        block = blockArray[i];
        if (
          block.type === blocks.BASIC_INPUT ||
          block.type === blocks.BASIC_OUTPUT
        ) {
          if (block.data.pins.length > 1) {
            for (var p in block.data.pins) {
              pin = block.data.pins[p];
              value = block.data.virtual ? '' : pin.value;
              code += 'set_io ';
              code += utils.digestId(block.id);
              code += '[' + pin.index + '] ';
              code += value;
              code += '\n';
            }
          } else if (block.data.pins.length > 0) {
            pin = block.data.pins[0];
            value = block.data.virtual ? '' : pin.value;
            code += 'set_io ';
            code += utils.digestId(block.id);
            code += ' ';
            code += value;
            code += '\n';
          }
        }
      }

      if (opt.boardRules) {
        // Declare init input ports

        var used = [];
        var initPorts = opt.initPorts || getInitPorts(project);
        let pname = '';
        for (i in initPorts) {
          var initPort = initPorts[i];
          if (used.indexOf(initPort.pin) !== -1) {
            break;
          }
          used.push(initPort.pin);

          // Find existing input block with the initPort value
          var found = false;
          for (j in blockArray) {
            block = blockArray[j];
            if (
              block.type === blocks.BASIC_INPUT &&
              !block.data.range &&
              !block.data.virtual &&
              initPort.pin === block.data.pins[0].value
            ) {
              found = true;
              used.push(initPort.pin);
              break;
            }
          }

          pname =
            initPorts[i].name.charAt(0) === '@'
              ? initPorts[i].name.substr(1)
              : initPorts[i].name;
          if (!found) {
            code += 'set_io v';
            code += pname;
            code += ' ';
            code += initPorts[i].pin;
            code += '\n';
          }
        }

        // Declare init output pins

        var initPins = opt.initPins || getInitPins(project);
        if (initPins.length > 1) {
          for (i in initPins) {
            code += 'set_io vinit[' + i + '] ';
            code += initPins[i].pin;
            code += '\n';
          }
        } else if (initPins.length > 0) {
          code += 'set_io vinit ';
          code += initPins[0].pin;
          code += '\n';
        }
      }

      return code;
    }

    function lpfCompiler(project, opt) {
      var i,
        block,
        pin,
        value,
        code = '';
      var blockArray = project.design.graph.blocks;
      opt = opt || {};

      code += '# -- Board: ';
      code += common.selectedBoard.name;
      code += '\n\n';

      for (i in blockArray) {
        block = blockArray[i];
        if (
          block.type === blocks.BASIC_INPUT ||
          block.type === blocks.BASIC_OUTPUT
        ) {
          //-- Future improvement: Both cases: 1-pin or multiple pins in an array
          //-- could be refactorized instead of repeating code
          //-- (i usually name this as plural and singular cases)

          if (block.data.pins.length > 1) {
            for (var p in block.data.pins) {
              pin = block.data.pins[p];
              value = block.data.virtual ? '' : pin.value;
              code += 'LOCATE COMP "';
              code += utils.digestId(block.id); //-- Future improvement: use pin.name. It should also be changed in the main module
              code += '[' + pin.index + ']" SITE "';
              code += value;
              code += '";\n';

              code += 'IOBUF  PORT "';
              code += utils.digestId(block.id);
              code += '[' + pin.index + ']" ';

              //-- Get the pullmode property of the physical pin (its id is pin.value)
              let pullmode = common.selectedBoard.pinout.find(
                (x) => x.value === value
              ).pullmode;
              if (
                pullmode === 'UP' ||
                pullmode === 'DOWN' ||
                pullmode === 'NONE'
              ) {
                code += 'PULLMODE=' + pullmode;
              }
              code += ' ;\n\n';
            }
          } else if (block.data.pins.length > 0) {
            pin = block.data.pins[0];
            value = block.data.virtual ? '' : pin.value;
            code += 'LOCATE COMP "';
            code += utils.digestId(block.id); //-- Future improvement: use pin.name. It should also be changed in the main module
            code += '" SITE "';
            code += value;
            code += '";\n';

            code += 'IOBUF  PORT "';
            code += utils.digestId(block.id);
            code += '" ';

            //-- Get the pullmode property of the physical pin (its id is pin.value)
            let pullmode = common.selectedBoard.pinout.find(
              (x) => x.value === value
            ).pullmode;
            if (
              pullmode === 'UP' ||
              pullmode === 'DOWN' ||
              pullmode === 'NONE'
            ) {
              code += 'PULLMODE=' + pullmode;
            }
            code += ' ;\n\n';
          }
        }
      }

      return code;
    }

    function listCompiler(project) {
      var i;
      var listFiles = [];

      if (project && project.design && project.design.graph) {
        var blockArray = project.design.graph.blocks;
        var dependencies = project.dependencies;

        // Find in blocks

        for (i in blockArray) {
          var block = blockArray[i];
          if (block.type === blocks.BASIC_MEMORY) {
            listFiles.push({
              name: utils.digestId(block.id) + '.list',
              content: block.data.list,
            });
          }
        }

        // Find in dependencies

        for (i in dependencies) {
          var dependency = dependencies[i];
          listFiles = listFiles.concat(listCompiler(dependency));
        }
      }

      return listFiles;
    }

    function testbenchCompiler(project) {
      var i, o, p;
      var code = '';

      code += '// Testbench template\n\n';

      code += '`default_nettype none\n';
      code += '`define DUMPSTR(x) `"x.vcd`"\n';
      code += '`timescale 10 ns / 1 ns\n\n';

      var ports = {
        in: [],
        out: [],
      };
      var content = '\n';

      content += '// Simulation time: 100ns (10 * 10ns)\n';
      content += 'parameter DURATION = 10;\n';

      // Parameters
      var _params = [];
      var params = mainParams(project);
      if (params.length > 0) {
        content += '\n// TODO: edit the module parameters here\n';
        let pname = '';
        content += '// e.g. localparam constant_value = 1;\n';
        for (p in params) {
          pname =
            params[p].name.charAt(0) === '@'
              ? params[p].name.substr(1)
              : params[p].name;
          content += 'localparam ' + pname + ' = ' + params[p].value + ';\n';
          _params.push(' .' + params[p].id + '(' + pname + ')');
        }
      }

      // Input/Output
      var io = mainIO(project);
      var input = io.input;
      var output = io.output;
      content += '\n// Input/Output\n';
      var _ports = [];
      let signed = '';
      for (i in input) {
        signed = '';
        pname = input[i].name;
        if (input[i].name.charAt(0) === '@') {
          pname = input[i].name.substr(1);
          signed = ' signed ';
        }
        content +=
          'reg ' +
          signed +
          (input[i].range ? input[i].range + ' ' : '') +
          pname +
          ';\n';
        _ports.push(' .' + input[i].id + '(' + pname + ')');
      }
      for (o in output) {
        signed = '';
        pname = output[o].name;
        if (output[o].name.charAt(0) === '@') {
          pname = output[o].name.substr(1);
          signed = ' signed ';
        }
        content +=
          'wire ' +
          signed +
          (output[o].range ? output[o].range + ' ' : '') +
          pname +
          ';\n';
        _ports.push(' .' + output[o].id + '(' + pname + ')');
      }

      // Module instance
      content += '\n// Module instance\n';
      content += 'main';

      //-- Parameters
      if (_params.length > 0) {
        content += ' #(\n';
        content += _params.join(',\n');
        content += '\n)';
      }

      content += ' MAIN';

      //-- Ports
      if (_ports.length > 0) {
        content += ' (\n';
        content += _ports.join(',\n');
        content += '\n)';
      }

      content += ';\n';

      // Clock signal
      var hasClk = false;
      for (i in input) {
        if (input[i].name.toLowerCase() === 'clk') {
          hasClk = true;
          break;
        }
      }
      if (hasClk) {
        content += '\n// Clock signal\n';
        content += 'always #0.5 clk = ~clk;\n';
      }

      content += '\ninitial begin\n';
      content += ' // File were to store the simulation results\n';
      content += ' $dumpfile(`DUMPSTR(`VCD_OUTPUT));\n';
      content += ' $dumpvars(0, main_tb);\n\n';
      content += ' // TODO: initialize the registers here\n';
      content += ' // e.g. value = 1;\n';
      content += ' // e.g. #2 value = 0;\n';
      var pname;
      for (i in input) {
        pname =
          input[i].name.charAt(0) === '@'
            ? input[i].name.substr(1)
            : input[i].name;
        content += ' ' + pname + ' = 0;\n';
      }
      content += '\n';
      content += ' #(DURATION) $display("End of simulation");\n';
      content += ' $finish;\n';
      content += 'end\n';

      var data = {
        name: 'main_tb',
        ports: ports,
        content: content,
      };
      code += module(data);

      return code;
    }

    function gtkwaveCompiler(project) {
      var code = '';

      var io = mainIO(project);
      var input = io.input;
      var output = io.output;
      let pname;
      for (var i in input) {
        pname =
          input[i].name.charAt(0) === '@'
            ? input[i].name.substr(1)
            : input[i].name;
        code +=
          'main_tb.' + pname + (input[i].range ? input[i].range : '') + '\n';
      }
      for (var o in output) {
        pname =
          output[o].name.charAt(0) === '@'
            ? output[o].name.substr(1)
            : output[o].name;
        code +=
          'main_tb.' + pname + (output[o].range ? output[o].range : '') + '\n';
      }

      return code;
    }

    function mainIO(project) {
      var input = [];
      var output = [];
      var inputUnnamed = 0;
      var outputUnnamed = 0;
      var graph = project.design.graph;
      let pname = '';
      for (var i in graph.blocks) {
        var block = graph.blocks[i];
        if (block.type === blocks.BASIC_INPUT) {
          if (block.data.name) {
            pname = block.data.name.replace('@', '');
            input.push({
              id: utils.digestId(block.id),
              name: pname.replace(/ /g, '_'),
              range: block.data.range,
            });
          } else {
            input.push({
              id: utils.digestId(block.id),
              name: inputUnnamed.toString(),
            });
            inputUnnamed += 1;
          }
        } else if (block.type === blocks.BASIC_OUTPUT) {
          if (block.data.name) {
            pname = block.data.name.replace('@', '');
            output.push({
              id: utils.digestId(block.id),
              name: pname.replace(/ /g, '_'),
              range: block.data.range,
            });
          } else {
            output.push({
              id: utils.digestId(block.id),
              name: outputUnnamed.toString(),
            });
            outputUnnamed += 1;
          }
        }
      }

      return {
        input: input,
        output: output,
      };
    }

    function mainParams(project) {
      var params = [];
      var paramsUnnamed = 0;
      var graph = project.design.graph;
      let pname = '';
      for (var i in graph.blocks) {
        var block = graph.blocks[i];
        if (block.type === blocks.BASIC_CONSTANT) {
          if (!block.data.local) {
            if (block.data.name) {
              pname = block.data.name.replace('@', '');
              params.push({
                id: utils.digestId(block.id),
                name: 'constant_' + pname.replace(/ /g, '_'),
                value: block.data.value,
              });
            } else {
              params.push({
                id: utils.digestId(block.id),
                name: 'constant_' + paramsUnnamed.toString(),
                value: block.data.value,
              });
              paramsUnnamed += 1;
            }
          }
        }
      }

      return params;
    }
  });
