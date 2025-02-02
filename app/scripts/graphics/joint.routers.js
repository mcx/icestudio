joint.routers.ice = (function (g, _, joint) {
  'use strict';

  var config = {
    step: 4,
    perpendicular: true,
    excludeEnds: [],
    excludeTypes: ['ice.Info'],
    maximumLoops: 300,
    startDirections: ['right', 'bottom'],
    endDirections: ['left', 'top'],
    directionMap: {
      right: { x: 1, y: 0 },
      bottom: { x: 0, y: 1 },
      left: { x: -1, y: 0 },
      top: { x: 0, y: -1 },
    },
    maxAllowedDirectionChange: 90,
    paddingBox: function () {
      var step = 2;
      return {
        x: -step,
        y: -step,
        width: 2 * step,
        height: 2 * step,
      };
    },
    directions: function () {
      var step = this.step;
      return [
        { offsetX: step, offsetY: 0, cost: step },
        { offsetX: 0, offsetY: step, cost: step },
        { offsetX: -step, offsetY: 0, cost: step },
        { offsetX: 0, offsetY: -step, cost: step },
      ];
    },
    penalties: function () {
      return {
        0: 0,
        45: this.step / 2,
        90: this.step / 2,
      };
    },
    fallbackRoute: _.constant(null),
    draggingRoute: null,
  };

  function ObstacleMap(opt, paper) {
    this.map = {};
    this.options = opt;
    this.paper = paper;
    this.mapGridSize = 100;
  }

  ObstacleMap.prototype.build = function (graph, link) {
    var opt = this.options;

    const excludedIds = new Set(
      opt.excludeEnds
        .map((end) => link.get(end))
        .map((end) => end.id)
        .map((id) => graph.getCell(id).id)
    );

    const excludedTypes = new Set(opt.excludeTypes);

    const excludedAncestors = new Set();

    const source = graph.getCell(link.get('source').id);
    if (source) {
      source.getAncestors().forEach((ancestor) => excludedAncestors.add(ancestor.id));
    }

    const target = graph.getCell(link.get('target').id);
    if (target) {
      target.getAncestors().forEach((ancestor) => excludedAncestors.add(ancestor.id));
    }

    const filteredElements = graph.getElements().filter((element) => {
      return (
        !excludedIds.has(element.id) &&
        !excludedTypes.has(element.get('type')) &&
        !excludedAncestors.has(element.id)
      );
    });

    const blockRectangles = filteredElements.map((element) => element.getBBox());

    const state = this.paper.options.getState();
    const plabels = document.querySelectorAll('.port-label');
    const labelRectangles = Array.from(plabels).map((label) => {
      const rect = V(label).bbox();
      return g.rect({
        x: (rect.x - state.pan.x) / state.zoom,
        y: (rect.y - state.pan.y) / state.zoom,
        width: rect.width / state.zoom,
        height: rect.height / state.zoom,
      });
    });

    //-- Building the obstacle map
    const mapGridSize = this.mapGridSize;
    const allRectangles = blockRectangles.concat(labelRectangles);

    allRectangles.forEach((bbox) => {
        const padding = opt.paddingBox();
        const paddedBbox = bbox.moveAndExpand(padding); 
        const origin = paddedBbox.origin().snapToGrid(mapGridSize);
        const corner = paddedBbox.corner().snapToGrid(mapGridSize);

        for (let x = origin.x; x <= corner.x; x += mapGridSize) {
          for (let y = origin.y; y <= corner.y; y += mapGridSize) {
            const gridKey = `${x}@${y}`;
            this.map[gridKey] = this.map[gridKey] || [];
            this.map[gridKey].push(paddedBbox);
          }
        }
    });

    return this;
  };


ObstacleMap.prototype.isPointAccessible = function (point) {
    var mapKey = point.clone().snapToGrid(this.mapGridSize).toString();
    return _.every(this.map[mapKey], function (obstacle) {
      return !obstacle.containsPoint(point);
    });
  };

  function SortedSet() {
    this.items = [];
    this.hash = {};
    this.values = {};
    this.OPEN = 1;
    this.CLOSE = 2;
  }

  SortedSet.prototype.add = function (item, value) {
    if (this.hash[item]) {
      this.items.splice(this.items.indexOf(item), 1);
    } else {
      this.hash[item] = this.OPEN;
    }

    this.values[item] = value;

    var index = _.sortedIndex(this.items, item, function (i) {
      return this.values[i];
    }, this);

    this.items.splice(index, 0, item);
  };

  SortedSet.prototype.remove = function (item) {
    this.hash[item] = this.CLOSE;
  };

  SortedSet.prototype.isOpen = function (item) {
    return this.hash[item] === this.OPEN;
  };

  SortedSet.prototype.isClose = function (item) {
    return this.hash[item] === this.CLOSE;
  };

  SortedSet.prototype.isEmpty = function () {
    return this.items.length === 0;
  };

  SortedSet.prototype.pop = function () {
    var item = this.items.shift();
    this.remove(item);
    return item;
  };

  function normalizePoint(point) {
    return g.point(
      point.x === 0 ? 0 : Math.abs(point.x) / point.x,
      point.y === 0 ? 0 : Math.abs(point.y) / point.y
    );
  }

  function reconstructRoute(parents, point, startCenter, endCenter) {
    var route = [];
    var prevDiff = normalizePoint(endCenter.difference(point));
    var current = point;
    var parent;

    while ((parent = parents[current])) {
      var diff = normalizePoint(current.difference(parent));

      if (!diff.equals(prevDiff)) {
        route.unshift(current);
        prevDiff = diff;
      }

      current = parent;
    }

    var startDiff = normalizePoint(g.point(current).difference(startCenter));

    if (!startDiff.equals(prevDiff)) {
      route.unshift(current);
    }

    return route;
  }

  function getRectPoints(bbox, directionList, opt) {
    var step = opt.step;
    var center = bbox.center();
    var startPoints = _.chain(opt.directionMap)
      .pick(directionList)
      .map(function (direction) {
        var x = (direction.x * bbox.width) / 2;
        var y = (direction.y * bbox.height) / 2;

        var point = center.clone().offset(x, y);

        if (bbox.containsPoint(point)) {
          point.offset(direction.x * step, direction.y * step);
        }

        return point.snapToGrid(step);
      })
      .value();

    return startPoints;
  }

  function getDirectionAngle(start, end, dirLen) {
    var q = 360 / dirLen;
    return Math.floor(g.normalizeAngle(start.theta(end) + q / 2) / q) * q;
  }

  function getDirectionChange(angle1, angle2) {
    var dirChange = Math.abs(angle1 - angle2);
    return dirChange > 180 ? 360 - dirChange : dirChange;
  }

  function estimateCost(from, endPoints) {
    var min = Infinity;

    for (var i = 0, len = endPoints.length; i < len; i++) {
      var cost = from.manhattanDistance(endPoints[i]);

      if (cost < min) {
        min = cost;
      }
    }

    return min;
  }

  function findRoute(start, end, map, opt) {
    var step = opt.step;
    var startPoints, endPoints;
    var startCenter, endCenter;

    if (start instanceof g.rect) {
      startPoints = getRectPoints(start, opt.startDirections, opt);
      startCenter = start.center().snapToGrid(step);
    } else {
      startCenter = start.clone().snapToGrid(step);
      startPoints = [startCenter];
    }

    if (end instanceof g.rect) {
      endPoints = getRectPoints(end, opt.endDirections, opt);
      endCenter = end.center().snapToGrid(step);
    } else {
      endCenter = end.clone().snapToGrid(step);
      endPoints = [endCenter];
    }

    startPoints = _.filter(startPoints, map.isPointAccessible, map);
    endPoints = _.filter(endPoints, map.isPointAccessible, map);

    if (startPoints.length > 0 && endPoints.length > 0) {
      var openSet = new SortedSet();
      var parents = {};
      var costs = {};

      _.each(startPoints, function (point) {
        var key = point.toString();
        openSet.add(key, estimateCost(point, endPoints));
        costs[key] = 0;
      });

      var dir, dirChange;
      var dirs = opt.directions;
      var dirLen = dirs.length;
      var loopsRemain = opt.maximumLoops;
      var endPointsKeys = _.invoke(endPoints, 'toString');

      var currentDirAngle;
      var previousDirAngle;

      while (!openSet.isEmpty() && loopsRemain > 0) {
        var currentKey = openSet.pop();
        var currentPoint = g.point(currentKey);
        var currentDist = costs[currentKey];
        previousDirAngle = currentDirAngle;
        currentDirAngle = parents[currentKey]
          ? getDirectionAngle(parents[currentKey], currentPoint, dirLen)
          : opt.previousDirAngle !== null
            ? opt.previousDirAngle
            : getDirectionAngle(startCenter, currentPoint, dirLen);

        if (endPointsKeys.indexOf(currentKey) >= 0) {
          dirChange = getDirectionChange(
            currentDirAngle,
            getDirectionAngle(currentPoint, endCenter, dirLen)
          );

          if (currentPoint.equals(endCenter) || dirChange < 180) {
            opt.previousDirAngle = currentDirAngle;

            return reconstructRoute(
              parents,
              currentPoint,
              startCenter,
              endCenter
            );
          }
        }

        for (var i = 0; i < dirLen; i++) {
          dir = dirs[i];
          dirChange = getDirectionChange(currentDirAngle, dir.angle);

          if (previousDirAngle && dirChange > opt.maxAllowedDirectionChange) {
            continue;
          }

          var neighborPoint = currentPoint
            .clone()
            .offset(dir.offsetX, dir.offsetY);
          var neighborKey = neighborPoint.toString();

          if (
            openSet.isClose(neighborKey) ||
            !map.isPointAccessible(neighborPoint)
          ) {
            continue;
          }

          var costFromStart = currentDist + dir.cost + opt.penalties[dirChange];

          if (
            !openSet.isOpen(neighborKey) ||
            costFromStart < costs[neighborKey]
          ) {
            parents[neighborKey] = currentPoint;
            costs[neighborKey] = costFromStart;
            openSet.add(
              neighborKey,
              costFromStart + estimateCost(neighborPoint, endPoints)
            );
          }
        }

        loopsRemain--;
      }
    }

    return opt.fallbackRoute(startCenter, endCenter, opt);
  }

  function resolveOptions(opt) {
    opt.directions = typeof opt.directions === 'function' ? opt.directions() : opt.directions;
    opt.penalties = typeof opt.penalties === 'function' ? opt.penalties() : opt.penalties;
    opt.paddingBox = typeof opt.paddingBox === 'function' ? opt.paddingBox : function() {
        var step = 2;
        return {
            x: -step,
            y: -step,
            width: 2 * step,
            height: 2 * step,
        };
    };

    for (var i = 0, no = opt.directions.length; i < no; i++) {
        var point1 = g.point(0, 0);
        var point2 = g.point(
            opt.directions[i].offsetX,
            opt.directions[i].offsetY
        );

        opt.directions[i].angle = g.normalizeAngle(point1.theta(point2));
    }
}
function router(vertices, opt, linkView) {
  resolveOptions(opt);

  //-- Important bug fixed, maintain this comment for some time to 
  //-- remember that. I'm removing "this" because we use "strict" and "this"
  //-- in this context could be "undefined"
  //--
  linkView.options.perpendicular = !!opt.perpendicular;

  linkView.sourceBBox.x += linkView.sourceBBox.width / 2;
  linkView.sourceBBox.y += linkView.sourceBBox.height / 2;
  linkView.sourceBBox.width = 0;
  linkView.sourceBBox.height = 0;

  linkView.targetBBox.x += linkView.targetBBox.width / 2;
  linkView.targetBBox.y += linkView.targetBBox.height / 2;
  linkView.targetBBox.width = 0;
  linkView.targetBBox.height = 0;

  var sourceBBox = g.rect(linkView.sourceBBox);
  var targetBBox = g.rect(linkView.targetBBox);

  var map = new ObstacleMap(opt, linkView.paper).build(
    linkView.paper.model,
    linkView.model
  );
  var oldVertices = _.map(vertices, g.point);
  var newVertices = [];
  var tailPoint = sourceBBox.center().snapToGrid(opt.step);

  var from;
  var to;

  for (var i = 0, len = oldVertices.length; i <= len; i++) {
    var partialRoute = null;

    from = to || sourceBBox;
    to = oldVertices[i];

    if (!to) {
      to = targetBBox;

      var endingAtPoint =
        !linkView.model.get('source').id || !linkView.model.get('target').id;

      if (endingAtPoint && _.isFunction(opt.draggingRoute)) {
        var dragFrom = from instanceof g.rect ? from.center() : from;
        partialRoute = opt.draggingRoute(dragFrom, to.origin(), opt);
      }
    }

    partialRoute = partialRoute || findRoute(from, to, map, opt);

    if (partialRoute === null) {
      if (!_.isFunction(joint.routers.orthogonal)) {
        throw new Error('Manhattan requires the orthogonal router.');
      }

      return joint.routers.orthogonal(vertices, opt, linkView);
    }

    var leadPoint = _.first(partialRoute);

    if (leadPoint && leadPoint.equals(tailPoint)) {
      partialRoute.shift();
    }

    tailPoint = _.last(partialRoute) || tailPoint;

    Array.prototype.push.apply(newVertices, partialRoute);
  }

  return newVertices;
}

return function (vertices, opt, linkView) {
  if (linkView.sourceMagnet) {
    opt.startDirections = [linkView.sourceMagnet.attributes.pos.value];
  }

  if (linkView.targetMagnet) {
    opt.endDirections = [linkView.targetMagnet.attributes.pos.value];
  }

  return router(vertices, _.extend({}, config, opt), linkView);
};


})(g, _, joint);
