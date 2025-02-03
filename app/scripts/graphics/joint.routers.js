  
joint.routers.ice = (function (g, _, joint) {

  'use strict';

  var config = {

    // size of the step to find a route
step: 8,

    // use of the perpendicular linkView option to connect center of element with first vertex
    perpendicular: true,

    // should be source or target not to be consider as an obstacle
    excludeEnds: [], // 'source', 'target'

    // should be any element with a certain type not to be consider as an obstacle
    excludeTypes: ['ice.Info'],

    // if number of route finding loops exceed the maximum, stops searching and returns
    // fallback route
    maximumLoops: 820,  //2000

    // possible starting directions from an element
    startDirections: ['right', 'bottom'],

    // possible ending directions to an element
    endDirections: ['left', 'top'],

    // specify directions above
    directionMap: {
      right: { x: 1, y: 0 },
      bottom: { x: 0, y: 1 },
      left: { x: -1, y: 0 },
      top: { x: 0, y: -1 }
    },

    // maximum change of the direction
    maxAllowedDirectionChange: 90,

    // padding applied on the element bounding boxes
    paddingBox: function () {

      var step = 2;

      return {
        x: -step,
        y: -step,
        width: 2 * step,
        height: 2 * step
      };
    },

    // an array of directions to find next points on the route
    directions: function () {

      var step = this.step;

      return [
        { offsetX: step, offsetY: 0, cost: step },
        { offsetX: 0, offsetY: step, cost: step },
        { offsetX: -step, offsetY: 0, cost: step },
        { offsetX: 0, offsetY: -step, cost: step }
      ];
    },

    // a penalty received for direction change
    penalties: function () {

      return {
        0: 0,
        45: this.step / 2,
        90: this.step / 2
      };
    },

    // * Deprecated *
    // a simple route used in situations, when main routing method fails
    // (exceed loops, inaccessible).
    // i.e.
    //  function(from, to, opts) {
    //    // Find an orthogonal route ignoring obstacles.
    //    var point = ((opts.previousDirAngle || 0) % 180 === 0)
    //            ? g.point(from.x, to.y)
    //            : g.point(to.x, from.y);
    //    return [point, to];
    //  },

    fallbackRoute: _.constant(null),

    // if a function is provided, it's used to route the link while dragging an end
    // i.e. function(from, to, opts) { return []; }
    draggingRoute: null
  };

  // Map of obstacles
  // Helper structure to identify whether a point lies in an obstacle.
  function ObstacleMap(opt, paper) {

    this.map = {};
    this.options = opt;
    this.paper = paper;
    // tells how to divide the paper when creating the elements map
    this.mapGridSize = 100;
  }

  ObstacleMap.prototype.build = function (graph, link) {

    var opt = this.options;

    // source or target element could be excluded from set of obstacles
    var excludedEnds = _.chain(opt.excludeEnds)
      .map(link.get, link)
      .pluck('id')
      .map(graph.getCell, graph).value();

    // Exclude any embedded elements from the source and the target element.
    var excludedAncestors = [];

    var source = graph.getCell(link.get('source').id);
    if (source) {
      excludedAncestors = _.union(excludedAncestors, _.map(source.getAncestors(), 'id'));
    }

    var target = graph.getCell(link.get('target').id);
    if (target) {
      excludedAncestors = _.union(excludedAncestors, _.map(target.getAncestors(), 'id'));
    }

    // builds a map of all elements for quicker obstacle queries (i.e. is a point contained
    // in any obstacle?) (a simplified grid search)
    // The paper is divided to smaller cells, where each of them holds an information which
    // elements belong to it. When we query whether a point is in an obstacle we don't need
    // to go through all obstacles, we check only those in a particular cell.
    var mapGridSize = this.mapGridSize;

    // Compute rectangles from all the blocks
    var blockRectangles = _.chain(graph.getElements())
      // remove source and target element if required
      .difference(excludedEnds)
      // remove all elements whose type is listed in excludedTypes array
      .reject(function (element) {
        // reject any element which is an ancestor of either source or target
        return _.contains(opt.excludeTypes, element.get('type')) || _.contains(excludedAncestors, element.id);
      })
      // change elements (models) to their bounding boxes
      .invoke('getBBox').value();

    // Compute rectangles from all the port labels
    var state = this.paper.options.getState();
   var plabels=document.querySelectorAll('.port-label');
    var labelRectangles=[];
      var rect = false;
    var i,npl;
    for(i=0,npl=plabels.length;i<npl;i++){
      rect = V(plabels[i]).bbox();
      labelRectangles.push( g.rect({
        x: (rect.x - state.pan.x) / state.zoom,
        y: (rect.y - state.pan.y) / state.zoom,
        width: rect.width / state.zoom,
        height: rect.height / state.zoom
      }));

    }
    
   // var labelRectangles = $('.port-label').map(function (index, node) {
   //   var rect = V(node).bbox();
   //   return g.rect({
   //     x: (rect.x - state.pan.x) / state.zoom,
   //     y: (rect.y - state.pan.y) / state.zoom,
   //     width: rect.width / state.zoom,
   //     height: rect.height / state.zoom
   //   });
   // }).toArray();

    var x,y,origin,corner;
    // Add all rectangles to the map's grid
    _.chain(blockRectangles.concat(labelRectangles))
      // expand their boxes by specific padding
      .invoke('moveAndExpand', opt.paddingBox)
      // build the map
      .foldl(function (map, bbox) {

        origin = bbox.origin().snapToGrid(mapGridSize);
        corner = bbox.corner().snapToGrid(mapGridSize);

        for ( x = origin.x; x <= corner.x; x += mapGridSize) {
          for (y = origin.y; y <= corner.y; y += mapGridSize) {

            var gridKey = x + '@' + y;

            map[gridKey] = map[gridKey] || [];
            map[gridKey].push(bbox);
          }
        }

        return map;

      }, this.map).value();

    return this;
  };

  ObstacleMap.prototype.isPointAccessible = function (point) {

    var mapKey = point.clone().snapToGrid(this.mapGridSize).toString();

    return _.every(this.map[mapKey], function (obstacle) {
      return !obstacle.containsPoint(point);
    });
  };

  // Sorted Set
  // Set of items sorted by given value.
  function SortedSet() {
    this.items = [];
    this.hash = {};
    this.values = {};
    this.OPEN = 1;
    this.CLOSE = 2;
  }

  SortedSet.prototype.add = function (item, value) {

    if (this.hash[item]) {
      // item removal
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

  // reconstructs a route by concating points with their parents
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

  // find points around the rectangle taking given directions in the account
  function getRectPoints(bbox, directionList, opt) {

    var step = opt.step;
    var center = bbox.center();
    var startPoints = _.chain(opt.directionMap).pick(directionList).map(function (direction) {

      var x = direction.x * bbox.width / 2;
      var y = direction.y * bbox.height / 2;

      var point = center.clone().offset(x, y);

      if (bbox.containsPoint(point)) {
        point.offset(direction.x * step, direction.y * step);
      }

      return point.snapToGrid(step);

    }).value();

    return startPoints;
  }

  // returns a direction index from start point to end point
  function getDirectionAngle(start, end, dirLen) {

    var q = 360 / dirLen;
    return Math.floor(g.normalizeAngle(start.theta(end) + q / 2) / q) * q;
  }

  function getDirectionChange(angle1, angle2) {

    var dirChange = Math.abs(angle1 - angle2);
    return dirChange > 180 ? 360 - dirChange : dirChange;
  }

  // heurestic method to determine the distance between two points
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

  // finds the route between to points/rectangles implementing A* alghoritm
  function findRoute(start, end, map, opt) {

    var step = opt.step;
    var startPoints, endPoints;
    var startCenter, endCenter;

    // set of points we start pathfinding from
    if (start instanceof g.rect) {
      startPoints = getRectPoints(start, opt.startDirections, opt);
      startCenter = start.center().snapToGrid(step);
    } else {
      startCenter = start.clone().snapToGrid(step);
      startPoints = [startCenter];
    }

    // set of points we want the pathfinding to finish at
    if (end instanceof g.rect) {
      endPoints = getRectPoints(end, opt.endDirections, opt);
      endCenter = end.center().snapToGrid(step);
    } else {
      endCenter = end.clone().snapToGrid(step);
      endPoints = [endCenter];
    }

    // take into account only accessible end points
    startPoints = _.filter(startPoints, map.isPointAccessible, map);
    endPoints = _.filter(endPoints, map.isPointAccessible, map);


    // Check if there is a accessible end point.
    // We would have to use a fallback route otherwise.
    if (startPoints.length > 0 && endPoints.length > 0) {


      // The set of tentative points to be evaluated, initially containing the start points.
      var openSet = new SortedSet();
      // Keeps reference to a point that is immediate predecessor of given element.
      var parents = {};
      // Cost from start to a point along best known path.
      var costs = {};

      _.each(startPoints, function (point) {
        var key = point.toString();
        openSet.add(key, estimateCost(point, endPoints));
        costs[key] = 0;
      });


      // directions
      var dir, dirChange;
      var dirs = opt.directions;
      var dirLen = dirs.length;
      var loopsRemain = opt.maximumLoops;
      var endPointsKeys = _.invoke(endPoints, 'toString');

      var currentDirAngle;
      var previousDirAngle;

      // main route finding loop
      while (!openSet.isEmpty() && loopsRemain > 0) {

        // remove current from the open list
        var currentKey = openSet.pop();
        var currentPoint = g.point(currentKey);
        var currentDist = costs[currentKey];
        previousDirAngle = currentDirAngle;
        // jshint -W116 
        currentDirAngle = parents[currentKey] ? getDirectionAngle(parents[currentKey], currentPoint, dirLen)
          : opt.previousDirAngle != null ? opt.previousDirAngle : getDirectionAngle(startCenter, currentPoint, dirLen);
        // jshint +W116 

        // Check if we reached any endpoint
        if (endPointsKeys.indexOf(currentKey) >= 0) {
          // We don't want to allow route to enter the end point in opposite direction.
          dirChange = getDirectionChange(currentDirAngle, getDirectionAngle(currentPoint, endCenter, dirLen));
          if (currentPoint.equals(endCenter) || dirChange < 180) {
            opt.previousDirAngle = currentDirAngle;
            return reconstructRoute(parents, currentPoint, startCenter, endCenter);
          }
        }

        // Go over all possible directions and find neighbors.
        for (var i = 0; i < dirLen; i++) {

          dir = dirs[i];
          dirChange = getDirectionChange(currentDirAngle, dir.angle);
          // if the direction changed rapidly don't use this point
          // Note that check is relevant only for points with previousDirAngle i.e.
          // any direction is allowed for starting points
          if (previousDirAngle && dirChange > opt.maxAllowedDirectionChange) {
            continue;
          }

          var neighborPoint = currentPoint.clone().offset(dir.offsetX, dir.offsetY);
          var neighborKey = neighborPoint.toString();
          // Closed points from the openSet were already evaluated.
          if (openSet.isClose(neighborKey) || !map.isPointAccessible(neighborPoint)) {
            continue;
          }

          // The current direction is ok to proccess.
          var costFromStart = currentDist + dir.cost + opt.penalties[dirChange];

          if (!openSet.isOpen(neighborKey) || costFromStart < costs[neighborKey]) {
            // neighbor point has not been processed yet or the cost of the path
            // from start is lesser than previously calcluated.
            parents[neighborKey] = currentPoint;
            costs[neighborKey] = costFromStart;
            openSet.add(neighborKey, costFromStart + estimateCost(neighborPoint, endPoints));
          }
        }

        loopsRemain--;
      }

    }

    // no route found ('to' point wasn't either accessible or finding route took
    // way to much calculations)
    return opt.fallbackRoute(startCenter, endCenter, opt);
  }

  // resolve some of the options
  function resolveOptions(opt) {

    opt.directions = _.result(opt, 'directions');
    opt.penalties = _.result(opt, 'penalties');
    opt.paddingBox = _.result(opt, 'paddingBox');

    for(var i=0,no=opt.directions.length;i<no;i++){
    //_.each(opt.directions, function (direction) {

      var point1 = g.point(0, 0);
      var point2 = g.point(opt.directions[i].offsetX, opt.directions[i].offsetY);

      opt.directions[i].angle = g.normalizeAngle(point1.theta(point2));
    }
  }

  // initiation of the route finding
  function router(vertices, opt) {

    resolveOptions(opt);

    // jshint -W040 

    // enable/disable linkView perpendicular option
    this.options.perpendicular = !!opt.perpendicular;

    // Force source/target BBoxes to be points

    this.sourceBBox.x += this.sourceBBox.width / 2;
    this.sourceBBox.y += this.sourceBBox.height / 2;
    this.sourceBBox.width = 0;
    this.sourceBBox.height = 0;

    this.targetBBox.x += this.targetBBox.width / 2;
    this.targetBBox.y += this.targetBBox.height / 2;
    this.targetBBox.width = 0;
    this.targetBBox.height = 0;

    // expand boxes by specific padding
    var sourceBBox = g.rect(this.sourceBBox);
    var targetBBox = g.rect(this.targetBBox);

    // pathfinding
    var map = (new ObstacleMap(opt, this.paper)).build(this.paper.model, this.model);
    var oldVertices = _.map(vertices, g.point);
    var newVertices = [];
    var tailPoint = sourceBBox.center().snapToGrid(opt.step);

    var from;
    var to;

    // find a route by concating all partial routes (routes need to go through the vertices)
    // startElement -> vertex[1] -> ... -> vertex[n] -> endElement
    for (var i = 0, len = oldVertices.length; i <= len; i++) {

      var partialRoute = null;

      from = to || sourceBBox;
      to = oldVertices[i];

      if (!to) {

        to = targetBBox;

        // 'to' is not a vertex. If the target is a point (i.e. it's not an element), we
        // might use dragging route instead of main routing method if that is enabled.
        var endingAtPoint = !this.model.get('source').id || !this.model.get('target').id;

        if (endingAtPoint && _.isFunction(opt.draggingRoute)) {
          // Make sure we passing points only (not rects).
          var dragFrom = from instanceof g.rect ? from.center() : from;
          partialRoute = opt.draggingRoute(dragFrom, to.origin(), opt);
        }
      }

      // if partial route has not been calculated yet use the main routing method to find one
      partialRoute = partialRoute || findRoute(from, to, map, opt);

      if (partialRoute === null) {
        // The partial route could not be found.
        // use orthogonal (do not avoid elements) route instead.
        if (!_.isFunction(joint.routers.orthogonal)) {
          throw new Error('Manhattan requires the orthogonal router.');
        }
        return joint.routers.orthogonal(vertices, opt, this);
      }

      var leadPoint = _.first(partialRoute);

      if (leadPoint && leadPoint.equals(tailPoint)) {
        // remove the first point if the previous partial route had the same point as last
        partialRoute.shift();
      }

      tailPoint = _.last(partialRoute) || tailPoint;

      Array.prototype.push.apply(newVertices, partialRoute);
    }

    // jshint +W040 

    return newVertices;
  }

  // public function
  return function (vertices, opt, linkView) {

    if (linkView.sourceMagnet) {
      opt.startDirections = [linkView.sourceMagnet.attributes.pos.value];
    }

    if (linkView.targetMagnet) {
      opt.endDirections = [linkView.targetMagnet.attributes.pos.value];
    }

    return router.call(linkView, vertices, _.extend({}, config, opt));
  };

})(g, _, joint);

/*joint.routers.ice = (function (g, _, joint) {
  'use strict';

  var config = {
    step: 8,
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
    maxAllowedDirectionChange: 180,
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
    fallbackRoute: function() { return null; },
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
        .map(end => link.get(end))
        .map(end => end.id)
        .map(id => graph.getCell(id).id)
    );

    const excludedTypes = new Set(opt.excludeTypes);

    const excludedAncestors = new Set();

    const source = graph.getCell(link.get('source').id);
    if (source) {
      source.getAncestors().forEach(ancestor => excludedAncestors.add(ancestor.id));
    }

    const target = graph.getCell(link.get('target').id);
    if (target) {
      target.getAncestors().forEach(ancestor => excludedAncestors.add(ancestor.id));
    }

    const filteredElements = graph.getElements().filter(element => {
      return (
        !excludedIds.has(element.id) &&
        !excludedTypes.has(element.get('type')) &&
        !excludedAncestors.has(element.id)
      );
    });

    const blockRectangles = filteredElements.map(element => element.getBBox());

    const state = this.paper.options.getState();
    const plabels = document.querySelectorAll('.port-label');
    const labelRectangles = Array.from(plabels).map(label => {
      const rect = V(label).bbox();
      return g.rect({
        x: (rect.x - state.pan.x) / state.zoom,
        y: (rect.y - state.pan.y) / state.zoom,
        width: rect.width / state.zoom,
        height: rect.height / state.zoom,
      });
    });

    // Building the obstacle map
    const mapGridSize = this.mapGridSize;
    const allRectangles = blockRectangles.concat(labelRectangles);

    allRectangles.forEach(bbox => {
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
  return (this.map[mapKey] || []).every(obstacle => !obstacle.containsPoint(point));
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
    // Note: sortedIndex from lodash would be hard to replace without significant change to the logic or performance
    // Here's a basic sort, but it's less efficient for large arrays
    // I'm improving it with best algorithm like min-heap, but need a lot of testing because only improve probably for
    // large sets and for medium sets could be worsk, put on TODO because for now the improvement is hight.
    this.items.push(item);
    this.items.sort((a, b) => this.values[a] - this.values[b]);
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
    var startPoints = Object.entries(opt.directionMap)
      .filter(([key]) => directionList.includes(key))
      .map(([_, direction]) => {
        var x = (direction.x * bbox.width) / 2;
        var y = (direction.y * bbox.height) / 2;

        var point = center.clone().offset(x, y);

        if (bbox.containsPoint(point)) {
          point.offset(direction.x * step, direction.y * step);
        }

        return point.snapToGrid(step);
      });

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

    startPoints = startPoints.filter(map.isPointAccessible.bind(map));
    endPoints = endPoints.filter(map.isPointAccessible.bind(map));

    if (startPoints.length > 0 && endPoints.length > 0) {
      var openSet = new SortedSet();
      var parents = {};
      var costs = {};

      for (let point of startPoints) {
        const key = point.toString();
        openSet.add(key, estimateCost(point, endPoints));
        costs[key] = 0;
      }

      var dir, dirChange;
      var dirs = opt.directions;
      var dirLen = dirs.length;
      var loopsRemain = opt.maximumLoops;
      var endPointsKeys = endPoints.map(point => point.toString());
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

        if (endPointsKeys.includes(currentKey)) {
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

    // Precompute normalized directions
    opt.directions = opt.directions.map(dir => ({ 
      ...dir, 
      norm: normalizePoint(g.point(dir.offsetX, dir.offsetY)) 
    }));

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

    // Important bug fixed, maintain this comment for some time to 
    // remember that. I'm removing "this" because we use "strict" and "this"
    // in this context could be "undefined"
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
    var oldVertices = vertices.map(g.point);
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

        if (endingAtPoint && typeof opt.draggingRoute === 'function') {
          var dragFrom = from instanceof g.rect ? from.center() : from;
          partialRoute = opt.draggingRoute(dragFrom, to.origin(), opt);
        }
      }

      partialRoute = partialRoute || findRoute(from, to, map, opt);

      if (partialRoute === null) {
        if (typeof joint.routers.orthogonal !== 'function') {
          throw new Error('Manhattan requires the orthogonal router.');
        }

        return joint.routers.orthogonal(vertices, opt, linkView);
      }

      var leadPoint = partialRoute[0];

      if (leadPoint && leadPoint.equals(tailPoint)) {
        partialRoute.shift();
      }

      tailPoint = partialRoute[partialRoute.length - 1] || tailPoint;

      newVertices.push(...partialRoute);
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

    return router(vertices, Object.assign({}, config, opt), linkView);
  };

})(g, _, joint);
*/ 



