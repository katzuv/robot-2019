var parameters = {
    imageBasePoints: [],
    basePoints: [],
    imagePathPoints: [],
    pathPoints: [],
    imageW: 654,
    imageH: 656,
    fieldH: 8.23,
    fieldW: 16.46,
    startX: 0,
    startY: 0.754126,
    robotH: 1,
    robotW: 0.86,
    rotationRadius: 0.8,
    rotationAngleStep: 6.8,
    robotCenterToSide: 0.5,
    robotCenterToBack: 0.325,
    yDirectionVarName: "Y_DIRECTION",
    isRelative: true,
    isBackwards: false
};

// First, checks if it isn't implemented yet.
if (!String.prototype.format) {
    String.prototype.format = function () {
        var args = arguments;
        return this.replace(/{(\d+)}/g, function (match, number) {
            return typeof args[number] != 'undefined'
                ? args[number]
                : match
                ;
        });
    };
}

function makeAllPointsCalculations() {
    imagePointsToBasePoints();
    basePointsToPathPoints();
    pathPointsToImagePathPoints();
}

function mToPx(point) {
    return {
        x: (point.x + (parameters.startX + (parameters.isBackwards ? parameters.robotW - parameters.robotCenterToBack : parameters.robotCenterToBack))) * (parameters.imageW / parameters.fieldW),
        y: (parameters.fieldH - point.y - (parameters.startY + parameters.robotH / 2)) * (parameters.imageH / parameters.fieldH)
    };
}

function pxToM(point) {
    return {
        x: point.x * (parameters.fieldW / parameters.imageW) - (parameters.startX + (parameters.isBackwards ? parameters.robotW - parameters.robotCenterToBack : parameters.robotCenterToBack)),
        y: (parameters.imageH - point.y) * (parameters.fieldH / parameters.imageH) - (parameters.startY + parameters.robotH / 2)
    };
}

function imagePointsToBasePoints() {
    parameters.basePoints = parameters.imageBasePoints.slice(0).map(pt => pxToM(pt)
)
    ;
    parameters.basePoints.splice(0, 0, {x: 0, y: 0});
}

function basePointsToImagePoints() {
    parameters.imageBasePoints = parameters.basePoints.slice(0).slice(1, parameters.basePoints.length).map(pt => mToPx(pt)
)
    ;
}

function pathPointsToImagePathPoints() {
    parameters.imagePathPoints = parameters.pathPoints.slice(0).map(pt => mToPx(pt)
)
}

function basePointsToPathPoints() {
    if (parameters.basePoints.length < 2) {
        parameters.pathPoints = parameters.basePoints;
        return parameters.basePoints;
    }
    // parameters.basePoints.splice(0, 0, {x:0,y:0}); // add first points [0, 0] for calculations
    parameters.pathPoints = parameters.basePoints.slice(0);
//    parameters.pathPoints.map(pt => {x:pt.x + Math.random() / 1000, y:pt.y});
    parameters.pathPoints.map(function (pt) {
        return {x: pt.x + Math.random() / 1000, y: pt.y}
    });
    var counter = 0;
    for (i = 1; i < parameters.basePoints.length - 1; i++) {
        var rotationPts = getRotationRadiusPoints(parameters.basePoints[i - 1], parameters.basePoints[i], parameters.basePoints[i + 1]);
        parameters.pathPoints.splice(i + counter, 1,...rotationPts
    )
        ;
        counter += rotationPts.length - 1;
    }
    return parameters.pathPoints;
}

function getAngle(p1, p2) {
    dx = p1.x - p2.x;
    dy = p1.y - p2.y;
    return Math.atan(dy / dx);
}

function getDist(p1, p2) {
    dx = p1.x - p2.x;
    dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function getRotationRadiusPoints(startPt, middlePt, endPt) {
//	 return [middlePt];
    return getRotationPoints(startPt, middlePt, endPt, parameters.rotationRadius, parameters.rotationAngleStep);
}

function getXYatLine(startPt, endPt, distanceFromStart) {
    var percent = distanceFromStart / getDist(startPt, endPt);
    var dx = endPt.x - startPt.x;
    var dy = endPt.y - startPt.y;
    var X = startPt.x + dx * percent;
    var Y = startPt.y + dy * percent;
    return ({x: X, y: Y});
}

function pathPointsToCode() {
    var result = "addSequential(new PathPointsCommand(new Point[]{";
    for (var i = 1; i < parameters.pathPoints.length; i++) {
        result += "<br>&nbsp&nbsp&nbsp new Point({0}, {1}".format(parameters.pathPoints[i].x, parameters.pathPoints[i].y);
        if (parameters.isRelative) {
            result += " * {0}".format(parameters.yDirectionVarName);
        }
        result += ")";
        if (i != parameters.pathPoints.length - 1)
            result += ",";
    }
    result += "}, {0}, {1}));".format(parameters.isBackwards, parameters.isRelative);
    return result;
}