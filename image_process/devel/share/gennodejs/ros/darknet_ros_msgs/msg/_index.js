
"use strict";

let BoundingBox = require('./BoundingBox.js');
let ImageWithBBoxes = require('./ImageWithBBoxes.js');
let ObjectCount = require('./ObjectCount.js');
let BoundingBoxes = require('./BoundingBoxes.js');
let CheckForObjectsActionGoal = require('./CheckForObjectsActionGoal.js');
let CheckForObjectsFeedback = require('./CheckForObjectsFeedback.js');
let CheckForObjectsActionResult = require('./CheckForObjectsActionResult.js');
let CheckForObjectsAction = require('./CheckForObjectsAction.js');
let CheckForObjectsActionFeedback = require('./CheckForObjectsActionFeedback.js');
let CheckForObjectsResult = require('./CheckForObjectsResult.js');
let CheckForObjectsGoal = require('./CheckForObjectsGoal.js');

module.exports = {
  BoundingBox: BoundingBox,
  ImageWithBBoxes: ImageWithBBoxes,
  ObjectCount: ObjectCount,
  BoundingBoxes: BoundingBoxes,
  CheckForObjectsActionGoal: CheckForObjectsActionGoal,
  CheckForObjectsFeedback: CheckForObjectsFeedback,
  CheckForObjectsActionResult: CheckForObjectsActionResult,
  CheckForObjectsAction: CheckForObjectsAction,
  CheckForObjectsActionFeedback: CheckForObjectsActionFeedback,
  CheckForObjectsResult: CheckForObjectsResult,
  CheckForObjectsGoal: CheckForObjectsGoal,
};
