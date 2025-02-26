
"use strict";

let Latency = require('./Latency.js');
let FlightCommand = require('./FlightCommand.js');
let WaypointList = require('./WaypointList.js');
let ProcessStatus = require('./ProcessStatus.js');
let NodeList = require('./NodeList.js');
let NodeStatus = require('./NodeStatus.js');
let Keypoint = require('./Keypoint.js');
let Box = require('./Box.js');
let FlightStateTransition = require('./FlightStateTransition.js');
let FlightEvent = require('./FlightEvent.js');
let TelemString = require('./TelemString.js');
let ImageDetections = require('./ImageDetections.js');
let ControlMessage = require('./ControlMessage.js');
let JoyDef = require('./JoyDef.js');
let Detection = require('./Detection.js');
let ImageSegmentation = require('./ImageSegmentation.js');

module.exports = {
  Latency: Latency,
  FlightCommand: FlightCommand,
  WaypointList: WaypointList,
  ProcessStatus: ProcessStatus,
  NodeList: NodeList,
  NodeStatus: NodeStatus,
  Keypoint: Keypoint,
  Box: Box,
  FlightStateTransition: FlightStateTransition,
  FlightEvent: FlightEvent,
  TelemString: TelemString,
  ImageDetections: ImageDetections,
  ControlMessage: ControlMessage,
  JoyDef: JoyDef,
  Detection: Detection,
  ImageSegmentation: ImageSegmentation,
};
