
"use strict";

let SetGPSPosition = require('./SetGPSPosition.js')
let LandGroup = require('./LandGroup.js')
let Takeoff = require('./Takeoff.js')
let SetLocalPosition = require('./SetLocalPosition.js')
let Reset = require('./Reset.js')
let TakeoffGroup = require('./TakeoffGroup.js')
let Land = require('./Land.js')

module.exports = {
  SetGPSPosition: SetGPSPosition,
  LandGroup: LandGroup,
  Takeoff: Takeoff,
  SetLocalPosition: SetLocalPosition,
  Reset: Reset,
  TakeoffGroup: TakeoffGroup,
  Land: Land,
};
