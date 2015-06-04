/**
 * Virtual HRP Robot
 *
 *
 * Author: Andrés Manelli
 * email: andresmanelli@gmail.com
 *
 * Asociación de Mecatrónica de Mendoza
 */

var VirtualRobot = function(robotName){

  'use strict';

  // Verify argument
  if(typeof robotName === 'undefined' || robotName ===null)
    return false;

  // Verify file
  var fs = require('fs');
  try{
    fs.statSync('./robots/'+robotName+'.js');
  }catch(e){
    return false;
  }

  var HRP = require('hrp');
  var HRPDefs = HRP(0,0,true); //Only definitions
  var zmq = require('zmq');

  // Open robot 
  var robot = require('./robots/'+robotName+'.js');  

  // References to Kinematics
  var inverseK = robot.inverseK;
  var directK = robot.directK;

  // Communication socket
  var virtualRobot = zmq.socket('rep');

  // Listener
  // TODO: HRP compilance!
  virtualRobot.on('message', function(request) {
    
    var req = request.toString();  
    
    if(req === HRPDefs.COMP_ACK()){
      virtualRobot.send(HRPDefs.COMP_ACK());
    }else if(req === HRPDefs.ROBOT_INFO()){
      virtualRobot.send(robot.strInfo);
    }else if(req.substr(0,HRPDefs.SET_EE_DIF_POS().length) === HRPDefs.SET_EE_DIF_POS()){
      req = req.slice(HRPDefs.SET_EE_DIF_POS().length+1,req.length-1);
      var cmdVal = req.split(':');
      for(var i=0;i<cmdVal.length;i++){
        cmdVal[i] = parseFloat(cmdVal[i]);
      }
      virtualRobot.moveEE(cmdVal);
      virtualRobot.send(HRPDefs.GENERAL_ACK());
    }else if(req === HRPDefs.GET_JOINTS()){
      virtualRobot.send(HRPDefs.joints2str(robot.joints));
    }
  });

  /**
   * Moves the enf effector by 'change'
   * @param  {Array} change Contains the desired x,y,z changes in the position
   * @return {Boolean}      true if change could be done, false otherwise
   */
  virtualRobot.moveEE = function(change){
    
    var newEEPos = {};
    
    newEEPos[X] = robot.EEPos[X]+change[0];
    newEEPos[Y] = robot.EEPos[Y]+change[1];
    newEEPos[Z] = robot.EEPos[Z]+change[2];
    
    if(!virtualRobot.updateJoints(newEEPos)){
      return false;
    }

    virtualRobot.updateEEPos();
    return true;
  };

  /**
   * Updates the end effector position with the new joints values
   * @return {Boolean} true if success, false otherwise
   */
  virtualRobot.updateEEPos = function(){
    var newEEPos = directK(robot.joints);
    
    for(var axis in newEEPos){
      if(robot.EEPos.hasOwnProperty(axis)){
        robot.EEPos[axis] = newEEPos[axis];
      }
    }
    
    return true;
  };

  /**
   * Update joints values with the new desired end effector position
   * @param  {Object} newEEPos Position of the end effector
   * @return {Boolean}         true if success, false otherwise
   */
  virtualRobot.updateJoints = function(newEEPos){
    var newJPos = inverseK(newEEPos,robot.joints);
    
    if(!newJPos)
      return false;
    
    for(var id in newJPos){
      if(robot.joints.hasOwnProperty(id)){
        robot.joints[id] = newJPos[id];
      }
    }
    
    return true;
  };

  // Wait for connection
  virtualRobot.bind('tcp://*:5555', function(err) {
    if (err) {
      console.log(err);
    } else {
      console.log("Listening on 5555…");
    }
  });

  // Set initial position. Just in case
  (function(){
    virtualRobot.updateEEPos();
  })();

  return virtualRobot;
};

module.exports = VirtualRobot;