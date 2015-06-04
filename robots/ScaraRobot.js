/**
 * Virtual HRP Robot
 *
 * -- SCARA
 *
 * First Arm Length: (Arm0L = 20)
 * Second Arm Length: (Arm1L = 26)
 * Third DOF: Traslation
 *
 * Author: Andrés Manelli
 * email: andresmanelli@gmail.com
 *
 * Asociación de Mecatrónica de Mendoza
 */

var ScaraRobot = (function(){

  'use strict';

  var HRP = require('hrp');
  var HRPDefs = HRP(0,0,true); //Only definitions
  
  var robot = {};

  // Arms lengths
  robot.Arm0L = 20;
  robot.Arm1L	= 26;

  // Position Labels
  robot.X = 'x';
  robot.Y = 'y';
  robot.Z = 'z';

  // Joints' IDs
  robot.Q0 = 10;
  robot.Q1 = 23;
  robot.Q2 = 45;

  /**
   * Computes the angle between two points.
   * @param  {Double} x1 X-Coord of the first point
   * @param  {Double} y1 Y-Coord of the first point
   * @param  {Double} x2 X-Coord of the second point
   * @param  {Double} y2 Y-Coord of the second point
   * @return {Double}    Angle in radians: (-180;180]
   */
  robot.ang = function(x1, y1, x2, y2){
      
      var deltax = (x2-x1);
      var deltay = (y2-y1);
      
      //Same point
      if(deltax == 0 && deltay == 0)
          return 0;
      
      if(deltax == 0){
          if(deltay > 0)
              return Math.PI/2;
          else if (deltay < 0)
              return -Math.PI/2;
      }
      if(deltay == 0){
        if(deltax > 0)
          return 0;
        else if (deltax < 0)
          return Math.PI;
      }
      
      var tmpang = Math.atan(deltay/deltax);
      
      if(deltax > 0 && deltay > 0){
        //Primer Cuadrante
      }
      else if(deltax < 0 && deltay > 0){
        //Segundo Cuadrante
        tmpang = tmpang + Math.PI;
      }
      else if(deltax < 0 && deltay < 0){
        //Tercer Cuadrante
        tmpang = tmpang - Math.PI;
      }
      else if(deltax > 0 && deltay < 0){
        //Cuarto Cuadrante
      }
      
      if(Math.abs(tmpang+robot.radians(180)) < 0.0001)
        tmpang = -tmpang

      return tmpang;
  };

  /**
   * Computes the intersection of two circles. The first circle is at the
   * origin. The radius of the first circle is Arm0L and the radius of the
   * second circle is Arm1L
   * @param  {Double} c2x X-Coord of the center of the second circle
   * @param  {Double} c2y Y-Coord of the center of the second circle
   * @return {Array}     Intersection: Two points:
   *                                   [0] = x1, [1] = y1, [2] = x2, [3] = y2
   */
  robot.int2Circ = function(c2x,c2y){
    
    var x1,x2,y1,y2;

    if(c2x === 0 && c2y !== 0){
      y1 = (robot.Arm0L*robot.Arm0L - robot.Arm1L*robot.Arm1L + c2y*c2y)/(2*c2y);
      y2 = y1;
      x1 = Math.sqrt(robot.Arm0L*robot.Arm0L - y1*y1);
      x2 = -x1;
      if(c2y === robot.Arm0L+robot.Arm1L)
          return [x1,y1,x1,y1];
      return [x1,y1,x2,y2];
    }
    
    if(c2y === 0 && c2x !== 0){
      x1 = (robot.Arm0L*robot.Arm0L - robot.Arm1L*robot.Arm1L + c2x*c2x)/(2*c2x);
      x2 = x1;
      y1 = Math.sqrt(robot.Arm0L*robot.Arm0L - x1*x1);
      y2 = -y1;
      if(c2x === robot.Arm0L+robot.Arm1L)
          return [x1,y1,x1,y1];
      return [x1,y1,x2,y2];
    }

    if(c2x === 0 && c2y === 0){
      return false;
    }
    
    var alpha = Math.atan(c2y/c2x);
    var c2x_p = c2x;
    
    if((c2x < 0 && c2y > 0) || (c2x < 0 && c2y < 0)){
      //Segundo o Tercer Cuadrante. Espejamos.
      alpha = -alpha;
      c2x_p = -c2x;
    }
        
    var R = [Math.cos(alpha),Math.sin(alpha),-Math.sin(alpha),Math.cos(alpha)];
    var R_1 = [Math.cos(alpha),-Math.sin(alpha),Math.sin(alpha),Math.cos(alpha)];
    var c2u = R[0]*c2x_p+R[1]*c2y;
    var c2v = 0; //Always
    
    var int = robot.int2Circ(c2u,c2v);
    x1 = R_1[0]*int[0]+R_1[1]*int[1];
    y1 = R_1[2]*int[0]+R_1[3]*int[1];
    x2 = R_1[0]*int[2]+R_1[1]*int[3];
    y2 = R_1[2]*int[2]+R_1[3]*int[3];
    
    if((c2x < 0 && c2y > 0) || (c2x < 0 && c2y < 0)){
      //Segundo o Tercer Cuadrante. Espejamos.
      x1 = -x1;
      x2 = -x2
    }
    
    return [x1,y1,x2,y2];
  }

  /**
   * Computes the Inverse Kinematics.
   * @param  {Object} phP    Position of the end effector
   * @param  {Object} artRef Joints values of the last position in DEGREES
   * @return {Object}        New Joints values if possible, false otherwise.
   *                         NOTE: artRef is necessary in order to choose between 
   *                         the two solutions found.
   */
  robot.inverseK = function(phP, artRef){
         
      var fartP = {};
      
      var x1,y1,x2,y2;
      var inter = robot.int2Circ(phP[robot.X],phP[robot.Y]);
      
      if(inter === false || isNaN(inter[0])){
        return false;
      }

      x1 = inter[0];
      y1 = inter[1];
      x2 = inter[2];
      y2 = inter[3];
      
      var ang11,ang12,ang21,ang22;

      ang11 = robot.ang(0,0,x1,y1);
      ang12 = robot.ang(0,0,x2,y2);
      ang21 = robot.ang(x1,y1,phP[robot.X],phP[robot.Y]) - ang11;
      ang22 = robot.ang(x2,y2,phP[robot.X],phP[robot.Y]) - ang12;   

      var w1 = 2*Math.abs(robot.radians(artRef[robot.Q0])-ang11) + Math.abs(robot.radians(artRef[robot.Q1])-ang21);
      var w2 = 2*Math.abs(robot.radians(artRef[robot.Q1])-ang12) + Math.abs(robot.radians(artRef[robot.Q1])-ang22);
      
      if(w1>w2 || w1==w2){
          fartP[robot.Q0] = ang12;
          fartP[robot.Q1] = ang22;
      }
      else{
          fartP[robot.Q0] = ang11;
          fartP[robot.Q1] = ang21;
      }
      
      // OK?
      fartP[robot.Q2] = phP[robot.Z];
      
      fartP[robot.Q0] = robot.degrees(fartP[robot.Q0]);
      fartP[robot.Q1] = robot.degrees(fartP[robot.Q1]);
      return fartP;
  };

  /**
   * Computes the Direct Kinematics
   * @param  {Object} joints Joints values in DEGREES
   * @return {Object}        Position of the end effector
   */
  robot.directK = function(joints){

    joints[robot.Q0] = robot.radians(joints[robot.Q0]);
    joints[robot.Q1] = robot.radians(joints[robot.Q1]);

    var x = robot.Arm0L*Math.cos(joints[robot.Q0])+robot.Arm1L*Math.cos(joints[robot.Q0]+joints[robot.Q1]);
    var y = robot.Arm0L*Math.sin(joints[robot.Q0])+robot.Arm1L*Math.sin(joints[robot.Q0]+joints[robot.Q1]);
    var z = joints[robot.Q2];
    
    var newEEPos = {};
    newEEPos[robot.X] = x;
    newEEPos[robot.Y] = y;
    newEEPos[robot.Z] = z;
    
    return newEEPos;
  };

  /**
   * Converts from degrees to radians
   * @param  {Double} degrees Angle in degrees
   * @return {Double}         Angle in radians
   */
  robot.radians = function(degrees) {
    return degrees * Math.PI / 180.0;
  };
   
  /**
   * Converts from radians to degrees
   * @param  {Double} radians Angle in radians
   * @return {Double}         Angle in degrees
   */
  robot.degrees = function(radians) {
    return radians * 180.0 / Math.PI;
  };

  // Robot first position
  robot.EEPos = {};
  robot.EEPos[robot.X] = 46;
  robot.EEPos[robot.Y] = 0;
  robot.EEPos[robot.Z] = 0;

  // Robot first joint values
  robot.joints = {};
  robot.joints[robot.Q0] = 0;
  robot.joints[robot.Q1] = 0;
  robot.joints[robot.Q2] = 0;

  // Robot basic information
  robot.robotInfo = {
    B: 'F.Ing-U.N.Cuyo', //Brand
    M: 'SCARA-F.Ing', //Model
    DOF: 3, //Degrees of freedom
    J: [10,23,45],  //Joints IDs
  };

  // Robot joints information
  robot.jointInfo = {};
  robot.jointInfo[robot.Q0] = {
    J_TYPE: 'R',
    J_DESC: 'Servo-Motor',
    J_RANGE: [-90, 90],
    J_UNITS: 'deg'
  };
  robot.jointInfo[robot.Q1] = {
    J_TYPE: 'R',
    J_DESC: 'Servo-Motor',
    J_RANGE: [-90, 90],
    J_UNITS: 'deg'
  }
  robot.jointInfo[robot.Q2] = {
    J_TYPE: 'T',
    J_DESC: 'No-Description',
    J_RANGE: [0, 100],
    J_UNITS: 'mm'
  }

  robot.strInfo = HRPDefs.robotInfo2str(robot.robotInfo,robot.jointInfo);

  return robot;
})();

module.exports = ScaraRobot;