describe('API test - ScaraRobot',function(){

	it('should exist as Object if robot is found',function(){
		var scara = require('../robots/ScaraRobot.js');
		scara.should.exist;
		scara.should.be.an('object');
	});

	it('should have the directK method',function(){
		var scara = require('../robots/ScaraRobot.js');
		scara.directK.should.exist;
		scara.directK.should.be.a('function');
	});

	it('should have the inverseK method',function(){
		var scara = require('../robots/ScaraRobot.js');
		scara.inverseK.should.exist;
		scara.inverseK.should.be.a('function');
	});

	describe('info test:',function(){
		var scara = require('../robots/ScaraRobot.js');
		
		it('info should exist',function(){
			scara.strInfo.should.exist;
			scara.robotInfo.should.exist;
			scara.jointInfo.should.exist;
		});

		it('robotInfo validation',function(){
			scara.robotInfo.B.should.equal('F.Ing-U.N.Cuyo');
			scara.robotInfo.M.should.equal('SCARA-F.Ing');
			scara.robotInfo.DOF.should.equal(3);
			scara.robotInfo.J.should.eql([10,23,45]);
		});
		
		it('jointInfo validation',function(){
			scara.jointInfo.should.have.all.keys(['10','23','45']);

			scara.jointInfo[10].J_TYPE.should.equal('R');
			scara.jointInfo[23].J_TYPE.should.equal('R');
			scara.jointInfo[45].J_TYPE.should.equal('T');

			scara.jointInfo[10].J_RANGE.should.eql([-90,90]);
			scara.jointInfo[23].J_RANGE.should.eql([-90,90]);
			scara.jointInfo[45].J_RANGE.should.eql([0,100]);

			scara.jointInfo[10].J_UNITS.should.equal('deg');
			scara.jointInfo[23].J_UNITS.should.equal('deg');
			scara.jointInfo[45].J_UNITS.should.equal('mm');

			scara.jointInfo[10].J_DESC.should.equal('Servo-Motor');
			scara.jointInfo[23].J_DESC.should.equal('Servo-Motor');
			scara.jointInfo[45].J_DESC.should.equal('No-Description');			
		});
	});

	describe('ang() tests',function(){
		var scara = require('../robots/ScaraRobot');	

		it('should be 45 deg for [0,0,3,3]',function(){
			scara.ang(0,0,3,3).should.be.closeTo(Math.PI/4,0.001);
		});
		it('expect 30 degrees',function(){
			scara.ang(0,0,Math.cos(scara.radians(30))*20,Math.sin(scara.radians(30))*20).should.be.closeTo(scara.radians(30),0.001);
		});
		it('expect -67 degrees',function(){
			scara.ang(0,0,Math.cos(scara.radians(-67))*20,Math.sin(scara.radians(-67))*20).should.be.closeTo(scara.radians(-67),0.001);
		});
		it('expect 143 degrees',function(){
			scara.ang(0,0,Math.cos(scara.radians(143))*20,Math.sin(scara.radians(143))*20).should.be.closeTo(scara.radians(143),0.001);
		});
		it('expect -170 degrees',function(){
			scara.ang(0,0,Math.cos(scara.radians(-170))*20,Math.sin(scara.radians(-170))*20).should.be.closeTo(scara.radians(-170),0.001);
		});
		it('expect 10 degrees',function(){
			scara.ang(0,0,Math.cos(scara.radians(370))*20,Math.sin(scara.radians(370))*20).should.be.closeTo(scara.radians(10),0.001);
		});
		it('expect -10 degrees',function(){
			scara.ang(0,0,Math.cos(scara.radians(350))*20,Math.sin(scara.radians(350))*20).should.be.closeTo(scara.radians(-10),0.001);
		});
		it('expect 180 degrees (NOT -180)',function(){
			scara.ang(0,0,Math.cos(scara.radians(180))*20,Math.sin(scara.radians(180))*20).should.be.closeTo(scara.radians(180),0.001);
		});
		it('expect 180 degrees (NOT -180)',function(){
			scara.ang(0,0,Math.cos(scara.radians(-180))*20,Math.sin(scara.radians(-180))*20).should.be.closeTo(scara.radians(180),0.001);
		});
	});

	describe('int2circ test',function(){
		var scara = require('../robots/ScaraRobot');

		it('expect ['+scara.Arm0L+',0,'+scara.Arm0L+',0]',function(){
			scara.int2Circ(scara.Arm0L+scara.Arm1L,0).should.eql([scara.Arm0L,0,scara.Arm0L,0]);
		});
		it('expect [0,'+scara.Arm0L+',0,'+scara.Arm0L+']',function(){
			scara.int2Circ(0,scara.Arm0L+scara.Arm1L).should.eql([0,scara.Arm0L,0,scara.Arm0L]);
		});
		it('expect [18.742,6.98,-18.742,6.98]',function(){
			var int = scara.int2Circ(0,25);
			var expectedInt = [18.742,6.98,-18.742,6.98];
			int.forEach(function(val,i){
				val.should.be.closeTo(expectedInt[i],0.01);
			});
		});
		it('expect [-19.803,2.801,13.076,-15.133]',function(){
			var int = scara.int2Circ(-12,-22);
			var expectedInt = [-19.803,2.801,13.076,-15.133];
			int.forEach(function(val,i){
				val.should.be.closeTo(expectedInt[i],0.01);
			});
		});
	});

	describe('directK tests: ',function(){
		var scara = require('../robots/ScaraRobot');
		var joints = {10: 0, 23: 0, 45: 14};

		it('test1',function(){
			var pos = scara.directK(joints);
			var expectedPos = {x: scara.Arm0L+scara.Arm1L, y: 0, z: 14};
			for(var coord in pos){
				if(pos.hasOwnProperty(coord)){
					pos[coord].should.be.closeTo(expectedPos[coord],0.001);
				}
			}
		});
		it('test2',function(){
			joints = {10: 0, 23: 90, 45: 2};
			var pos = scara.directK(joints);
			var expectedPos = {x: scara.Arm0L, y: scara.Arm1L, z: 2};
			for(var coord in pos){
				if(pos.hasOwnProperty(coord)){
					pos[coord].should.be.closeTo(expectedPos[coord],0.001);
				}
			}
		});
		it('test3',function(){
			joints = {10: 0, 23: -90, 45: -1};
			var pos = scara.directK(joints);
			var expectedPos = {x: scara.Arm0L, y: -scara.Arm1L, z: -1};
			for(var coord in pos){
				if(pos.hasOwnProperty(coord)){
					pos[coord].should.be.closeTo(expectedPos[coord],0.001);
				}
			}
		});
		it('test4',function(){
			joints = {10: 45, 23: 0, 45: 3};
			var pos = scara.directK(joints);
			var expectedPos = {x: 32.527, y: 32.527, z: 3};
			for(var coord in pos){
				if(pos.hasOwnProperty(coord)){
					pos[coord].should.be.closeTo(expectedPos[coord],0.001);
				}
			}
		});
		it('test5',function(){
			joints = {10: 17, 23: -17, 45: 1};
			var pos = scara.directK(joints);
			var expectedPos = {x: 45.126, y: 5.847, z: 1};
			for(var coord in pos){
				if(pos.hasOwnProperty(coord)){
					pos[coord].should.be.closeTo(expectedPos[coord],0.001);
				}
			}
		});
	});	

	describe('inverseK tests',function(){
		var scara = require('../robots/ScaraRobot');

		it('test1',function(){
			var pos = {x: 0, y: 0, z: 14};
			var expectedJoints = {10: 0, 23: 0, 45: 0};
			var joints = scara.inverseK(pos,expectedJoints);
			for(var joint in joints){
				if(joints.hasOwnProperty(joint)){
					joints[joint].should.be.closeTo(expectedJoints[joint],0.002);
				}
			}
		});

		it('test2',function(){
			var pos = {x: scara.Arm0L, y: scara.Arm1L, z: 2};
			var expectedJoints = {10: 0, 23: 90, 45: 2};
			var joints = scara.inverseK(pos,expectedJoints);			
			for(var joint in joints){
				if(joints.hasOwnProperty(joint)){
					joints[joint].should.be.closeTo(expectedJoints[joint],0.002);
				}
			}
		});

		it('test3',function(){
			var pos = {x: scara.Arm0L, y: -scara.Arm1L, z: -1};
			var expectedJoints = {10: 0, 23: -90, 45: -1};
			var joints = scara.inverseK(pos,expectedJoints);
			for(var joint in joints){
				if(joints.hasOwnProperty(joint)){
					joints[joint].should.be.closeTo(expectedJoints[joint],0.002);
				}
			}
		});

		it('test4',function(){
			var pos = {x: 32.527, y: 32.527, z: 3};
			var expectedJoints = {10: 45, 23: 0, 45: 3};
			var joints = scara.inverseK(pos,expectedJoints);
			for(var joint in joints){
				if(joints.hasOwnProperty(joint)){
					joints[joint].should.be.closeTo(expectedJoints[joint],0.002);
				}
			}
		});

		it('test5',function(){
			var pos = {x: 45.126095, y: 5.847434, z: 1};
			var expectedJoints = {10: 17, 23: -17, 45: 1};
			var joints = scara.inverseK(pos,expectedJoints);
			for(var joint in joints){
				if(joints.hasOwnProperty(joint)){
					joints[joint].should.be.closeTo(expectedJoints[joint],0.002);
				}
			}
		});

		it('test6',function(){
			var pos = {x: -46, y: 0, z: -9};
			var expectedJoints = {10: 180, 23: 0, 45: -9};
			var joints = scara.inverseK(pos,expectedJoints);
			for(var joint in joints){
				if(joints.hasOwnProperty(joint)){
					joints[joint].should.be.closeTo(expectedJoints[joint],0.002);
				}
			}
		});
	});
});