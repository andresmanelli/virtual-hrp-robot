describe('API test - virtual-hrp-robot',function(){
	
	it('virtual-hrp-robot.js should exist',function(){
		var VR = require('../virtual-hrp-robot.js');
		VR.should.exist;
	});

	it('should return false if no robotName',function(){
		var VR = require('../virtual-hrp-robot.js')();
		VR.should.be.false;
	});

	it('should be false if file doesn\'t exits',function(){
		var VR = require('../virtual-hrp-robot.js')('asd');
		VR.should.be.false;
	});
});