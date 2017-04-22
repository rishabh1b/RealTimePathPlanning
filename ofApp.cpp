#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
#ifdef randomSeed
	ofSeedRandom(randomSeed);
#endif // randomSeed
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	ofSetWindowTitle("Dynamic-obstacles");
	ofBackground(255, 255, 255);
	
	map.setup();

#ifdef randomSeed
	std::cout << "RandomSeed:" << randomSeed << endl;
#endif

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Setup:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::update(){
	if (!updateFlag) return;
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG

	map.update();

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Update:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::draw(){
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG

	map.render();

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Draw:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'p')
	{
		updateFlag = !updateFlag;
	}
	else if(key=='g')
	{
		map.grid = !map.grid;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
