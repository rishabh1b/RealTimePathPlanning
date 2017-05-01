#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
#ifdef randomSeed
	ofSeedRandom(randomSeed);
#endif // randomSeed
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	ofSetWindowTitle("Dynamic-obstacles");
	ofBackground(200,200,200,200);
	//map = new Enviroment();
	//car.setup();

	ofVec2f w;
	w.set(ofGetWidth() / 2, 0);
	wall = new maze(w);
	obstacles *ob = wall;
	obst.push_back(ob);

	w.set(ofGetWidth() / 2, 0.6*ofGetHeight());
	wall = new maze(w);
	ob = wall;
	obst.push_back(ob);

	w.set(ofGetWidth() / 4, 0.4*ofGetHeight());
	wall = new maze(w, 60, 0.2*ofGetHeight());
	ob = wall;
	obst.push_back(ob);

	w.set(0.75*ofGetWidth(), 0.4*ofGetHeight());
	wall = new maze(w, 60, 0.2*ofGetHeight());
	ob = wall;
	obst.push_back(ob);

	for (unsigned int i = 0; i < numberOfobst; i++)
	{
		obstacles *ob = new obstacles();
		//OBST = new movingObst();
		//obstacles *ob = OBST;
		obst.push_back(ob);
	}
	//
	OBST = new movingObst();
	ob = OBST;
	obst.push_back(ob);

	cout << "Obst size: " << obst.size() << endl;

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
#ifdef automatic

	for (auto i : obst) {
		i->move(obst);
		//cout << "location: " << i->loc() << "Radius: " << i->rad() << endl;
		//cout << i.getX() << "  " << i.getY() << endl;
	}
#endif // automatic

	if (map!= NULL) map->update(car,obst);
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

	for (auto i : obst) {
		i->render();
	}
	if (map != NULL) map->render();
	if (car!= NULL) car->render();

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
		map->grid = !map->grid;
	}
	else if (key == 'x') {
		ofImage img;
		img.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
		img.save("screenshot.png");
	}
#ifdef manual
	OBST->move(key);
#endif // manual

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
	ofVec2f loc;
	loc.set(x, y);
	if (button == 0) {
		if (car != NULL) {
			map->targetSet(loc);
		}
	}
	else if (button == 2) {
		car = new Robot(loc);
		map = new Enviroment(car->getLocation());
	}
	else
	{

	}
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
