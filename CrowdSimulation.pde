//Change the below parameters to change the scenario/roadmap size
int numObstacles = 30;
int numNodes  = 300;

//parameters below are for agents
static int maxNumAgents = 30;
int numAgents = 15;

float k_goal = 10f; 
float k_avoid = 300f;
float agentRad = 20;
float goalRad = 15;
float goalSpeed = 100;
float epsi = 5;
float maxAcc = 3000;

//tracker for which node in the astar paths to follow
int[] pathStop = new int[maxNumAgents];

//The agent states
Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
Vec2[] agentAcc = new Vec2[maxNumAgents];

//The agent goals
Vec2[] goalPos = new Vec2[maxNumAgents];
  
  
//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii


static int maxNumNodes = 3000;
Vec2[] nodePos = new Vec2[maxNumNodes];

ArrayList<ArrayList<Integer>> paths;

//IMAGE LOADING!!!
ArrayList<PImage> images = new ArrayList<PImage>(5);
PImage asteroid;
PImage ship;


int strokeWidth = 2;
void setup(){
  size(1024,768,P3D);
  roadMap();
  asteroid = loadImage("sprites/Asteroid.png");
  ship = loadImage("sprites/UFO_Image.png");
  images.add(loadImage("sprites/Planet_1.png"));
  images.add(loadImage("sprites/Planet_2.png"));
  images.add(loadImage("sprites/Planet_3.png"));
  images.add(loadImage("sprites/Planet_4.png"));
  images.add(loadImage("sprites/Planet_5.png"));
  
  //Set initial velocities to cary agents towards their goals
  for (int i = 0; i < numAgents; i++){
    //get the path
    ArrayList<Integer> curPath = paths.get(i);
    if (curPath.size() < 2) continue;
    int goal = curPath.get(pathStop[i]);
    agentVel[i] = nodePos[goal].minus(agentPos[i]);
    if (agentVel[i].length() > 0)
      agentVel[i].setToLength(goalSpeed);
  }
}

int numCollisions;
float pathLength;
boolean reachedGoal;

void roadMap(){
  // long startTime, endTime;
  
  placeRandomObstacles(numObstacles);
  
  for (int i = 0; i < numAgents; i++) {
    agentPos[i] = sampleFreePos();
    goalPos[i] = sampleFreePos();
    pathStop[i] = 1;
  }

  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  
  paths = planPath(numAgents, agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}

boolean paused = true;
void draw(){
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(20, 0, 50); //dark indigo 
  noStroke();
  noFill();
  textureMode(NORMAL);
  
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
  
  
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i] - 5;
    PImage img = images.get(i%5);
    circle(c.x,c.y,r*2);
    beginShape();
    texture(img);
    vertex(c.x-r, c.y-r, 0, 0);
    vertex(c.x+r, c.y-r, 1, 0);
    vertex(c.x+r, c.y+r, 1, 1);
    vertex(c.x-r, c.y+r, 0, 1); 
    endShape();
  }
  //Draw the first circle a little special b/c the user controls it
  fill(240);
  strokeWeight(2);
  circle(circlePos[0].x,circlePos[0].y,circleRad[0]*2);
  beginShape();
  texture(asteroid);
  vertex(circlePos[0].x-circleRad[0], circlePos[0].y-circleRad[0], 0, 0);
  vertex(circlePos[0].x+circleRad[0], circlePos[0].y-circleRad[0], 1, 0);
  vertex(circlePos[0].x+circleRad[0], circlePos[0].y+circleRad[0], 1, 1);
  vertex(circlePos[0].x-circleRad[0], circlePos[0].y+circleRad[0], 0, 1); 
  endShape();
  strokeWeight(1);
  
  //Draw agents and goals
  for (int i = 0; i < numAgents; i++){
    fill(250,30,50);
    //circle(nodePos[goalNode].x,nodePos[goalNode].y,20);
    circle(goalPos[i].x,goalPos[i].y,2*goalRad);
    // strokeWeight(1);
    noFill();
    //circle(nodePos[startNode].x,nodePos[startNode].y,20);
    circle(agentPos[i].x,agentPos[i].y,2*agentRad);
    beginShape();
    texture(ship);
    vertex(agentPos[i].x-agentRad, agentPos[i].y-agentRad, 0, 0);
    vertex(agentPos[i].x+agentRad, agentPos[i].y-agentRad, 1, 0);
    vertex(agentPos[i].x+agentRad, agentPos[i].y+agentRad, 1, 1);
    vertex(agentPos[i].x-agentRad, agentPos[i].y+agentRad, 0, 1); 
    endShape();
  }
  
  
  /*
  //Draw PRM Nodes
  fill(255);
  for (int i = 0; i < numNodes; i++){
    circle(nodePos[i].x,nodePos[i].y,5);
  }
  
  //Draw graph
  stroke(100,100,100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }
 
  // draw lines
  for (int i = 0; i < numAgents; i++){
    ArrayList<Integer> curPath = paths.get(i);
    if (curPath.size() > 0 && curPath.get(0) == -1) continue; //No path found
    
    //Draw Planned Path
    stroke(20,255,40);
    strokeWeight(5);
    if (curPath.size() == 0){
      line(agentPos[i].x,agentPos[i].y,goalPos[i].x,goalPos[i].y);
      continue;
    }
    line(agentPos[i].x,agentPos[i].y,nodePos[curPath.get(1)].x,nodePos[curPath.get(1)].y);
    for (int j = 1; j < curPath.size() - 2; j++){
      int curNode = curPath.get(j);
      int nextNode = curPath.get(j+1);
      line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
    }
    line(goalPos[i].x,goalPos[i].y,nodePos[curPath.get(curPath.size()-2)].x,nodePos[curPath.get(curPath.size()-2)].y);
  }
  */
}

boolean shiftDown = false;
void keyPressed(){
  if (key == 'r'){
    roadMap();
    paused = true;
    numObstacles = 30;
    return;
  }
  
  if (key == ' ') {
    paused = !paused;
    return;
  }
  
  if (key == 'a'){
    //switch between A* and RRT!
    return;
  }
  
  if (keyCode == SHIFT){
    shiftDown = true;
  }
  
  float speed = 10;
  if (shiftDown) speed = 30;
  if (keyCode == RIGHT){
    circlePos[0].x += speed;
  }
  if (keyCode == LEFT){
    circlePos[0].x -= speed;
  }
  if (keyCode == UP){
    circlePos[0].y -= speed;
  }
  if (keyCode == DOWN){
    circlePos[0].y += speed;
  }
  generateRandomNodes(numNodes, circlePos, circleRad);
  paths = planPath(numAgents, agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  for(int i = 0; i < numAgents; i++){
    // Vec2[] nodePos = trees[numAgent];
    // int[] parent = parents[numAgent];
    pathStop[i] = 1;
  }
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
}

void mousePressed(){
  Vec2 mousePos = new Vec2(mouseX, mouseY);
  boolean insideAnyCircle = circleInCircleList(agentRad, circlePos,circleRad,numObstacles,mousePos,2);
  if (mouseButton == LEFT){//set goal Pos to all be same
    boolean insideAnyGoal = circleInAgentOrGoal(agentRad,goalPos,goalRad,numAgents,mousePos,2);
    boolean insideAnyPos = circleInAgentOrGoal(agentRad,agentPos,agentRad,numAgents,mousePos,2);
    if(!(insideAnyCircle || insideAnyGoal || insideAnyPos)){
      for (int i = 0; i < numAgents; i++) {
        goalPos[i] = mousePos;
      }
    }
  } else {  
    if (insideAnyCircle) {    // delete an obstacle
     int index = -1;//WILL be updated
     for(int i = 0; i<numObstacles; i++){
       if(mousePos.distanceTo(circlePos[i]) < circleRad[i]){
         index = i;
         break;
       }
     }
     if(index != -1){
       numObstacles--;
       for(int j = index; j < numObstacles; j++){
         circlePos[j] = circlePos[j+1];
         circleRad[j] = circleRad[j+1];
       }
     }
    } else { // add an obstacle
      numObstacles++;
      float rad = (30+20*pow(random(1),3));
      int id = closestNode(mousePos, agentPos, numAgents);
      if(mousePos.distanceTo(agentPos[id]) > agentRad + rad && mousePos.distanceTo(goalPos[id]) > goalRad + rad){
        circlePos[numObstacles-1] = mousePos;
        circleRad[numObstacles-1] = rad;
        generateRandomNodes(numNodes, circlePos, circleRad);
      }
    }
  }
  for (int i = 0; i < numAgents; i++) {
    pathStop[i] = 1;
  }
  paused = true;
  paths = planPath(numAgents, agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}
