//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    
  //////////// make a minkowski sum here/////////////
      float[] minkowski = new float[circleRadii.length];
      for (int r = 0; r < circleRadii.length; r++) {
        minkowski[r] = circleRadii[r] + agentRad;
      }
    boolean insideAnyCircle = pointInCircleList(circleCenters,minkowski,numObstacles,randPos,2);
    // boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,minkowski,numObstacles,randPos,2);
      // insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (30+20*pow(random(1),3));
  }
  circleRad[0] = 30; //Make the first obstacle big
}


Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(agentRad + random(width - 2 * agentRad), agentRad + random(height - 2 * agentRad));
  boolean insideAnyCircle = circleInCircleList(agentRad,circlePos,circleRad,numObstacles,randPos,2);
  boolean insideAnyGoal = circleInAgentOrGoal(agentRad,goalPos,goalRad,numAgents,randPos,2);
  boolean insideAnyPos = circleInAgentOrGoal(agentRad,agentPos,agentRad,numAgents,randPos,2);
  while (insideAnyCircle || insideAnyGoal || insideAnyPos){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = circleInCircleList(agentRad, circlePos,circleRad,numObstacles,randPos,2);
    insideAnyGoal = circleInAgentOrGoal(agentRad,goalPos,goalRad,numAgents,randPos,2);
    insideAnyPos = circleInAgentOrGoal(agentRad,agentPos,agentRad,numAgents,randPos,2);
  }
  return randPos;
}

/*
void pathQuality(){
  Vec2 dir;
  hitInfo hit;
  float segmentLength;
  numCollisions = 9999; pathLength = 9999;
  if (curPath.size() == 1 && curPath.get(0) == -1) return; //No path found  
  
  pathLength = 0; numCollisions = 0;
  
  if (curPath.size() == 0 ){ //Path found with no nodes (direct start-to-goal path)
    segmentLength = startPos.distanceTo(goalPos);
    pathLength += segmentLength;
    dir = goalPos.minus(startPos).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength);
    if (hit.hit) numCollisions += 1;
    return;
  }
  
  segmentLength = startPos.distanceTo(nodePos[curPath.get(0)]);
  pathLength += segmentLength;
  dir = nodePos[curPath.get(0)].minus(startPos).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength);
  if (hit.hit) numCollisions += 1;
  
  
  for (int i = 0; i < curPath.size()-1; i++){
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    segmentLength = nodePos[curNode].distanceTo(nodePos[nextNode]);
    pathLength += segmentLength;
    
    dir = nodePos[nextNode].minus(nodePos[curNode]).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[curNode], dir, segmentLength);
    if (hit.hit) numCollisions += 1;
  }
  
  int lastNode = curPath.get(curPath.size()-1);
  segmentLength = nodePos[lastNode].distanceTo(goalPos);
  pathLength += segmentLength;
  dir = goalPos.minus(nodePos[lastNode]).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[lastNode], dir, segmentLength);
  if (hit.hit) numCollisions += 1;
}
*/
