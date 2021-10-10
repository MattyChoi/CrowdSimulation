//Update agent positions & velocities based acceleration
void moveAgent(float dt){
  //Compute accelerations for every agents
  for (int i = 0; i < numAgents; i++){
    agentAcc[i] = computeAgentForces(i);
  }
  //Update position and velocity using (Eulerian) numerical integration
  for (int i = 0; i < numAgents; i++){
    agentVel[i].add(agentAcc[i].times(dt));
    agentPos[i].add(agentVel[i].times(dt));
  }
}


Vec2 computeAgentForces(int id){
  Vec2 acc = new Vec2(0,0);
  
  // for cohesion
  Vec2 avgPos = new Vec2(0,0);
  int countAtt = 0;
  
  // for alignment
  Vec2 avgVel = new Vec2(0,0);
  int countAli = 0;
  
  //get the path
  ArrayList<Integer> curPath = paths.get(id);
  if (curPath.size() < 2) return acc;
  
  // change direction of agent
  if (agentPos[id].distanceTo(nodePos[curPath.get(pathStop[id])]) < agentRad + epsi) 
    pathStop[id]++;
    
  // if already at goal, stop moving the agent
  if (pathStop[id] == curPath.size()) pathStop[id] = curPath.size()-1;
  
  // find goal force
  Vec2 goalVel;
  int goal = curPath.get(pathStop[id]);
  if (goal == numNodes+1) {
    goalVel = goalPos[id].minus(agentPos[id]);
    if (goalVel.length() > goalSpeed) goalVel.setToLength(goalSpeed);    // prevent jitter
  } else {
    goalVel = nodePos[goal].minus(agentPos[id]);
    goalVel.setToLength(goalSpeed);    // prevent jitter
  }
  Vec2 goalForce = goalVel.minus(agentVel[id]);
  acc.add(goalForce.times(k_goal));
  
  float ttc;
  // find collision avoidance force
  Vec2 avoidForce = new Vec2(0,0);
  for (int i = 0; i < numAgents; i++) {
    if (i != id) {
      ttc = computeTTC(agentPos[id], agentVel[id], agentRad, agentPos[i], agentVel[i], agentRad);
      if (ttc != -1) {
        Vec2 curFuture = agentPos[id].plus(agentVel[id].times(ttc)); 
        Vec2 nbrFuture = agentPos[i].plus(agentVel[i].times(ttc));
        avoidForce = curFuture.minus(nbrFuture).normalized();
        
        /*
        if (1/ttc > 10 && avoidForce.x > 0) {
          Vec2 perp = new Vec2(avoidForce.y, -avoidForce.x);
          acc.add(perp.times(0.5));
        }
        */
        
        acc.add(avoidForce.times(k_avoid * 1/(ttc)));
      }
          
      /*
      // add avoid forces from different goals?
      ttc = computeTTC(agentPos[id], agentVel[id], agentRad, goalPos[i], new Vec2(0,0), goalRad);
      if (ttc != -1) {
        Vec2 curFuture = agentPos[id].plus(agentVel[id].times(ttc)); 
        avoidForce = curFuture.minus(goalPos[i]).normalized();
        acc.add(avoidForce.times(k_avoid * 1/(ttc)));
      }
      */
      
      // add boids forces
      float dist = agentPos[i].distanceTo(agentPos[id]);
      
      // don't account for particles that are far away in separation force 
      if (dist < .01 || dist > 50) continue; //TODO: Why do we not need to skip i == j? because dist < 0.01 when i == j
      
      // attraction count
      if (dist < 60 && dist > 0){    // looks for neighbors in a radius of 60
        avgPos.add(agentPos[i]);
        countAtt += 1;
      }
      
      // alignment count
      if (dist < 40 && dist > 0){    // looks for neighbors in a radius of 40
        avgVel.add(agentVel[i]);
        countAli += 1;
      }
      
      //Seperation force (push away from each neighbor if we are too close)
      Vec2 seperationForce = agentPos[id].minus(agentPos[i]).normalized();
      seperationForce.setToLength(2*k_avoid/pow(dist,2));    // force gets smaller exponentially as dist is larger
      acc = acc.plus(seperationForce);
        
    }
    
  }  
  /*
  //Atttraction force (move towards the average position of our neighbors
  avgPos.mul(1.0/countAtt);  // take average
  if (countAtt >= 1){
    Vec2 attractionForce = avgPos.minus(agentPos[id]);
    attractionForce.normalize();
    attractionForce.mul(1.0);      // change constant to change attraction force strength
    attractionForce.clampToLength(10);
    acc = acc.plus(attractionForce);
  }
  */
    
  //Alignment force (maintain a relatively similar velocity as nearby boids)
  avgVel.mul(1.0/countAli);
  if (countAli >= 1){
    Vec2 towards = avgVel.minus(agentVel[id]);
    towards.normalize();
    acc = acc.plus(towards.times(2));
  }
  
  //Wander force
  Vec2 randVec = new Vec2(1-random(2),1-random(2));
  acc = acc.plus(randVec.times(10.0)); 
  
  // add avoidance force from obstacles
  for (int i = 0; i < numObstacles; i++) {
    if (i != id) {
      ttc = computeTTC(agentPos[id], agentVel[id], agentRad, circlePos[i], new Vec2(0,0), circleRad[i]);
      if (ttc != -1 && ttc < 1) {
        Vec2 curFuture = agentPos[id].plus(agentVel[id].times(ttc)); 
        avoidForce = curFuture.minus(circlePos[i]).normalized();
        acc.add(avoidForce.times(k_avoid * 1/ttc));
      }
    }
  }
  
  //limits the amount the agents can accelerate
  acc.clampToLength(maxAcc);
  return acc;
       //- Find the ttc between agent "id" and it's neighbor "j" (make sure id != j)
       //- If the ttc is negative (or there is no collision) then there is no avoidance force w.r.t agent j
       //- Predict where both agent's well be at the moment of collision (time = TTC)
       //  ie., A_future = A_current + A_vel*ttc; B_future + B_current + B_vel*ttc
       //- Find the relative vector between the agents at the moment of collision, normalize it
       //  i.e, relative_future_direction = (A_future - B_future).normalized()
       //- Compute the per-agent avoidance force by scaling this direction by k_avoid and
       //  dividing by ttc (smaller ttc -> more urgent collisions)
       //  acc += k_avoid * (1/ttc) * relative_future_direction
}


//Return at what time agents 1 and 2 collide if they keep their current velocities
// or -1 if there is no collision.
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  //Compute sum of radii
  float radii = radius1 + radius2;
 
  //Compute displacement vector pointing pos1 to pos2
  Vec2 diff = pos2.minus(pos1);
  
  //Compute velocity vector pointing from vel1 to vel2
  Vec2 velDiff = vel2.minus(vel1);
 
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = velDiff.lengthSqr();
  float b = 2*dot(velDiff,diff); //-2*dot(l_dir,toCircle)
  float c = diff.lengthSqr() - (radii*radii); //different of squared distances
 
  float d = b*b - 4*a*c; //discriminant
 
  if (d >=0 ){
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision!
    if (t >= 0) return t;
    return -1;
  }
 
  return -1; //We are not colliding, so there is no good t to return
}
