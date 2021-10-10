
/////////
// Point Intersection Tests
/////////

//Returns true iff the point, pointPos, is inside the box defined by boxTopLeft, boxW, and boxH
boolean pointInBox(Vec2 boxTopLeft, float boxW, float boxH, Vec2 pointPos){
  Vec2 diff = pointPos.minus(boxTopLeft);
  if (diff.x < boxW && diff.y < boxH) {
    return true;
  }
  return false;
}

//Returns true iff the point, pointPos, is inside a circle defined by center and radius r
// If eps is non-zero, count the point as "inside" the circle if the point is outside, but within the distance eps of the edge
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos, float eps){
  float dist = pointPos.distanceTo(center);
  if (dist < r + eps) {
    return true;
  }
  return false;
}

//Returns true if the point is inside any of the circles defined by the list of centers,"centers", and corisponding radii, "radii".
// As above, count point within "eps" of the circle as inside the circle
//Only check the first "numObstacles" circles.
boolean pointInCircleList(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  int size = centers.length;
  if (size < numObstacles) {
    numObstacles = size;
  }
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (pointInCircle(center, r, pointPos, eps)){
      return true;
    }
  }
  return false;
}

//Returns true iff the point, pointPos, is inside a circle defined by center and radius r
// If eps is non-zero, count the point as "inside" the circle if the point is outside, but within the distance eps of the edge
boolean circleInCircle(float rad, Vec2 center, float r, Vec2 pointPos, float eps){
  float dist = pointPos.distanceTo(center);
  if (dist < r + rad + eps) {
    return true;
  }
  return false;
}

//Returns true if the point is inside any of the circles defined by the list of centers,"centers", and corisponding radii, "radii".
// As above, count point within "eps" of the circle as inside the circle
//Only check the first "numObstacles" circles.
boolean circleInCircleList(float rad, Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  int size = centers.length;
  if (size < numObstacles) {
    numObstacles = size;
  }
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (circleInCircle(rad, center, r, pointPos, eps)){
      return true;
    }
  }
  return false;
}


//Returns true if the point is inside any of the circles defined by the list of centers,"centers", and corisponding radii, "radii".
// As above, count point within "eps" of the circle as inside the circle
//Only check the first "numObstacles" circles.
boolean circleInAgentOrGoal(float rad, Vec2[] centers, float radius, int numAgents, Vec2 pointPos, float eps){
  for (int i = 0; i < numAgents; i++){
    Vec2 center =  centers[i];
 
    if (center == null) return false;
    if (circleInCircle(rad, center, radius, pointPos, eps)){
      return true;
    }
  }
  return false;
}


/////////
// Ray Intersection Tests
/////////

//This struct is used for ray-obstaclce intersection.
//It store both if there is a collision, and how far away it is (int terms of distance allong the ray)
class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

//Constuct a hitInfo that records if and when the ray starting at "ray_start" and going in the direction "ray_dir"
// hits the circle centered at "center", with a radius "radius".
//If the collision is further away than "max_t" don't count it as a collision.
//You may assume that "ray_dir" is always normalized
hitInfo rayCircleIntersect(Vec2 center, float radius, Vec2 ray_start, Vec2 ray_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(ray_start);
 
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = ray_dir.length();        // since ray_dir is normalized
  float b = -2*dot(ray_dir,toCircle); //-2*dot(l_dir,toCircle), negative because we want toCircle to be pointing the opposite way
  float c = toCircle.lengthSqr() - (radius*radius); //different of squared distances
 
  float disc = b*b - 4*a*c; //discriminant
 
  if (disc >= 0){
    //If d is positive we know the line is colliding
    float t1 = (-b - sqrt(disc))/(2*a); //Optimization: we typically only need the first collision!
    float t2 = (-b + sqrt(disc))/(2*a); //Optimization: we only need the first collision
    //println(hit.t,t1,t2);
    if (t1 > 0 && t1 < max_t){
      hit.hit = true;
      hit.t = t1;
    }
    else if (t1 < 0 && t2 > 0){
      hit.hit = true;
      hit.t = t2;
    }
  }
  
  return hit;
}

//Constuct a hitInfo that records if and when the ray starting at "ray_start" and going in the direction "ray_dir"
// hits any of the circles defined by the list of centers,"centers", and corisponding radii, "radii"
//If the collision is further away than "max_t" don't count it as a collision.
//You may assume that "ray_dir" is always normalized
//Only check the first "numObstacles" circles.
hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii, int numObstacles, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  int size = centers.length;
  if (size < numObstacles) {
    numObstacles = size;
  }
  for (int i = 0; i < numObstacles; i++) {
    Vec2 center = centers[i];
    float radius = radii[i];
    hitInfo curHit = rayCircleIntersect(center, radius, l_start, l_dir, max_t);
    if (curHit.hit && curHit.t < hit.t) {
      hit = curHit;
    }
  }
  return hit;
}


hitInfo rayGoalListIntersect(Vec2[] centers, float radius, int numObstacles, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  int size = centers.length;
  if (size < numObstacles) {
    numObstacles = size;
  }
  for (int i = 0; i < numObstacles; i++) {
    Vec2 center = centers[i];
    hitInfo curHit = rayCircleIntersect(center, radius, l_start, l_dir, max_t);
    if (curHit.hit && curHit.t < hit.t) {
      hit = curHit;
    }
  }
  return hit;
}
