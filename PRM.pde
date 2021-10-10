import javafx.util.Pair;
import java.util.Comparator;
import java.util.PriorityQueue;

//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      
      // add a limit to distance between nodes
      // if (distBetween > 100) continue;
      
  //////////// make a minkowski sum here/////////////
      float[] minkowski = new float[radii.length];
      for (int r = 0; r < radii.length; r++) {
        minkowski[r] = radii[r] + agentRad;
      }
      
      hitInfo circleListCheck = rayCircleListIntersect(centers, minkowski, numObstacles, nodePos[i], dir, distBetween);
      // hitInfo goalListCheck = rayGoalListIntersect(goalPos, agentRad + goalRad, numAgents, nodePos[i], dir, distBetween);
      // hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit) { // && !goalListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}


//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}


//This is probably a good idea and you should use it...
int secondNode(int id, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    if (i == id) continue;
    float dist = nodePos[i].distanceTo(nodePos[id]);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}


ArrayList<ArrayList<Integer>> planPath(int numAgents, Vec2[] agentPos, Vec2[] goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<ArrayList<Integer>> paths = new ArrayList<ArrayList<Integer>>();
  
  for (int i = 0; i < numAgents; i++) {
    ArrayList<Integer> path = new ArrayList();
    
    int startID = numNodes;
    int goalID = numNodes+1;
    nodePos[startID] = agentPos[i];
    nodePos[goalID] = goalPos[i];
    
    connectNeighbors(centers, radii, numObstacles, nodePos, numNodes+2);
    path = runAstar(nodePos, numNodes+2, startID, goalID);
    // path = runBFS(nodePos, numNodes+2, startID, goalID);
    
    paths.add(path);
  }
  return paths;
}

//runAstar (A start algorithm)
ArrayList<Integer> runAstar(Vec2[] nodePos, int numNodes, int startID, int goalID){
  // use priority queue instead of arraylist
  PriorityQueue <Pair <Float,Integer> > fringe = new PriorityQueue< Pair <Float,Integer> >(numNodes, new Comparator<Pair <Float,Integer>>() {
    public int compare(Pair <Float,Integer> lhs, Pair <Float,Integer> rhs) {
      if (lhs.getKey() > rhs.getKey()) return +1;
      if (lhs.getKey() == rhs.getKey()) return 0;
      return -1;
    }
  });
  // ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(new Pair(nodePos[startID].distanceTo(nodePos[goalID]), startID));
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  Pair <Float,Integer> node;
  while (fringe.size() > 0){
    node = fringe.poll();
    float prevScore = node.getKey();
    int currentNode = node.getValue();
    float dist = prevScore - nodePos[currentNode].distanceTo(nodePos[goalID]);    // subtract heuristic to get distance traveled
    // fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        // fringe.add(neighborNode);
        float gn = nodePos[neighborNode].distanceTo(nodePos[currentNode]);        // calculate new distance made
        float hn = nodePos[neighborNode].distanceTo(nodePos[goalID]);             // add heuristic
        float score = dist + gn + hn;
        fringe.add(new Pair(score, neighborNode));
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode != -1){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  return path;
}

//BFS (Breadth First Search)
ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  return path;
}
