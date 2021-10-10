# CrowdSimulation
I used Processing to simulate multiple agents navigating an obstacle-filled environment using A* search on a graph of networks created through probabilistic roadmap (PRM).

## Demos
Press `space` to start simulation and mouse left click to assign a singular goal for multiple agents     |  Press `space` to pause the simulation, mouse right click to remove and add obstacles, and `r` key to recreate the environment
:-------------------------:|:-------------------------:
![part1](https://user-images.githubusercontent.com/77593187/136684890-053cc9ee-abc5-4442-9d85-039ab7518a74.gif) | ![part2](https://user-images.githubusercontent.com/77593187/136684893-95ff4eba-df2c-4689-8600-914b5f7b1073.gif)

## Commands
Users can interact with our simulation in a variety of ways"
* Space bar  -  freeze/unfreeze. Agents will remain in place until the space bar is pressed
* 'r' key - reset simulation, new obstacles, agents, and paths will be spawned
* Mouse left click - reassign goal position. If multiple agents are in the scene,  all agents will now move towards this goal. 
* Mouse right click - Add/remove obstacles. If an existing obstacle is clicked, it will be removed. Otherwise, a new obstacle will be added. Simulation must be paused when editing obstacles.

## Challenges
One challenge I had for the PRM/A* implementation was the crowd simulation. Dubbed the "Beyblade Phenomenon", the spaceships would take the same edge or go towards the same PRM node in a relatively similar slope. As a result, the TTC forces would be large in magnitude and point in opposite, parallel directions, causing the spaceships to ricochet off of each other like a sweet Beyblade battle. To avoid this I tested out adding a force perpendicular to the TTC force when two spaceships were in deadlock, which ended up causing the spaceships to swirl around each other infinitely due to the spaceships still trying to reach the same PRM node, creating a "sticky" force. Ultimately, I lessened each spaceship's path's need to be similar to their respective A* planned path, put a constraint on the acceleration to avoid ricochet, and took inspiration from boid forces by using separation forces to keep the spaceships away from each other. 

## Image Sources:
Asteroid: https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.nicepng.com%2Fmaxp%2Fu2w7e6t4w7o0i1q8%2F&psig=AOvVaw1i2WecOsKujau9mNYq7Xnc&ust=1633477982214000&source=images&cd=vfe&ved=0CAkQjRxqFwoTCKCsrev5sfMCFQAAAAAdAAAAABAJ
Planet_1: https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.pngaaa.com%2Fdetail%2F1381749&psig=AOvVaw3jCniEyZCJ2eM8sH-ub5dt&ust=1633478053948000&source=images&cd=vfe&ved=0CAkQjRxqFwoTCODZlJH6sfMCFQAAAAAdAAAAABAD 
Planet_2: https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.pngegg.com%2Fen%2Fsearch%3Fq%3Dcartoon%2Bplanet&psig=AOvVaw3jCniEyZCJ2eM8sH-ub5dt&ust=1633478053948000&source=images&cd=vfe&ved=0CAkQjRxqFwoTCODZlJH6sfMCFQAAAAAdAAAAABAK 
Planet_3: https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.pngkey.com%2Fdetail%2Fu2y3q8q8u2w7q8o0_solar-system-planet-cartoon-cartoon-planet%2F&psig=AOvVaw3jCniEyZCJ2eM8sH-ub5dt&ust=1633478053948000&source=images&cd=vfe&ved=0CAkQjRxqFwoTCODZlJH6sfMCFQAAAAAdAAAAABAQ 
Planet_4: https://listimg.pinclipart.com/picdir/s/530-5302363_planeten-clipart-spaceclip-transparent-green-planet-clipart-png.png 
Planet_5: https://e7.pngegg.com/pngimages/678/600/png-clipart-yellow-planet-space-asteroids-cartoon-saturn.png 
Ship: https://img.favpng.com/4/7/21/cartoon-unidentified-flying-object-clip-art-png-favpng-ppvq6wp3rnQh3RwUKyNprSsRC.jpg 
