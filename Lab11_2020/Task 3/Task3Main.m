%% MIT 2.12 Introduction to Robotics
%% Virtual Lab 11 - Robot Manipulation and Trajectory Planning with Matlab
%% Task 3. Pick and Place using Collision Free trajectories
%% April 2020
% 
%% Deliverables: Please include as part of your writeup the answers to the 
%% questions and the screenshots requested.

%% original example: https://www.mathworks.com/help/nav/ug/motion-planning-with-rrt-for-manipulators.html

% 
% During the last section of the lab, we used Nonlinear Model Predictive Control 
% to generate a collision-free trajectory. We also want to introduce you to another 
% method of collision-free path planning, RRT*.
% 
% The rapidly-exploring random tree (RRT) algorithm is one of the most common 
% and simple path planning methods.Throughout the years a LOT of variations have 
% been developed to optimize the planning process.  This algorithm operates in 
% configuration space. A robot's configuration space is the *space* of possible 
% positions the *robot* may attain. So in our case it would be a data point in 
% 7-dimensional space with each joint position being one of the points in the 
% vector, *q*=[q0,q1,q2,q3,q4,q5,q6]. 
% 
% 
% 
% In RRT, the path is planned by building a tree starting from the configuration 
% of the robot. When a point in the configuration space is randomly sampled, it 
% is checked if that point collides with an obstacle in the space (such as a table). 
% If the sampled point has no collisions, it is then checked if the straight line 
% path between the sampled point and the nearest existing point in the tree has 
% any collisions. If this straight line path has no collisions, the sampled point 
% is added to the tree with the nearest point as its parent node. If there is 
% a collision, this point is thrown out. The algorithm ends when a node is generated 
% within the goal region, or a limit is hit. Don't worry, in this task you don't 
% have to build this algorithm yourself. If you are interested in learning more 
% about this type of path planning, there are a lot of resources available online 
% and here at MIT. You can also check out some of these examples using python: 
% <https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning>
% 
% This task uses a |plannerRRTStar| object to sample states and plan the robot 
% motion.RRTStar (or RRT*) is simply an optimized version of RRT. 
% 
% *0. Initial Setup*
% 
% Load a Kinova Jaco model from the robot library. This particular model includes 
% the three-finger gripper.

clc 
clear all
close all

kin = loadrobot('kinovaJacoJ2S7S300');
ee='j2s7s300_end_effector';


%% Create The Environment
% Using collision object primitives, add a floor, two table tops, and a cylinder 
% can. Specify the size and pose of these objects. The provided image shows the 
% environment created.

floor = collisionBox(1, 1, 0.01);
tabletop1 = collisionBox(0.4,1,0.02);
tabletop1.Pose = trvec2tform([0.3,0,0.6]);
tabletop2 = collisionBox(0.6,0.2,0.02);
tabletop2.Pose = trvec2tform([-0.2,0.4,0.5]);
can = collisionCylinder(0.03,0.16);
can.Pose = trvec2tform([0.3,0.2,0.7]);
can2 = collisionCylinder(0.03,0.16);
can2.Pose = trvec2tform([0.2,0.0,0.7]);

% Set a target psotion for the cans.
targetPos = [0.2,-.2,0.7];
targetPos2=[-0.2,0.35,0.51];
%% Customize The State Space For Manipulator
% The Kinova arm has ten degrees of freedom (DoFs), with the last three DoFs 
% corresponding to the fingers. Only use the first seven DoFs for the planning 
% and keep the fingers at zero configuration (open wide). An |ExampleHelperRigidBodyTreeStateSpace| 
% state space is created to represent the configuration space (joint space). |ExampleHelperRigidBodyTreeStateSpace| 
% samples feasible states for the robot arm. The |sampleUniform| function of the 
% state space alternates between the following two sampling strategies with equal 
% probability:   
%% 
% * Uniformly random sample the end effector pose in the *Workspace Goal Region* 
% around the reference goal pose, then map it to the joint space through inverse 
% kinematics. Joint limits are respected.
% * Uniformly random sample in the joint space. Joint limits are respected.
%% 
% The first sampling strategy helps guide the RRT planner towards the goal region 
% in the task space so that RRT can converge to a solution faster instead of getting 
% lost in the seven DoF joint space.
% 
% Using *Workspace Goal Region* (WGR) instead of single goal pose increases 
% the chance of finding a solution by biasing samples to the goal region. WGR 
% defines a continuum of acceptable end-effector poses for certain tasks. For 
% example, the robot can approach from multiple directions to grasp a cup of water 
% from the side, as long as it doesn't collide with the environment. The concept 
% of WGR is first proposed by Dmitry Berenson et al [1] in 2009. This algorithm 
% later evolved into *Task Space Regions* [2]. A WGR consists of three parts:
%% 
% * |Twgr_0| - The reference transform of a WGR in world ({0}) coordinates
% * |Te_w| - The end-effector offset transform in the {w} coordinates, {w} is 
% sampled from WGR
% * |Bounds| - A 6-by-2 matrix of bounds in the WGR reference coordinates. The 
% first three rows of |Bounds| set the allowable translation along the x, y, and 
% z axes (in meters) respectively and the last three set the allowable rotations 
% about the allowable rotations about the x, y, and z axes (in radians). Note 
% that the Roll-Pitch-Yaw (RPY) Euler angles are used as they can be intuitively 
% specified.
%%  Create state space 


ss = ExampleHelperRigidBodyTreeStateSpace(kin);
ss.EndEffector = ee;

%% Customize The State Validator
% The customized state validator, |ExampleHelperValidatorRigidBodyTree|, provides 
% rigid body collision checking between the robot and the environment. This validator 
% checks sampled configurations and the planner should discard invalid states.

sv = ExampleHelperValidatorRigidBodyTree(ss);

% Add obstacles in the environment
addFixedObstacle(sv,tabletop1, 'tabletop1', [71 161 214]/256);
addFixedObstacle(sv,tabletop2, 'tabletop2', [71 161 214]/256);
addFixedObstacle(sv,can, 'can', 'r');
addFixedObstacle(sv,can2, 'can2', 'g');
addFixedObstacle(sv,floor, 'floor', [1,0.5,0]);

% Skip collision checking for certain bodies for performance
skipCollisionCheck(sv,'root'); % root will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_link_base'); % base will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_end_effector'); % this is a virtual frame

%% Define workspace goal region
%You can define and concatenate multiple WGRs in one planning problem. In this 
% example, only one WGR is allowed.

% Create state space and set workspace goal regions (WGRs)

% Define the workspace goal region (WGR)
% This WGR tells the planner that the can shall be grasped from
% the side and the actual grasp height may wiggle at most 1 cm.

% This is the orientation offset between the end-effector in grasping pose and the can frame
R = [0 0 1; 1 0 0; 0 1 0]; 
%R=axang2tform([0 1 0 pi]); you can use this function to help create
%different rotation matricies

pt2=zeros(4,3);
pt2(:,end+1)=[0;0;.12;1];
Tw_0 = can.Pose;%+pt2 ; %position in
% Tw_0= [1.0000         0         0    0.2000
%          0    1.0000         0         0
%          0         0    1.0000    0.65000
%          0         0         0    1.0000];
Te_w = rotm2tform(R);
%Te_w=axang2tform([0 1 0 -pi/2]) ;
%Te_w=axang2tform([0 1 0 pi]) ;
bounds = [0 0;       % x
          0 0;       % y
          0 0.01;    % z
          0,0;       % R
          0,0;       % P
         -pi pi];    % Y
     
setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);


%% Plan The Grasp Motion
% Use the |plannerRRT| object with the customized state space and state validator 
% objects. Specify the start and goal configurations by using |inverseKinematics| 
% to solve for configrations based on the end-effector pose.  Specify the |GoalReachedFcn| 
% using |exampleHelperIsStateInWorkspaceGoalRegion|, which checks if a path reaches 
% the goal region.
% 
%% *Note: Sometimes the planning takes a long time (matlab isn't really known 
% for its speed). As long as your environment isn't changing (when you're playing 
% around with visualizing) I would recommend saving the planned states (see lines 
% 272, 334,385 and 440) and using the matlab command window for visualizing. Or for 
% example when you have one path planned but you want to plan the second one you 
% can save the first set of states as a unique variable, then just make sure to 
% not clear all of your variables. This way you don't have to replan it. On
% my computer it took about 7 minutes total to plan all four reaching motions. 
% in the project folder I have included pathplan.mat which is a file that contains
% the results of when I ran the planning on my computer. You cal load this file
% to quickly visualize what is supposed to happen*


%%


% Set the validation distance
sv.ValidationDistance = 0.01;
% Set random seeds for repeatable results
rng(0,'twister') % 0

% Compute the reference goal configuration. Note this is applicable only when goal bias is larger than 0. 
Te_0ref = Tw_0*Te_w; % Reference end-effector pose in world coordinates, derived from WGR
ik = inverseKinematics('RigidBodyTree',kin);
refGoalConfig = ik(ss.EndEffector,Te_0ref,ones(1,6),homeConfiguration(ss.RigidBodyTree));

% Compute the initial configuration (end-effector is initially under the table)
T = Te_0ref;
T(1,4) = 0.3;
T(2,4) = 0.0;
T(3,4) = 0.4;
initConfig = ik(ss.EndEffector,T,ones(1,6),homeConfiguration(ss.RigidBodyTree));
% initConfig=homeConfiguration(ss.RigidBodyTree);

ss.UseConstrainedSampling = false;

% Create the planner from previously created state space and state validator
planner = plannerRRTStar(ss,sv);

% If a node in the tree falls in the WGR, a path is considered found.
planner.GoalReachedFcn = @exampleHelperIsStateInWorkspaceGoalRegion;

% Set the max connection distance.
planner.MaxConnectionDistance = 0.5;

% With WGR, there is no need to specify a particular goal configuration (use
% initConfig to hold the place).
% As a result, one can set GoalBias to zero.
planner.GoalBias = 0;


%% Draw robot before planning, just to visualize part of the example

% Draw the robot in its initial config.
figure
ax = show(kin,refGoalConfig);
zlim(ax, [-0.03, 1.4])
xlim(ax, [-1, 1])
ylim(ax, [-1, 1])
% Render the environment.
hold on
showObstacles(sv, ax);
view(146, 33)
camzoom(1.5)

%draw the robot in its reference goal config
figure
ax = show(kin,initConfig);
zlim(ax, [-0.03, 1.4])
xlim(ax, [-1, 1])
ylim(ax, [-1, 1])
% Render the environment.
hold on
showObstacles(sv, ax);
view(146, 33)
camzoom(1.5)


% add a break point here if you want to inspect the visualization before moving on
close all
%%  Plan the motion
disp('planning')
tic
[pthObj,solnInfo] = plan(planner,initConfig, initConfig);
toc

%% Visualize The Grasp Motion
% The found path is first smoothed through a recursive corner-cutting strategy 
% before the motion is animated.

% Smooth the path.
interpolate(pthObj,100);
newPathObj = exampleHelperPathSmoothing(pthObj,sv);
interpolate(newPathObj,200);

figure
states1 = newPathObj.States;

% Draw the robot.
ax = show(kin,states1(1,:));
zlim(ax, [-0.03, 1.4])
xlim(ax, [-1, 1])
ylim(ax, [-1, 1])


% Render the environment.
hold on
showObstacles(sv, ax);
view(146, 33)
camzoom(1.5)

% Show the motion.
for i = 2:length(states1)
    show(kin,states1(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end


q1 = states1(i,:);

% Grab the can.
q1 = exampleHelperEndEffectorGrab(sv,'can',q1, ax);


%% draw the target positions for the cylinders
%targetPos = [0.2,0,0.7];
%targetPos2 = [.3,0.3,0.71];
exampleHelperDrawHorizontalCircle(targetPos,0.02,'r',ax);
exampleHelperDrawHorizontalCircle(targetPos2,0.02,'g',ax);
%% *Question:*
% What does R do in this case? (see line 145) What happens if you 
% change R? Are there other Rotation matricies that will give us a good starting 
% configuration? Can you approach the cylinder from another angle (such as vertically) 
% instead of horizontally? Experiment with changing R and include a screenshot 
% and a few thoughts.
%% Plan The Move Motion
% During the move motion, keep the cylinder can level at all time to avoid spill. 
% Specify an additional constraint on the interim manipulator configurations for 
% the RRT planner. Turn the constraint on by setting the |UseConstrainedSampling| 
% property to true.

Tw_0 = trvec2tform(targetPos+[0,0,0.08]); 
Te_w = rotm2tform(R);
bounds =  [0 0;       % x
           0 0;       % y
           0 0;       % z
           0 0;       % R
           0 0;       % P
          -pi pi];    % Y
setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);
ss.UseConstrainedSampling = true;
planner.MaxConnectionDistance = 0.05;
disp('planning')
tic
[pthObj2,~] = plan(planner,q1,q1);
toc

%visualize the placing motion
states2 = pthObj2.States;

view(ax, 152,45)
[m,n]=size(states2);
for i = 2:m
    show(kin,states2(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end
q2 = states2(i,:);

% let go of the can
q2 = exampleHelperEndEffectorRelease(sv,q2,ax);

%%  *Question:*
% What happens if you change ss.UseConstrainedSampling to false?  (see like
% 321. Describe the difference in motion and discuss what is happening during the movement. 
% Does it have anything to do with bounds?
% 
% *Question:* 
% What happens if you vary the bounds? Is the solution any faster to compute? Slower? Why might that be?


%% Plan a Second Grasp Motion
% We can pick and place a second cylinder in basically the same way!

rng(0,'twister') % 0
R2 = R; 
Tw_02 = can2.Pose; %position in
Te_w2 = rotm2tform(R2);
bounds2 = [0 0;       % x
          0 0;       % y
          0 0.01;    % z
          0 0;       % R
          0 0;       % P
         -pi pi];    % Y
     
setWorkspaceGoalRegion(ss,Tw_02,Te_w2,bounds2);
Te_0ref2 = Tw_02*Te_w2; % Reference end-effector pose in world coordinates, derived from WGR
refGoalConfig = ik(ss.EndEffector,Te_0ref2,ones(1,6),q2);
initConfig = q2; %use last state from prior movement
exampleHelperDrawHorizontalCircle(targetPos,0.02,'r',ax);
exampleHelperDrawHorizontalCircle(targetPos2,0.02,'g',ax);
disp('planning')
tic
[pthObj3,solnInfo] = plan(planner,initConfig, initConfig);
toc
%% Visualize The Second Grasp Motion
interpolate(pthObj3,100);
newPathObj3 = exampleHelperPathSmoothing(pthObj3,sv);
interpolate(newPathObj3,200);

states3 = newPathObj3.States;

% Draw the robot.
% figure
clf
ax = show(kin,states3(1,:));
zlim(ax, [-0.03, 1.4])
xlim(ax, [-1, 1])
ylim(ax, [-1, 1])
% 
 exampleHelperDrawHorizontalCircle(targetPos,0.02,'r',ax);
 exampleHelperDrawHorizontalCircle(targetPos2,0.02,'g',ax);
% Render the environment.
hold on
showObstacles(sv, ax);
view(146, 33)
camzoom(1.5)


% Show the motion.
for i = 2:length(states3)
    show(kin,states3(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end


q3 = states3(i,:);

% Grab the can.
q3= exampleHelperEndEffectorGrab(sv,'can2',q3, ax);

view(ax, 152,45)

%% Plan The Second Move Motion

Tw_03 = trvec2tform(targetPos2+[0,0,0.08]); 
R3 =R;
Te_w3 = rotm2tform(R3);

bounds =  [0 0;       % x
           0 0;       % y
           0 0;       % z
           0 0;       % R
           0 0;       % P
          -pi pi];    % Y
setWorkspaceGoalRegion(ss,Tw_03,Te_w3,bounds);
ss.UseConstrainedSampling = false;
planner.MaxConnectionDistance = 0.05;
disp('planning')
exampleHelperDrawHorizontalCircle(targetPos,0.02,'r',ax);
exampleHelperDrawHorizontalCircle(targetPos2,0.02,'g',ax);

tic
[pthObj4,~] = plan(planner,q3,q3);
toc
states4 = pthObj4.States;

%% 
% Visualize the motion.

view(ax, 152,45)
for i = 2:length(states4)
    show(kin,states4(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end
q4 = states4(i,:);

% let go of the can
q4= exampleHelperEndEffectorRelease(sv,q4,ax);

%%  *Question:*
% Try adding additional/different cylinders to pick up as well as additional 
% collision objects to avoid! There are collision boxes, cylinders and spheres. 
% Include a few screen shots of your new environment.
