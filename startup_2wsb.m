clc;clear all;

%% System Parameters
sys.w = 0.44;                               % Width
sys.r_w = 0.1;                              % Radius of Wheel
sys.j_m_th=0.153;                           % Base pitch moment of inertia
sys.j_p_th= 0.125;                          % Pendulum pitch moment of interia    
sys.j_w= 0.013;                             % Rotational inertia of wheel
sys.m_b= 15.747;                            % Base mass
sys.m_w= 5.44;                              % Wheel mass
sys.m_p=4;                                  % Pendulum mass
sys.l= 0.53;                                % Pendulum length
sys.j_psi= 0.576;                           % Base yaw moment of inertia
sys.g= 9.8056;

%% Simulation Parameters 
sim.x0= [0.75 ; 0.65 ;0; 0; 0;0;0];
sim.xf= [2.4; 4.15 ; pi/2; 0; 0;0;0];
sim.obs_num=2; sim.obs_diam = 0.6; 
sim.obs_x= [0.40+0.75;0.30+0.75;0.35+2;0.35+1.5;];
sim.obs_y= [2.2;3.2;2.75;1.25];
sim.tsim=30;
sim.x_min= 0.25;
sim.x_max= 2.75;
sim.y_min=0;
sim.y_max= 4.25;
mpc_init.x0= sim.x0;
mpc_init.x= repmat(sim.x0,pl.N+1,1).';
mpc_init.
%% Planner Initialization - NMPC Planner
pl.Ts = 0.05; % Sampling Rate 
pl.Q  = diag([1;1;0.02]); % Penalty State
pl.N  = 100; % Prediction Horizon
pl.R  = diag([0.5;0.5]); % Penalty Input
pl.Q2 = diag([0.5;0.5]);
pl.Q3= 50;
pl.v_max = 4.5; % Max Forward Vel
pl.v_min = -4.5; % Max Backward Vel
pl.psi_dot_max = 0.785398; % Max AntiClockwise Yawrate
pl.psi_dot_min =-0.785398; % Max Clockwise Yawrate
pl.a_max = 2.9; % Max Accel
pl.a_min = -2.9; % Max Decel
pl.ego_safety_radius= 0.6;
pl.tau_min= -11.5; % Max Reverse Torque
pl.tau_max= -pl.tau_min; % Max Forward Torque
pl.lin_sys= setup_lin_sys(sys);

EXPORT =1; COMPILE =1;
planner_block_setup;

%% Viz Setup - For simulation graphics visualization
viz.w= sys.w;
viz.l= sys.r_w*2+0.02;
viz.marker_h= viz.l;
viz.marker_w= viz.w/4;
viz.map_x= [0, 3, 3, 0, 0];
viz.map_y= [0, 0, 4.5, 4.5, 0];
viz.ang= 0:0.005:2*pi;
viz.obs_circ_x= sim.obs_diam/2*cos(viz.ang);
viz.obs_circ_y= sim.obs_diam/2*sin(viz.ang);
viz.rob_circ_x= (sys.w/2+0.01)*cos(viz.ang);
viz.rob_circ_y= (sys.w/2+0.01)*sin(viz.ang);