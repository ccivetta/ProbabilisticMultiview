%% Initialize UR
ur = urRTDEClient('10.0.0.100');

%% Initialize simulation
sim = URsim;
sim.Initialize('UR10');
sim.Home;

%% Load joint angles - planar 
x = 0;
for i = 1:7
    q(i,:) = deg2rad([-.01 -140.55 100.8 -123.89 0 30-x]);
    x = x + 10;
end

x = 0;
for i = 8:14
    q(i,:) = deg2rad([-.01 -154.43 126.67 -129.01 0 30-x]);
    x = x + 10;
end

x = 0;
for i = 15:21
    q(i,:) = deg2rad([0 -166.42 147.22 -132.68 0 30-x]);
    x = x + 10;
end

x = 0; 
for i = 22:28
    q(i,:) = deg2rad([0 -175.86 160.74 -134.81 0 0-x]);
    x = x + 10;
end

x = 0; 
for i = 29:35
    q(i,:) = deg2rad([0 -92.5 130.56 -145.73 0 -35-x]);
    x = x + 10;
end

x = 0; 
for i = 36:42
    q(i,:) = deg2rad([0 -67.13 138.7 -157.57 0 -60-x]);
    x = x + 10;
end

x = 0; 
for i = 43:49
    q(i,:) = deg2rad([0 -101.61 113.03 -134.95 0 -25-x]);
    x = x + 10;
end

x = 0; 
for i = 50:56
    q(i,:) = deg2rad([0 -99.77 87.42 -124.64 0 -10-x]);
    x = x + 10;
end

x = 0; 
for i = 57:63
    q(i,:) = deg2rad([0 -161.08 100.31 -104.88 0 20-x]);
    x = x + 10;
end

x = 0; 
for i = 64:70
    q(i,:) = deg2rad([0 -180.5 122.94 -105.15 0 12-x]);
    x = x + 10;
end

x = 0; 
for i = 71:77
    q(i,:) = deg2rad([0 -216.11 135.17 -94.31 0 30-x]);
    x = x + 10;
end

x = 0; 
for i = 78:84
    q(i,:) = deg2rad([0 -248.08 142.81 -82.43 0 40-x]);
    x = x + 10;
end
%% Save - planar
save('URJoingAngles_Planar',"q");

%% Check joints
for i = 1:size(q,1)
    sim.Joints = q(i,:);
    pause(0.5);
end

%% Run on real robot
for i = 1:size(q,1)
    %ur.sendJointConfigurationAndWait(q(i,:));
    if mod(i-1,7) == 0
    ur.sendJointConfigurationAndWait(q(i,:),...
        'EndTime',3.0,'Velocity',2*pi,'Acceleration',4*pi);
    else
        ur.sendJointConfigurationAndWait(q(i,:),...
        'EndTime',1.0,'Velocity',2*pi,'Acceleration',4*pi);
    end
    dq = ur.readJointVelocity;
    while any(dq ~= 0)
        % Robot is moving
        fprintf('[ ')
        fprintf('%f ',dq);
        fprintf(']\n');
        pause(1/100);
        dq = ur.readJointVelocity;
    end
end

%% Load joint angles - datacapture 
q = [];
q(1,:) = deg2rad([35.72 -10.64 -78.5 -84.63 -33.25 0.45]);
q(2,:) = deg2rad([30.24 14.75 -86.73 -109.41 -27.88 0.45]);
q(3,:) = deg2rad([65.6 7.63 -77.31 -109.4 -61.25 0.2]);
q(4,:) = deg2rad([103.61 9.42 -91.74 -96.36 -107.9 0.13]);
q(5,:) = deg2rad([93.13 51.1 -141.6 -96.49 -107.45 0.13]);
q(6,:) = deg2rad([147.03 7.16 -90.0 -96.55 -163.38 0.14]);
q(7,:) = deg2rad([152.72 1.25 -50.68 -130.16 -165.73 0.14]);
q(8,:) = deg2rad([161.12 33.85 -72.79 -149.39 -179.76 0.14]);
q(9,:) = deg2rad([189.17 -0.01 -53.56 -118.78 -200.81 0.17]);
q(10,:) = deg2rad([191.04 -21.27 -47.28 -104.55 -196.47 0.17]);
%% Save - datacapture
save('URJoingAngles_dataCapture10',"q");

%% Check joints
for i = 1:size(q,1)
    sim.Joints = q(i,:);
    pause(0.5);
end

%% Run on real robot
for i = 1:size(q,1)
    ur.sendJointConfigurationAndWait(q(i,:),...
        'EndTime',3.0,'Velocity',2*pi,'Acceleration',4*pi);
end

%% Load joint angles - calibration 
q = [];
q(1,:) = deg2rad([73.76 -15.1 4.08 -170.4 -69.14 0.21]);
q(2,:) = deg2rad([56.74 -38.48 28.25 -170.42 -53.18 0.21]);
q(3,:) = deg2rad([62.64 -38.5 38.33 -160.27 -53.18 27.91]);
q(4,:) = deg2rad([98.96 -34.24 -21.06 -100.0 -102.49 -3.15]);
q(5,:) = deg2rad([88.93 -61.32 21.88 -120.66 -103.06 18.27]);
q(6,:) = deg2rad([39.47 -45.04 39.65 -169.52 -20.98 17.51]);
q(7,:) = deg2rad([77.95 -45.19 45.52 -178.57 -91.21 33.25]);
q(8,:) = deg2rad([87.34 11.31 -2.69 -195.5 -91.53 -38.23]);
q(9,:) = deg2rad([89.49 11.33 -10.35 -187.35 -91.43 17.18]);
q(10,:) = deg2rad([80.01 11.12 -18.89 -187.36 -91.42 17.18]);
q(11,:) = deg2rad([81.15 -41.71 45.48 -187.59 -91.42 34.11]);
q(12,:) = deg2rad([92 -47.66 59.76 -187.31 -91.42 56.44]);
q(13,:) = deg2rad([81.5 -21.54 25.71 -187.36 -91.43 -8.03]);
q(14,:) = deg2rad([91.67 -50.51 3.38 -125.03 -91.43 -8.02]);
q(15,:) = deg2rad([92.47 -25.86 -10.72 -124.42 -103.9 34.01]);
q(16,:) = deg2rad([87.56 -13.88 22.47 -195.76 -91.42 33.99]);
q(17,:) = deg2rad([87.54 5.57 22.53 -224.87 -91.39 -33.03]);
q(18,:) = deg2rad([83.22 5.42 -23.69 -158.86 -91.39 3.65]);
%% Save - datacapture
save('URJoingAngles_calibration',"q");
%% Check joints
for i = 1:size(q,1)
    sim.Joints = q(i,:);
    pause(0.5);
end

%% Run on real robot
for i = 1:size(q,1)
    ur.sendJointConfigurationAndWait(q(i,:),...
        'EndTime',3.0,'Velocity',2*pi,'Acceleration',4*pi);
end