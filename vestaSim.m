clc;
clear;

% Initial Conditions
% Velocity
v=[0 0 120000]; %m/s
% Position
p=[0 500000 0]; %meters
% Angular Rate
w=0.01871958; %deg/s
n=50;
[X Y Z]=sphere(n);
X=265000*X;
Y=265000*Y;
Z=265000*Z;
for i=1:length(X(:))
    C(i)=vestaGravCart(X(i),Y(i),Z(i));
end
C=reshape(C,n+1,n+1);

import gtsam.*

dt=.1;
figure;     
hold on;

% Realistic Camera Calibration
K = Cal3_S2(1000,1000,0,320,240);
   
% GTSAM Data Structures
factors = gtsam.NonlinearFactorGraph;
initial = gtsam.Values;

% Noise Models
noiseModels.measurement = noiseModel.Isotropic.Sigma(2, 5.0);
noiseModels.pose = noiseModel.Diagonal.Sigmas([10000 10000 10000 0.1 0.1 0.1]');

proj_samp = Sampler(noiseModels.measurement, rand);
pose_samp = Sampler(noiseModels.pose, rand);

pose_id = 0;
for i=1:240
    g=vestaGravCart(p(i,1),p(i,2),p(i,3));
    % Velocity
    v(i+1,:)=v(i,:)-g*dt*p(i,:)/norm(p(i,:));
    p(i+1,:)=p(i,:)+v(i,:)*dt;
    p(i+1,:)=[cosd(w) -sind(w) 0; sind(w) cosd(w) 0; 0 0 1]*p(i+1,:)';
    
    [cam, points] = generateMeasurement(p(i+1,:), X, Y, Z, K);
    sym_cam = symbol('x',pose_id);
    cam_noise = pose_samp.sample;
    cam_point = Point3(cam_noise(1:3));

    cam_noise_pose = Pose3(Rot3, cam_point);

    pose = cam.pose.compose(cam_noise_pose);
    initial.insert(sym_cam, pose);
    
    for j=1:length(points)
        % Points [X Y Z dist id]
        proj = cam.project(Point3(points(j,1), points(j,2), points(j,3)));
        
        % Corrupt the proj measurement
        proj_noise = proj_samp.sample;
        

        proj = proj.compose(Point2(proj_noise));
        sym_proj = symbol('l', points(j,5));
        
        if(~initial.exists(sym_proj))
            initial.insert(sym_proj, Point3(points(j,1), points(j,2), points(j,3)));
        end
        
        fact = gtsam.GenericProjectionFactorCal3_S2(proj, noiseModels.measurement,...
            sym_cam, sym_proj, K);
        factors.add(fact);
    end
    
    pose_id = pose_id + 1;
end


optimizer = LevenbergMarquardtOptimizer(factors, initial);
result = optimizer.optimizeSafely();

% Plotting
surf(X,Y,Z,C);
plot3(p(:,1),p(:,2),p(:,3));
plot3DTrajectory(initial, 'r', 1, 0.3);
plot3DTrajectory(result, 'g', 1, 0.3);
%     gtsam.plotPose3(cam.pose,[], 200000);
% plot3(points(:,1), points(:,2), points(:,3), 'go', 'LineWidth', 1000)
plot3DPoints(result);
axis equal;





