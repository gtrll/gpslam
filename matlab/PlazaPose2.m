% 2D SLAM example using Range + Odometry data, Plaza datasets
% dataset format: http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/log.html
% @author Jing Dong
% @date 02.05.2016

clear
close all
import gtsam.*
import gpslam.*


%% read datasets
datafile = 'data/Plaza2.mat';
load(datafile);

nr_pose = size(GT, 1);      % total number of poses
nr_land = size(TL, 1);      % total number of landmarks
nr_range = size(TD, 1);
land_idx = TL(:,1);         % landmark indices

% get range measurments bias fixed by linear fitting, and remove outliers
% range_trans is linear fitting coefficient
% range_outlier_mask is 0-1 mask of whether measurement is outlier
[range_trans, range_outlier_mask] = range_measure_fit(GT, TL, TD);


%% settings
useLinearPose2 = false;         % use linear [x y \theta] or SE(2) system states
addOdometry = true;             % use odometry information

initGroundTruth = false;        % use ground truth do initialization
addFirstPosePrior = true;       % add pose prior to first pose (fix position of whole system)
addFirstVelZeroPrior = false;   % add velocity prior to first velocity
addLandmarkPrior = true;        % add (loss) prior to landmarks to fix scale and position of whole system

useGaussNewton = false;         % use GN or LM
optimizeStopRelErr = 1e-6;      % relative error thresh hold to stop the optimization

% GP settings
QcModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 1e-1);

% noise models
firstPriorModel = noiseModel.Diagonal.Sigmas([1 1 pi]' * 1);
odomNoiseModel = noiseModel.Diagonal.Sigmas([1 1 pi]' * 1e-3);
rangeNoiseModel = noiseModel.Diagonal.Sigmas(0.5);
landPriorNoiseModel = noiseModel.Diagonal.Sigmas([1 1]' * 1);


%% initial factor graph

graph = NonlinearFactorGraph;
init_values = Values;

% initial landmarks
for i=1:nr_land
    land_point = Point2(TL(i,2), TL(i,3));
    
    % init landmark values as ground truth
    init_values.insert(symbol('l', land_idx(i)), land_point);
    
    % prior
    if addLandmarkPrior
        graph.add(PriorFactorPoint2(symbol('l', land_idx(i)), land_point, ...
            landPriorNoiseModel));
    end
end

% accumulated odometry poses
DRp_pose2 = zeros(nr_pose, 3);
DRp_pose2(1,:) = [GT(1, 2), GT(1, 3), GT(1, 4)+init_heading_offset];

last_pose = Pose2(GT(1, 2), GT(1, 3), GT(1, 4)+init_heading_offset);
last_vec = [GT(1, 2), GT(1, 3), GT(1, 4)+init_heading_offset]';

% first pose and velocity prior
if addFirstPosePrior
    if useLinearPose2
        graph.add(PriorFactorVector(symbol('x', 1), last_vec, firstPriorModel));
    else
        graph.add(PriorFactorPose2(symbol('x', 1), last_pose, firstPriorModel));
    end
end
if addFirstVelZeroPrior
    graph.add(PriorFactorVector(symbol('v', 1), zeros(3,1), firstPriorModel));
end

added_range_meas_idx = 1;       % range measurement idx should be added next

fprintf('building factor graph ...\n');
for pose_idx = 1:nr_pose
    
    if mod(pose_idx, 100) == 0
        fprintf('read pose idx %d of %d ...\n', pose_idx, nr_pose);
    end
    
    curr_time = GT(pose_idx, 1);
    
    if pose_idx > 1
        last_time = GT(pose_idx-1, 1);
        delta_t = curr_time - last_time;
        
        %% odometry & GP prior
        % get odom measurement pose
        odom_pose = Pose2(DR(pose_idx-1, 2), 0.0, DR(pose_idx-1, 3));
        new_pose = last_pose.compose(odom_pose);
        % odom vector
        odom_vec_meas = [DR(pose_idx-1, 2), 0.0, DR(pose_idx-1, 3)]';
        odom_vec = [new_pose.x() - last_pose.x(), new_pose.y() - last_pose.y(), ...
            odom_pose.theta()]';
        new_vec = last_vec + odom_vec;
        
        % accumulate odom poses
        if useLinearPose2
            DRp_pose2(pose_idx,:) = new_vec';
        else
            DRp_pose2(pose_idx,:) = [new_pose.x(), new_pose.y(), new_pose.theta()];
        end
        
        % add odometry factor
        if addOdometry
            if useLinearPose2
                graph.add(OdometryFactor2DLinear(symbol('x', pose_idx-1), symbol('x', pose_idx), ...
                    odom_vec_meas, odomNoiseModel));
            else
                graph.add(BetweenFactorPose2(symbol('x', pose_idx-1), symbol('x', pose_idx), ...
                    odom_pose, odomNoiseModel));
            end
        end
        
        % add GP prior factor
        if useLinearPose2
            graph.add(GaussianProcessPriorLinear3(symbol('x', pose_idx-1), ...
                symbol('v', pose_idx-1), symbol('x', pose_idx), symbol('v', pose_idx), ...
                delta_t, QcModel));
        else
            graph.add(GaussianProcessPriorPose2(symbol('x', pose_idx-1), ...
                symbol('v', pose_idx-1), symbol('x', pose_idx), symbol('v', pose_idx), ...
                delta_t, QcModel));
        end
  
        % for next iter
        last_pose = new_pose;
        last_vec = new_vec;
         
        
        %% add range measurements
        while added_range_meas_idx <= nr_range & TD(added_range_meas_idx, 1) <= curr_time
            
            % check outlier mask, remove outliers
            if ~range_outlier_mask(added_range_meas_idx)
                
                % get meas info
                tau = TD(added_range_meas_idx, 1) - last_time;
               
                % range bias fix
                meas_range = range_trans(1) * TD(added_range_meas_idx, 4) + range_trans(2);
                meas_land_idx = TD(added_range_meas_idx, 3);

%                 fprintf('range %d from pose %d to land %d, tau = %d, delta_t = %d \n', ...
%                     added_range_meas_idx, pose_idx, meas_land_idx, tau, delta_t);

                % add interpolate range factor
                if useLinearPose2
                    graph.add(GPInterpolatedRangeFactor2DLinear(meas_range, ...
                        symbol('x', pose_idx-1), symbol('v', pose_idx-1), ...
                        symbol('x', pose_idx), symbol('v', pose_idx), ...
                        symbol('l', meas_land_idx), rangeNoiseModel, QcModel, ...
                        delta_t, tau));
                else
                    graph.add(GPInterpolatedRangeFactorPose2(meas_range, ...
                        rangeNoiseModel, QcModel, symbol('x', pose_idx-1), ...
                        symbol('v', pose_idx-1), symbol('x', pose_idx), symbol('v', pose_idx), ...
                        symbol('l', meas_land_idx), delta_t, tau));
                end
            end
            
            added_range_meas_idx = added_range_meas_idx + 1;
        end
        
    end

    %% init values
    if useLinearPose2
        % if not use ground truth, just use last pose to initialize
        if initGroundTruth
            init_pose = [GT(pose_idx, 2), GT(pose_idx, 3), GT(pose_idx, 4)+init_heading_offset]';
        else
            init_pose = last_vec;
        end
        init_values.insert(symbol('x', pose_idx), init_pose);
        
    else
        if initGroundTruth
            init_pose = Pose2(GT(pose_idx, 2), GT(pose_idx, 3), GT(pose_idx, 4)+init_heading_offset);
        else
            init_pose = last_pose;
        end
        init_values.insert(symbol('x', pose_idx), init_pose);
    end
    
    init_vel = zeros(3,1);
    init_values.insert(symbol('v', pose_idx), init_vel);
    
end


%% optimize!
if useGaussNewton
    params = GaussNewtonParams;
    optimizer = GaussNewtonOptimizer(graph, init_values, params);
else
    params = LevenbergMarquardtParams;
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, params);
end

% optimize 
fprintf('init error: %d\n', optimizer.error());

iter_time = [];
opt_last_err = 1e20;
% tic
while (opt_last_err - optimizer.error())/opt_last_err > optimizeStopRelErr
    opt_last_err = optimizer.error();
    tic
    optimizer.iterate();
    iter_time = [iter_time, toc];
    fprintf('new error: %d\n', optimizer.error());
end
% fprintf('Total Optimization Time: %d\n', toc)
results = optimizer.values;

% average time
fprintf('Average Optimization Iteration Time: %d\n', mean(iter_time))


%% evaluate error

% extract results to matrix
OPT = zeros(nr_pose, 3);
OPT_LAND = zeros(nr_land, 2);
for i=1:nr_pose
    if useLinearPose2
        OPT(i, :) = results.atVector(symbol('x', i))';
    else
        opt_pose2 = results.atPose2(symbol('x', i));
        OPT(i, :) = [opt_pose2.x() opt_pose2.y(), opt_pose2.theta()];
    end
end
for i=1:nr_land
    opt_land = results.atPoint2(symbol('l', land_idx(i)));
    OPT_LAND(i, :) = [opt_land.x() opt_land.y()];
end

% position err
pos_err = OPT(:,1:2) - GT(:,2:3);
pos_err_sqrt = sqrt(pos_err(:,1).^2 + pos_err(:,2).^2);
fprintf('Average Position Error: %d\n', mean(pos_err_sqrt))

% rotation err
rot_err = OPT(:,3) - (GT(:,4)+init_heading_offset);
for i=1:nr_pose
    while rot_err(i) < -pi
        rot_err(i) = rot_err(i) + 2*pi;
    end
    while rot_err(i) > pi
        rot_err(i) = rot_err(i) - 2*pi;
    end
end
fprintf('Average Rotation Error: %d\n', mean(abs(rot_err)) * 180 / pi)

% land err err
land_err = OPT_LAND(:,1:2) - TL(:,2:3);
land_err_sqrt = sqrt(land_err(:,1).^2 + land_err(:,2).^2);
fprintf('Average Landmark Error: %d\n', mean(land_err_sqrt))


%% plot results

% plot results
h = figure(1);
hold on
axis equal

% plot traj
plot(DRp(:,2), DRp(:,3), '-.', 'Color',[1 0 0])

plot(GT(:,2), GT(:,3), '-', 'Color',[0 0.7 0])
plot(OPT(:,1), OPT(:,2), '-', 'Color',[0 0 1])

% plot land
plot(TL(:,2), TL(:,3), '*', 'Color',[0 0.7 0], 'MarkerSize', 8, 'Linewidth', 1.5)
plot(OPT_LAND(:,1), OPT_LAND(:,2), '+', 'Color',[0 0 1], 'MarkerSize', 8, 'Linewidth', 1.5)

% marks
ax = gca;
ax.XTick = [-100:20:100];
ax.YTick = [-100:20:100];
% box on

legend('Dead Reckoned Path', 'Ground Truth Trajectory', 'Estimated Trajectory', ...
    'Ground Truth Landmark', 'Estimated Landmark')
xlabel('X (m)')
ylabel('Y (m)')
hold off

