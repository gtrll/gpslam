% AHRS example without using GP
% Jing 02.08.2016

clear
close all
import gtsam.*
import gpslam.*

%% settings
useGyro = true;
useAcc = true;
useGaussNewton = false;
useGyroInit = true;

datasetMaxTime = 90.0;
optimizeStopRelErr = 1e-6;
maxIteration = 100;

% noisemodel
biasPriorModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 1e-2);
biasBetweenModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 1e-4);
gyroNoiseModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 1e-4);
gyroNoiseModelMat = [1 0 0; 0 1 0; 0 0 1] * 1e-3;
accNoiseModel = noiseModel.Diagonal.Sigmas([1 1]' * 0.1);
firstRotPriorModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 0.1);

%% load data

dataset_path = '/home/jing/data/imu/mocap_022304/parsed_matlab/';
att_mocap = true;

% IMU format: seq, time(sec), gyro_x/y/z, acc_x/y/z
imu_filename = 'RAW_IMU_DATA_matlab.txt';
% ATT format: seq, time(sec), ori_x/y/z/w
% att_filename = 'ATTITUDE_IMU_DATA_matlab.txt';
att_filename = 'MOCAP_POSE_DATA_matlab.txt';

IMU = load(strcat(dataset_path, imu_filename));
ATT = load(strcat(dataset_path, att_filename));

% mocap reformat
if att_mocap
    ATT = ATT(:, [1:2, 6:9]);
end

nr_imu = size(IMU, 1);
nr_att = size(ATT, 1);

% convert ATT to ypr format
load_att_ypr = false;

% ATT_YPR format: y/p/r
ATT_YPR = zeros(nr_att, 3);
att_ypr_filename = 'ATT_YPR_matlab.mat';

if load_att_ypr
    load(strcat(dataset_path, att_ypr_filename), 'ATT_YPR');
else
    for i=1:nr_att
        % wxyz in gtsam Rot3 constructor
        rot3 = Rot3.Quaternion(ATT(i, 6), ATT(i, 3), ATT(i, 4), ATT(i, 5));
        ATT_YPR(i,:) = rot3.ypr();
    end
    save(strcat(dataset_path, att_ypr_filename), 'ATT_YPR');
end

% % mocap reformat
% if att_mocap
%     ATT_YPR(:,2:3) = [ATT_YPR(:,3), -ATT_YPR(:,2)];
% end

%% plot gound truth
figure(2)

% pitch
subplot(3,1,1)
hold on
plot(ATT(:, 2), ATT_YPR(:,2), 'g-');
axis([0 datasetMaxTime -pi pi])
hold off
title('Pitch')

% roll
subplot(3,1,2)
hold on
plot(ATT(:, 2), ATT_YPR(:,3), 'g-');
axis([0 datasetMaxTime -pi pi])
hold off
title('Roll')

% yaw
subplot(3,1,3)
hold on
plot(ATT(:, 2), ATT_YPR(:,1), 'g-');
axis([0 datasetMaxTime -pi pi])
hold off
title('Yaw')


%% build factor graph using gyro and acc

graph = NonlinearFactorGraph;
gyro_graph = NonlinearFactorGraph;
init_values = Values;

meas_time = 0;
var_idx = 1;
while meas_time < datasetMaxTime & var_idx <= nr_imu
    
    meas_time = IMU(var_idx, 2);
    fprintf('Time: %d\n', meas_time)
    
    % first bias: add prior
    if var_idx == 1
        graph.add(PriorFactorRot3(symbol('x', 1), Rot3, firstRotPriorModel));
        insertVectorPriorFactor3(graph, symbol('b', 1), zeros(3,1), biasPriorModel);
        
        gyro_graph.add(PriorFactorRot3(symbol('x', 1), Rot3, firstRotPriorModel));
        insertVectorPriorFactor3(gyro_graph, symbol('b', 1), zeros(3,1), biasPriorModel);
    else
        % gyro factors
        biasHat = zeros(3,1);
        delta_t = IMU(var_idx, 2) - IMU(var_idx-1, 2);
        pim = PreintegratedAhrsMeasurements(biasHat, gyroNoiseModelMat);
        pim.integrateMeasurement(IMU(var_idx, 3:5)', delta_t);
        zcoriolis = zeros(3,1);

        gyro_graph.add(AHRSFactor(symbol('x', var_idx - 1), symbol('x', var_idx), ...
            symbol('b', var_idx - 1), pim, zcoriolis));
        
        if useGyro
            graph.add(AHRSFactor(symbol('x', var_idx - 1), symbol('x', var_idx), ...
                symbol('b', var_idx - 1), pim, zcoriolis));
        end
        
        % bias factors
        insertVectorBetweenFactor3(graph, symbol('b', var_idx - 1), symbol('b', var_idx), ...
            zeros(3,1), biasBetweenModel);
        insertVectorBetweenFactor3(gyro_graph, symbol('b', var_idx - 1), symbol('b', var_idx), ...
            zeros(3,1), biasBetweenModel);
    end
    
    % acc factors
    if useAcc
        % TODO: what's the correct acc input format?
        graph.add(Rot3AttitudeFactor(symbol('x', var_idx), ...
            Unit3(Point3(0, 0, 1)), accNoiseModel, ...
            Unit3(Point3(IMU(var_idx, 6), IMU(var_idx, 7), IMU(var_idx, 8)))));
    end
    
    % next iter
    var_idx = var_idx + 1;
end
nr_opt = var_idx - 1;


%% init values

% init zero rotation values
for i=1:nr_opt
    init_values.insert(symbol('x', i), Rot3);
    init_values.insert(symbol('b', i), zeros(3,1));
end

if useGyroInit
    % optimize gyro only graph
    fprintf('Optimizing Gyro only graph ... \n\n');
    if useGaussNewton
        params = GaussNewtonParams;
        optimizer = GaussNewtonOptimizer(gyro_graph, init_values, params);
    else
        params = LevenbergMarquardtParams;
        optimizer = LevenbergMarquardtOptimizer(gyro_graph, init_values, params);
    end
    gyro_only_results = optimizer.optimize();
    
    for i=1:nr_opt
        init_values.update(symbol('x', i), gyro_only_results.atRot3(symbol('x', i)));
    end
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
iter_count = 0;
tic
while (opt_last_err - optimizer.error())/opt_last_err > optimizeStopRelErr ...
        && iter_count < maxIteration
    opt_last_err = optimizer.error();
    optimizer.iterate();
    iter_count = iter_count + 1;
    fprintf('iter %d: new error: %d\n', iter_count, optimizer.error());
end
fprintf('Total Optimization Time: %d\n', toc)
results = optimizer.values;


%% prepare results
OPT = zeros(nr_opt, 3);
GYRO = zeros(nr_opt, 3);
BIAS = zeros(nr_opt, 3);

for i=1:nr_opt
    rot3 = results.atRot3(symbol('x', i));
    OPT(i,:) = [rot3.yaw, rot3.pitch, rot3.roll];
    
    BIAS(i,:) = [results.atVector(symbol('b', i))'];
    
    if useGyroInit
        rot3 = gyro_only_results.atRot3(symbol('x', i));
        GYRO(i,:) = [rot3.yaw, rot3.pitch, rot3.roll];
    end
end


%% plot
figure

% pitch
subplot(3,1,1)
hold on
plot(ATT(:, 2), ATT_YPR(:,2), 'g-');
if useGyroInit
    plot(IMU(1:nr_opt, 2), GYRO(:,2), 'r-.');
end
plot(IMU(1:nr_opt, 2), OPT(:,2), 'b-');
axis([0 datasetMaxTime -pi pi])
hold off
title('Pitch')

% roll
subplot(3,1,2)
hold on
plot(ATT(:, 2), ATT_YPR(:,3), 'g-');
if useGyroInit
    plot(IMU(1:nr_opt, 2), GYRO(:,3), 'r-.');
end
plot(IMU(1:nr_opt, 2), OPT(:,3), 'b-');
axis([0 datasetMaxTime -pi pi])
hold off
title('Roll')

% bias
subplot(3,1,3)
hold on
plot(IMU(1:nr_opt, 2), BIAS(:,1), 'r-');
plot(IMU(1:nr_opt, 2), BIAS(:,2), 'g-');
plot(IMU(1:nr_opt, 2), BIAS(:,3), 'b-');
axis([0 datasetMaxTime -1 1])
hold off
title('Bias')




