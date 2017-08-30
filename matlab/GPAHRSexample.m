% AHRS example with GP, fuse asynchronous accelerometer and gyroscope data
% @author Jing Dong 
% @date 02.15.2016

clear
close all
import gtsam.*
import gpslam.*


%% settings        
useGyro = true;         % use gyroscope information
useAcc = true;          % use accelerometer information
useGaussNewton = false; % use GN or LM solver

datasetMaxTime = 50.0;  % maximum dataset process time
optimizeStopRelErr = 1e-6;  % relative error threshold to stop optimization 
maxIteration = 100;     % max iteration to stop optimization 

% GP settings
QcModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 100);

% noisemodel
biasPriorModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 1e-2);
biasBetweenModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 1e-4);
gyroNoiseModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 1e-4);
gyroNoiseModelMat = [1 0 0; 0 1 0; 0 0 1] * 1e-3;
accNoiseModel = noiseModel.Diagonal.Sigmas([1 1]' * 0.1);
firstRotPriorModel = noiseModel.Diagonal.Sigmas([1 1 1]' * 0.1);


%% load data

% Collected dataset has synchronous acc/gyro data.
% To create an asynchronous dataset, we skip some measurements in different
% frequency. Here we setup the data rate you want for acc/gyro data.
% Time unit is second
gyro_dt = 0.005;
acc_dt = 0.02;

dataset_path = 'data/';

% IMU format: seq, time(sec), gyro_x/y/z, acc_x/y/z
imu_filename = 'RAW_IMU_DATA_matlab.txt';
% ATT format: seq, time(sec), ori_x/y/z/w
att_filename = 'MOCAP_POSE_DATA_matlab.txt';

IMU = load(strcat(dataset_path, imu_filename));
MOCAP = load(strcat(dataset_path, att_filename));

% get attitute data from motion captured poses
ATT = MOCAP(:, [1:2, 6:9]);

nr_imu = size(IMU, 1);
nr_att = size(ATT, 1);

% ATT_YPR format yaw/pitch/roll : y/p/r
ATT_YPR = zeros(nr_att, 3);
for i=1:nr_att
    % wxyz in gtsam Rot3 constructor
    rot3 = Rot3.Quaternion(ATT(i, 6), ATT(i, 3), ATT(i, 4), ATT(i, 5));
    ATT_YPR(i,:) = rot3.ypr();
end

biasHat = zeros(3,1);       % bias prior: zero
zcoriolis = zeros(3,1);     % coriolis force: ignore here


%% build factor graph using gyro and acc
graph = NonlinearFactorGraph;
gyro_graph = NonlinearFactorGraph;

meas_time = 0;
last_gyro_meas_time = 0;
last_acc_meas_time = 0;
add_rot_state = false;

% cached acc factor information, since they can only be added when state is
% determined by gyro
cached_acc_add_idx = [];
nr_acc = 0;

var_idx = 1;
cached_state_meas_idx = [];
meas_idx = 1;

fprintf('building factor graph ...\n')
while meas_time < datasetMaxTime & meas_idx <= nr_imu
    
    meas_time = IMU(meas_idx, 2);
    
    if (floor(meas_time) > floor(last_gyro_meas_time))
        fprintf('Read dataset time: %ds ...\n', floor(meas_time))
    end
    
    if meas_idx > 1
        delta_t = IMU(meas_idx, 2) - IMU(meas_idx-1, 2);
    end
    
    
    % acc factors, based on time interval with last
    if useAcc & meas_time - last_acc_meas_time >= acc_dt
        cached_acc_add_idx = [cached_acc_add_idx, meas_idx];
        % next iter
        last_acc_meas_time = meas_time;
        nr_acc = nr_acc + 1;
    end
    
    
    % process Gyro measurements and decide system states time stamps 
    
    % add states: start, end, and time stamp meet interval
    if var_idx == 1 | meas_idx == nr_imu | meas_time - last_gyro_meas_time >= gyro_dt
        
        % first state: bias add prior
        if var_idx == 1
            graph.add(PriorFactorRot3(symbol('x', 1), Rot3.Ypr(ATT_YPR(1,1), ...
                ATT_YPR(1,2), ATT_YPR(1,3)), firstRotPriorModel));
            graph.add(PriorFactorVector(symbol('b', 1), zeros(3,1), biasPriorModel));
            
            gyro_graph.add(PriorFactorRot3(symbol('x', 1), Rot3.Ypr(ATT_YPR(1,1), ...
                ATT_YPR(1,2), ATT_YPR(1,3)), firstRotPriorModel));
            gyro_graph.add(PriorFactorVector(symbol('b', 1), zeros(3,1), biasPriorModel));
            
        % check gyro rate: if dt > gyro_dt then add new state
        else
            dt = meas_time - last_gyro_meas_time;       % for GP
            
            % integerate gyro for last step
            pim.integrateMeasurement(IMU(meas_idx, 3:5)', delta_t);
            
            % gyro factors
            gyro_graph.add(AHRSFactor(symbol('x', var_idx - 1), symbol('x', var_idx), ...
                symbol('b', var_idx - 1), pim, zcoriolis));
            
            if useGyro
                graph.add(AHRSFactor(symbol('x', var_idx - 1), symbol('x', var_idx), ...
                    symbol('b', var_idx - 1), pim, zcoriolis));
            end
            
            % bias factors
            graph.add(BetweenFactorVector(symbol('b', var_idx - 1), symbol('b', var_idx), ...
                zeros(3,1), biasBetweenModel));
            gyro_graph.add(BetweenFactorVector(symbol('b', var_idx - 1), symbol('b', var_idx), ...
                zeros(3,1), biasBetweenModel));
            
            % GP prior
            graph.add(GaussianProcessPriorRot3(...
                symbol('x', var_idx -1), symbol('v', var_idx -1), ...
                symbol('x', var_idx), symbol('v', var_idx), ...
                dt, QcModel));
            
            % add cached acc factor
            for i=1:numel(cached_acc_add_idx)
                
                acc_idx = cached_acc_add_idx(i);
                
                % check whether interpolated factor
                if acc_idx == meas_idx
                    % non-inter
                    % TODO: what's the correct acc input format?
                    graph.add(Rot3AttitudeFactor(symbol('x', var_idx), ...
                        Unit3(Point3(0, 0, 1)), accNoiseModel, ...
                        Unit3(Point3(IMU(meas_idx, 6), IMU(meas_idx, 7), IMU(meas_idx, 8)))));
                else
                    % inter
                    tau = IMU(acc_idx, 2) - last_gyro_meas_time;
                    % TODO: what's the correct acc input format?
                    graph.add(GPInterpolatedAttitudeFactorRot3(...
                        symbol('x', var_idx -1), symbol('v', var_idx -1), ...
                        symbol('x', var_idx), symbol('v', var_idx), ...
                        dt, tau, QcModel, accNoiseModel, Unit3(Point3(0, 0, 1)), ...
                        Unit3(Point3(IMU(meas_idx, 6), IMU(meas_idx, 7), IMU(meas_idx, 8)))));
                end
            end
            cached_acc_add_idx = [];        % clear cache
        end
        
        % prepare pre-integerated measurements
        pim = PreintegratedAhrsMeasurements(biasHat, gyroNoiseModelMat);
        
        % next iter: add state
        last_gyro_meas_time = meas_time;
        cached_state_meas_idx = [cached_state_meas_idx, meas_idx];
        var_idx = var_idx + 1;
        add_rot_state = true;
        
    % not added as new state, just do pre-integeration
    else
        % just do integeration
        pim.integrateMeasurement(IMU(meas_idx, 3:5)', delta_t);
        
        % next iter: not add state
        add_rot_state = false;
    end
    
    % meas next iter
    meas_idx = meas_idx + 1;
    
    % actual dataset length used
    actual_time = meas_time;
end

nr_opt = var_idx - 1;

fprintf('\nNumber of Gyroscope measurements used: %d\n', nr_opt)
fprintf('Number of Accelerometer measurements used: %d\n', nr_acc)

fprintf('\nActual Gyroscope data frequency: %fHz\n', nr_opt/actual_time)
fprintf('Actual Accelerometer data frequency: %fHz\n', nr_acc/actual_time)


%% init values
init_values = Values;

% init zero rotation values
for i=1:nr_opt
    init_values.insert(symbol('x', i), Rot3);
    init_values.insert(symbol('b', i), zeros(3,1));
end

% optimize gyro-only graph, update the initial values by gyro-only values
fprintf('\nOptimizing Gyro only graph (for initialization) ... \n\n');
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

% velocity for GP
for i=1:nr_opt
    init_values.insert(symbol('v', i), zeros(3,1));
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
fprintf('Total Optimization Time: %f\n', toc)
results = optimizer.values;


%% prepare results
OPT = zeros(nr_opt, 3);
GYRO = zeros(nr_opt, 3);
BIAS = zeros(nr_opt, 3);

for i=1:nr_opt
    rot3 = results.atRot3(symbol('x', i));
    OPT(i,:) = [rot3.yaw, rot3.pitch, rot3.roll];
    
    BIAS(i,:) = [results.atVector(symbol('b', i))'];
    
    rot3 = gyro_only_results.atRot3(symbol('x', i));
    GYRO(i,:) = [rot3.yaw, rot3.pitch, rot3.roll];
end


%% plot

% pitch
h = figure(1);
subplot(3,1,1)
hold on
plot(ATT(:, 2), ATT_YPR(:,2), '-', 'Color', [0, 0.8, 0]);
plot(IMU(cached_state_meas_idx, 2), GYRO(:,2), 'r-.');
plot(IMU(cached_state_meas_idx, 2), OPT(:,2), 'b-');
axis([0 datasetMaxTime -pi pi])
hold off
% marks
title('Pitch')
ylabel('Rad')
xlabel('t (s)')
box on
legend('Ground truth', 'Gyro-only', 'Estimated', 'Location', 'northwest')


% roll
subplot(3,1,2);
hold on
plot(ATT(:, 2), ATT_YPR(:,3), '-', 'Color', [0, 0.8, 0]);
plot(IMU(cached_state_meas_idx, 2), GYRO(:,3), 'r-.');
plot(IMU(cached_state_meas_idx, 2), OPT(:,3), 'b-');
axis([0 datasetMaxTime -pi pi])
hold off
% marks
title('Roll')
ylabel('Rad')
xlabel('t (s)')
box on
legend('Ground truth', 'Gyro-only', 'Estimated', 'Location', 'northwest')


% bias
subplot(3,1,3)
hold on
plot(IMU(cached_state_meas_idx, 2), BIAS(:,1), 'r-');
plot(IMU(cached_state_meas_idx, 2), BIAS(:,2), 'g-');
plot(IMU(cached_state_meas_idx, 2), BIAS(:,3), 'b-');
axis([0 datasetMaxTime -1e-2 1e-2])
hold off
% marks
box on
title('Bias')
xlabel('t (s)')
legend('Pitch', 'Roll', 'Yaw', 'Location', 'northwest')




