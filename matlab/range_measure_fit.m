function [range_trans, range_outlier_mask] = range_measure_fit(GT, TL, TD)
%RANGE_MEASURE_FIT fit range measurment bias be least-square with ground
%truth
%   Detailed explanation goes here

% Range fitting for the Plaza dataset using least square
nr_pose = size(GT, 1);       % time step to investigate
nr_range = size(TD, 1);       % time step to investigate
T = GT(:,1);

%% Construct data
% Landmark locations ground truth
lx = TL(:,2);    
ly = TL(:,3);
l_true = [lx, ly];
% forward / inverse map of landmark index
Lindx = TL(:,1);
Lindx_inv = zeros(max(Lindx)+1, 1);
for i=1:numel(Lindx)
    Lindx_inv(Lindx(i)+1) = i;
end

% Landmark range measurements
Range = zeros(nr_range, 3);    % [time index, landmark index, range; ... ]
v = 1;      % the current range measurement index
t = TD(v, 1);           % when the current range measurement took place
i = 1;      % pose indx
while i <= nr_pose && v <= nr_range
    % Already passed the range measurement?
    t = TD(v, 1);
    if (T(i) >= t)
        % See the range measure is close to which time index
        % If close to i-1
        if abs(T(i-1) - t) <= abs(T(i)-t)
            Range(v,1) = i-1;
        else
            Range(v,1) = i;
        end
        Range(v,2) = Lindx_inv(TD(v,3)+1);
        Range(v,3) = TD(v,4);
        v = v+1;
    else
        i = i+1;
    end
end


true_range = zeros(nr_range, 1);
measured_range = zeros(nr_range, 1);
for i = 1:nr_range
    x = GT(Range(i, 1), 2:3)';
    l = l_true(Range(i, 2), :)';
    true_range(i) = sqrt(sum((x-l).^2) );
    measured_range(i) = Range(i, 3);
end

%% Least square
b = true_range;
A = [measured_range ones(nr_range, 1)];
x = A \ b;
range_trans = x;

%% outlier mask
range_outlier_mask = zeros(nr_range, 1);
outlier_limit = 2.0;
for i=1:nr_range
    fit_range = range_trans(1) * measured_range(i) + range_trans(2);
    if abs(fit_range - true_range(i)) > outlier_limit
        range_outlier_mask(i) = 1;
    else
        range_outlier_mask(i) = 0;
    end
end

meas_range_inlier = [];
true_range_inlier = [];
meas_range_outlier = [];
true_range_outlier = [];
for i=1:nr_range
    if range_outlier_mask(i)
        meas_range_outlier = [meas_range_outlier; measured_range(i)];
        true_range_outlier = [true_range_outlier; true_range(i)];
    else
        meas_range_inlier = [meas_range_inlier; measured_range(i)];
        true_range_inlier = [true_range_inlier; true_range(i)];
    end
end

%% update estimate by inliers
b = true_range_inlier;
A = [meas_range_inlier ones(numel(meas_range_inlier), 1)];
x = A \ b;
range_trans = x;

% %% plot
% figure(100)
% hold on
% title('Measurements fitting')
% 
% plot(true_range_inlier, meas_range_inlier, 'b+')
% 
% meas_range_updated = range_trans(1) * meas_range_inlier + range_trans(2);
% plot(true_range_inlier, meas_range_updated, 'g+')
% 
% plot([0, max(true_range_inlier)], [0, max(true_range_inlier)], 'k-')
% 
% plot(true_range_outlier, meas_range_outlier, 'r+')
% 
% legend('Raw inlier', 'Fitted inlier', 'Y = X', 'Raw outlier')
% xlabel('Ground Truth Range (m)')
% ylabel('Measured/Fitted Range (m)')
% hold off




