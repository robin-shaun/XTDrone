% Copyright (c) 2017, California Institute of Technology.
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
%    this list of conditions and the following disclaimer.
% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% The views and conclusions contained in the software and documentation are
% those of the authors and should not be interpreted as representing official
% policies, either expressed or implied, of the California Institute of
% Technology.
%
%%% calibrate_bundle.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Script to determine AprilTag bundle relative poses to a "master" tag.
%
% Instructions:
%   Record a bagfile of the /tag_detections topic where you steadily
%   point the camera at the AprilTag bundle such that all the bundle's
%   individual tags are visible at least once at some point (the more the
%   better). Run the script, then copy the printed output into the tag.yaml
%   configuration file of apriltag_ros.
%
% $Revision: 1.0 $
% $Date: 2017/12/17 13:37:34 $
% $Author: dmalyuta $
%
% Originator:        Danylo Malyuta, JPL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% User inputs

% Relative directory of calibration bagfile
calibration_file = 'data/calibration.bag';

% Bundle name
bundle_name = 'my_bundle';

% Master tag's ID
master_id = 0;

%% Make sure matlab_rosbag is installed

if ~exist('matlab_rosbag-0.5.0-linux64','file')
    websave('matlab_rosbag', ...
            ['https://github.com/bcharrow/matlab_rosbag/releases/' ...
             'download/v0.5/matlab_rosbag-0.5.0-linux64.zip']);
    unzip('matlab_rosbag');
    delete matlab_rosbag.zip
end
addpath('matlab_rosbag-0.5.0-linux64');

%% Load the tag detections bagfile

bag = ros.Bag.load(calibration_file);
tag_msg = bag.readAll('/tag_detections');

clear tag_data;
N = numel(tag_msg);
t0 = getBagTime(tag_msg{1});
for i = 1:N
    tag_data.t(i) = getBagTime(tag_msg{i})-t0;
    for j = 1:numel(tag_msg{i}.detections)
        detection = tag_msg{i}.detections(j);
        if numel(detection.id)>1
            % Can only use standalone tag detections for calibration!
            % The math allows for bundles too (e.g. bundle composed of
            % bundles) but the code does not, and it's not that useful
            % anyway
            warning_str = 'Skipping tag bundle detection with IDs';
            for k = 1:numel(detection.id)
                warning_str = sprintf('%s %d',warning_str,detection.id(k));
            end
            warning(warning_str);
            continue;
        end
        tag_data.detection(i).id(j) = detection.id;
        tag_data.detection(i).size(j) = detection.size;
        % Tag position with respect to camera frame
        tag_data.detection(i).p(:,j) = detection.pose.pose.pose.position;
        % Tag orientation with respect to camera frame
        % [w;x;y;z] format
        tag_data.detection(i).q(:,j) = ...
                         detection.pose.pose.pose.orientation([4,1,2,3]);
    end
end

%% Compute the measured poses of each tag relative to the master tag

master_size = []; % Size of the master tag

% IDs, sizes, relative positions and orientations of detected tags other
% than master
other_ids = [];
other_sizes = [];
rel_p = {};
rel_q = {};

createT = @(p,q) [quat2rotmat(q) p; zeros(1,3) 1];
invertT = @(T) [T(1:3,1:3)' -T(1:3,1:3)'*T(1:3,4); zeros(1,3) 1];

N = numel(tag_data.detection);
for i = 1:N
    this = tag_data.detection(i);
    
    mi = find(this.id == master_id);
    if isempty(mi)
        % Master not detected in this detection, so this particular
        % detection is useless
        continue;
    end
    
    % Get the master tag's rigid body transform to the camera frame
    T_cm = createT(this.p(:,mi), this.q(:,mi));
    
    % Get the rigid body transform of every other tag to the camera frame
    for j = 1:numel(this.id)
        % Skip the master, but get its size first
        if isempty(master_size)
            master_size = this.size(j);
        end
        % We already have the rigid body transform from the master tag to
        % the camera frame (T_cm)
        if j == mi
            continue;
        end
        
        % Add ID to detected IDs, if not already there
        id = this.id(j);
        if ~ismember(id, other_ids)
            other_ids(end+1) = id;
            other_sizes(end+1) = this.size(j);
            rel_p{end+1} = [];
            rel_q{end+1} = [];
        end
        
        % Find the index in other_ids corresponding to this tag
        k = find(other_ids == id);
        assert(numel(k) == 1, ...
               'Tag ID must appear exactly once in the other_ids array');
        
        % Get this tag's rigid body transform to the camera frame
        T_cj = createT(this.p(:,j), this.q(:,j));
        
        % Deduce this tag's rigid body transform to the master tag's frame
        T_mj = invertT(T_cm)*T_cj;
        
        % Save the relative position and orientation of this tag to the
        % master tag
        rel_p{k}(:,end+1) = T_mj(1:3,4);
        rel_q{k}(:,end+1) = rotmat2quat(T_mj);
    end
end

assert(~isempty(master_size), ...
      sprintf('Master tag with ID %d not found in detections', master_id));

%% Compute (geometric) median position of each tag in master tag frame

geometricMedianCost = @(x,y) sum(sqrt(sum((x-y).^2)));

options = optimset('MaxIter',1000,'MaxFunEvals',1000, ...
                   'Algorithm','interior-point', ...
                   'TolFun', 1e-6, 'TolX', 1e-6);

M = numel(rel_p);
rel_p_median = nan(3, numel(other_ids));
for i = 1:M
    % Compute the mean position as the initial value for the minimization
    % problem
    p_0 = mean(rel_p{i},2);
    
    % Compute the geometric median
    [rel_p_median(:,i),~,exitflag] = ...
                fminsearch(@(x) geometricMedianCost(rel_p{i}, x), p_0, options);
    assert(exitflag == 1, ...
           sprintf(['Geometric median minimization did ' ...
                    'not converge (exitflag %d)'], exitflag));
end

%% Compute the average orientation of each tag with respect to the master tag

rel_q_mean = nan(4, numel(other_ids));
for i = 1:M
    % Use the method in Landis et al. "Averaging Quaternions", JGCD 2007
    
    % Check the sufficient uniqueness condition
    % TODO this is a computational bottleness - without this check, script
    % returns much faster. Any way to speed up this double-for-loop?
    error_angle{i} = [];
    for j = 1:size(rel_q{i},2)
        q_1 = rel_q{i}(:,j);
        for k = 1:size(rel_q{i},2)
            if j==k
                continue;
            end
            q_2 = rel_q{i}(:,k);
            q_error = quatmult(quatinv(q_1),q_2);
            % Saturate to valid acos range, which prevents imaginary output
            % from acos due to q_error_w being infinitesimaly (to numerical
            % precision) outside of valid [-1,1] range
            q_error_w = min(1,max(q_error(1),-1));
            error_angle{i}(end+1) = 2*acos(q_error_w);
            if 2*acos(q_error_w) >= pi/2
                warning(['Quaternion pair q_%u and q_%u for tag ID %u ' ...
                         'are more than 90 degrees apart!'], ...
                        j,k,other_ids(i));
            end
        end
    end
    
    % Average quaternion method
    Q = rel_q{i};
    [V, D] = eig(Q*Q.');
    [~,imax] = max(diag(D)); % Get the largest eigenvalue
    rel_q_mean(:,i) = V(:,imax); % Corresponding eigenvector
    if rel_q_mean(1,i) < 0
        rel_q_mean(:,i) = -rel_q_mean(:,i); % Ensure w positive
    end
end

%% Print output to paste in tags.yaml

% Head + master tag
fprintf([ ...
'tag_bundles:\n' ...
'  [\n' ...
'    {\n' ...
'      name: ''%s'',\n' ...
'      layout:\n' ...
'        [\n'], bundle_name);

% All other tags detected at least once together with master tag
for i = 0:numel(other_ids)
    newline = ',';
    if i == numel(other_ids)
        newline = '';
    end
    if i == 0
        fprintf('          {id: %d, size: %.2f, x: %.4f, y: %.4f, z: %.4f, qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f}%s\n', ...
                master_id, master_size, 0, 0, 0, 1, 0, 0, 0, newline);
    else
        fprintf('          {id: %d, size: %.2f, x: %.4f, y: %.4f, z: %.4f, qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f}%s\n', ...
                other_ids(i), other_sizes(i), rel_p_median(1,i), ...
                rel_p_median(2,i), rel_p_median(3,i), rel_q_mean(1,i), ...
                rel_q_mean(2,i), rel_q_mean(3,i), rel_q_mean(4,i), newline);
    end
end

% Tail
fprintf([ ...
'        ]\n'...
'    }\n'...
'  ]\n']);

%% Local functions

function t = getBagTime(bagfile)
    t = double(bagfile.header.stamp.sec)+ ...
        double(bagfile.header.stamp.nsec)/1e9;
end

function R = quat2rotmat(q)
    % Creates an ACTIVE rotation matrix from a quaternion
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    R = [1-2*(y^2+z^2)  2*(x*y-w*z)    2*(x*z+w*y)
         2*(x*y+w*z)    1-2*(x^2+z^2)  2*(y*z-w*x)
         2*(x*z-w*y)    2*(y*z+w*x)    1-2*(x^2+y^2)];
end

function q = rotmat2quat(R)
    % Adapted for MATLAB from
    % http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    tr = R(1,1) + R(2,2) + R(3,3);

    if tr > 0
      S = sqrt(tr+1.0) * 2; % S=4*qw 
      qw = 0.25 * S;
      qx = (R(3,2) - R(2,3)) / S;
      qy = (R(1,3) - R(3,1)) / S; 
      qz = (R(2,1) - R(1,2)) / S; 
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3)) 
      S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*qx 
      qw = (R(3,2) - R(2,3)) / S;
      qx = 0.25 * S;
      qy = (R(1,2) + R(2,1)) / S; 
      qz = (R(1,3) + R(3,1)) / S; 
    elseif (R(2,2) > R(3,3))
      S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S=4*qy
      qw = (R(1,3) - R(3,1)) / S;
      qx = (R(1,2) + R(2,1)) / S; 
      qy = 0.25 * S;
      qz = (R(2,3) + R(3,2)) / S; 
    else
      S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S=4*qz
      qw = (R(2,1) - R(1,2)) / S;
      qx = (R(1,3) + R(3,1)) / S;
      qy = (R(2,3) + R(3,2)) / S;
      qz = 0.25 * S;
    end

    q = [qw qx qy qz]';
end

function c = quatmult(a,b)
    % More humanly understandable version:
    % Omegaa = [a((1)) -a((2):(4)).'
    %           a((2):(4)) a((1))*eye((3))-[0 -a((4)) a((3)); a((4)) 0 -a((2));-a((3)) a((2)) 0]];
    % c = Omegaa*b;
    % More optimized version:
    c_w = a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
    c_x = a(1)*b(2) + a(2)*b(1) - a(3)*b(4) + a(4)*b(3);
    c_y = a(1)*b(3) + a(3)*b(1) + a(2)*b(4) - a(4)*b(2);
    c_z = a(1)*b(4) - a(2)*b(3) + a(3)*b(2) + a(4)*b(1);
    c = [c_w; c_x; c_y; c_z];
end

function qinv = quatinv(q)
    qinv = [q(1); -q(2:4)];
end
