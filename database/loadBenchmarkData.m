function [U_dt, Y_dt, t_dt] = loadBenchmarkData(csv_filename, T_ini, T_f, varargin)
% LOADBENCHMARKDATA Preprocess IDSIA Flight Data for DeePC Construction
%   Reads raw CSV benchmark data, converts motors to physical thrust/torques 
%   using corrected calibration matrices, applies 4th order Zero-Phase 
%   Butterworth filtering, and extracts the optimal 'Square Hankel' window.
%
%   INPUTS:
%       csv_filename - Name of the file inside database/idsia_benchmark/
%       T_ini        - Past horizon horizon (e.g., 6)
%       T_f          - Future prediction horizon (e.g., 25)
%
%   Optional Name-Value:
%       'IncludeAttitude' - Include roll/pitch/yaw in Y_dt (default: false).
%
%   OUTPUTS:
%       U_dt - Control Inputs [T, tau_x, tau_y, tau_z] (4 x T_optimal)
%       Y_dt - Dynamic States [x, y, z, vx, vy, vz, wx, wy, wz]
%              or [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
%       t_dt - Trimmed time vector (1 x T_optimal)
%
%   See also: DEEPCSETUP

    % --- 0. Parse Inputs ---
    p = inputParser;
    addRequired(p, 'csv_filename');
    addRequired(p, 'T_ini', @isnumeric);
    addRequired(p, 'T_f', @isnumeric);
    addParameter(p, 'StartIdx', 100, @isnumeric); % Skip initial noisy hover samples
    addParameter(p, 'IncludeAttitude', false, @islogical);
    parse(p, csv_filename, T_ini, T_f, varargin{:});
    opts = p.Results;

    % Relative path resolution assuming function is in /database/
    file_path = fullfile(fileparts(mfilename('fullpath')), 'idsia_benchmark', csv_filename);
    
    if ~isfile(file_path)
        error('Data file not found at: %s\nDid you copy the benchmark data correctly?', file_path);
    end

    raw = readtable(file_path);

    % --- 1. Signal Extraction ---
    Fs = 100; % 100 Hz sampling rate from paper
    t = raw.t';
    
    % Raw outputs
    pos = [raw.x, raw.y, raw.z]';
    vel = [raw.vx, raw.vy, raw.vz]';
    rates = [raw.wx, raw.wy, raw.wz]';
    euler = quatToEuler([raw.qx, raw.qy, raw.qz, raw.qw]');
    
    % Raw Inputs (rad/s)
    motors = [raw.m1_rads, raw.m2_rads, raw.m3_rads, raw.m4_rads]';

    % --- 2. Zero-Phase Butterworth Filtering (Table 4 from paper) ---
    % 4th order butterworth relative to Fs/2 (Nyquist)
    
    % Position: 10 Hz cutoff
    [b_pos, a_pos] = butter(4, 10 / (Fs/2));
    pos_f = filtfilt(b_pos, a_pos, pos')';
    
    % Velocity & Angular Rates: 18 Hz cutoff
    [b_vel, a_vel] = butter(4, 18 / (Fs/2));
    vel_f = filtfilt(b_vel, a_vel, vel')';
    euler_f = filtfilt(b_vel, a_vel, euler')';
    rates_f = filtfilt(b_vel, a_vel, rates')';
    
    % Motors: 20 Hz Cutoff
    [b_mot, a_mot] = butter(4, 20 / (Fs/2));
    motors_f = filtfilt(b_mot, a_mot, motors')';
    
    % Assemble output state
    if opts.IncludeAttitude
        y_full = [pos_f; vel_f; euler_f; rates_f];
    else
        y_full = [pos_f; vel_f; rates_f];
    end
    
    % --- 3. Corrected Physics Mapping (from LSQ Appendix) ---
    % The benchmark specifies a "Cross (X)" configuration mapping matrix.
    k_F = 3.72e-8;    % N/(rad/s)^2
    k_M = 7.73e-11;   % Nm/(rad/s)^2
    L = 0.0325;       % m (Arm length)
    
    w1_2 = motors_f(1,:).^2;
    w2_2 = motors_f(2,:).^2;
    w3_2 = motors_f(3,:).^2;
    w4_2 = motors_f(4,:).^2;
    
    T     = k_F * (w1_2 + w2_2 + w3_2 + w4_2);
    tau_x = k_F * L * (-w1_2 - w2_2 + w3_2 + w4_2);
    tau_y = k_F * L * (-w1_2 + w2_2 + w3_2 - w4_2);
    tau_z = k_M * (-w1_2 + w2_2 - w3_2 + w4_2); % Corrected sign for yaw torque
    
    u_full = [T; tau_x; tau_y; tau_z];

    % --- 4. Square Hankel Sampling (Coulson Eq 5.9 Rule) ---
    % We need T observations such that the data constraint matrix is square.
    % Image mapping: U_p, Y_p, U_f, Y_f concatenation.
    m = size(u_full, 1);
    p_dims = size(y_full, 1);
    
    % Ideal minimal length to satisfy persistency of excitation
    T_ideal = max([ ...
        (m + 1) * (T_ini + T_f + p_dims) - 1, ...
        (m + p_dims + 1) * (T_ini + T_f) - 1 ...
    ]);

    % Add a minor buffer to make it comfortably overdetermined / robust (e.g., +20)
    T_target = ceil(T_ideal) + 20;
    
    end_idx = opts.StartIdx + T_target - 1;
    
    if end_idx > length(t)
        warning('Requested time horizon + square Hankel buffer exceeds %s dataset size. Using maximum available.', csv_filename);
        end_idx = length(t);
    end
    
    % Slice Data
    U_dt = u_full(:, opts.StartIdx:end_idx);
    Y_dt = y_full(:, opts.StartIdx:end_idx);
    t_dt = t(opts.StartIdx:end_idx);
    
    fprintf('Loaded [%s] -> %d samples extracted (Square Hankel Optimal: %d).\n', ...
            csv_filename, size(U_dt, 2), T_ideal);
end

function euler = quatToEuler(q)
%QUATTOEULER Convert [qx; qy; qz; qw] samples to ZYX roll/pitch/yaw.
    qx = q(1, :);
    qy = q(2, :);
    qz = q(3, :);
    qw = q(4, :);

    roll = atan2(2 .* (qw .* qx + qy .* qz), ...
                 1 - 2 .* (qx.^2 + qy.^2));

    pitch_arg = 2 .* (qw .* qy - qz .* qx);
    pitch_arg = max(-1, min(1, pitch_arg));
    pitch = asin(pitch_arg);

    yaw = atan2(2 .* (qw .* qz + qx .* qy), ...
                1 - 2 .* (qy.^2 + qz.^2));

    euler = [roll; pitch; yaw];
end
