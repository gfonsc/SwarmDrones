function [Up, Yp, Uf, Yf] = buildHankelMatrix(uData, yData, T_ini, T_f)
% BUILDHANKELMATRIX  Construct and partition Hankel matrices for DeePC.
%
%   [Up, Yp, Uf, Yf] = buildHankelMatrix(uData, yData, T_ini, T_f)
%
%   Builds the block Hankel matrices from input/output data and partitions
%   them into past (p) and future (f) blocks as defined in Eq. (5.4):
%
%       [U_p]                                   [Y_p]
%       [   ] := H_{T_ini + T_f}(u^d_{[1,T]}),  [   ] := H_{T_ini + T_f}(y^d_{[1,T]})
%       [U_f]                                   [Y_f]
%
%   where U_p has the first T_ini block rows and U_f has the last T_f block rows.
%
%   Inputs:
%       uData  - Input data matrix (m x T), m = number of inputs, T = samples
%       yData  - Output data matrix (p x T), p = number of outputs
%       T_ini  - Number of block rows in past partition (initial condition)
%       T_f    - Number of block rows in future partition (prediction horizon)
%
%   Outputs:
%       Up - Past input Hankel:   (m * T_ini) x numCols
%       Yp - Past output Hankel:  (p * T_ini) x numCols
%       Uf - Future input Hankel: (m * T_f)   x numCols
%       Yf - Future output Hankel:(p * T_f)   x numCols
%
%   where numCols = T - L + 1, and L = T_ini + T_f is the total depth.
%
%   Reference: Coulson thesis, Eq. (5.4) and Definition 2.5

    %% Validate inputs
    [m, T] = size(uData);
    [p, Ty] = size(yData);
    assert(T == Ty, 'uData and yData must have the same number of columns (T=%d vs %d).', T, Ty);

    L = T_ini + T_f;              % Total depth of Hankel matrix
    numCols = T - L + 1;          % Number of columns

    assert(numCols >= 1, ...
        'Insufficient data: T=%d samples, need at least L=T_ini+T_f=%d.', T, L);

    %% Build full Hankel matrices for u and y
    %  H(w) has L block rows, each of dimension q (q=m for inputs, q=p for outputs)
    %  H(w) is (q*L) x numCols
    Hu = singleHankel(uData, L, m, T, numCols);
    Hy = singleHankel(yData, L, p, T, numCols);

    %% Partition into past and future blocks — Eq. (5.4)
    %  Past:   first T_ini block rows
    %  Future: last  T_f   block rows
    Up = Hu(1 : m*T_ini,       :);
    Uf = Hu(m*T_ini+1 : end,   :);

    Yp = Hy(1 : p*T_ini,       :);
    Yf = Hy(p*T_ini+1 : end,   :);
end


function H = singleHankel(data, L, q, T, numCols)
% SINGLEHANKEL  Build a block Hankel matrix for a single signal.
%
%   Definition 2.5 (Coulson thesis):
%   For a signal w = (w(1), w(2), ..., w(T)) with w(t) in R^q,
%   the Hankel matrix of depth L is:
%
%       H_L(w) = [ w(1)   w(2)   ...  w(T-L+1) ]
%                [ w(2)   w(3)   ...  w(T-L+2) ]
%                [  :      :           :       ]
%                [ w(L)   w(L+1) ...  w(T)     ]
%
%   Each w(t) is a column vector of dimension q, so H_L has q*L rows.

    H = zeros(q * L, numCols);

    for i = 1:L
        rowStart = (i - 1) * q + 1;
        rowEnd   = i * q;
        H(rowStart:rowEnd, :) = data(:, i : i + numCols - 1);
    end
end
