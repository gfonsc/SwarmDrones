function checkPersistency(Up, Yp, Uf, uData, params)
% CHECKPERSISTENCY  Verify persistency of excitation and data sufficiency.
%
%   checkPersistency(Up, Yp, Uf, uData, params)
%
%   Performs two checks from the thesis:
%
%   1. RANK CONDITION — The stacked matrix [Up; Yp; Uf] must have full
%      row rank for the Fundamental Lemma to hold (Lemma 2.2).
%
%   2. DATA LENGTH CONDITION — Eq. (5.9):
%      T >= max{ (m+1)(T_ini + T_f + n) - 1,  (m+p+1)(T_ini + T_f) - 1 }
%
%      The second term ensures the Hankel matrix is at least square,
%      which is critical for nonlinear/noisy systems (Section 5.3.2).
%
%   Inputs:
%       Up     - Past input Hankel matrix
%       Yp     - Past output Hankel matrix
%       Uf     - Future input Hankel matrix
%       uData  - Raw input data (m x T), used to extract dimensions
%       params - Struct with T_ini, T_f, verbose
%
%   Reference: Coulson thesis, Eq. (5.9) and Section 5.3.2

    [m, T] = size(uData);
    p      = size(Yp, 1) / params.T_ini;
    T_ini  = params.T_ini;
    T_f    = params.T_f;
    numCols = size(Up, 2);

    %% Check 1: Rank of [Up; Yp; Uf]
    %  For the Fundamental Lemma (Lemma 2.2), this matrix must have full
    %  row rank = m*T_ini + p*T_ini + m*T_f
    stackedMatrix = [Up; Yp; Uf];
    numRows       = size(stackedMatrix, 1);
    matrixRank    = rank(stackedMatrix);
    isFullRank    = (matrixRank >= numRows);

    if params.verbose
        fprintf('  Persistency check: rank([Up;Yp;Uf]) = %d / %d rows', ...
            matrixRank, numRows);
        if isFullRank
            fprintf(' [PASS]\n');
        else
            fprintf(' [WARN: rank deficient]\n');
        end
    end

    if ~isFullRank
        warning('deepc:rankDeficient', ...
            ['[Up; Yp; Uf] is rank-deficient (rank=%d, rows=%d).\n' ...
             'The input data may not be persistently exciting of sufficient order.\n' ...
             'Consider collecting more data or using richer excitation signals.'], ...
            matrixRank, numRows);
    end

    %% Check 2: Data length condition — Eq. (5.9)
    %  T >= (m+p+1)*(T_ini + T_f) - 1  (square Hankel condition)
    %  Note: we cannot check the first term of Eq. (5.9) without knowing n
    %  (state dimension), which is unknown in the data-driven setting.
    T_min_square = (m + p + 1) * (T_ini + T_f) - 1;
    isSquareOrWide = (numCols >= numRows);

    if params.verbose
        fprintf('  Data length: T=%d, T_min(square Hankel)=%d, numCols=%d vs numRows=%d', ...
            T, T_min_square, numCols, numRows);
        if T >= T_min_square
            fprintf(' [PASS]\n');
        else
            fprintf(' [WARN: Hankel is tall]\n');
        end
    end

    if T < T_min_square
        warning('deepc:insufficientData', ...
            ['Data length T=%d is less than the square-Hankel threshold %d.\n' ...
             'Eq. (5.9): T >= (m+p+1)*(T_ini+T_f) - 1 = %d.\n' ...
             'Performance may degrade significantly (see thesis Figure 5.3).'], ...
            T, T_min_square, T_min_square);
    end

    if ~isSquareOrWide
        warning('deepc:tallHankel', ...
            ['Hankel matrix has %d columns < %d rows (tall matrix).\n' ...
             'For nonlinear/noisy systems, the Hankel matrix should be at least\n' ...
             'square so its column space spans the trajectory space (Section 5.3.2).'], ...
            numCols, numRows);
    end
end
