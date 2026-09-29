# SwarmDrones

Production-quality implementation of data-driven predictive controllers and autonomous navigation for quadcopter swarm applications.

## Repository Structure

```
SwarmDrones/
├── controllers/
│   └── deepc/                              ← DeePC controller (self-contained)
│       ├── deepcSetup.m                    ← OFFLINE: Hankel + precompute QP
│       ├── deepcAlgorithm.m                ← ONLINE:  solve QP each control step
│       └── functions/
│           ├── buildHankelMatrix.m          ← Eq. (5.4): Hankel partitioning
│           ├── checkPersistency.m           ← Eq. (5.9): PE verification
│           ├── computeSteadyStateG.m        ← Eq. (5.7): g_r steady-state mapper
│           ├── buildHessian.m               ← Eq. (5.5-5.6): constant QP Hessian
│           ├── buildLinearCost.m            ← Per-step linear cost vector
│           ├── buildConstraintMatrix.m      ← Equality + inequality constraints
│           ├── solveQP.m                    ← MOSEK-first / quadprog fallback
│           ├── extractSolution.m            ← u* = Uf*g*, y* = Yf*g*
│           └── validateParams.m             ← Parameter validation + defaults
├── path_planner/                            ← Obstacle avoidance planner
│   ├── pathPlannerSetup.m                   ← INIT:   configure FSM + parameters
│   ├── pathPlannerStep.m                    ← STEP:   one FSM tick → setpoint r(t)
│   └── functions/
│       ├── detectObstacles.m                ← FOV-based obstacle detection
│       ├── checkCollision.m                 ← Collision detection
│       ├── createObstacleEnvironment.m      ← Pre-built obstacle scenarios
│       └── plotAvoidanceResults.m           ← Visualization
├── examples/
│   └── demo_linear_system.m                 ← End-to-end LTI demo
└── README.md
```

## Architecture: Two Separate Worlds

```
┌─────────────────────────────────────────┐
│  PATH PLANNER (planning layer)          │
│  pathPlannerSetup() → pathPlannerStep() │
│                                         │
│  Input:  pos, yaw, obstacles            │
│  Output: setpoint r(t), yaw_cmd         │
└──────────────┬──────────────────────────┘
               │  r(t)
               ▼
┌─────────────────────────────────────────┐
│  CONTROLLER (tracking layer)            │
│  deepcSetup()  → deepcAlgorithm()       │
│  (or MPC, LQR, PID — same interface)    │
│                                         │
│  Input:  r(t), uIni, yIni               │
│  Output: u(t)                           │
└─────────────────────────────────────────┘
```

The two layers communicate **only** through function inputs and outputs:
- **Path planner** knows nothing about controllers
- **Controllers** know nothing about obstacles
- The simulation loop is the only place where both are called

## Path Planner

Implementation of **wall-follower obstacle avoidance** based on:

> N. Ramires, "Desenvolvimento de um algoritmo de evasão de obstáculos para VANTs," TCC, UTFPR, 2023.

### Features

1. **Lateral FSM** — 7-state wall-follower (ALIGN → GO_TO_GOAL → ORIENT → TRANSLATE → ALIGN)
2. **Parallel Altitude FSM** — Independent vertical escape (IDLE / ASCENDING / DESCENDING)
3. **Three FOV Modes** — `cone`, `corridor`, or `dual` (Nycolas §7.1 proposal)
4. **Stuck Detection** — Distance divergence monitoring + lateral escape
5. **3D Obstacles** — Cylindrical obstacles with height range [z_min, z_max]

### Quick Start

```matlab
%% 1. OFFLINE: Setup planner
params.Ts = 0.01;
params.fov_mode = 'corridor';
planner = pathPlannerSetup([10; 10; 1.0], params);

%% 2. Create obstacles
obstacles = createObstacleEnvironment('mixed_3d');

%% 3. ONLINE: Planning loop
for k = 1:N
    [setpoint, yaw_cmd, planner, info] = pathPlannerStep(planner, pos, yaw, obstacles);

    % Feed setpoint to ANY controller:
    [uOpt, ~, ~] = deepcAlgorithm(model, uIni, yIni, setpoint);
    % ... or MPC, LQR, PID ...
end
```

## DeePC Controller

Implementation of **Data-Enabled Predictive Control (DeePC)** based on:

> J. Coulson, "Data-Driven Predictive Control," PhD Thesis, ETH Zürich, 2021.

### Quick Start

```matlab
%% 1. OFFLINE: Setup with training data
params.T_ini = 6;  params.T_f = 25;
params.lambda_y = 7.5e8;  params.lambda_g = 500;
params.Q = 10 * eye(p);   params.R = 0.01 * eye(m);

model = deepcSetup(u_train, y_train, params);

%% 2. ONLINE: Control loop (Algorithm 5.1)
for t = 1:T_sim
    [uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r);

    uApply = uOpt(1:m);              % Receding horizon: first input
    yMeas  = plantStep(x, uApply);   % Apply to plant

    uIni = [uIni(m+1:end); uApply];  % Shift input buffer
    yIni = [yIni(p+1:end); yMeas];   % Shift output buffer
end
```

### Solver Support

The QP solver (`solveQP.m`) uses **MOSEK-first with quadprog fallback**:
- If MOSEK is installed → uses MOSEK (faster, recommended)
- If not → falls back to MATLAB's `quadprog` (Optimization Toolbox)

### Thesis Equation Traceability

| Function | Thesis Reference |
|----------|-----------------|
| `buildHankelMatrix.m` | Def. 2.5, Eq. (5.4) |
| `checkPersistency.m` | Lemma 2.2, Eq. (5.9) |
| `computeSteadyStateG.m` | Eq. (5.7) |
| `buildHessian.m` | Eq. (5.5), (5.6) |
| `buildConstraintMatrix.m` | Eq. (5.5) constraints |
| `deepcAlgorithm.m` | Algorithm 5.1 |

## Requirements

- MATLAB R2020b or later
- Optimization Toolbox (for `quadprog`)
- MOSEK (optional, for faster QP solving)

## License

TBD