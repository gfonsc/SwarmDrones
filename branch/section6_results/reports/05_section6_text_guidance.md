# Section 6 Text Guidance

## 1. Subsection 6.1 (No Obstacle)
"Figure 6a illustrates the unconstrained trajectory tracking performance of both the data-driven DeePC and the model-based MPC within the labyrinth boundaries. Without explicit obstacle avoidance constraints activated, both predictive formulations successfully track the dynamic serpentine reference. The data-driven approach demonstrates remarkable parity with the analytical MPC, validating the fidelity of the input-output Hankel subspace mapping."

## 2. Subsection 6.2 (With Obstacles)
"Upon activating the obstacle-avoidance state machine (Figures 6b and 6c), both controllers dynamically generate safe local bounds via successive linearization. As the quadrotor approaches the cylindrical obstacles, the optimization successfully commands deviations from the nominal reference. Figure 6h confirms that the formulation aggressively penalizes collision paths; however, owing to the finite prediction horizon ($N=8$) and the soft-constraint formulation ($\rho_{obs}=10^4$), minor transient penetrations into the expanded safety buffer occur, preventing artificial infeasibility during sharp maneuvers."

## 3. Subsection 6.3 (Time-Lagged Visualization)
"To illustrate the receding-horizon predictive mechanism, a snapshot of the trajectory optimization is visualized. The algorithm continuously forecasts the quadrotor's spatial state across the upcoming horizon to evaluate potential boundary intersections. This predictive spatial awareness allows the drone to commence evasive maneuvers proactively, ensuring sufficient response time despite actuation lag."

## 4. Figure Interpretation Notes
- **Trajectory Maps:** Show the spatial deviations caused by the constraints.
- **Safety Margin:** Values $h < 0$ mean the drone breached the *padded* safe margin $r_{safe}$, not necessarily the physical pillar.
- **State Machine:** Demonstrates the logic transitioning cleanly from nominal tracking to active avoidance.

## 5. What NOT to Claim
- Do NOT claim perfect, guaranteed, zero-violation safety. The $h(k)$ plot clearly dips slightly below zero.
- Do NOT claim DeePC has better tracking than MPC. They perform similarly, which is the actual intended takeaway.
