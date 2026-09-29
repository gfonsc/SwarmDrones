# Obstacle Constraint Effectiveness Report

## 1. Safety Metrics Audit
- **DeePC (Iterative):** The minimum safety margin $h(k)$ observed in the result structures was approximately $-0.0985$ m.
- **MPC (Iterative):** The minimum safety margin was similarly observed below zero.
- **Safety Respected?** No. Absolute safety was not perfectly maintained. Both controllers allowed slight penetrations into the expanded safety buffer ($r_{safe} = r_{obs} + 0.25$m).

## 2. Constraint Formulation
- **Inside Optimization:** Yes. The obstacle constraints were explicitly included inside the DeePC QP solver and MPC solver as linear inequalities (hyperplanes). 
- **Soft Constraints:** Both solvers formulated the obstacle boundaries as soft constraints by adding slack variables to the QP. The slack penalty weight $\rho_{obs}=10^4$ was not sufficient to prevent all margin violations, likely because of the short prediction horizon ($N=8$) preventing the drone from decelerating early enough before sharp turns.

## 3. Effectiveness Conclusion
The obstacle constraints were *effective* at keeping the quadrotor from crashing into the physical cylinder cores, but they were *not absolute*. The results demonstrate that soft-constrained successive linearization provides a feasible evasion path, but claims of "perfect" or "guaranteed" collision avoidance are false and must be strictly avoided in the article.
