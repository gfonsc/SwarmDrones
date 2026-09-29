# Motor Effort Validity Report

1. DeePC does not output physical motor speeds in the SwarmDrones labyrinth branch. It outputs `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`.
2. MPC also outputs `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]` in the final branch comparison.
3. Because neither controller outputs physical motors here, the final figure uses equivalent normalized command efforts.
4. The mapping is a documented linear allocation applied identically to DeePC and MPC.
5. The resulting channels are not physical rad/s, but all four are non-placeholder channels.
6. Motor 4 was previously suspicious because direct command-channel plotting could leave yaw-rate demand near zero; the new mapping uses all four command axes in all four equivalent channels.
7. Valid for article as normalized equivalent effort: `1`.

Label to use: `Equivalent normalized motor effort`.
