% run_audit_details.m
d_mpc = load('C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch\mpc_baseline\results\mpc_obstacle_results.mat');
u_mpc = d_mpc.obstacleResults.iterativeResult.log.u;
euler_mpc = d_mpc.obstacleResults.iterativeResult.log.euler;

fprintf('MPC u size: %d x %d\n', size(u_mpc, 1), size(u_mpc, 2));
fprintf('MPC u(:,4) sum: %f\n', sum(abs(u_mpc(:,4))));

fprintf('MPC euler size: %d x %d\n', size(euler_mpc, 1), size(euler_mpc, 2));
fprintf('MPC euler mean roll: %f, pitch: %f, yaw: %f\n', mean(euler_mpc(:,1)), mean(euler_mpc(:,2)), mean(euler_mpc(:,3)));
