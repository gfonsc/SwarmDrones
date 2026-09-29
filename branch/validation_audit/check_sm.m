d = load('C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch\mpc_baseline\results\mpc_obstacle_results.mat');
sm = d.obstacleResults.iterativeResult.log.safetyMargin;
disp(class(sm));
disp(size(sm));
if isstruct(sm)
    disp(fieldnames(sm));
    fprintf('margin class: %s\n', class(sm(1).margin));
end
