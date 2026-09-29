d1 = load('C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch\results\iterative_obstacle_results.mat');
disp(class(d1.result.safetyMargin));
disp(size(d1.result.safetyMargin));
if isstruct(d1.result.safetyMargin)
    disp(fieldnames(d1.result.safetyMargin));
end
