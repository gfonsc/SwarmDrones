data = load('C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch\mpc_baseline\results\mpc_baseline_results.mat');
disp('MPC baseline fields:');
disp(fieldnames(data));
if isfield(data, 'results')
    disp(fieldnames(data.results));
end
