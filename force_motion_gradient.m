function qin = force_motion_gradient(q0, qi, qd, n_iter, alpha, t, n, a, Ks, Kp, pos_obj, dt, H, P, type)

% iterative learning control to reduce the tracking error
% G is the system
% qi is the initial input
% qd is the desired trajectory for tracking
% n is the number of iterations
% alpha is the learning rate
% t is the time
% q0: initial joint angles

qin = qi;

for i = 1:n_iter
    out = get_output(q0, qin, n, Ks, Kp, pos_obj, dt, H, P, type);
    qsim_1 = out;
    
    %figure(i)
    %plot(t, qsim_1(6, :));
    %qsim_1 = lsim(G,qin,t);
        
    %qsim_1 = qsim_1';
    if i == n_iter 
        figure(i+1)
        %%%%%%%%%%%%%%%%%
        h = plot(t,qd(6, :),'-',t, qin(6, :), '-.', t, qsim_1(6, :),'--');
        set(h(1), 'linewidth', 2);
        set(h(2), 'linewidth', 2);
        set(h(3), 'linewidth', 2);
        set(gca,'fontsize',16, 'fontweight','bold');
        xlabel('Time (s)','fontweight','bold','fontsize',16);
        ylabel('Force (N)','fontweight','bold','fontsize',16);
        lgd = legend('desired output', 'optimal input', 'optimal output');
        lgd.FontSize = 16;
        lgd.FontWeight = 'bold';
        disp(norm(qd(6, 201:end) - qsim_1(6, 201:end)));
    end
    
    % tracking error
    disp("the force tracking error is: ")
    norm(qd(6, 201:end) - qsim_1(6, 201:end))
    
    error = qsim_1 - qd;
    
    err_flip = flip(error);
    u = qin + a*err_flip;
    
    out2 = get_output(q0, u, n, Ks, Kp, pos_obj, dt, H, P, type);
    qsim_2 = out2;
    
    %qsim_2 = lsim(G,u,t);
        
    %qsim_2 = qsim_2';
    error_2 = qsim_2 - qsim_1;
    err_flip_2 = flip(error_2);
   
    qin = qin - alpha*err_flip_2;
    
end

%%%%%%