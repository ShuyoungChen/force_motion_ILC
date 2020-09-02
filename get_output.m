function out = get_output(q0, qin, n, Ks, Kp, pos_obj, dt, H, P, type)
                          
    out = zeros(6, n);
    q_c = q0;
    q_pre = q0;
    
    Fdz = 1000;
    
    i = 1;
    while i <= n
        %%%%%%%%%%%%
        q_new = q_c;
        
        % forward kinematics
        [~,pos] = fwdkin(q_new,type,H,P,6);
        
        Fd0 = 50;
        if pos(3) >= pos_obj
            F = 0;  
            v_z = Kp*20*(F-Fd0);
            vel_cmd = [0,0,0,0,0,v_z]';
        else
            F = -Ks*(pos(3)-pos_obj);
            if F < Fdz-0.5 && i == 1
                v_z = Kp/2*(F-Fdz);
                vel_cmd = [0,0,0,0,0,v_z]';
            else
                cur_in = qin(:, i);
                v_z = Kp*(F-cur_in(6));
                cur_in(6) = v_z;

                q_dot = (q_new - q_pre)/dt;

                J = getJacobian(q_new,type,H,P,6);
                v = J*q_dot;
                out(:, i) = [v(1:5); F];
                vel_cmd = cur_in;
                i = i+1;
            end
        end
        
        J = getJacobian(q_new,type,H,P,6);
        joints_vel = inv(J)*vel_cmd;
  
        q_c = q_new + joints_vel*dt;    
        q_pre = q_new;
    end
end