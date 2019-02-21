classdef actuatorDiffDrive < actuator
    % Actuator of Differential Drive Robot 
    % calculate space displacement giving control input
    % using the dynamical model of robot 
    %   �˴���ʾ��ϸ˵��
    
    properties
        R
        L
    end
    
    methods
        function obj = actuatorDiffDrive(R,L)
            %ACTUATOR differential drive dynamics
            obj.R = R;
            obj.L = L;
            obj.dynamics = DifferentialDrive(obj.R,obj.L);
        end
        
        function vel = actuate(obj,control,pose)
            vRef = control.vRef;
            wRef = control.wRef;
            % compute velocity in the world frame 
            [wL,wR] = inverseKinematics(obj.dynamics,vRef,wRef);
            [v,w] = forwardKinematics(obj.dynamics,wL,wR);
            velB = [v;0;w]; % Body velocities [vx;vy;w]
            vel = bodyToWorld(velB,pose);  % Convert from body to world
        end
    end
end

