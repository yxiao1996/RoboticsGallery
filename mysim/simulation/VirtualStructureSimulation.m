classdef VirtualStructureSimulation < simulation
    %VIRTUALSTRUCTURESIMULATION 
    % A simulation of multi-robot system formation control
    % using virtual structure method
    
    properties
        waypoints     % waypoints of virtual structure
        idx
        VS            % circle virtual structure
        controller_vs % controller for the virtual structure
        actuator_vs   % actuator for the virtual structure
        prev_poses
        deadzone_radius
    end
    
    methods
        function obj = VirtualStructureSimulation(map,swarmInfo)
            %VIRTUALSTRUCTURESIMULATION 
            % construct the simulation
            obj.sampleTime = 0.05;
            obj.deadzone_radius = 0.1;
            obj.numRobots = swarmInfo.numRobots;
            obj.world = world(swarmInfo);
            obj.controllers = cell(1,obj.numRobots);
            % compute virtual structure
            obj.VS = obj.compute_VS();
            % plan the trajectory of the virtual structure
            start = mean(swarmInfo.poses,2);
            goal = [2 2];
            map_inf = copy(map);
            inflate(map_inf,1.0);
            %show(map_inf);
            waypoints = planPRM(map_inf,start,goal);
            obj.waypoints = waypoints;
            obj.controller_vs = DiffDrivePursueWayPoints(waypoints);
            obj.actuator_vs = actuatorDiffDrive(0.1,0.5);
            % assign target pursuit controller for each robot
            for i = 1:obj.numRobots
                obj.controllers{i} = DiffDriveFollowVS(obj.deadzone_radius);
            end
            % assign differential drive actuator to each robot
            for i = 1:obj.numRobots
                robotInfo = swarmInfo.infos{i};
                R = robotInfo.wheel_radius;
                L = robotInfo.body_width;
                if (robotInfo.type == "DiffDrive")
                    obj.actuators{i} = actuatorDiffDrive(R,L);
                end
            end
            % setup environment physics
            obj.physics = AABB(map,swarmInfo.numRobots,0.25,true);
            obj.prev_poses = swarmInfo.poses;
            obj.idx = 1;
        end
        
        function obj = step(obj)
            % sensor readings
            readings = obj.sensor_phase();
            % fit virtual structure into current system
            %poses = obj.world.get_poses();
            %[R,t] = obj.fit_VS(poses);

            %vs_xy = t(1:2);
            %vs_theta = angle(R(1,1)+1j*R(2,1));
            %vs_pose = [vs_xy' vs_theta]';
            % compute control for virtual structure
            %vs_control = obj.controller_vs.compute_control(vs_pose,readings);
            % actuatr virtual structure
            %vs_vel = obj.actuator_vs.actuate(vs_control,vs_pose);
            %vs_pose = vs_pose + vs_vel * obj.sampleTime;
            vs_pose = obj.waypoints(obj.idx,:);
            if (obj.idx < size(obj.waypoints,1))
                obj.idx = obj.idx + 1;
            end
            next_R = obj.compute_SO2(0);
            next_t = vs_pose(1:2);
            vs_points = obj.VS(1:2,:)' * next_R + next_t;
            %disp(vs_points);
            % control robot to follow virtual structure
            reach_flag = false;
            while(~reach_flag)
                targets = obj.assign_target(vs_points);
                controls = obj.control_phase(readings,targets);
                poses = obj.actuate_phase(controls);
                obj = obj.physics_phase(poses);
                obj.visualize();
                reach_flag = obj.check_reach(poses,targets);
            end
            hold on;
            obj.draw_VS(vs_points);hold off;
        end
        
        function controls = control_phase(obj,readings,targets)
            poses = obj.world.get_poses();
            controls = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                ctl = obj.controllers{i};
                pose = poses(:,i);
                target = targets(i,:);
                controls{i} = ctl.compute_control(target,pose);
            end
        end
        
        function poses = actuate_phase(obj,controls)
            poses = obj.world.get_poses(); % current system states
            for i = 1:obj.numRobots
                actuator = obj.actuators{i};
                pose = poses(:,i);
                control = controls{i};
                vel = actuator.actuate(control,pose);
                poses(:,i) = pose  + vel * obj.sampleTime;
            end
        end
        
        function visualize(obj)
            readings = obj.world.readSensors();
            obj.world.visualize(readings);
        end
        
        function vs = compute_VS(obj)
            vs = zeros(3,obj.numRobots);
            radius = 0.75;
            for i = 1:obj.numRobots
                angle = (2*pi/obj.numRobots)*i;
                vs(1:2,i) = radius * [cos(angle) sin(angle)];
            end
        end
        
        function [R,t] = fit_VS(obj,poses)
            xy = zeros(3,obj.numRobots);
            xy(1:2,:) = poses(1:2,:);
            %disp(xy);
            %disp(obj.VS);
            [R,t] = icp(xy,obj.VS);
        end
        
        function draw_VS(obj,vs_pose)
            vs_pose = vs_pose';
            for i = 1:obj.numRobots-1
                p1 = vs_pose(:,i);
                p2 = vs_pose(:,i+1);
                plot([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',2);
            end
            p1 = vs_pose(:,obj.numRobots);
            p2 = vs_pose(:,1);
            plot([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',2);
        end
        
        function R = compute_SO2(obj,a)
            R = [
                cos(a) -sin(a);
                sin(a) cos(a)
            ];
        end
        
        function new_targets = assign_target(obj,targets)
            new_targets = zeros(size(targets));
            poses = obj.world.get_poses();
            cost_mat = zeros(obj.numRobots,obj.numRobots);
            % construct cost matrix
            for i = 1:obj.numRobots
                for j = 1:obj.numRobots
                    p_i = poses(1:2,i);
                    t_j = targets(j,:);
                    cost_mat(i,j) = sqrt((p_i(1)-t_j(1))^2 + (p_i(2)-t_j(2))^2);
                end
            end
            % hungarian algorithm
            [assignment,cost] = munkres(cost_mat);
            for i = 1:obj.numRobots
                id = assignment(i);
                new_targets(i,:) = targets(id,:);
            end
        end
        
        function reach = check_reach(obj,poses,targets)
            target = targets';
            poses = poses(1:2,:);
            error = 0;
            for i = 1:obj.numRobots
                p = poses(:,i);
                t = target(:,i);
                error = error + sqrt((p(1)-t(1))^2+(p(2)-t(2))^2);
            end
            if (error < (obj.deadzone_radius+0.02) * obj.numRobots)
                reach = true;
            else
                reach = false;
            end
        end
    end
end

