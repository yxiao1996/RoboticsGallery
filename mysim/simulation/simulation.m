classdef simulation
    %SIMULATION Abstract class for a simulation
    
    properties
        sampleTime
        numRobots
        world
        physics
        controllers
        actuators
    end
    
    methods (Abstract)
        %sensor_phase(obj)
        control_phase(obj)
        actuate_phase(obj)
        %physics_phase(obj)
        visualize(obj)
        step(obj)
    end
    
    methods
        function readings = sensor_phase(obj)
            readings = obj.world.readSensors();
        end
        
        function obj = physics_phase(obj,poses)
            poses = obj.physics.check_obstacles(poses,obj.prev_poses);
            %poses = obj.physics.check_robots(poses,obj.prev_poses);
            obj.world = obj.world.update_poses(poses);
            obj.prev_poses = poses;
        end
        
    end
    
end

