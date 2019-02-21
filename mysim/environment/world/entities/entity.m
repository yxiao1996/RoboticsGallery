classdef entity
    %ENTITY base class for all entities in the simulated environment
    % the properties stored for each entity are used for visualization
    % 
    
    properties
        position
        orientation
        type
        shape
    end
    
    methods
        function obj = entity(postion,orientation,type,shape)
            %ENTITY constructor for the entity
            %   �˴���ʾ��ϸ˵��
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 �˴���ʾ�йش˷�����ժҪ
            %   �˴���ʾ��ϸ˵��
            outputArg = obj.Property1 + inputArg;
        end
    end
end

