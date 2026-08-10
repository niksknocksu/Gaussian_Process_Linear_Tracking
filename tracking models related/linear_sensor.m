classdef linear_sensor < sensor_model
    properties
    end
    
    methods
        function z = get_meas(object, varargin)
            target_x = varargin{1};
            target_y = varargin{2};
            if numel(varargin) > 2
                target_z = varargin{3};
            end
            
            switch object.dim
                case 1
                    error('Dimension value 1 supplied for linear sensor, minimum 2 required.')

                case 2
                    R = diag([object.sigma_x^2, object.sigma_y^2]);
                    z = [target_x; target_y] + mvnrnd(zeros(object.dim,1), R)';

                case 3
                    R = diag([object.sigma_x^2, object.sigma_y^2, object.sigma_z^2]);
                    z = [target_x; target_y; target_z] + mvnrnd(zeros(object.dim,1), R)';
            end
        end

        function R = get_R(object)
            switch object.dim
              case 2
                  R = diag([object.sigma_x^2, object.sigma_y^2]);
              case 3
                  diag([object.sigma_x^2, object.sigma_y^2, object.sigma_z^2]);
            end
        end

    end    
end
   