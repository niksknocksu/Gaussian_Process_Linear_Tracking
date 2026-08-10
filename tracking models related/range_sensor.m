classdef range_sensor <  sensor_model
   properties
   end
   methods
       function z = get_meas(object, varargin)
           target_x = varargin{1};
           target_y = varargin{2};
           if numel(varargin) > 2
            target_z = varargin{3};
           end
		   R = object.sigma_r^2;
           switch numel(varargin)
               case 1
                   error("Need at least two coordinates")
               case 2	
                   z = norm([target_x, target_y]) + mvnrnd(zeros(object.dim,1), R)';
               case 3
                   z = norm([target_x, target_y, target_z]) + mvnrnd(zeros(object.dim,1), R)';
           end
       end
       
       function H = get_Jacobian(obj, varargin)
			% Returns jacobian matrix in the case of constant velocity states
			target_x = varargin{1};
			target_y = varargin{2};
			x_rel = target_x - obj.x_pos;
			y_rel = target_y - obj.y_pos;
			if numel(varargin) > 2
				target_z = varargin{3};
				z_rel = target_z - obj.z_pos;
			end
           
          switch numel(varargin)
              case 1
                  % 2-D target
				  % Add code here
				  error("Need at least two coordinates")
              case 2
                  % 3-D target
                  H = [x_rel/sqrt(x_rel^2 + y_rel^2),  y_rel/sqrt(x_rel^2 + y_rel^2), 0, 0];
			  case 3
					H = [x_rel/sqrt(x_rel^2 + y_rel^2 + z_rel^2),  y_rel/sqrt(x_rel^2 + y_rel^2 + z_rel^2), z_rel/sqrt(x_rel^2 + y_rel^2 + z_rel^2);...
						0, 0, 0];			
          end
       end
   end
end