% Author : Nikhil Sharma, nikhil.sharma.mcmaster@gmail.com

classdef sensor_model < handle
	% Abstract class to derive sub-sensor classes from. See a derived class or usage.
    properties(Access=public)
        sensor_type;
        sigma_r=0; % range standard deviation
        sigma_azim=0; % azimuth standard deviation, azimuth measured with x axis
        sigma_elev = 0; % Elevation standard deviation, elevation measured w.r.t the x-y plane
        sigma_x =0 
        sigma_y = 0
        sigma_z = 0
        dim = 2; % measurement vector dimension
        x_pos = 0;
        y_pos = 0;
        z_pos = 0;
        lat_pos = 0;
        lon_pos = 0;
        alt_pos = 0';
        file_name;
		
		% Fill in more properties here
    end
    
    methods
        function object = sensor_model(file_name_in)
            object.file_name = file_name_in;
        end
        
        function initialize(obj)
            myFile_struct = xml2struct(obj.file_name);
            obj.sensor_type = myFile_struct.sensorParam.sensorType.Text;
            obj.dim = str2double(myFile_struct.sensorParam.dim.Text);
            
            if strcmp(obj.sensor_type, 'radar') || strcmp(obj.sensor_type, 'range')
                obj.sigma_r = str2double(myFile_struct.sensorParam.sigmaR.Text);
                obj.sigma_azim = str2double(myFile_struct.sensorParam.sigmaAzim.Text);
                obj.sigma_elev = str2double(myFile_struct.sensorParam.sigmaElev.Text);
            end
            
            if strcmp(obj.sensor_type, 'linear')
                obj.sigma_x = str2double(myFile_struct.sensorParam.sigmaX.Text);
                obj.sigma_y = str2double(myFile_struct.sensorParam.sigmaY.Text);
                obj.sigma_z = str2double(myFile_struct.sensorParam.sigmaZ.Text);
            end

            obj.x_pos = str2double(myFile_struct.sensorParam.xPos.Text);
            obj.y_pos = str2double(myFile_struct.sensorParam.yPos.Text);
            obj.z_pos = str2double(myFile_struct.sensorParam.zPos.Text);

            obj.lat_pos = str2double(myFile_struct.sensorParam.latPos.Text);
            obj.lon_pos = str2double(myFile_struct.sensorParam.lonPos.Text);
            obj.alt_pos = str2double(myFile_struct.sensorParam.altPos.Text);
            

        end
		
		function set_location_cart(obj, varargin)
			n = numel(varargin);
			obj.x_pos = varargin{1};
			obj.y_pos = varargin{2};
			if n > 2
				obj.z_pos = varargin{3};
			end
		end
    end
end