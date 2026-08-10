% A basic class for declaring Gaussian distributed objects

classdef GaussianClass
    properties
       mean; 
    end 
    properties
       covariance; 
    end
    
    methods    
        function object = GaussianClass(meanValue,covValue)
            if size(meanValue,2)~= 1 
               meanValue = meanValue';  % Convert to column vector
            end
            object.mean = meanValue;
            object.covariance = covValue;
        end 
        
        function y = pdf(object,x)
            if size(x,2) ~= 1
               x = x'; % Convert to column vector
            end
            y = (1/(sqrt(det(2.*pi.*object.covariance)))) * exp(-0.5 * (((x-object.mean)'/(object.covariance))*((x-object.mean))));
        end   
    end
end