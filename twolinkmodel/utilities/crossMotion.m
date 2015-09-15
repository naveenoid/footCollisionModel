function vx = crossMotion(twistInMatlabForm)
    twistInMatlabForm_lin = twistInMatlabForm(1:3);
    twistInMatlabForm_angular = twistInMatlabForm(4:end);
    vx = [skew(twistInMatlabForm_angular),  skew(twistInMatlabForm_lin);
          zeros(3,3),                       skew(twistInMatlabForm_angular)];
end