
function vx_star = crossStar(twistInMatlabForm)
    twistInMatlabForm_lin = twistInMatlabForm(1:3);
    twistInMatlabForm_angular = twistInMatlabForm(4:end);
    
    vx_star = [skew(twistInMatlabForm_angular), zeros(3,3);
                skew(twistInMatlabForm_lin), skew(twistInMatlabForm_angular)];
end