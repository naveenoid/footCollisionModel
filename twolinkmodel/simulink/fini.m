filename = strcat('data',filesep, 'incl_');

if (plane_incl < 0)
    filename = strcat(filename, 'm');
end

filename = strcat(filename, num2str(floor(abs(plane_incl * 180 / pi))));

filename = strcat(filename, '_k');
filename = strcat(filename, num2str(floor(k)));
filename = strcat(filename, '_d');
filename = strcat(filename, num2str(floor(d)));
filename = strcat(filename, '.mat');

save(filename);