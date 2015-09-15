clear;

experiment_name = 'inc0_stiff1';
data_type = {'left_wrench', 'right_wrench'};%,'left_leg_state','right_leg_state', 'torso_state'};
data = struct;

for i = 1: length(data_type)
    datafile = strcat('data', filesep, experiment_name, filesep, data_type{i}, filesep, 'data.log');
    dataloaded = load(datafile);

    data.(data_type{i}).time = dataloaded(:,2) - dataloaded(1,2);
    data.(data_type{i}).data = dataloaded(:,3:end);
    
end

%parse state (different format)
data_type = {'left_leg_state','right_leg_state', 'torso_state'};
part_size = [6, 6, 3];

for i = 1: length(data_type)
    datafile = strcat('data', filesep, experiment_name, filesep, data_type{i}, filesep, 'data.log');
    dataloaded = readStateExt(part_size(i), datafile);
    dataloaded.time = dataloaded.time - dataloaded.time(1);
    
    data.(data_type{i}) = dataloaded; 
end


%temp until fix on data acqusition
%l/r_sole => world orientation
% Rdyn = iDynTree.Rotation.RotZ(pi) * iDynTree.Rotation.RotY(pi/2);
Rdyn = [0, 0, 1; 0, -1, 0; 1, 0, 0];
% R = blkdiag(Rdyn.toMatlab(), Rdyn.toMatlab());
R = blkdiag(Rdyn, Rdyn);

for i = 1: size(data.left_wrench.data,1)
    data.left_wrench.data(i,:) = (R * data.left_wrench.data(i,:)')';
end
for i = 1: size(data.right_wrench.data,1)
    data.right_wrench.data(i,:) = (R * data.right_wrench.data(i,:)')';
end

%compute CoP
%%TODO: check on real robot
footSize = [-0.05, 0.05;
            -0.05,  0.1]; %x (min-max); y (min-max)

CoP_l = [-data.left_wrench.data(:,5) ./ data.left_wrench.data(:,3), data.left_wrench.data(:,4) ./ data.left_wrench.data(:,3)];
CoP_r = [-data.right_wrench.data(:,5) ./ data.right_wrench.data(:,3), data.right_wrench.data(:,4) ./ data.right_wrench.data(:,3)];

figure
plot(data.left_wrench.time, CoP_l);
title('Left CoP');
legend('x','y');

figure
plot(data.right_wrench.time, CoP_r);
title('Right CoP');
legend('x','y');

