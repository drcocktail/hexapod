function [vertices, faces] = chassisPrototype(robot)
%CHASSISPROTOTYPE Return local vertices/faces for the hexagonal chassis.

t = robot.baseAngles(:);
top = [robot.bodyRadius * cos(t), robot.bodyRadius * sin(t), ...
    ones(6, 1) * (robot.bodyHeight / 2)];
bottom = [robot.bodyRadius * cos(t), robot.bodyRadius * sin(t), ...
    -ones(6, 1) * (robot.bodyHeight / 2)];

vertices = [top; bottom];
faces = [
    1 2 3 4 5 6
    7 8 9 10 11 12
    1 2 8 7 NaN NaN
    2 3 9 8 NaN NaN
    3 4 10 9 NaN NaN
    4 5 11 10 NaN NaN
    5 6 12 11 NaN NaN
    6 1 7 12 NaN NaN
];
end
