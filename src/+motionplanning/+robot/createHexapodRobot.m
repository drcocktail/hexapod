function robot = createHexapodRobot(options)
%CREATEHEXAPODROBOT Build a normalized robot model from config values.

robot.bodyRadius = options.bodyRadius;
robot.bodyHeight = options.bodyHeight;
robot.clearance = options.clearance;
robot.L_coxa = options.L_coxa;
robot.L_femur = options.L_femur;
robot.L_tibia = options.L_tibia;
robot.maxStepZ = options.maxStepZ;

robot.baseAngles = linspace(0, 2 * pi, 7);
robot.baseAngles(end) = [];

robot.maxReach = robot.L_coxa + robot.L_femur + robot.L_tibia;
robot.minFemurTibiaReach = abs(robot.L_femur - robot.L_tibia);
robot.nominalFootRadius = robot.bodyRadius + robot.L_coxa + robot.L_femur * 0.7;
robot.visualFootRadius = robot.bodyRadius + robot.L_coxa + robot.L_femur * 0.6;
end
