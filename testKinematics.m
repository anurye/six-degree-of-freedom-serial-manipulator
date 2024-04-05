% Main function
function tests = testKinematics
tests = functiontests(localfunctions);
end


% File fixture
function setupOnce(testCase)
% Add files to MATLAB PATH 
initialize

% Create a serial link object - representing the robot arm
testCase.TestData.robot = actual;
% Define tolerance
testCase.TestData.tol = 1e-6;
end


% Test functions
function testDirectKinematics(testCase)
% Define number of tests
numTests = 10;

% Get the robot arm object
robot = testCase.TestData.robot;

% perform test
for i = 1:numTests
    q = rand(size(robot.links, 2), 1) .* (robot.qlim(:,2) - ...
        robot.qlim(:,1)) + robot.qlim(:,1);
    Tee_computed = direct_kine(dh, q);
    Tee_toolbox = double(robot.fkine(q));

    % Check if the computed forward kinematics results are almost equal
    verifyTrue(testCase, isequal(abs(Tee_computed - Tee_toolbox) < ...
        testCase.TestData.tol, ones(size(Tee_computed))), ...
        sprintf('Direct kinematics test %d failed.', i));
end
end


function testInverseKinematics(testCase)
% Define number of test
numTests = 10;

% Get the robot arm object
robot = testCase.TestData.robot;

% perform test
for i = 1:numTests
    q = rand(size(robot.links, 2), 1) .* (robot.qlim(:,2) - ...
        robot.qlim(:,1)) + robot.qlim(:,1);
    Tee = direct_kine(dh, q);
    qc = inv_kine(Tee, dh, q);
    TeeComputed = direct_kine(dh, qc);

    % Check if the computed inverse kinematics results are within tolerance
    verifyTrue(testCase, isequal(abs(Tee - TeeComputed) < ...
        testCase.TestData.tol, ones(size(TeeComputed))), ...
        sprintf('Inverse kinematics test %d failed.', i));
end
end
