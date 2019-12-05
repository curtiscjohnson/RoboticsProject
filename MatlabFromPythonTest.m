% This function is to test call from python. 
% See https://www.mathworks.com/help/matlab/matlab_external/call-user-script-and-function-from-python.html

% The python syntax to call this function is (after everything is installed, see repo Readme for more details):

    % import matlab.engine
    % eng = matlab.engine.start_matlab()
    % ans = eng.triarea(1.0,5.0)
    % ans should return 2.5

function a = MatlabFromPythonTest(b,h)
    a = 0.5*b.*h;
end