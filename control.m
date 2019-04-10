function [Id,Iq,I_alpha, I_beta] = control(Ia,Ib,Ic,HALL_A,HALL_B,HALL_C,command_Q,command_D)

% most of the global variables that are used throughout the program are
% created and listed here
global counter     % Counter for duty cycle
global delta       % Time since previous HALL sensor
global step        % Sample period
global delta_A     % Value of HALL A one step prior
global delta_B     % Value of HALL B one step prior
global delta_C     % Value of HALL C one step prior
global w           % Speed in radians per second
global theta       % Angle in radians
global timer       % Time since start of program
global integral_Q  % Value of descrete integral for Q current
global integral_D  % Value of descrete integral for D current

A = 0; % Sets initial value for phase A voltage
B = 0; % Sets initial value for phase B voltage
C = 0; % Sets initial value for phase C voltage

% Using current readings from the current sensors we calculate the alpha
% and beta currents of the motor
[I_alpha,I_beta] = clarke(Ia,Ib,Ic);

% Continuously tracking the amount of time since the previous Hall effecto
% sensor reading to keep track of speed and position of motor
delta = delta + step; 

% Continuously keeps track of the time passed since the beginning of the
% program to change when we switch from trapezpoidal to FOC
timer = timer + step; 

if timer >= 0.0001 % Problem in function fix later
    read_pos(HALL_A,HALL_B,HALL_C); % Keeps track of the position of the motor
end


delta_A = HALL_A; % Saves previous state of Hall effect sensor A
delta_B = HALL_B; % Saves previous state of Hall effect sensor B
delta_C = HALL_C; % Saves previous state of Hall effect sensor C

theta = theta + w*step; % Integrating to keep track of angle

[Id,Iq] = park(I_alpha,I_beta,4*theta); % Calculate direct and quadrature currents

counter = counter + 1; % Keeping track of duty cycle

% Runs trapezoidal control for .1 seconds then flips to FOC
if timer < .1
    trapezoid(command_Q,HALL_A,HALL_B,HALL_C);

% Flipping Field Oriented Control
else
    % PI control for quadrature current
    error_Q = ((command_Q/100)*30) - Iq;
    integral_Q = integral_Q + (error_Q*step);
    u_Q = 1000*integral_Q + 1000*error_Q;
    
    % PI control for direct current
    error_D = (command_D - Id);
    integral_D = integral_D + (error_D*step);
    u_D = 1000*integral_D + 1000*error_D;
    
    % Calculate inverse parke transformation for PI control output
    [I_alpha,I_beta] = inverse_parke(u_D,u_Q,4*theta);
    
    % Calculate inverse clarke transformation for PI control output
    [A,B,C] = inverse_clarke(I_alpha,I_beta);
    % Raises or lowers voltages based on coontrol signal
    regulate(A,B,C,Ia,Ib,Ic); 
end
    
end

% regulate is a function that comapares the values of the 
% voltages on the line and tries to raise them if they are below
% the value that the PI controller is asking for
% or lowers them if the actual value is less than what the PI controller
% asks for
function regulate(command_A,command_B,command_C,A,B,C)
    global q1
    global q2
    global q3
    global q4
    global q5
    global q6
    
    if A < command_A
        q1 = 1;
        q2 = 0;
    else
        q1 = 0;
        q2 = 1;
    end
    if B < command_B
        q3 = 1;
        q4 = 0;
    else
        q3 = 0;
        q4 = 1;
    end
    if C < command_C
        q5 = 1;
        q6 = 0;
    else
        q5 = 0;
        q6 = 1;
    end
end


% This function takes in alpha/beta voltages/currents and the angle of the
% motor computes the direct/quadrature voltages/currents
function [d,q] = park(alpha,beta,theta)
    d = cos(theta)*alpha + sin(theta)*beta;
    q = cos(theta)*beta - sin(theta)*alpha;
end

% This function takes in a/b/c voltages/currents
% and computes the alpha/beta voltages/currents
function [alpha,beta] = clarke(a,b,c)
    alpha = a;
    beta = (1/(sqrt(3))*(b-c));
end

% This function takes in direct/gaudrature voltages/currents
% and the angle of the motor and performs the reverse parke transformation
% giving alpha/beta voltages/currents
function [alpha,beta] = inverse_parke(d,q,theta)
    alpha = d*cos(theta) - q*sin(theta);
    beta = q*cos(theta) + d*sin(theta);
end


% This function takes in alpha/beta voltages/currents and 
% performs the reverse clarke transformation and returns a/b/c
% voltages/currents
function [a,b,c] = inverse_clarke(alpha,beta)
    a = alpha;
    b = (1/2)*((-alpha) + (sqrt(3)*beta));
    c = (1/2)*((-alpha) - (sqrt(3)*beta));
end

% This function takes in a counter which helps define a duty cycle
% along with a command 0 to 100 percent and the position of the motor
% via the hall affect sensors and produces a trapazoid output signal
% to the 3 half bridges
function trapezoid(command,HALL_A,HALL_B,HALL_C)
    global q1
    global q2
    global q3
    global q4
    global q5
    global q6
    global counter
    if counter <= command
        if HALL_A && ~HALL_B && HALL_C
            q1 = 1;
            q2 = 0;
            q3 = 0;
            q4 = 1;
            q5 = 1;
            q6 = 0;

        elseif HALL_A && ~HALL_B && ~HALL_C
            q1 = 1;
            q2 = 0;
            q3 = 0;
            q4 = 1;
            q5 = 0;
            q6 = 1;

        elseif HALL_A && HALL_B && ~HALL_C
            q1 = 1;
            q2 = 0;
            q3 = 1;
            q4 = 0;
            q5 = 0;
            q6 = 1;

        elseif ~HALL_A && HALL_B && ~HALL_C
            q1 = 0;
            q2 = 1;
            q3 = 1;
            q4 = 0;
            q5 = 0;
            q6 = 1;

        elseif ~HALL_A && HALL_B && HALL_C
            q1 = 0;
            q2 = 1;
            q3 = 1;
            q4 = 0;
            q5 = 1;
            q6 = 0; 

        elseif ~HALL_A && ~HALL_B && HALL_C
            q1 = 0;
            q2 = 1;
            q3 = 0;
            q4 = 1;
            q5 = 1;
            q6 = 0;
        end
    else
        q1 = 0;
        q2 = 0;
        q3 = 0;
        q4 = 0;
        q5 = 0;
        q6 = 0;
    end
    if counter >= 100
        counter = 0;
    end
end

% This function takes the three Hall effect sensor outputs
% along with the amount of time that has passed and integrates to keep
% track of the position of the motor for use in the parke transformations
% and inverse parke transformation
function read_pos(HALL_A,HALL_B,HALL_C)
    global prev_HALL
    global delta_A
    global delta_B
    global delta_C
    global delta 
    global w
    
    if((HALL_A-delta_A ~= 0 || HALL_B-delta_B ~= 0 || HALL_C-delta_C ~= 0))

        if HALL_A-delta_A == -1
            if prev_HALL == 3
                w = -1*((2*pi)/24)/delta;

            elseif prev_HALL == 2
                w = ((2*pi)/24)/delta;
            else
                w = 0;

            end
            prev_HALL = 1;

        elseif HALL_A-delta_A == 1
            if prev_HALL == 3
                w = -1*((2*pi)/24)/delta;

            elseif prev_HALL == 2
                w = ((2*pi)/24)/delta;
            else
                w = 0;

            end
            prev_HALL = 1;

        elseif HALL_B-delta_B == -1
            if prev_HALL == 1
                w = -1*((2*pi)/24)/delta;

            elseif prev_HALL == 3
                w = ((2*pi)/24)/delta;

            else
                w = 0;

            end
            prev_HALL = 2;

        elseif HALL_B-delta_B == 1
            if prev_HALL == 1
                w = -1*((2*pi)/24)/delta;

            elseif prev_HALL == 3
                w = ((2*pi)/24)/delta;

            else
                w = 0;

            end
            prev_HALL = 2;

        elseif HALL_C-delta_C == -1
            if prev_HALL == 2
                w = -1*((2*pi)/24)/delta;

            elseif prev_HALL == 1
                w = ((2*pi)/24)/delta;

            else
                w = 0;

            end
            prev_HALL = 3;

        elseif HALL_C-delta_C == 1
            if prev_HALL == 1
                w = ((2*pi)/24)/delta;

            elseif prev_HALL == 2
                w = -1*((2*pi)/24)/delta;

            else
                w = 0;

            end
            prev_HALL = 3;

        end

        delta = 0;
    end
end


% This function is incomplete
% it will be used as a PI controller in software
function [U] = PID(Kp,Ki,Kd)
    error = target - value;
    I = i + error*step;
end