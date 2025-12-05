clc;
close all;
clear;


% Inputs;
personal_number = 041210;
global overshoot_max;
overshoot_max = 5; % Task 3.2 criteria

[J, umax] = lab3robot(personal_number);




% Constants from the lab instructions
% u - voltage
% theta_L - robot arm angle
% theta_m - motor axle angle
% I_a - motor current
% T - torque

L_m = 2;    % Induction
R_m = 21;   % Resistance
b = 1;      % Friction coefficient
J;          % Moment of inertia
K_T = 38;   % Material constant
K_m = 0.5;  % Material constant
n = 1/20;   % Gearing factor






%% 3
s = tf('s');

G_current = 1 / (s*L_m + R_m);
G_torque = K_T;
G_motor_velocity = 1 / (J*s + b);
G_feedback_voltage = -K_m;

G_motor_angle = 1 / s;
G_arm_angle = n;

G_open_voltage_loop = G_current * G_torque * G_motor_velocity;
G_closed_voltage_loop = G_open_voltage_loop / (1 - G_feedback_voltage * G_open_voltage_loop);

G = G_closed_voltage_loop * G_motor_angle * G_arm_angle;
G
lab3robot(G, personal_number);

%% 3.2 Proportional control
disp(" ");disp(" ");disp("3.2 Proportional control");
% Requirements
% Oversoot less than 5%
% Rise time as short as possible

% Use secant method to find F that meets the overshoot requirement
function ret = get_F_with_requirements(G, start_guess_1, start_guess_2)
    function o = get_overshoot_err(G, k)        
        Gc = feedback(k*G, 1);
        [y, tout] = step(Gc);
        info = stepinfo(y, tout);
        
        global overshoot_max;
        o = info.Overshoot - overshoot_max;
    end
    
    overshoot_err_prev = get_overshoot_err(G, start_guess_1);    
    k_prev = start_guess_1;
    k_current = start_guess_2;
    
    ks = [k_prev, k_current];

    for i = 1:10
        overshoot_err_current = get_overshoot_err(G, k_current);
        k_next = k_current - overshoot_err_current * (k_current - k_prev) / (overshoot_err_current - overshoot_err_prev);
        
        k_prev = k_current;
        k_current = k_next;
        overshoot_err_prev = overshoot_err_current;
        
        ks(end + 1) = k_current;
    end
    ret = ks;
end
function plot_step_with_info(G, F)
    Gc = feedback(F*G, 1);
    [y, tout] = step(Gc);
    info = stepinfo(y, tout);
    
    plot(tout, y, "DisplayName", sprintf('F: %.3f Rise time: %.10f s  Overshoot: %.10f %', F, info.RiseTime, info.Overshoot));
end

figure;
valid_regulators = get_F_with_requirements(G, 2, 10);
for F = valid_regulators;
    hold on; plot_step_with_info(G, F);
end

yline(1.05, "DisplayName", "5 % overshoot");
title("Step responses for regulators that satisfy the requirements");
legend();
disp("Assignment 2");
% disp(valid_regulators);
chosen_K = valid_regulators(end);
chosen_rise_time = stepinfo(feedback(valid_regulators(end)*G, 1)).RiseTime;
disp(sprintf("Chosen F: %.3f gives a rise time of %.10f s", chosen_K, chosen_rise_time);


FG = chosen_rise_time * G;
[Gain_margin_linear, Phase_margin, G_m_freq, crossover_freq] = margin(FG);
figure; margin(FG);
disp("Assignment 3");
disp(sprintf('Cross-over frequency: %.3f rad/s   Phase margin: %.3f deg', crossover_freq, Phase_margin));

disp("Assignment 4");
disp("1. The magnitude plot adds K to it. The phase plot doesn't change.");
disp("2. The speed of the system is limited by the maximum voltage u_max");
disp("3. The robot arm starts oscillating or just spins around at maximum rpm");
disp("4. At the same gain as the gain marigin of G will the system become unstable.")
disp(sprintf('   This occurs at K = %.3f   and makes the phase margin become 0 degrees', margin(G))); % Gets the gain margin of only G without any gain K




%% 3.3 Improved control
% Requirements
% Oversoot less than 5%
% The closed loop system should be four times faster than what was possible with proportional control.