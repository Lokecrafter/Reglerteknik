clc;
close all;
clear;


% Inputs;
personal_number = 041210;
global overshoot_max;
overshoot_max = 5; % Task 3.2 criteria

global umax;

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
disp(' ');disp(' ');disp('3.2 Proportional control');
% Requirements
% Oversoot less than 5%
% Rise time as short as possible

% Use secant method to find F that meets the overshoot requirement
function ret = get_F_prop_with_requirements(G, start_guess_1, start_guess_2)
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
function plot_step_with_info(G, F, additional_info, plot_u)
    Gc = feedback(F*G, 1);
    [y, tout] = step(Gc);
    info = stepinfo(y, tout);
    
    plot(tout, y, 'DisplayName', sprintf('%s Rise time: %.10f s  Overshoot: %.10f %', additional_info, info.RiseTime, info.Overshoot));
    [u, utout] = step(F / (1 + F * G));
    if plot_u
        global umax;
        plot(utout, u, 'DisplayName', sprintf('u(t). u(t)_{max}: %.2f V', max(u)));
        yline(umax, 'r--', 'DisplayName', sprintf('u_{max} = %.2f V', umax));
    end
end
figure;
valid_regulators = get_F_prop_with_requirements(G, 2, 10);
for F_prop = valid_regulators;
    hold on; plot_step_with_info(G, F_prop, sprintf('F: %.3f', F_prop), false);
end

yline(1.05, 'DisplayName', '5 % overshoot');
title('Step responses for regulators that satisfy the requirements');
legend();
disp('Assignment 2');
% disp(valid_regulators);
chosen_K_prop = valid_regulators(end);
chosen_rise_time = stepinfo(feedback(valid_regulators(end)*G, 1)).RiseTime;
disp(sprintf('Chosen F: %.3f gives a rise time of %.10f s', chosen_K_prop, chosen_rise_time));


[Gain_margin_linear, current_phase_margin, G_m_freq, crossover_freq] = margin(chosen_K_prop * G);
figure; margin(chosen_K_prop * G);
disp('Assignment 3');
disp(sprintf('Cross-over frequency: %.3f rad/s   Phase margin: %.3f deg', crossover_freq, current_phase_margin));

disp('Assignment 4');
disp('1. The magnitude plot adds K to it. The phase plot doesnt change.');
disp('2. The speed of the system is limited by the maximum voltage u_max');
disp('3. The robot arm starts oscillating or just spins around at maximum rpm');
disp('4. At the same gain as the gain marigin of G will the system become unstable.')
disp(sprintf('   This occurs at K = %.3f   and makes the phase margin become 0 degrees', margin(G))); % Gets the gain margin of only G without any gain K




%% 3.3 Improved control
% Requirements
% 1. Oversoot less than 5%.
% 2. The closed loop system should be four times faster than what was possible with proportional control.
% 3. The control signal should satisfy |u(t)|< umax for all t when the refer- ence signal is a unit step, 
%    where umax is given by the function lab3robot (see Section 3.1)
% 4. The stationary control error should be smaller than 0.05 when the reference signal is a unit ramp (hint: a ramp is an integrated step)
% 5. We do not want to amplify measurement disturbances too much, i.e.
%    the high-frequency amplification of the controller should not be made unnecessarily large.
% 6. In order to decrease possible problems with nonlinearities we do not want the controller gain for low frequencies to be too large.


disp('Assignment 5');
disp('1. It is the cross-over frequency. It should be multiplied by 4 to make it 4 times faster than the P-controller.');
disp('2. The phase margin should be kept the same to make sure the overshoot is under 5%');






disp('Assignment 6');
[~, P_phase_margin, ~, P_crossover_freq] = margin(chosen_K_prop * G);
[~, G_phase_margin, ~, G_crossover_freq] = margin(G);
desired_phase_margin = P_phase_margin;
% desired_phase_margin = 70;
desired_crossover_freq = 4 * P_crossover_freq;


phase_increase = desired_phase_margin - (180 + rad2deg(angle(freqresp(G, desired_crossover_freq))));
disp(sprintf('Desired phase margin: %.3f deg', desired_phase_margin));
disp(sprintf('Desired crossover frequency: %.3f rad/s', desired_crossover_freq));
disp(sprintf('Phase increase at desired cross-over frequency: %.3f deg', phase_increase)); %#ok<*DSPSP>

Y = 1;


B = 0.15;       % TODO: Edit Beta to be exactly as formula 5.4 in the book.
B = (1 - sind(phase_increase)) / (1 + sind(phase_increase));

% T_I = 10/desired_crossover_freq;
T_I = 0;
T_D = 1/(desired_crossover_freq * sqrt(B));
K = 1;

F = K * ((T_D*s + 1)/(B*T_D*s + 1)) * ((T_I*s + 1)/(T_I*s + Y));


K = 1 / abs(freqresp(F*G, desired_crossover_freq));


F = K * ((T_D*s + 1)/(B*T_D*s + 1)) * ((T_I*s + 1)/(T_I*s + Y));
disp(sprintf('K: %.3f   T_I: %.3f   T_D: %.3f   Beta: %.3f   Gamma: %.3f', K, T_I, T_D, B, Y));
% T_I = 1 / (tan(-diff_phase_margin) * desired_crossover_freq);


% [~, new_phase_margin] = margin(F*G);
% new_phase_margin


figure;
hold on;
margin(G);
margin(feedback(F*G, 1));
margin(F*G);
xline(desired_crossover_freq)
legend('G', 'G-closed', 'FG, G-open');

figure;
hold on;
% step(feedback(F*G, 1));
% step(F);
plot_step_with_info(G, F, '', true);
legend();






e1_max = 0.05; % Max stationary error for unit ramp

K_v_desired = 1/e1_max;
K_v_lead = dcgain(s*F*G); % 
% K_v_lag = 1/Y; % Explanation
Y = K_v_lead / K_v_desired;
T_I = 10 / desired_crossover_freq;


% TODO: Compensate for the phase lost by the lag component by adjusting T_D and B




disp('Assignment 7');
disp('')
S_prop = 1 / (1 + chosen_K_prop * G);
S = 1 / (1 + F*G);


frequencies = logspace(0, 3, 1000);


figure;
hold on;
bodemag(S_prop, S);
bodemag(S_prop, S);
yline(1);

disp('We need to evaluate when the function S(jw) is less than 1 (0 dB) to see when disturbances are damped.');

title('Sensitivity functions comparison');
legend('P-control', 'Improved control');



delta_G1 = (s + 10)/40;
delta_G2 = (s + 10)/(4*s + 0.04);

figure;
hold on;
bodemag(1 - S);
bodemag(1/delta_G1);
bodemag(1/delta_G2);
legend('T(iω)', '1/delta_G1 ', '1/delta_G2 ');




A = [
    0, n, 0;
    0, -b/J, K_T/J;
    0, -K_m/L_m, -R_m/L_m
];
B = [
    0; 
    0; 
    1/L_m
];
C = [1, 0, 0];

controllable_rank = rank(ctrb(A, B));
obsv_rank = rank(obsv(A, C));

disp("Assignment 11. Yes both ranks are 3 which means they are controllable and observable.");





















% 1. Lead-Lag Kompensator
%     Insamlad data: Lead-lag är en dynamisk regulator som opererar på felet e(t)=r(t)−y(t). Den behöver bara en sensor (för utsignalen y).
%     Fysisk Implementering: Den kan implementeras antingen som en analog krets (med OP-förstärkare, resistorer och kondensatorer), eller som en digital algoritm (i en mikrokontroller) som beräknar u(t) baserat på e(t) och e(t−1) (tidigare värden).
%     Enkelhet: Den är enkel att implementera fysiskt (kräver få sensorer).
% 2. Tillståndsåterkoppling (State Feedback Controller)
%     Insamlad data: Kontrollagen u=−Lx+l0​r kräver att alla element i tillståndsvektorn x är kända. Tillståndsvektorn kan inkludera position, hastighet, ström, etc.
%     Fysisk Implementering: Detta kräver flera olika sensorer, en för varje tillståndsvariabel i x. Till exempel: för en motor krävs troligen en lägesgivare OCH en hastighetsgivare.
%     Utmaningen (Estimering): Ofta är inte alla tillståndsvariabler fysiskt mätbara eller det är för dyrt med sensorer. I sådana fall måste man komplettera kontrollagen med en Tillståndsobservatör (t.ex. Luenberger-observatör eller Kalmanfilter) som uppskattar de omätbara tillstånden baserat på u och de mätbara y.
% Sammanfattning: Implementeringen av State Feedback är komplexare och kräver antingen fler sensorer eller en tillståndsobservatör utöver själva regulatoralgoritmen.