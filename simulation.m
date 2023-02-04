close all
clc
clear all


%setting the parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k = 1e-6;
theta = 0.25;
tau = 1.8;



K_c_PI1= 0.15/k+(0.35-(theta*tau)/(theta+tau)^2)*(tau/(k*theta));
tau_i_PI1 = 0.35*theta+(13*theta*tau^2)/(tau^2+12*theta*tau+7*theta^2);


K_c_PI2= 1/k*(tau/(tau/3+theta));
tau_i_PI2 = tau;



K_c_PID1= 1/k*(0.2+0.45*(tau/theta));
tau_i_PID1 = theta*(0.4*theta+0.8*tau)/(theta+0.1*tau);
tau_d_PID1 = (0.5*theta*tau)/(0.3*theta+tau);

K_c_PID2= 1/k*(2*tau+theta)/(2*tau/3+theta);
tau_i_PID2= tau+theta/2;
tau_d_PID2 = (tau*theta)/(2*tau+theta);




% starting the Simulink model and extracting the step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim('simulationModel.slx')


 
 time=simoutPI1.Time;
 PI_amigo=simoutPI1.Data;
 PI_imc=simoutPI2.Data;
 PID_amigo=simoutPID1.Data;
 PID_imc=simoutPID2.Data;

 

% plotting the responses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot(time,PI_amigo,'DisplayName', 'PI-AMIGO')
grid on
hold on
plot(time,PI_imc,'g','DisplayName', 'PI-IMC')
plot(time,PID_amigo,'DisplayName', 'PID-AMIGO')
plot(time,PID_imc,'r','DisplayName', 'PID-IMC')
legend('show', 'Location', 'southeast');




% calculating the rise time of the responses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
finalValue=1;
dt=2e-3;
crossing_indexPI1 = find(PI_amigo >= finalValue, 1)
crossing_indexPI2 = find(PI_imc >= finalValue, 1);
crossing_indexPID1 = find(PID_amigo >= finalValue, 1);
crossing_indexPID2 = find(PID_imc >= finalValue, 1);

rise_time_PI_amigo = (crossing_indexPI1 - 1) * dt -0.25
rise_time_PI_imc = (crossing_indexPI2 - 1) * dt -0.25
rise_time_PID_amigo = (crossing_indexPID1 - 1) * dt -0.25
rise_time_PID_imc = (crossing_indexPID2 - 1) * dt -0.25



% calculating the settling time of the responses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Defining the tolerance (2%)
tolerance = 0.02 * finalValue;

PI_amigo_reversed=PI_amigo(end:-1:1);
PI_imc_reversed=PI_imc(end:-1:1);
PID_amigo_reversed=PID_amigo(end:-1:1);
PID_imc_reversed=PID_imc(end:-1:1);

% Find the first time where the response is within the tolerance
idx_PI_amigo =size(PI_amigo,1) - find(abs(PI_amigo_reversed - finalValue) > tolerance, 1);
idx_PI_imc = size(PI_imc,1) - find(abs(PI_imc_reversed - finalValue) > tolerance, 1);
idx_PID_amigo = size(PID_amigo,1) - find(abs(PID_amigo_reversed - finalValue) > tolerance, 1);
idx_PID_imc = size(PID_imc,1) - find(abs(PID_imc_reversed - finalValue) > tolerance, 1);


% Calculate the settling time
settling_time_PI_amigo = idx_PI_amigo * dt-0.25
settling_time_PI_imc = idx_PI_imc * dt-0.25
settling_time_PID_amigo = idx_PID_amigo * dt-0.25
settling_time_PID_imc = idx_PID_imc * dt-0.25



err_PI_AMIGO= (finalValue - PI_amigo).^2;
err_PI_IMC= (finalValue - PI_imc).^2;
err_PID_AMIGO= (finalValue - PID_amigo).^2;
err_PID_IMC= (finalValue - PID_imc).^2;

ISE_PI_AMIGO = trapz(time,err_PI_AMIGO)
ISE_PI_IMC = trapz(time,err_PI_IMC)
ISE_PID_AMIGO = trapz(time,err_PID_AMIGO)
ISE_PID_IMC = trapz(time,err_PID_IMC)

