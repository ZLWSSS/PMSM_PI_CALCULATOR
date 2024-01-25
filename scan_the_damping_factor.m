clear, close all

LPF_const =  1.989436788648692e-05;
phase_resistor = 0.2447;
phase_Ls = 0.00038275;
EMF_Constant = 0.00708491;
NPP = 21;
K_b = phase_resistor/phase_Ls;
inertia = 373e-6;
sampling_period_cur = 0.000025;
sampling_period = 1e-7;

%%delta_vector
delta_vector = [1.5, 2.5, 4, 6, 10, 20, 35, 70];
super_refine_delta_vector=[25,26,27,28,29,30,31];
refine_delta_vector = [10,20,30,40,50,60,70];

vector_use = delta_vector;
choose_done = 1;
damp_factor = 29;
K_a = 8;

K = 3*NPP*EMF_Constant/(2*inertia);
H=tf([1],[1]);
figure(1);

for i = 1 : length(vector_use)
  K_c = 1/(K*LPF_const*vector_use(i));
  K_d = 1 / (vector_use(i)* vector_use(i)* LPF_const);
  sys=tf([K*K_c K*K_c*K_d],[LPF_const 1 0 0]);
  sys_dis = c2d(sys, sampling_period, 'zoh');
  margin(sys_dis);
  legend_str{i}=['Delta=',num2str(vector_use(i))];
  hold on;
end
legend(legend_str);

figure(2);
for i = 1 : length(vector_use)
    K_c = 1/(K*LPF_const*vector_use(i));
    K_d = 1 / (vector_use(i)* vector_use(i)* LPF_const);
    sys=tf([K*K_c K*K_c*K_d],[LPF_const 1 0 0]);
    T= feedback(sys,H);
    sys_dis = c2d(T, sampling_period, 'zoh');
    margin(sys_dis);
    hold on;
end
legend(legend_str);


figure(3)
for i = 1 : length(vector_use)
    K_c = 1/(K*LPF_const*vector_use(i));
    K_d = 1 / (vector_use(i)* vector_use(i)* LPF_const);
    sys=tf([K*K_c K*K_c*K_d],[LPF_const 1 0 0]);
    T= feedback(sys,H);
    sys_dis = c2d(T, sampling_period, 'zoh');
    stepplot(sys_dis);
    hold on;
end
legend(legend_str);

if(choose_done == 1)
    Ka_ub = pi*phase_Ls/(5*sampling_period_cur);
    Ka_lb = pi*phase_Ls/(damp_factor*LPF_const);
    figure(4);
    K_c = 1/(K*LPF_const*damp_factor);
    K_d = 1 / (damp_factor* damp_factor* LPF_const);
    sys_vel=tf([K*K_c K*K_c*K_d],[(phase_Ls/K_a)*LPF_const (LPF_const+(phase_Ls/K_a)) 1 0 0]);
    sys_cur = tf([1],[(phase_Ls/K_a) 1]);
    sys_vel_cl = feedback(sys_vel, H);
    sys_vel_cl_dis = c2d(sys_vel_cl, sampling_period, 'zoh');
    subplot(1,4,1);
    margin(sys_vel);
    title('Vel OpenLoop');
    subplot(1,4,2);
    margin(sys_cur);
    title('Cur CloseLoop')
    subplot(1,4,3);
    margin(sys_vel_cl_dis);
    title('Vel CloseLoop')
    subplot(1,4,4);
    stepplot(sys_vel_cl_dis);
    title('Vel Step');

    figure(4);
end






