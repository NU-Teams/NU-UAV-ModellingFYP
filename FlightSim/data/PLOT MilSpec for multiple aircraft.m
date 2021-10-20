%% Setup

clear
close all
addpath("Config_1")
addpath("Config_2")
addpath("Config_3")
addpath("Config_4")
addpath("PC9_100kts")
addpath("PC9_300kts")


%% Extract data

la1 = load('Config_1\load_alpha.mat');
la2 = load('Config_2\load_alpha.mat');
la3 = load('Config_3\load_alpha.mat');
la4 = load('Config_4\load_alpha.mat');
laPC100 = load('PC9_100kts\load_alpha.mat');
laPC300 = load('PC9_300kts\load_alpha.mat');
load_alpha_approach = [la1.load_alpha la2.load_alpha laPC100.load_alpha];
load_alpha_cruise = [la3.load_alpha la4.load_alpha laPC300.load_alpha];

zlo1 = load('Config_1\zeta_long.mat');
zlo2 = load('Config_2\zeta_long.mat');
zlo3 = load('Config_3\zeta_long.mat');
zlo4 = load('Config_4\zeta_long.mat');
zloPC100 = load('PC9_100kts\zeta_long.mat');
zloPC300 = load('PC9_300kts\zeta_long.mat');
zeta_long_approach = [zlo1.zeta_long zlo2.zeta_long zloPC100.zeta_long];
zeta_long_cruise = [zlo3.zeta_long zlo4.zeta_long zloPC300.zeta_long];

Wnlo1 = load('Config_1\Wn_long.mat');
Wnlo2 = load('Config_2\Wn_long.mat');
Wnlo3 = load('Config_3\Wn_long.mat');
Wnlo4 = load('Config_4\Wn_long.mat');
WnloPC100 = load('PC9_100kts\Wn_long.mat');
WnloPC300 = load('PC9_300kts\Wn_long.mat');
Wn_long_approach = [Wnlo1.Wn_long Wnlo2.Wn_long  WnloPC100.Wn_long];
Wn_long_cruise = [Wnlo3.Wn_long Wnlo4.Wn_long WnloPC300.Wn_long];

zla1 = load('Config_1\zeta_lat.mat');
zla2 = load('Config_2\zeta_lat.mat');
zla3 = load('Config_3\zeta_lat.mat');
zla4 = load('Config_4\zeta_lat.mat');
zlaPC100 = load('PC9_100kts\zeta_lat.mat');
zlaPC300 = load('PC9_300kts\zeta_lat.mat');
zeta_lat_approach = [zla1.zeta_lat zla2.zeta_lat zlaPC100.zeta_lat];
zeta_lat_cruise = [zla3.zeta_lat zla4.zeta_lat zlaPC300.zeta_lat];

Wnla1 = load('Config_1\Wn_lat.mat');
Wnla2 = load('Config_2\Wn_lat.mat');
Wnla3 = load('Config_3\Wn_lat.mat');
Wnla4 = load('Config_4\Wn_lat.mat');
WnlaPC100 = load('PC9_100kts\Wn_lat.mat');
WnlaPC300 = load('PC9_300kts\Wn_lat.mat');
Wn_lat_approach = [Wnla1.Wn_lat Wnla2.Wn_lat  WnlaPC100.Wn_lat];
Wn_lat_cruise = [Wnla3.Wn_lat Wnla4.Wn_lat WnlaPC300.Wn_lat];


%% Short Period Response (rows 1 and 2) <rows have equal values>

zeta_SP_approach = zeta_long_approach(1,:);
zeta_SP_cruise = zeta_long_cruise(1,:);
Wn_SP_approach = Wn_long_approach(1,:);
Wn_SP_cruise = Wn_long_cruise(1,:);


%% Plot Short Period

Figures(1) = figure('Name','Milspec1');
subplot(2,1,1)
    semilogx(zeta_SP_approach(3), Wn_SP_approach(3), 'xr','LineWidth', 1.5')
    title('Short Period Mode: Approach Condition (100 kts)')
    xlabel(' Damping Ratio \zeta SP')
    ylabel(' Nat. Frequency \omega_n SP')
    hold on
    semilogx(zeta_SP_approach(1), Wn_SP_approach(1), 'og','LineWidth', 1.5')
    semilogx(zeta_SP_approach(2), Wn_SP_approach(2), 'ob','LineWidth', 1.5')
    legend('PC-9 Nominal x_c_g', 'PC-21 without Trainee', 'PC-21 with Trainee','Location','northwest')
    grid on
    xticks([0.1:0.1:1])
    xlim([0.1 1])
    ylim([0 5])
    hold off
    
subplot(2,1,2)
    semilogx(zeta_SP_cruise(3), Wn_SP_cruise(3), 'xr','LineWidth', 1.5')
    title('Short Period Mode: Cruise Condition (300 kts)')
    xlabel(' Damping Ratio \zeta SP')
    ylabel(' Nat. Frequency \omega_n SP')
    hold on
    semilogx(zeta_SP_cruise(1), Wn_SP_cruise(1), 'og','LineWidth', 1.5')
    semilogx(zeta_SP_cruise(2), Wn_SP_cruise(2), 'ob','LineWidth', 1.5')
    legend('PC-9 Nominal x_c_g', 'PC-21 without Trainee', 'PC-21 with Trainee','Location','northwest')
    grid on
    xticks([0.1:0.1:1])
    xlim([0.1 1])
    ylim([5 10])
    hold off

Figure(2) = figure('Name','Milspec2');
subplot(2,1,1)
    loglog(load_alpha_approach(3), Wn_SP_approach(3), 'xr','LineWidth', 1.5')
    title('Short Period Mode Approach Condition (100 kts)')
    xlabel(' n/\alpha [g/rad]')
    ylabel(' Nat. Frequency \omega_n SP')
    hold on
    loglog(load_alpha_approach(1), Wn_SP_approach(1), 'og','LineWidth', 1.5')
    loglog(load_alpha_approach(2), Wn_SP_approach(2), 'ob','LineWidth', 1.5')
    loglog(logspace(0.699,2), logspace(-0.046,0.602), 'k', 'LineWIdth', 1.5);
    loglog(logspace(0.176, 0.699), logspace(-0.046,-0.046), 'k', 'LineWIdth', 1.5);
    loglog(logspace(0.176, 0.176), logspace(-0.046,0.38), 'k', 'LineWIdth', 1.5);
    loglog(logspace(log10(1.5),2), logspace(log10(2.413),1.301), 'k', 'LineWIdth', 1.5);
    legend('PC-9 Nominal x_c_g', 'PC-21 without Trainee', 'PC-21 with Trainee','Lvl. 1 Envelope', 'Location','southeast')
    grid on
    ylim([0.8 20])
    %xlim([1 10])
    hold off
subplot(2,1,2)
    loglog(load_alpha_cruise(3), Wn_SP_cruise(3), 'xr','LineWidth', 1.5')
    title('Short Period Mode: Cruise Condition (300 kts)')
    xlabel(' n/\alpha [g/rad]')
    ylabel(' Nat. Frequency \omega_n SP')
    hold on
    loglog(load_alpha_cruise(1), Wn_SP_cruise(1), 'og','LineWidth', 1.5')
    loglog(load_alpha_cruise(2), Wn_SP_cruise(2), 'ob','LineWidth', 1.5')
    loglog(logspace(0,2), logspace(-0.523,0.447), 'k', 'LineWIdth', 1.5);
    loglog(logspace(0,2), logspace(0.301,1.301), 'k', 'LineWIdth', 1.5);
    legend('PC-9 Nominal x_c_g', 'PC-21 without Trainee', 'PC-21 with Trainee', 'Lvl. 1 Envelope','Location','southeast')
    %xlim([10 100])
    ylim([0.8 20])
    grid on
    hold off
    
%% Dutch Roll

zeta_dutch_approach = zeta_lat_approach(4,:);
zeta_dutch_cruise = zeta_lat_cruise(4,:);
% the dutch role is underdamped
Wn_dutch_approach = Wn_lat_approach(4,:);
Wn_dutch_cruise = Wn_lat_cruise(4,:);


zeta_dutch_min = 0.08;
Wn_dutch_min_approach = 0.4;
Wnzeta_dutch_min = 0.15; 
Wnzeta_domain = 0.05:0.01:0.5;
Wnzeta_domain(Wnzeta_domain<zeta_dutch_min) = zeta_dutch_min;
Wnzeta_range = 0.15./Wnzeta_domain;

Wnzeta_range_approach = Wnzeta_range;
Wnzeta_range_approach(Wnzeta_range<Wn_dutch_min_approach) = Wn_dutch_min_approach;

Wn_dutch_min_cruise = 1.0;
Wnzeta_range_cruise = Wnzeta_range;
Wnzeta_range_cruise(Wnzeta_range<Wn_dutch_min_cruise) = Wn_dutch_min_cruise;

Figure(3) = figure('Name','Milspec3');
subplot(2,1,1)
    title('Dutch Role Mode: Approach Condition (100 kts)')
    xlabel(' Damping Ratio \zeta DR')
    ylabel(' Nat. Frequency \omega_n DR')
    hold on
    plot(zeta_dutch_approach(3), Wn_dutch_approach(3), 'xr','LineWidth', 1.5)
    plot(zeta_dutch_approach(1), Wn_dutch_approach(1), 'og','LineWidth', 1.5)
    plot(zeta_dutch_approach(2), Wn_dutch_approach(2), 'ob','LineWidth', 1.5)
    
    plot(Wnzeta_domain, Wnzeta_range_approach, 'k','LineWidth', 1.5)
    plot(zeta_dutch_min*[1 1], [Wnzeta_range_approach(1) 3], 'k', 'LineWidth', 1.5)
    legend('PC-9 Nominal x_c_g', 'PC-21 without Trainee', 'PC-21 with Trainee','lvl. 1 min Requirement')
    grid on
    hold off
subplot(2,1,2)
    title('Dutch Role Mode: Cruise Condition (300 kts)')
    xlabel(' Damping Ratio \zeta DR')
    ylabel(' Nat. Frequency \omega_n DR')
    hold on
    plot(zeta_dutch_cruise(3), Wn_dutch_cruise(3), 'xr','LineWidth', 1.5)
    plot(zeta_dutch_cruise(1), Wn_dutch_cruise(1), 'og','LineWidth', 1.5)
    plot(zeta_dutch_cruise(2), Wn_dutch_cruise(2), 'ob','LineWidth', 1.5)

    plot(Wnzeta_domain, Wnzeta_range_cruise, 'k','LineWidth', 1.5)
    plot(zeta_dutch_min*[1 1], [Wnzeta_range_cruise(1) 6], 'k', 'LineWidth', 1.5)
    legend('PC-9 Nominal x_c_g', 'PC-21 without Trainee', 'PC-21 with Trainee','lvl. 1 min. Requirement')
    grid on
    hold off
