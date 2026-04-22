%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% AutoKjoring_PID_Kap10
%
% Automatisk kjøring av EV3 langs baneprofil med PID-regulator.
% Bygger på AutoKjoring_P_Kap10, men P-leddet er utvidet til PID.
%
% Måling:
%   y(k) = lysmåling fra fargesensor
%
% Referanse:
%   r(k) = ønsket lysnivå
%
% Reguleringsavvik:
%   e(k) = r(k) - y(k)
%
% Styresignal:
%   u_pid(k) = P(k) + I(k) + D(k)
%
% Motorpådrag:
%   u_A(k) = u0(k) + steeringSign*u_pid(k)
%   u_B(k) = u0(k) - steeringSign*u_pid(k)
%
% Logger samme kvalitetsmål som under manuell kjøring:
% y(k), r(k), e(k), u_A(k), u_B(k), IAE, MAE, TV_A, TV_B, y_mean, sigma,
% Ts, kjøretid og antall målinger.
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

clear; close all; clc
set(groot,'defaultTextInterpreter','none');
set(groot,'defaultLegendInterpreter','none');
set(groot,'defaultAxesTickLabelInterpreter','none');

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% EXPERIMENT SETUP
online   = true;     % true = kjør på EV3, false = last inn lagret fil
plotting = false;    % true under testing, false ved endelig datainnsamling

filename = 'AutoKjoring_PID_u0_6_4_Kp_0p30_Ki_0p02_Kd_0p04.mat';

%-------------------- PARAMETERE DU TESTER -------------------------------
% Basispådrag
u0_straight = 6;     % basispådrag på rettstrekning
u0_curve    = 4;     % lavere basispådrag i sving
adaptiveBaseSpeed = true;   % true = bruk lavere u0 i sving
pidTurnThreshold  = 8;      % grense for når vi sier "nå er vi i sving"

% PID-parametere
Kp = 0.30;
Ki = 0.02;
Kd = 0.04;

% Filtrering og integratorgrenser
tau_e = 0.05;        % filtrering av e(k) til D-leddet
I_max = 25;          % øvre grense for I-leddet
I_min = -25;         % nedre grense for I-leddet

% Retning på styring
steeringSign = 1;    % bytt til -1 hvis roboten svinger feil vei

% Grenser og stopp
maxMotorSpeed = 100;
allowReverse  = false;  % false = motorene får ikke gå bakover
maxTime       = 100;    % automatisk stopp etter maxTime sekunder

% Referanse for lysmåling
% 'startValue'  -> bruker første lysmåling som referanse
% 'manual'      -> bruker verdien i r_manual
referenceMode = 'startValue';
r_manual      = 40;

% Metadata til tabell
plattform  = 'PC';
stromkilde = '230V';
%--------------------------------------------------------------------------

if online
    % LEGO EV3 og joystick (joystick brukes kun som nødstoppsknapp)
    mylego   = legoev3('USB');
    joystick = vrjoystick(1);

    % sensorer
    myColorSensor = colorSensor(mylego);
    myGyroSensor  = gyroSensor(mylego);
    resetRotationAngle(myGyroSensor);

    % motorer
    motorA = motor(mylego,'A');
    motorB = motor(mylego,'B');
    motorA.resetRotation;
    motorB.resetRotation;

    % sikker stopp før start
    motorA.Speed = 0;
    motorB.Speed = 0;
    stop(motorA,1);
    stop(motorB,1);

else
    load(filename)

    if ~exist('t','var'), error('Fant ikke tidsvektor t i filen.'); end
    if ~exist('Lys','var'), Lys = zeros(size(t)); end
    if ~exist('GyroAngle','var'), GyroAngle = zeros(size(t)); end
    if ~exist('VinkelPosMotorA','var'), VinkelPosMotorA = zeros(size(t)); end
    if ~exist('VinkelPosMotorB','var'), VinkelPosMotorB = zeros(size(t)); end
    if ~exist('u_A','var'), u_A = zeros(size(t)); end
    if ~exist('u_B','var'), u_B = zeros(size(t)); end
    if ~exist('u_A_unsat','var'), u_A_unsat = zeros(size(t)); end
    if ~exist('u_B_unsat','var'), u_B_unsat = zeros(size(t)); end
    if ~exist('y','var'), y = zeros(size(t)); end
    if ~exist('r','var'), r = zeros(size(t)); end
    if ~exist('e','var'), e = zeros(size(t)); end
    if ~exist('e_f','var'), e_f = zeros(size(t)); end
    if ~exist('P','var'), P = zeros(size(t)); end
    if ~exist('I','var'), I = zeros(size(t)); end
    if ~exist('D','var'), D = zeros(size(t)); end
    if ~exist('u_pid','var'), u_pid = zeros(size(t)); end
    if ~exist('u0_vec','var'), u0_vec = zeros(size(t)); end
    if ~exist('curveFlag','var'), curveFlag = zeros(size(t)); end
    if ~exist('IAE','var'), IAE = zeros(size(t)); end
    if ~exist('MAE','var'), MAE = zeros(size(t)); end
    if ~exist('TV_A','var'), TV_A = zeros(size(t)); end
    if ~exist('TV_B','var'), TV_B = zeros(size(t)); end
    if ~exist('y_mean','var'), y_mean = zeros(size(t)); end
    if ~exist('sigma','var'), sigma = zeros(size(t)); end
    if ~exist('Ts','var'), Ts = zeros(size(t)); end

    online = false;
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% FIGURES
fig1 = figure;
set(fig1,'units','normalized','outerposition',[0.04 0.08 0.78 0.82])
drawnow

fig2 = figure;
set(fig2,'units','normalized','outerposition',[0.84 0.56 0.14 0.22])
drawnow

fig3 = figure;
set(fig3,'units','normalized','outerposition',[0.82 0.08 0.17 0.38])
drawnow

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% INITIALISERING
JoyMainSwitch = 0;
k = 0;
duration = tic;

while ~JoyMainSwitch

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % GET TIME AND MEASUREMENTS
    k = k + 1;

    if online
        if k == 1
            playTone(mylego,500,0.1)
            tic
            t(1) = 0;
        else
            t(k) = toc;
        end

        % sensorer
        Lys(k)       = double(readLightIntensity(myColorSensor,'reflected'));
        GyroAngle(k) = double(readRotationAngle(myGyroSensor));

        % motorposisjon
        VinkelPosMotorA(k) = double(motorA.readRotation);
        VinkelPosMotorB(k) = double(motorB.readRotation);

        % nødstoppsknapp
        [~,JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);

    else
        if k == length(t)
            JoyMainSwitch = 1;
        end

        if plotting
            pause(0.03)
        end
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % STOPPKRITERIUM PÅ TID
    if toc(duration) > maxTime
        JoyMainSwitch = 1;
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % TIDSSKRITT
    if k == 1
        Ts(1) = 0.05;
    else
        Ts(k) = t(k) - t(k-1);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % MÅLESIGNAL
    y(k) = Lys(k);

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % REFERANSE
    if k == 1
        switch lower(referenceMode)
            case 'startvalue'
                r(1) = y(1);
            case 'manual'
                r(1) = r_manual;
            otherwise
                error('referenceMode må være ''startValue'' eller ''manual''.')
        end
    else
        r(k) = r(1);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % REGULERINGSAVVIK
    e(k) = r(k) - y(k);

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % PID-REGULATOR
    if k == 1
        e_f(1)   = e(1);

        P(1)     = Kp*e(1);
        I(1)     = 0;
        D(1)     = 0;
        u_pid(1) = P(1) + I(1) + D(1);

        % Variabelt basispådrag i sving
        if adaptiveBaseSpeed && abs(u_pid(1)) > pidTurnThreshold
            u0_vec(1)   = u0_curve;
            curveFlag(1) = 1;
        else
            u0_vec(1)   = u0_straight;
            curveFlag(1) = 0;
        end

    else
        % P-ledd
        P(k) = Kp*e(k);

        % Filtrert error til D-ledd
        alpha_e = 1 - exp(-Ts(k)/tau_e);
        e_f(k)  = (1 - alpha_e)*e_f(k-1) + alpha_e*e(k);

        % D-ledd
        D(k) = Kd*(e_f(k) - e_f(k-1))/Ts(k);

        % Kandidat for I-leddet med trapesmetoden
        I_candidate = I(k-1) + 0.5*Ki*(e(k-1) + e(k))*Ts(k);
        I_candidate = ClampValue(I_candidate, I_min, I_max);

        % Midlertidig PID for å avgjøre om vi er i sving
        u_pid_candidate = P(k) + I_candidate + D(k);

        if adaptiveBaseSpeed && abs(u_pid_candidate) > pidTurnThreshold
            u0_temp = u0_curve;
            curveFlag(k) = 1;
        else
            u0_temp = u0_straight;
            curveFlag(k) = 0;
        end

        % Midlertidige motorpådrag
        u_A_test = u0_temp + steeringSign*u_pid_candidate;
        u_B_test = u0_temp - steeringSign*u_pid_candidate;

        if allowReverse
            u_A_test_sat = ClampValue(u_A_test,-maxMotorSpeed,maxMotorSpeed);
            u_B_test_sat = ClampValue(u_B_test,-maxMotorSpeed,maxMotorSpeed);
        else
            u_A_test_sat = ClampValue(u_A_test,0,maxMotorSpeed);
            u_B_test_sat = ClampValue(u_B_test,0,maxMotorSpeed);
        end

        % Enkel anti-windup:
        % hvis vi er mettet og feilen prøver å gjøre metningen verre,
        % så fryses I-leddet
        satFlag = (u_A_test ~= u_A_test_sat) || (u_B_test ~= u_B_test_sat);

        if satFlag && sign(e(k)) == sign(u_pid_candidate)
            I(k) = I(k-1);
        else
            I(k) = I_candidate;
        end

        % Endelig PID-signal
        u_pid(k) = P(k) + I(k) + D(k);

        % Endelig valg av basispådrag
        if adaptiveBaseSpeed && abs(u_pid(k)) > pidTurnThreshold
            u0_vec(k)   = u0_curve;
            curveFlag(k) = 1;
        else
            u0_vec(k)   = u0_straight;
            curveFlag(k) = 0;
        end
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % MOTORPÅDRAG
    u_A_unsat(k) = u0_vec(k) + steeringSign*u_pid(k);
    u_B_unsat(k) = u0_vec(k) - steeringSign*u_pid(k);

    if allowReverse
        u_A(k) = ClampValue(u_A_unsat(k), -maxMotorSpeed, maxMotorSpeed);
        u_B(k) = ClampValue(u_B_unsat(k), -maxMotorSpeed, maxMotorSpeed);
    else
        u_A(k) = ClampValue(u_A_unsat(k), 0, maxMotorSpeed);
        u_B(k) = ClampValue(u_B_unsat(k), 0, maxMotorSpeed);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % KVALITETSMÅL SOM I MANUELL KJØRING
    if k == 1
        IAE(1) = 0;
        MAE(1) = abs(e(1));

        TV_A(1) = 0;
        TV_B(1) = 0;

        y_mean(1) = y(1);
        sigma(1)  = 0;
    else
        % IAE med trapesmetoden
        IAE(k) = IAE(k-1) + 0.5*(abs(e(k)) + abs(e(k-1)))*Ts(k);

        % MAE rekursivt
        MAE(k) = ((k-1)*MAE(k-1) + abs(e(k))) / k;

        % Total variation
        TV_A(k) = TV_A(k-1) + abs(u_A(k) - u_A(k-1));
        TV_B(k) = TV_B(k-1) + abs(u_B(k) - u_B(k-1));

        % Løpende middelverdi og standardavvik av lysmålingen
        y_mean(k) = mean(y(1:k));
        sigma(k)  = std(y(1:k));
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % SEND MOTORPÅDRAG
    if online
        if u_A(k) == 0
            motorA.Speed = 0;
            stop(motorA,1);
        else
            motorA.Speed = u_A(k);
            start(motorA);
        end

        if u_B(k) == 0
            motorB.Speed = 0;
            stop(motorB,1);
        else
            motorB.Speed = u_B(k);
            start(motorB);
        end
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % PLOT
    if plotting || JoyMainSwitch
        %---------------------- Resultatfigur ------------------------------
        figure(fig1)

        subplot(4,2,1)
        plot(t(1:k),GyroAngle(1:k),'b')
        grid on
        title('Gyrovinkel')
        ylabel('[deg]')

        subplot(4,2,2)
        plot(t(1:k),y(1:k),'b')
        hold on
        plot(t(1:k),r(1:k),'r--','LineWidth',1.2)
        hold off
        grid on
        title('Lysmåling og referanse')
        ylabel('[-]')
        legend('y(k)','r(k)','Location','best')

        subplot(4,2,3)
        plot(t(1:k),e(1:k),'b')
        hold on
        plot(t(1:k),e_f(1:k),'r--')
        hold off
        grid on
        title('Reguleringsavvik')
        ylabel('e(k)')
        legend('e(k)','e_f(k)','Location','best')

        subplot(4,2,4)
        plot(t(1:k),u_A(1:k),'b')
        hold on
        plot(t(1:k),u_B(1:k),'r')
        plot(t(1:k),u0_vec(1:k),'k--')
        hold off
        grid on
        title('Motorpådrag')
        ylabel('speed')
        legend('u_A(k)','u_B(k)','u_0(k)','Location','best')

        subplot(4,2,5)
        plot(t(1:k),IAE(1:k),'b')
        hold on
        plot(t(k),IAE(k),'ko','MarkerFaceColor','k')
        hold off
        grid on
        title('Integral of absolute error')
        ylabel('IAE')

        subplot(4,2,6)
        plot(t(1:k),TV_A(1:k),'b')
        hold on
        plot(t(1:k),TV_B(1:k),'r')
        plot(t(k),TV_A(k),'ko','MarkerFaceColor','k')
        plot(t(k),TV_B(k),'ks','MarkerFaceColor','k')
        hold off
        grid on
        title('Total variation')
        ylabel('TV')
        legend('TV_A','TV_B','Location','best')

        subplot(4,2,7)
        plot(t(1:k),MAE(1:k),'b')
        hold on
        plot(t(k),MAE(k),'ko','MarkerFaceColor','k')
        hold off
        grid on
        title('Mean absolute error')
        ylabel('MAE')
        xlabel('Tid [s]')

        subplot(4,2,8)
        plot(t(1:k),Ts(1:k),'b')
        grid on
        title('Tidsskritt T_s')
        ylabel('[s]')
        xlabel('Tid [s]')

        %---------------------- Histogram --------------------------------
        figure(fig2)
        clf
        histogram(y(1:k))
        grid on
        title('Histogram av lysmåling')
        xlabel('y')
        ylabel('antall')

        %---------------------- PID-bidrag --------------------------------
        figure(fig3)
        clf

        subplot(2,1,1)
        plot(t(1:k),P(1:k),'b')
        hold on
        plot(t(1:k),I(1:k),'r')
        plot(t(1:k),D(1:k),'g')
        plot(t(1:k),u_pid(1:k),'k','LineWidth',1.2)
        hold off
        grid on
        title('PID-bidrag')
        ylabel('bidrag')
        legend('P(k)','I(k)','D(k)','u_{pid}(k)','Location','best')

        subplot(2,1,2)
        plot(t(1:k),u0_vec(1:k),'k--','LineWidth',1.2)
        hold on
        plot(t(1:k),u_A(1:k),'b')
        plot(t(1:k),u_B(1:k),'r')
        hold off
        grid on
        title('Basispådrag og motorpådrag')
        xlabel('Tid [s]')
        ylabel('speed')
        legend('u_0(k)','u_A(k)','u_B(k)','Location','best')

        drawnow
    end
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% STOP MOTORS
if online
    motorA.Speed = 0;
    motorB.Speed = 0;
    stop(motorA,1);
    stop(motorB,1);
    playTone(mylego,700,0.15)
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% SAVE DATA
if online
    save(filename)
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% PRINT FINAL RESULTS
fprintf('\nFinished.\n')
fprintf('u0_straight        = %.3f\n', u0_straight)
fprintf('u0_curve           = %.3f\n', u0_curve)
fprintf('Kp                 = %.3f\n', Kp)
fprintf('Ki                 = %.3f\n', Ki)
fprintf('Kd                 = %.3f\n', Kd)
fprintf('Samples            = %d\n', k)

if length(Ts) > 1
    meanTs = mean(Ts(2:end));
else
    meanTs = Ts(1);
end

fprintf('Mean Ts            = %.4f s\n', meanTs)
fprintf('Kjoretid           = %.3f s\n', t(k))
fprintf('Referanse r        = %.3f\n', r(end))
fprintf('y_mean             = %.3f\n', y_mean(end))
fprintf('sigma              = %.3f\n', sigma(end))
fprintf('IAE                = %.3f\n', IAE(end))
fprintf('MAE                = %.3f\n', MAE(end))
fprintf('TV_A               = %.3f\n', TV_A(end))
fprintf('TV_B               = %.3f\n', TV_B(end))

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% RUNSUMMARY TIL TABELL
RunSummary = struct;
RunSummary.filename        = filename;
RunSummary.u0_straight     = u0_straight;
RunSummary.u0_curve        = u0_curve;
RunSummary.Kp              = Kp;
RunSummary.Ki              = Ki;
RunSummary.Kd              = Kd;
RunSummary.tau_e           = tau_e;
RunSummary.I_min           = I_min;
RunSummary.I_max           = I_max;
RunSummary.plattform       = plattform;
RunSummary.stromkilde      = stromkilde;
RunSummary.plotting        = plotting;
RunSummary.referanse       = r(end);
RunSummary.y_mean          = y_mean(end);
RunSummary.sigma           = sigma(end);
RunSummary.IAE             = IAE(end);
RunSummary.MAE             = MAE(end);
RunSummary.TV_A            = TV_A(end);
RunSummary.TV_B            = TV_B(end);
RunSummary.meanTs          = meanTs;
RunSummary.kjoretid        = t(k);
RunSummary.antallMalinger  = k;

disp(' ')
disp('RunSummary:')
disp(RunSummary)

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% SAVE ALL RELEVANT DATA
save(filename, ...
    't','Ts', ...
    'Lys','GyroAngle', ...
    'VinkelPosMotorA','VinkelPosMotorB', ...
    'y','r','e','e_f', ...
    'P','I','D','u_pid', ...
    'u0_vec','curveFlag', ...
    'u_A','u_B','u_A_unsat','u_B_unsat', ...
    'IAE','MAE','TV_A','TV_B','y_mean','sigma', ...
    'u0_straight','u0_curve','adaptiveBaseSpeed','pidTurnThreshold', ...
    'Kp','Ki','Kd','tau_e','I_min','I_max', ...
    'steeringSign','maxMotorSpeed','allowReverse','maxTime', ...
    'referenceMode','r_manual', ...
    'plattform','stromkilde','RunSummary');

% Optional: lagre figurer
% exportgraphics(fig1, strrep(filename,'.mat','_resultater.pdf'),'ContentType','vector');
% exportgraphics(fig2, strrep(filename,'.mat','_histogram.pdf'),'ContentType','vector');
% exportgraphics(fig3, strrep(filename,'.mat','_PIDbidrag.pdf'),'ContentType','vector');

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% LOKAL FUNKSJON
function y = ClampValue(x, xmin, xmax)
y = min(max(x, xmin), xmax);
end