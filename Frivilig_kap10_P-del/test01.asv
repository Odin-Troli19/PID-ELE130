%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% AutoKjoring_P_Kap10
%
% Automatisk kjøring av EV3 langs baneprofil med P-regulator.
% Bygger på ManuellKjoring_Kap9, men joystick-styring er erstattet
% av automatisk styring basert på lysmåling.
%
% Logger samme kvalitetsmål som under manuell kjøring:
% y(k), r(k), e(k), u_A(k), u_B(k), IAE, MAE, TV_A, TV_B, y_mean, sigma,
% Ts, kjøretid og antall målinger.
%
% Viktig:
% - u0 > 0 gir framdrift
% - Kp bestemmer hvor hardt roboten svinger tilbake mot referansen
% - steeringSign må kanskje byttes fra 1 til -1 dersom roboten svinger feil vei
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

clear; close all; clc
set(groot,'defaultTextInterpreter','none');
set(groot,'defaultLegendInterpreter','none');
set(groot,'defaultAxesTickLabelInterpreter','none');

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% EXPERIMENT SETUP
online   = true;    % true = kjør på EV3, false = last inn lagret fil
plotting = false;   % true under testing, false ved endelig datainnsamling

% Velg filnavn som passer parameterne dine
filename = 'AutoKjoring_P_u0_5_Kp_0p20.mat';

%-------------------- PARAMETERE SOM DU TESTER ----------------------------
u0 = 5z;             % grunnpådrag framover; 5
Kp = 0.5;          % P-forsterkning

%1.forsøk kp=0.20; kp=0.3;kp=0.5
steeringSign = 1;   % bytt til -1 hvis roboten svinger feil vei

maxMotorSpeed = 100;  % maksimal EV3-speed
allowReverse  = false; % false = ingen revers i sving, mer stabil kjøring
maxTime       = 100;    % automatisk stopp etter 29 sekunder

% Referanse for lysmåling
% 'startValue'  -> bruker første lysmåling som referanse
% 'manual'      -> bruker verdien i r_manual
referenceMode = 'startValue';
r_manual      = 45;

% Metadata til tabell 10.1
plattform   = 'PC';
stromkilde  = '230V';
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
    % Last lagret data
    load(filename)

    % Sikre at nødvendige variabler finnes ved offline-plotting
    if ~exist('t','var'), error('Fant ikke tidsvektor t i filen.'); end
    if ~exist('Lys','var'), Lys = zeros(size(t)); end
    if ~exist('GyroAngle','var'), GyroAngle = zeros(size(t)); end
    if ~exist('VinkelPosMotorA','var'), VinkelPosMotorA = zeros(size(t)); end
    if ~exist('VinkelPosMotorB','var'), VinkelPosMotorB = zeros(size(t)); end
    if ~exist('u_A','var'), u_A = zeros(size(t)); end
    if ~exist('u_B','var'), u_B = zeros(size(t)); end
    if ~exist('y','var'), y = zeros(size(t)); end
    if ~exist('r','var'), r = zeros(size(t)); end
    if ~exist('e','var'), e = zeros(size(t)); end
    if ~exist('IAE','var'), IAE = zeros(size(t)); end
    if ~exist('MAE','var'), MAE = zeros(size(t)); end
    if ~exist('TV_A','var'), TV_A = zeros(size(t)); end
    if ~exist('TV_B','var'), TV_B = zeros(size(t)); end
    if ~exist('y_mean','var'), y_mean = zeros(size(t)); end
    if ~exist('sigma','var'), sigma = zeros(size(t)); end
    if ~exist('Ts','var'), Ts = zeros(size(t)); end
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% FIGURES
fig1 = figure;
set(gcf,'units','normalized','outerposition',[0.05 0.08 0.78 0.82])
drawnow

fig2 = figure;
set(gcf,'units','normalized','outerposition',[0.85 0.55 0.13 0.22])
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

        % nødstoppsknapp på joystick
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
                r(1) = y(1);       % samme idé som i manuell kjøring
            case 'manual'
                r(1) = r_manual;   % fast verdi valgt på forhånd
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
    % P-REGULATOR FOR STYRING
    u_sving(k) = Kp * e(k);

    % Fordel styringsbidraget på motorene
    u_A_unsat(k) = u0 + steeringSign * u_sving(k);
    u_B_unsat(k) = u0 - steeringSign * u_sving(k);

    % Metning
    if allowReverse
        u_A(k) = ClampValue(u_A_unsat(k), -maxMotorSpeed, maxMotorSpeed);
        u_B(k) = ClampValue(u_B_unsat(k), -maxMotorSpeed, maxMotorSpeed);
    else
        u_A(k) = ClampValue(u_A_unsat(k), 0, maxMotorSpeed);
        u_B(k) = ClampValue(u_B_unsat(k), 0, maxMotorSpeed);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % KVALITETSMÅL SOM I KAPITTEL 9
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

        % Total variation for motorpådrag
        TV_A(k) = TV_A(k-1) + abs(u_A(k) - u_A(k-1));
        TV_B(k) = TV_B(k-1) + abs(u_B(k) - u_B(k-1));

        % Løpende middelverdi og standardavvik for lysmålingen
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
        grid on
        title('Reguleringsavvik')
        ylabel('e(k)')

        subplot(4,2,4)
        plot(t(1:k),u_A(1:k),'b')
        hold on
        plot(t(1:k),u_B(1:k),'r')
        yline(u0,'k--','HandleVisibility','off')
        hold off
        grid on
        title('Motorpådrag')
        ylabel('speed')
        legend('u_A(k)','u_B(k)','Location','best')

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

        figure(fig2)
        clf
        histogram(y(1:k))
        grid on
        title('Histogram av lysmåling')
        xlabel('y')
        ylabel('antall')

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
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% SAVE DATA
if online
    save(filename)
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% RESULTATER TIL TABELL 10.1
fprintf('\nFinished.\n')
fprintf('u0                 = %.3f\n', u0)
fprintf('Kp                 = %.3f\n', Kp)
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

% Lag en oppsummeringsstruktur som kan brukes videre til tabell 10.1
RunSummary = struct;
RunSummary.filename      = filename;
RunSummary.u0            = u0;
RunSummary.Kp            = Kp;
RunSummary.plattform     = plattform;
RunSummary.stromkilde    = stromkilde;
RunSummary.plotting      = plotting;
RunSummary.referanse     = r(end);
RunSummary.y_mean        = y_mean(end);
RunSummary.sigma         = sigma(end);
RunSummary.IAE           = IAE(end);
RunSummary.MAE           = MAE(end);
RunSummary.TV_A          = TV_A(end);
RunSummary.TV_B          = TV_B(end);
RunSummary.meanTs        = meanTs;
RunSummary.kjoretid      = t(k);
RunSummary.antallMalinger = k;

disp(' ')
disp('RunSummary:')
disp(RunSummary)

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% LOKAL FUNKSJON
function y = ClampValue(x, xmin, xmax)
y = min(max(x, xmin), xmax);
end