
% P delen
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P04_P_del
%
% Hensikten med programmet er å styre
% hastigheten til en motor med en P-del
%
% Følgende  motorer brukes: 
%  - motor A
%
%--------------------------------------------------------------------------

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%         EXPERIMENT SETUP, FILENAME AND FIGURE

clear; close all   % Alltid lurt å rydde workspace opp først
online = 1;     % Online mot EV3 eller mot lagrede data?
plotting = false;  % Skal det plottes mens forsøket kjøres 
filename = 'P04_Kp_10.mat'; % Navnet på datafilen når online=0.

if online  
   mylego = legoev3('USB');
   joystick = vrjoystick(1);
   [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);

   % motorer
   motorA = motor(mylego,'A');
   motorA.resetRotation;
else
    % Dersom online=false lastes datafil.
    load(filename)
    % Siden while-løkken styres av en timer også i offlin, 
    % så vil du kunne at ikke hele figuren dukker opp dersom 
    % du ønsker plotting = true. Derfor settes begge false 
    online = false;
    plotting = false;
end

fig1=figure;
%set(gcf,'Position',[.., .., .., ..])
drawnow

% setter skyteknapp til 0, og initialiserer tellevariabel k
JoyMainSwitch=0;
k=0;
%----------------------------------------------------------------------

% Starter stoppeklokke for å stoppe 
% eksperiment automatisk når t>29 sekund. 
% Du kan også stoppe med skyteknappen som før.
duration = tic;

while ~JoyMainSwitch

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                       GET TIME AND MEASUREMENT
    % Få tid og målinger fra sensorer, motorer og joystick
    
    % oppdater tellevariabel
    k=k+1;

    if online
        if k==1
            % Spiller av lyd slik at du vet at innsamlingen har startet
            playTone(mylego,500,0.1)   % 500Hz i 0.1 sekund
            tic
            t(1) = 0;
        else
            t(k) = toc;
        end
        
        % motorer
        VinkelPosMotorA(k) = double(motorA.readRotation);
           
        % Data fra styrestikke. Utvid selv med andre knapper og akser
        [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);
    else
        % Når k er like stor som antall elementer i datavektpren Tid,
        % simuleres det at bryter på styrestikke trykkes inn.
        if k==length(t)
            JoyMainSwitch=1;
        end
        
        if plotting
            % Simulerer tiden som EV3-Matlab bruker på kommunikasjon 
            % når du har valgt "plotting=true" i offline
            pause(0.03)
        end
    end
    %--------------------------------------------------------------
    

    
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER

    % Stopper automatisk når t>29 sekund
    if toc(duration) > 29
        JoyMainSwitch = 1;
    end

    % Tilordne måling til variable
    %   Motorens vinkelposisjon 
    x1(k) = VinkelPosMotorA(k);

    if k==1
        % Regulatorparameter
        Kp = 0.005;    % start med lave verdier, typisk 0.005

        % Referanse-verdier og tidspunkt, og indeks for å spille lyd
        tidspunkt =  [0, 2,  6,   10,   14,  18];  % sekund
        RefVerdier = [0 300 600, 900, 1200, 500];  % grader/s
        RefVerdiIndeks = 1;

        % Initialverdier 
        tau_pos = 0.2;     % Tidskonstant, filtrert vinkelposisjon 
        x1_f(1) = 0;      % Filtrert vinkelposisjon
        x2(1) = 0;        % Vinkelhastighet motor

        % Måling, referansen, reguleringsavvik
        y(1) = x2(1);         % Måling vinkelhastighet
        r(1) = RefVerdier(1); % Referanse
        e(1) = r(1) - y(1);   % Reguleringsavvik

        % Initialverdi P-delen
        P(1) = 0;     

    else 
        % Beregninger av tidsskritt
        Ts = t(k) - t(k-1);

        % Filtrert vinkelposisjon x1_f(k)
        alfa_pos  = 1 - exp(-Ts/tau_pos);  % tidsavhengig alfa
        x1_f(k) = (1 - alfa_pos)*x1_f(k-1) + alfa_pos*x1(k);

        % Motorens vinkelhastighet (derivert av filtrert posisjon)
        x2(k) = (x1_f(k) - x1_f(k-1))/Ts;

        % Målingen y er vinkelhastighet
        y(k) = x2(k);     

        % Stegvis referanse: holder forrige verdi til neste tidspunkt nås
        r(k) = interp1(tidspunkt, RefVerdier, t(k), 'previous', 'extrap');

        % Reguleringssavvik
        e(k) = r(k) - y(k);

        % Lag kode for P-bidraget
        P(k) = Kp*e(k);

        % Spiller av varierende frekvens ved hvert skifte
        if online && r(k) ~= r(k-1)
            RefVerdiIndeks = RefVerdiIndeks + 1;
            playTone(mylego,RefVerdier(RefVerdiIndeks),0.5)   
        end
    end
    
    u_A(k) = P(k);

    %tester etterpå fro se forskjellen
    %u_A(k) = max(min(P(k),100),-100);

    if online
        motorA.Speed = u_A(k);
        start(motorA)
    end
    %--------------------------------------------------------------

 
    
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA
    % Husk at syntaksen plot(Tid(1:k),data(1:k))
    % for gir samme opplevelse i online=0 og online=1 siden
    % hele datasettet (1:end) eksisterer i den lagrede .mat fila

    % Plotter enten i sann tid eller når forsøk avsluttes
    if plotting || JoyMainSwitch
        figure(fig1)
        subplot(3,1,1)
        plot(t(1:k),r(1:k),'k--');
        hold on
        plot(t(1:k),y(1:k),'b-');
        hold off
        grid on
        ylabel('[$^{\circ}$/s]')
        text(t(k),r(k),['$',sprintf('%1.0f',r(k)),'^{\circ}$/s']);
        text(t(k),y(k),['$',sprintf('%1.0f',y(k)),'^{\circ}$/s']);
        title('M{\aa}lt vinkelhastighet og referanse')

        subplot(3,1,2)
        plot(t(1:k),e(1:k),'b-');
        grid on
        title('Reguleringsavvik')
        ylabel('[$^{\circ}$/s]')

        subplot(3,1,3)
        plot(t(1:k),u_A(1:k),'b-');
        hold on
        yline(100, 'k:','linewidth',2,'HandleVisibility','off')
        yline(-100, 'k:','linewidth',2,'HandleVisibility','off')
        hold off
        grid on
        title('P-bidraget')
        xlabel('Tid [sek]')


        % tegn naa (viktig kommando)
        drawnow
    end
    %--------------------------------------------------------------
end


% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%           STOP MOTORS

if online
    stop(motorA);
end

subplot(3,1,1)
text(22,630,'friksjon')
legend('$\{r_k\}$','$\{y_k\}$')
subplot(3,1,3)
legend(['P-del, $K_p$=',num2str(Kp)])

%------------------------------------------------------------------












% kjøring
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% ManuellKjoring_Kap9
%
% Manuell kjøring av EV3 med:
% - joystick-kalibrering
% - robust og tilgivende styring
% - logging av kapittel 9-signaler og kvalitetsmål:
%   y(k), r(k), e(k), u_A(k), u_B(k), IAE, MAE, TV_A, TV_B, y_mean, sigma
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

clear; close all; clc
set(groot,'defaultTextInterpreter','none');
set(groot,'defaultLegendInterpreter','none');
set(groot,'defaultAxesTickLabelInterpreter','none');

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% EXPERIMENT SETUP
online   = true;
plotting = false;   % true under testing, false ved endelig datainnsamling
filename = 'ManuellKjoring_Kap9_4.mat';

%-------------------- TUNING ----------------------------------------------
joySamples = 5;      % median over several joystick reads

% Big center area, but not so big that turning dies
neutralReleaseFwd  = 0.22;
neutralEngageFwd   = 0.34;

neutralReleaseTurn = 0.28;
neutralEngageTurn  = 0.42;

centerTrackAlpha = 0.01;   % very slow center tracking while released

% Response shape
driveExpo = 1.8;
turnExpo  = 2.8;

% Gains
driveGain = 0.85;
turnGain  = 0.72;

% Top speed
maxMotorSpeed = 22;

% Tiny commands forced to zero
stopThreshold = 3;

% Optional trim
motorTrimA = 0;
motorTrimB = 0;

% Straight-driving protection
assistFwdLevel = 0.18;

% Turning thresholds
reverseTurnThresholdBase  = 0.38;
reverseTurnThresholdSlope = 0.10;

forwardTurnThresholdBase  = 0.30;
forwardTurnThresholdSlope = 0.08;
%--------------------------------------------------------------------------

if online
    % LEGO EV3 and joystick
    mylego   = legoev3('USB');
    joystick = vrjoystick(1);

    % Sensors
    myColorSensor = colorSensor(mylego);
    myGyroSensor  = gyroSensor(mylego);
    resetRotationAngle(myGyroSensor);

    % Motors
    motorA = motor(mylego,'A');
    motorA.resetRotation;
    motorB = motor(mylego,'B');
    motorB.resetRotation;

    % Hard stop before calibration
    motorA.Speed = 0;
    motorB.Speed = 0;
    stop(motorA,1);
    stop(motorB,1);

    %--------------------------------------------------------------
    % CALIBRATION ROUTINE
    disp(' ')
    disp('==============================================')
    disp('JOYSTICK CALIBRATION STARTING')
    disp('Follow the instructions in the command window.')
    disp('==============================================')
    disp(' ')

    % Step 1: center, do not touch
    disp('STEP 1/8: DO NOT TOUCH THE JOYSTICK. Hold it centered.')
    playTone(mylego,400,0.15)
    pause(1.5)
    [centerX1,centerY1] = ReadJoystickMedian(joystick,60);

    % Step 2: right
    disp('STEP 2/8: Move joystick RIGHT and HOLD it there.')
    playTone(mylego,500,0.15)
    pause(1.5)
    [rightX,~] = ReadJoystickMedian(joystick,60);

    % Step 3: left
    disp('STEP 3/8: Move joystick LEFT and HOLD it there.')
    playTone(mylego,550,0.15)
    pause(1.5)
    [leftX,~] = ReadJoystickMedian(joystick,60);

    % Step 4: back
    disp('STEP 4/8: Pull joystick BACK and HOLD it there.')
    playTone(mylego,600,0.15)
    pause(1.5)
    [~,backY] = ReadJoystickMedian(joystick,60);

    % Step 5: forward
    disp('STEP 5/8: Push joystick FORWARD and HOLD it there.')
    playTone(mylego,650,0.15)
    pause(1.5)
    [~,forwardY] = ReadJoystickMedian(joystick,60);

    % Step 6: back to center
    disp('STEP 6/8: Release joystick and let it return to CENTER.')
    playTone(mylego,700,0.15)
    pause(1.5)
    [centerX2,centerY2] = ReadJoystickMedian(joystick,60);

    % Step 7: play around
    disp('STEP 7/8: Move the joystick around / play with it for a moment.')
    playTone(mylego,750,0.15)
    pause(2.0)
    [~,~] = ReadJoystickMedian(joystick,60);

    % Step 8: back to center again
    disp('STEP 8/8: Release joystick and let it return to CENTER again.')
    playTone(mylego,800,0.15)
    pause(1.5)
    [centerX3,centerY3] = ReadJoystickMedian(joystick,60);

    % Final center from multiple center captures
    JoyCenterX = median([centerX1 centerX2 centerX3]);
    JoyCenterY = median([centerY1 centerY2 centerY3]);

    % Asymmetric spans from calibration
    turnRightSpan = abs(rightX   - JoyCenterX);
    turnLeftSpan  = abs(leftX    - JoyCenterX);
    backSpan      = abs(backY    - JoyCenterY);
    forwardSpan   = abs(forwardY - JoyCenterY);

    % Safety against bad calibration
    minSpan = 0.20;
    turnRightSpan = max(turnRightSpan,minSpan);
    turnLeftSpan  = max(turnLeftSpan,minSpan);
    backSpan      = max(backSpan,minSpan);
    forwardSpan   = max(forwardSpan,minSpan);

    fprintf('\nCalibration results:\n')
    fprintf('Center X         = %.4f\n', JoyCenterX)
    fprintf('Center Y         = %.4f\n', JoyCenterY)
    fprintf('Right span       = %.4f\n', turnRightSpan)
    fprintf('Left span        = %.4f\n', turnLeftSpan)
    fprintf('Back span        = %.4f\n', backSpan)
    fprintf('Forward span     = %.4f\n', forwardSpan)

    disp('Calibration complete.')
    playTone(mylego,900,0.20)
    pause(0.5)

else
    load(filename)

    if ~exist('JoyForoverRaw','var'), JoyForoverRaw = zeros(size(t)); end
    if ~exist('JoySvingRaw','var'),   JoySvingRaw   = zeros(size(t)); end
    if ~exist('JoyForover','var'),    JoyForover    = zeros(size(t)); end
    if ~exist('JoySving','var'),      JoySving      = zeros(size(t)); end
    if ~exist('u_A','var'),           u_A           = zeros(size(t)); end
    if ~exist('u_B','var'),           u_B           = zeros(size(t)); end
    if ~exist('Lys','var'),           Lys           = zeros(size(t)); end
    if ~exist('GyroAngle','var'),     GyroAngle     = zeros(size(t)); end
    if ~exist('VinkelPosMotorA','var'), VinkelPosMotorA = zeros(size(t)); end
    if ~exist('VinkelPosMotorB','var'), VinkelPosMotorB = zeros(size(t)); end
    if ~exist('NeutralFlag','var'),   NeutralFlag   = zeros(size(t)); end

    % Default offline values
    JoyCenterX = 0;
    JoyCenterY = 0;
    turnRightSpan = 1;
    turnLeftSpan  = 1;
    backSpan      = 1;
    forwardSpan   = 1;
end

fig1 = figure;
set(gcf,'units','normalized','outerposition',[0.05 0.08 0.78 0.80])
drawnow

fig2 = figure;
set(gcf,'units','normalized','outerposition',[0.85 0.55 0.13 0.22])
drawnow

JoyMainSwitch = 0;
k = 0;
neutralLock = true;

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

        % Sensors
        Lys(k)       = double(readLightIntensity(myColorSensor,'reflected'));
        GyroAngle(k) = double(readRotationAngle(myGyroSensor));

        % Motor positions
        VinkelPosMotorA(k) = double(motorA.readRotation);
        VinkelPosMotorB(k) = double(motorB.readRotation);

        %----------------------------------------------------------
        % Read joystick several times and take median
        ax1 = zeros(1,joySamples);
        ax2 = zeros(1,joySamples);
        btn = zeros(1,joySamples);

        for i = 1:joySamples
            [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);
            ax1(i) = JoyAxes(1);
            ax2(i) = JoyAxes(2);
            btn(i) = JoyButtons(1);
        end

        JoyMainSwitch = max(btn);

        %----------------------------------------------------------
        % Apply calibrated center and calibrated span

        % Turn axis: different scaling for left/right
        corrX = median(ax1) - JoyCenterX;
        if corrX >= 0
            rawTurn = corrX / turnRightSpan;
        else
            rawTurn = corrX / turnLeftSpan;
        end

        % Forward/back axis
        corrY = median(ax2) - JoyCenterY;

        % forward = positive, back = negative
        if corrY <= 0
            rawFwd = -corrY / forwardSpan;
        else
            rawFwd = -corrY / backSpan;
        end

        % Clamp
        rawTurn = max(min(rawTurn,1),-1);
        rawFwd  = max(min(rawFwd ,1),-1);

        JoySvingRaw(k)   = rawTurn;
        JoyForoverRaw(k) = rawFwd;

    else
        if k == length(t)
            JoyMainSwitch = 1;
        end

        if plotting
            pause(0.03)
        end

        rawTurn = JoySvingRaw(k);
        rawFwd  = JoyForoverRaw(k);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % NEUTRAL LOCK WITH HYSTERESIS
    if neutralLock
        if abs(rawFwd) > neutralEngageFwd || abs(rawTurn) > neutralEngageTurn
            neutralLock = false;
        else
            % Track slight center drift only while released
            if online
                JoyCenterX = (1-centerTrackAlpha)*JoyCenterX + centerTrackAlpha*median(ax1);
                JoyCenterY = (1-centerTrackAlpha)*JoyCenterY + centerTrackAlpha*median(ax2);
            end

            rawTurn = 0;
            rawFwd  = 0;
        end
    else
        if abs(rawFwd) < neutralReleaseFwd && abs(rawTurn) < neutralReleaseTurn
            neutralLock = true;
            rawTurn = 0;
            rawFwd  = 0;
        end
    end

    NeutralFlag(k) = neutralLock;

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % SHAPING AND MOTOR COMMANDS
    if neutralLock
        JoySving(k)   = 0;
        JoyForover(k) = 0;
        u_A(k) = 0;
        u_B(k) = 0;

    else
        % Forward deadband
        if abs(rawFwd) < neutralReleaseFwd
            fwdCmd = 0;
        else
            fwdCmd = sign(rawFwd) * ...
                (abs(rawFwd) - neutralReleaseFwd) / (1 - neutralReleaseFwd);
        end

        % Turn suppression
        turnThreshold = neutralReleaseTurn;

        if rawFwd <= -assistFwdLevel
            turnThreshold = max(turnThreshold, ...
                reverseTurnThresholdBase + reverseTurnThresholdSlope*abs(rawFwd));
        elseif rawFwd >= assistFwdLevel
            turnThreshold = max(turnThreshold, ...
                forwardTurnThresholdBase + forwardTurnThresholdSlope*abs(rawFwd));
        end

        turnThreshold = min(turnThreshold,0.55);

        if abs(rawTurn) < turnThreshold
            turnCmd = 0;
        else
            turnCmd = sign(rawTurn) * ...
                (abs(rawTurn) - turnThreshold) / (1 - turnThreshold);
        end

        % Shape signals
        JoyForover(k) = sign(fwdCmd)  * abs(fwdCmd)^driveExpo;
        JoySving(k)   = sign(turnCmd) * abs(turnCmd)^turnExpo;

        % Additional turn damping while moving
        effectiveTurnGain = turnGain * (1 - 0.45*abs(JoyForover(k)));
        effectiveTurnGain = max(effectiveTurnGain,0.18);

        % Arcade drive mix
        left  = driveGain * JoyForover(k) + effectiveTurnGain * JoySving(k);
        right = driveGain * JoyForover(k) - effectiveTurnGain * JoySving(k);

        % Normalize
        mixMax = max([1, abs(left), abs(right)]);
        left  = left  / mixMax;
        right = right / mixMax;

        % Motor commands
        u_A(k) = round(maxMotorSpeed * left)  + motorTrimA;
        u_B(k) = round(maxMotorSpeed * right) + motorTrimB;

        % Kill tiny commands
        if abs(u_A(k)) <= stopThreshold
            u_A(k) = 0;
        end
        if abs(u_B(k)) <= stopThreshold
            u_B(k) = 0;
        end

        % Final clamp
        u_A(k) = max(min(u_A(k), maxMotorSpeed), -maxMotorSpeed);
        u_B(k) = max(min(u_B(k), maxMotorSpeed), -maxMotorSpeed);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % TIME STEP
    if k == 1
        Ts(1) = 0.05;
    else
        Ts(k) = t(k) - t(k-1);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % KAPITTEL 9 SIGNALER OG KVALITETSMÅL
    y(k) = Lys(k);

    if k == 1
        r(1) = y(1);          % referanse = første lysmåling på startfeltet
        e(1) = r(1) - y(1);

        IAE(1) = 0;
        MAE(1) = abs(e(1));

        TV_A(1) = 0;
        TV_B(1) = 0;

        y_mean(1) = y(1);
        sigma(1)  = 0;
    else
        r(k) = r(1);
        e(k) = r(k) - y(k);

        % IAE med trapesmetoden
        IAE(k) = IAE(k-1) + 0.5*(abs(e(k)) + abs(e(k-1)))*Ts(k);

        % MAE rekursivt
        MAE(k) = ((k-1)*MAE(k-1) + abs(e(k))) / k;

        % Total variation
        TV_A(k) = TV_A(k-1) + abs(u_A(k) - u_A(k-1));
        TV_B(k) = TV_B(k-1) + abs(u_B(k) - u_B(k-1));

        % Løpende middelverdi og standardavvik av y
        y_mean(k) = mean(y(1:k));
        sigma(k)  = std(y(1:k));
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % SEND MOTOR COMMANDS
    if online
        if neutralLock
            motorA.Speed = 0;
            motorB.Speed = 0;
            stop(motorA,1);
            stop(motorB,1);
        else
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
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % PLOT
    if plotting || JoyMainSwitch
        figure(fig1)

        subplot(4,2,1)
        plot(t(1:k),GyroAngle(1:k),'b')
        grid on
        title('Vinkelmåling, gyrosensor')
        ylabel('[deg]')

        subplot(4,2,2)
        plot(t(1:k),y(1:k),'b')
        hold on
        plot(t(1:k),r(1:k),'r--')
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
        hold off
        grid on
        title('Pådrag motor A og B')
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
        title('Histogram av lysmåling y')
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
% PRINT FINAL RESULTS
fprintf('\nFinished.\n')
fprintf('Samples  = %d\n', k)

if length(Ts) > 1
    fprintf('Mean Ts  = %.4f s\n', mean(Ts(2:end)))
else
    fprintf('Mean Ts  = %.4f s\n', Ts(1))
end

fprintf('MAE      = %.3f\n', MAE(end))
fprintf('IAE      = %.3f\n', IAE(end))
fprintf('TV_A     = %.3f\n', TV_A(end))
fprintf('TV_B     = %.3f\n', TV_B(end))
fprintf('y_mean   = %.3f\n', y_mean(end))
fprintf('sigma    = %.3f\n', sigma(end))

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% LOCAL FUNCTION
function [mx,my] = ReadJoystickMedian(joystick,N)
ax1 = zeros(1,N);
ax2 = zeros(1,N);

for ii = 1:N
    [JoyAxes,~] = HentJoystickVerdier(joystick);
    ax1(ii) = JoyAxes(1);
    ax2(ii) = JoyAxes(2);
    pause(0.005)
end

mx = median(ax1);
my = median(ax2);
end