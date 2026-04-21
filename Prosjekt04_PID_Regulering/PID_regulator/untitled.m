%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P04_PID
%
% Hensikten med programmet er å styre
% hastigheten til en motor med en komplett PID-regulator
%
% Følgende  motorer brukes: 
%  - motor A
%
%--------------------------------------------------------------------------


%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%         EXPERIMENT SETUP, FILENAME AND FIGURE

clear; close all   % Alltid lurt å rydde workspace opp først
online = true;     % Online mot EV3 eller mot lagrede data?
plotting = true;  % Skal det plottes mens forsøket kjøres
filename = 'P04_PID2.mat'; % Navnet på datafilen når online=0.

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
end

fig1=figure;
set(fig1,'Position',[657   257   477   618])
drawnow

% setter skyteknapp til 0, og initialiserer tellevariabel k
JoyMainSwitch=0;
k=0;
%----------------------------------------------------------------------

% starter stoppeklokke for å kunne
% stoppe eksperiment når t>29 sekund.
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
        % Naar k er like stor som antall elementer i datavektpren Tid,
        % simuleres det at bryter paa styrestikke trykkes inn.
        if k==numel(t)
            JoyMainSwitch=1;
        end

        % simulerer EV3-Matlab kommunikasjon i online=false
        pause(0.001)
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
        % Regulatorparametre
        u0 = 0;
        Kp = ..;  
        Ki = ..;   
        Kd = ..; 
        tau_e = ..;  % tidskonstant filtrering av e(k)
        I_max = ..;
        I_min = ..;

        % Initialverdier 
        tau_pos = 0.2;     % Tidskonstant, filtrert vinkelposisjon 
        x1_f(1) = 0;      % Filtrert vinkelposisjon
        x2(1) = 0;        % Vinkelhastighet motor

        % Måling, referansen, reguleringsavvik
        y(1) = x2(1);        % Måling vinkelhastighet
        r(1) = 0;            % Referanse
        e(1) = r(1)-y(1);    % Reguleringsavvik
        e_f(1) = e(1);       % Filtrert reg.avvik for D-ledd

        % Initialverdi PID-regulatorens deler
        P(1) = 0;       % P-del
        I(1) = 0;       % I-del
        D(1) = 0;       % D-del

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

        % Lag kode for PID-regulatoren
        P(k) = ..;
        I(k) = ..;

        % Integratorbegrensing
        % if I(k) > I_max
        %     I(k) = I_max;
        % elseif I(k) < I_min
        %     I(k) = I_min;
        % end

        alfa_e  = ..;  % tidsavhengig alfa
        e_f(k) = ..;
        D(k) = ..;

        % Spiller av varierende lyd ved hvert referanseskifte
        if online && r(k) ~= r(k-1)
            RefVerdiIndeks = RefVerdiIndeks + 1;
            playTone(mylego,RefVerdier(RefVerdiIndeks),0.5)   
        end

        %   para = [Kp, Ki, Kd, I_max, I_min, tau_e, Ts];
        %   [P(k),I(k),D(k),e_f(k)] = MinPID(I(k-1),e_f(k-1),e(k-1:k),para) 
    end
    
    % Beregner totalpådrag
    u_A(k) = u0 + P(k) + I(k) + D(k);

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
        grid
        ylabel('[$^{\circ}$/s]')
        text(t(k),r(k),['$',sprintf('%1.0f',r(k)),'^{\circ}$/s']);
        text(t(k),y(k),['$',sprintf('%1.0f',y(k)),'^{\circ}$/s']);
        title('M{\aa}lt vinkelhastighet og referanse')

        subplot(3,1,2)
        plot(t(1:k),e(1:k),'b-');
        hold on
        plot(t(1:k),e_f(1:k),'r--');
        hold off
        grid
        title('Reguleringsavvik')
        ylabel('[$^{\circ}$/s]')

        subplot(3,1,3)
        plot(t(1:k),P(1:k),'b-');
        hold on
        plot(t(1:k),I(1:k),'b-.');
        plot(t(1:k),D(1:k),'b--');        
        plot(t(1:k),u_A(1:k),'k-');
        yline(100, 'k:','linewidth',2,'HandleVisibility','off')
        yline(-100, 'k:','linewidth',2,'HandleVisibility','off')
        hold off        
        grid
        title('Bidragene P, I, og D og totalp{\aa}draget')
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
legend('$\{r_k\}$','$\{y_k\}$')
subplot(3,1,2)
legend('$\{e_k\}$',['$\{e_{f,k}\}$, $\tau_e$=',num2str(tau_e),' s'])
subplot(3,1,3)
legend(['P-del, $K_p$=',num2str(Kp)],...
    ['I-del,  $K_i$=',num2str(Ki)],...
    ['D-del,  $K_d$=',num2str(Kd),'og $\tau_e$=',num2str(tau_e)],...
    '$\{u_k\}$')

%------------------------------------------------------------------





