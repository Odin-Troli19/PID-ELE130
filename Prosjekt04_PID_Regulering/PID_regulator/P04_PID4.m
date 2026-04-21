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

clear; close all
online = true;     
plotting = false;
filename = 'P04_PID2.mat';

if online
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);

    % motorer
    motorA = motor(mylego,'A');
    motorA.resetRotation;
else
    load(filename)
    online = false;
    plotting = false;
end

fig1 = figure;
set(fig1,'Position',[657   257   477   618])
drawnow

JoyMainSwitch = 0;
k = 0;

duration = tic;

while ~JoyMainSwitch

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                       GET TIME AND MEASUREMENT

    k = k + 1;

    if online
        if k == 1
            playTone(mylego,500,0.1)
            tic
            t(1) = 0;
        else
            t(k) = toc;
        end

        VinkelPosMotorA(k) = double(motorA.readRotation);

        [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);
    else
        if k == numel(t)
            JoyMainSwitch = 1;
        end
        pause(0.001)
    end
    %--------------------------------------------------------------



    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER

    if toc(duration) > 29
        JoyMainSwitch = 1;
    end

    x1(k) = VinkelPosMotorA(k);

    if k == 1
        % Regulatorparametre
        u0 = 0;
        Kp = 0.08;
        Ki = 0.06;
        Kd = 0.010;
        tau_e = 0.01;
        I_max = 100;
        I_min = -100;

        % Referanse-verdier og tidspunkt
        tidspunkt = [0, 2, 6, 10, 14, 18];
        RefVerdier = [0 300 600 900 1200 500];
        RefVerdiIndeks = 1;

        % Initialverdier
        tau_pos = 0.2;
        x1_f(1) = 0;
        x2(1) = 0;

        % Måling, referanse og reguleringsavvik
        y(1) = x2(1);
        r(1) = 0;
        e(1) = r(1) - y(1);
        e_f(1) = e(1);

        % Initialverdier regulatorledd
        P(1) = 0;
        I(1) = 0;
        D(1) = 0;

    else
        % Tidsskritt
        Ts = t(k) - t(k-1);

        % Filtrert vinkelposisjon
        alfa_pos = 1 - exp(-Ts/tau_pos);
        x1_f(k) = (1 - alfa_pos)*x1_f(k-1) + alfa_pos*x1(k);

        % Vinkelhastighet
        x2(k) = (x1_f(k) - x1_f(k-1))/Ts;

        % Måling
        y(k) = x2(k);

        % Referanse
        r(k) = interp1(tidspunkt, RefVerdier, t(k), 'previous', 'extrap');

        % Reguleringsavvik
        e(k) = r(k) - y(k);

        % P-ledd
        P(k) = Kp*e(k);

        % I-ledd
        I(k) = I(k-1) + 0.5*Ki*(e(k-1) + e(k))*Ts;

        % Integratorbegrensing
        if I(k) > I_max
            I(k) = I_max;
        elseif I(k) < I_min
            I(k) = I_min;
        end

        % Filtrering av avvik til D-leddet
        alfa_e = 1 - exp(-Ts/tau_e);
        e_f(k) = (1 - alfa_e)*e_f(k-1) + alfa_e*e(k);

        % D-ledd
        D(k) = Kd*(e_f(k) - e_f(k-1))/Ts;

        % Lyd ved referanseskift
        if online && r(k) ~= r(k-1)
            RefVerdiIndeks = RefVerdiIndeks + 1;
            playTone(mylego,RefVerdier(RefVerdiIndeks),0.5)
        end
    end

    % Totalpådrag som skal plottes
    u_A(k) = u0 + P(k) + I(k) + D(k);

    % Pådrag som faktisk sendes til motoren
    u_motor = max(min(u_A(k),100),-100);

    if online
        motorA.Speed = u_motor;
        start(motorA)
    end

    %--------------------------------------------------------------



    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA

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
    ['I-del, $K_i$=',num2str(Ki)],...
    ['D-del, $K_d$=',num2str(Kd),' og $\tau_e$=',num2str(tau_e)],...
    '$\{u_k\}$')

save(filename)

%------------------------------------------------------------------