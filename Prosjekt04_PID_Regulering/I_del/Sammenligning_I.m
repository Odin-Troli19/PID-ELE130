
% Fil for å lage figur som sammenligner resultat.
%   a) I utgangspunktet er linje 3 og 4 kommentert 
%   b) Kjør først linje 1 og 2
%   c) Kommenter deretter ut linje 1 og 2, kjør linje 3
%   d) Kommenter deretter ut linje 3, kjør linje 4

%clear;close all                           % linje 1
%load('P04_Ki_XY.mat');farge = 'b';        % linje 2, liten Ki
% load('P04_Ki_YZ.mat');farge = 'r';        % linje 3, litt større Ki
% load('P04_Ki_ZX.mat');farge = [0 0.6 0];  % linje 4, størst Ki

%clear;close all
%load('P04_Ki_001.mat');farge = 'g';
%load('P04_Ki_004.mat');farge = 'b';
%load('P04_Ki_005.mat');farge = 'r';
load('P04_Ki_006.mat');farge = [0 0.6 0];

% Åpne figure nummer 1
figure(1)
set(gcf,'outerposition',[ 569   241   545   738])

subplot(3,1,1)
plot(t,r,'k--');
hold on
plot(t,y,'-','color',farge);
text(22,630,'friksjon')
grid on
ylabel('[$^{\circ}$/s]')
title('M{\aa}lt vinkelhastighet og referanse')

subplot(3,1,2)
plot(t,e,'-','color',farge);
hold on
grid on
title('Reguleringsavvik')
ylabel('[$^{\circ}$/s]')

subplot(3,1,3)
plot(t,u_A,'-','color',farge);
hold on
yline(100, 'k:','linewidth',2,'HandleVisibility','off')
yline(-100, 'k:','linewidth',2,'HandleVisibility','off')
grid on
title('I-bidraget')
xlabel('Tid [sek]')
%legend('$K_i=..$', '$K_i=..$', '$K_i=..$')
legend('$K_i=0.1$', '$K_i=0.2$', '$K_i=0.3$')