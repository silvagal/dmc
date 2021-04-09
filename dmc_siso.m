clear; clc; close all;

%user settings
ledPin = 'D8';
servoPin = 'D7';
deltaT_blink = 0.1;

%Matlab support package
port = 'COM5';
board = 'Uno';

% Create arduino object with the add-on library 
a = arduino(port, board,'Libraries', {'Ultrasonic', 'Servo'});

% Create ultrasonic object 
sensor = ultrasonic(a,'D4','D5');

% Create servo object 
servo = servo(a,servoPin);

%% Processo
z = tf('z');
tfz = (0.2713 *(z^(-3)))/(1 - 0.8351*(z^-1));

%% Definir os parâmetros do controlador 
P = 10;        %Horizonte de predição
M = 5;         %Horizonte de controle
lambda = 1;    %ponderação na ação de controle

%% Obter o vetor de resposta ao degrau
g = step(tfz);    %resposta ao degrau
N = length(g);    %número de amostras de resposta ao degrau

%% Cálculo da matriz G
G = toeplitz(g(1:P),g(1)*eye(1,M));          %cria-se uma matriz diagonal com as amostras em degrau
K = (inv(G' * G + lambda * eye(M)))*(G');    %cálculo da matriz de ganho
K_ganho = K(1,:);                            %pega a primeira linha da matriz de ganho

%% Algoritmo DMC
u(10:30) = 1;
ym(1:30) = 0;
referencia(1:30) = 1.6401;
deltaU(1:30) = 0;
duf(1:30) = 0;
somatorio = 0;
Nm=30;
% Obtain sensed distance
dist = readDistance(sensor)

for k = 4:1000
    resp_livre = zeros(1,P);                 %resposta livre
    ym(k) = 0.2713*u(k-3) - 0.8351*ym(k-1);  %saída do sistema
    for kk = 1:P
        for i = 1:Nm
            vect_g(i) = g(kk + i) - g(i);
        end
        resp_livre(1:P) = ym(k) + vect_g*duf';
    end
   
    % Cálculo do Controle
    deltaU(k) = K*(referencia(k) - resp_livre');
    u(k) = u(k-1) + deltaU(k);
    aux = duf(1:end-1); 
    duf = [deltaU(k) aux];
        %Send position to servo
    %loop that sends the control response to the servo motor 
    for angle = 0 : 1/180 : u(k)
        writePosition(servo, angle);
    end
end
