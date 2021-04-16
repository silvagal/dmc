clear; clc; close all;

%user settings
ledPin = 'D8';
servoPin = 'D7';
deltaT_blink = 0.1;

%%Matlab support package
port = 'COM5';
board = 'Uno';

%% Create arduino object with the add-on library 
a = arduino(port, board,'Libraries', {'Ultrasonic', 'Servo'});

%% Create ultrasonic object 
sensor = ultrasonic(a,'D4','D5');

%% Create servo object 
servo = servo(a,servoPin);

%% Função de transferência do sistema
s = tf('s');
tfs = 18.62/(s^2);
Ts = 1;                       
tfz = c2d(tfs,Ts)
%% Definir os parâmetros do controlador 
P = 10;                                             %Horizonte de predição
M = 2;                                              %Horizonte de controle
lambda = 1;                                         %ponderação na ação de controle

%% Obter o vetor de resposta ao degrau
g = step(tfz);                                      %resposta ao degrau
N = length(g);                                      %número de amostras de resposta ao degrau

%% Cálculo da matriz G
G = toeplitz(g(1:P),g(1)*eye(1,M));          %cria-se uma matriz diagonal com as amostras em degrau
K = (inv(G' * G + lambda * eye(M)))*(G');    %cálculo da matriz de ganho
K_ganho = K(1,:);                            %pega a primeira linha da matriz de ganho

%% Algoritmo DMC
temp = 100;               %Tempo de funcionamento do sistema
u(10:temp) = 1;
ym(1:temp) = 0;
Nm=length(g);
referencia(10:temp) = 20;
deltaU(1:temp) = 0;
somatorio = 0;
vect_g = zeros(1,Nm);
duf=zeros(1,Nm);

%% Aplicação DMC
for k = 3:temp
    dist = readDistance(sensor)*100;   %leitura do sensor ultrassonico em centímetros
    resp_livre = zeros(1,P);                                 %resposta livre
    ym(k) = dist;                                            %saída forçada
    for kk = 1:P
        for i = 1:(Nm-P)
            vect_g(i) = g(kk + i) - g(i);
        end
        for i = (Nm - P + 1) : Nm
             vect_g(i) = g(Nm) - g(i);
        end
        resp_livre(1:P) = ym(k) + vect_g*duf';
    end
   
    deltaU(k) = K_ganho*(referencia(k) - resp_livre');
    u(k) = u(k-1) + deltaU(k);
    aux = duf(1:end-1);
    duf = [deltaU(k) aux];
    y_angle = mapfun(ym(k), 0, 180, 0, 1)
    %% Send position to servo
    for angle = 0 : 1/180 : y_angle
        writePosition(servo, angle);
    end
end
plot(u);
title('lambda = 0.5')
xlabel('Período de amostragem');
%ylabel('Sinal de Controle u'); 
ylabel('Saída do sistema');
hold on;
plot(referencia,'--');
