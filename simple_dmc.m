
%% DMC method applied to a water heater

clear; close all; clc;
%% Processo
z = tf('z');
tfz = (0.2713*z^(-3))/(1 - 0.8351*z^(-1));
%% Definir os parâmetros do controlador 
P = 10;                                      %Horizonte de predição
M = 5;                                       %Horizonte de controle
lambda = 0.5;                                  %ponderação na ação de controle

%% Obter o vetor de resposta ao degrau
g = step(tfz);                               %resposta ao degrau
N = length(g);                               %número de amostras de resposta ao degrau
plot(g);
%% Cálculo da matriz G
G = toeplitz(g(1:P),g(1)*eye(1,M));          %cria-se uma matriz diagonal com as amostras em degrau
K = (inv(G' * G + lambda * eye(M)))*(G')    %cálculo da matriz de ganho
K_ganho = K(1,:);                            %pega a primeira linha da matriz de ganho

%% Algoritmo DMC
temp = 100;
u(10:temp) = 1;
ym(1:temp) = 0;
Nm=length(g);
referencia(10:temp) = 1.6401;

vect_g = zeros(1,Nm);
duf=zeros(1,Nm);
for k = 4:temp
    resp_livre = zeros(1,P);                        %resposta livre
    ym(k) = 0.2713*u(k-3) + 0.8351*ym(k-1); 
    for kk = 1:P
        for i = 1:(Nm-P)
            vect_g(i) = g(kk + i) - g(i);
        end
        for i = (Nm - P + 1) : Nm
             vect_g(i) = g(Nm) - g(i);
        end
        resp_livre(1:P) = ym(k) + vect_g*duf';
    end
   
%     Cálculo do Controle
    deltaU(k) = K_ganho*(referencia(k) - resp_livre');
    u(k) = u(k-1) + deltaU(k);
    aux = duf(1:end-1); 
    duf = [deltaU(k) aux];
end

plot(ym);
title('lambda = 0.5')
xlabel('Período de amostragem');
%ylabel('Sinal de Controle u'); 
ylabel('Saída do sistema');
hold on;
plot(referencia,'--');
