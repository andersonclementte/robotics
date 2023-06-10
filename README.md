# README

## Atividade 1:
- Fazer download do CoppeliaSim
- Instanciar um robé SCARA (MTB robot)
- Montar um script que Ié a pose de um dummy (pode ser o que vem junto do SCARA)
- Montar um script que controle as juntas do robô para posicioná-las em 4 ângulos arbitrários.

## Atividade 2

Fazer cinemática direta do SCARA (figura em anexo), considerando
offset = 0,1 m; 11 = 0.475 m, 12 = 0,4m e 0 m<=d_4<=0.1 m. Entregar um documento em PDF mostrando:

- Atabela de parémetros DH
- Calculo das matrizes de transformação até o efetuador
- Codigo da função de cinemática direta fkine
- Print do objeto SerialLink gerado na robotics toolbox (link em anexo)
- Comparação do resultado da função fkine implementada e a função fkine dentro do objeto SerialLink gerado para as seguintes configurações:

(a) theta_1 = 0; theta_2 = 0; theta_3 = 0;d_4=0
(b) theta_1 = pi/2; theta_2 = -pi/2; theta_3 = 0;d_4=0
(c) theta_t = pi/2; theta_2 = -pi/2; theta_3 = 0; d_4= 0.05
- Enviar comandos para o CoppeliaSim

## Lista 3

Usando o mesmo manipulador SCARA da atividade de cinemática direta, implementar uma função de cinemática inversa que passe o vetor (x, y, z, phi) e retorne os valores das juntas (theta1, theta2, theta3, d4). Caso não seja possível chegar na pose desejada, deve mandar um erro, ou aviso.  
Mande o código da função e envie prints de resultados da função com as seguintes entradas:  
- (0.2, 0.1, -0.015, pi/4)  
- (0.5, 0.1, -0.015, pi/4)  
- (0.15, 0.15, 0, pi)  
DICA: Pode usar o exemplo do manipulador planar mostrado na aula Cinemática Inversa 01 (Aula 10), de forma que o d4 é computado a partir do valor de z.

## Projeto AB2

1 - Escolher ou montar um robô manipulador com no mínimo 4 juntas (exceto o SCARA), onde ao menos 3 juntas de revolução. (DICA: Tem o urdf do DENSO no link de repositório do github abaixo, caminho: "vp6242_description/urdf/vp6242.urdf". DICA2: Capítulo 18 do livro Programming Robots with ROS mostra como montar o urdf do CougarBot). O resultado dessa primeira etapa é uma descrição do robô, preferencialmente no formato URDF (ver link).  
2 - Configurar control-loop para cada junta no Coppelia, senão o robô vai cair.  
3 - Montar tabela de parâmetros de Denavit-Hartenberg (DH)  
4 - Computar cinemática direta  
5 - Implementar algoritmo de cinemática diferencial (DICA: Seção 3.2 do livro Robotics do Siciliano)  
6 - Plotar erros (como na atividade do SCARA)

## Controle Cinemático SCARA

Valores de elos do SCARA: l1 = 0.475 e l2 = 0.4.  
Na cena scara_scene.ttt temos o manipulador SCARA e um sistema de referência dummy /reference. O objetivo da atividade é realizar o controle cinemático (usando resolved_rate) para fazer com que o efetuador do SCARA chegue no /reference.  
Roteiro  
1 - Implementar controle cinemático (DICA: Na extração dos parâmetros, considerar apenas yaw -> alpha, já que roll é sempre pi e pitch é sempre 0. Ver link anexo)  
2 - Plotar valor de erro (um subplot) e valores das 4 juntas.
