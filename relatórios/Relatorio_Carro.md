# Relatório técnico: carro.py

Este relatório descreve o funcionamento do script `carro.py`, que simula um robô 4WD percorrendo uma pista com paredes e obstáculos, controlado por um balanço lateral tipo PID, com telemetria enviada ao Node-RED e geração de logs/plots.

## Objetivo
Percorrer uma pista retilínea de comprimento definido, evitando obstáculos usando um conjunto de raios (LiDAR simplificado) e controlando a direção por diferença de distâncias laterais (balanceamento), até cruzar a linha de chegada.

## Estrutura do código
- Configurações e constantes (velocidades, thresholds, PID, URL Node-RED).
- Geração do URDF 4WD (`create_mobile_robot_4wd_urdf`).
- Envio de telemetria (`send_to_node_red`).
- Construção da linha de chegada (`create_finish_line`).
- Ambiente (pista, paredes laterais e obstáculos aleatórios) (`create_env`).
- Sensores por ray casting (`get_sensors`).
- Controle de motores (balanceamento + desvio) (`calculate_motor_speeds`).
- Funções auxiliares de juntas e `main` (loop de simulação, logs, câmera, telemetria e término).

## Configurações principais
- `NODE_RED_URL`: `http://localhost:1880/dados-robo`.
- Velocidades e limites:
  - `BASE_SPEED = 20.0`: velocidade base de cada lado.
  - `MAX_SPEED_MOTOR = 30.0`: saturação das velocidades de motor.
  - `OBSTACLE_THRESHOLD = 2.0`: distância mínima no raio central para acionar desvio de obstáculo.
  - `WALL_BALANCE_THRESHOLD = 2.5`: valor de referência para manter equilíbrio lateral (usado no controle por erro entre lados).
- PID-like balance:
  - `KP_BALANCE = 10.0`, `KD_BALANCE = 1.0`.
  - `LAST_ERROR`: armazena erro anterior para derivada.
- Pista e raios:
  - `TRACK_LENGTH = 40.0`.
  - `RAY_LENGTH = 5.0`.

## URDF do robô 4WD
- `create_mobile_robot_4wd_urdf(filename)` gera um URDF simples com:
  - `base_link` como caixa (visual, colisão e inércia).
  - 4 rodas (`fl_wheel`, `fr_wheel`, `rl_wheel`, `rr_wheel`) conectadas por juntas contínuas, com eixos alinhados para controle por velocidade.

## Telemetria para Node-RED
- `send_to_node_red(data_dict)`: POST assíncrono tolerante a falhas.
- No loop principal (`main`):
  - A cada 12 frames: envia `{ "y": pos[1], "status": "running" }`.
  - Ao completar o percurso (quando `pos[1] > TRACK_LENGTH - 2`): envia `{ "status": "completed" }`.

## Linha de chegada
- `create_finish_line(y_pos, track_width)`: desenha um padrão quadriculado (branco/preto) na posição de chegada, sem colisão, para visualização.

## Ambiente
- `create_env(length)`:
  - Carrega `plane.urdf`.
  - Cria paredes laterais ao longo do comprimento.
  - Gera 10 cilindros como obstáculos com posições aleatórias controladas por uma grade de Y e faixa segura de X.
  - Chama `create_finish_line(length-1, track_width)` para a linha de chegada.

## Sensores (ray casting)
- `get_sensors(robot_id)`:
  - Obtém orientação do robô e constrói vetores `fwd` e `left` a partir da matriz de rotação.
  - Define 5 direções: frontal esquerda (fwd+left*1.5), sub-esquerda, frente, sub-direita, frontal direita.
  - Dispara raios de `start = pos + [0,0,0.2]` até `start + n*RAY_LENGTH` e retorna distâncias por `hitFraction * RAY_LENGTH`.
  - Resultado: tupla `(d_fl, d_l, d_c, d_r, d_fr)`.

## Controle (balanceamento + desvio)
- `calculate_motor_speeds(sensors, dt)`:
  - Desvio de obstáculo: se `d_c < OBSTACLE_THRESHOLD`, reduz base e aplica rotação proporcional comparando mínimos dos sensores laterais (vira para o lado com mais espaço).
  - Balanceamento lateral tipo PID: erro `e = d_r - d_l`, derivada `de = (e - LAST_ERROR)/dt`, saída `steer = KP_BALANCE*e + KD_BALANCE*de`.
  - Velocidades: `vL = BASE_SPEED + steer`, `vR = BASE_SPEED - steer`, saturadas em `[-MAX_SPEED_MOTOR, MAX_SPEED_MOTOR]`.

## Main (execução da simulação)
- Conecta GUI, desativa HUD, define gravidade e timestep (`dt=1/240`).
- Cria ambiente e carrega URDF do robô com orientação inicial (`yaw≈1.57`).
- Recupera índices das juntas (`fl_joint`, `fr_joint`, `rl_joint`, `rr_joint`).
- Loop:
  - Condição de chegada: `pos[1] > TRACK_LENGTH - 2` marca sucesso, envia telemetria e encerra.
  - Lê sensores, calcula velocidades, aplica controle de velocidade (`VELOCITY_CONTROL`) a todas as rodas com `MAX_FORCE`.
  - Faz `stepSimulation`.
  - Loga métricas: posição, tempo, distâncias (L/C/R) e velocidades (L/R).
  - Atualiza câmera a cada ~1s mirando `pos`.
  - Envia telemetria a cada 12 frames com `y` e `status`.
- Finalização:
  - Desconecta do PyBullet.
  - Se houver logs, salva CSV `gauntlet_otimizado.csv`, plota trajetória com `matplotlib`.

## Observações e tuning
- Ajuste `OBSTACLE_THRESHOLD` para tornar o desvio mais/menos agressivo.
- `KP_BALANCE` e `KD_BALANCE` regulam o quão rápido o robô corrige desalinhamento com as paredes.
- `RAY_LENGTH` impacta o alcance dos sensores; aumentar pode antecipar desvios.
- A telemetria está mínima (apenas `y` e `status`) para reduzir overhead.
