# Relatório técnico: aspirador.py

Este relatório descreve o funcionamento do script `aspirador.py`, que implementa uma simulação de um robô aspirador usando PyBullet, com telemetria integrada ao Node-RED.

## Objetivo
Simular a navegação de um robô em um ambiente 3D (com altura fixa), detectando obstáculos com um LiDAR simples e consumindo "sujeiras" espalhadas pelo mapa. O robô evita colisões, realiza manobras de escape quando preso e, ao terminar a limpeza, retorna à base. Durante a execução, envia dados de status ao Node-RED.

## Estrutura do código
- `CONFIGURAÇÕES E NODE-RED`: URLs, nomes de arquivos, variáveis globais e parâmetros de controle.
- `URDF DO ROBÔ`: garantia de existência de um URDF mínimo.
- `AMBIENTE`: criação de paredes, cilindros (obstáculos) e esferas (sujeiras), com filtros de colisão para não interferirem indevidamente.
- `SENSORES`: LiDAR por ray casting com 9 raios de -90° a +90°.
- `EPISÓDIO`: loop principal de controle de navegação, câmera, colisões, telemetria e término.
- `MAIN`: conexão ao PyBullet, construção do ambiente e execução de múltiplos episódios.

## Configurações e Node-RED
- `NODE_RED_URL`: `http://localhost:1880/dados-robo`.
- Variáveis globais: `OBST_IDS`, `OBST_DATA`, `DIRT_IDS`, `DIRT_TOTAL_COUNT`.
- Parâmetros de controle:
  - `LINEAR_SPEED = 0.8`: velocidade linear base.
  - `COLLISION_DISTANCE = 0.5`: limiar de colisão pelo sensor frontal (raio central).
  - `ROTATION_SPEED = 0.05`: ganho base de rotação.
  - `MAX_STEPS = 3600`: número máximo de passos por episódio.
- `send_to_node_red(data_dict)`: envia payload JSON ao Node-RED com timeout de 0,05s.

## URDF do robô
- `ensure_urdf()`: cria um URDF mínimo (caixa) se `mobile_robot_4wd.urdf` não existir, garantindo que sempre haja um corpo para simulação.

## Ambiente
- `build_env()`:
  - Reinicia a simulação e carrega o plano (`plane.urdf`).
  - Cria 4 paredes delimitando a área: x ∈ [-7, 7], y ∈ [-3, 17].
  - Gera 20 cilindros estáticos como obstáculos; registra em `OBST_DATA` (x, y, raio de colisão).
  - Gera 25 "sujeiras" (esferas leves) com margem das paredes e longe de obstáculos; grava `DIRT_TOTAL_COUNT`.
  - Tenta configurar máscara de colisão para reduzir contatos espúrios.

## Sensor LiDAR
- `simple_lidar(robot_id, n=9, ray_len=2.0)`:
  - Calcula matriz de rotação a partir do quat do robô.
  - Para 9 ângulos entre -π/2 e π/2, dispara raios no espaço global.
  - Converte `hitFraction` em distância (`dist = fraction * ray_len`).
  - Retorna lista com distâncias: `left = d[0]`, `center = d[n//2]`, `right = d[-1]`.

## Lógica de navegação
- Estados e variáveis:
  - `home_active`: se verdadeiro, objetivo é retornar à base `(0,0)` após limpar toda sujeira.
  - `detour_active/detour_point`: waypoint temporário para evitar órbitas/prisões perto de paredes.
  - `jitter_cooldown/escape_turn`: manobra de escape após contato.
  - `stuck_counter`: detecta pouca movimentação com obstáculo à frente.
  - `orbit_counter/last_target_dist`: mitiga órbita quando alvo está perto das paredes e o progresso não melhora.
- Controle:
  - Seleção de alvo: sujeira mais próxima; se `detour_active` estiver ligado, segue `detour_point`.
  - `target_bias`: erro angular limitado via `np.clip` aproxima o heading ao alvo.
  - Evita colisões: se `center < COLLISION_DISTANCE`, vira para longe do normal de impacto (quando disponível) ou executa recuo + giro lateral.
  - Parede próxima: adiciona viés para recentrar (evitar bordas).
  - Manobra de escape: recuo e giro com ângulo `escape_turn` quando há contato.
  - Detour: cria waypoint deslocado para dentro ao detectar órbita repetida ou múltiplas colisões a caminho do alvo.
  - Integração: atualiza posição com `step_time = 0.04`, aplica limites de margem das paredes e orienta via novo quaternion.

## Telemetria para Node-RED
- Envio periódico a cada 12 frames com payload:
  - `x`, `y`: posição atual.
  - `dirt_remaining`: quantidade restante de sujeira (`len(DIRT_IDS)`).
  - `dirt_total`: total inicial (`DIRT_TOTAL_COUNT`).
  - `status`: rótulo textual do estado atual ("CLEANING", "HOME", "COLLISION", "STUCK", "FINISHED").
  - `status_msg`: mensagem amigável para dashboard.
- Eventos de término:
  - Ao exceder `MAX_STEPS`: envia `status = FINISHED`, `status_msg = "Limite de tempo atingido."` e encerra.
  - Ao chegar à base quando `home_active`: `status = FINISHED`, `status_msg = "Limpeza e retorno concluídos!"` e encerra.

## Contatos e escape
- Filtra contatos para ignorar sujeiras (`DIRT_IDS`).
- Marca visual rápida com `addUserDebugLine` no ponto de contato.
- Ao colidir: recuo (`back_step`) e cálculo de `escape_turn` usando normal de impacto (quando disponível) ou jitter aleatório.

## Câmera
- Atualiza a câmera com `resetDebugVisualizerCamera` mirando na posição alvo do robô, distância 4.0, yaw 0°, pitch -45°.

## Execução (MAIN)
- `main()`:
  - Conecta primeiro em GUI, senão DIRECT.
  - Configura `pybullet_data` como caminho adicional.
  - Reseta e constrói o ambiente.
  - Executa 3 episódios, imprime a duração (número de passos) de cada um.
  - Desconecta ao final.

## Observações e tuning
- Os limites de distância e ganhos (`COLLISION_DISTANCE`, `ROTATION_SPEED`, `LINEAR_SPEED`) podem ser ajustados conforme desempenho desejado.
- O intervalo de telemetria (12 frames) pode ser aumentado para reduzir carga no Node-RED.
- O mecanismo de `detour_active` ajuda a evitar órbitas perto das paredes e repetidas colisões a caminho de um alvo difícil.
