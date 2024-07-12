# Projeto de Veículo Guiado Automaticamente (AGV)

### Circuitos Microprocessados - 2024/01 - Unisinos

## 1. Motivação

Os Veículos Guiados Automaticamente (AGVs) estão sendo cada vez mais utilizados em ambientes industriais e grandes armazéns. Um exemplo primordial de sua aplicação é a operação dos robôs do armazém da Amazon, como mostrado na primeira parte do seguinte vídeo: [Robôs do Armazém da Amazon](https://www.youtube.com/watch?v=1-KS0-xICks).

## 2. Objetivo

O objetivo deste projeto é desenvolver um AGV utilizando os kits de construção de robôs disponíveis no laboratório. O AGV deve ser capaz de navegar ao longo de linhas marcadas no chão e seguir um caminho pré-determinado.

## 3. Tarefas

- O robô deve se mover em uma grade marcada no chão. Um exemplo da grade é mostrado na Figura 1.
- O robô deve seguir um caminho específico que será desenhado aleatoriamente. Cada grupo terá um caminho diferente para executar.
- Os robôs podem e devem navegar na grade SIMULTANEAMENTE.
- O robô deve ter sistemas anti-colisão.
- O robô deve ser capaz de tomar decisões para alcançar seu objetivo.
- Barreiras artificiais serão usadas para simular obstáculos ou outros robôs.

A grade mostrada na Figura 1 representa os possíveis movimentos para o AGV. Cada célula na grade representa uma posição onde o AGV pode se mover ao longo das linhas marcadas no chão.

![Grade](/files/grid.png)

**Figura 1:** Grade com os possíveis movimentos para o AGV.

## 4. Avaliação

O robô deve realizar todas as tarefas listadas no Capítulo 3. A avaliação será dividida em 4 partes:

- Movimento ao longo das linhas -> 1 ponto
- Execução do caminho pré-determinado -> 3 pontos
- Sistema anti-colisão -> 2 pontos
- Sistema de tomada de decisão -> 2 pontos
- O robô realiza todas as tarefas sem interferência humana -> 2 pontos

# Fluxogramas

## Fluxograma 1: Execução do caminho pré-determinado

![Fluxograma 1](/files/principal.svg)

Este fluxograma descreve o algoritmo para o AGV executar o caminho pré-determinado. Ele inclui etapas para ler o caminho, calcular o próximo movimento e ajustar a posição do AGV de acordo.

## Fluxograma 3: Sistema anti-colisão

![Fluxograma 3](/files/anti-colisao.svg)

Este fluxograma demonstra o algoritmo para o sistema anti-colisão do AGV. Ele inclui etapas para detectar obstáculos, calcular caminhos alternativos e evitar colisões com outros robôs ou obstáculos.

## Fluxograma 4: Sistema de tomada de decisão

![Fluxograma 4](/files/tomadadecisao.svg)

Este fluxograma apresenta o algoritmo para o sistema de tomada de decisão do AGV. Ele inclui etapas para analisar a situação atual, avaliar possíveis ações e selecionar a decisão ótima para alcançar o objetivo do AGV.
