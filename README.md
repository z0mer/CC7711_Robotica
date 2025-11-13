# 🤖 Projeto Webots: Busca Dinâmica de Caixa Leve

Este repositório contém a solução para o projeto da disciplina de Robótica (CC7711), desenvolvido no simulador **Webots**.

## 👥 Integrantes do Grupo
* **Anna Carolina Zomer** (RA: 22.224.017-8)
* **Humberto Pellegrini** (RA: 22.224.019-4)

## 🎯 Objetivo
O objetivo deste projeto é controlar um robô (e-puck) para que ele seja capaz de:
1.  **Identificar dinamicamente** qual é a caixa de **menor massa** no ambiente (dentre várias caixas disponíveis).
2.  Navegar até a caixa alvo desviando de obstáculos.
3.  Ao alcançar a caixa (distância < 0.1m), parar a busca e iniciar um **movimento de giro sobre o próprio eixo** para sinalizar o sucesso.

## 📹 Demonstração (Vídeo)
Confira o funcionamento do robô encontrando a caixa leve e executando a tarefa:

### [CLIQUE AQUI PARA ASSISTIR AO VÍDEO DO PROJETO](https://github.com/z0mer/CC7711_Robotica/blob/main/RobozinhoFuncionando.mp4)

## ⚙️ Como funciona o Código
O algoritmo foi desenvolvido em **C** e opera através de uma máquina de estados:

* **Seleção do Alvo:** O robô varre os objetos do cenário (`CAIXA01` a `CAIXA64`). Ele prioriza objetos que tenham o campo `mass` definido explicitamente. Caso contrário, utiliza o volume como critério de desempate.
* **Navegação (Busca):** Utiliza controle proporcional para ajustar a direção do robô em direção ao alvo selecionado.
* **Desvio de Obstáculos:** Utiliza sensores de proximidade (`ps0` a `ps7`) para detectar colisões iminentes e realizar manobras de desvio ou evasão (ré).
* **Estado Final:** Ao chegar no alvo, entra no estado `ROTACAO_FINAL`, onde gira indefinidamente.

## 📂 Estrutura do Repositório
* `/controllers`: Contém o código fonte em C (`.c`) do controlador do robô.
* `/worlds`: Arquivo do mundo (`.wbt`) utilizado para os testes.

## 🔗 Referências
Este projeto utilizou como base o material de apoio e o mundo de exemplo disponibilizados no repositório:
* [rdestro/CC7711-WeBots](https://github.com/rdestro/CC7711-WeBots)
