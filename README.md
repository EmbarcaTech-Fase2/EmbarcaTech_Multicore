# EmbarcaTech_Multicore
<p align="center">
  <img src="Group 658.png" alt="EmbarcaTech" width="300">
</p>

## Projeto: Utilização de dois núcleos do RP2040 na BitDogLab

![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/-Raspberry_Pi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)
![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=for-the-badge&logo=github&logoColor=white)
![Windows 11](https://img.shields.io/badge/Windows%2011-%230079d5.svg?style=for-the-badge&logo=Windows%2011&logoColor=white)

## Descrição do Projeto

Este projeto demonstra o funcionamento de uso dos dois núcleos do RP2040 (Raspberry Pi Pico) com suporte a PIO (programa `ws2812.pio`) para controlar LEDs WS2812. É uma base para experimentar concorrência entre núcleos, PIO e periféricos típicos do Pico.

## Visão geral

- Arquivo principal: `Multicore.c` — demonstra como iniciar um segundo núcleo, compartilhar dados e usar PIO para dirigir uma fita WS2812.
- Programa PIO: `ws2812.pio` — shader PIO para comunicação com LEDs WS2812.
- Build: CMake + Pico SDK (diretório `build/` contém artefatos gerados quando compilado).

Este projeto é ideal para estudos de: multicore (core0/core1), PIO, controle de LEDs em tempo real e integração com o Pico SDK.

## Hardware (conexões sugeridas)

- Raspberry Pi Pico / Pico W (RP2040)
- Fita WS2812 (data in) -> GPIO 7 (padrão neste repositório)
- VCC da fita -> 5V (ou 3.3V dependendo da fita) — verifique especificações
- GND comum entre Pico e fita LED

Nota: ajuste o pino no código se você usar outro GPIO.

## Como compilar e gravar

Pré-requisitos:
- Pico SDK instalado e configurado
- Toolchain (arm-none-eabi) disponível
- VS Code com extensão Raspberry Pi Pico (opcional)

Build (via linha de comando):

1. Crie diretório de build (se não existir):

```powershell
cmake -S . -B build
cmake --build build
```

2. O binário gerado estará em `build/` (ex.: `Multicore.uf2` ou `Multicore` dependendo da configuração).

Usando as tasks do VS Code (pré-configuradas neste workspace):

- Execute a task "Compile Project" disponível no painel de tasks para chamar o `ninja` do Pico SDK e compilar.
- Para gravar: conecte o Pico em BOOTSEL e copie o arquivo `.uf2` gerado para o dispositivo, ou use a task "Run Project"/"Flash" se você tiver as ferramentas configuradas (ver `tasks.json` no VS Code).

## Estrutura principal do repositório

- `Multicore.c` — código de exemplo que configura e usa o segundo núcleo
- `ws2812.pio` / `ws2812.pio.h` — código PIO e header gerado
- `CMakeLists.txt` — projeto CMake para Pico SDK
- `build/` — saída de build (gerada)
- `lib/` — bibliotecas auxiliares (drivers, helpers) — alguns arquivos de sensores/LEDs podem existir aqui

## O que o exemplo faz

- Inicializa periféricos necessários (PIO, GPIOs)
- Carrega o programa PIO para controlar WS2812
- Inicia um segundo núcleo que pode executar uma tarefa paralela (animações, processamento de dados) enquanto o núcleo principal faz outra coisa
- Mostra como passar dados entre núcleos usando buffers/locks simples


