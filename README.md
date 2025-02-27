# Sistema de Irrigação Automatizado

Este projeto implementa um sistema de irrigação automatizado utilizando um Raspberry Pi Pico. O sistema monitora os níveis de umidade do solo e a iluminação ambiente por meio de potenciômetros e aciona os componentes necessários para manter as condições ideais de irrigação.

## Funcionalidades
- Leitura dos níveis de umidade e iluminação utilizando potenciômetros.
- Exibição das informações em um display OLED SSD1306.
- Indicação de status utilizando LEDs RGB e um buzzer.
- Controle e interação por meio de botões.

## Hardware Utilizado
- Raspberry Pi Pico
- Display OLED SSD1306
- LEDs RGB WS2812
- Buzzer
- Potenciômetros (simulando sensores de umidade e iluminação)
- Botões para controle

## Dependências
Para compilar e rodar o código, é necessário instalar:
- SDK do Raspberry Pi Pico
- Biblioteca para controle de LEDs WS2812
- Biblioteca SSD1306 para o display OLED
- Ferramentas de compilação C/C++

## Como Compilar e Executar
1. Clone este repositório.
2. Configure o ambiente do Raspberry Pi Pico.
3. Compile o código com CMake.
4. Envie o arquivo .uf2 para a placa via USB.

## Espaço para Demonstração em Vídeo
*https://youtu.be/sH9JOxKk1fg*

## Autor
*Victor Gabriel Guimarães Lopes*
