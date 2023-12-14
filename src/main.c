#undef F_CPU
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Typedefs, por conforto.
typedef short int16_t;
typedef int int32_t;
typedef long int64_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long uint64_t;

// Velocidade do som para cálculo da distância.
#define VELOCIDADE_SOM 343U // Em m/s.
#define DIST_MINIMA 25U // Em cm.

// Acionamento dos motores.
#define PARADO      0b00000000
#define FRENTE      0b00011000
#define TRAS        0b00000110
#define HORARIO     0b00001010
#define ANTIHORARIO 0b00010100
#define MUDAR_DIRECAO(dir) PORTC = (PORTC & 0b11100001) | dir // Muda a direção do carrinho.
#define ESTA_MOVENDO(dir) ((PORTC & 0b00011110) == dir) // Verifica a direção atual do movimento do carrinho.
#define DELAY_GIRO_MS 50 // Tempo de duração dos comandos HORARIO e ANTIHORARIO

// Largura de pulso para cada porcentagem
#define OCR_70_PERCENT 178
#define OCR_80_PERCENT 204
#define OCR_100_PERCENT 255

// Constantes somadas às larguras de pulso para calibração adequada das rodas.
#define OCR_RODA_ESQ_BIAS 0
#define OCR_RODA_DIR_BIAS 12

// Macro para definir largura de pulso.
#define MUDAR_VELOCIDADE(power) \
  OCR2A = power - OCR_RODA_ESQ_BIAS;\
  OCR2B = power - OCR_RODA_DIR_BIAS

// Comandos de entrada.
#define COM_FRENTE      'w'
#define ERR_OBSTACULO   'X'
#define COM_TRAS        's'
#define COM_HORARIO     'd'
#define COM_ANTIHORARIO 'a'
#define COM_PARADO      'q'
#define COM_70          '7'
#define COM_80          '8'
#define COM_100         '0'
#define COM_DIST        'e'

// Mensagens.
char MSG_FRENTE[]       = "FRENTE\n";
char MSG_OBSTACULO[]    = "OBSTACULO\n";
char MSG_TRAS[]         = "TRAS\n";
char MSG_ANTIHORARIO[]  = "ANTI-HORARIO\n";
char MSG_HORARIO[]      = "HORARIO\n";
char MSG_PARADO[]       = "PARADO\n";
char MSG_70[]           = "Velocidade 70%\n";
char MSG_80[]           = "Velocidade 80%\n";
char MSG_100[]          = "Velocidade 100%\n";
char MSG_DIST[]         = "DDDcm\n";

// Variáveis de controle de comando.
volatile uint8_t flag_comando = 0; // Flag: 1 se for recebido um comando, 0 caso contrário.
volatile char comando = COM_PARADO; // Último comando recebido.

// Controle de mensagem.
volatile char *msg = MSG_PARADO; // Buffer com a mensagem a ser enviada pela UART.
volatile uint32_t msg_idx = 0; // Índice do último caractére enviado.
volatile uint32_t contador_msg = 0; // Contagem de tempo para o envio da mensagem.

// Controle do medidor.
volatile uint8_t flag_echo = 0; // Flag: 1 se houver uma nova medida de echo, 0 caso contrário.
volatile uint16_t distancia = 0; // Distância medida pelo sensor, em centímetros.
volatile uint64_t contador_medida = 0; // Contagem de tempo de duração do sinal ECHO.
volatile uint32_t contador_dist = 0; // Contagem de tempo para medida de distancia.

// Controle do LED.
volatile uint64_t periodo_led = 0; // Período do piscar do LED em função da distância. Calculado periodicamente.
volatile uint64_t contador_led = 0; // Contagem de tempo do piscar do LED.

/* CONFIGURAÇÃO PWM
=============================================
Período do PWM
  TPWM 	= prescaler/fclk * 256
        = 32/16e6 * 256
        = 0.512ms (1.95kHz)

Largura do Pulso
  Pwidth	= prescaler/fclk * (OCRA/B + 1)

Ajuste da porcentagem alterando o valor de OCRA/B
  (OCRA/B+1) = porcentagem*256
  70%  => OCRA/B = 178
  80%  => OCRA/B = 204
  100% => OCRA/B = 255

Fast PWM com TOP = 0xFF   (WGM = 011)
Modo não invertido        (COM2A/B = 10)
  TCCR2A = [COM2A[1:0] COM2B[1:0] - - WGM2[1:0]]
  TCCR2A = 0b10100011;

Prescaler 32              (CS2 = 0b011)
Sem Force Output Compare  (FOC2A/B = 0)
  TCCR2B = [FOC2A FOC2B - - WGM22 CS2[2:0]]
  TCCR2B = 0b00000011;
=============================================
*/
/* CONFIGURAÇÃO TEMPORIZADOR0
=============================================
Tempo entre interrupções

Tint 	= prescaler/fclk * (OCR0A + 1)
        = 8/16e6 * (99 + 1)
        = 50 us

Comparação desconectada   (COM0A/B = 00)
Modo CTC                  (WGM = 010)
  TCCR0A = [COM0A1 COM0A0 COM0B1 COM0B0 - - WGM01 WGM00]
  TCCR0A = 0b00000010;

Sem force output compare  (FOC0A/B = 0)
Prescaler igual a 8       (CS0(2:0) = 010)
  TCCR0B = [FOC0A FOC0B - - WGM02 CS02 CS01 CS00]
  TCCR0B = 0b00000010;

Output Compare A Int      (OCIE0A = 1)
  TIMSK0 = [- - - - - OCIE0B OCIE0A TOIE0]
  TIMSK0 = 0b010;

OCR0A = 99;
=============================================
*/
/* CONFIGURAÇÃO UART
=============================================
- double-speed desativado
- multi-processador desabilitado;
- 8 bits por frame;
- Modo assíncrono;
- Sem bits de paridade;
- 1 bit de parada;
- Baud rate 9.600 bps.

Modo double-speed e multiprocessamento desabilitados
UCSR0A = [x x x x x x U2Xn MPCMn];
  UCSR0A = 0

Interrupções e habilitando canais
UCSR0B = [RXCIE TXCIE UDRIE RXEN TXEN UCSZ2 x x]
  UCSR0B = 0b11011000

Seleção de modo, paridade e bit de parada
UCSR0C = [UMSEL1 UMSEL0 UPM1 UMP0 USBS UCSZ1 UCSZ0 x]
  UCSR0C = 0b00000110

Baud Rate = 9600 = Fosc/16*(UBRR0+1)
UBRR0 = 103
=============================================
*/

// Função de execução de comando.
void executarComando();

// Interrupção de PIN CHANGE 2.
ISR(PCINT2_vect) {
  // Acionada quando é detectada uma mudança no sinal ECHO do sensor de distância.
  if (PIND & 0b00000100) contador_medida = 0; // Caso ECHO seja 1, reinicia a contagem de tempo do echo antes da medida.
  else flag_echo = 1; // Caso echo seja 0, habilita a flag de echo.
}

// Interrupção do receptor UART.
ISR(USART_RX_vect) {
  if (!flag_comando) {
    // Ativa a flag e salva o comando.
    flag_comando = 1;
    comando = UDR0;
  }
}

// Interrupção do transmissor UART.
ISR(USART_TX_vect) {
  if (msg[++msg_idx] != '\0') UDR0 = msg[msg_idx];
  else msg_idx = 0;
}

// Interrução do temporizador. Ocorre a cada 50us.
ISR(TIMER0_COMPA_vect) {
  if (!flag_echo) ++contador_medida; // Incrementa a contagem de tempo do echo quando estiver sendo medido.
  PORTC &= ~(0b00000001); // Desabilita TRIG.

  // Se 1s tiver se passado desde a última mensagem, reenvie a mensagem atual e reinicie a contagem.
  if (++contador_msg >= 20000U) { // 50us * 20000 = 1000000us = 1s
    UDR0 = msg[msg_idx];
    contador_msg = 0; // Reinicia a contagem de tempo da mensagem.
  }

  // Aciona TRIG do sensor de distância a cada 200ms.
  if (++contador_dist >= 4000U) { // 50us * 4000 = 200000us = 200ms
    PORTC |= 0b00000001;
    contador_dist = 0; // Reinicia contagem de tempo da medida de distância.
  }

  // Pisca o LED de acordo com a medida de distância.
  if (++contador_led >= periodo_led) {
    /* Relação distância-frequência:
      25cm -> 30hz
      45cm -> 10hz
      75cm -> 5hz
      315cm -> 1hz 
    */
    if (distancia <= DIST_MINIMA) PORTC |= 0b00100000; // Se a distância medida for menor que a distância mínima, mantenha o LED aceso.
    else PORTC ^= 0b00100000; // Caso contrário, inverta o sinal do LED.
    contador_led = 0; // Reinicia a contagem de tempo do LED.
  }
}

void config() {
  cli();

  /* CONFIGURAÇÕES GPIO
  DDRC:
  - [5]: LED 
  - [4:1]: Sinal do motor
  - [0]: TRIG (sensor de distância)
  DDRB[3] (OC2A): Ativação do motor (roda esquerda)
  DDRD[3] (OC2B): Ativação do motor (roda direita)
  */
  DDRC |= 0b00111111;
  DDRB |= 0b00001000;
  DDRD |= 0b00001000;
  MUDAR_DIRECAO(PARADO);

  // Configurações da UART.
  UCSR0A = 0b00000000;
  UCSR0B = 0b11011000;
  UCSR0C = 0b00000110;
  UBRR0L = 103;
  UBRR0H = 0;

  // Temporização do sistema com Timer/Counter0 em modo CTC (50us entre interrupções).
  OCR0A = 99;
  TIMSK0 = 0b00000010;
  TCCR0B = 0b00000010;
  TCCR0A = 0b00000010;

  // Configuração do PWM com Timer/Counter2.
  TCCR2B = 0b00000011;
  TCCR2A = 0b10100011;
  MUDAR_VELOCIDADE(OCR_70_PERCENT);

  // Configuração da interrupção PCINT2
  PCICR = 0b00000100; // Habilita máscara do pino PCINT18. 
  PCMSK2 = 0b00000100; // Habilita interrupção PCINT2.

  sei();
}

int main() {

  config();

  while (1) {
    if (flag_echo) {
      // Executar se houver uma nova medida de echo.
      distancia = (VELOCIDADE_SOM * contador_medida * 50) / 20000; // Calcula distância.
      flag_echo = 0; // Desabilita flag do echo.
      threshold = (distancia - 15) * (200) / 3; // Calcula o período do piscar do LED.

      // Coloca a medida de distância na string MSG_DIST.
      MSG_DIST[0] = (char)((distancia / 100) % 10 + 0x30); // Centenas.
      MSG_DIST[1] = (char)((distancia / 10) % 10 + 0x30); // Dezenas.
      MSG_DIST[2] = (char)(distancia % 10 + 0x30); // Unidades.
    }

    // Se estiver andando para frente e for detectado um obstáculo próximo, mude o comando para ERR_OBSTACULO.
    if (ESTA_MOVENDO(FRENTE) && distancia <= DIST_MINIMA && comando != ERR_OBSTACULO) {
      comando = ERR_OBSTACULO;
      flag_comando = 1; // Aciona processamento de comandos.
    }

    // Verifica se algum comando foi recebido.
    if (flag_comando) {
      flag_comando = 0;
      executarComando();
    }
  }

  return 0;
}

void executarComando() {
  switch (comando) {
    case COM_FRENTE: // COMANDO: Para frente.
      MUDAR_DIRECAO(FRENTE);
      msg = MSG_FRENTE;
      break;
    case COM_ANTIHORARIO: // COMANDO: Giro antihorário.
      MUDAR_DIRECAO(ANTIHORARIO);
      _delay_ms(DELAY_GIRO_MS);
      MUDAR_DIRECAO(PARADO);
      msg = MSG_ANTIHORARIO;
      break;
    case COM_TRAS: // COMANDO: Para trás.
      MUDAR_DIRECAO(TRAS);
      msg = MSG_TRAS;
      break;
    case COM_HORARIO: // COMANDO: Giro horário.
      MUDAR_DIRECAO(HORARIO);
      _delay_ms(DELAY_GIRO_MS);
      MUDAR_DIRECAO(PARADO);
      msg = MSG_HORARIO;
      break;
    case COM_PARADO: // COMANDO: Parar movimento.
      MUDAR_DIRECAO(PARADO);
      msg = MSG_PARADO;
      break;
    case COM_DIST: // COMANDO: Mostrar distância.
      msg = MSG_DIST;
      break;
    case COM_70: // COMANDO: Usar 70% de velocidade.
      MUDAR_VELOCIDADE(OCR_70_PERCENT);
      msg = MSG_70;
      break;
    case COM_80: // COMANDO: Usar 80% de velocidade.
      MUDAR_VELOCIDADE(OCR_80_PERCENT);
      msg = MSG_80;
      break;
    case COM_100: // COMANDO: Usar 100% de velocidade.
      MUDAR_VELOCIDADE(OCR_100_PERCENT);
      msg = MSG_100;
      break;
    case ERR_OBSTACULO: // ERRO: Obstáculo em frente! Parar movimento.
      MUDAR_DIRECAO(PARADO);
      msg = MSG_OBSTACULO;
      break;
    default: // Em caso de comando inválido, retornar.
      return;
  }
  // Reinicia a mensagem e o zera a contagem de tempo desde a última mensagem.
  msg_idx = 0;
  UDR0 = msg[msg_idx];
  contador_msg = 0;
}