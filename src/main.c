#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define Vsom 343

// Acionamento dos motores
#define PARADO 0b00000000
#define FRENTE 0b00011000
#define TRAS 0b00000110
#define HORARIO 0b00001010
#define ANTIHORARIO 0b00010100

// Largura de pulso para cada porcentagem
#define OCR_70_PERCENT 178
#define OCR_80_PERCENT 204
#define OCR_100_PERCENT 255

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
  TCCR2A = [COM2A(1:0) COM2B(1:0) - - WGM2(1:0)]
  TCCR2A = 0b10100011;

Prescaler 32              (CS2 = 0b011)
Sem Force Output Compare  (FOC2A/B = 0)
  TCCR2B = [FOC2A FOC2B - - WGM22 CS2(2:0)]
  TCCR2B = 0b11;
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

/* UART CONFIG
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

// Mensagens
char msg_frente[] = "FRENTE\n";
char msg_obstaculo[] = "OBSTACULO\n";
char msg_tras[] = "TRAS\n";
char msg_antihorario[] = "ANTI-HORARIO\n";
char msg_horario[] = "HORARIO\n";
char msg_parado[] = "PARADO\n";
char msg_70[] = "Velocidade 70%\n";
char msg_80[] = "Velocidade 80%\n";
char msg_100[] = "Velocidade 100%\n";
char msg_dist[] = "DDDcm\n";

// Variáveis de controle de comando
volatile unsigned char flag_comando = 0; // 1 quando tem comando para ser executado
volatile char comando = 'q';

// Controle de mensagem
volatile char *msg = msg_parado;        // buffer com a mensagem
volatile unsigned int contador_msg = 0; // base de tempo para mensagem
volatile unsigned int msg_idx = 0;      // índice do último caractere enviado

// Controle do medidor
volatile unsigned long distancia = 0;       // distância em centímetros
volatile unsigned long contador_medida = 0; // contagens durante a medida de distancia
volatile unsigned int contador_dist = 0;    // base de tempo para medida de distancia
volatile unsigned char flag_echo = 0;

// Controle do Led
volatile unsigned long contador_led = 0;

volatile unsigned long threshold = 0;

void executar_comando();

// Interrupção de PINCHANGE (ECHO)
ISR(PCINT2_vect)
{
  char estado_echo = (PIND & 0b100) >> 2;
  if (estado_echo)
  {
    contador_medida = 0;
  }
  else
  {
    flag_echo = 1;
  }
}

// Interrupção do receptor UART
ISR(USART_RX_vect)
{
  // Recebeu dado
  if (!flag_comando)
  {
    // Ativa a flag e salva o comando
    flag_comando = 1;
    comando = UDR0;
  }
}

// Interrupção do transmissor UART
ISR(USART_TX_vect)
{
  // Dado transmitido
  if (msg[msg_idx + 1] == '\0')
  {
    // Fim da string, reseta o índice
    msg_idx = 0;
  }
  else
  {
    // Envia o proximo caractere
    // Nova interrupção será gerada em seguida
    msg_idx++;
    UDR0 = msg[msg_idx];
  }
}

// Interrução do temporizador
ISR(TIMER0_COMPA_vect)
{
  // Entra aqui a cada 50 us

  // Base de tempo da mensagem
  contador_msg++;

  // Base de tempo da medida de dist.
  contador_dist++;
  if (!flag_echo)
  {
    contador_medida++;
  }
  PORTC &= ~(0b1); // Desliga trigger

  // Base de tempo para o LED
  contador_led++;

  if (contador_msg >= 20000)
  {
    // Entra aqui a cada 1s
    // Envia a mensagem
    UDR0 = msg[msg_idx];
    contador_msg = 0;
  }
  if (contador_dist >= 4000)
  {
    // Entra aqui a cada 200ms
    // Seta trigger
    PORTC |= 1;
    contador_dist = 0;
  }
  if (contador_led >= threshold)
  {
    // 25cm -> 30hz
    // 45cm -> 10hz
    // 75cm -> 5hz
    // 315cm -> 1hz

    if (distancia < 20)
    {
      // Deixa o bit aceso
      PORTC |= 0b100000;
    }
    else
    {
      // Toggle bit do led
      PORTC ^= 0b100000;
    }
    contador_led = 0;
  }
}

void config(void)
{
  cli();

  // Configuração GPIO (Led)
  // Pino A5
  DDRC |= 0b00100000;

  // Configuração GPIO (Motor)
  // A1, A2, A3 e A4 controlam o motor
  DDRC |= 0b00011110;
  PORTC = PARADO;
  // PWM do motor (pinos 3 e 11)
  DDRD |= 0b00001000;
  DDRB |= 0b00001000;

  // Configurações da UART
  UCSR0A = 0;
  UCSR0B = 0b11011000;
  UCSR0C = 0b00000110;
  UBRR0L = 103;
  UBRR0H = 0;

  // Temporizador CTC (50us entre interrupções)
  OCR0A = 99;
  TIMSK0 = 0b00000010;
  TCCR0B = 0b00000010;
  TCCR0A = 0b00000010;

  // Configuração PWM
  TCCR2B = 0b00000011;
  TCCR2A = 0b10100011;
  OCR2A = OCR_70_PERCENT;
  OCR2B = OCR_70_PERCENT;

  // Configuração do Pino TRIG (A0)
  DDRC |= 1;

  // Configuração do Pino ECHO (2)
  // PCINT18 ativada
  PCICR = 0b100;
  PCMSK2 = 0b100;

  sei();
}

int main()
{

  config();

  while (1)
  {
    if (flag_echo)
    {
      // Terminou de medir a distância
      distancia = (Vsom * contador_medida * 50) / 20000;
      flag_echo = 0;
      threshold = (distancia - 15) * (200) / 3;
      msg_dist[0] = (char)(distancia / 100 + 0x30);
      msg_dist[1] = (char)((distancia / 10) % 10 + 0x30);
      msg_dist[2] = (char)(distancia % 10 + 0x30);
    }

    // Verifica se tem obstáculo
    // Está indo para frente quando
    char frente = (PORTC & 0b11110) ^ FRENTE;
    if (frente == 0 && distancia <= 25 && comando != 'X')
    {
      comando = 'X';
      flag_comando = 1;
    }

    // Verifica se algum comando foi recebido
    if (flag_comando)
    {
      flag_comando = 0;
      executar_comando();
    }
  }

  return 0;
}

void executar_comando()
{
  switch (comando)
  {
  case 'w': // Para frente
    PORTC = FRENTE;
    msg = msg_frente;
    break;

  case 'a': // Giro Anti-horario
    PORTC = ANTIHORARIO;
    _delay_ms(50);
    PORTC = PARADO;
    msg = msg_antihorario;
    break;

  case 's': // Para tras
    PORTC = TRAS;
    msg = msg_tras;
    break;

  case 'd': // Giro horario
    PORTC = HORARIO;
    _delay_ms(50);
    PORTC = PARADO;
    msg = msg_horario;
    break;

  case 'q': // Desliga motores
    PORTC = PARADO;
    msg = msg_parado;
    break;

  case 'e': // Retorna última medida de distância
    msg = msg_dist;
    break;

  case '7': // 70%
    OCR2A = OCR_70_PERCENT;
    OCR2B = OCR_70_PERCENT;
    msg = msg_70;
    break;

  case '8': // 80%
    OCR2A = OCR_80_PERCENT;
    OCR2B = OCR_80_PERCENT;
    msg = msg_80;
    break;

  case '0': // 100%
    OCR2A = OCR_100_PERCENT;
    OCR2B = OCR_100_PERCENT;
    msg = msg_100;
    break;

  case 'X': // Obstáculo na frente
    PORTC = PARADO;
    msg = msg_obstaculo;
    break;
  default:
    return;
  }
  // Reseta a mensagem
  msg_idx = 0;
  UDR0 = msg[msg_idx];
  contador_msg = 0;
}