#include <ros.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <longitudinal_control/Encoder.h>

#define pi (3.1415926535)
#define RESOLUTION (360)
#define REDUCAO (7.8)
#define sample_delay (100)

ros::NodeHandle encoder;
//std_msgs::Float32 velocidade;
//std_msgs::Float32 posicao;
longitudinal_control::Encoder msg_pub;

ros::Publisher pub_data("encoder_data", &msg_pub);
//ros::Publisher chatter("posicao", &posicao);

int EncoderPos = 0;
float rpm = 0;
float velo = 0.0;
float velo_roda = 0.0;
float raio = 0.08;
float deslocamento = 0.0;
float desloca_atual = 0.0;
long start_time = 0;
unsigned int pulseCount = 0;
unsigned int tempo_atual = 0;
unsigned int tempo_anterior = 0;

void setup() {
  encoder.initNode();
  //encoder.advertise(chatter);
  encoder.advertise(pub_data);
  
  Serial.begin(57600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  void pulsos();

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, pulsos, RISING);
}

void loop(){
  deslocamento = 0.0;
  pulseCount = 0;
      while(true){
          // satura a variavel EncoderPos
          if(EncoderPos > 100){
            EncoderPos = 100;
            }
          else if (EncoderPos < -100){
            EncoderPos = -100;
           }
          // Calcula deslocamento e velocidade pelos pulsos do encoder
          if (millis() - tempo_anterior >= 100){
            // calculo velocidade em rpm
            rpm = (pulseCount*(60000.f/((unsigned int)millis() - tempo_anterior)))/RESOLUTION;
            desloca_atual = pulseCount; // atribui os pulsos a uma variavel
            // analisa o sentido de rotacao  a partir do sinal da variavel EncoderPos e se for negativo atribui velocidade negativa e deslocamento negativo.
            if (EncoderPos < 0) {
              rpm = -1*rpm;
              desloca_atual = -1*desloca_atual;
            }
            // calculo do deslocamento total em metros
            deslocamento = deslocamento + ((desloca_atual/RESOLUTION)/REDUCAO)*(2*pi*raio);;
            // Converte velocidade de rpm para m/s
            velo = raio*rpm*(pi/30);
            velo_roda = velo/REDUCAO;         
            // atribui valor a ser publicado no topico
            //velocidade.data = velo_roda;
            msg_pub.velocity = velo_roda;
            msg_pub.pos = deslocamento;
            
            // Zera o contador de pulsos
            pulseCount = 0;
            // Atualiza o temporizador
            tempo_anterior = (unsigned int)millis();
          }
          
          //Publica os valores nos topicos
          //chatter.publish( &velocidade );
          pub_data.publish( &msg_pub );
          encoder.spinOnce();
          
          delay(sample_delay);
      }
   // }
  //}
}
// Conta os pulsos do Encoder
void pulsos(){
  pulseCount = pulseCount + 1;
// Checa o sentido de rotacao do encoder
  if(abs(rpm) < 300){
    // Sentido positivo (para frente)
    if (digitalRead(3) == HIGH){
      EncoderPos = EncoderPos + 1;
    }
    // Sentido negativo (para tras)
    else{
      EncoderPos = EncoderPos - 1;
     }
    }
  }
  






