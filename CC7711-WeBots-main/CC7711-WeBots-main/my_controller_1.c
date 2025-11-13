/*
  Exemplo introdutório de uso do WeBots
 */


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */

//TIME_STEP é o incremento de tempo usado na simulação
//512 é um valor MUITO alto.... mas ajuda para ler o que está sendo mostrado no console
//para a simulação final, melhor usar valores menores.... 16 ou 32!
#define TIME_STEP 512


#define QtddSensoresProx 8
#define QtddLeds 10
#define TamanhoTexto 256



/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

int main(int argc, char **argv) {

  
  int i=0;
  char texto[TamanhoTexto]={0};
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;
  
  

  /* necessary to initialize webots stuff */
   wb_robot_init();
 
   /*
    * You should declare here WbDeviceTag variables for storing
    * robot devices like this:
    *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
    *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
    */
  
  
  
  //exemplo para referenciar um objeto no mundo
  //    Alterar a proprieddade SUPERVISOR do e-Puck para TRUE
  //    Mudar o DEF da Wooden Box para CAIXA
   WbNodeRef caixa = wb_supervisor_node_get_from_def("CAIXA");
  
  
  //configurando MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);

  //motores parados
  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);

  
   //configura Sensores de Proximidade
   WbDeviceTag SensorProx[QtddSensoresProx];
   char nomeSensor[10]={0};
   
   for(i=0;i<QtddSensoresProx;i++){
       sprintf(nomeSensor,"ps%d",i); //form os nomes dos sensores
       SensorProx[i] = wb_robot_get_device(nomeSensor);
       wb_distance_sensor_enable(SensorProx[i],TIME_STEP);
    }


    //config leds
    WbDeviceTag Leds[QtddLeds];
    Leds[0] = wb_robot_get_device("led0");
    wb_led_set(Leds[0],-1);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */

  while (wb_robot_step(TIME_STEP) != -1) {
    

    //lendo sensores de proximidade
    for(i=0;i<QtddSensoresProx;i++){
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i])-60;
       sprintf(texto,"%s|%d: %6.0f  ",texto,i,LeituraSensorProx[i]);
    }
    
    //capturando posição da caixa
    const double *PosicaoCaixa = wb_supervisor_node_get_position(caixa);
    sprintf(texto,"%s|caixa em x=%5.2f, y=%5.2f, z=%5.2f",texto,PosicaoCaixa[0],PosicaoCaixa[1],PosicaoCaixa[2]);
    
    //mostrando valores lidos
    printf("%s\n",texto);
 
     //pisca o led
    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1); 

    //define o sentido de rotação em função da posição da caixa
    //lembre-se, é apenas um exemplo!
    if(PosicaoCaixa[0]*PosicaoCaixa[1]<0)
      AceleradorDireito= -1.0, AceleradorEsquerdo= 1.0;
    else
      AceleradorDireito= 1.0, AceleradorEsquerdo= -1.0;
    
 
    wb_motor_set_velocity(MotorEsquerdo, 6.28* AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito , 6.28* AceleradorDireito);
    
    //limpa texto para a proxima iteração
    memset(texto,0,TamanhoTexto);

  };


  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */

  wb_robot_cleanup();

  return 0;

}