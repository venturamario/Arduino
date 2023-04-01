/*
PRÁCTICA 3  -   SISTEMAS OPERATIVOS
GRUPO: LA PEPA
Mario Ventura Burgos, Luis Miguel Vargas Durán, Felip Toni Font Vicens
02-2022
*/

//__________________________________________________________________________________________________________________
//                                                     PRÁCTICA 3
//__________________________________________________________________________________________________________________


//-------------------------------------------USAR SOLO UN NÚCLEO (app_cpu)------------------------------------------
//UNICORE: app_cpu = 0
//2 COREs: app_cpu = 1 (prog_cpu = 0)
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

//------------------------------------------------------INCLUDE-----------------------------------------------------
#include <time.h>   //PARA INCLUIR TEMPORIZADORES

//------------------------------------------------------DEFINE------------------------------------------------------
#define INFINITE 1                          //Para iteraciones infinitas
#define PROTECT_SERIAL_PORT 1               //Serial port
#define PROTECT_DEADLOCK 1                  //Prevenir deadlock
#define NUM_TASKS 5                         //Nº de filósofs
#define MAX_NUMBER_ALLOWED (NUM_TASKS - 1)  //Max. filosofos que acceden a la sección crítica (N-1)
#define ESPERA 200                          //Espera de la tarea vTaskDelay

//-------------------------------------------TIEMPO INFINITO PREDETERMINADO-------------------------------------------
//Viene por defecto. En caso de necesitarlo, quitar el comentario (decomentar)
//#define INCLUDE_vTaskSuspend

//--------------------------------------------DECLARACIÓN DE VARIABLES-------------------------------------------------
static SemaphoreHandle_t chopstick[NUM_TASKS];  //Tenedores ocupados/libres
static SemaphoreHandle_t mutex_serial;          //Esperar a que serial acabe impresión
static char buf[50];                            //Variable buf para mostrar mensajes. Es una cadena de caracteres
static int n_filosofo = 0;                      //Filósofo n
enum { TASK_STACK_SIZE = 2048 };                //Bytes in ESP32, words in vanilla FreeRTOS (recomendado ponerlo)

//Aviso a la tarea de que el filósofo ha acabado
#if INFINITE == 0
  static SemaphoreHandle_t done_sem; 
#endif
//Nº filosofos en la mesa. Se deja entrar a todos menos a uno para que no se produzca deadlock
#if PROTECT_DEADLOCK
  static SemaphoreHandle_t xCountingSemaphore;  
#endif



//----------------------------------------------------TAREAS----------------------------------------------------------

/*
-----< ACLARACIÓN PREVIA SOBRE EL SIGNIFICADO DE LOS MENSAJES >----- 
-  TOC TOC →  quiere entrar a comer
-  |▄|     →  se ha sentado a comer
-  ¡o      →  coge palillo izquierdo
-  ¡o¡     →  coge palillo derecho  
-  /o\ ÑAM →  está comiendo
-  ¡o_     →  suelta palillo derecho
-  _o      →  suelta palillo izquierdo
-  |_|     →  sale de comer

-----< ORDEN DE LOS SUCESOS EN LA SIMULACIÓN DE LA CENA DE FILÓSOFOS >----- 
1. Se pide permiso para entrar a comer
2. Si se concede permiso, el filósofo n se sienta a la mesa con la intención de comer
3. El filósofo n coge el tenedor izquierdo
4. El filósofo n coge el tenedor derecho
5. El filósofo n se come el plato de espaguetis
6. El filósofo n deja el tendedor derecho
7. El filósofo n deja el tenedor izquierdo
8. El filósofo n "sale de comer"
*/


//SIMULACIÓN DE LA COMIDA DE FILÓSOFOS PROPUESTA POR DIJKSTRA
void cenaFilosofos(void *parameters) {
  
  //VARIABLES NECESARIAS PARA LA IMPLEMENTACIÓN
  int n = *(int *)parameters;    //Filósofo n

  //SE USA LA FUNCIÓN RANDOM PARA ESPERAR UN TIEMPO ALEATORIO ENTRE 0 Y "ESPERA"
  //RECORDAR QUE HEMOS DEFINIDO "ESPERA" CON VALOR 200, ES DECIR, 200ms
  //portTICK_PERIOD_MS ES UNA MACRO YA DEFINIDA POR DEFECTO EN freeRTOS. POR DEFECTO 1 TICK = 1ms
  vTaskDelay(random(0, ESPERA) / portTICK_PERIOD_MS);


  //ITERACIÓN INFINITA EN LA QUE LOS FILÓSOFOS COMEN
  while(1) {
      
      //FILÓSOFO N PIDE PERMISO PARA ENTRAR A LA MESA
      #if PROTECT_SERIAL_PORT
        //Coger mutex
        xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO UN FILÓSOFO PIDE PERMISO PARA ENTRAR
      sprintf(buf, "- Filósofo %i: TOC TOC", (n+1)); //Se usa (n+1) para que no aparezca "filósofo 0". Es una pura cuestión de estética.
      Serial.println(buf);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif


      //FILÓSOFO N SE SIENTA A COMER
      #if PROTECT_DEADLOCK
        //Coger mutex
        xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
      #endif
      #if PROTECT_SERIAL_PORT
        //Coger mutex
        xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO UN FILÓSOFO SE SIENTA A COMER
      sprintf(buf, "- Filósofo %i: |▄|", (n+1));
      Serial.println(buf);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif


      //EL HECHO DE COMER PUEDE PRODUCIR DEADLOCK. SE ESTABLECEN PRIORIDADES PARA QUE ESTO NO SUCEDA
      //Variables auxiliares que necesitaremos para esto:
      int ind1;     //Índice 1
      int ind2;     //Índice 2
      if (n < (n + 1) % NUM_TASKS) {
        ind1 = n;
        ind2 = (n + 1) % NUM_TASKS;
      } else {
        ind1 = (n + 1) % NUM_TASKS;
        ind2 = n;
      }
      //YA SE HAN ESTABLECIDO PRIORIDADES
      //SE PUEDE PROCEDER A COGER CUBIERTOS Y COMER



      //FILÓSOFO N COGE EL TENEDOR DE LA IZQUIERDA
      xSemaphoreTake(chopstick[ind1], portMAX_DELAY);
      #if PROTECT_SERIAL_PORT
        //Coger mutex
        xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO UN FILÓSOFO COGE EL TENEDOR IZQUIERDO
      sprintf(buf, "- Filósofo %i: ¡o", (n+1));
      Serial.println(buf);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif
      
      
      /*
      Cuando un filósofo ha cogido el palillo de la izquierda, pasa un tiempo aleatorio
      pensando en sus cosas de entre 0 y ESPERA (el tiempo de espera definido en el código)
      hasta coger el de su derecha
      SE USA LA FUNCIÓN RANDOM PARA ESPERAR UN TIEMPO ALEATORIO ENTRE 0 Y "ESPERA"
      */
      vTaskDelay(random(0,ESPERA) / portTICK_PERIOD_MS);


      //FILÓSOFO N COGE EL TENEDOR DE LA DERECHA
      xSemaphoreTake(chopstick[ind2], portMAX_DELAY);
      #if PROTECT_SERIAL_PORT
        //Coger mutex
        xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO UN FILÓSOFO COGE EL TENEDOR DERECHO
      sprintf(buf, "- Filósofo %i: ¡o¡", (n+1));
      Serial.println(buf);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif
      

      /*
      Además, antes de decidirse a entrar a comer, todos los filósofos pasan
      un tiempo aleatorio pensando entre 0 y ESPERA
      SE USA LA FUNCIÓN RANDOM PARA ESPERAR UN TIEMPO ALEATORIO ENTRE 0 Y "ESPERA"
      */
      vTaskDelay(random(0,ESPERA) / portTICK_PERIOD_MS);


      //FILÓSOFO N COME
      #if PROTECT_SERIAL_PORT
        //Coger mutex
        xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO UN FILÓSOFO COME
      //Se ponen dos barras "\" porque si no el IDE se confunde y nos manda un aviso. Aún así se imprimirá "/o\ ÑAM"
      sprintf(buf, "- Filósofo %i: /o\\ ÑAM", (n+1));
      Serial.println(buf);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif
      

      /*
      Los filósofos pasan un tiempo aleatorio (de entre 0 y ESPERA) comiendo
      SE USA LA FUNCIÓN RANDOM PARA ESPERAR UN TIEMPO ALEATORIO ENTRE 0 Y "ESPERA"
      */
      vTaskDelay(random(0,ESPERA) / portTICK_PERIOD_MS);


      //FILÓSOFO N HA ACABADO DE COMER Y DEVUELVE EL TENEDOR DERECHO
      #if PROTECT_SERIAL_PORT
        //Coger mutex
        xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO EL FILÓSOFO N DEVUELVE EL TENEDOR DERECHO
      sprintf(buf, "- Filósofo %i: ¡o", (n+1));
      Serial.println(buf);
      //Dejar mutex
      xSemaphoreGive(chopstick[ind2]);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif


      //FILÓSOFO N HA ACABADO DE COMER Y DEVUELVE EL TENEDOR IZQUIERDO
      #if PROTECT_SERIAL_PORT
        //Coger mutex
        xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO EL FILÓSOFO N DEVUELVE EL TENEDOR IZQUIERDO
      sprintf(buf, "- Filósofo %i: _o", (n+1));
      Serial.println(buf);
      //Dejar mutex
      xSemaphoreGive(chopstick[ind1]);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif
      

      //FILÓSOFO N SALE DE COMER
      #if PROTECT_SERIAL_PORT
      //Coger mutex
      xSemaphoreTake(mutex_serial, portMAX_DELAY);
      #endif
      //IMPRIMIR MENSAJE PARA CUANDO EL FILÓSOFO N SALE DE COMER
      sprintf(buf, "- Filósofo %i: |_|", (n+1));
      Serial.println(buf);
      #if PROTECT_SERIAL_PORT
        //Dejar mutex
        xSemaphoreGive(mutex_serial);
      #endif

      //LIBERAR ESPACIO PARA EL FILÓSOFO QUE NO PUDO ENTRAR Y ELIMINAR TAREAS EN CASO DE NO SER INFINITO
      #if INFINITE == 0
        xSemaphoreGive(done_sem);
        vTaskDelete(NULL);
      #endif
      #if PROTECT_DEADLOCK
        xSemaphoreGive(xCountingSemaphore);
      #endif
  }  
}


//CONFIGURAR TAREA setup() DE freeRTOS
void setup() {

  //MUTEX
  mutex_serial = xSemaphoreCreateMutex();
  xSemaphoreGive(mutex_serial);

  //CONFIGURAR serial EN 115200 BAUDIOS
  Serial.begin(115200);

  //AÑADIR UNA ESPERA ANTES DE COMENZAR (1000ms = 1s)
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //INTRODUCCIÓN PREVIA A LA ITERACIÓN INFINITA DONDE LOS FILÓSOFOS COMEN
  Serial.println();
  Serial.println("-----------< CENA DE FILÓSOFOS >-----------");

  //NOMBRE DE LA TAREA
  char task_name[25];

  //CREAR OBJETOS KERNEL NECESARIOS PARA EL CORRECTO FUNCIONAMIENTO DEL PROGRAMA
  #if INFINITE == 0
    done_sem = xSemaphoreCreateCounting(NUM_TASKS, 0);
    xSemaphoreGive(done_sem);
  #endif
  for (int i = 0; i < NUM_TASKS; i++) {
    chopstick[i] = xSemaphoreCreateMutex();
  }
  #if PROTECT_DEADLOCK
    xCountingSemaphore = xSemaphoreCreateCounting(MAX_NUMBER_ALLOWED,MAX_NUMBER_ALLOWED);
    xSemaphoreGive(xCountingSemaphore);
  #endif


  //USAR TAREA xTaskCreatePinnedToCore CON cenaFilosofos PARA HACER QUE LA SIMULACIÓN COMIENCE
  for (n_filosofo = 0; n_filosofo < NUM_TASKS; n_filosofo++) {

    #if PROTECT_SERIAL_PORT
    //Coger mutex
    xSemaphoreTake(mutex_serial, portMAX_DELAY);
    #endif
    //IMPRIMIR MENSAJE PARA CUANDO UN FILÓSOFO NACE / SE AÑADE
    sprintf(task_name, "@ Filósofo %i", (n_filosofo+1));
    //Nuevamente se usa (n_filosofo+1) por una simple cuestión estética. No afecta a la ejecución 
    Serial.println(task_name); 
    #if PROTECT_SERIAL_PORT
      //Dejar mutex
      xSemaphoreGive(mutex_serial);
    #endif
    
    //CREAR UNA TAREA EN UN CORE EN CONCRETO PAERA TENER MÁS CONTROL SOBRE EL USO DEL CORE
    xTaskCreatePinnedToCore(
                cenaFilosofos,        //Función a implementar en la tarea
                task_name,            //Nombre de la tarea
                TASK_STACK_SIZE,      //Tamaño en words
                (void *)&n_filosofo,  //Input de la tarea
                2,                    //Prioridad
                NULL,                 //Task handle
                app_cpu);             //Core donde la tarea correrá (escogido al principio)
  }

  //SALTO DE LÍNEA
  Serial.println();

  #if INFINITE == 0
    //ESPERA HASTA QUE LOS FILÓSOFOS TERMINAN DE COMER
    for(int i = 0; i <= NUM_TASKS; i++) {
      xSemaphoreTake(done_sem, portMAX_DELAY);
    }

    //LOS N FILÓSOFOS HAN COMIDO (NO SE HA PRODUCIDO INANICIÓN) SIN QUE SE PRODUZCA INTERBLOQUEO
    #if PROTECT_SERIAL_PORT
      xSemaphoreTake(mutex_serial, portMAX_DELAY);
    #endif
    //MENSAJE PARA INFORMAR DE QUE LOS N FILÓSOFOS HAN COMIDO SIN PROBLEMAS DE DEADLOCK
    sprintf(buf, "Fin - Ningún bloqueo"); 
    Serial.println(buf);
    #if PROTECT_SERIAL_PORT
      //Dejar mutex
      xSemaphoreGive(mutex_serial);
    #endif
  #endif
}

//TAREA loop() DE freeRTOS
void loop() {
}
