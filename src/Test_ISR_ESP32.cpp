#include <Arduino.h>


#include <FS.h>
#include <SPIFFS.h>
//#include <sqlite3.h>

/*
 * Información extraída desde https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
 * 
 * LOS TEMPORIZADORES.
 * 
 * El ESP32 tiene dos grupos de temporizadores, cada uno con dos temporizadores de hardware de propósito 
 * general. Todos los temporizadores están basados en contadores de 64 bits y un prescalador de 16 bits [1]
 * 
 * El preescalador se utiliza para dividir la frecuencia de la señal base (normalmente 80 MHz), y es
 * utilizado para incrementar/decrementar el contador del temporizador [2]. 
 * 
 * Como el preescalador tiene 16 bits, entonces puede dividir la frecuencia de la señal de reloj en un
 * factor desd 2 a 65536 [2], proporcionándonos un montón de libertar en la configuración.
 * 
 * Los contadores de los temporizadores pueden ser configurador para contar ascendete o descendentemente y
 * y soportan la recarga tanto automática como por el sketch [2].
 * 
 * También pueden generar alarmas cuando alcanzan un valor específico, definido por el sketch [2]. Los 
 * valores del contador pueden ser leidos por el sketch [2].
 * 
 * 
 * VARIABLES GLOBALES.
 * 
 * El código empieza declarando algunas variables globales. La primera será un contado que se utilizará
 * por la rutina de servicio de interrupción (ISR) para indicar al loop principal de que ha ocurrido
 * una interrupción. Esto es así porque la ISR debe ejecutarse tan rápido como sea posible y no debería
 * realizar nunca operaciones largas, como escribir por Serial. De esta forma, una buena práctica es hacer
 * que la ISR solo señale la ocurrancia de la interrupción y delegue la lógica de programa a 'loop'.
 * 
 * El contador (de interrupciones) también es útil porque si por alguna razón la gestión de la interrupción en el 'loop' toma
 * más tiempo del esperado y mientras tanto ocurren más interrupciones, estas no se pierden porque el 
 * contador se irá incrementando convenientemente. Por el contrario, si se utiliza un 'flag' como mecanismo
 * de señal, entonces se pondrá a 'true', y las interrupciones se perderán en el 'loop' puesto que se 
 * asume que se ha producido solo una interrupción adicional.
 * 
 * Puesto que la variable contador será compartida entre 'loop' y la ISR, entonces necesita ser declarada
 * con la palabra reservada 'volatile', que evitará ser eliminada por la optimización del compilador. 
 * 
 *          volatile int interruptCounter;
 * 
 * Tendremos un contador adicional para contar cuantas interrupciones han ocurrido desde el principio del
 * sketch. Será usada solamente por el 'loop' y de esta forma no necesita ser declarada 'volatile'
 * 
 * 
 *          int totalInterruptCounter;
 * 
 * 
 * Para poder configurar el temporizador, necesitamos un puntero a una variable de tipo 'hw_timer_t', que 
 * usaremos en la función 'setup' del sketch.
 * 
 * 
 *          hw_timer_t * timer = NULL;
 * 
 * Por último, necesitamos declarar una variable del tipo 'portMUX', que será usada para controlar la
 * sincronización entre el 'loop' y la ISR al modificar una variable compartida.
 * 
 * 
 *          portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 * 
 * 
 * LA FUNCIÓN 'setup'
 * 
 * Como es habitual, iniciamos la función 'setup' abriendo la conexión serie.
 * 
 *          Serial.begin(115200);
 * 
 * A continuación, inicializamos nuestro temporizador con una llamada a la función 'timerBegin', que
 * devolverá un puntero a una structura de tipo 'hw_timer_t', la misma que la de la variable global que
 * declaramos anteriormente.
 * 
 * Como entrada, esta función recibe el número del temporizador que queremos usar de 0 a 3 (ya que 
 * disponemos de 4 temporizadores por hardware), el valor del preescalador y un flag que indica si el 
 * contador debería contar ascendentemente (true) o descendentemente (false).
 * 
 * Para esta demo, usaremos el primer temporizador y pasaremos 'true' al último parámetro, de forma que
 * el contador cuente hacia arriba.
 * 
 * En relación al preescalador, la frecuencia típica de la señal base usada por los contadores del ESP32 es
 * de 80 MHz, lo que quiere decir que el incremento del contador se realizará 80 000 000 veces por segundo.
 * Usaremos el preescalador para simplificar los cálculos. De esta forma, si dividimos este valor por 80 
 * (usando 80 como valor del preescalador), tendremos una señal con una frecuencia de 1 MHz, que incrementará
 * el contador 1 000 000 de veces por segundo.
 * 
 * Con una frecuancia de 1 MHz, tenemos que el contador será incrementado cada microsegundo.De esta manera,
 * usando un preescalador de 80, cuando llamemos a la función para establecer el valor del contador para
 * generar la interrupción, estaremos especificando el valor en microsegundos.
 * 
 * 
 *          timer = timerBegin(0, 80, true);
 * 
 * 
 * 
 * Pero antes de habilitar el temporizador, necesitamos asociarlo con una función de manejo, que será 
 * ejecutada cuando la interrupción sea generada. Esto se realiza con una llamada a la función
 * 'timerAttachInterrupt'.
 * 
 * Esta función recibe como entrara un puntero al temporizador inicialidado, que almacenamos en la variable
 * global, la dirección de la función que manejará la interrupción y un flag que indica si la interrupción
 * que se generará es por flanco (true) o por nivel (false).
 * 
 * 
 *          timerAttachInterrupt(timer, &onTimer, true);
 * 
 * A continuación usaremos la función 'timerAlarmWrite' para indicar el valor del contador para el cuál se 
 * generará la interrupción del temporizador. De esta forma función recibe como primera entrada un puntero
 * al temporizador, como segunda valor la cuenta para la cual se deberá generar la interrupción, y como
 * tercera un flag que indica si el temporizador deberá recargarse automáticamente después de general la
 * interrupción.
 * 
 * 
 * Así, como primer argumento, pasamos la variable global del temporizador, como tercero ponemos 'true' para
 * que se recargue el contador y de esta forma la interrupción sea generada de forma periódica.
 * 
 * Con relación al segundo argumento, recordemos que configuramos el preescalador de forma que pudieramos
 * trabajar en microsegundos. Así, por ejemplo, si queremos generar una interrupción por segundo, pasamos como
 * valor 1 000 000 (de microsegundos), que es igual a 1 segundo.
 * 
 * 
 *          timerAlarmWrite(timer, 1000000, true);
 * 
 * Terminamos la función de 'setup' habilitando el temporizador con una llamada a la función 'timerAlarmEnable',
 * pasándole como parámetro nuestra variable temporizador.
 * 
 * 
 *          timerAlarmEnable(timer);
 * 
 * 
 * 'setup' quedaría de esta forma:
 *
 *          void setup() {
 *              Serial.begin(115200);
 * 
 *              timer = timerBegin(0, 80, true);
 *              timerAttachInterrupt(timer, &onTimer, true);
 *              timerAlarmWrite(timer, 1000000, true);
 *              timerAlarmEnable(timer);
 *          }
 * 
 * 
 * LA FUNCIÓN 'loop'
 * 
 * En 'loop' será donde manipulemos las interrupciones del timer, después de que sean señalizadas por la ISR. Usaremos
 * un sondeo para comprobar el valor del contador de interrupción. Comprobaremos si 'interruptCounter' es mayor que cero y
 * si lo es, entraremos en la rutina de manipulación. Allí, lo primero que haremos es decrementar este contador, señalizando
 * que la interrupción ha sido reconocida y está siendo gestionada.
 * 
 * Puesto que esta variable está compartida con la ISR, lo haremos dentro de una sección 'crítica', que especificaremos
 * mediante el uso de las macros 'portENTER_CRITICAL' y 'portEXIT_CRITICAL'. Ambas llamadas reciben como argumento la
 * dirección de nuestra variable de tipo 'portMUX_TYPE'.
 * 
 * 
 *          if (interruptCounter > 0) {
 *              portENTER_CRITICAL(&timerMux);
 *              interruptCounter--;
 *              portEXIT_CRITICAL(&timerMux);
 * 
 *              // Interrupt handling code
 *          }
 * 
 * 
 * La manipulación de la interrupción consiste simplemente en incrementar el contador con el número total de interrupciones
 * que han ocurrido desde el inicio de sketch e imprimirlas al puerto serie. El código final de la función 'loop' es:
 * 
 * 
 *          void loop() {
 *              if (interruptCounter > 0) {
 *                  portENTER_CRITICAL(&timerMux);
 *                  interruptCounter--;
 *                  portEXIT_CRITICAL(&timerMux);
 *                  
 *                  totalInterruptCounter++;
 * 
 *                  Serial.print("An interrupt as occurred. Total number: ");
 *                  Serial.println(totalInterruptCounter);
 *              }
 *          }
 * 
 * 
 * EL CÓDIDO DE LA ISR (INTERRUPT SERVICE ROUTINE)
 * 
 * La ISR necesita ser una función que devuelva void y no reciba argumentos. Esta función será tan simple como incrementar
 * el contador de interrupciones que señalizará al 'loop' de que ha ocurrido una interrupción. Eso se hará dentro de la
 * sección 'crítica', declarada con las macros 'portENTER_CRITICAL_ISR' y 'portEXIT_CRITICAL_ISR', que reciben como parámetro
 * la dirección de la variable global de tipo 'portMUX_TYPE' declarada anterioremente.
 * 
 * La ISR debe tener el atributo IRAM_ATTR, de forma que el compilador ubique el código en IRAM. Además, las ISR solo deberían
 * llamar a funciones también ubicadas el IRAM. 
 * 
 * El código completo de la ISR sería.
 * 
 * 
 *              void IRAM_ATTR onTimer() {
 *                  portENTER_CRITICAL_ISR(&timerMux);
 *                  interruptCounter++;
 *                  portEXIT_CRITICAL_ISR(&timerMux);
 *              }
 * 
 * 
 * [1] https://esp-idf.readthedocs.io/en/v1.0/api/timer.html
 * [2] http://espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
 *  
 */
 

volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}
 
void setup() {
 
  Serial.begin(115200);
 
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
 
}
 
void loop() {
 
  if (interruptCounter > 0) {
 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
 
    totalInterruptCounter++;
 
    Serial.print("Ha ocurrido una interrupciob. Total hasta ahora: ");
    Serial.println(totalInterruptCounter);
 
  }
}




