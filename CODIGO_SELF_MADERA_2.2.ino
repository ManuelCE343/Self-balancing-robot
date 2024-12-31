#include <Wire.h>   //I2C Library
#include <math.h>   //Math library

// Variables de trabajo PID
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
//------------------Los Valors de PID cambian con cada diseño
double kp =17.267;  //double Kp = ->21;21.46;23.33;23.24
double kd = 0.611;  //double Kd = ->0.8;1;1.048;1.112
double ki = 122.0829;  //double Ki = ->140;89.89;101.9;121.4

int SampleTime = 1000; // Tiempo de muestreo 1 segundo.
double outMin, outMax;
bool vertical = false;     //es el robot lo suficientemente vertical como para correr
float robot_speed;
/////////////////////////////////////////////////////////////////////////////////
//IMU ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/////////////////////////////////////////////////////////////////////////////////

#define MPU6050 0x68    //IMU Address

//Se utiliza para sacar la IMU del modo de reposo
#define PWR_MGMT_1 0x6B   

//GLOBAL VARIABLES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

long accel_x, accel_y, accel_z, acc_total_vector;
int16_t gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int16_t tempReading;
float angle_pitch, angle_roll, acc_angle_pitch, acc_angle_roll;
float angle_pitch_output, angle_roll_output;
unsigned long looptime;
boolean set_gyro_angles;
boolean first_up = true;
boolean Reset = false;

//COSAS DEL ACELERÓMETRO~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//Configuración de Accel, AFS_SEL selecciona el rango
#define ACCEL_CONFIG 0x1C   

//Registros de medición del acelerómetro, muestra los últimos valores del acelerómetro, cada eje está compuesto por 2 registros. Se puede encontrar en el mapa de datos del registro del MPU-6050. Estos registros son de sólo lectura, registros de cara al usuario 
#define ACCEL_XOUT_15_8 0x3B   //Por ejemplo, este registro y
#define ACCEL_XOUT_7_0 0x3C                                   // este registro compone el eje x
#define ACCEL_YOUT_15_8 0x3D
#define ACCEL_YOUT_7_0 0x3E
#define ACCEL_ZOUT_15_8 0x3F
#define ACCEL_ZOUT_7_0 0x40

//COSAS DEL GIROSCOPIO~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define GYRO_CONFIG 0x1B

//Registros del sensor del giroscopio 
#define GYRO_XOUT_15_8 0x43
#define GYRO_XOUT_7_0 0x44
#define GYRO_YOUT_15_8 0x45
#define GYRO_YOUT_7_0 0x46
#define GYRO_ZOUT_15_8 0x47
#define GYRO_ZOUT_7_0 0x48

//Sensor de TEMPERATURA~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Registros del sensor de temperatura. 
#define TEMP_OUT_15_8 0x41
#define TEMP_OUT_7_0 0x42


void imu_setup(){ //Inicializar Acelerómetro, Giroscopio; establecer los bits correctos en la IMU
  Wire.begin(MPU6050);                //configurar imu como esclavo
  
  Wire.beginTransmission(MPU6050);    //enviar la dirección de la IMU
  Wire.write(PWR_MGMT_1);             //necesito escribir ALGO en el vector de gestión de energía para obtener un valor utilizable, y encender la imu
  Wire.write(0x00);                   //Poner 0 en el buffer, no importa lo que pongas aquí
  Wire.endTransmission();             // Enviar 0 al registro de gestión de energía, encender la IMU y obtener valores legibles  
  
  Wire.beginTransmission(MPU6050);    //Establece los bits de configuración del MPU6050
  Wire.write(ACCEL_CONFIG);           //Seleccionar el bit de calibración del acelerómetro
  Wire.write(0x10);                   //Escribir Ajuste del rango del acelerómetro, +/- 8g
  Wire.endTransmission();             //enviar los ajustes del rango del acelerómetro

  Wire.beginTransmission(MPU6050);    //MPU6050 Dirección
  Wire.write(GYRO_CONFIG);            //Registro de dirección del giroscopio
  Wire.write(0x08);                   //Registro de escala de sensibilidad del giroscopio, +/- 500 deg/s
  Wire.endTransmission();



}

void gyro_calibration(){    //Clase de calibración
for(int cycle_count = 0; cycle_count < 2000; cycle_count++){    //Este bucle toma 2000 lecturas de los ejes x,y y z del giroscopio
  read_MPU6050();                       //Tomar los valores brutos del MPU6050 
  gyro_x_cal += gyro_x;               //Agregar el nuevo valor bruto al valor total del giroscopio 
  gyro_y_cal += gyro_y;
  gyro_z_cal += gyro_z;
  delay(3);                //añade 3us de retardo para simular un bucle de 250hz.
}
gyro_x_cal /= 2000;   //dividir por 2000 para obtener los valores medios de calibración
gyro_y_cal /= 2000;
gyro_z_cal /= 2000;

}

 
void read_MPU6050(){
  Wire.beginTransmission(MPU6050);   //Configuración de la IMU como esclavo 
  Wire.write(ACCEL_XOUT_15_8);    //Enviar el registro de inicio solicitado
  Wire.endTransmission();
  Wire.requestFrom(MPU6050,14);  //solicitar 14 bytes de datos para ser leídos, no cortar la conexión. Esto llamará automáticamente a los siguientes registros después de ACCEL_XOUT_15_8 durante "14" ciclos, para que los 14 datos de los siguientes registros secuenciales                                       //is read and placed into the buffer to be sent. In this case that is convienent because the next 14 registers are all data registers for the accelerometer and gyroscope 
  while(Wire.available() < 14);  
  accel_x = (Wire.read()<<8) | Wire.read();    //Lectura de datos en ráfaga, int16_t maneja el complemento a 2 que sale de la IMU. Necesitamos leer todos los datos a la vez para asegurar que nuestras lecturas son de la misma instancia en el tiempo
                                                     //Estamos leyendo secuencialmente los 14 bytes solicitados al principio del bucle, y combinando dos registros juntos para obtener el valor completo de 16 bits para cada eje 
  accel_y = (Wire.read()<<8) | Wire.read();    //Eje Y
  accel_z = (Wire.read()<<8) | Wire.read();    //Eje Z
 
  tempReading = (Wire.read()<<8) | Wire.read();    

  gyro_x = (Wire.read()<<8) | Wire.read();
  gyro_y = (Wire.read()<<8) | Wire.read();
  gyro_z= (Wire.read()<<8) | Wire.read();

 
}

void angle_calc(){   // pasar las direcciones de aceleración y giroscopio.

read_MPU6050();

gyro_x -= gyro_x_cal;   // Resta el valor de calibración del giroscopio del valor actual del mismo. 
gyro_y -= gyro_y_cal;
gyro_z -= gyro_z_cal;

angle_pitch += gyro_x * .0000611;   //Calcular el ángulo recorrido en el último periodo de 4ms y añadirlo al valor de angle_pitch
angle_roll += gyro_y * .0000611;    //Calcular el ángulo recorrido en el último periodo de 4ms y añadirlo al valor de angle_roll

//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) La función Arduino sin está en radianes
angle_pitch += angle_roll * sin(gyro_z * 0.000001066);   //Si la IMU ha guiñado transfiere el ángulo de balanceo al ángulo de cabeceo 
angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);   //Si la IMU ha guiñado transfiere el ángulo de cabeceo al ángulo de balanceo

//Accelerometer Angle Calculations:
acc_total_vector = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));   //Calcular el vector total del Acelerómetro (Vector de Gravedad)
//57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
acc_angle_pitch = asin((float)accel_y/acc_total_vector)* 57.296;       //Calcular el ángulo de inclinación
acc_angle_roll = asin((float)accel_x/acc_total_vector)* -57.296;        //Calcular el ángulo de balanceo

//Coloque el nivel de burbuja MPU-6050 y anote los valores en las dos líneas siguientes para la calibración
  acc_angle_pitch -= 0;                                             //Valor de calibración del acelerómetro para el paso , -.5
  acc_angle_roll -= 0;                                               //Valor de calibración del acelerómetro para el balanceo, -3

if(set_gyro_angles){                                                 //Si la IMU ya está iniciada
    //Complimentary Filter
    angle_pitch = angle_pitch * 0.9996 + acc_angle_pitch * 0.0004;     //Corregir la deriva del ángulo de cabeceo del giroscopio con el ángulo de cabeceo del acelerómetro
    angle_roll = angle_roll * 0.9996 + acc_angle_roll * 0.0004;        //Corregir la deriva del ángulo de balanceo del giroscopio con el ángulo de balanceo del acelerómetro
    //angle_pitch = angle_pitch;     
    //angle_roll = angle_roll;   
  }
  else{                                                                //En el primer arranque, ajuste los valores de cabeceo y balanceo del giroscopio para corregir las irregularidades del terreno
    angle_pitch = acc_angle_pitch;                                     //Configura el ángulo de cabeceo del giroscopio igual al ángulo de cabeceo del acelerómetro 
    angle_roll = acc_angle_roll;                                       //Configura el ángulo de balanceo del giroscopio igual al ángulo de balanceo del acelerómetro 
    set_gyro_angles = true;                                            //Ajustar la bandera de inicio de la IMU
  }

//Para amortiguar los ángulos de cabeceo y balanceo se utiliza otro filtro complementario, HACE UNA GRAN DIFERENCIA. Traza una línea en el plotter de serie y mira, angle_pitch vs angle_pitch_output por ejemplo
angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Toma el 90% del valor de tono de salida y añade el 10% del valor de tono bruto
angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Toma el 90% del valor del rollo de salida y añade el 10% del valor del rollo bruto

//angle_pitch_output = angle_pitch; 
//angle_roll_output = angle_roll; 
Input= angle_roll_output;
//Serial.print(Input);
//Serial.println();
//check if robot is vertical
  if (Input > 50 || Input < -50) vertical = false;
  if (Input < 15 && Input > -15) vertical = true;


}

////-----------------Control de Motores-----------------------------------------------------------------------------------------------------------------
//******************************************************************************************************************************************************

int ENA = 5;//derecha
int IN1 = 7;//derecha
int IN2 = 6;//derecha
int IN3 = 8;//izquierda
int IN4 = 9;//izquierda
int ENB = 10;//izquierda

int minAbsSpeed_izq =25; // Velocidad mínima a la que giran los motores para superar la inercia y atrapar el robot
int minAbsSpeed_der= 40;
int velocidad_izq,velocidad_der,currentSpeed,currentSpeed_2;

 //Clase que controla la dirección y velocidad de los motores
void mover(int leftSpeed, int rightSpeed)
{
    if (rightSpeed < 0)
    {
        rightSpeed = min(rightSpeed, -1*minAbsSpeed_der);
        rightSpeed = max(rightSpeed, -255);
    }
    else if (rightSpeed > 0)
    {
        rightSpeed = max(rightSpeed, minAbsSpeed_der);
        rightSpeed = min(rightSpeed, 255);
    }
    
    int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed_der, 255);

    if (leftSpeed < 0)
    {
        leftSpeed = min(leftSpeed, -1*minAbsSpeed_izq);
        leftSpeed = max(leftSpeed, -255);
    }
    else if (leftSpeed > 0)
    {
        leftSpeed = max(leftSpeed, minAbsSpeed_izq);
        leftSpeed = min(leftSpeed, 255);
    }
    
    int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed_izq, 255);
    
    digitalWrite(IN1, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN2, rightSpeed > 0 ? LOW : HIGH);
    digitalWrite(IN3, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN4, leftSpeed > 0 ? LOW : HIGH);
    analogWrite(ENA, realRightSpeed );
    analogWrite(ENB, realLeftSpeed );
}

//// esta funcion hace girar hacia la izquierda


void gira_izq(int velocidad)
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    int minSpeed;
    if (velocidad < 0)
    {
    velocidad = min(velocidad, -1*minAbsSpeed_izq);
    velocidad = max(velocidad, -255);
    minSpeed = min(minAbsSpeed_izq,255-abs(velocidad));
    analogWrite(ENA, minSpeed );
    analogWrite(ENB, -1*velocidad);
    }
    else
    {
    velocidad = max(velocidad, minAbsSpeed_izq);
    velocidad = min(velocidad, 255);        
    minSpeed = min(minAbsSpeed_izq,255-abs(velocidad));
    analogWrite(ENA, velocidad );
    analogWrite(ENB, minSpeed );
    }
}
void gira_der(int velocidad)
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);   
    int minSpeed;
    
    if (velocidad < 0)
    {
    velocidad = min(velocidad, -1*minAbsSpeed_der);
    velocidad = max(velocidad, -255);    
    minSpeed = min(minAbsSpeed_der,255-abs(velocidad));
    analogWrite(ENA, -1*velocidad);
    analogWrite(ENB, minSpeed);
    }
    else
    {
    velocidad = max(velocidad, minAbsSpeed_der);
    velocidad = min(velocidad, 255);
    minSpeed = min(minAbsSpeed_der,255-abs(velocidad));
    analogWrite(ENA, minSpeed );
    analogWrite(ENB, velocidad);
    }
}

void stop_m(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
    
}

void startup(){                 //initializa the robot
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);        
  pinMode(ENB, OUTPUT);

}
// ************************************** PID CONTROL *******************************************************************************
//*************************************************************************************************************************************
void PID_COM()
{
  if(vertical==true){
    if (Reset== true){
      gyro_x =0;   // Resta el valor de calibración del giroscopio del valor actual del mismo. 
      gyro_y =0;
      gyro_z =0;
      angle_pitch=0;   //Si la IMU ha guiñado transfiere el ángulo de balanceo al ángulo de cabeceo 
      angle_roll=0;
      }
  Reset=false;
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=SampleTime)
  {
    // Calculamos todos los errores.
    double error = Setpoint - Input;
    ITerm+= (ki * error);
    if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
      // Calculamos la función de salida del PID.
      Output = kp * error + ITerm- kd * dInput;

     
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
      mover(Output,Output); 
  
        // Guardamos el valor de algunas variables para el próximo recálculo.
        lastInput = Input;
        lastTime = now;
  }
  }
  else{
    stop_m();
    lastInput = 0;
    ITerm = 0;
    Reset=true;
    

  }
}
void SETUP()
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
  kp = kp;
  ki = ki * SampleTimeInSec;
  kd = kd / SampleTimeInSec;
}
void TIEMPO_MUESTREO(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio = (double)NewSampleTime / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

void LIMITES(double Min, double Max)
{
  if(Min > Max) return;
    outMin = Min;
    outMax = Max;
  if(Output > outMax) Output = outMax;
  else if(Output < outMin) Output = outMin;
  if(ITerm> outMax) ITerm= outMax;
  else if(ITerm< outMin) ITerm= outMin;
}

// **********************************************// SETUP //*****************************************************************************************
//******************************************************************************************************************************************************
void setup() {
  delay(500);  //Demora para dar al usuario la oportunidad de poner el robot sobre su espalda antes de que comience la calibración del giroscopio
  pinMode(13, OUTPUT); // Establecer el LED de estado como salida
  imu_setup();    //Inicializar la IMU
  digitalWrite(13, LOW); // apagar el led mientras se calibra
  gyro_calibration();
  digitalWrite(13, HIGH); //Encender el led una vez finalizada la calibración 
  startup();
  SETUP();
  TIEMPO_MUESTREO(10); 
  LIMITES(-255, 255);
  looptime = micros();    //Empezar a llevar la cuenta de cuándo se inicia el bucle
}

void loop() {
  angle_calc();                                                                                                  //calcular el ángulo del robot
  PID_COM();                                                                   // Ejecutar la rutina de equilibrado que consiste en los bucles PID de ángulo y velocidad y la lógica asociada
  
  //Serial.println(float((micros() - looptime))/1000);                           //código que imprime la duración del bucle en milisegundos 
  while(micros() - looptime < 4000);                                             //Asegúrese de que el bucle se ejecuta consistentemente a 4ms antes de preceder, 4000 microsegundos = 4 ms.
                                                                              //El bucle debe durar <4ms sin importar en qué fase de operación se encuentre el robot, de lo contrario podría haber problemas.
  looptime = micros(); 

}
