#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <queue.h>
#include <semphr.h>
#include <timer.h>

//Controls the servo motor
Servo myservo;

// Ports
#define temp_pin A3
#define switch_pin 13
#define servo_pin 3
#define DC_pin 5

// Function prototype
void vTimerCallback1SecExpired();
void interruptHandlerLow();
void interruptHandlerHigh();
void TaskServoMotor( void *pvParameters );
void TaskDCMotor( void *pvParameters );

// Create the timer instance
Timer timer;
// Declaring a global variables 
QueueHandle_t sensorValueQueue;
SemaphoreHandle_t binarySemaphoreLow;
SemaphoreHandle_t binarySemaphoreHigh;

// The setup function runs once when you press reset or power the board
void setup()
{

  /*--------- Initialize serial communication at 9600 bits per second-------------*/
  Serial.begin( 9600 );

  while( !Serial )
  {
    ;  // wait for serial port to connect
  }

  /*---------------------- Create a Timer and start it ---------------------------*/
  // The timer will repeat every 1 sec
  timer.setInterval(1000); 

  // The function to be called
  timer.setCallback(vTimerCallback1SecExpired);

  // Start the timer
  timer.start();

  /*------------------------------ Create a Queue --------------------------------*/
  sensorValueQueue = xQueueCreate(
    1                  // Queue length
    , sizeof( float )  // Queue item size
  );

  /*------------------------------- Create Tasks ---------------------------------*/
  if( sensorValueQueue != NULL )
  {
    // This task consumes the queue if it was created
    xTaskCreate(
      TaskDCMotor      // A pointer to the task function
      ,  "DcMotor"     // Descriptive task name used during profiling (A name just for humans)
      ,  128           // Stack size
      ,  NULL          // Pointer to task parameters
      ,  1             // Task priority
      ,  NULL          // Returns a task handle
    );
  }
  xTaskCreate(
    TaskServoMotor
    ,  "ServoMotor"
    ,  128
    ,  NULL
    ,  1 
    ,  NULL
  );

  /*------------------------ Create 2 binary Semaphores ---------------------------*/
  binarySemaphoreLow = xSemaphoreCreateBinary();
  if ( binarySemaphoreLow != NULL ) 
  {
    // Attach interrupt for Arduino digital pin
    attachInterrupt( digitalPinToInterrupt( switch_pin ), interruptHandlerLow, LOW );
  }
  binarySemaphoreHigh = xSemaphoreCreateBinary();
  if ( binarySemaphoreHigh != NULL ) 
  {
    attachInterrupt( digitalPinToInterrupt( switch_pin ), interruptHandlerHigh, HIGH );
  }

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Things are done in Tasks
  // Update the timer
  timer.update();
}

/*----------------------------------------------------------------------------------*/
/*--------------------------------------- ISR --------------------------------------*/
/*----------------------------------------------------------------------------------*/

void interruptHandlerLow() 
{
  // Give semaphore in the interrupt handler
  xSemaphoreGiveFromISR( binarySemaphoreLow, NULL );
}

void interruptHandlerHigh() 
{
  // Give semaphore in the interrupt handler
  xSemaphoreGiveFromISR( binarySemaphoreHigh, NULL );
}

/*----------------------------------------------------------------------------------*/
/*------------------------------- Tempreture Sensor --------------------------------*/
/*----------------------------------------------------------------------------------*/

// This function publish data in the queue if it was created
void vTimerCallback1SecExpired() 
{
  float sensorValue;
  // read the input on analog pin 3
  sensorValue = analogRead( temp_pin );
  //convert to Celsius
  sensorValue = sensorValue * 5 / 1023;
  sensorValue -= 0.5;
  sensorValue *= 100;
  
  // Post an item on a queue.
  xQueueSend( sensorValueQueue, &sensorValue, portMAX_DELAY );
  // print out the value you read
  Serial.println( sensorValue );
}

/*----------------------------------------------------------------------------------*/
/*-------------------------------------- Tasks -------------------------------------*/
/*----------------------------------------------------------------------------------*/

/*------------------------------------- DC Motor -----------------------------------*/

void TaskDCMotor( void *pvParameters ) 
{
  char *pcTaskName = ( char * ) pvParameters;
  float valueFromQueue = 0;
  //calculates the dc motor speed base on the temperature
  int DCSpeed;

  // initialize digital pin 5 as an output
  pinMode( DC_pin, OUTPUT );

  for ( ;; )
  {
    // Read an item from a queue
    if ( xQueueReceive( sensorValueQueue, &valueFromQueue, portMAX_DELAY ) == pdPASS ) { // If can retrieve an item from queue
      // print out the name of this task
      Serial.println( pcTaskName );
      if ( valueFromQueue > 19 && valueFromQueue < 41 )
      {
        // (current_temp - 20)*60 is the value that should be written in dc motor
        DCSpeed = map( ( valueFromQueue - 20 )*60 , 0 , 1023, 2 , 155 );
        analogWrite( DC_pin, DCSpeed );
      }
      else
      {
        Serial.println( "Temperature is not in a valid range!" );
      }
    }
  }
}

/*-------------------------------------- Servo Motor ---------------------------------*/

void TaskServoMotor( void *pvParameters )
{
  char *pcTaskName = ( char * ) pvParameters;

  //measures the speed base on the value in switch_state
  float speedMode;
  //shows the servo motor spinning direction
  bool direction = true;
  //position of the servo motor
  float pos = 1;

  // initialize digital pin 3 as an output
  pinMode( servo_pin, OUTPUT );
  myservo.attach( servo_pin );

  for ( ;; )
  {
    // print out the name of this task
    Serial.println( pcTaskName );

    // try to take the low semaphore
    if ( xSemaphoreTake( binarySemaphoreLow, portMAX_DELAY ) == pdPASS ) 
    {
      speedMode = 50 * 100;
    }
    // try to take the high semaphore
    if ( xSemaphoreTake( binarySemaphoreHigh, portMAX_DELAY ) == pdPASS ) 
    {
      speedMode = 25 * 100;
    }
    if ( pos < 180 && direction )
    {
      pos = pos + 180 / speedMode;
    }
    else
    {
      direction = false;
      pos = pos - 180 / speedMode;
      if( pos <= 1 )
        direction = true;
    }
    myservo.write( pos );
  }
}
