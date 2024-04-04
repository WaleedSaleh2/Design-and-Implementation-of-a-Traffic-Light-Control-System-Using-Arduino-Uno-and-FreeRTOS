// As we use FreeRTOS to implement the system and we semaphore to synchronize the tasks and LCD display
// we need to include Arduino_FreeRTOS.h , semphr.h , and  LiquidCrystal.h libraries. 
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <LiquidCrystal.h>

/* Task function prototypes */
void traffic1_2Task(void *pvParameter);
void traffic3Task(void *pvParameter);
void InterruptTask(void *pvParameters); 

/* Interrupt function prototype */
void button_ISR();

/* Define constants */
#define WaitingTime 5000    // Waiting time for each traffic light in the green state in mille second, default number is 5000 ms

#define BuzzerPin 10        // Pin number for the buzzer

#define InterruptPin 2      // Pin number for the push button interrupt

// Pin numbers for the traffic light 1 and 2 LEDs (both traffic lights 1 & 2 are 
// connected to the same pins, because they have same functionlity)
#define RED1 4              
#define YELLOW1 5
#define GREEN1 6

#define RED2 7              // Pin numbers for the traffic light 3 LEDs
#define YELLOW2 8
#define GREEN2 9

/* Global variables */
// Flag to indicate if the push button has been pressed and it is boolean flag, if the value of the flag true that mean has press first
// time and the all traffics will suspended, and if pressed again then all traffics will resuem.
bool PushButtonFlag = true;      

int8_t displayMassage = 1;   // To display the name of the project 

/* Task handles and semaphores */
TaskHandle_t task1_2Handle;             // Handle for traffic1Task
TaskHandle_t task3Handle;               // Handle for traffic2Task
SemaphoreHandle_t xSemaphore1;          // Semaphore for traffic1Task
SemaphoreHandle_t xSemaphore2;          // Semaphore for traffic2Task
SemaphoreHandle_t buttonSemaphore;      // Semaphore for push button interrupt

/* LCD instance */
LiquidCrystal lcd(14, 15, 16, 17, 18, 19);

void setup() {
  /* Initialize the LCD display */
  lcd.begin(16,2);

  /* Display the initial message on the LCD */
  lcd.setCursor(0,0);
  lcd.print(" Traffic Lights");
  lcd.setCursor(0,1);
  lcd.print("     System");

  /* Set the pin modes for the LEDs */
  pinMode(RED1, OUTPUT);
  pinMode(YELLOW1, OUTPUT);
  pinMode(GREEN1, OUTPUT);
  
  pinMode(RED2, OUTPUT);
  pinMode(YELLOW2, OUTPUT);
  pinMode(GREEN2, OUTPUT);

  /* Set the pin mode for the buzzer */
  pinMode(BuzzerPin, OUTPUT);

  /* Set the pin mode for the interrupt pin */
  pinMode(InterruptPin, INPUT_PULLUP);

  /* Create the semaphores */
  xSemaphore1 = xSemaphoreCreateBinary();
  xSemaphore2 = xSemaphoreCreateBinary();
  buttonSemaphore = xSemaphoreCreateBinary();

  /* Attach the interrupt handler to the interrupt pin */
  attachInterrupt(digitalPinToInterrupt(InterruptPin), button_ISR, RISING);

  /* Create the tasks */
  xTaskCreate(traffic1_2Task, "Traffic 1 & 2 Task", 128, NULL, 0, &task1_2Handle);
  xTaskCreate(traffic3Task, "Traffic 3 Task", 128, NULL, 0, &task3Handle);
  xTaskCreate(InterruptTask, "Interrupt Task for a push Button", 128, NULL, 3, NULL );

  /* Give the semaphore to the first task */
  xSemaphoreGive(xSemaphore1);

  /* Start the FreeRTOS scheduler */
  vTaskStartScheduler();
}

void loop() {
  // Empty loop, All jobs will done in the tasks 
}

/* Interrupt service routine for the push button */
void button_ISR(){
  // we use interrupt in pin2 so we will use int0 
  // Disable int0 interrupt
  EIMSK = 0; 
  static BaseType_t context_switch_flag;

  PushButtonFlag = !PushButtonFlag;

  /* Give the semaphore from the ISR context */
  xSemaphoreGiveFromISR(buttonSemaphore, &context_switch_flag);

  /* Check if a context switch is required */
  if (context_switch_flag == pdTRUE){
    portYIELD_FROM_ISR();
  }
}

/* Task function for handling the push button interrupt */
void InterruptTask(void *pvParameters)  
{
  while(1){
    /* Wait for the semaphore to be given */
    if (xSemaphoreTake(buttonSemaphore, 50) == pdTRUE){
      /* here each time the pushbutton will be pressed the if section will execute and the next 
         pressed the else section will execute, and so on... */
      if(PushButtonFlag){
        /* Display the "STOP" message on the LCD */
        lcd.setCursor(15,1);
        lcd.print("S");

        /* Suspend the traffic light tasks */
        vTaskSuspend(task1_2Handle);
        vTaskSuspend(task3Handle);
      }
      /* If the push button has already been pressed */
      else{
        /* Remove the "STOP" message from the LCD */
        lcd.setCursor(15,1);
        lcd.print(" ");

        /* Resume the traffic light tasks */
        vTaskResume(task1_2Handle);
        vTaskResume(task3Handle);
      }
      
        /* 50ms for debounce */
        vTaskDelay(100);

        /* Enable the interrupt pin again */
        EIMSK = 1; 
    }
  }
}

/* Task function for controlling traffic lights 1 and 2 */
void traffic1_2Task(void *pvParameter) {
  while (1) {
    /* Wait for the semaphore to be given */
    xSemaphoreTake(xSemaphore1, portMAX_DELAY);

    // For the first time, the name of the project will be displayed on the LCD, and the displayMessage variable will be 
    // incremented by one so will not be displayed again.
    if(displayMassage == 1){
      vTaskDelay(3000 / portTICK_PERIOD_MS); // wait for 3 second to display the initial message
      lcd.setCursor(0,0);
      lcd.print("               ");
      lcd.setCursor(0,1);
      lcd.print("           ");
      displayMassage++;
    }

    /* Update the LCD with the current state and task number */
    lcd.setCursor(0,0);
    lcd.print("Traffic No. 1&2");

    /* Set the initial state to red for both traffic lights 1 and 2 and 3*/
    lcd.setCursor(0,1);
    lcd.print("State: R");
    digitalWrite(RED1, HIGH);
    digitalWrite(RED2, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Transition to yellow for traffic light 1 and 2 */
    digitalWrite(YELLOW1, HIGH);
    digitalWrite(RED1, LOW);
    lcd.setCursor(0,1);
    lcd.print("State: Y");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Transition to green for traffic light 1 and 2*/
    digitalWrite(YELLOW1, LOW);
    digitalWrite(GREEN1, HIGH);
    lcd.setCursor(0,1);
    lcd.print("State: G");
    lcd.setCursor(9,1);
    lcd.print("T:");

    /* Countdown the remaining time for the green light */
    for(int i= WaitingTime/1000; i>=0; i--){
      lcd.setCursor(11,1);
      lcd.print(i);
      lcd.print("s");
      if(i < 10){       // if counter i < 10 update the format of display the counter on the LCD
        lcd.setCursor(13,1);
        lcd.print(" ");
      }
      if(i==0){         // if counter i == 0 then clear the counter display on the LCD
        lcd.setCursor(9,1);
        lcd.print(" ");
        lcd.print(" ");
        lcd.print(" ");
        lcd.print(" ");
        break;
      } 
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* Flash the green light and sound the buzzer */
    digitalWrite(GREEN1, LOW);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(GREEN1, HIGH);
    digitalWrite(BuzzerPin, HIGH);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(GREEN1, LOW);
    digitalWrite(BuzzerPin, LOW);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(GREEN1, HIGH);
    digitalWrite(BuzzerPin, HIGH);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    digitalWrite(GREEN1, LOW);
    digitalWrite(BuzzerPin, LOW);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(GREEN1, HIGH);
    digitalWrite(BuzzerPin, HIGH);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    /* Transition to yellow for traffic light 1 and 2 */
    lcd.setCursor(0,1);
    lcd.print("State: Y");
    digitalWrite(YELLOW1, HIGH);
    digitalWrite(GREEN1, LOW);
    digitalWrite(BuzzerPin, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Transition to red for both traffic lights 1 and 2 */
    lcd.setCursor(0,1);
    lcd.print("State: R");
    digitalWrite(RED1, HIGH);
    digitalWrite(YELLOW1, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    /* Give the semaphore to traffic3Task to start its execution */
    xSemaphoreGive(xSemaphore2);      
    taskYIELD(); 
  }
}

/* Task function for controlling traffic light 3 */
void traffic3Task(void *pvParameter) {
  while (1) {
    /* Wait for the semaphore to be given */
    xSemaphoreTake(xSemaphore2, portMAX_DELAY);

    /* Update the LCD with the current state and task number */
    lcd.setCursor(11,0);
    lcd.print(" 3");
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(" ");

    /* Set the initial state to red for all traffic lights */
    lcd.setCursor(0,1);
    lcd.print("State: R");
    digitalWrite(RED1, HIGH);
    digitalWrite(RED2, HIGH);    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Transition to yellow for traffic light 3 */
    lcd.setCursor(0,1);
    lcd.print("State: Y");
    digitalWrite(YELLOW2, HIGH);
    digitalWrite(RED2, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Transition to green for traffic light 3 */
    digitalWrite(YELLOW2, LOW);
    digitalWrite(GREEN2, HIGH);
    lcd.setCursor(0,1);
    lcd.print("State: G");
    lcd.setCursor(9,1);
    lcd.print("T:");

    /* Countdown the remaining time for the green light */
    for(int i= WaitingTime/1000; i>=0; i--){
      lcd.setCursor(11,1);
      lcd.print(i);
      lcd.print("s");
      if(i < 10){
        lcd.setCursor(13,1);
        lcd.print(" ");
      }
      if(i==0){
        lcd.setCursor(9,1);
        lcd.print(" ");
        lcd.print(" ");
        lcd.print(" ");
        lcd.print(" ");
        break;
      } 
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* Flash the green light and sound the buzzer */
    digitalWrite(GREEN2, LOW);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(BuzzerPin, HIGH);
    digitalWrite(GREEN2, HIGH);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(BuzzerPin, LOW);
    digitalWrite(GREEN2, LOW);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(BuzzerPin, HIGH);
    digitalWrite(GREEN2, HIGH);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    digitalWrite(BuzzerPin, LOW);
    digitalWrite(GREEN2, LOW);
    vTaskDelay(400/ portTICK_PERIOD_MS);
    digitalWrite(GREEN2, HIGH);
    digitalWrite(BuzzerPin, HIGH);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    /* Transition to yellow for traffic light 3 */
    lcd.setCursor(0,1);
    lcd.print("State: Y");
    digitalWrite(YELLOW2, HIGH);
    digitalWrite(GREEN2, LOW);
    digitalWrite(BuzzerPin, LOW);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Transition to red for traffic light 3 */
    lcd.setCursor(0,1);
    lcd.print("State: R");
    digitalWrite(RED2, HIGH);
    digitalWrite(YELLOW2, LOW);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    /* Give the semaphore to traffic1Task to start its execution again */
    xSemaphoreGive(xSemaphore1);
    taskYIELD();
  }
}


