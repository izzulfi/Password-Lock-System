#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
// #include <Arduino_FreeRTOS.h>
// #include <semphr.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

#define normal 1
#define salah 2
#define benar 3

//SemaphoreHandle_t Mutex1; 
SemaphoreHandle_t xSerialSemaphore;
int state = normal;
char Keypad (void);
void Task1( void *pvParameters);
void Task2( void *pvParameters);
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
void setup() {
  // put your setup code here, to run once:
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  xTaskCreate(Task1,  (const portCHAR *)"Task 1", 200,  NULL,  1  ,  NULL );
  xTaskCreate(Task2,  (const portCHAR *)"Task 2", 200,  NULL,  1  ,  NULL );
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void Task1( void *pvParameters) {
    (void) pvParameters;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    pinMode (23, INPUT_PULLUP);
    pinMode (24, INPUT_PULLUP);
    pinMode (25, INPUT_PULLUP);
    pinMode (26, INPUT_PULLUP);
    pinMode (27, OUTPUT);
    pinMode (28, OUTPUT);
    pinMode (29, OUTPUT);
    lcd.begin(16, 2);
    boolean change = false;
    int i; char key[10]; int count = 0; int wrong = 0;char waktu[10];
    char init_pass[6] = {'1','2','3','4','5'};
    char pass[10]; char menu; char change_pass[6];
    if (EEPROM.read(0) == NULL) {
      vTaskDelay (10 / portTICK_PERIOD_MS);
    for (i=0;i<=4;i++) {
        EEPROM.write (i, init_pass[i]);
        vTaskDelay ( 10 / portTICK_PERIOD_MS );  
      }
    }
    for (;;) {
        if (change == false) {
            lcd.clear();
            lcd.setCursor (3,0);
            lcd.print ("Input Pass");
            for (i=0;i<=5;i++) {
                key[i] = Keypad();
                while (key[i] == ' ') {
                    key[i] = Keypad();
                    vTaskDelay (150 / portTICK_PERIOD_MS);
                  }
                  if (i<=4) {
                    pass[i] = EEPROM.read(i);
                    vTaskDelay (10 / portTICK_PERIOD_MS);
                    if (key[i] == pass[i]) {
                        count++;
                      }
                      lcd.setCursor (i+5,1);
                      lcd.print ('*');
                    }else {
                        while (key[5] != '#') {
                            vTaskDelay (150 / portTICK_PERIOD_MS);
                            key[5] = Keypad();
                          }
                      }
              }
              if (count == 5) {
                  lcd.clear();
                  lcd.setCursor (3,0);
                  lcd.print ("Pass Benar");
                  vTaskDelay (1000 / portTICK_PERIOD_MS);
                  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 10 ) == pdTRUE )
                  {
                      state = benar;
                      vTaskDelay (100 / portTICK_PERIOD_MS);  
                      xSemaphoreGive( xSerialSemaphore );
                  }              
                  change = true;
                  count = 0;
                  wrong = 0;
                } else {
                    wrong++;
                    lcd.clear();
                    lcd.setCursor (3,0);
                    lcd.print ("Pass Salah");
                    vTaskDelay (1000 / portTICK_PERIOD_MS);
                    count = 0;
                    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 10 ) == pdTRUE )
                    {
                      state = salah;
                      vTaskDelay (100 / portTICK_PERIOD_MS); 
                      xSemaphoreGive( xSerialSemaphore );
                    }                
                  }
              if (wrong >= 3) {                         
                taskENTER_CRITICAL();
                {
                state = normal;
                vTaskDelay (100 / portTICK_PERIOD_MS); 
                    for (i=0;i<=21;i++) {
                        vTaskDelay ( 1000 / portTICK_PERIOD_MS );
                        lcd.setCursor (3,0);
                        lcd.print ("Sistem Idle");
                        lcd.setCursor (8,1);                       
                        // waktu = i+ '0';
                        lcd.print (itoa(i, waktu, 10));
                      }
                      wrong = 0;
               }
               taskEXIT_CRITICAL();                 
                }
                
          }
           if (change == true) {
              lcd.clear();
              lcd.setCursor (0,0);
              lcd.print ("1 : Input Pass");
              lcd.setCursor (0,1);
              lcd.print ("2 : Change Pass");
              menu = Keypad();
              vTaskDelay ( 150 / portTICK_PERIOD_MS );
              if (menu == '1') {
                  change = false;
                }else if (menu == '2') {
                    lcd.clear();
                    lcd.setCursor (3,0);
                    lcd.print ("Change Pass");
                    for (i=0;i<=5;i++) {
                        change_pass[i] = Keypad();
                        while (change_pass[i] == ' ') {                       
                          change_pass[i] = Keypad();
                          vTaskDelay (150 / portTICK_PERIOD_MS);
                          }
                          if (i<=4) {
                              EEPROM.write (i, change_pass[i]);
                              vTaskDelay (10 / portTICK_PERIOD_MS);
                              lcd.setCursor (i+5,1);
                              lcd.print ("*");                             
                            } else {
                                while (change_pass[5] != '#') {                                  
                                  change_pass[5] = Keypad();
                                  vTaskDelay (150 / portTICK_PERIOD_MS);
                                  }
                            }
                      }
                  }
            } 
      }
  }

void Task2 (void *pvParameters) {
    (void) pvParameters;
    int kondisi; int nextstate;
    pinMode (14, OUTPUT);
    for (;;) {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 10 ) == pdTRUE )
      {
          kondisi = state;
          xSemaphoreGive( xSerialSemaphore );
      }
        switch (kondisi) {
            case (normal) :
              digitalWrite (14, LOW);
              nextstate = state; break;
            case (salah) :
              digitalWrite (14, HIGH);
              vTaskDelay( 100 / portTICK_PERIOD_MS );
              digitalWrite (14, LOW);
              vTaskDelay( 100 / portTICK_PERIOD_MS );
              nextstate = state; break;
            case (benar) :
              digitalWrite (14, HIGH);
              vTaskDelay( 1000 / portTICK_PERIOD_MS );
              digitalWrite (14, LOW);
              nextstate = normal; break;
          }
          kondisi = nextstate;
          if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 10 ) == pdTRUE )
          {
          state = kondisi;
          xSemaphoreGive( xSerialSemaphore );
      }
      }
  }

char Keypad (void) {
    PORTA = 0b11011111;
    if (digitalRead(23) == LOW) { 
      return '1';
    } else if (digitalRead (24) == LOW) {
      return '4';
    } else if (digitalRead (25) == LOW) {
      return '7';
    } else if (digitalRead (26) == LOW) {
      return '*';
    }
    PORTA = 0b10111111;
    if (digitalRead(23) == LOW) {
      return '2';
    } else if (digitalRead (24) == LOW) { 
      return '5';
    } else if (digitalRead (25) == LOW) {
      return '8';
    } else if (digitalRead (26) == LOW) {
      return '0';
    }
    PORTA = 0b01111111;
    if (digitalRead(23) == LOW) {
      return '3';
    } else if (digitalRead (24) == LOW) { 
      return '6';
    } else if (digitalRead (25) == LOW) {
      return '9';
    } else if (digitalRead (26) == LOW) {
      return '#';
    }
    return ' ';
  }
