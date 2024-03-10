#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <map>
#include <string>
#include <cstring> // for memcpy
#include <STM32FreeRTOS.h>
#include "../lib/ES_CAN/ES_CAN.h"

/*
skipped:
knob class
2.6: Synthesiser modules should be configurable as senders or receivers. easiest to do this with preprocessor directives (e.g. #ifdef), 
    but it may be more user friendly to allow the mode to be changed at runtime. 
    You could also support auto-configuration with the handshake signals to define roles in a multi-module keyboard. 
    There should also be a method of choosing the octave number of a module.
*/

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes [] = {51076056, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007189, 96418755};

  // Add information to the OLED display to show which note is selected.
  std::map<uint32_t, std::string> notesMap = {
    {51076056, "C"}, 
    {54113197, "C#"}, 
    {57330935, "D"}, 
    {60740010, "D#"}, 
    {64351799, "E"}, 
    {68178356, "F"}, 
    {72232452, "F#"}, 
    {76527617, "G"}, 
    {81078186, "G#"}, 
    {85899346, "A"}, 
    {91007189, "A#"}, 
    {96418755, "B"}
  };

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

// global variables
  volatile uint32_t currentStepSize;
  QueueHandle_t msgInQ; // queue handler for receiver
  QueueHandle_t msgOutQ; // queue handler for transmission
  SemaphoreHandle_t CAN_TX_Semaphore; // transmit thread will use a semaphore to check when it can place a message in the outgoing mailbox

// store system state that is used in more than one thread
struct {
  std::bitset<32> inputs;
  int8_t rotation;  // maybe i shld have more rotations?
  SemaphoreHandle_t mutex;  
  uint8_t RX_Message[8]={0}; // Does this only initialise or does it constantly get set to 0?
} sysState;

// TODO make a knob class
// Update the rotation value using the latest state of the quadrature inputs
// Set upper and lower limits
// Read the current rotation value
// class Knob {
//   private:
//     std::bitset<2> prevState;
//     std::bitset<4> combinedBits;
//     int8_t rotation; // I dont like this
//     int8_t rotationVariable;
//     int8_t upperLimit;
//     int8_t lowerLimit;
//   public: 
//     Knob(int lower, int upper): prevState(0b00), rotation(0), rotationVariable(0), upperLimit(upper), lowerLimit(lower){
//     }

//     void setLimits(int newLower, int newUpper){
//       upperLimit = newUpper;
//       lowerLimit = newLower;
//     }

//     // int8_t readRotation(){ // sth wrong
//     //   xSemaphoreTake(sysState.mutex, portMAX_DELAY);  
//     //   rotation = sysState.rotation;
//     //   xSemaphoreGive(sysState.mutex);
//     //   return rotation;
//     // }

//     void updateRotation(std::bitset<4> cols){
//       combinedBits[3] = prevState[1];
//       combinedBits[2] = prevState[0];
//       combinedBits[1] = cols[1];
//       combinedBits[0] = cols[0];
//       switch (combinedBits.to_ulong()){
//         case 0b0001: 
//           rotationVariable = 1;
//           break;
//         case 0b0100:
//           rotationVariable = -1;
//           break;
//         case 0b1011: 
//           rotationVariable = -1;
//           break;
//         case 0b1110: 
//           rotationVariable = 1;
//           break;
//         // "Impossible" transitions. no change to rotation
//         case 0b0011:
//           break;
//         case 0b0110:
//           break;
//         case 0b1001:
//           break;
//         case 0b1100:
//           break;
//         default:
//           rotationVariable = 0;
//           break;
//       }
//       //Assign the current state to the previous state once decoding is complete
//       prevState[0] = cols[0];
//       prevState[1] = cols[1];
//       // update rotation  
//       xSemaphoreTake(sysState.mutex, portMAX_DELAY);  
//       if (sysState.rotation + rotationVariable < upperLimit && sysState.rotation + rotationVariable >= lowerLimit) { // upper and lower limits
//         sysState.rotation += rotationVariable;
//       }
//       xSemaphoreGive(sysState.mutex);
//     }
// };

// // Knobs
// Knob knob3(0,9);

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

// Interrupt service
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  int8_t knob3Rotation = __atomic_load_n(&sysState.rotation, __ATOMIC_RELAXED);
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, Vout + 128);
}

// Incoming CAN messages will be written into the queue in an ISR
void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

std::bitset<4> readCols(){
  std::bitset<4> result;

  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);

  digitalWrite(RA0_PIN,rowIdx & 0x01);
  digitalWrite(RA1_PIN,rowIdx & 0x02);
  digitalWrite(RA2_PIN,rowIdx & 0x04);

  digitalWrite(REN_PIN,HIGH);
}

// scans 12 keys + knob 3
void scanKeysTask(void * pvParameters) {
  // local vars
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<4> cols;
  std::bitset<2> prevState = 0b00;
  uint32_t localCurrentStepSize;
  int8_t rotation = 0;
  std::bitset<4> combinedBits;  // what a terrible name. for prev and current rotation
  int8_t rotationVariable;
  uint8_t TX_Message[8]={0}; // stores an outgoing message

  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    // need this so the keyboard stops screaming if a key is not pressed
    localCurrentStepSize = 0;
    for (int i=0; i<3; i++){
      setRow(i);
      delayMicroseconds(3);
      cols = readCols();

      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      // should use switch cases, this is a bad implementation
      // also can MAYBE use the C function memcpy() or C++ std::copy for this purpose. 
      for (int j=0; j<4; j++){
        sysState.inputs[4*i+j] = cols[j];
        if (cols[j]==0){
          localCurrentStepSize = stepSizes[4*i+j];
          TX_Message[2] = 4*i+j; // store	Note number 0-11
        }
      }
      xSemaphoreGive(sysState.mutex);
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);

    // store state of key press or releases
    if (localCurrentStepSize == 0){
      TX_Message[0] = 'R';   // converts to 0x50 for P (key pressed), and 0x52 for R (Key released). ASCII
    }
    else{
      TX_Message[0] = 'P';
    }

    // knob 3
    setRow(3);
    cols = readCols();
    combinedBits[3] = prevState[1];
    combinedBits[2] = prevState[0];
    combinedBits[1] = cols[1];
    combinedBits[0] = cols[0];
    switch (combinedBits.to_ulong()){
      case 0b0001: 
        rotationVariable = 1;
        break;
      case 0b0100:
        rotationVariable = -1;
        break;
      case 0b1011: 
        rotationVariable = -1;
        break;
      case 0b1110: 
        rotationVariable = 1;
        break;
      // "Impossible" transitions. no change to rotation
      case 0b0011:
        break;
      case 0b0110:
        break;
      case 0b1001:
        break;
      case 0b1100:
        break;
      default:
        rotationVariable = 0;
        break;
    }
    //Assign the current state to the previous state once decoding is complete
    prevState[0] = cols[0];
    prevState[1] = cols[1];
    // update rotation  
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);  
    if (sysState.rotation + rotationVariable < 9 && sysState.rotation + rotationVariable >= 0) { // upper and lower limits
      sysState.rotation += rotationVariable;
      if (localCurrentStepSize != 0){ // only update when a key is pressed
        TX_Message[1] = sysState.rotation; // 	store Octave number 0-8
      }
    }
    xSemaphoreGive(sysState.mutex);
    // CAN_TX(0x123, TX_Message); // send the message over the bus. CAN message ID is fixed as 0x123 
    /*
    If you look in the library you’ll see that the above function (CAN_TX(0x123, TX_Message);) polls until there is space in the outgoing mailbox to place the message in. 
    So if you send a lot of messages at the same time, scanKeysTask() might get stuck waiting for the bus to become available. 
    It also not a thread safe function and its behaviour could be undefined if messages are being sent from two different threads. 
    A queue is useful here too so that messages can be queued up for transmission.
    */
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY); // RTOS function to place an item on the transmit queue
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  // uint8_t RX_Message[8]={0};
  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello darkness my old friend");  // write something to the internal memory

    // keys
    u8g2.setCursor(2,20);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY); // to guard access to sysState
    u8g2.print(sysState.inputs.to_ulong(),HEX); 
    xSemaphoreGive(sysState.mutex);

    // knob 3
    u8g2.setCursor(30,20);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);  
    u8g2.print(sysState.rotation, DEC);
    xSemaphoreGive(sysState.mutex);
    // u8g2.print(knob3.readRotation()); // reads current rotation value

    // display octave and note number
    // while (CAN_CheckRXLevel()){ 
	  //   CAN_RX(ID, RX_Message);
    // }
    u8g2.setCursor(66,30);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);  
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);
    xSemaphoreGive(sysState.mutex);

    // note played
    u8g2.setCursor(2,30);
    u8g2.print(notesMap[static_cast<uint32_t>(currentStepSize)].c_str());

    // transfer internal memory to the display
    u8g2.sendBuffer();

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters){
  uint32_t localCurrentStepSize;
  uint8_t local_RX_Message[8]={0};
  while(1){
    xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);

    // see if this is a bad use of mutex, cuz there's another atomic store in here. Can I not just use local_RX_Message for the conditionals lol
    xSemaphoreTake(sysState.mutex, portMAX_DELAY); 
    for (int i = 0; i < 8; i++) {
      sysState.RX_Message[i] = local_RX_Message[i];
    }
    if (sysState.RX_Message[0] == 'R'){
      currentStepSize = 0;
    }
    else {
      // changes octaves. Once knob classes are done, [1] should not be linked to volume
      if (sysState.RX_Message[1]-4 < 0){
        localCurrentStepSize = stepSizes[sysState.RX_Message[2]] >> (4-sysState.RX_Message[1]);
      }
      else{
        localCurrentStepSize = stepSizes[sysState.RX_Message[2]] << (sysState.RX_Message[1]-4);
      }
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }
    xSemaphoreGive(sysState.mutex);
  }
}

void CAN_TX_Task (void * pvParameters) { // two blocking statements because it must obtain a message from the queue and take the semaphore before sending the message
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY); // obtain a message from the queue
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY); // and take the semaphore before sending the message
		CAN_TX(0x123, msgOut);
	}
}

// ISR which will give the semaphore each time a mailbox becomes available
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
// determine the amount of stack to allocate to a thread:
// uxTaskGetStackHighWaterMark() returns the largest amount of stack that a thread has ever needed. 
//You can allocate a large stack at first and then optimise when the code is working. 
// You need to ensure that all the code has been exercised before you report the stack high water mark.

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // Create the mutex for sysState
  sysState.mutex = xSemaphoreCreateMutex();

  // initialise CAN bus
  CAN_Init(true); //True: loopback mode. receive and acknowledge its own messages. 
                  // False: disables loopback mode and allow the MCUs to receive each others’ messages.
  setCANFilter(0x123,0x7ff);  //initialises the reception ID filter. only messages with the ID 0x123 will be received. 
                              // The second parameter is the mask, and 0x7ff means that every bit of the ID must match the filter for the message to be accepted
  CAN_RegisterRX_ISR(CAN_RX_ISR); // called whenever a CAN message is received by passing a pointer to the relevant library function.
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  // initialise queue handler for CAN bus msgs
  msgInQ = xQueueCreate(36,8); // Minimum time to send a CAN frame is ~0.7ms, so a queue length of 36 will take at least 25ms to fill; 
                                // second param: size of each item in BYTES.
  msgOutQ = xQueueCreate(36, 8);
  //  maxCount == 3 so it can’t accumulate a count greater than the number of mailboxes. Why need both init and max? why not assume init is max?
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); // initialised to 3, so can be taken 3 times by transmit thread, and blocked on the fourth attempt.

  // uncomment for sound. Its the timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // threads
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateHandle /* Pointer to store the task handle */
  );

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    4,			/* Task priority */
    &scanKeysHandle /* Pointer to store the task handle */
  );

  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decodeTask",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority 25.2 ms*/
    &decodeTaskHandle /* Pointer to store the task handle */
  );

  TaskHandle_t CAN_TX_TaskHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CAN_TX_Task",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority. 60ms*/
    &CAN_TX_TaskHandle /* Pointer to store the task handle */
  );

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}