#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <map>
#include <string>
#include <STM32FreeRTOS.h>
#include "../lib/ES_CAN/ES_CAN.h"

/*
skipped:
2.6: Synthesiser modules should be configurable as senders or receivers. easiest to do this with preprocessor directives (e.g. #ifdef), 
    but it may be more user friendly to allow the mode to be changed at runtime. 
    You could also support auto-configuration with the handshake signals to define roles in a multi-module keyboard. 
    There should also be a method of choosing the octave number of a module.
*/

//=========================================================PREPROCESSORS========================================
// comment these out as appropriate
// #define DISABLE_THREADS
// #define DISABLE_SOUND_ISR
// #define DISABLE_CAN_ISR
// #define TEST_SCANKEYS
// #define TEST_DISPLAY

//===========================================================Constants==================================================================
  const bool loopbackState = true;
  const uint32_t interval = 100; //Display update interval
  const double baseFreq = 440.0;
  const double TemperamentRatio = std::pow(2.0, 1.0/12.0);
  const uint32_t sampleFreq = 22000;
  const uint32_t stepSizes[] = {
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -9.0))) / sampleFreq), // C
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -8.0))) / sampleFreq),  // C#
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -7.0))) / sampleFreq),  // D
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -6.0))) / sampleFreq),  // D#
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -5.0))) / sampleFreq),  // E
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -4.0))) / sampleFreq),  // F
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -3.0))) / sampleFreq),  // F#
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -2.0))) / sampleFreq),  // G
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, -1.0))) / sampleFreq),  // G#
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, 0.0))) / sampleFreq),   // A  
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, 1.0))) / sampleFreq),  // A# 
    static_cast<uint32_t>(((1LL << 32) * (baseFreq * std::pow(TemperamentRatio, 2.0))) / sampleFreq)  // B
  };
  std::map<uint32_t, std::string> notesMap = {{0, "C"}, 
                                              {1, "C#"}, 
                                              {2, "D"}, 
                                              {3, "D#"}, 
                                              {4, "E"}, 
                                              {5, "F"}, 
                                              {6, "F#"}, 
                                              {7, "G"}, 
                                              {8, "G#"}, 
                                              {9, "A"}, 
                                              {10, "A#"}, 
                                              {11, "B"}
                                              };

//==========================================================Pin definitions===================================================================
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

//=======================================================Global variables==========================================================
  volatile uint32_t currentStepSize[10] = {0}; // Polyphonycchange. most people have 10 fingers
  uint8_t currentStepSizeLength = sizeof(currentStepSize)/sizeof(currentStepSize[0]);
  QueueHandle_t msgInQ;               // queue handler for receiver
  QueueHandle_t msgOutQ;              // queue handler for transmission
  SemaphoreHandle_t CAN_TX_Semaphore; // transmit thread will use a semaphore to check when it can place a message in the outgoing mailbox

// store system state that is used in more than one thread
struct {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;  
  uint8_t RX_Message[8]={0};
} sysState;

//============================================Function to set outputs using key matrix=====================================
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

//=========================================================Key selection=================================================
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

//=======================================================KNOB CLASS=======================================================
// Functionality:
// Updates the rotation value using the latest state of the quadrature inputs
// Sets upper and lower limits
// Reads the current rotation value
class Knob { 
  private:
      int8_t limUpper;
      int8_t limLower;
      struct {
        int8_t delta;
        std::bitset<4> stateTransition; 
        std::bitset<2> prevState;
        int8_t rotation;
        SemaphoreHandle_t mutex;
      } knobState;
  public:
      // Constructor
      Knob(int initialValue, int upperLimit, int lowerLimit) {
          
          limUpper = upperLimit;
          limLower = lowerLimit;
          knobState.stateTransition = 0;
          knobState.prevState = 0;
          knobState.rotation = initialValue;
          knobState.mutex = xSemaphoreCreateMutex();
      }

      // Function to update the knob value
      void updateRotation(std::bitset<2> currentState) { //delta to be +-1, the latest state of the quadrature inputs
          xSemaphoreTake(knobState.mutex, portMAX_DELAY);  // Lock the mutex (when out of scope it unlocks)
          knobState.stateTransition[3] = knobState.prevState[1];
          knobState.stateTransition[2] = knobState.prevState[0];
          knobState.stateTransition[1] = currentState[1];
          knobState.stateTransition[0] = currentState[0];

          switch (knobState.stateTransition.to_ulong()){
            case 0b0001: 
              knobState.delta = 1;
              break;
            case 0b0100:
              knobState.delta = -1;
              break;
            case 0b1011: 
              knobState.delta = -1;
              break;
            case 0b1110: 
              knobState.delta = 1;
              break;
            // "Impossible" transitions. no change to delta
            case 0b0011:
              break;
            case 0b0110:
              break;
            case 0b1001:
              break;
            case 0b1100:
              break;
            default:
              knobState.delta = 0;
              break;
          }
          //Assign the current state to the previous state once decoding is complete
          knobState.prevState[0] = currentState[0];
          knobState.prevState[1] = currentState[1];
          knobState.rotation += knobState.delta;

          // Check and limit the knob value within the specified range
          if (knobState.rotation > limUpper) {
              knobState.rotation = limUpper;
          } 
          else if (knobState.rotation < limLower) {
              knobState.rotation = limLower;
          }
          xSemaphoreGive(knobState.mutex);
      }

      // Function to read the current knob value
      int8_t readRotation() const {
          int8_t currentRotation = __atomic_load_n(&knobState.rotation, __ATOMIC_RELAXED);
          return currentRotation;
      }
};

// Knobs
Knob knob3(0, 8, 0); // volume
Knob knob2(0, 8, 0); // octave

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// =================================================Interrupt services======================================================
void sampleISR() {  //polyphony mega changes
  static uint32_t phaseAcc[10] = {0};
  int32_t Vout[10] = {0};
  int32_t Vout_polyphony;
  for (int i = 0; i < currentStepSizeLength; i++){
    phaseAcc[i] += currentStepSize[i];  
    Vout[i] = (phaseAcc[i] >> 24) - 128;

    if (Vout_polyphony+Vout[i] > 4294967295){         // cutoff
      Vout_polyphony = 4294967295;
    }
    else{
      Vout_polyphony += Vout[i];
    }
  }
  int8_t knob3Rotation = knob3.readRotation();
  Vout_polyphony = Vout_polyphony >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, Vout_polyphony + 128);
}

// Incoming CAN messages will be written into the queue in this ISR
void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// ISR which will give the semaphore each time a mailbox becomes available
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

// ======================================================Thread functions=================================================
// scans 12 keys + knobs
void scanKeysTask(void * pvParameters) {
  // local vars
  // bad coding practise used for sysState.RX_Message[4]<<8) +sysState.RX_Message[3] :)))) Polyphony
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<4> cols;
  std::bitset<2> prevState = 0b00;
  std::bitset<2> currentState = 0b00;
  uint32_t localCurrentStepSize;
  int8_t rotation = 0;
  std::bitset<4> combinedBits;  // what a terrible name. for prev and current rotation
  int8_t rotationVariable;
  uint8_t TX_Message[8]={0}; // stores an outgoing message

  while (1){
    #ifndef TEST_SCANKEYS
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #endif

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
        #ifdef TEST_SCANKEYS
        cols[j] = 1;
        #endif
        sysState.inputs[4*i+j] = cols[j];
        // if (cols[j]==0){
        //   localCurrentStepSize = stepSizes[4*i+j];            //Polyphony: may need to change this
        //   TX_Message[2] = 4*i+j; // store	Note number 0-11
        // }
      }
      // Polyphony: if a key is pressed, that bit == 1. This is really the last 12 bits of sysState.inputs
      TX_Message[3] = static_cast<uint8_t>(~(sysState.inputs.to_ulong() & 0xFF));        // extract the last 8 bits
      TX_Message[4] = static_cast<uint8_t>((~(sysState.inputs.to_ulong()>>8))& 0x0F);   // extract the next 8 bits
      xSemaphoreGive(sysState.mutex);
    }
    // Polyphony: may need to change this for chords. Imagine needing local data could never be me trollolololol
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    // __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);

    // store state of key press or releases
    if ((TX_Message[4]<<8) + TX_Message[3] != 0){
      TX_Message[0] = 'P';   // converts to 0x50 for P (key pressed), and 0x52 for R (Key released). ASCII
    }
    else{
      TX_Message[0] = 'R';
    }

    // knob 3 and 2
    setRow(3);
    cols = readCols();
    currentState[0] = cols[0];
    currentState[1] = cols[1];
    // update rotation  
    knob3.updateRotation(currentState);
    currentState[0] = cols[2];
    currentState[1] = cols[3];
    knob2.updateRotation(currentState);
    if ((TX_Message[4]<<8) + TX_Message[3] != 0){ // only update when a key is pressed. Polyphony changes were made.
        TX_Message[1] = knob2.readRotation(); // 	store Octave number 0-8
    }
    // CAN_TX(0x123, TX_Message); // send the message over the bus. CAN message ID is fixed as 0x123 
    /*
    If you look in the library you’ll see that the above function (CAN_TX(0x123, TX_Message);) polls until there is space in the outgoing mailbox to place the message in. 
    So if you send a lot of messages at the same time, scanKeysTask() might get stuck waiting for the bus to become available. 
    It also not a thread safe function and its behaviour could be undefined if messages are being sent from two different threads. 
    A queue is useful here too so that messages can be queued up for transmission.
    */
   //TODO: should uodate only when there is a change
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY); // RTOS function to place an item on the transmit queue

    #ifdef TEST_SCANKEYS
      break;
    #endif
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  uint16_t pressedNotes = (sysState.RX_Message[4]<<8) +sysState.RX_Message[3]; // in the form 0b0100... 
  while (1){
    #ifndef TEST_DISPLAY
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #endif

    //Update display
    u8g2.clearBuffer();                                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);                 // choose a suitable font
    u8g2.drawStr(2,10,"Hello darkness my old friend");  // write something to the internal memory

    // keys
    u8g2.setCursor(2,20);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);      // to guard access to sysState
    u8g2.print(sysState.inputs.to_ulong(),HEX); 
    xSemaphoreGive(sysState.mutex);

    // knob 2
    u8g2.setCursor(30,20);
    u8g2.print(knob2.readRotation(), DEC);

    // knob 3
    u8g2.setCursor(40,20);
    u8g2.print(knob3.readRotation(), DEC);

    // display press/release, octave and note number
    u8g2.setCursor(66,30);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);  
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    // u8g2.print(sysState.RX_Message[2]);
    u8g2.print(pressedNotes, DEC);
    xSemaphoreGive(sysState.mutex);

    // display note played. THis no longer works after polyphony
    u8g2.setCursor(2,30);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY); 
    for (int i=0; i < 12; i++){
      if (pressedNotes%2==1){
        u8g2.print(notesMap[i].c_str());
      }
      pressedNotes = pressedNotes >> 1;
    }
    xSemaphoreGive(sysState.mutex);

    // transfer internal memory to the display
    u8g2.sendBuffer();

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    #ifdef TEST_DISPLAY
    break;
    #endif
  }
}

// Polyphony mega changes. for loops added for updating currentStepSize, which is now an array
void decodeTask(void * pvParameters){
  uint32_t localCurrentStepSize;
  uint8_t local_RX_Message[8]={0};
  uint16_t keyState = 0;
  uint8_t currentStepSizeIndex;
  while(1){
    localCurrentStepSize = 0;
    xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);
    // see if this is a bad use of mutex, cuz there's another atomic store in here. Can I not just use local_RX_Message for the conditionals lol
    xSemaphoreTake(sysState.mutex, portMAX_DELAY); 
    for (int i = 0; i < 8; i++) {
      sysState.RX_Message[i] = local_RX_Message[i];
    }

    keyState = (sysState.RX_Message[4]<<8) + sysState.RX_Message[3]; // in the form 0b00100001

    if (sysState.RX_Message[0] == 'R'){
      currentStepSizeIndex = 0;
      for (int i = 0; i < currentStepSizeLength-1; i++){ // need to go through whole thing
        currentStepSize[i] = 0;
      }
    }
    else {
      // Changes octave.
      // Polyphony: does not use [2] anymore, will have to change depending on how bytes are used
      // PS this is some horrible code and I feel closer to death for every passing minute
      currentStepSizeIndex = 0;
      for (int i = 0; i < 12; i++){
        if (keyState%2==1){
          if (sysState.RX_Message[1]-4 < 0){
            localCurrentStepSize = stepSizes[i] >> (4-sysState.RX_Message[1]); 
            __atomic_store_n(&currentStepSize[currentStepSizeIndex], localCurrentStepSize, __ATOMIC_RELAXED);
            __atomic_load(&currentStepSize[currentStepSizeIndex], &localCurrentStepSize, __ATOMIC_RELAXED);
          }
          else if (sysState.RX_Message[1]-4 >= 0){
            localCurrentStepSize = stepSizes[i] << (sysState.RX_Message[1]-4);
            __atomic_store_n(&currentStepSize[currentStepSizeIndex], localCurrentStepSize, __ATOMIC_RELAXED);
            __atomic_load(&currentStepSize[currentStepSizeIndex], &localCurrentStepSize, __ATOMIC_RELAXED);
          }
          currentStepSizeIndex += 1;
          if (currentStepSizeIndex > 9){
            currentStepSizeIndex = 9;     // I am reluctant to ever increase the size of this array, as it could be bad
          }
        }
        keyState = keyState >> 1;
      }
      //what is Vmax tho??
    }
    xSemaphoreGive(sysState.mutex);
  }
}

void CAN_TX_Task (void * pvParameters) {              // two blocking statements because it must obtain a message from the queue and take the semaphore before sending the message
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);    // obtain a message from the queue
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);  // and take the semaphore before sending the message
		CAN_TX(0x123, msgOut);
	}
}

//====================================================SETUP=====================================================

// determine the amount of stack to allocate to a thread:
// uxTaskGetStackHighWaterMark() returns the largest amount of stack that a thread has ever needed. 
// You can allocate a large stack at first and then optimise when the code is working. 
// You need to ensure that all the code has been exercised before you report the stack high water mark.
void setup() {
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
  Serial.println("Hello world!");

  // Create the mutex for sysState
  sysState.mutex = xSemaphoreCreateMutex();

  // initialise CAN bus
  CAN_Init(loopbackState);    //True: loopback mode. receive and acknowledge its own messages. 
                              // False: disables loopback mode and allow the MCUs to receive each others’ messages.
  // why 7ff??
  setCANFilter(0x123,0x7ff);  //initialises the reception ID filter. only messages with the ID 0x123 will be received. 
                              // The second parameter is the mask, and 0x7ff means that every bit of the ID must match the filter for the message to be accepted
  #ifndef DISABLE_CAN_ISR
  CAN_RegisterRX_ISR(CAN_RX_ISR); // called whenever a CAN message is received by passing a pointer to the relevant library function.
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  CAN_Start();

  // initialise queue handler for CAN bus msgs
  msgInQ = xQueueCreate(36,8);        // Minimum time to send a CAN frame is ~0.7ms, so a queue length of 36 will take at least 25ms to fill; 
                                      // second param: size of each item in BYTES.
  #ifdef TEST_SCANKEYS
  msgOutQ = xQueueCreate(386, 8);
  #endif
  #ifndef TEST_SCANKEYS
  msgOutQ = xQueueCreate(36, 8);      // 36 normal. 386 for TEST_SCANKEYS
  #endif

  //  maxCount == 3 so it can’t accumulate a count greater than the number of mailboxes. Why need both init and max? why not assume init is max?
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); // initialised to 3, so can be taken 3 times by transmit thread, and blocked on the fourth attempt.

  // For sound. Its the timer
  #ifndef DISABLE_SOUND_ISR
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(sampleFreq, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  #endif

  // threads
  #ifndef DISABLE_THREADS
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		    /* Text name for the task */
    256,      		        /* Stack size in words, not bytes */
  NULL,			              /* Parameter passed into the task */
    1,			              /* Task priority */
    &displayUpdateHandle  /* Pointer to store the task handle */
  );

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		  /* Text name for the task */
    64,      		    /* Stack size in words, not bytes */
    NULL,			      /* Parameter passed into the task */
    4,			        /* Task priority */
    &scanKeysHandle /* Pointer to store the task handle */
  );

  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
    decodeTask,		    /* Function that implements the task */
    "decodeTask",		  /* Text name for the task */
    64,      		      /* Stack size in words, not bytes */
    NULL,			        /* Parameter passed into the task */
    3,			          /* Task priority 25.2 ms*/
    &decodeTaskHandle /* Pointer to store the task handle */
  );

  TaskHandle_t CAN_TX_TaskHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,		    /* Function that implements the task */
  "CAN_TX_Task",		    /* Text name for the task */
    64,      		        /* Stack size in words, not bytes */
    NULL,			          /* Parameter passed into the task */
    2,			            /* Task priority. 60ms*/
    &CAN_TX_TaskHandle  /* Pointer to store the task handle */
  );
  vTaskStartScheduler();
  #endif

  // test
  #ifdef TEST_SCANKEYS
    uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++) {
      displayUpdateTask(NULL);
    }
    uint32_t execution_time = micros() - startTime;
    Serial.print("Execution time in microseconds: ");
    Serial.println(execution_time);
    while(1);
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
}