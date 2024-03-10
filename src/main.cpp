#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <map>
#include <string>
#include <STM32FreeRTOS.h>

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
  volatile uint8_t TX_Message[8] = {0}; // stores an outgoing message

// store system state that is used in more than one thread
struct {
  std::bitset<32> inputs;
  int8_t rotation;  // maybe i shld have more rotations?
  SemaphoreHandle_t mutex;  
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
        }
      }
      xSemaphoreGive(sysState.mutex);
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);

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
    }
    xSemaphoreGive(sysState.mutex);
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

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

    // note played
    u8g2.setCursor(2,30);
    u8g2.print(notesMap[static_cast<uint32_t>(currentStepSize)].c_str());
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
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

  // Create the mutex 
  sysState.mutex = xSemaphoreCreateMutex();

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
    2,			/* Task priority */
    &scanKeysHandle /* Pointer to store the task handle */
  );

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}