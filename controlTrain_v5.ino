// ___________             .__       _________                __                .__  .__                
// \__    ___/___________  |__| ____ \_   ___ \  ____   _____/  |________  ____ |  | |  |   ___________ 
//   |    |  \_  __ \__  \ |  |/    \/    \  \/ /  _ \ /    \   __\_  __ \/  _ \|  | |  | _/ __ \_  __ \
//   |    |   |  | \// __ \|  |   |  \     \___(  <_> )   |  \  |  |  | \(  <_> )  |_|  |_\  ___/|  | \/
//   |____|   |__|  (____  /__|___|  /\______  /\____/|___|  /__|  |__|   \____/|____/____/\___  >__|   
//                       \/        \/        \/            \/                                  \/       
// By Julie Glasdam & David Holte

// DOKUMENTATION:
// 1. Vi sender en tom pakke (idle packet), som står og kører mens brugeren får lov at indtaste input
// 2. Vi skifter den tomme pakke ud med en pakke indeholdende brugerens input
// 3. Brugeren kan vælge at sende en pakke til et accessory istedet, og vil bruge promptet for input
// 4. accAddr_Calc() metoden udregner den korrekte addresse med en algoritme.
// 5. En IOA Sensor registrerer tog1 på ydrebanen og sender en packet til sporskiftet så tog2 kan køre ind på indersporet

// KOMMENTARER:
// 1. Alle code comments står på samme linje som den kode kommentaren er til og er indenteret for letlæselighed.
// 2. De forskellige funktionaliteter er separeret med en linje for hurtigt overblik.




// ÆNDRINGER:
// - Fjernet DIRPIN & LEDPIN
// - Fjernet sensorFlag i loop

//=================================================================================
#define OUTPIN                 4            // Output til tog/bane
#define SENSORPIN              7            // Input pin for the potentiometer

// Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT            0x8D         // 58usec pulse length 141 255 - 141 = 114
#define TIMER_LONG             0x1B         // 116usec pulse length 27 255 - 27  = 228
#define PREAMBLE               0            // state
#define SEPERATOR              1            // state
#define SENDBYTE               2            // state
#define MAXPACK                3            // Hvor mange typer a packets der må gemmes i struct-arrayet

// unsigned char = 0 to 255

unsigned int  sensorValue    = 0;           // IOA Sensor value
unsigned int  userInput      = 0xFFFF;      // initialiser variablen

// Variables used for State Machine:
unsigned char last_timer     = TIMER_SHORT; // store last timer value
unsigned char latency;                      //
unsigned char pulseType      = 0;           // used for short or long pulse
bool          second_isr     = false;       // pulse up or down
unsigned char state          = PREAMBLE;    // initialize first state
unsigned char preamble_count = 16;          // counter for preamble
unsigned char outbyte        = 0;           // 
unsigned char cbit           = 0x80;        // 
unsigned int  packetIndex    = 0;           // index for packets 
unsigned int  byteIndex      = 0;           // index for bits in byte

// Variables used for Packet Assembling:
unsigned char preample1;                    // preample part 1
unsigned char preample2;                    // preample part 2
unsigned char addr;                         // byte: adresse                                                 
unsigned char data;                         // byte: kommando                                                
unsigned char addr2          = 255;         // byte: adresse 2
unsigned char data2          = 0;           // byte: kommando 2
unsigned char checksum;                     // byte: checksum 
unsigned char inputType;                    // flag for accessory data byte
unsigned char locoAddr       = 36;          // default address
unsigned char locoSpeed      = 96;          // default speed
unsigned char locoDir;                      // direction for the locomotive
unsigned int  accAddr;                      // byte: accessory address 
unsigned char accData;                      // byte: accessory data
unsigned int  accAddrSens;                  // byte: accessory address for sensor
unsigned char accDataSens;                  // byte: accessory data for sensor



// Pointer to show conventional syntax
unsigned char *checksumPtr   = &checksum;   // pointer til checksum


// Struct of data for the packages to send
struct Packet 
{
   unsigned char data[7];                   // initializing array to use in struct
   unsigned char len;                       // length 
};

// Array struct of previous struct, each packet in array containing data and number of bytes to be send after preample
struct Packet packet[MAXPACK] = 
{ 
    { { 0xFF,     0, 0xFF, 0, 0, 0, 0}, 3},       // idle packet
    { { 0,        0, 0,    0, 0, 0, 0}, 3},       // packet 1
    { { 0,        0, 0,    0, 0, 0, 0}, 3}        // packet 2
}; 
                                

/*Setup Timer2.
Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
Returns the time load value which must be loaded into TCNT2 inside your ISR routine. */
void SetupTimer2()
{
  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timer clock = 16MHz/8 = 2MHz oder 0,5usec
  TCCR2A = 0; //page 203 - 206 ATmega328/P
  TCCR2B = 2; //Page 206
/*         bit 2     bit 1     bit0
            0         0         0       Timer/Counter stopped 
            0         0         1       No Prescaling
            0         1         0       Prescaling by 8
            0         0         0       Prescaling by 32
            1         0         0       Prescaling by 64
            1         0         1       Prescaling by 128
            1         1         0       Prescaling by 256
            1         1         1       Prescaling by 1024
*/
  TIMSK2 = 1<<TOIE2;                        // Timer2 Overflow Interrupt Enable - page 211 ATmega328/P   
  TCNT2  = TIMER_SHORT;                     // load the timer for its first cycle
}


int count = 0;

/* Capture the current timer value TCTN2. This is how much error we have due to interrupt latency and the work in this function. 
   Reload the timer and correct for latency. Timer2 overflow interrupt vector handler. Runs every 58(?) ms 
   
   - First run: outpacket is set to idle packet
   
   */
ISR(TIMER2_OVF_vect) 
{
     
// for every second interupt just toggle signal
  if (second_isr)                           
  {  
     digitalWrite(OUTPIN,1);
     second_isr = false;    
     latency    = TCNT2;                    // set timer to last value
     TCNT2      = latency + last_timer; 
  }
  else                                      
  {   
     digitalWrite(OUTPIN,0);
     second_isr = true;
     
     switch(state)  
     {
       case PREAMBLE: // --------------------------------
           // Send 1 - 16 times
           pulseType = 1;                   
           preamble_count--;
           
           // When 16 is sent
           if (preamble_count == 0)  
           {  
              state = SEPERATOR;            // advance to next state
              packetIndex++;                // get next message

              // Make sure the next part of the message to send is never "out of scope". Because switch case keeps looping?
              if (packetIndex >= MAXPACK)  
              {  
                packetIndex = 0; 
              }  
              byteIndex = 0;                // start msg with byte 0
           }
           break;
           
        case SEPERATOR: // --------------------------------
           pulseType = 0;                   // long pulse and then advance to next state
           state = SENDBYTE;                // goto next byte ...

           // packet[packetIndex] is the 1, 2 or 3 in last struct.
           // data[byteIndex] is which part of the message in 1, 2 or 3 
           outbyte = packet[packetIndex].data[byteIndex]; 
           cbit = 0x80;                     // send this bit next time first         
           break;
           
        case SENDBYTE: // --------------------------------
           // Iterate through byte to send, to get the seperate bits, by moving through cbit
           if ((outbyte & cbit)!=0)  
           { 
              pulseType = 1;                // send short pulse    
           }
           else  
           {
              pulseType = 0;                // send long pulse     
           }
           cbit = cbit >> 1;


           // When cbit is 0, the byte is over
           if (cbit == 0)                   
           {        
              byteIndex++; 

              // is there a next byte in the message 
              if (byteIndex >= packet[packetIndex].len)  
              {  
                // Not sending end bit, but preample is 1??
                 state = PREAMBLE;          // this was already the XOR byte then advance to preamble
                 preamble_count = 16;
              }

              // If there is another message go to a separator instead
              else                          // send separtor and advance to next byte
              {  
                 state = SEPERATOR;
              }
           }
           break;
     }   

     // Sending pulses
     if (pulseType)                         // data = 1 short pulse
     {  
        latency    = TCNT2;
        TCNT2      = latency + TIMER_SHORT;
        last_timer = TIMER_SHORT;
     }  
     else                                   // data = 0 long pulse
     {   
        latency    = TCNT2;
        TCNT2      = latency + TIMER_LONG; 
        last_timer = TIMER_LONG;
        
     } 
  }
  
}




void setup() 
{
    Serial.begin(9600);                     
    pinMode(OUTPIN,OUTPUT);                 // enable DCC output pin
    pinMode(SENSORPIN,INPUT_PULLUP);        // enable IOA sensor pin. Pullup for at undgå forstyrrelser (så den ikke læser unødvendigt input)
    SetupTimer2();                              
    addr2     = 0xFF;                       // default for idle packet
    data2     = 0;                          // default for idle packet
}


void loop() 
{
    // Get input from sensor to see if train is in place
    sensorValue = digitalRead(SENSORPIN);

    // Spørg bruger efter info om beskeder de vil sende til toget/bane   
    inputType = getUserInput(1, "Skriv 0 hvis du vil sende til Tog, og 1 hvis du vil sende til et Accessory");

    // Skriv til tog -----------------------------
    // Sets packet[1].data[...] and packet[2].data[...], but packet[2] is idle
    if (inputType == 0)                    
    {
        // Get the address, direction and speed, to create a packet to send. The bytes are set to decimal, not binary
        locoAddr   = getUserInput(127, "Vælg hvilket lokomotiv du vil addressere: (1-127).");
        locoDir    = getUserInput(1,   "Vælg hvilken vej du vil køre: (0-1).");
        locoSpeed  = getUserInput(31,  "Vælg hastighed: (2-15), Stop: (0), Emergency Stop: (1).");
    
        locoSpeed += 64;
        
        // What
        if(locoDir == 1)                    // forward direction
        {
          locoSpeed += 32;                  // ?
          addr       = locoAddr;            // userinput lægges ned i variablen
          data       = locoSpeed;           // userinput lægges ned i variablen
          addr2      = 255;                 // idle
          data2      = 0;                   // idle
        }
   
    }
    // Skriv til accessory -----------------------------
    // Sets packet[1].data[...] and packet[2].data[...]
    else if (inputType == 1)               
    {
        // Get the address, direction and speed, to create a packet to send. The bytes are set to decimal, not binary
        // Two first variables are set as parameters, and other variables are set in accAddr_Calc
        accAddr       = getUserInput(512, "Vælg adressen på sporskifte / lyssignal: (1-512).");
        accData       = getUserInput(1,   "0 = straight/grøn, 1 = turn/red: (0-1)."); 
        accAddr_Calc(accAddr, accData);     // her udregner vi accessory addressen
    }
    
    IOA_Sensor(sensorValue);                // Læser på IOA sensor input

    packet_Assembler();                     // assemble packet addressing train
    
    delay(20);                              

    addr  = locoAddr;                       // userinput lægges ned i variablen
    data  = locoSpeed;                      // userinput lægges ned i variablen
    addr2 = 255;                            // idle
    data2 = 0;                              // idle
    
    packet_Assembler();                     // assemble packet addressing accessory 
} 

unsigned int getUserInput(unsigned int max, const char packet[]) 
{
    unsigned int userInput = 0xFFFF;        // initialiser variablen
    
    Serial.println(packet);
    
    while (userInput > max) 
    {
        if (Serial.available() > 0)         // Check om der er input 
        {
           userInput = Serial.parseInt();   // parse int fra userinput
           Serial.print("Received  ");      // Fortæl brugeren at beskeden er modtaget
           Serial.print(userInput, DEC);
           Serial.println();
        }
    }
    return userInput;
}

// Put together the adress for the accessory packet. Params: Addressen (101-102), straight/turn (0-1)
void accAddr_Calc(unsigned int input, unsigned char flag) 
{
    unsigned int address;                   // address of accessory
    unsigned int x;                         // til at invertere bits   
    unsigned int y;                         // til at invertere bits
    unsigned char reg;                      // register
    unsigned char tmpAddr;                  // temporary variable for addr
    unsigned char tmpData;                  // temp var for data

    if ((input % 4) == 0)
    {
      address = (input / 4);
      reg     = (input % 4);
    } 
    else 
    {
      address = ((input / 4) + 1);    
      reg     = ((input % 4) - 1);
    }
      tmpAddr = address & 63;
      tmpAddr = tmpAddr + 128;
      tmpData = 128;
      x       = 0;
      y       = address & 64;
    
    if(y == 0)                              // invertere 3 højeste bits
    {
      x      += 64;
      y       = address & 128;
    }
    if(y == 0)        
    {
      x      += 128;
      y       = address & 256;  
    }
    if(y == 0)        
    {
      x      += 256;
      x       = x >> 2;
      tmpData+= x;      
      tmpData = tmpData + (reg << 1);
    }
    if(flag == 1)                           // true/false på grøn/rød eller straight/turn
    {
      tmpData+= 9;
      addr    = tmpAddr;
      data    = tmpData;
      addr2   = tmpAddr;                  
      data2   = tmpData - 8; 

     
    }
    else
    {
      tmpData+= 8;
      addr    = tmpAddr;
      data    = tmpData;
      addr2   = tmpAddr;
      data2   = tmpData - 8; 

   
    }
}

// Send packet to change lanes if train is in the way
void IOA_Sensor(int value)
{
  if (value == 0)
  {
    accAddrSens = 101;
    accDataSens = 0;
    accAddr_Calc(accAddrSens, accDataSens); 
  }
  else if (value == 1)
  {
    accAddrSens = 101;
    accDataSens = 1;
    accAddr_Calc(accAddrSens, accDataSens); 
  }
  packet_Assembler();
 
} 

// Take variables at the top and put them in structs
void packet_Assembler() 
{
   noInterrupts();                          // stop interrupten så man er sikker på intet er blevet scrambled undervejs operationerne

   // Packet 1:
   checksumPtr = addr ^ data;                  // Her kører vi en checksum operation der sammenligner de to bytes for fejl


   // Assemble packet 2:
   packet[1].data[0] = addr; 
   packet[1].data[1] = data;
   packet[1].data[2] = checksumPtr;

   // Packet 2:
   checksumPtr = addr2 ^ data2;                // Her kører vi en checksum operation der sammenligner de to bytes for fejl


   // Assemble packet 2:
   packet[2].data[0] = addr2; 
   packet[2].data[1] = data2;
   packet[2].data[2] = *checksumPtr;

   
   
   interrupts();                            // start interrupt igen
}
