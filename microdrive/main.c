
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include <string.h>
#include "sd_card.h"
#include "ff.h"
#include "ff_stdio.h"

typedef unsigned char byte;

/*
1A -> GPIO0 TRACK1
1B -> GPIO1 TRACK2 
2A -> GPIO2 COMMS_CLK
2B -> GPIO6 WR.PR
3A -> NC
3B -> NC
4A -> GPIO4 COMMS_IN
4B -> U2 9V
5A -> GPIO5 ERASE
5B -> GPIO3 R/W ON TEST QL Read High is 2.2V so connected directly without voltage divider
6A -> GND
6B -> GND
7A -> GND
7B -> GNDCOMMS_CLKLED
8A -> GND
*/

#define QL_SECTOR_SIZE          686
#define QL_HEADER_SIZE          28          // 28
#define QL_BLOCK_SIZE           538         // 12 PREAMBLE + 2 HEADER + 2 CHECKSUM + 8 PREAMBLE + 512 DATA + 2 CHECKSUM = 538 DATA LENGTH
#define QL_EXTRA_DATA           86          // 84 bytes + 2 checksum

#define QL_HEADER_GAP           3590        // 3590 to get 3600μs (3.6ms)
#define QL_SECTOR_GAP           7200        // 7200μs (7.2ms) is the sector gap on oqtadrive. Why?   //5490 to get 5520μs (5.52ms)

#define SELECT_IGNORE_TIME      100         // μs

// MDV FILE SPECIFICATION 
//#define MDV_SECTOR_SIZE 0x02AE          /* 686 */
#define MDV_FILE_HEADER_SIZE 0x10       /* 16 */
#define MDV_SECTOR_HEADER_SIZE 0x28     /* 40  preamble1 12 bytes + header 16 bytes + preamble2 12 bytes = 40*/
#define MDV_PREAMBLE_SIZE 0x0C          /* 12  */
#define MDV_SECTOR_DATA_SIZE 0x21A      /* 538 524 + 2 Checksum + 12 Preamble = 538 DATA LENGTH*/
#define SECTOR_FOOTER_SIZE 0x7A         /* 122 */
#define MDV_BLOCK_SIZE     538

#define NR_ONE_BITS 8

const int PIN_BUTTON_PREVIOUS = 8;
const int PIN_BUTTON_NEXT = 9;
const int PIN_BUTTON_ENTER = 10;
const int PIN_BUTTON_BACK = 11;

const int PIN_LED_GREEN = 12;
const int PIN_LED_2 = 13;
const int PIN_LED_3 = 14;
const int PIN_LED_4 = 15;

// Use this pin to send data from the PICO to the QL
const int PIN_TRISTATE_BUFFER_QL_READ = 26; // ACTIVE LOW - Use this pin to send data from the PICO to the QL - PURPLE WIRE
const uint32_t MASK_TRISTATE_BUFFER_QL_READ = 1 << PIN_TRISTATE_BUFFER_QL_READ;

// Use this pin to send data from the QL to the PICO
const int PIN_TRISTATE_BUFFER_QL_WRITE = 27;  //  ACTIVE LOW - Use this pin to send data from the QL to the PICO - YELLOW WIRE
const uint32_t MASK_TRISTATE_BUFFER_QL_WRITE = 1 << PIN_TRISTATE_BUFFER_QL_WRITE;

const int PIN_COMMS_IN    = 4;  // 4A -> GPIO4 COMMS_IN
const int PIN_WR_PROTECT  = 6;  // 2B -> GPIO6 WR.PR 
const int PIN_COMMS_CLK   = 2;  // 2A -> GPIO2 COMMS_CLK HIGH INACTIVE on IF1, LOW on QL; interrupt 
const int PIN_ERASE       = 5;  // 5A -> GPIO5 ERASE LOW active 
const int PIN_READ_WRITE  = 3;  // 5B -> GPIO3 R/W READ is HIGH; interrupt
const int PIN_TRACK_1     = 0;  // MDV_1A 
const int PIN_TRACK_2     = 1;  // MDV_1B

const uint32_t MASK_COMMS_CLK   = 1 << PIN_COMMS_CLK;
const uint32_t MASK_COMMS_IN    = 1 << PIN_COMMS_IN;
const uint32_t MASK_WR_PROTECT  = 1 << PIN_WR_PROTECT;
const uint32_t MASK_ERASE       = 1 << PIN_ERASE;
const uint32_t MASK_READ_WRITE  = 1 << PIN_READ_WRITE;
const uint32_t MASK_TRACK_1     = 1 << PIN_TRACK_1;
const uint32_t MASK_TRACK_2     = 1 << PIN_TRACK_2;
const uint32_t MASK_BOTH_TRACKS = MASK_TRACK_1 | MASK_TRACK_2;

const int PIN_BUZZER = 28; // GPIO28

const int NumOfSectors = 255;
const int MDVFileSize = QL_SECTOR_SIZE*NumOfSectors;
char *AllSectors = NULL;

// volatile storage for MD select
volatile uint16_t reg_select;
volatile uint8_t reg_data;

void InitMDVPins(void);
int TestSDCard(void);
int InitSDCard(void);
int LoadMDV(char *fname);
int LoadSector(FF_FILE *MDVfile, char *Buff, int SectorNum);
void f_error_to_string(FRESULT f, char*message);

bool CheckButtonPressed(uint ButtonPin);
bool ButtonPrevPressed=false;
bool ButtonNextPressed=false;
bool ButtonEnterPressed=false;
bool ButtonBackPressed=false;
int WaitAndCheckButtons(int ms);

bool CheckCSum(int CalcCheckSum, byte CS1, byte CS2);
int CalcSectorCheckSum(byte *buf, int offset, int len);
int MDVCheckSum(int sum);
void HexPrint(byte *buf, int offset, int bytes_per_row, int len);
void PrintMDVSector(byte *buf);
void PrintMDVData(byte *buf);
void PrintMDVHeader(byte *buf);

// __force_inline
static bool IsQLReading(void);
static bool IsQLWriting(void);
void PinsIdle();
void PinsQLRead();
void PinsQLWrite();
void PinsDriveActiveQlRead();
void gpio_callback(uint gpio, uint32_t events) ;
volatile uint8_t SelectedDrive = 255;
volatile uint8_t ClockInCount = 0;  

bool send_bit(uint32_t tr1, uint32_t tr2);
bool send_buffer(uint8_t* buf, uint16_t offset, uint16_t len);
bool send_buffer_(uint8_t* buf, uint16_t len);
bool do_send_sector(int sector, uint8_t index);
//__force_inline static uint32_t save_and_disable_interrupts(void);
//__force_inline static void restore_interrupts(uint32_t status);

void WaitMicroseconds(uint64_t microsecs);

// FAT file system for SD card.
FATFS fs;

uint8_t mdv_index=0xff;
int main() 
{
    uint16_t old_select = 0;    

    InitMDVPins();
    PinsIdle();

    gpio_init(PIN_BUZZER);
    gpio_set_dir(PIN_BUZZER, GPIO_OUT);
    gpio_put(PIN_BUZZER, 0);

    const uint led_pid = PICO_DEFAULT_LED_PIN;
    gpio_init(led_pid);
    gpio_set_dir(led_pid, GPIO_OUT);    

    stdio_init_all();

    AllSectors = malloc(MDVFileSize);
    if(AllSectors == NULL)
    {
         printf("Failed to allocate sector memory.\n");
         while(true)
         {
            gpio_put(led_pid, 1);        
            sleep_ms(250);
            gpio_put(led_pid, 0);
            sleep_ms(250);
         }
    }
    else
    {
        memset(AllSectors, 0, MDVFileSize);
    }    

    //printf("Press any key to proceed.\r\n");
    //getchar();
    // Wait until the ENTER button is pressed
    while(!CheckButtonPressed(PIN_BUTTON_ENTER))
    {
        ;        
    }

    char fname[] = "SPACEINV.MDV";

    int time = 500;
    int res = InitSDCard();
    if(res < 0)
    {
        time = 1000;
    }
    else
    {
        res = LoadMDV(fname);
        if(res < 0)
        {
            time = (-res)*1000;
        }
    }
    //LoadMDV("SPACEINV.MDV");
/*
    int time = 500;
    int res = TestSDCard();
    if(res < 0)
    {
        time = 2000;
    }
*/
    // Print first sector (0)
    PrintMDVSector((byte *)AllSectors);
    // Print Last sector (254)
    PrintMDVSector((byte *)AllSectors+(QL_SECTOR_SIZE*(NumOfSectors-1)));

    // Start receiving interrupts from QL
    reg_select = old_select = reg_data = 0;
    gpio_set_irq_enabled_with_callback(PIN_COMMS_CLK, GPIO_IRQ_EDGE_FALL , true, &gpio_callback);
    /*
        To add more interrupt vectors use
        gpio_set_irq_enabled(22, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    */

/*
PinsQLRead();
while(1)
{    
        gpio_put(PIN_TRACK_1, 1);
        gpio_put(PIN_TRACK_2, 1);

        sleep_ms(3000);

        gpio_put(PIN_TRACK_1, 0);
        gpio_put(PIN_TRACK_2, 0);

        sleep_ms(3000);
}
*/

    int Sector=1;
    int LedState = 1;

/* Check timings*/

PinsQLRead();                          
do
{
    gpio_put(PIN_LED_GREEN, LedState);                            
                            
    if(!do_send_sector(Sector, 0))
    {
        SelectedDrive = 0;
        //break;
    }
    printf("Sector:%d\r\n", Sector);
    LedState = LedState ^ 1;
                            
    Sector--;
    if(Sector==-1)
    {
        Sector=NumOfSectors-1;
    }                               
}
while (1);

/* Check timings end */
    
    SelectedDrive = 0;
    if(AllSectors == NULL)
    {
        printf("Failed to allocate sector memory.\n");
    }
    else
    {
        while(1)
        {   
            
            if (SelectedDrive) 
            {
                old_select=SelectedDrive;            
                WaitMicroseconds(SELECT_IGNORE_TIME);
            }
            /*
            if(reg_select)
            {
                old_select=reg_select;            
                WaitMicroseconds(SELECT_IGNORE_TIME);
            }
            if (reg_select==old_select && reg_select>0)    
            */
            if (SelectedDrive==old_select && SelectedDrive>0)                                     
            {                    
                printf("SEL:%02x\n\r",SelectedDrive);
                gpio_put(led_pid, 1);
                /*
                mdv_index=0xff;
                
                if (old_select & 0x80) mdv_index=7;
                if (old_select & 0x40) mdv_index=6;
                if (old_select & 0x20) mdv_index=5;
                if (old_select & 0x10) mdv_index=4;
                if (old_select & 0x08) mdv_index=3;
                if (old_select & 0x04) mdv_index=2;
                if (old_select & 0x02) mdv_index=1;
                if (old_select & 0x01) mdv_index=0;                      
                //SelectedDrive = mdv_index;
                */
            //uint8_t OldSelctedDrive = 0;
                //if(mdv_index != 0xFF) 
                if(SelectedDrive > 2 && SelectedDrive<8)
                {
                    printf("Microdrive '%d' selected,\n\r",SelectedDrive);
                    // if we have a valid index
                    /*
                    if(mdv_index != 0xff)
                    {
                        printf("Microdrive '%d' selected,\n\r",mdv_index);
                    } 
                    */               
                //if(SelectedDrive > 2)
                    //if (mdv_index != 0xff && mdv_index > 2) 
                    //{
                        //OldSelctedDrive = SelectedDrive;
                        PinsQLRead();                          

                        do
                        {
                            gpio_put(PIN_LED_GREEN, LedState);                            
                            
                            if(!do_send_sector(Sector, 0))
                            {
                                SelectedDrive = 0;
                                break;
                            }
                            printf("Sector:%d\r\n", Sector);
                            LedState = LedState ^ 1;
                            
                            Sector--;
                            if(Sector==-1)
                            {
                                Sector=NumOfSectors-1;
                            } 
                              
                            /*
                            Sector++;
                            if(Sector==NumOfSectors)
                            {
                                Sector=0;
                            }
                            */
                        }
                        while (SelectedDrive == old_select);

                        // Disable output to QL through buffer
                        gpio_put(PIN_TRISTATE_BUFFER_QL_READ, 1);

                        //while (SelectedDrive==old_select); //OldSelctedDrive == SelectedDrive
                        
                        //reg_select = old_select = 0;
                        //old_select = SelectedDrive = 0;
                        PinsIdle();
                        gpio_put(led_pid, 0);
                        LedState = 0;
                        gpio_put(PIN_LED_GREEN, LedState);
                    //}   
                }  
                SelectedDrive = 0;
            }
        }        
    }

    while (true) {
        printf("Hello, world!\n");        

        gpio_put(led_pid, 1);
        gpio_put(PIN_LED_GREEN, 1);
        
        //sleep_ms(time);
        int res = WaitAndCheckButtons(time);
        if(res > 0)
        {
            if(ButtonBackPressed)
            {
                printf("Button Back Pressed\n"); 
            }
            if(ButtonEnterPressed)
            {
                printf("Button Enter Pressed\n"); 
            }
            if(ButtonNextPressed)
            {
                printf("Button Next Pressed\n"); 
            }
            if(ButtonPrevPressed)
            {
                printf("Button Prev Pressed\n"); 
            }
        }

        gpio_put(led_pid, 0);
        gpio_put(PIN_LED_GREEN, 0);
        sleep_ms(time);
        if(AllSectors == NULL)
        {
            printf("Failed to allocate sector memory.\n");
        }
    }
    return 0;
}

// inline 
static bool IsQLWriting(void)
{
    int rw = gpio_get(PIN_READ_WRITE);
    if(rw == 0)
    {
        return false;
    }
    return true;
}

// inline 
static bool IsQLReading(void)
{
    // QL switches COMMS_CLK to HIGH and keeps it HIGH as long as its interested in reading more data.
    int comms_clk = gpio_get(PIN_COMMS_CLK);
    //int rw = gpio_get(PIN_READ_WRITE);
    if(comms_clk == 1 && IsQLWriting())
    {
        return true;
    }
    return false;
}

//uint8_t InterruptCounter=0;
//uint32_t CommsINreg=0;
void gpio_callback(uint gpio, uint32_t events) 
{
    /*
    if(gpio==22)DoSomething();
    if(gpio==21)DoSomethingElse('.');
    */
   if(gpio==PIN_COMMS_CLK)
   {       
        
        int comms_in = gpio_get(PIN_COMMS_IN);
        if(comms_in == 1)
        {
            SelectedDrive = 1;            
        }
        else
        {
            SelectedDrive++;
        }
        
        /*
            ClockInCount++;
            int comms_in = gpio_get(PIN_COMMS_IN);
            if(comms_in == 1)
            {
                SelectedDrive = ClockInCount;
                ClockInCount = 0;
            }
        */  
       /*
        reg_select<<=1;
        // or in the previous data bit
        reg_select|=reg_data;
        // save current data bit
        reg_data=gpio_get(PIN_COMMS_IN);                     
        */
   }
}

void CheckOutputRegRead(void)
{
    PinsQLRead();
    gpio_put(PIN_TRACK_1, 1);
    // Read output 
    uint32_t reg = gpio_get_all();       
    // get only track bits
    uint32_t tr = reg & MASK_TRACK_1;
    if(tr == MASK_TRACK_1)
    {
        printf("TRACK 1:1\r\n");
    }
    gpio_put(PIN_TRACK_1, 0);
    // Read output
    reg = gpio_get_all();       
    // get only track bits
    tr = reg & MASK_TRACK_1;
    if((tr & MASK_TRACK_1) == 0)
    {
        printf("TRACK 1:0\r\n");
    }
    PinsIdle();
}

// send a complete sector to the QL
// during the sending the QL can pull the write line down
// to update the current sector!
bool do_send_sector(int sector, uint8_t index)
{
    bool sect_update=0;
    // myled4=0;
/*
    if(!IsQLReading())
    {
        SelectedDrive = 0;
        return false;
    }
*/
    // set pins to output
    //PinsQLRead();    

    // send the header
    //if(send_buffer((uint8_t *)AllSectors+(QL_SECTOR_SIZE*sector), 0, QL_HEADER_SIZE) == false)
    uint8_t *SectorStart = (uint8_t *)AllSectors+(QL_SECTOR_SIZE*sector);
    //if(send_buffer_((uint8_t *)(AllSectors+(QL_SECTOR_SIZE*sector)), QL_HEADER_SIZE) == false)
    if(send_buffer_(SectorStart, QL_HEADER_SIZE) == false)
    {
        return false;
    }    
    PrintMDVHeader((uint8_t *)(AllSectors+(QL_SECTOR_SIZE*sector)));

    /*
    // Wait for header gap
    headerGap = true;
    if (timerEnabled()) 
    { 
          // COMMS_CLK timer may be active at this point      
          stopTimer();
    }
    setTimer(ZX_IF1 ? TIMER_HEADER_GAP_ZX_IF1 : TIMER_HEADER_GAP_QL, endHeaderGap);      
    */

// Timing check start    
gpio_put(PIN_BUZZER, 0);
// Timing check end

    uint64_t start_t = time_us_64();
    while (start_t+QL_HEADER_GAP > time_us_64()) 
    {          
        /*
          //if (checkRecordingOrStop())         
          if(!IsReadActive())
          {
              sect_update=1;
              break;
          }          
          _delay_us(20.0);
        */
        if(!IsQLReading())
        {
            SelectedDrive = 0;
            return false;
        }
        sleep_us(10.0);
    }

// Timing check start
gpio_put(PIN_BUZZER, 1);
// Timing check end

    //send_buffer((sector_buffer+HEADER_SIZE), BLOCK_SIZE + EXTRA_BLOCK_SIZE);    
    //if(send_buffer((uint8_t *)AllSectors+(QL_SECTOR_SIZE*sector), QL_HEADER_SIZE, QL_BLOCK_SIZE) == false)

    SectorStart = SectorStart + QL_HEADER_SIZE;
    //if(send_buffer_((uint8_t *)(AllSectors+((QL_SECTOR_SIZE*sector)+QL_HEADER_SIZE)), QL_BLOCK_SIZE+QL_EXTRA_DATA) == false)
    //if(send_buffer_((uint8_t *)(AllSectors+((QL_SECTOR_SIZE*sector)+QL_HEADER_SIZE)), QL_BLOCK_SIZE) == false)
    if(send_buffer_(SectorStart, QL_BLOCK_SIZE) == false)
    {
        return false;
    }
    PrintMDVData(SectorStart);
    /*
    // check if we should rewrite this sector
    if (sect_update) {
        //MD_DATA1.input();
        //MD_DATA2.input();
        PinsWrite(); // Data flows from MDV->Nano->ESP32 thus MDV is WRITING

        // computer wants to update this sector
        // so receive the data from the QL
        do_receive();

        
        // seek to start of data block of this sector in SD card file
        //fseek(fp, (sector*MDV_SECTOR_SIZE)+HEADER_SIZE+PREAMBLE_SIZE, SEEK_SET);
        // write data block part from buffer (skip first two FF bytes)
        //fwrite(buffer+2,BLOCK_SIZE-PREAMBLE_SIZE,1,fp);
        ESP32_WriteMDVSector(index, sector, sector_buffer, MDV_SECTOR_SIZE);
        //myled4=0;
        PinsRead();
    } 
    else 
    {
        // else send data block from buffer
        send_buffer((sector_buffer+HEADER_SIZE), BLOCK_SIZE + EXTRA_BLOCK_SIZE);        
    }
    */
    // wait GAP2 time ---- AK no need for this since sector read from serial requires 6ms
    //while (LPC_CT32B0->TC < GAP2_TIME);
    start_t = time_us_64();
    while (start_t+QL_SECTOR_GAP > time_us_64()) 
    {   
        if(!IsQLReading())
        {
            SelectedDrive = 0;
            return false;
        }              
        sleep_us(20.0);
    }
                
    //PinsDriveActiveQlRead();    
    //MD_DATA1.input();
    //MD_DATA2.input();
    return true;
}

// Send a buffer full of data to the QL
bool send_buffer_(uint8_t* buf, uint16_t len)
{
    int n,m;
    //unsigned int track1=0,track2=0;
    uint8_t track1=0,track2=0;

    //noInterrupts();    
    //uint32_t inter_status = save_and_disable_interrupts();
    // send them bytes            
    n=0;
    while (n < len) 
    {
        track1 = buf[n++];
        for (m=0; m<4; m++) {
            if(send_bit(track1 & 1, track2 & 1) == false) 
            {
                // Update mode activated or not selected any more
                return false;
            }
            track1>>=1;
            track2>>=1;
        }
        //n++;
        track2 |= buf[n++];
        for (m=0; m<4; m++) {
            if(send_bit(track1 & 1, track2 & 1) == false)
            {
                // Update mode activated or not selected any more
                return false;
            }
            track1>>=1;
            track2>>=1;
        }
        //n++;
    }
    // send last bits of track 2
    for (m=0; m<4; m++) 
    {
        if(send_bit(track1 & 1, track2 & 1) == false)
        {
            // Update mode activated or not selected any more
            return false;
        }
        track1>>=1;
        track2>>=1;
    }
    
    // set idle level (high)    
    //MD_DATA1=1;
    //MD_DATA2=1;         
    gpio_put(PIN_TRACK_1, 1);
    gpio_put(PIN_TRACK_2, 1);
   
    //interrupts();
    //restore_interrupts(inter_status);
    return true;
}

// Send a buffer full of data to the QL
bool send_buffer(uint8_t* buf, uint16_t offset, uint16_t len)
{
    int n,m;
    //unsigned int track1=0,track2=0;
    uint8_t track1=0,track2=0;

    //noInterrupts();    
    //uint32_t inter_status = save_and_disable_interrupts();
    // send them bytes
    n=offset;
    uint16_t count_end = offset+len;
    
    while (n<count_end) 
    {
        track1 = buf[n];
        for (m=0; m<4; m++) {
            if(send_bit(track1 & 1, track2 & 1) == false) 
            {
                // Update mode activated or not selected any more
                return false;
            }
            track1>>=1;
            track2>>=1;
        }
        n++;
        track2 |= buf[n];
        for (m=0; m<4; m++) {
            if(send_bit(track1 & 1, track2 & 1) == false)
            {
                // Update mode activated or not selected any more
                return false;
            }
            track1>>=1;
            track2>>=1;
        }
        n++;
    }
    // send last bits of track 2
    for (m=0; m<4; m++) 
    {
        if(send_bit(track1 & 1, track2 & 1) == false)
        {
            // Update mode activated or not selected any more
            return false;
        }
        track1>>=1;
        track2>>=1;
    }
    
    // set idle level (high)    
    //MD_DATA1=1;
    //MD_DATA2=1;         
    gpio_put(PIN_TRACK_1, 1);
    gpio_put(PIN_TRACK_2, 1);
   
    //interrupts();
    //restore_interrupts(inter_status);
    return true;
}

__force_inline bool send_bit(uint32_t tr1, uint32_t tr2)
{    
    // Read output register C
    uint32_t reg = gpio_get_all();
    
    // Save other bits
    uint32_t safe =  reg & ~MASK_BOTH_TRACKS; // Get only non track bits
    
    // Invert previous tracks state to start new bit and get only track bits
    //register uint32_t tr = (~reg) & MASK_BOTH_TRACKS;    
    uint32_t tr = (~reg) & MASK_BOTH_TRACKS;    
        
    // Start next bit cycle
    uint64_t start_us = time_us_64();
    
    // Set portC to previous contents but with track bits inverted
    gpio_put_all(safe | tr);    

    // Do this while timer is running for exact timing results   
    
    // if a '1' then reverse output halfway
    // selectively toggle pins according to value
    uint32_t tmp_tr1 = tr & MASK_TRACK_1;
    uint32_t tmp_tr2 = tr & MASK_TRACK_2;    
    tr = ~tr;
    if(tr1 == 1)
    {
      tmp_tr1 = tr & MASK_TRACK_1;
    }
    
    if(tr2 == 1)
    {
      tmp_tr2 = tr & MASK_TRACK_2;      
    }
     
    tr = safe | tmp_tr1 | tmp_tr2;    
    
    // wait half bit time    
    // this waits for QL_HALF_BIT_TIME as measured from baseline which is taken at start of cycle (5μs)
    while(time_us_64()<start_us+5) 
    {
        /*c=gpio_get_all();
        if((c&MASK_COMM)!=MASK_COMM) { // if COMMS from IF1 goes low then quit
            return false; // all done but end
        */
       /*
        if(!IsQLReading())
        {
            SelectedDrive = 0;
            return false;
        }
        */
    } 
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    // Write out the track data        
    gpio_put_all(tr);    

    // And return false if update mode set by QL or selected drive changed
    /*
    if(UpdateMode == 1 || SelectChanged == 1)
    {
        return false;
    }
    */
    
    // wait until end of bit time (10μs)
    // waitForElapsedCycles(baseline, QL_FULL_BIT_TIME);
    while(time_us_64()<start_us+9) 
    {
        /*c=gpio_get_all();
        if((c&MASK_COMM)!=MASK_COMM) { // if COMMS from IF1 goes low then quit
            return false; // all done but end
        */
       /*
        if(!IsQLReading())
        {
            SelectedDrive = 0;
            return false;
        }
        */
    }
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    asm volatile("nop \n nop \n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
    /*
    // Return false if update mode set by QL or selected drive changed
    if(UpdateMode == 1 || SelectChanged == 1)
    {
        return false;
    }
    */
    return true;
}

void WaitMicroseconds(uint64_t microsecs)
{
    uint64_t start = time_us_64();
    while(time_us_64() < (start+microsecs))
    {
        sleep_us(1);
    }
}

// Wait for given ms and check for buttons pressed.
// If a button is pressed the the coresonding global var (ButtonPrevPressed, ButtonNextPressed, 
// ButtonEnterPressed, ButtonBackPressed) is set to true.
// If buttons are pressed the last one is returned. Otherwise 0.
int WaitAndCheckButtons(int ms)
{
    ButtonPrevPressed=false;
    ButtonNextPressed=false;
    ButtonEnterPressed=false;
    ButtonBackPressed=false;
    int res=0;
    int n=0;
    while(n<ms)
    {
        sleep_ms(1);
        bool debounce = false;
        int prev = gpio_get(PIN_BUTTON_PREVIOUS);
        if(prev==0)
        {
            debounce=true;
        }
        int next = gpio_get(PIN_BUTTON_NEXT);
        if(next==0)
        {
            debounce=true;
        }
        int enter = gpio_get(PIN_BUTTON_ENTER);
        if(enter==0)
        {
            debounce=true;
        }
        int back = gpio_get(PIN_BUTTON_BACK);
        if(back==0)
        {
            debounce=true;
        }
        if(debounce)
        {
            sleep_ms(30);
            n=n+30;
            if(prev == gpio_get(PIN_BUTTON_PREVIOUS) && prev==0)
            {
                ButtonPrevPressed=true;
                res = PIN_BUTTON_PREVIOUS;
            }
            if(next == gpio_get(PIN_BUTTON_NEXT) && next==0)
            {
                ButtonNextPressed=true;
                res = PIN_BUTTON_NEXT;
            }
            if(enter == gpio_get(PIN_BUTTON_ENTER) && enter==0)
            {
                ButtonEnterPressed=true;
                res = PIN_BUTTON_ENTER;
            }
            if(back == gpio_get(PIN_BUTTON_BACK) && back==0)
            {
                ButtonBackPressed=true;
                res = PIN_BUTTON_BACK;
            }
        }        
        n++;
    }
    return res;
}

bool CheckButtonPressed(uint ButtonPin)
{
    int res = gpio_get(ButtonPin);
    if(res == 0)
    {
        // Debounce
        sleep_ms(50);            
        if(gpio_get(PIN_BUTTON_ENTER) == res)
        {
            return true;    
        }
    }
    return false;
}

int InitSDCard(void)
{
    FRESULT fr;
    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        return -1;
    }

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
         return -2;
    }

    return 1;
}

int LoadMDV(char *fname)
{
    FRESULT fr;
    FIL fil;

    // Open file for reading
    fr = f_open(&fil, fname, FA_READ);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        return -4;
    }
    int SectCount=0;
    for(SectCount=0; SectCount<NumOfSectors; SectCount++)    
    {
        if(LoadSector(&fil, AllSectors, SectCount) < 0)
        {
            return -8;
        }
    }
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        return -12;
    }

    return 1;
}

int LoadSector(FIL *MDVfile, char *Buff, int SectorNum)
{
    UINT BytesRead = 0;
    FRESULT res = f_read (MDVfile, Buff+(SectorNum*QL_SECTOR_SIZE), QL_SECTOR_SIZE, &BytesRead);	
    if(res != FR_OK || BytesRead!=QL_SECTOR_SIZE)
    {
        printf("ERROR: Could not read from MDV file sector %d\r\n", SectorNum);
        char message[120];
        f_error_to_string(res, message);
        printf("ERROR message: %s\r\n", message);
        return -1;
    }
    return 1;
}

void f_error_to_string(FRESULT f, char*message)
{
    switch(f)
    {
        case FR_OK:                 strcpy(message, " (0) Succeeded"); break;
	    case FR_DISK_ERR:           strcpy(message, " (1) A hard error occurred in the low level disk I/O layer"); break;
	    case FR_INT_ERR:            strcpy(message, " (2) Assertion failed "); break;
	    case FR_NOT_READY:          strcpy(message, " (3) The physical drive cannot work"); break;
	    case FR_NO_FILE:            strcpy(message, " (4) Could not find the file"); break;
	    case FR_NO_PATH:            strcpy(message, " (5) Could not find the path"); break;
	    case FR_INVALID_NAME:       strcpy(message, " (6) The path name format is invalid"); break;
	    case FR_DENIED:             strcpy(message, " (7) Access denied due to prohibited access or directory full"); break;
	    case FR_EXIST:              strcpy(message, " (8) Access denied due to prohibited access"); break;
	    case FR_INVALID_OBJECT:     strcpy(message, " (9) The file/directory object is invalid"); break;
	    case FR_WRITE_PROTECTED:    strcpy(message, " (10) The physical drive is write protected"); break;
	    case FR_INVALID_DRIVE:	    strcpy(message, " (11) The logical drive number is invalid"); break;
	    case FR_NOT_ENABLED:        strcpy(message, " (12) The volume has no work area"); break;
	    case FR_NO_FILESYSTEM:	    strcpy(message, " (13) There is no valid FAT volume"); break;
	    case FR_MKFS_ABORTED:       strcpy(message, " (14) The f_mkfs() aborted due to any problem"); break;
	    case FR_TIMEOUT:            strcpy(message, " (15) Could not get a grant to access the volume within defined period"); break;
	    case FR_LOCKED:	            strcpy(message, " (16) The operation is rejected according to the file sharing policy"); break;
	    case FR_NOT_ENOUGH_CORE:    strcpy(message, " (17) LFN working buffer could not be allocated"); break;
	    case FR_TOO_MANY_OPEN_FILES:strcpy(message, " (18) Number of open files > FF_FS_LOCK"); break;
	    case FR_INVALID_PARAMETER:	strcpy(message, " (19) Given parameter is invalid"); break;
        default : strcpy(message, "UNKOWN ERROR CODE"); break;
    }
}

int TestSDCard(void)
{
    FRESULT fr;
    FIL fil;
    int ret;
    char buf[100];
    char filename[] = "test02.txt";

    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        return -1;
    }

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
         return -2;
    }

    // Open file for writing ()
    fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        return -3;
    }

    // Write something to file
    ret = f_printf(&fil, "This is another test\r\n");
    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
        return -4;
    }
    ret = f_printf(&fil, "of writing to an SD card.\r\n");
    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
        return -5;
    }

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        return -6;
    }

    // Open file for reading
    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        return -7;
    }

    // Print every line in file over serial
    printf("Reading from file '%s':\r\n", filename);
    printf("---\r\n");
    
    while (f_gets(buf, sizeof(buf), &fil)) {
        printf(buf);
    }
    
   /*
    while (f_gets(AllSectors, sizeof(buf), &fil)) {
        printf(buf);
    }
    */
    printf("\r\n---\r\n");

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        return -8;
    }

    // Unmount drive
    f_unmount("0:");
    return 1;
}

void PinsDriveActiveQlRead()
{
    gpio_init(PIN_TRACK_1);
    gpio_set_dir(PIN_TRACK_1, GPIO_IN);
    gpio_pull_up(PIN_TRACK_1);
    gpio_init(PIN_TRACK_2);
    gpio_set_dir(PIN_TRACK_2, GPIO_IN);
    gpio_pull_up(PIN_TRACK_2);

    //gpio_disable_pulls(PIN_COMMS_CLK);
    gpio_pull_up(PIN_COMMS_CLK);
    //gpio_pull_up(PIN_COMMS_IN);

    gpio_disable_pulls(PIN_COMMS_IN);

    gpio_pull_up(PIN_ERASE);
    gpio_disable_pulls(PIN_READ_WRITE);
}

void PinsIdle()
{
    // Disable I/O  to QL through buffer
    gpio_put(PIN_TRISTATE_BUFFER_QL_READ, 1);
    gpio_put(PIN_TRISTATE_BUFFER_QL_WRITE, 1);

    // Tracks
    gpio_init(PIN_TRACK_1);
    gpio_set_dir(PIN_TRACK_1, GPIO_IN);
    gpio_init(PIN_TRACK_2);
    gpio_set_dir(PIN_TRACK_2, GPIO_IN);

    //gpio_disable_pulls(PIN_COMMS_CLK);
    gpio_pull_up(PIN_COMMS_CLK);
    //gpio_pull_up(PIN_COMMS_IN);

    gpio_disable_pulls(PIN_COMMS_IN);

    gpio_pull_up(PIN_ERASE);
    gpio_disable_pulls(PIN_READ_WRITE);
}

// QL will write to the MDV
void PinsQLWrite()
{
    // Disable PICO->QL direction
    gpio_put(PIN_TRISTATE_BUFFER_QL_READ, 1);
    
    // Enable QL->PICO direction
    gpio_put(PIN_TRISTATE_BUFFER_QL_WRITE, 0);

    gpio_pull_up(PIN_COMMS_CLK);
    gpio_disable_pulls(PIN_COMMS_IN);
    gpio_pull_up(PIN_READ_WRITE);
    gpio_pull_up(PIN_ERASE);
    // Tracks
    gpio_init(PIN_TRACK_1);
    gpio_set_dir(PIN_TRACK_1, GPIO_IN);
    gpio_init(PIN_TRACK_2);
    gpio_set_dir(PIN_TRACK_2, GPIO_IN);
}

// QL will read from the MDV
void PinsQLRead()
{
    // Enable output to QL through buffer for 5V from 3.3V
    gpio_put(PIN_TRISTATE_BUFFER_QL_READ, 0);
    // Disable QL->PICO direction
    gpio_put(PIN_TRISTATE_BUFFER_QL_WRITE, 1);

    gpio_pull_up(PIN_COMMS_CLK);
    gpio_disable_pulls(PIN_COMMS_IN);
    gpio_pull_up(PIN_READ_WRITE);
    gpio_pull_up(PIN_ERASE);
    // Tracks
    gpio_init(PIN_TRACK_1);
    gpio_set_drive_strength(PIN_TRACK_1, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_dir(PIN_TRACK_1, GPIO_OUT);
    gpio_init(PIN_TRACK_2);
    gpio_set_drive_strength(PIN_TRACK_2, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_dir(PIN_TRACK_2, GPIO_OUT);
}

void InitMDVPins(void)
{
    gpio_init(PIN_TRISTATE_BUFFER_QL_READ);
    gpio_set_dir(PIN_TRISTATE_BUFFER_QL_READ, GPIO_OUT);
    gpio_put(PIN_TRISTATE_BUFFER_QL_READ, 1);

    gpio_init(PIN_TRISTATE_BUFFER_QL_WRITE);
    gpio_set_dir(PIN_TRISTATE_BUFFER_QL_WRITE, GPIO_OUT);
    gpio_put(PIN_TRISTATE_BUFFER_QL_WRITE, 1);

    gpio_init(PIN_COMMS_IN);
    gpio_set_dir(PIN_COMMS_IN, GPIO_IN);
    //gpio_pull_up(PIN_COMMS_IN);
    //gpio_pull_down(PIN_COMMS_IN);

    gpio_init(PIN_COMMS_CLK);
    gpio_set_dir(PIN_COMMS_CLK, GPIO_IN);
    gpio_pull_up(PIN_COMMS_CLK);

    gpio_init(PIN_ERASE);
    gpio_set_dir(PIN_ERASE, GPIO_IN);
    gpio_pull_up(PIN_ERASE);

    gpio_init(PIN_READ_WRITE);
    gpio_set_dir(PIN_READ_WRITE, GPIO_IN);
    gpio_pull_up(PIN_READ_WRITE);

    gpio_init(PIN_WR_PROTECT);
    gpio_set_dir(PIN_WR_PROTECT, GPIO_OUT);

    // tracks initially as inputs which are high impedance
    gpio_init(PIN_TRACK_1);
    //gpio_set_drive_strength(PIN_TRACK_1, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_dir(PIN_TRACK_1, GPIO_IN);
    gpio_init(PIN_TRACK_2);
    //gpio_set_drive_strength(PIN_TRACK_2, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_dir(PIN_TRACK_2, GPIO_IN);

    gpio_init(PIN_LED_GREEN);
    gpio_set_dir(PIN_LED_GREEN, GPIO_OUT);

    gpio_init(PIN_BUTTON_PREVIOUS);
    gpio_set_dir(PIN_BUTTON_PREVIOUS, GPIO_IN);
    gpio_pull_up (PIN_BUTTON_PREVIOUS);

    gpio_init(PIN_BUTTON_NEXT);
    gpio_set_dir(PIN_BUTTON_NEXT, GPIO_IN);
    gpio_pull_up (PIN_BUTTON_NEXT);

    gpio_init(PIN_BUTTON_ENTER);
    gpio_set_dir(PIN_BUTTON_ENTER, GPIO_IN);
    gpio_pull_up (PIN_BUTTON_ENTER);

    gpio_init(PIN_BUTTON_BACK);
    gpio_set_dir(PIN_BUTTON_BACK, GPIO_IN);
    gpio_pull_up (PIN_BUTTON_BACK);
}


// Display MDV file sector through serial port
//
void PrintMDVSector(byte *buf)
{
  int SDS = MDV_PREAMBLE_SIZE;
  printf("Sector Marker: 0X%02X\r\n", buf[SDS]);
  printf("Sector #: 0X%02X (%d)\r\n", buf[SDS+1], buf[SDS+1]);
  // Cartridge  Name 2 - 11 10bytes
  printf("Cartridge  Name : %c%c%c%c%c%c%c%c%c%c\r\n", buf[SDS+2],buf[SDS+3],buf[SDS+4],buf[SDS+5],buf[SDS+6],buf[SDS+7],buf[SDS+8],buf[SDS+9],buf[SDS+10],buf[SDS+11]);
  printf("Random Bytes: %02X %02X\r\n", buf[SDS+12],buf[SDS+13]);
  printf("Header:");
  HexPrint(buf, SDS, 16, 16);
  //Serial.printf("Header: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", buf[SDS+14],buf[SDS+15],buf[SDS+16],buf[SDS+17],buf[SDS+18],buf[SDS+19],buf[SDS+20],buf[SDS+21],buf[SDS+22],buf[SDS+23],buf[SDS+24],buf[SDS+25],buf[SDS+26],buf[SDS+27]);
  //Serial.printf("File Number: %02X\r\n", buf[SDS+14]);
  //Serial.printf("Block Number: %02X\r\n", buf[SDS+15]);
  //Serial.printf("CheckSum: %02X %02X\r\n", buf[SDS+28],buf[SDS+29]);
  printf("Data:\r\n");
  // Print payload as hex
  //HexPrint(buf, SDS+MDV_SECTOR_HEADER_SIZE, 32, SECTOR_DATA_SIZE);
  // MDV_SECTOR_HEADER_SIZE = 40 (preamble1 + header(16) + preamble2)
  // MDV_SECTOR_DATA_SIZE 0x21A  /* 538 524 + 2 Checksum + 12 Preamble = 538 DATA LENGTH*/
  HexPrint(buf, MDV_SECTOR_HEADER_SIZE, 32, MDV_SECTOR_DATA_SIZE-MDV_PREAMBLE_SIZE);
  printf("\r\n");
  printf("CheckSum: %02X %02X\r\n", buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-2],buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-1]);
  int csum = CalcSectorCheckSum(buf, MDV_SECTOR_HEADER_SIZE+12, 512);
  printf("Calculated CheckSum: %04X\r\n",csum);
  if(CheckCSum(csum, buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-2],buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-1]) == true)
  {
    printf("Checksum OK.\r\n");
  }
  else
  {
    printf("Checksum INVALID.\r\n");
  }
  printf("Footer:\r\n");
  //HexPrint(buf, SECTOR_DATA_SIZE+SDS+MDV_SECTOR_HEADER_SIZE+2, 32, MDV_SECTOR_SIZE-(SECTOR_DATA_SIZE+SDS+MDV_SECTOR_HEADER_SIZE+2));
  printf("\r\n");
}

// Display MDV file sector HEADER through serial port
void PrintMDVHeader(byte *buf)
{
  int SDS = MDV_PREAMBLE_SIZE;
  printf("Sector Marker: 0X%02X\r\n", buf[SDS]);
  printf("Sector #: 0X%02X (%d)\r\n", buf[SDS+1], buf[SDS+1]);
  // Cartridge  Name 2 - 11 10bytes
  printf("Cartridge  Name : %c%c%c%c%c%c%c%c%c%c\r\n", buf[SDS+2],buf[SDS+3],buf[SDS+4],buf[SDS+5],buf[SDS+6],buf[SDS+7],buf[SDS+8],buf[SDS+9],buf[SDS+10],buf[SDS+11]);
  printf("Random Bytes: %02X %02X\r\n", buf[SDS+12],buf[SDS+13]);
  printf("Header:");
  HexPrint(buf, SDS, 16, 16);
}
// Display MDV file sector DATA through serial port
void PrintMDVData(byte *buf)
{
  //Serial.printf("Header: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", buf[SDS+14],buf[SDS+15],buf[SDS+16],buf[SDS+17],buf[SDS+18],buf[SDS+19],buf[SDS+20],buf[SDS+21],buf[SDS+22],buf[SDS+23],buf[SDS+24],buf[SDS+25],buf[SDS+26],buf[SDS+27]);
  //Serial.printf("File Number: %02X\r\n", buf[SDS+14]);
  //Serial.printf("Block Number: %02X\r\n", buf[SDS+15]);
  //Serial.printf("CheckSum: %02X %02X\r\n", buf[SDS+28],buf[SDS+29]);
  printf("Data:\r\n");
  // Print payload as hex
  //HexPrint(buf, SDS+MDV_SECTOR_HEADER_SIZE, 32, SECTOR_DATA_SIZE);
  // MDV_SECTOR_HEADER_SIZE = 40 (preamble1 + header(16) + preamble2)
  // MDV_SECTOR_DATA_SIZE 0x21A  /* 538 524 + 2 Checksum + 12 Preamble = 538 DATA LENGTH*/
  HexPrint(buf, 0, 32, MDV_SECTOR_DATA_SIZE);
  printf("\r\n");
  printf("CheckSum: %02X %02X\r\n", buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-2],buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-1]);
  int csum = CalcSectorCheckSum(buf, MDV_SECTOR_HEADER_SIZE+12, 512);
  printf("Calculated CheckSum: %04X\r\n",csum);
  if(CheckCSum(csum, buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-2],buf[MDV_SECTOR_HEADER_SIZE-MDV_PREAMBLE_SIZE+MDV_SECTOR_DATA_SIZE-1]) == true)
  {
    printf("Checksum OK.\r\n");
  }
  else
  {
    printf("Checksum INVALID.\r\n");
  }
  printf("Footer:\r\n");
  HexPrint(buf, MDV_SECTOR_DATA_SIZE, 32, QL_EXTRA_DATA);  
  printf("\r\n");
}

// HEX Print a byte buffer
//
void HexPrint(byte *buf, int offset, int bytes_per_row, int len)
{
  // Print sector in lines
  printf("%04X:", offset);
  int lines = len/bytes_per_row;
  if(lines*bytes_per_row < len) lines = lines+1;
  int k,n;
  int bt = 0;
  for(k=0; k<lines; k++)
  {
    for(n=0; n<bytes_per_row; n++)
    {
      bt = offset+(k*bytes_per_row)+n;
      // Each line bytes_per_row bytes
      if(bt<len+offset)
      {
        printf("%02X ", buf[bt]);
      }
      else
      {
        printf("   ");
      }
    }
    for(n=0; n<bytes_per_row; n++)
    {
      // Each line bytes_per_row bytes
      bt = offset+(k*bytes_per_row)+n;
      if(bt<len+offset)
      {
        if(isprint(buf[bt]))
        {
          printf("%c", buf[bt]);
        }
        else
        {
           printf(".");
        }
      }
      else
      {
        break;
      }
    }
    bt++;
    printf("\r\n");
    if(k<lines-1)
    {
      printf("%04X:", bt);
    }
  }
}

int MDVCheckSum(int sum)
{
   return (0x0F0F + sum) % 0x10000;
}

int CalcSectorCheckSum(byte *buf, int offset, int len)
{
  int n;
  int csum = 0x0F0F;
  for(n=offset; n<len+offset; n++)
  {
    csum = (csum + buf[n]);
  }
  return csum % 0x10000;
}

bool CheckCSum(int CalcCheckSum, byte CS1, byte CS2)
{
  char CalcCSum[8];
  char MDVCSum[8];
  sprintf(CalcCSum, "%04X", CalcCheckSum);
  sprintf(MDVCSum, "%02X%02X", CS2, CS1);
  if(strcmp(CalcCSum, MDVCSum) == 0)
  {
    return true;
  }
  else
  {
    return true;
  }
}

/*! \brief Save and disable interrupts
 *  \ingroup hardware_sync
 *
 * \return The prior interrupt enable status for restoration later via restore_interrupts()
 */
/*
__force_inline static uint32_t save_and_disable_interrupts(void) {
    uint32_t status;
    __asm volatile ("mrs %0, PRIMASK" : "=r" (status)::);
    __asm volatile ("cpsid i");
    return status;
}
*/
/*! \brief Restore interrupts to a specified state
 *  \ingroup hardware_sync
 *
 * \param status Previous interrupt status from save_and_disable_interrupts()
 */
/*
__force_inline static void restore_interrupts(uint32_t status) {
    __asm volatile ("msr PRIMASK,%0"::"r" (status) : );
}
*/