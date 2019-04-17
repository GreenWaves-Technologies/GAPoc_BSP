/*
 * Copyright (c) 2019, GreenWaves Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of GreenWaves Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// #############################################################################
// ##### DECLARATIONS  ########################################################

// ==  Handy "defines" for application, used locally   =====================

#define ON_LCD  0
#define ON_PC   1    
#define NO_DISP   0    

/** ****************************
// Select here if you want the pictures to be displayed on an Adafruit LCD attached to GAPOC
// (select ON_LCD) or to be saved as .ppm on PC (select ON_PC) -- in this case you must have JTAG connected 
// or not displayed at all (select NO_DISP)
** ****************************/
// ===========================

#define DISPLAY     ON_PC     // options other than ON_PC not implemented yet !

// ===========================


// TODO - Move this to BSP library

#define PIC_WIDTH    (2*80)   // IR Proxy Frame = 80x80 pixels at 2bytes per pix (14bits provided as 2 successive bytes) -- temp. info is *not* included in data
#define PIC_HEIGHT   80    
#define PIC_SIZE    (PIC_WIDTH*PIC_HEIGHT)  

// Move this to BSP:
#define IRPROXY_TINT    0x1     // Integration Time (in number of cycles -- Range 1:255)
#define     IRPROXY_TINT_DFLT   0x010

#define IRPROXY_GFID    0x2     // 12-bit ADC value for GFID - Range 0 for OV to 4095 for 3.6V
#define     IRPROXY_GFID_DFLT   0xBB0

#define IRPROXY_GSK     0x3     // 12-bit ADC value for VSK - Range 0 for OV to 4095 for 3.6V
#define     IRPROXY_GSK_DFLT    0x540

#define IRPROXY_CKDIV   0x4     // (USE WITH CAUTION) 
#define     IRPROXY_CKDIV_DEFLT   0x002    
#define     IRPROXY_CKDIV_INV_PIXCLK_MASK       0x1
#define     IRPROXY_CKDIV_INV_PIXCLK_SHIFT      11
#define     IRPROXY_CKDIV_DATA_PACKING_MASK     0x1
#define     IRPROXY_CKDIV_DATA_PACKING_SHIFT    10
#define     IRPROXY_CKDIV_VSYNC_POL_MASK        0x1
#define     IRPROXY_CKDIV_VSYNC_POL_SHIFT       9
#define     IRPROXY_CKDIV_HSYNC_POL_MASK        0x1
#define     IRPROXY_CKDIV_HSYNC_POL_SHIFT       8   // !!! BEWARE !!! Looks like inverting HSYNC pol doesnt work; VSYNC is lost if trying to use it
#define     IRPROXY_CKDIV_SENSOR_CLKDIV_MASK    0x1F
#define     IRPROXY_CKDIV_SENSOR_CLKDIV_SHIFT   0

#define IRPROXY_GMS    0x5     // Sensor Gain / Mirror / Size settings -- XXXXX|Gain3|Gain2|Gain1|X|0|UpRow|UpCol
#define     IRPROXY_GMS_DFLT   0x053

#define IRPROXY_TRIGGER_ADDR    0xE     // Trigger address parameter (send 0x010, then use Trigger Value register to set working mode
#define     IRPROXY_TRIGGER_ADDR_MAGIC_NUM  0x010

#define IRPROXY_SW_TRIGGER      0xE     // Software trigger command. If s/w trigger is enabled, send 0x011 to get 1 frame
#define     IRPROXY_SW_TRIGGER_MAGICNUM     0x011

#define IRPROXY_TRIGGER_VAL     0xF    // Trigger Mode Selection
#define     IRPROXY_TRIGGER_VAL_DFLT      0x0
#define     IRPROXY_TRIGGER_VAL_FREERUN     0x000  // free running mode
#define     IRPROXY_TRIGGER_VAL_TRIGMODE_A  0x001  // 'Ulis' trigger mode
#define     IRPROXY_TRIGGER_VAL_TRIGMODE_B  0x002  // 'FullScale' trigger mode
#define     IRPROXY_TRIGGER_VAL_SWTRIG      0x003  // software trigger mode



// ====  Includes    ==========================================================

#include "GAPOC_BSP_General.h"

#include "mbed_wait_api.h"


// ====  Application's Custom Types    ========================================
// <none>

    
// ==== Application's public global variables  ==============================

GAP_L2_DATA   unsigned char image_buffer_14bpp[ PIC_WIDTH*PIC_HEIGHT ];   // 14 bits per pixel, on 16 bits with 2MSBs="00"  -- for exploitation by algorithms



// ==== Application's own global variables  ====================================

CPI_Type *const cpi_address[] = CPI_BASE_PTRS;

static volatile uint32_t Picture_Index = 0;  // to manage non-blocking picture transfer (snapshot)
 
static GAP_L2_DATA    cpi_config_t cpi_config;
static GAP_L2_DATA    cpi_transfer_t cpiTransfer;  
static GAP_L2_DATA    cpi_handle_t hCPI;  

GAP_L2_DATA   spi_t   spim1;      
GAP_L2_DATA  uint16_t  spi_word16;
        
      
// ==== Application's Function Prototypes    ===================================


// TODO: Move to BSP library
void GAPOC_IRProxy_WriteReg12(uint8_t RegAddr, uint16_t WriteVal);
void GAPOC_IRProxy_SPI_Init();


// Local helper functions
static void Callback_Single_Shot();   
static void save_pict_ppm( unsigned char* pic_buffer);



// #############################################################################
// ##### MAIN APPLICATION  ###########################################

int main()
{

    DBG_PRINT("\nBasic Test of Proxy with ULIS IR Sensor on DF12 Connector CONN8\n\n");
    
    DBG_PRINT("Through your #define DISPLAY, you have selected to ");
    
#if  DISPLAY == ON_LCD
    DBG_PRINT("display an image of captured IR data on LCD\n\n");
    
#elif  DISPLAY == ON_PC
    DBG_PRINT("save an image as .ppm e.g. for diplay on PC\n\n");
    // This is to be able to use debug bridge to sace .ppm picture on host PC :
    BRIDGE_Init();
    printf("Connecting to bridge\n");
    BRIDGE_Connect(0, NULL);
    printf("Connection done\n\n");
    
#elif  DISPLAY == NO_DISP
    DBG_PRINT("not to display an image\n\n");   
    
#else
    #error "you didn't properly #define DISPLAY"   
#endif
// TODO -- ON_LCD not supported yet
    
    

    // ----  Initializations   -------------------------------------------------------------
    
    //  --  Initalize Board (GPIO direction and default level, supplies, etc.)       
    GAPOC_BSP_Board_Init();
    
    
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
// TEMPORARY -- 
    // To avoid pad frame bug
    // To be used with modified board or sensor emulator,
    // won't work with IRProxy as is since this GPIO enables MCLK   
    GAPOC_AnyPin_Config( B34, PULL, uPORT_MuxAlt0 );  
        // pin GAP_B34 keeps default function (I2C_SDA) rather than use as GPIO_CIS_CLK -- Pull-Down ensures GPIO_CIS_CLK will be low so clk active
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 

  
    // -- Initialize SPI1 for Proxy Use
    GAPOC_IRProxy_SPI_Init();

    // --  Initialize Heartbeat LED (LED enabled only if DIP Switch S6 is on) 
    GAPOC_GPIO_Init_Pure_Output_High( GAPOC_HEARTBEAT_LED );
 
    // -- If wished, enable Vsync-controlled LED  (keep GPIO_CIS_LED_ENB = hi-Z to disable)
    GAPOC_GPIO_Init_Pure_Output_Low( GPIO_CIS_LED_ENB );  // Set Low to have LED on when Vsync is high, High to have LED on when Vsync is low


    // -- Init CPI i/f        ------------------------------------  

    CPI_Init(cpi_address[0], CPI_PCLK, CPI_HSYNC, CPI_VSYNC,
             CPI_DATA0, CPI_DATA1, CPI_DATA2, CPI_DATA3,
             CPI_DATA4, CPI_DATA5, CPI_DATA6, CPI_DATA7); 
        // UDMA_Init() is included in the above                


    // -- Configure CPI i/f       -----------------------------------  


    CPI_GetDefaultConfig(&cpi_config);
    cpi_config.row_len = PIC_WIDTH ; 
    cpi_config.resolution      = PIC_SIZE;
    cpi_config.format          = BYPASS_BIGEND;  // Monochrome
    cpi_config.shift           = 0;
    cpi_config.slice_en        = 0;
    cpi_config.frameDrop_en    = 0;
    cpi_config.frameDrop_value = 0;
    cpi_config.wordWidth       = 16;  // 8, 16 or 32bits // IN FACT IGNORED in CPI DRIVER [gap_cpi.c] !!!!
        // Those settings will be used by CPI_Enable
 
   
    // Set-up uDMA for getting data from CPI:

    cpiTransfer.data        = image_buffer_14bpp;
    cpiTransfer.dataSize    = PIC_SIZE;  // !! MUST BE <128K
    cpiTransfer.configFlags = UDMA_CFG_DATA_SIZE(1); //0 -> 8bit //1 -> 16bit //2 -> 32bit -- ALWAYS USE 16-bits for CPI uDMA (?)
        // Will be used by CPI Reception function (blocking or non-blocking)
        

    DBG_PRINT("Width=%d, Height=%d, Size=%d\n", cpi_config.row_len, PIC_HEIGHT, cpiTransfer.dataSize);



              
    // -- Start IR Proxy:              ----------------------------------------------------------------------

    #define  MIN_INTERCOMMAND_DELAY_ms  10  // Proxy requires 10ms min between 2 SPI commands ! (...that much, really ?!?)
    
    // --  Enable 3V3A/3V3D   -------------------
    GAPOC_GPIO_Init_Pure_Output_High( GPIO_CIS_APWRON );              
    GAPOC_GPIO_Init_Pure_Output_High( GPIO_CIS_DPWRON );  
    
    
    // --  Initialize trigger signal   ---------    
     GAPOC_GPIO_Set_High(GPIO_CIS_TRIGGER);      // Make sure trigger input is inactive (active high for snapshot mode A, active low for snapshot mode B)
        
        
    // --  Master Clock On   --------------------
    // When voltages are established, wait 150ms and apply Master Clock
    #define  MCLOCK_TURNON_LATENCY_ms       150.0
    wait( (float)(MCLOCK_TURNON_LATENCY_ms)/1000.0);
    GAPOC_GPIO_Set_Low( GPIO_CIS_CLK );      // Turn On MCLK (active low control)                                   
        // Note: assuming here this signal controls enable of dedicated ClkGen, not used as clock itself
        //  (board assembly option)
      // !!! -- No effect if I/O was not configured as GPIO to work around pad mux bug !    
            
    // NOTE: GAPOC_B (0.1) when using dedicated ClkGen for MCLK provides 2.048MHz
    //  while latest Proxy DS specifies 3.8MHz (earlier versions said 1.9MHZ with 2.048 OK)
    //  Impact: frame rate limited to 30Hz instead of 60Hz


    #define  IRPROXY_SETTLING_TIME_ms       150.0
    wait( (float) IRPROXY_SETTLING_TIME_ms/1000.0  );  // else upcoming SPI prog may not be properly taken into account ? (min duration TBC)

     
    // == Proxy is now ready to be configured through SPI Bus : ====
                   
    // ---  Trigger mode Register  - --- 
    GAPOC_IRProxy_WriteReg12( IRPROXY_TRIGGER_ADDR, IRPROXY_TRIGGER_ADDR_MAGIC_NUM);
    wait(MIN_INTERCOMMAND_DELAY_ms/1000.0);        
    GAPOC_IRProxy_WriteReg12( IRPROXY_TRIGGER_VAL, IRPROXY_TRIGGER_VAL_TRIGMODE_B);    // "FullScale trigger" mode
    wait(MIN_INTERCOMMAND_DELAY_ms/1000.0);                  

    // --- Integration Time Register  ---
    GAPOC_IRProxy_WriteReg12( IRPROXY_TINT, 0x010);     // DS says (hardwired) default is 0x10, FullScale suggests 0x20 (or 20dec?)
    wait(MIN_INTERCOMMAND_DELAY_ms/1000.0);    
        
    // --- GFID Register   --------------
    GAPOC_IRProxy_WriteReg12( IRPROXY_GFID, 0x0BB);     // DS says default is 0xBB0, FullScale suggests 0x0BB 
    wait(MIN_INTERCOMMAND_DELAY_ms/1000.0);    
            
    // --- GSK  Register   --------------
    GAPOC_IRProxy_WriteReg12( IRPROXY_GSK, 0x11C);      // DS says default is 0x540,  FullScale suggests 0x11C
    wait(MIN_INTERCOMMAND_DELAY_ms/1000.0);    
           
    // --- GMS  Register   --------------
    GAPOC_IRProxy_WriteReg12( IRPROXY_GMS, 0x053);      // DS says default is 0x053  (FullScale too --> Gain 2, Gain 0, UpCol, UpRow bits)
    wait(MIN_INTERCOMMAND_DELAY_ms/1000.0);    
        
    // --- ClkDiv Register  -------------    (NB: Not taken into account if done as first access ??? TBC)
    // Don't rely on default clkdiv  specified in DS as seen not to match h/w 
    GAPOC_IRProxy_WriteReg12( IRPROXY_CKDIV, 0x01);  // from DS, would expect PCLK = 0.5*MCLK/(2**(0x01+1)) = 2.048MHz/8 = 256KHz -- actually observing 512KHz..

        
    DBG_PRINT("IR Proxy Ready and Configured\n");  



    // -- Now capture frames in Snapshot mode with non-blocking reception ---------------------------------------
    
    
    // Enable CPI
    // (CPI will be continuously active -- alternatively, we could
    // disable it after picture taken, enable back prior to shooting next picture etc... To have more clock gating and save power)
    CPI_Enable(cpi_address[0], &cpi_config); // Activate configuration of CPI channel -- Starts gated clock of CPI; needed if disabled in callback (to save power)

      
    // Prepare handles for non-blocking camera capture
    CPI_ReceptionCreateHandle(cpi_address[0], &hCPI, Callback_Single_Shot, NULL);  
    DBG_PRINT("CPI Handle created\n");


    //  ===   MAIN LOOP    =================================================================================================       

     GAPOC_GPIO_Set_Low(GPIO_CIS_TRIGGER);      // Assert trigger signal 
     
//    while(1)  // Comment out -- for now capture a single frame after sensor has stabilized     
    #define NUM_DUMMY_SNAPSHOTS 1000
    for (uint32_t i=0; i <NUM_DUMMY_SNAPSHOTS; i++)
    {
      
        // -- Enable capture by camera (non-blocking)
        CPI_ReceptionNonBlocking(cpi_address[0], &hCPI, &cpiTransfer);  

        while (Picture_Index ==0);  // wait for flag from callback signalling image capture finished

        // Got picture...   
        Picture_Index =0;

    }      


    // Save last snapshot (with 14bpp to 8bpp conversion) for display on PC or LCD
    save_pict_ppm( image_buffer_14bpp );
    
    
    
    //  ===   END PROCESSING   =================================================================================================           

    DBG_PRINT("\nDone, Switching off IR Proxy\n");
    

    // --  Switch off IR sensor   ------------------------------
    
    // Set integration time to zero
     GAPOC_IRProxy_WriteReg12( IRPROXY_TINT, 0x000);   
    
    // Power off analog supply and wait 100ms
    GAPOC_GPIO_Set_Low( GPIO_CIS_APWRON );              
    #define  ANA_TO_DIG_POWEROFF_LATENCY_ms 100
    wait((float)ANA_TO_DIG_POWEROFF_LATENCY_ms/1000.0);

    // Power off digital supply 
    GAPOC_GPIO_Set_Low( GPIO_CIS_DPWRON );      
        
    // Switch off master clock to IR sensor
    GAPOC_GPIO_Set_High( GPIO_CIS_CLK );      // Turn Off MCLK (active low control) [signal goes to ClkGen powered by always on 2V5]                                     
        // !!! -- No effect if I/O was not configured as GPIO to work around pad mux bug !    
        
    // ----------------------------------------------------------
    
    GAPOC_GPIO_Set_Low(GPIO_CIS_TRIGGER);  // don't keep high GPIO driving powered off device

    BRIDGE_Disconnect(NULL);


    return (0);    
}


// #############################################################################
// ##### LOCAL FUNCTION DEFINITIONS  ###########################################


static void Callback_Single_Shot()   
{   
    Picture_Index++;    
}

// ----------------------------------------------------------------------
static void save_pict_ppm( unsigned char* pic_buffer_14bpp)
{
    static uint32_t imgNum = 0;
    static char imgName[50];    
    static unsigned char pic_buffer_8bpp[ PIC_WIDTH*PIC_HEIGHT /2];  // 8bpp = 8 MSBs of above + rounding  -- for display/debug only
  
    
    // Could add some rounding
    // but BEWARE if original value is 0x3FF then rounding would lead to 0x100 rather than 0xFF ...does not fit on 8 bits (wraps to 0x00!) 
    // Not coped with yet below


    for (uint32_t i=0; i<PIC_SIZE/2; i++)
    { 
    
        // Could do rounding but BEWARE
        //  if original value is 0x3FF then rounding would lead to 0x100 rather than 0xFF ...does not fit on 8 bits (wraps to 0x00!) 
        // Not coped with yet below
        /*
        if ( (pic_buffer_14bpp[i+1] & 0x3F) > (1<<5) )  // Are LSBs we're going to drop > 0.5x max value they can represent i.e > 2**5 ?
        {
            uint8_t rounding = 0x1;
        }  
        else 
         {
            rounding = 0x0;
        }         
        pic_buffer_8bpp[i] = ( (image_buffer_14bpp[2*i] <<2) + (image_buffer_14bpp[2*i+1] >>6) + rounding );
        */
        
        pic_buffer_8bpp[i] = ( (image_buffer_14bpp[2*i] <<2) + (image_buffer_14bpp[2*i+1] >>6)  );   // no rounding here
  
        DBG_PRINT("%d.",(int)pic_buffer_8bpp[i]);
    }
    DBG_PRINT("\n\n");

        

        sprintf(imgName, "../../../img_OUT%d.ppm", (int)imgNum++);
        printf("\nimgName: %s\n", imgName);
        WriteImageToFile(imgName, PIC_WIDTH/2, PIC_HEIGHT, (pic_buffer_8bpp));
}
        
        

// ------------------------------------------------------------

void GAPOC_IRProxy_SPI_Init()
{
        // SPI pins init, SPI udma channel init 
        spi_init(&spim1, SPI1_MOSI, NC, SPI1_SCLK, SPI1_CSN0_A5);
//        spi_init(&spim1, B3, SPI1_MISO, B4, SPI1_CSN0_A5);


        // SPI bits, cpha, cpol configuration 
        spi_format(&spim1, 16, 0, 0);  // 16-bit words, idle level =low
            // BEWARE - For some reason when doing 16-bit SPI transfers, SPI send MSB first in ech byte and LSByte first 
            
        // Set SPI fequency *
        spi_frequency(&spim1, SPI_FQCY_IRSENSOR_KHZ*1000);

        DBG_PRINT("\nSPI1 initalized\n");
           
}


// -----------------------------------------------------------------

/** Function : GAPOC_IRProxy_WriteReg12 
    Action : Using SPI1, Writes value WriteVal into 12-bit register of the IR Proxy specified by its 4-bit address RegAddr
*/
     
void GAPOC_IRProxy_WriteReg12(uint8_t RegAddr, uint16_t WriteVal)  // RegAddr to fit on 4 bits and WriteVal on 12 bits
{

#if GAPOC_DEBUG == 1
    if ( (RegAddr >>4) != 0x0)
    {
        DBG_PRINT("Reg. Address must fit on 4 bits !  -- Stopping Here\n");
        while(1);
    }
    if ( (WriteVal >>12) != 0x0)
    {
        DBG_PRINT("Reg. Value must fit on 12 bits !  -- Stopping Here\n");
        while(1);
    }
#endif

    spi_word16 =  (RegAddr <<12) | (WriteVal );  // Addr = 4 MSB, Value = 12 LSB

    // Ulis Proxy wants 16-bit with MSB first : A3.A2.A1.A0.D11.D10.......D1.D0
    // while (when doing 16-bit SPI transfers) SPI send MSB first in ech byte and LSByte first (strange!?)
    // So swap MSBbyte and LSBbyte
    spi_word16 =  (spi_word16 & 0xFF) <<8 | spi_word16 >>8 ; 
                
    spi_master_cs(&spim1, 0);            
    spi_master_write(&spim1, spi_word16);
        // NB - When doing 16-bit SPI transfers, it appears SPI send MSB first in ech byte and LSByte first ??!!??       
    spi_master_cs(&spim1, 1);    
}
        

// ## END OF FILE ##############################################################################


