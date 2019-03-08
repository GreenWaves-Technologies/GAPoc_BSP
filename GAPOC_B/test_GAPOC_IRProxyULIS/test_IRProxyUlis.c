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
// <none>
#define		MT9V034_READ_MODE_ROW_BIN_MASK		(3 << 0)
#define		MT9V034_READ_MODE_ROW_BIN_SHIFT		0

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
#define     IRPROXY_CKDIV_DATA_PACKING_SHIFT    11
#define     IRPROXY_CKDIV_DATA_PACKING_MASK     0x1
#define     IRPROXY_CKDIV_INV_PIXCLK_SHIFT      10
#define     IRPROXY_CKDIV_VSYNC_POL_MASK        0x1
#define     IRPROXY_CKDIV_VSYNC_POL_SHIFT       9
#define     IRPROXY_CKDIV_HSYNC_POL_MASK        0x1
#define     IRPROXY_CKDIV_HSYNC_POL_SHIFT       8
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
// <none>


// ==== Application's own global variables  ====================================

static  cpi_config_t cpi_config;
static  spi_t   spim1;   
GAP_L2_DATA static uint16_t  spi_word16;
        
        
// ==== Application's Function Prototypes    ===================================


void GAPOC_IRProxy_SPI_Init();
void GAPOC_IRProxy_CPI_Init();
void GAPOC_IRProxy_WriteReg12(uint8_t RegAddr, uint16_t WriteVal);


// #############################################################################
// ##### MAIN APPLICATION  ###########################################

int main()
{

    DBG_PRINT("\nBasic Test of Proxy with ULIS IR Sensor on DF12 Connector CONN8\n\n");


    // ----  Initializations   -------------------------------------------------------------
    
    //  --  Initalize Board (GPIO direction and default level, supplies, etc.)       
    GAPOC_BSP_Board_Init();
    
    
    // -- Initialize GPIO with impact on IR Sensor Proxy

    // Make Clock to Proxy is disabled
    GAPOC_GPIO_Init_Pure_Output_Low( GPIO_CIS_CLK );                 
      
    // Make sure power supplies to Proxy are Off
    GAPOC_GPIO_Init_Pure_Output_Low( GPIO_CIS_APWRON );              
    GAPOC_GPIO_Init_Pure_Output_Low( GPIO_CIS_DPWRON );                 
   
    // Make sure trigger signal to Proxy is low
    GAPOC_GPIO_Init_Pure_Output_Low( GPIO_CIS_TRIGGER );                 
      
    // Initialize Heartbeat LED (LED enabled only if DIP Switch S6 is on) 
    GAPOC_GPIO_Init_Pure_Output_High( GAPOC_HEARTBEAT_LED );

    // Enable Vsync-controlled LED
    GAPOC_GPIO_Init_Pure_Output_Low( GPIO_CIS_LED_ENB );  
    
    DBG_PRINT("\nGPIOs used as Enable/Disable initalized\n");    


    // -- Initialize SPI1 for Proxy Use
    GAPOC_IRProxy_SPI_Init();
   
    // -- Initialize CPI for Proxy Use
    GAPOC_IRProxy_CPI_Init();    
    


    // ----  Start IR Sensor      -------------------------------------------------------
        
    // Power up proxy digital and analog voltages
    GAPOC_GPIO_Set_High( GPIO_CIS_APWRON );   // Turn On AVDD                                
    GAPOC_GPIO_Set_High( GPIO_CIS_DPWRON );   // Turn On DVDD  
            
    // When voltages are established, wait 150ms and apply Master Clock
    #define  IRPROXY_VDD_SETTLING_TIME_ms   1
    wait( (float)IRPROXY_VDD_SETTLING_TIME_ms/1000.0);
    GAPOC_GPIO_Set_High( GPIO_CIS_CLK );      // Turn On MCLK                                    
        // Note: assuming here this signal controls enable of dedicated ClkGen, not used as clock itself
        //  (board assembly option)
    
    // Proxy is now ready to receive data from SPI Bus...
    
    // Set integration Time Value to 20
    GAPOC_IRProxy_WriteReg12( IRPROXY_TINT, 0x020);
    
    // Set GFID Value to 187
    GAPOC_IRProxy_WriteReg12( IRPROXY_GFID, 0x0BB);
        
    // Set GSK Value to 284
    GAPOC_IRProxy_WriteReg12( IRPROXY_GSK, 0x11C);
        
    // Set GMS value to enable Gain 2, Gain 0, UpCol, UpRow bits
    GAPOC_IRProxy_WriteReg12( IRPROXY_GMS, 0x053);
        
    // With this configuration, K035 PRoxy is ready to provide full image @30fps
   


    // ----  Start Acquisition through CPI       -------------------------------------------------------
    // TODO
    
    
    
    //  ---   Main Loop             --------------------------------------------------------------------       
    
    #define LED_ON_TIME_ms   1000
    #define LED_OFF_TIME_ms  1000
    while(1)
    {

        GAPOC_GPIO_Set_High( GAPOC_HEARTBEAT_LED );       
        wait( (float)(LED_ON_TIME_ms)/1000.0 );        
      
        GAPOC_GPIO_Set_Low( GAPOC_HEARTBEAT_LED );                                                  
        wait( (float)(LED_OFF_TIME_ms)/1000.0 ); 

    }

}   
    
// ======================================================================================




// ------------------------------------------------------------

void GAPOC_IRProxy_SPI_Init()
{
        // SPI pins init, SPI udma channel init 
        spi_init(&spim1, SPI1_MOSI, NC, SPI1_SCLK, SPI1_CSN0_A5);

        // SPI bits, cpha, cpol configuration 
        spi_format(&spim1, 16, 0, 0);  // 16-bit words, idle level =low
            // BEWARE - For some reason when doing 16-bit SPI transfers, SPI send MSB first in ech byte and LSByte first 
            
        // Set SPI fequency *
        spi_frequency(&spim1, SPI_FQCY_IRSENSOR_KHZ*1000);

        DBG_PRINT("\nSPI1 initalized\n");
           
}
    
// -------------------------------------------------------------

void GAPOC_IRProxy_CPI_Init()
{
    #define SENSOR_FRAME_WIDTH    80
    #define SENSOR_FRAME_HEIGHT   80
    CPI_Type *const cpi_address[] = CPI_BASE_PTRS;

        CPI_Init(cpi_address[0], CPI_PCLK, CPI_HSYNC, CPI_VSYNC,
             CPI_DATA0, CPI_DATA1, CPI_DATA2, CPI_DATA3,
             CPI_DATA4, CPI_DATA5, CPI_DATA6, CPI_DATA7); 

        UDMA_Init((UDMA_Type *)cpi_address[0]);                    

        CPI_GetDefaultConfig(&cpi_config);
        cpi_config.row_len = SENSOR_FRAME_WIDTH ; 
            // !! Keep equal to target width
            // Can normally be a portion of window width in slice mode 
            // but this is buggy on GAP8 Cut1
        cpi_config.resolution      = ( SENSOR_FRAME_WIDTH * SENSOR_FRAME_HEIGHT );
        cpi_config.format          = BYPASS_BIGEND;  // Monochrome
        cpi_config.shift           = 0;
        cpi_config.slice_en        = 0;
        cpi_config.frameDrop_en    = 0;
        cpi_config.frameDrop_value = 0;
        cpi_config.wordWidth       = 16;  // 8, 16 or 32bits // IN FACT IGNORED in CPI DRIVER [gap_cpi.c] !!!!
        DBG_PRINT("\nCPI i/f initalized\n");
 
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


