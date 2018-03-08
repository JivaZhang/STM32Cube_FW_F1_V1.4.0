/*! ----------------------------------------------------------------------------
 *  @file    dw_main.c
 *  @brief   main loop for the aRanging application
 *
 * @author Kai Zhao
 */
/* Includes */

#include "port.h"

#include "my_ranging.h"

#include "deca_types.h"

#include "deca_spi.h"
#include "stdio.h"

#define SOFTWARE_VER_STRING    "Version 0.1" //

#define SWS1_TXSPECT_MODE	0x80  //Continuous TX spectrum mode
#define SWS1_ANC_MODE 		0x08  //anchor mode
#define SWS1_SHF_MODE		0x10  //short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE		0x20  //64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE		0x40  //channel 5 mode (switch S1-7)


int dr_mode = 0;


uint8 s1switch = 0 | SWS1_SHF_MODE | SWS1_64M_MODE | SWS1_CH5_MODE ;
int chan, tagaddr, ancaddr, prf;

#define LCD_BUFF_LEN (100)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];

int ranging = 0;

//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
InitConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
						3,              // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,               // channel
                        9,               // preambleCode
                        DWT_PRF_64M,     // prf
                        DWT_BR_110K,     // datarate
                        DWT_PLEN_1024,   // preambleLength
                        DWT_PAC32,       // pacSize
                        1,               // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
//                    //mode 8
//                    {
//                        5,              // channel
//                        9,             // preambleCode
//                        DWT_PRF_64M,    // prf
//                        DWT_BR_6M8,    // datarate
//                        DWT_PLEN_128,   // preambleLength
//                        DWT_PAC8,       // pacSize
//                        0,       // non-standard SFD
//                        (129 + 8 - 8) //SFD timeout
//                    }
//										{
//                        2,              // channel
//						3,              // preambleCode
//                        DWT_PRF_16M,    // prf
//                        DWT_BR_110K,    // datarate
//                        DWT_PLEN_4096,  // preambleLength
//                        DWT_PAC32,      // pacSize
//                        0,       // non-standard SFD
//                        (4097 + 64 - 32) //SFD timeout
//                    }

{
                        2,              // channel
						3,              // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    }

};


uint32 inittestapplication(uint8 s1switch);


int decarangingmode(uint8 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_64M_MODE)
    {
        mode = mode + 2;
    }
    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 4;
    }

    return mode;
}
uint32 inittestapplication(uint8 s1switch)
{
    uint32 devID ;
    int result;

    port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = dwt_readdevid() ;
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_wakeup_dw1000();

        devID = dwt_readdevid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred

    port_set_dw1000_fastrate();
    devID = dwt_readdevid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    

    init_CB_enable_IRQ();
    dr_mode = decarangingmode(s1switch);

    chan = chConfig[dr_mode].channelNumber ;
    prf = (chConfig[dr_mode].pulseRepFreq == DWT_PRF_16M)? 16 : 64 ;

    instance_config(&chConfig[dr_mode]) ;                  // Set operating channel etc

    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time

//    instance_init_timings();
		
		DeviceListInit();   //initial device list

    return devID;
}


/*
 * @fn      main()
 * @brief   main entry point
**/
int dw_main(void)
{

//    peripherals_init();

    port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

//    s1switch = port_is_boot1_on(0) << 1 // is_switch_on(TA_SW1_2) << 2
//    		| port_is_switch_on(TA_SW1_3) << 2
//    		| port_is_switch_on(TA_SW1_4) << 3
//    		| port_is_switch_on(TA_SW1_5) << 4
//		    | port_is_switch_on(TA_SW1_6) << 5
//    		| port_is_switch_on(TA_SW1_7) << 6
//    		| port_is_switch_on(TA_SW1_8) << 7;
			status_info_t* inst = instance_get_local_structure_ptr();
	
				uint8 temp[2]={0x00,0x0A};
				if(GPIO_ReadInputDataBit(SWITCH_IO_GROUP, SWITCH_BIT5)==1){
					temp[0]=0x01 | temp[0];
#ifdef DEBUG_ENABLE	
					writetoLCD( 1, 1, "1");
#endif
				}else{
#ifdef DEBUG_ENABLE	
					writetoLCD( 1, 1, "0");
#endif
				}
				if(GPIO_ReadInputDataBit(SWITCH_IO_GROUP, SWITCH_BIT6)==1){
					temp[0]=0x02 | temp[0];
#ifdef DEBUG_ENABLE	
					writetoLCD( 1, 1, "1");
#endif
				}else{
#ifdef DEBUG_ENABLE	
					writetoLCD( 1, 1, "0");
#endif
				}
				if(GPIO_ReadInputDataBit(SWITCH_IO_GROUP, SWITCH_BIT7)==1){
					temp[0]=0x04 | temp[0];
#ifdef DEBUG_ENABLE						
					writetoLCD( 1, 1, "1");
#endif
				}else{
#ifdef DEBUG_ENABLE	
					writetoLCD( 1, 1, "0");
#endif
				}
				if(GPIO_ReadInputDataBit(SWITCH_IO_GROUP, SWITCH_BIT8)==1){
					temp[0]=0x08 | temp[0];
#ifdef DEBUG_ENABLE						
					writetoLCD( 1, 1, "1");
#endif
				}else{
#ifdef DEBUG_ENABLE	
					writetoLCD( 1, 1, "0");
#endif
				}
				
	
			if(port_is_switch_on(TA_TorA)){
				
				memcpy(inst->ShortAdd,temp,2);
				
				s1switch =s1switch |SWS1_ANC_MODE;
#ifdef DEBUG_ENABLE	
				memset(dataseq, 0x0, sizeof(dataseq));
				memcpy(dataseq, (const uint8 *) "Anchor", 6);
				writetoLCD( 6, 1, dataseq); //send some data
#endif

			}else{
				
				memcpy(&inst->ShortAdd[0],temp,2);
				
#ifdef DEBUG_ENABLE								
				memset(dataseq, 0x0, sizeof(dataseq));
				memcpy(dataseq, (const uint8 *) "Tag", 3);
				writetoLCD( 3, 1, dataseq); //send some data
#endif

			}
    

    {



        if(inittestapplication(s1switch) == (uint32)-1)
        {
#ifdef DEBUG_ENABLE	
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &dataseq[0]);
            memset(dataseq, ' ', LCD_BUFF_LEN);
            memcpy(dataseq, (const uint8 *) "ERROR   ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(dataseq, (const uint8 *) "  INIT FAIL ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
#endif
            return 0; //error
        }

    }

		if(s1switch & SWS1_ANC_MODE)
    {
        inst->mode = ANCHOR;
    }
    else
    {
        inst->mode = TAG;
    }
    port_EnableEXT_IRQ();
    // main loop
    while(1)
    {
			if(inst->mode == TAG){
				tag_loop();
			}else{
				anchor_loop();
			}

    }



}






