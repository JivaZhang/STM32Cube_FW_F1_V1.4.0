/*! ----------------------------------------------------------------------------
 *  @file    my_app_common.c
 *  @brief   DecaWave application level common instance functions
 *
 * @author Kai
 */
#include "compiler.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_spi.h"

#include "my_ranging.h"

extern const uint16 rfDelays[2];
extern const tx_struct txSpectrumConfig[8];


static status_info_t set_info;


static uint8 msg_recieve[POLL_FRAME_LEN_BYTES];




void anchor_loop(void);



void instance_txcallback(const dwt_cb_data_t *txd)
{

		instancerxon(0,0);
		
		
}

void instance_rxtimeoutcallback(const dwt_cb_data_t *rxd)
{
	writetoLCD( 1, 1, "D");
	instancerxon(0,0);
}

void instance_rxerrorcallback(const dwt_cb_data_t *rxd)
{
	writetoLCD( 1, 1, "C");
	instancerxon(0,0);

}

void instance_rxgoodcallback(const dwt_cb_data_t *rxd)
{



		switch(rxd->fctrl[0])
		{
			
			//if get a blink msg
			case FC_1_BLINK:
			

				writetoLCD( 1, 1, "B"); //send some data

			break;
			case FC_1:
				dwt_readrxdata((uint8 *)&msg_recieve[0], rxd->datalength, 0);  // Read Data Frame	
			   //if get a range init msg
				if(msg_recieve[5+2*ADD_LEN]==RANGING_INIT){
					writetoLCD( 1, 1,"I" ); //send some data				


				}else if(msg_recieve[9]==POLL_ACK){
					writetoLCD( 1, 1,"A" );

				}else if(msg_recieve[9]==FINAL){
					writetoLCD( 1, 1,"F" );
		
				}else if(msg_recieve[9]==POLL){
					writetoLCD( 1, 1,"P" );

				}else if(msg_recieve[9]==RANGE){
					writetoLCD( 1, 1,"R" );
					
				}
				
				
			break;
		}
	set_info.RxStatus=0;
	
			


}

#define TXCFG_ADDRESS  (0x10) // OTP address at which the TX power calibration value is stored
							  // The TX power configuration read from OTP (6 channels consecutively with PRF16 then 64, e.g. Ch 1 PRF 16 is index 0 and PRF 64 index 1)
#define ANTDLY_ADDRESS (0x1C) // OTP address at which the antenna delay calibration value is stored
extern const uint8 chan_idx[];

// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device opetation
//
void instance_config(InitConfig_t *config)
{

    uint32 power = 0;

    set_info.txAntennaDelay = 0;
    set_info.rxAntennaDelay = 0;

    set_info.configData.chan = config->channelNumber ;
    set_info.configData.rxCode =  config->preambleCode ;
    set_info.configData.txCode = config->preambleCode ;
    set_info.configData.prf = config->pulseRepFreq ;
    set_info.configData.dataRate = config->dataRate ;
    set_info.configData.txPreambLength = config->preambleLen ;
    set_info.configData.rxPAC = config->pacSize ;
    set_info.configData.nsSFD = config->nsSFD ;
    set_info.configData.phrMode = DWT_PHRMODE_STD ;
    set_info.configData.sfdTO = config->sfdTO;

    //configure the channel parameters
    dwt_configure(&set_info.configData) ;

    //NOTE: For EVK1000 the OTP stores calibrated antenna and TX power values for configuration modes 3 and 5,

    //check if to use the antenna delay calibration values as read from the OTP
    if(dwt_otprevision() <= 1) //in revision 0, 1 of EVB1000/EVK1000
    {
    	uint32 antennaDelay;
    	uint32 otpPower[12];

    	//MUST change the SPI to < 3MHz as the dwt_otpread will change to XTAL clock
    	port_set_dw1000_slowrate(); //reduce SPI to < 3MHz

    	dwt_otpread(ANTDLY_ADDRESS, &antennaDelay, 1);

    	set_info.txAntennaDelay = ((antennaDelay >> (16*(config->pulseRepFreq - DWT_PRF_16M))) & 0xFFFF) >> 1;

    	set_info.rxAntennaDelay = set_info.txAntennaDelay ;

    	//read any data from the OTP for the TX power
    	dwt_otpread(TXCFG_ADDRESS, otpPower, 12);

    	port_set_dw1000_fastrate(); //increase SPI to max

        power = otpPower[(config->pulseRepFreq - DWT_PRF_16M) + (chan_idx[set_info.configData.chan] * 2)];
    }

    // if nothing was actually programmed then set a reasonable value anyway
    if(set_info.txAntennaDelay == 0)//otherwise a default values should be used
    {
    	set_info.rxAntennaDelay = set_info.txAntennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
    }

    // -------------------------------------------------------------------------------------------------------------------
    // set the antenna delay, we assume that the RX is the same as TX.
    dwt_setrxantennadelay(set_info.txAntennaDelay);
    dwt_settxantennadelay(set_info.txAntennaDelay);

    if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    set_info.configTX.power = power;
    set_info.configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

    //configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&set_info.configTX);

    set_info.antennaDelayChanged = 0;

    if(config->preambleLen == DWT_PLEN_64) //if preamble length is 64
	{
    	port_set_dw1000_slowrate(); //reduce SPI to < 3MHz

		dwt_loadopsettabfromotp(0);

		port_set_dw1000_slowrate(); //increase SPI to max
    }
    set_info.RxStatus=0;


}

// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init(void)
{

    int result;
    //uint16 temp = 0;

    set_info.mode =  ANCHOR;                                // assume listener,

    // Reset the IC (might be needed if not getting here from POWER ON)
    dwt_softreset();

	//we can enable any configuration loding from OTP/ROM on initialisation
    result = dwt_initialise(DWT_LOADUCODE) ;

    //this is platform dependent - only program if DW EVK/EVB
//    dwt_setleds(3) ; //configure the GPIOs which control the leds on EVBs

    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialise has failed
    }

    //enable TX, RX states on GPIOs 6 and 5
//    dwt_setlnapamode(1,1); //硬件引脚没接？

 //   instanceclearcounts() ;

    set_info.sleepingEabled = 1;





    dwt_geteui(set_info.eui64);

    set_info.clockOffset = 0;
    return 0 ;
}


// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------
/* @fn 	  instance_get_local_structure_ptr
 * @brief function to return the pointer to local instance data structure
 * */
status_info_t* instance_get_local_structure_ptr(void)
{
	return &set_info;
}
//////////////////////////////////////////////////
///////////////////Main Loop /////////////////////
//////////////////////////////////////////////////


void anchor_loop(void){
	if(set_info.RxStatus ==0){
		instancerxon(0,0);
		set_info.RxStatus=1;
	}

	
}



