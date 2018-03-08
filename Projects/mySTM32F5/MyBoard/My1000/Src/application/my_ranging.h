/*! ----------------------------------------------------------------------------
 *  @file    my_ranging.h
 *  @brief   DecaWave header for application level 
 *
 * @attention
 *
 * @author Kai
 */
#ifndef _MY_RANGING_H_
#define _MY_RANGING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "port.h"
#include "deca_types.h"
#include "deca_device_api.h"

/******************************************************************************************************************
********************* NOTES on Decaranging EVK1000 application features/options ***********************************************************
*******************************************************************************************************************/


#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power
#define CORRECT_TIME_BY_RePWR (1)
//#define ANCHOR_ORI_OUTPUT_ENABLE  (1)  //for anchor to output original data for Antenna Delay Computation

	#define DEBUG_ENABLE        //either   Debug or BLEUART would be defined
//#define TAG_BLEUART_OUTPUT    (1)
#if TAG_BLEUART_OUTPUT==1
#define OUTPUT_SPI_OR_UART    (0)  // UART=1 SPI=0	
#endif
/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/


#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.


#define USING_64BIT_ADDR (0) //when set to 0 - the DecaRanging application will use 16-bit addresses


//! callback events
#define DWT_SIG_RX_NOERR            0
#define DWT_SIG_TX_DONE             1       // Frame has been sent
#define DWT_SIG_RX_OKAY             2       // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE          6       // ACK frame has been sent (as a result of auto-ACK)
#define DWT_SIG_RX_BLINK			7		// Received ISO EUI 64 blink message
#define DWT_SIG_RX_PHR_ERROR        8       // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT       11      // Got preamble detection timeout (no preamble detected)

#define DWT_SIG_TX_PENDING          12      // TX is pending
#define DWT_SIG_TX_ERROR            13      // TX failed
#define DWT_SIG_RX_PENDING          14      // RX has been re-enabled
#define DWT_SIG_DW_IDLE             15      // DW radio is in IDLE (no TX or RX pending)

#define SIG_RX_UNKNOWN			99		// Received an unknown frame

// Existing frames type in ranging process.
#define MAX_EVENT_NUMBER  4
enum
{
    BLINK = 0,
    RNG_INIT,
    POLL,
    RESP,
    FINAL,
    FRAME_TYPE_NB
};

#define FC_1 0x41
#define FC_1_BLINK 0xC5
#define FC_2 0x8C
#define FC_2_SHORT 0x88

#define PAN_ID_1 0xCA
#define PAN_ID_2 0xDE
#define BROADCAST_1 0xFF
#define BROADCAST_2 0xFF

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define BLINK 4
#define RANGING_INIT 5

#define MAX_DEV_NUM 6


#define FRAME_CRC					2
#define BLINK_LEN  (1+1+8+2+FRAME_CRC)
#define ADD_LEN_S 2
#define ADD_LEN_L 8
#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2


#if USING_64BIT_ADDR==1
#define ADD_LEN ADD_LEN_L
#define FRAME_HEADER_LEN (2+1+2+ADD_LEN+ADD_LEN+1)
#else
#define ADD_LEN ADD_LEN_S
#define FRAME_HEADER_LEN (2+1+2+ADD_LEN+ADD_LEN+1)
#endif
#define STANDARD_FRAME_SIZE         127

#define MAX_ANC_NUM   6

#define RNG_INIT_FRAME_LEN_BYTES   (FRAME_HEADER_LEN+  ADD_LEN_L+ADD_LEN_S+FRAME_CRC)
#define POLL_FRAME_LEN_BYTES       (FRAME_HEADER_LEN+ (ADD_LEN+2)*MAX_ANC_NUM+FRAME_CRC)
#define RANGE_FRAME_LEN_BYTES       (FRAME_HEADER_LEN+ 10+(ADD_LEN+5)*MAX_ANC_NUM+FRAME_CRC)
#define RESP_FRAME_LEN_BYTES       (FRAME_HEADER_LEN +FRAME_CRC)
#define FINAL_FRAME_LEN_BYTES      (FRAME_HEADER_LEN+  ADD_LEN+8+FRAME_CRC)


#define BLINK_SLEEP_DELAY					250 //ms
#define POLL_SLEEP_DELAY					500 //ms
#define CHECK_DEV_INTERVAL        500 //ms
#define DEVICE_TIMEOUT_PERIOD     2000   //remove a dvice if  it did not response in 2S



//in us
#define DEFAULT_REPLY_DELAY_TIME 70000     //for preamble 1024  in us
#define DEFAULT_RANGE_DELAY  10400  //us
#define MINUM_FINAL_DELAY         1000   //us

#define DEFAULT_STATUS_CHECK_INTERVAL  10000

#define IMMEDIATE_RESPONSE (1)


// Response time possible units: microseconds or milliseconds.
#define RESP_DLY_UNIT_US 0
#define RESP_DLY_UNIT_MS 1


static const float TIME_RES     = 0.000015650040064103f;     //to us
static const float TIME_RES_INV = 63897.6f;     //to us
static const float DISTANCE_OF_RADIO     = 0.0046917639786159f;
static const float DISTANCE_OF_RADIO_INV = 213.139451293f;
static const uint8 FAULT_TIME_MASK[5]={0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint32 RANGE_SEND_DELAY= TIME_RES_INV*DEFAULT_RANGE_DELAY ;

#define USE_OTP_ANTD  0
static const uint16 INPUT_ANT_DELAY=    16384;
static const uint16  ANT_DELAY_ROW[9]=  {16384,               //@
																							  16384+79,       //A
																								16384+88,       //B
																								16384+92,       //C
																								16384+67,       //D
																								16384  ,       //E
																								16384+83,       //F
																								16384+104,      //G
																								16384};
static const int64_t TIME_OVERFLOW = 0x10000000000; //1099511627776LL

static const uint64  Min_Final_interval=DEFAULT_REPLY_DELAY_TIME*TIME_RES_INV;

//static const float AnchorXYZ[3*8]={1,1,0,
//																						 0,1.5,1.8,         //A
//																						 2,7,0,             //B
//																						 5.01,1.5,1.8,      //C
//																						 2.6,2.5,1.2,       //D
//																			 			 2,1,0,             //E
//																		  			2,1,0,              //F
//																			  		2.5,0.1,1 };            //G
static const float AnchorXYZ[3*8]={1,1,0,
																						 0.25,1,2.35,         //A
																						 0,0,0,             //B
																						 0,0,0,      //C
																						 0.25,10.94,2.34,       //D
																			 			 0,0,0,             //E
																		  			7.66,8,2.35,              //F
																			  		7.7,3.81,2.36 };            //G


typedef uint64_t        uint64 ;

typedef int64_t         int64 ;


typedef enum Modes{LISTENER, TAG, ANCHOR, TAG_TDOA, NUM_MODES} INST_MODE;

typedef enum RangingSteps{
	STEP_IDLE,
	//for tags
	STEP_BLINK_SENDING,
	STEP_BLINK_SENT,
	STEP_POLL_SENDING,
	STEP_POLL_SENT,
	STEP_RANGE_PULLED,  //set timer to sent range
	STEP_RANGE_SENT,
	
	//for anchors
	STEP_RANGE_INIT_SENDING,
	STEP_RANGE_INIT_SENT,
	STEP_POLL_ACK_PULLED,
	STEP_POLL_ACK_SENT,
	STEP_FINAL_PULLED,
	STEP_FINAL_SENT,
	
	
}RANGING_STEP;

//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above

typedef enum inst_states
{
    TA_INIT, //0

} INST_STATES;

typedef struct
{
	uint8  type;			// event type

	uint8  isactivate;	    // set if there is a pending event (i.e. DW is not in IDLE (TX/RX pending)
	uint16 dataLength ;
	uint8 databuf[30];

	uint64 timeStamp ;		// last timestamp (Tx or Rx)

	uint32 timeStamp32l ;		   // last tx/rx timestamp - low 32 bits
	uint32 timeStamp32h ;		   // last tx/rx timestamp - high 32 bits

}event_data_t ;

typedef enum event_type
{
	SendRangeInit,
	SendFinal,
	SendPollAck,
}event_type;


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message


typedef struct
{
    uint8 channelNumber ;       // valid range is 1 to 11
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO;  //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} InitConfig_t ;


/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/




typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;



typedef struct
{
	INST_MODE mode;				//instance mode (tag or anchor)
	INST_STATES AppState ;
	
	//configuration structures
	dwt_config_t    configData ;	//DW1000 channel configuration
	uint16			txAntennaDelay ; //DW1000 TX antenna delay
	uint16			rxAntennaDelay ; //DW1000 RX antenna delay
	uint8 antennaDelayChanged;
	// "MAC" features
    uint8 frameFilteringEnabled ;	//frame filtering is enabled

    // Is sleeping between frames enabled?
    uint8 sleepingEabled; //Ranging Init message can tell tag to stay in IDLE between ranging exchanges

	//timeouts and delays

	int tagBlinkSleepTime_ms;

	dwt_txconfig_t  configTX ;		//DW1000 TX power configuration


	uint16 delayedReplyTime;		

    // Pre-computed frame lengths for frames involved in the ranging process,
    // in microseconds.
    uint32 frameLengths_us[FRAME_TYPE_NB];        //for time saving and higher freq
		
	uint64 timePollSent;
	uint64 timeRangeSent;
	uint8 timePollSentBytes[5];
	uint8 timeRangeSentBytes[5];

	//Tag function address/message configuration
		uint8   eui64[8];				// devices EUI 64-bit address
		uint8  ShortAdd[2];		    // Tag's short address (16-bit) used when USING_64BIT_ADDR == 0
//   uint8   frameSN;				// modulo 256 frame sequence number - it is incremented for each new frame transmittion


    double clockOffset ;


  uint8 tagListLen ;

	uint8 tagList[8];
	uint8 smartPowerEn;
	
	uint32 Time_next_blink;
	uint32 Time_next_Dev_Check;
	uint32 Time_next_Poll;
	uint32 Time_next_Ranging;
	uint32 Time_next_Trinagle;
	uint32 Time_next_status_Check;
	uint8 active_device_num;
	
	event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
		
  uint8 ranging_step;
	int last_send_tag_idx;
	uint8 poll_replied_num;
	uint8 final_recieved_num;
	uint16 recieve_flag;
	float CurrentXYZ[3];

} status_info_t ;




typedef struct
{
	//device ID
	uint8         _ownAddress[8];
	uint8         _shortAddress[2];
	uint8         _isactivate;

	uint16_t     _replyDelayTimeUS;

	
	float _range;
	int16_t _RXPower;
	int16_t _FPPower;
	int16_t _quality;
        
        //functions which contains the date: (easier to put as public)
	// timestamps to remember    
	
	// for tag
  uint64 timePollSent;
	uint64 timePollAckReceived;
	uint64 timeRangeSent;
	
	uint8 timePollSentBytes[5];
	uint8 timePollAckReceivedBytes[5];
	uint8 timeRangeSentBytes[5];
	
	//for Anchor
	uint64 timePollReceived;
	uint64 timePollAckSent;
	uint64 timeRangeReceived;
	uint64 timeFinalSent;

	uint8 timePollReceivedBytes[5];
	uint8 timeRangeReceivedBytes[5];
	uint8 timePollAckSentBytes[5];
	
	uint32 timeDelayToSet;   //dwt API need
	uint64 dertatime;       //read from poll
	
	uint8 step_inAnchor_fortag;
	uint32 lastActionTime;
	int64_t round1;
	int64_t round2;
	int64_t reply1;
	int64_t reply2;
	float timeT;
	float x;
	float y;
	float z;
	uint8 SEQ_range;
}device_info;


// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
static const uint8   BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31, 0, 36, 65, 84, 97, 106, 110, 112};
static const uint8   BIAS_500_64[] = {110, 105, 100, 93, 82, 69, 51, 27, 0, 21, 35, 42, 49, 62, 71, 76, 81, 86};
static const uint8   BIAS_900_16[] = {137, 122, 105, 88, 69, 47, 25, 0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
static const uint8   BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};
  #define       BIAS_500_16_ZERO  10
	#define       BIAS_500_64_ZERO  8
	#define       BIAS_900_16_ZERO  7
	#define       BIAS_900_64_ZERO  7

//ST's HAL layer adds extra delay compared to bare metal implementation
//#define CUBEMX_DELAY (100)

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// configure TX/RX callback functions that are called from DW1000 ISR
void instance_rxerrorcallback(const dwt_cb_data_t *rxd);
void instance_rxtimeoutcallback(const dwt_cb_data_t *rxd);
void instance_rxgoodcallback(const dwt_cb_data_t *rxd);
void instance_txcallback(const dwt_cb_data_t *txd);

//app_functions
int init_CB_enable_IRQ(void);
void instance_init_timings(void);
uint64 instance_get_addr(void);
void instancerxon( int delayed, uint64 delayedReceiveTime);
	
//app_common
int instance_init(void);
void instance_config(InitConfig_t *config);
void instancesettagsleepdelay(int sleepdelay, int blinksleepdelay); //sleep in ms
void tag_loop(void);
void anchor_loop(void);
void DeviceListInit(void);

float getReceivePower(void);

void MinsqProccess(void);
uint8 getSEQ(void);

status_info_t* instance_get_local_structure_ptr(void);

#ifdef __cplusplus
}
#endif

#endif
