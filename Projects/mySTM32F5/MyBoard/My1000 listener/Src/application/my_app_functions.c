/*! ----------------------------------------------------------------------------
 *  @file    my_app_functions.c
 *  @brief   DecaWave application level message exchange for ranging 
 *.
 *
 * @author Kai
 */
#include "compiler.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "deca_regs.h"

#include "lib.h"

#include "my_ranging.h"

// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
//
// function to construct the message/frame header bytes
//
// -------------------------------------------------------------------------------------------------------------------



// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int init_CB_enable_IRQ(void)
{


    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);

    //this is platform dependent - only program if DW EVK/EVB
//    dwt_setleds(3) ; //configure the GPIOs which control the LEDs on EVBs

    dwt_setcallbacks(instance_txcallback, instance_rxgoodcallback, instance_rxtimeoutcallback, instance_rxerrorcallback);


    return 0 ;
}


extern uint8 dwnsSFDlen[];

// Pre-compute frame lengths, timeouts and delays needed in ranging process.
// /!\ This function assumes that there is no user payload in the frame.
void instance_init_timings(void)
{
    status_info_t* inst = instance_get_local_structure_ptr();
    uint32 pre_len;
    int sfd_len;

//	  static const int data_len_bytes[FRAME_TYPE_NB] = {
//        BLINK_FRAME_LEN_BYTES, RNG_INIT_FRAME_LEN_BYTES, POLL_FRAME_LEN_BYTES,
//        RESP_FRAME_LEN_BYTES, FINAL_FRAME_LEN_BYTES};
//    int i;
    // Margin used for timeouts computation.
//    const int margin_sy = 10 + CUBEMX_DELAY; //For ST's HAL/Cube Mx need bigger margin/longer timout as "immediate" response comes later

    // All internal computations are done in tens of picoseconds before
    // conversion into microseconds in order to ensure that we keep the needed
    // precision while not having to use 64 bits variables.

    // Compute frame lengths.
    // First step is preamble plus SFD length.
    sfd_len = dwnsSFDlen[inst->configData.dataRate];
    switch (inst->configData.txPreambLength)
    {
    case DWT_PLEN_4096:
        pre_len = 4096;
        break;
    case DWT_PLEN_2048:
        pre_len = 2048;
        break;
    case DWT_PLEN_1536:
        pre_len = 1536;
        break;
    case DWT_PLEN_1024:
        pre_len = 1024;
        break;
    case DWT_PLEN_512:
        pre_len = 512;
        break;
    case DWT_PLEN_256:
        pre_len = 256;
        break;
    case DWT_PLEN_128:
        pre_len = 128;
        break;
    case DWT_PLEN_64:
    default:
        pre_len = 64;
        break;
    }
    pre_len += sfd_len;
    // Convert preamble length from symbols to time. Length of symbol is defined
    // in IEEE 802.15.4 standard.
    if (inst->configData.prf == DWT_PRF_16M)
        pre_len *= 99359;
    else
        pre_len *= 101763;
    // Second step is data length for all frame types.
//    for (i = 0; i < FRAME_TYPE_NB; i++)
//    {
//        // Compute the number of symbols for the given length.
//        inst->frameLengths_us[i] = data_len_bytes[i] * 8
//                         + CEIL_DIV(data_len_bytes[i] * 8, 330) * 48;
//        // Convert from symbols to time and add PHY header length.
//        if(inst->configData.dataRate == DWT_BR_110K)
//        {
//            inst->frameLengths_us[i] *= 820513;
//            inst->frameLengths_us[i] += 17230800;
//        }
//        else if (inst->configData.dataRate == DWT_BR_850K)
//        {
//            inst->frameLengths_us[i] *= 102564;
//            inst->frameLengths_us[i] += 2153900;
//        }
//        else
//        {
//            inst->frameLengths_us[i] *= 12821;
//            inst->frameLengths_us[i] += 2153900;
//        }
//        // Last step: add preamble length and convert to microseconds.
//        inst->frameLengths_us[i] += pre_len;
//        inst->frameLengths_us[i] = CEIL_DIV(inst->frameLengths_us[i], 100000);
//    }
//    // Final frame wait timeout time.
//    inst->fwtoTime_sy = US_TO_SY_INT(inst->frameLengths_us[FINAL])
//                        + RX_START_UP_SY + margin_sy;
//    // Ranging init frame wait timeout time.
//    inst->fwtoTimeB_sy = US_TO_SY_INT(inst->frameLengths_us[RNG_INIT])
//                         + RX_START_UP_SY + margin_sy;
//    // Delay between blink transmission and ranging init reception.
//    inst->rnginitW4Rdelay_sy =
//        US_TO_SY_INT((RNG_INIT_REPLY_DLY_MS * 1000) - inst->frameLengths_us[BLINK])
//        - RX_START_UP_SY;
//    // Delay between anchor's response transmission and final reception.
//    inst->txToRxDelayAnc_sy = US_TO_SY_INT(TAG_TURN_AROUND_TIME_US) - RX_START_UP_SY - CUBEMX_DELAY;

//    // No need to init txToRxDelayTag_sy here as it will be set upon reception
//    // of ranging init message.

//    // Delay between blink reception and ranging init message transmission.
//    inst->rnginitReplyDelay = convertmicrosectodevicetimeu(RNG_INIT_REPLY_DLY_MS * 1000);
//    // Delay between poll reception and response transmission. Computed from
//    // poll reception timestamp to response transmission timestamp so you have
//    // to add poll frame length to delay that must be respected between frames.


    // Smart Power is automatically applied by DW chip for frame of which length
    // is < 1 ms. Let the application know if it will be used depending on the
    // length of the longest frame.
    if (inst->frameLengths_us[FINAL] <= 1000)
        inst->smartPowerEn = 1;
    else
        inst->smartPowerEn = 0;
}
uint64 instance_get_addr(void) //get own address
{
    status_info_t* inst = instance_get_local_structure_ptr();
    uint64 x = (uint64) inst->eui64[0];
    x |= (uint64) inst->eui64[1] << 8;
    x |= (uint64) inst->eui64[2] << 16;
    x |= (uint64) inst->eui64[3] << 24;
    x |= (uint64) inst->eui64[4] << 32;
    x |= (uint64) inst->eui64[5] << 40;
    x |= (uint64) inst->eui64[6] << 48;
    x |= (uint64) inst->eui64[7] << 56;


    return (x);
}

// -------------------------------------------------------------------------------------------------------------------
//
// Turn on the receiver with/without delay
//
void instancerxon( int delayed, uint64 delayedReceiveTime)
{
    if (delayed)
    {
        uint32 dtime;
        dtime =  (uint32) (delayedReceiveTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

    dwt_rxenable(delayed) ;  //- as when fails -1 is returned             // turn receiver on, immediate/delayed

} // end instancerxon()

