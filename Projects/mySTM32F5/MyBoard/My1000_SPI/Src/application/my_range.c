/*! ----------------------------------------------------------------------------
 *  @file    my_range.c
 *  @brief   main loop for the aRanging application
 *
 * @author Kai Zhao
 */
/* Includes */

#include "port.h"

#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"
#include "stdio.h"

#define SOFTWARE_VER_STRING    "Version 0.1" //

#define SWS1_TXSPECT_MODE	0x80  //Continuous TX spectrum mode
#define SWS1_ANC_MODE 		0x08  //anchor mode
#define SWS1_SHF_MODE		0x10  //short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE		0x20  //64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE		0x40  //channel 5 mode (switch S1-7)

//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
instanceConfig_t chConfig[8] ={
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
                        5,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};