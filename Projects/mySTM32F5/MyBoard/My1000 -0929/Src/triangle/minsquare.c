/**************************************************************************************************
  Filename:       minsquare.c
  Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
  Revision:       $Revision: 44370 $

  Description:    minsquare
Author : Kai Zhao
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */



#include "minsquare.h"
#include "my_ranging.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */



/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */



/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
 extern status_info_t set_info;
 extern device_info DeviceList[MAX_DEV_NUM];

void MinsqProccess(void){
  float Orixyz[3]={0,0,0};
  float dertaxyz[3]={0,0,0};
	int num=0;
	
  for (int i = 0; i < MAX_ANC_NUM; i++){
		if(DeviceList[i]._isactivate==1 && DeviceList[i].SEQ_range==getSEQ()){
			Orixyz[0]+=DeviceList[i].x;
			Orixyz[1]+=DeviceList[i].y;
			Orixyz[2]+=DeviceList[i].z;
			if(DeviceList[i]._range<0){
				DeviceList[i]._range=0;
			}
			num++;
		}
   
  }
	if(num<3){
		return;
	}
	
	
  Orixyz[0]=Orixyz[0]/num;
  Orixyz[1]=Orixyz[1]/num;
	Orixyz[2]=Orixyz[1]/num;

  Cordinate tagtmp;
  tagtmp.xyz[0]=Orixyz[0];
  tagtmp.xyz[1]=Orixyz[1];
  tagtmp.xyz[2]=Orixyz[2];
  
  float M[3*MAX_ANC_NUM];
  float Mt[MAX_ANC_NUM*3];
  float A[9];
  float Ainv[9];
  float b[MAX_ANC_NUM];
  float temp[MAX_ANC_NUM*3];
  int isSmallEnough=0;
  int loopCounter=0;
  
  while(isSmallEnough==0 && loopCounter<MAXLOOP){
    tagtmp.xyz[0]+=dertaxyz[0];
    tagtmp.xyz[1]+=dertaxyz[1];
    tagtmp.xyz[2]+=dertaxyz[2];
		num=0;
    for (int i = 0; i < MAX_ANC_NUM; i++){
			if(DeviceList[i]._isactivate==1 && DeviceList[i].SEQ_range==getSEQ()){
				M[3*num]  =2*(-DeviceList[i].x+tagtmp.xyz[0]);
				M[3*num+1]=2*(-DeviceList[i].y+tagtmp.xyz[1]);
				M[3*num+2]=2*(-DeviceList[i].z+tagtmp.xyz[2]);
				b[num]=DeviceList[i]._range*DeviceList[i]._range  -(M[3*i]*M[3*i]+M[3*i+1]*M[3*i+1]+M[3*i+2]*M[3*i+2])/4;
				num++;
			}
    }
	
    MatrixTran(M,Mt,num,3);
    MatricMul(Mt,M,A,3,num,3);
    MatrixRev3(A,Ainv);
    MatricMul(Ainv,Mt,temp,3,3,num);
    MatricMul(temp,b,dertaxyz,3,num,1);
    if(dertaxyz[0]<0.01 && dertaxyz[1]<0.01 && dertaxyz[2]<0.01 ){
      isSmallEnough=1;
    }
    loopCounter++;
  }
  tagtmp.xyz[0]+=dertaxyz[0];
  tagtmp.xyz[1]+=dertaxyz[1];
  tagtmp.xyz[2]+=dertaxyz[2];
	
	set_info.CurrentXYZ[0]=tagtmp.xyz[0];
  set_info.CurrentXYZ[1]=tagtmp.xyz[1];
	set_info.CurrentXYZ[2]=tagtmp.xyz[2];
  
}


  

/*********************************************************************
*********************************************************************/
