/**************************************************************************************************
  Filename:       simpleGATTprofile.c
  Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
  Revision:       $Revision: 44370 $

  Description:    This file contains the Simple GATT profile sample GATT service 
                  profile for use with the BLE sample application.

  Copyright 2010 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */


#include "matrix.h"

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


/*********************************************************************** 
* 函数名称：  乘法
* MatricMul() 
* 
*函数参数： 
*  int a[]  -第一个矩阵 
*  int b[]  -第二个矩阵 
*  int c[]  -结果矩阵 
*  int m    -第一个矩阵为m行 
*  int Middle  -第一个矩阵为Middle列，第二个矩阵为Middle行 
*  int n    -第二个矩阵为n列 
*  返回值： 无返回值 
* 
*  说明：无 
***********************************************************************/  
  
void MatricMul(float a[],float b[],float c[],int m,int Middle,int n)  
{  
    for (int i=0;i<m;i++ )  
    {  
        for (int j=0;j<n;j++)  
        {  
            float sum = 0;  
            for (int k=0;k<Middle;k++)  
            {  
                sum += a[i*Middle+k] * b[k*n+j];  
            }  
            c[i*n+j] = sum;  
        }  
    }  
}  

/*********************************************************************** 
* 函数名称： 转置矩阵
* Matrixtran() 
* 
*函数参数： 
*  int a[]  -第一个矩阵 
*  int c[]  -结果矩阵 
*  int m    -第一个矩阵为m行 
*  int n    -第一个矩阵为n列 
*  返回值： 无返回值 
* 
*  说明：无 
***********************************************************************/  
void MatrixTran(float a[],float c[],int n,int m){
  for (int i = 0; i < n; i++) {  
            for (int j = 0; j < m; j++) {  
                *(c+j*n+i) = *(a+i*m+j);  
            }  
        }  
}

/*********************************************************************** 
* 函数名称： 三阶求逆矩阵
* MatrixRev3() 
* 
*函数参数： 
*  int a[]  -第一个矩阵 
*  int c[]  -结果矩阵 
*  返回值： 无返回值 
* 
*  说明：无 
***********************************************************************/  
void MatrixRev3(float a[],float c[]) {
  float A= (a[0]*(a[4]*a[8]-a[5]*a[7])     -a[3]*(a[1]*a[8]-a[2]*a[7])             +a[6]*(a[1]*a[5]-a[2]*a[4]));
              
   c[0]=(a[4]*a[8]-a[5]*a[7])/A;
   c[1]=(a[2]*a[7]-a[1]*a[8])/A;
   c[2]=(a[1]*a[5]-a[2]*a[4])/A;
   c[3]=(a[5]*a[6]-a[3]*a[8])/A;
   c[4]=(a[0]*a[8]-a[2]*a[6])/A;
   c[5]=(a[3]*a[2]-a[0]*a[5])/A;
   c[6]=(a[3]*a[7]-a[6]*a[4])/A;
   c[7]=(a[1]*a[6]-a[0]*a[7])/A;
   c[8]=(a[0]*a[4]-a[3]*a[1])/A;
   
   
}


///*********************************************************************** 
//* 函数名称： 求逆矩阵
//* Matrixtran() 
//* 
//*函数参数： 
//*  int a[]  -第一个矩阵 
//*  int c[]  -结果矩阵 
//*  int m    -第一个矩阵为m行 
//*  int n    -第一个矩阵为n列 
//*  返回值： 无返回值 
//* 
//*  说明：无 
//***********************************************************************/  
//void MatrixRev(float a[],float c[],int m) {  
//        // 先是求出行列式的模|data|  
//        float A = getHL(a,m);  
//        // 创建一个等容量的逆矩阵  
//
//        float cc[3*MaxAnchorNum]; 
//        if(m==2 ){
//        	cc[0] = a[3]/ A;
//        	cc[3] = a[0]/ A;
//        	cc[1] = a[1]/ A;
//        	cc[2] = a[2]/ A;
//        	return ;
//        }
//  
//        for (int i = 0; i < m; i++) {  
//            for (int j = 0; j < m; j++) {  
//                float num;  
//                if ((i + j) % 2 == 0) {  
//                   float d[2*(MaxAnchorNum-1)];
//                   getDY(a,d,m,m, i + 1, j + 1);
//                    num = getHL(d,m-1);  
//                } else { 
//                  float d[2*(MaxAnchorNum-1)];
//                  getDY(a,d,m,m, i + 1, j + 1);
//                    num = -getHL(d,m-1);  
//                }  
//
//                *(cc+i*m+j) = num / A;  
//            }  
//        }  
//  
//
//  
//        MatrixTran(cc, c,m);
//}  
//
//
//
//void getDY(float data[],float newData[],int H,int V, int h, int v) {  
////        float newData[(H-1)*(V-1)];  
//  
//        for (int i = 0; i < H-1; i++) {  
//  
//            if (i < h - 1) {  
//                for (int j = 0; j < V-1; j++) {  
//                    if (j < v - 1) {  
//                        newData[i*(V-1)+j] = data[i*V+j];  
//                    } else {  
//                        newData[i*(V-1)+j] = data[i*V+j + 1];  
//                    }  
//                }  
//            } else {  
//                for (int j = 0; j < V-1; j++) {  
//                    if (j < v - 1) {  
//                        newData[i*(V-1)+j] = data[(i + 1)*V+j];  
//                    } else {  
//                        newData[i*(V-1)+j] = data[(i + 1)*V+j+ 1];  
//                    }  
//                }  
//  
//            }  
//        }  
//
//  
//    }
//
//float getHL(float data[], int num){
//  // 终止条件  
//        if (num == 2) {  
//            float total = 0; 
//            total =data[0] * data[3] - data[1] * data[2];  
//            return total;
//        }  
//  
//        float total = 0;  
//        // 根据data 得到行列式的行数和列数  
//
//        // 创建一个大小为num 的数组存放对应的展开行中元素求的的值  
//        float nums[MaxAnchorNum];  
//  
//        for (int i = 0; i < num; i++) {  
//            if (i % 2 == 0) {  
//              float d[2*(MaxAnchorNum-1)];
//              getDY(data,d,num,num, 1, i + 1);
//                nums[i] = data[i] * getHL(d,num-1);  
//            } else {  
//              float d[2*(MaxAnchorNum-1)];
//              getDY(data,d,num,num, 1, i + 1);
//                nums[i] = -data[i] * getHL(d,num-1);  
//            }  
//        }  
//        for (int i = 0; i < num; i++) {  
//            total += nums[i];  
//        }   
//        return total;  
//}



/*********************************************************************
*********************************************************************/
