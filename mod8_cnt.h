/* =================================================================================
File name:        MOD6_CNT.H          
===================================================================================*/

#ifndef __MOD8_CNT_H__
#define __MOD8_CNT_H__

typedef struct { Uint32  TrigInput;   	// Input: Modulo 6 counter trigger input - Q0 (0x00000000 or 0x00007FFF)
				 Uint32  Counter;	    // Output: Modulo 6 counter output - Q0 (0,1,2,3,4,5)			
			   } MOD8CNT;

/*-----------------------------------------------------------------------------
Default initalizer for the MOD8CNT object.
-----------------------------------------------------------------------------*/                     
#define MOD8CNT_DEFAULTS { 0,0 }

/*------------------------------------------------------------------------------
	MOD8_CNT Macro Definition
------------------------------------------------------------------------------*/


#define MOD8CNT_MACRO(v)												\
																		\
 if (v.TrigInput > 0)													\
   {																	\
     if (v.Counter == 7)    /* Reset the counter when it is 7 */		\
       v.Counter = 0;													\
     else																\
       v.Counter++;         /* Otherwise, increment by 1 */				\
   }																	

#endif // __MOD_8CNT_H__
