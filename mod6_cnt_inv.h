/* =================================================================================
File name:        MOD6_CNT_INV.H          
===================================================================================*/

#ifndef __MOD6_CNT_INV_H__
#define __MOD6_CNT_INV_H__

typedef struct { Uint32  TrigInput;   	// Input: Modulo 6 counter trigger input - Q0 (0x00000000 or 0x00007FFF)
				 Uint32  Counter;	    // Output: Modulo 6 counter output - Q0 (0,1,2,3,4,5)
				 Uint32 Direction;
			   } MOD6CNTINV;	            

/*-----------------------------------------------------------------------------
Default initalizer for the MOD6CNTINV object.
-----------------------------------------------------------------------------*/                     
#define MOD6CNTINV_DEFAULTS { 0,0,1 }

/*------------------------------------------------------------------------------
	MOD6_CNT_INV Macro Definition
------------------------------------------------------------------------------*/


#define MOD6CNTINV_MACRO(v)												\
																		\
 if (v.TrigInput > 0)													\
   {																	\
			   if (v.Counter == 0)    /* Reset the counter when it is 0 */		\
				v.Counter = 5;													\
				else																\
				v.Counter--;         /* Otherwise, decrement by 1 */				\
   }
#endif // __MOD_6CNT_INV_H__
