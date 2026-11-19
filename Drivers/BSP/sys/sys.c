#include "sys.h"  

//ÓÃÓÚµ÷ÊÔµÄÏµÍ³±äÁ¿
DEBUG_t debug;

// æ‰¹æ¬¡5ï¼šåˆ é™¤ Version_GPIO_Init æ­»ç 
// ï¼ˆåŸè“å›¾æ¡ç›®ï¼šæ“ä½œ PE3/PE2/PC13ï¼Œä¸‰è„šä¸åœ¨ 43 å¼•è„š C50C_HAL.ioc
// å†…ï¼Œä¸” grep è¯å®å…¨æ ‘æ— è°ƒç”¨è€…ï¼ŒæŒ‰è“å›¾é£é™©4 åˆ é™¤è€Œéè¿ç§»ï¼›ç¡¬ä»¶ç‰ˆæœ¬
// æ£€æµ‹ç”±å…¶å®ƒé€”å¾„æ‰¿æ‹…ï¼Œè¿™é‡Œä¿ç•™é™æ€é»˜è®¤å€¼ï¼‰ã€‚
// æ‰¹æ¬¡5 è¯¯åˆ æ¢å¤ getSW_Ver
// ï¼ˆåˆ  Version_GPIO_Init æ—¶ NEL æ··åˆè¡Œç»ˆæ­¢ç¬¦å¯¼è‡´ b\\n åˆ†å‰²è¯¯åˆ¤å‡½æ•°è¾¹ç•Œï¼Œæœ¬å‡½æ•°è¢«è¿å¸¦åˆ é™¤ï¼›æŒ‰åŸæ ·æ¢å¤ï¼Œshow_task.c:288 è°ƒç”¨ï¼‰
// è¾“å…¥ç‰ˆæœ¬å‚æ•°,è¿”å›ç‰ˆæœ¬ä¿¡æ¯å‡½æ•°
const uint8_t* getSW_Ver(SOFTWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_00: return "V1.00";
		//å¾…å®šåç»­ç‰ˆæœ¬.
	}
	
	return "unknown";
}



const uint8_t* getHW_Ver(HARDWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_0: return "V1.0";
		case V1_1: return "V1.1";
		case V1_2: return "V1.2";
		//´ı¶¨ºóĞø°æ±¾.
	}
	
	return "unknown";
}


//ÏµÍ³¼¶±äÁ¿³õÊ¼»¯
//ÉèÖÃÈíÓ²¼ş°æ±¾ĞÅÏ¢.Ó²¼ş°æ±¾ÓÉÓ²¼ş³õÊ¼»¯ºó²ÅÄÜÈ·¶¨.
void SYS_VAL_t_Init(SYS_VAL_t* p)
{
	//µ±Ç°Èí¼ş°æ±¾
	p->SoftWare_Ver = V1_00;
	
	p->HardWare_Ver = HW_NONE;//´ıÓ²¼ş³õÊ¼»¯ºóÈ·¶¨
	
	p->Time_count = 0;
	p->HardWare_charger = 0;
	p->HardWare_Ranger = 0;
	p->SecurityLevel = 1; //ÏµÍ³°²È«µÈ¼¶,Ä¬ÈÏÉèÖÃ1.²»´øÃüÁî¶ªÊ§±£»¤
	p->LED_delay = 1; //ÏµÍ³×´Ì¬Ö¸Ê¾µÆÉÁË¸¼ä¸ô,µ¥Î» ms
}



//THUMBÖ¸Áî²»Ö§³Ö»ã±àÄÚÁª
//²ÉÓÃÈçÏÂ·½·¨ÊµÏÖÖ´ĞĞ»ã±àÖ¸ÁîWFI  
/* GNU ç‰ˆæœ¬ï¼šKeil çš„ __asm å‡½æ•°æ— æ³•è¢« arm-none-eabi-gcc ç¼–è¯‘ï¼Œ
 * è¿™é‡Œæ”¹ç”¨ GCC å†…è”æ±‡ç¼–å®ç°åŒæ ·è¯­ä¹‰ã€‚ */
void WFI_SET(void)
{
  __asm volatile ("wfi");
}
/* å…³é—­å¯å±è”½ä¸­æ–­ï¼ˆä¸å« fault / NMIï¼‰ */
void INTX_DISABLE(void)
{
  __asm volatile ("cpsid i" ::: "memory");
}
/* æ‰“å¼€å¯å±è”½ä¸­æ–­ */
void INTX_ENABLE(void)
{
  __asm volatile ("cpsie i" ::: "memory");
}
/* è®¾ç½®ä¸»æ ˆé¡¶åœ°å€ */
void MSR_MSP(u32 addr)
{
  __asm volatile ("msr msp, %0" : : "r" (addr) : "memory");
}
















