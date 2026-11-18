#include "sys.h"  

//ÓÃÓÚµ÷ÊÔµÄÏµÍ³±äÁ¿
DEBUG_t debug;

//°æ±¾¿ØÖÆÒı½Å
void Version_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//ÉÏÀ­
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//ÉÏÀ­
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//ÉÏÀ­
	GPIO_Init(GPIOC, &GPIO_InitStructure);
} 


//ÊäÈë°æ±¾²ÎÊı,·µ»Ø°æ±¾ĞÅÏ¢º¯Êı
const uint8_t* getSW_Ver(SOFTWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_00: return "V1.00";
		//´ı¶¨ºóĞø°æ±¾.
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
















