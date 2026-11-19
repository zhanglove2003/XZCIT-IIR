static void USART1_FN(void)
{
#if C50C_USART1_TEXT
    /* Send at most 16 bytes per 50 ms cycle; do not block the control task
       for an entire formatted line at 115200 baud. */
    static char line[512];
    static unsigned int pos, len;
    unsigned int budget = 32;
    if (pos == len) {
        int n = snprintf(line, sizeof(line),
            "C50C stop=%u mode=%u A:%ld,%ld,%d B:%ld,%ld,%d C:%ld,%ld,%d D:%ld,%ld,%d\r\n",
            (unsigned)robot_control.FlagStop, (unsigned)robot_control.ControlMode,
            (long)(robot.MOTOR_A.Target * 1000.0f), (long)(robot.MOTOR_A.Encoder * 1000.0f), robot.MOTOR_A.Output,
            (long)(robot.MOTOR_B.Target * 1000.0f), (long)(robot.MOTOR_B.Encoder * 1000.0f), robot.MOTOR_B.Output,
            (long)(robot.MOTOR_C.Target * 1000.0f), (long)(robot.MOTOR_C.Encoder * 1000.0f), robot.MOTOR_C.Output,
            (long)(robot.MOTOR_D.Target * 1000.0f), (long)(robot.MOTOR_D.Encoder * 1000.0f), robot.MOTOR_D.Output);
        if (n <= 0 || (unsigned int)n >= sizeof(line)) { pos = len = 0; return; }
        pos = 0;
        len = (unsigned int)n;
    }
    while (pos < len && budget-- > 0) uart1_send((unsigned char)line[pos++]);
#else

  unsigned char i = 0;	
	
	for(i=0; i<SEND_DATA_SIZE; i++)
	{
		uart1_send(Send_Data.buffer[i]);
	}	 
	
	if( SysVal.HardWare_Ranger==1 )
	{
		for(i=0; i<RangerFRAME_LEN; i++)
		{
			uart1_send(rangerbuffer[i]);
		}
	}
#endif
}
