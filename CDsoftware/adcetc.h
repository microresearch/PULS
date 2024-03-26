extern void send_command(int command, void *message);
char buffx[24];


void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void EXTI4_IRQHandler(void){ // working NSR 
  uint32_t tmp, tmpp;
  if (EXTI_GetITStatus(EXTI_Line4) != RESET) { // PC4
  intflag[0]=1; //NSR
  EXTI_ClearITPendingBit(EXTI_Line4);
 }
 }

void EXTI9_5_IRQHandler(void){ // PC5 RSR works and PB6 LSR share same line but both work out
    
  if (EXTI_GetITStatus(EXTI_Line5) != RESET) { //RSR  PC5
    intflag[3]=1; //RSR
    EXTI_ClearITPendingBit(EXTI_Line5);
 }

  if (EXTI_GetITStatus(EXTI_Line6) != RESET) { //LSR PB6
    intflag[1]=1; //LSR
    EXTI_ClearITPendingBit(EXTI_Line6);
 } 

  if (EXTI_GetITStatus(EXTI_Line7) != RESET) {// CSR moved to PB7
    intflag[2]=1; //CSR
    EXTI_ClearITPendingBit(EXTI_Line7);
 } 
}

void TIM4_IRQHandler(void) 

{
  uint32_t temp;
  static uint32_t modecnt=0;
  //  volatile static uint16_t tmp;
  static uint32_t flipperr=0;
  
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update); 

  /*  
      tmp^=1;
      if (tmp) GPIOB->BSRRH = (1)<<4;  // clear bits PB2
      else   GPIOB->BSRRL=(1)<<4; //  write bits   
  */

  // map ADCs:

    // 0: nspd, 1: nlen, 2: nmode
    // 3: lspd, 4: llen, 5: lmode
    // 6: rspd, 7: rlen, 8: rmode // adc6 fixed hw
    // 9: cspd, 10: clen, 11: cmode

  // modes are NOT inverted!
  // maybe we can slow down modes
    modecnt++; 
    if (modecnt>256){ // 128: what is the speed of this? 2/1 measured as 10 Hz (so 20x second ok) 256 say
    /* // flipper is on fake clock for CC
    flipperr^=1;
    if (flipperr) GPIOB->BSRRH=clk_route_new[2]; // we get from tail
     else GPIOB->BSRRL=clk_route_new[2];
    */
      modecnt=0;
      
  //moden
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  temp=ADC_GetConversionValue(ADC1);
  mode[0]=mapping[temp>>2];
  if (lastmode[0]!=mode[0]) {
    gate[0].changed=1;
  }
  else gate[0].changed=0;
  lastmode[0]=mode[0];
  
  // modec
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  temp=ADC_GetConversionValue(ADC1);
  mode[2]=mapping[temp>>2];

  if (lastmode[2]!=mode[2]) {
    gate[2].changed=1;
  }
  else gate[2].changed=0;
  lastmode[2]=mode[2];

  // model
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  temp=ADC_GetConversionValue(ADC1);
  mode[1]=mapping[temp>>2];

  if (lastmode[1]!=mode[1]) {
    gate[1].changed=1;
  }
  else gate[1].changed=0;
  lastmode[1]=mode[1];

  // moder we dont need to record
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  temp=ADC_GetConversionValue(ADC1);
  mode[3]=mapping[temp>>2];
  
  if (lastmode[3]!=mode[3]) gate[3].changed=1; // bug fixed 28/1/
  else gate[3].changed=0;
  lastmode[3]=mode[3];
    }
  
  // speedn
  totn=totn-smoothn[nn];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  smoothn[nn]=ADC_GetConversionValue(ADC1);
  totn+=smoothn[nn];
  nn++;
  if (nn>=SMOOTHINGS) nn=0;
  temp=totn/SMOOTHINGS;  
  CV[0]=4095-temp;

  // speedl
  totl=totl-smoothl[ll];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  smoothl[ll]=ADC_GetConversionValue(ADC1);
  totl+=smoothl[ll];
  ll++;
  if (ll>=SMOOTHINGS) ll=0;
  temp=totl/SMOOTHINGS;

  CV[1]=4095-temp;
  
  // speedr
  totr=totr-smoothr[rr];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  smoothr[rr]=ADC_GetConversionValue(ADC1);
  totr+=smoothr[rr];
  rr++;
  if (rr>=SMOOTHINGS) rr=0;
  temp=totr/SMOOTHINGS;  
  CV[3]=4095-temp;
  
  // speedc
  totc=totc-smoothc[cc];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles); // was 10
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
   smoothc[cc]=ADC_GetConversionValue(ADC1);
    totc+=smoothc[cc];
    cc++;
    if (cc>=SMOOTHINGS) cc=0;
    temp=totc/SMOOTHINGS;
    CV[2]=4095-temp;

  // CVL0 NN
  totnn=totnn-smoothnn[nnn];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  smoothnn[nnn]=ADC_GetConversionValue(ADC1);
  totnn+=smoothnn[nnn];
  nnn++;
  if (nnn>=SMOOTHINGS) nnn=0;
  temp=totnn/SMOOTHINGS;  
  CVL[0]=4095-temp;

  
  // CVL1 LL
  totll=totll-smoothll[lll];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  smoothll[lll]=ADC_GetConversionValue(ADC1);
  totll+=smoothll[lll];
  lll++;
  if (lll>=SMOOTHINGS) lll=0;
  temp=totll/SMOOTHINGS;  
  CVL[1]=4095-temp;

  // CVL2 CC
  totcc=totcc-smoothcc[ccc];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  temp=ADC_GetConversionValue(ADC1);
  smoothcc[ccc]=ADC_GetConversionValue(ADC1);
  totcc+=smoothcc[ccc];
  ccc++;
  if (ccc>=SMOOTHINGS) ccc=0;
  temp=totcc/SMOOTHINGS;  
  CVL[2]=4095-temp;
  
  // CVL3 RR
  totrr=totrr-smoothrr[rrr];
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_144Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  smoothrr[rrr]=ADC_GetConversionValue(ADC1);
  totrr+=smoothrr[rrr];
  rrr++;
  if (rrr>=SMOOTHINGS) rrr=0;
  temp=totrr/SMOOTHINGS;  
  CVL[3]=4095-temp;
}
