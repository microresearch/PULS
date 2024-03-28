/*
NEURON/IAM
  */ 

#include "stm32f4xx.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "adc.h"
#include "stdlib.h"
#include <math.h>
#include "resources.h"
#include "modes.h"

#define FULLINE 64000 // this length is /32 for array so 16000 is say 2 seconds 64 is 8

uint32_t delay_line[FULLINE/32]; // single delay line - 32 bits x FULLLINE = 64000 at 16khz=4 seconds???

static heavens gate[4]; 

static uint32_t CV[4]={1,1,1,1};
static uint32_t CVL[4]={1,1,1,1};

#define LOWEST 0.000005f
volatile uint32_t intflag[4]={0,0,0,0}; // interrupt flag...

// 100 is lowest
#define delay()						 do {	\
    register unsigned int ix;					\
    for (ix = 0; ix < 100; ++ix)				\
      __asm__ __volatile__ ("nop\n\t":::"memory");		\
  } while (0)

static uint32_t binary[5]={0,0,0,0}; // binary global routing
static uint32_t ADCin;

static uint32_t count=0; // for route
static uint32_t daccount=0; // for dacfrom
static uint32_t spdcount=0; // for spdfrom
static uint32_t tailcount=0; // for tail choice

#include "macros.h"

#define FULL 0b11111111111111111111111111111111 //32 bits full
#define HALB 0b10101010101010101010101010101010 //32 bits 1010
#define MAXVALUE 4095
#define SMOOTHINGS 32 // we can hold 65536 of our 16 bit ADC values...speed

uint32_t cc=0, totc=0, smoothc[SMOOTHINGS]={0};
uint32_t ll=0, totl=0, smoothl[SMOOTHINGS]={0};
uint32_t rr=0, totr=0, smoothr[SMOOTHINGS]={0};
uint32_t nn=0, totn=0, smoothn[SMOOTHINGS]={0};

uint32_t ccc=0, totcc=0, smoothcc[SMOOTHINGS]={0};
uint32_t lll=0, totll=0, smoothll[SMOOTHINGS]={0};
uint32_t rrr=0, totrr=0, smoothrr[SMOOTHINGS]={0};
uint32_t nnn=0, totnn=0, smoothnn[SMOOTHINGS]={0};


uint32_t mode[4]={0,0,0,0};
uint32_t lastmode[4]={1,1,1,1};
//uint32_t lastmodec, lastmoden, lastmodel, lastmoder;
//uint32_t lastlastmodec, lastlastmoden, lastlastmodel, lastlastmoder;

//volatile uint32_t intflag[4]={0,0,0,0}; // interrupt flag...

uint32_t clksr_[4]={HALB,HALB,HALB,HALB};
uint32_t clksr__[4]={0,0,0,0};
uint32_t clksrG_[4]={0,0,0,0};
 
// for generic CLK fake puls routing
//LSRCLK-pulse9=PB12, RSRCLK-pulse10=PB13, CSRCLCK-pulse11=PB14
// 000,001,010,011,100,101,110,111
static uint32_t clk_route[8]={0,
			      (1<<12),
			      (1<<13),
			      (1<<12) | (1<<13),
			      (1<<14),
			      (1<<12) | (1<<14),
			      (1<<14) | (1<<13),
			      (1<<12) | (1<<13) | (1<<14)
};

// for single clk routes
static uint32_t clk_route_new[4]={0, // no clk route 0 
			      (1<<12),
  			      (1<<14),
			      (1<<13)
};

static uint32_t LFSR_[4]={0xf0fff,0xf0ff,0xff00f,0xff};
static uint32_t LFSR__[4]={0xff,0xff,0xff,0xff};
static uint32_t ADCshift_[4]={0xffff,0xffff,0xffff,0xffff};
static uint32_t ADCGshift_[4]={0xffff,0xffff,0xffff,0xffff};
static uint32_t Gshift__[5]={0,0,0,0,0};

static uint32_t GGshift_[4][4]={ // for freezers
 {0xff,0xff,0xff,0xff},
  {0xff,0xff,0xff,0xff},
  {0xff,0xff,0xff,0xff},
  {0xff,0xff,0xff,0xff}
};

static uint32_t GGGshift_[4]; // gshift is 4 even though we don't use one // GG is ghost in ghost

#include "tables.h" // routing and other tables

uint32_t sieve[4]={3,3,3,3}; // previous one... - changed to R 21/12/2021
uint32_t oppose[4]={2,3,0,1};

static uint32_t train[4]={0,0,0,0};

static uint32_t prev_stat[4]={0,0,0,0};

static uint32_t pulsins[4]={0,1<<8,0,1<<7}; //N,L,C,R
static uint32_t pulse[4]={0,1,0,1};
static uint32_t LR[4]={0,1,0,1};
static uint32_t flipd[4]={0,0,0,0};
uint32_t pulsouts[8]={0, 0,  1<<2, 1<<15, 1<<4, 1<<12, 1<<3, 1<<11};

volatile uint16_t *pulsoutHI[8]={&(GPIOB->BSRRL), &(GPIOB->BSRRL), &(GPIOB->BSRRL), &(GPIOC->BSRRL), &(GPIOB->BSRRL), &(GPIOA->BSRRL), &(GPIOB->BSRRL), &(GPIOA->BSRRL) };
//                                  0              0              PB2                PC15           PB4               PA12           PB3                PA11 
volatile uint16_t *pulsoutLO[8]={&(GPIOB->BSRRH), &(GPIOB->BSRRH), &(GPIOB->BSRRH), &(GPIOC->BSRRH), &(GPIOB->BSRRH), &(GPIOA->BSRRH), &(GPIOB->BSRRH), &(GPIOA->BSRRH) }; // both are 16 bit registers

volatile uint32_t *clkins[4]={&(GPIOC->IDR), &(GPIOB->IDR), &(GPIOB->IDR), &(GPIOC->IDR)}; // checked
uint32_t clkinpins[4]={1<<4, 1<<6, 1<<7, 1<<5};

#define DELAY_SIZE 6 // was 40 --- 3*width=16 = 3*16=48-5=43 - use 7 for simplea // used but just to hit 1
static int32_t delay_buffer[4][2] = { 0 }; // was 48 but it doesn't need to be so big

static inline void new_data(uint32_t data, uint32_t ww)
{
  delay_buffer[ww][0] = delay_buffer[ww][1];
    delay_buffer[ww][1] = data;
}

inline float sc_fold(float in, float lo, float hi) {
    float x, c, range, range2;
    x = in - lo;

    // avoid the divide if possible
    if (in >= hi) {
        in = hi + hi - in;
        if (in >= lo)
            return in;
    } else if (in < lo) {
        in = lo + lo - in;
        if (in < hi)
            return in;
    } else
        return in;

    if (hi == lo)
        return lo;
    // ok do the divide
    range = hi - lo;
    range2 = range + range;
    c = x - range2 * floorf(x / range2);
    if (c >= range)
        c = range2 - c;
    return c + lo;
}


static uint32_t ordercount=0;
static uint32_t resetz=1;
static uint32_t glob=0; // glob is now global index for global funcs 

#include "adcetc.h" // now all of the other functions so can work on modes
//#include "geogen.h" // newer generators
//#include "exp_port.h" // ports from exp...++etc includes L now
//#include "geomantic.h" // new geomantic codebase in progress
//#include "geoCC.h"
//#include "geoNN.h"
//#include "geoLL.h"
//#include "geoRR.h"
//#include "test.h"

// check slowest speed
/*
void SRspeedtest(uint8_t w){ // null
  static uint32_t tgg[4]={0,0,0,0};
  //  HEADSIN;
  speedf_[w]=logspeed[CV[w]>>2]; // 10 bits
  //  speedf_[w]=slowerlog[CV[w]>>2]; // 10 bits
  gate[w].time_now += speedf_[w];
  gate[w].last_time = gate[w].int_time;
  gate[w].int_time = gate[w].time_now;

  if(gate[w].last_time<gate[w].int_time)      {
    tgg[w]^=1;
    if (tgg[w]==1) gate[w].dac=4095;
    else gate[w].dac=0;
    gate[w].time_now-=1.0f;
    gate[w].int_time=0;	

  }
}
*/

// params to copy
float Nparams[7][7]={ {0.1f, 0.1f, 0.5f, 0.5f},
		      {1.0f, 3.0f, 1.0f, 5.0f, 0.005f, 4.0f, -1.6f},
		      {0.5f, 0.5f, -60.0f, 48.0f},
		      {0.05f, 0.03f},
		      {0.004f, 1.0f, 36.0f, -12.0f, 0.3f, 120.0f},
		      {1.0f, 1.0f, 0.0001f, 0.1f, 1.0f},
};

float Nstats[7][4]={ {0.0, 0.0},
		     {-1.6, 4.0, 2.75},
		     {0.1, 0.1},
		     {-0.65},
		     {70.0, 0.05, 0.54, 0.34},
		     {0.1, 0.1},
};

void mode_init(void){
  uint32_t x,y;
  for (x=0;x<4;x++){
    gate[x].pulsestart[0]=0;
    gate[x].pulsestart[1]=0;
    gate[x].pulselen[0]=1;
    gate[x].pulselen[1]=1;
    gate[x].pulsecnt[0]=0;
    gate[x].pulsecnt[1]=0;
    gate[x].pulseup[0]=0;
    gate[x].pulsek[0]=0;
    gate[x].pulsekk[0]=0;
    gate[x].pulsek[1]=0;
    gate[x].pulsekk[1]=0;
    gate[x].opts=0;
    gate[x].trigger=0;
    gate[x].changed=0;

  gate[x].Rspeed=1.0f;
  gate[x].Rlength=MAXDEL-1;
  gate[x].Rcount=1.0;
  gate[x].Rstart=0;
  
  gate[x].Pspeed=1.0f;
  gate[x].Plength=MAXDEL-1;
  gate[x].Pcount=1.0;
  gate[x].Pstart=0;

  gate[x].SRspeed=1.0f;
  gate[x].SRlength=FULLINE-1;
  gate[x].SRcount=0.0;
  gate[x].SRstart=0;
  
  gate[x].SPspeed=1.0f;
  gate[x].SPlength=FULLINE-1;
  gate[x].SPcount=1.0;
  gate[x].SPstart=1;

  gate[x].process=0;  
  
  for (y=0;y<MAXDEL/32;y++){
    gate[x].delay_line[y]=0;
  }
  }
  gate[0].neuron=0;
  gate[1].neuron=5;
  gate[2].neuron=5; 
  gate[3].neuron=5;
  
  for (x=0;x<7;x++){ 
    for (y=0;y<7;y++){
      gate[0].Nparam[x][y]=Nparams[x][y];
      gate[1].Nparam[x][y]=Nparams[x][y];
      gate[2].Nparam[x][y]=Nparams[x][y];
      gate[3].Nparam[x][y]=Nparams[x][y];
    }
  }

    for (x=0;x<7;x++){
    for (y=0;y<4;y++){
      gate[0].Nstat[x][y]=Nstats[x][y];
      gate[1].Nstat[x][y]=Nstats[x][y];
      gate[2].Nstat[x][y]=Nstats[x][y];
      gate[3].Nstat[x][y]=Nstats[x][y];
    }
    }

}
  
// params: b0, b1, urate, wrate
// static: u, w
float fitzhugh(float input, uint32_t reset, uint32_t w){   // Fitzhugh from SC. SLUGen
  //  float urate=0.1f, wrate=0.1f, b0=0.5f, b1=0.5f;
  float urate=gate[w].Nparam[0][0];
  float wrate=gate[w].Nparam[0][1];
  float b0=gate[w].Nparam[0][2];
  float b1=gate[w].Nparam[0][3];
  
  //  static float u=0.0, ww=0.0;
  float u=gate[w].Nstat[0][0];
  float ww=gate[w].Nstat[0][1];
  
  //  urate=CV[0]/4095.0;
  //  wrate=CVL[0]/4095.0;

  //  if (k>1024) {u=0.0; w=0.0;}// reset on comparator
  // or, reset on pulse
  if (reset) {u=0.0; ww=0.0;}
  
  //  float dudt= urate*((u-(0.33333*u*u*u)-w));
  float dudt= urate*((u-(0.33333*u*u*u)-ww + input));
  float dwdt= wrate*(b0+b1*u-ww);
  
  u+=dudt;
  ww+=dwdt;

  if ((u>1.0) || (u<-1.0)) u=fabsf(fmodf((u-1.0),4.0)-2.0)-1.0;
  gate[w].Nstat[0][0]=u;
  gate[w].Nstat[0][1]=ww;
  
  return u;
  }

// params: a,b,c,d,r,s,xR
// static: x,y,z
float hindmarshrose(float input, uint32_t reset, uint32_t w){ // CV? TODO // https://jamesmccaffrey.wordpress.com/2020/01/27/hindmarsh-rose-model-simulation-using-c-or-python/
  //  float a = 1.0;  float b = 3.0;
  //  float c = 1.0;  float d = 5.0;
  //  float r = 0.005;  float s = 4.0;
  //  float xR = -1.6; 
  float a=gate[w].Nparam[1][0];
  float b=gate[w].Nparam[1][1];
  float c=gate[w].Nparam[1][2];
  float d=gate[w].Nparam[1][3];
  float r=gate[w].Nparam[1][4];
  float s=gate[w].Nparam[1][5];
  float xR=gate[w].Nparam[1][6];
  float scale = 0.1;
  
  //  static float x = -1.6;  // membrane potential
  //  static float y = 4.0;
  //  static float z = 2.75;
  float x=gate[w].Nstat[1][0];
  float y=gate[w].Nstat[1][1];
  float z=gate[w].Nstat[1][2];

//  input*=4.0;
  float dx = scale * (y + (b * x * x) - (a * x * x * x) - z + input);
  float dy = scale * (c - (5 * x * x) - y);
  float dz = scale * (r * (s * (x - xR) - z));

   x += dx;
   y += dy;
   z += dz;
   gate[w].Nstat[1][0]=x;
   gate[w].Nstat[1][1]=y;
   gate[w].Nstat[1][2]=z;
   return x;
  }

// params: dt, aa, bb, cc, dd
// static: uu, vv
float izhikevich(float input, uint32_t reset, uint32_t w){   // Izhikevich: https://www.izhikevich.org/publications/net.m and https://www.izhikevich.org/publications/spikes.htm
  float dvdtt, dudtt;//, aa=0.5, bb=0.5, cc=-60, dd=48;
  float dt=0.01;
  float aa=gate[w].Nparam[2][0]*4.0f;
  float bb=gate[w].Nparam[2][1]*4.0f;
  float cc=gate[w].Nparam[2][2];
  float dd=gate[w].Nparam[2][3];

  //   static float vv=0.1, uu=0.1;
  float vv=gate[w].Nstat[2][0];
  float uu=gate[w].Nstat[2][1];
  
  //   aa=CV[0]/4095.0;
  //   bb=CVL[0]/4095.0;
   input=input*60.0;     
   dvdtt=((0.04*powf(vv,2))+ (5*vv) + ((140-uu)+input)) *dt;
   dudtt=(aa*(bb*vv-uu)) *dt;
   vv+=dvdtt;
   uu+=dudtt;   
   if (vv>=30) {vv=cc; uu=uu+dd;};
   gate[w].Nstat[2][0]=vv;
   gate[w].Nstat[2][1]=uu;
   return vv/60.0;
}

// params: R, C
// static: v
float LIF(float input, uint32_t reset, uint32_t w){   // Leaky integrate and fire https://github.com/Jumaruba/SNN/blob/master/Neurons/LIF.py

  //   float R = 0.05; // leakx
  //   float C = 0.03; // cap
  float uR = -0.4; // resting
  float thrs = 0.3; // threshold
  float maxV = 0.9; // pulse
  float dt=0.01;
  float dv;

  float R=gate[w].Nparam[3][0];
  float C=gate[w].Nparam[3][1];

   //   static float v = -0.65; 
  float v=gate[w].Nstat[3][0];

   //   R=CV[0]/4095.0;
   //   C=CVL[0]/4095.0;
   
   if (v >= thrs) v = uR;
     else {
       dv = ((-v + R * input) / (R * C)) * dt; //  def fu(self, v, I):        return 
       v += dv;
     }
   if (v >= thrs) v = maxV;
   gate[w].Nstat[3][0]=v;
   return v;
}

float alphaM(float V){
  return (2.5-0.1*(V+65)) / (expf(2.5-0.1*(V+65)) -1);
}
  
float betaM(float V){
  return 4*expf(-(V+65)/18);
}

float alphaH(float V){
  return 0.07*expf(-(V+65)/20);
}

float betaH(float V){
  return 1/(expf(3.0-0.1*(V+65))+1);
}

float alphaN(float V){
  return (0.1-0.01*(V+65)) / (expf(1-0.1*(V+65)) -1);
}

float betaN(float V){
  return 0.125*expf(-(V+65)/80);
}

// params: gNa0, ENa, gK0, EK, gL0, EL
// static: V,m,h,n
float HH(float input, uint32_t reset, uint32_t w){   // https://mark-kramer.github.io/Case-Studies-Python/HH.html
  float dt = 0.01;
  //  float gNa0 = 120;//   # [mS/cm^2]\n,
  //  float ENa  = 115;//  # [mV]\n,
  //  float gK0  = 36;//   # [mS/cm^2]\n,
  //  float EK   = -12;//  # [mV]\n,
  //  float gL0  = 0.3;//  # [mS/cm^2]\n,
  //  float EL   = 10.6;//CV[0]/40.0;//10.6;// # [mV]\n,
  float gNa0=gate[w].Nparam[4][5];
  float ENa=gate[w].Nparam[4][1]*200.0;
  float gK0=gate[w].Nparam[4][2];
  float EK=gate[w].Nparam[4][3];
  float gL0=gate[w].Nparam[4][4];
  float EL=gate[w].Nparam[4][0]*400.0;
  
  //  static float V=70.0, m=0.05, h=0.54, n=0.34;
  float V=gate[w].Nstat[4][0];
  float m=gate[w].Nstat[4][1];
  float h=gate[w].Nstat[4][2];
  float n=gate[w].Nstat[4][3];
  
  float dV, dM, dH, dN;
  if (reset) {V=70.0, m=0.05, h=0.54, n=0.34;};
  //  EL=CVL[0]/40.0;
  //  EK=(CV[0]/40.0)-60;
  dV = dt*(gNa0*powf(m,3)*h*(ENa-(V+65)) + gK0*powf(n,4)*(EK-(V+65)) + gL0*(EL-(V+65)) + input);
  dM = dt*(alphaM(V)*(1-m) - betaM(V)*m);
  dH = dt*(alphaH(V)*(1-h) - betaH(V)*h);
  dN = dt*(alphaN(V)*(1-n) - betaN(V)*n);

  V+=dV; m+=dM; h+=dH, n+=dN;
  gate[w].Nstat[4][0]=V;
  gate[w].Nstat[4][1]=m;
  gate[w].Nstat[4][2]=h;
  gate[w].Nstat[4][3]=n;
  return V;
}

// params: xrate, yrate, alpha, beta, eta
// static: x, y
float termanwang(float input, uint32_t reset, uint32_t w){      // TermanWang: SLUgens
  //  float xrate= 0.0001;//CV[0]/4095.0;//0.01;
  //  float yrate= 0.1;
  //  float alpha,beta;
  //  float eta=1.0;
  float xrate=gate[w].Nparam[5][2];
  float yrate=gate[w].Nparam[5][3];
  float alpha=gate[w].Nparam[5][0];
  float beta=gate[w].Nparam[5][1];
  float eta=gate[w].Nparam[5][4];
  
  //  static float x,y;
  float x=gate[w].Nstat[5][0];
  float y=gate[w].Nstat[5][1];

  //  alpha=CV[0]/4095.0; 
  //  beta=CVL[0]/4095.0;

  input*=2.0;
  float dxdt= xrate* ( (3.0*x) + (x*x*x) + 2.0 -y + (input));  //+ 2 left out since can be returned via input
  float dydt= yrate* (eta * ((alpha*(1.0+(tanhf(x*beta)))) -y));//?
  
  x+=dxdt;
  y+=dydt;
  if ((x>1.0) || (x<-1.0)) x=sc_fold(x, -1.0f, 1.0f);
  gate[w].Nstat[5][0]=x;
  gate[w].Nstat[5][1]=y;
  return x;
}

// nada
float nadan(float input, uint32_t reset, uint32_t w){ // passes through
  return input;
}  

// if we just have HH on the top one for speed?
float (*Nfunc[7])(float input, uint32_t reset, uint32_t w)={fitzhugh, hindmarshrose, izhikevich, LIF, HH, termanwang, nadan}; // would be nice to have 8 or 9 or just double up
// could also have different available params

float (*NfuncsinHH[7])(float input, uint32_t reset, uint32_t w)={fitzhugh, hindmarshrose, izhikevich, LIF, termanwang, nadan, nadan}; // would be nice to have 8 or 9 or just doubl

uint32_t remapN[8]={0,1,2,3,4,5,6,0};


// delayline
uint32_t accessbit(uint32_t *delayline, uint32_t count){
  uint32_t tmp, byte, bit;
  byte=count/32; // which byte
  bit=count-(byte*32);
  tmp=(delayline[byte]>>bit)&1;
  return tmp;
}

void writebit(uint32_t *delayline, uint32_t value, uint32_t count){
  uint32_t tmp, byte, bit;
  byte=count/32; // which byte
  bit=count-(byte*32);
  delayline[byte] &= ~(1<<bit);
  if (value==1) delayline[byte]+=(1<<bit);
}

inline static float mod0p(float value, float length, uint32_t whichone) 
{
  while (value > (length-1))
        value -= length;
  if (value<=0.0) value+=gate[whichone].SPstart;
  if (value<0.01) value=gate[whichone].SPstart;
    return value;
}

inline static float mod0r(float value, float length, uint32_t whichone) 
{
  while (value > (length-1))
        value -= length;
  if (value<=0.0) value+=gate[whichone].SRstart;
  if (value<0.01) value=gate[whichone].SRstart;
  return value;
}


//  gate[whichone].SRcount+=gate[whichone].SRspeed;
//  if (gate[whichone].SRcount>gate[whichone].SRlength) gate[whichone].SRcount=gate[whichone].SRstart;
//    f[dacc].l[0].play_cnt=mod0N(f[dacc].l[0].play_cnt+speedy, start, end, lengthy, dacc, 0);

// define basic delay line and nada for pulses
uint32_t sharedelaywr(uint32_t input, uint32_t whichone){// read and write
  // read first
  gate[whichone].SPcount=mod0p(gate[whichone].SPcount+gate[whichone].SPspeed, gate[whichone].SPlength, whichone);
  uint32_t val=accessbit(delay_line, gate[whichone].SPcount);
  // write
  gate[whichone].SRcount=mod0r(gate[whichone].SRcount+gate[whichone].SRspeed, gate[whichone].SRlength, whichone);
  writebit(delay_line, input, gate[whichone].SRcount);
  return val;
}

// read only from delay line
uint32_t sharedelayr(uint32_t input, uint32_t whichone){
  uint32_t val=accessbit(delay_line, gate[whichone].SPcount);
  gate[whichone].SPcount=mod0r(gate[whichone].SPcount+gate[whichone].SPspeed, gate[whichone].SPlength, whichone);
  return val;
}

// and further logic with what is there: XOR, OR, AND

uint32_t nadap(uint32_t input){
  return input;
}

uint32_t kinroute[4]= {3,0,1,2};

float kinf(uint32_t k, uint32_t o, uint32_t d){ // process 8 input options
  float tmp;
  switch(o){
  case 0:
    tmp=((float)k/4095.0);
    break;
  case 1:
    tmp=((float)k/4095.0);
    tmp*=gate[kinroute[d]].kout;
    break;
  case 2:
    tmp=gate[kinroute[d]].kout;
    break;
  case 3:      
    tmp=1.0f * gate[kinroute[d]].pulseout[0];
    break;
  case 4:
    tmp=1.0f * gate[d].pulsein[0]; // any stretch of this - single impulse?
    break;
  case 5:
    tmp=0.0f;
    break;
  case 6: // AND
    tmp=((float)k/4095.0);
    tmp*=gate[kinroute[d]].pulseout[0];
    break;
  case 7: // OR or XOR
    tmp=((float)k/4095.0);
    tmp*=(~gate[kinroute[d]].pulseout[0]);
    break;
  }
  return tmp;
}

uint32_t *resetter[4][4]={{&gate[0].pulsein[0], &gate[0].pulsek[0], &gate[0].pulsek[1], &gate[3].pulseout[0]},
			  {&gate[1].pulsein[0], &gate[1].pulsek[0], &gate[0].pulsek[0], &gate[0].pulseout[0]}, // as top is only one with input
			    {&gate[2].pulsein[0], &gate[2].pulsek[0], &gate[0].pulsek[0], &gate[1].pulseout[0]},
			    {&gate[3].pulsein[0], &gate[3].pulsek[0], &gate[0].pulsek[0], &gate[2].pulseout[0]}};


void TIM2_IRQHandler(void) // running with period=1024, prescale=32 at 2KHz - how fast can we run this?
// period 32, prescaler 8 = toggle of 104 KHz
// 4 and 4 we go up to 800 KHz
{
  uint32_t www=0, ww=0, first=0, flipperrr=0, bitn=1;
  uint32_t tmp, tmpp, k, tmppp, reset=0;
  static uint32_t ping=0;
  
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // needed
  //////////////////////////////////////////////////////
  // TOP MODE
  // top one here www=0
    if (intflag[0]) { // process INT
      gate[0].pulsein[0]=1;
      intflag[0]=0;
    }
    else gate[0].pulsein[0]=0;
    
  // ADC input into k;
  ADCgeneric; 
  tmpp= (gate[0].opts>>3)&7; // 8 options
  gate[0].kin=kinf(k, tmpp, 0);     

  tmp=mode[0]>>4; // 4 modes!   // mode - but mode[0] is mapped to 0-63 in mapping
  switch(tmp){
  case 0: // choose neuron and set other params: reset, neuron input: - select neuron with x. y param as nature of reset, input: no reset, pulse as reset, etc...
    gate[0].neuron=CV[0]>>9;// 8=3 bitsx
    gate[0].opts=CVL[0]>>8; //     // reset is 2 bits, input could be 3 bits // so 5 bits
    reset=*(resetter[0][gate[0].opts&3]);
    gate[0].kout=(*Nfunc[remapN[gate[0].neuron]])(gate[0].kin, reset, 0); 
    break;
  case 1: // set params for neuron, opts stay as they were...
    gate[0].Nparam[gate[0].neuron][0]=CV[0]/4095.0;
    gate[0].Nparam[gate[0].neuron][1]=CVL[0]/4095.0;
    gate[0].kout=(*Nfunc[remapN[gate[0].neuron]])(gate[0].kin, reset, 0); 
    break;
  case 2: //      2- delay line or pass through, shared or local delay line.. some params: speed, length, start
    gate[0].kout=(*Nfunc[remapN[gate[0].neuron]])(gate[0].kin, reset, 0); // run neuron though
    // what goes into delay_line, local line, and kout
    gate[0].SPlength=CV[2]<<4;
    if (gate[0].SPlength==0) gate[0].SPlength=1;
    if (gate[0].SPlength>=FULLINE) gate[0].SPlength=FULLINE-1;
    gate[0].SRlength=CVL[0]<<4;
    if (gate[0].SRlength==0) gate[0].SRlength=1;
    if (gate[0].SRlength>=FULLINE) gate[0].SRlength=FULLINE-1;
    break;
  case 3: // length of output pulses - we have delay line
    gate[0].kout=(*Nfunc[remapN[gate[0].neuron]])(gate[0].kin, reset, 0); // run neuron though
    gate[0].pulselen[0]=CV[0]>>4;
    if (gate[0].pulselen[0]==0) gate[0].pulselen[0]=1;
    gate[0].pulselen[1]=CVL[0]>>4;
    if (gate[0].pulselen[1]==0) gate[0].pulselen[1]=1;
    break;
  }

  if (gate[0].kout>0.5) tmp=1; // delay line always
  else tmp=0;
  gate[0].pulseout[0]=sharedelaywr(tmp, www);
    
  if (gate[0].kin>0.5f && gate[0].pulsekk[0]==0) { // process input into pulse
    gate[0].pulsek[0]=1; // but only once on rise
    gate[0].pulsekk[0]=1;
  }
  else if (gate[0].kin<0.5f){
    gate[0].pulsekk[0]=0;
  }
  
  // trial pulse length TEST! 100 uS pulse with current divider - how we can get shorter - other interrupt, shortest or these length, run faster with just one HH
  if (gate[0].kout>0.5f && gate[0].pulseup[0]==0) { // how to trigger only on upward pulse // *2 for 1 and 3 below
    gate[0].pulsestart[0]=1; 
    gate[0].pulseup[0]=1;
  }
  else if (gate[0].kout<0.5f) gate[0].pulseup[0]=0;
  
  if (gate[0].pulsestart[0]==1) { 
    gate[0].pulsecnt[0]++;
    if (gate[0].pulsecnt[0]>gate[0].pulselen[0]) {
      gate[0].pulsestart[0]=0;
      gate[0].pulsecnt[0]=0;
  }
  }

  // HW out
  if (gate[0].pulsestart[0]) *pulsoutLO[2]=pulsouts[2];	 
  else *pulsoutHI[2]=pulsouts[2];      
  gate[0].pulsek[0]=0; // so is refreshed
  gate[0].pulsek[1]=0;


  
  // next 3...
   for (www=1;www<4;www++){ // top is above neuron model which is also a pulse

     // pulse ins: how we could deal with coincidence detection? also different for clk/interrupts and pulse ins!
    if (intflag[www]) { // process INT
    gate[www].pulsein[0]=1;
    intflag[www]=0;
    }
    else gate[www].pulsein[0]=0;
    gate[www].pulsein[1]=!(GPIOC->IDR & pulsins[www]); 

  tmp=mode[www]>>4; // 4 modes!   // mode - but mode[www] is mapped to 0-63 in mapping
 switch(tmp){
  case 0: // choose neuron and set other params: reset, neuron input: - select neuron with x. y param as nature of reset, input: no reset, pulse as reset, etc...
    //    gate[0].Nparam[gate[0].neuron][0]=CV[0]/4095.0;
    //    gate[0].Nparam[gate[0].neuron][1]=CVL[0]/4095.0;
    gate[www].neuron=CV[www]>>9;// 8=3 bitsx
    gate[www].opts=CVL[www]>>8; //     // reset is 2 bits, input could be 3 bits // so 5 bits
    reset=*(resetter[www][gate[www].opts&3]);
    tmpp= (gate[www].opts>>3)&7; // 8 options
    gate[www].kin=gate[kinroute[www]].kout;
    gate[www].kout=(*NfuncsinHH[remapN[gate[www].neuron]])(gate[www].kin, reset, www); 
    break;
  case 1: // set params for neuron, opts stay as they were...
    gate[www].Nparam[gate[www].neuron][0]=CV[www]/4095.0;
    gate[www].Nparam[gate[www].neuron][1]=CVL[www]/4095.0;
    tmpp= (gate[www].opts>>3)&7; // 8 options
    gate[www].kin=gate[kinroute[www]].kout;
    gate[www].kout=(*NfuncsinHH[remapN[gate[www].neuron]])(gate[www].kin, reset, www); 
    break;
  case 2: //      2- delay line or pass through, shared or local delay line.. some params: speed, length, start
    tmpp= (gate[www].opts>>3)&7; // 8 options
    gate[www].kin=gate[kinroute[www]].kout;
    gate[www].kout=(*NfuncsinHH[remapN[gate[www].neuron]])(gate[www].kin, reset, www); // run neuron though
    // what goes into delay_line, local line, and kout
       gate[www].SPlength=CV[2]<<4;
       if (gate[www].SPlength==0) gate[www].SPlength=1;
       if (gate[www].SPlength>=FULLINE) gate[www].SPlength=FULLINE-1;
       gate[www].SRlength=CVL[www]<<4;
       if (gate[www].SRlength==0) gate[www].SRlength=1;
       if (gate[www].SRlength>=FULLINE) gate[www].SRlength=FULLINE-1;
       break;
  case 3: // length of output pulses - we have delay line
    tmpp= (gate[www].opts>>3)&7; // 8 options
    gate[www].kin=gate[kinroute[www]].kout;
    gate[www].kout=(*NfuncsinHH[remapN[gate[www].neuron]])(gate[www].kin, reset, www); // run neuron though
    gate[www].pulselen[0]=CV[www]>>4;
    if (gate[www].pulselen[0]==0) gate[www].pulselen[0]=1;
    gate[www].pulselen[1]=CVL[www]>>4;
    if (gate[www].pulselen[1]==0) gate[www].pulselen[1]=1;
    break;
  }

  if (gate[www].kout>0.5) tmp=1;
  else tmp=0;
  gate[www].pulseout[0]=sharedelaywr(tmp, www);
  
  if (gate[www].kin>0.5f && gate[www].pulsekk[0]==0) { // process input into pulse
    gate[www].pulsek[0]=1; // but only once on rise
    gate[www].pulsekk[0]=1;
  }
  else if (gate[www].kin<0.5f){
    gate[www].pulsekk[0]=0;
  }
  
  // trial pulse length TEST! 100 uS pulse with current divider - how we can get shorter - other interrupt, shortest or these length, run faster with just one HH
  if (gate[www].kout>0.5f && gate[www].pulseup[0]==0) { // how to trigger only on upward pulse // *2 for 1 and 3 below
    gate[www].pulsestart[0]=1; 
    gate[www].pulseup[0]=1;
  }
  else if (gate[www].kout<0.5f) gate[www].pulseup[0]=0;
  
  if (gate[www].pulsestart[0]==1) { 
    gate[www].pulsecnt[0]++;
    if (gate[www].pulsecnt[0]>gate[www].pulselen[0]) {
      gate[www].pulsestart[0]=0;
      gate[www].pulsecnt[0]=0;
  }
  }

  // we need to do for second pulse for 1 and 3
  //     gate[www].pulseout[1]=gate[www].pulseout[0]|gate[www].pulsein[1]; // OR // XOR?
  if (gate[www].pulseout[0]) { 
    gate[www].pulsestart[1]=1; 
    gate[www].pulseup[1]=1;
  }
  else if (!gate[www].pulseout[0]) gate[www].pulseup[1]=0;
  
  if (gate[www].pulsestart[1]==1) { 
    gate[www].pulsecnt[1]++;
    if (gate[www].pulsecnt[1]>gate[www].pulselen[1]) {
      gate[www].pulsestart[1]=0;
      gate[www].pulsecnt[1]=0;
  }
  }

  
   }

   // HW out: top: 2, left 3,4, lower 5, right 6,7
   // but no on-shot length - 6 timers for 6 outs
  // HW out

   if (gate[1].pulsestart[0]) *pulsoutLO[3]=pulsouts[3];	 // left
   else *pulsoutHI[3]=pulsouts[3];      
   if (gate[1].pulsestart[1]) *pulsoutLO[4]=pulsouts[4];	 
   else *pulsoutHI[4]=pulsouts[4];      

   if (gate[2].pulsestart[0]) *pulsoutLO[5]=pulsouts[5];	 // mid
   else *pulsoutHI[5]=pulsouts[5];
   
   if (gate[3].pulsestart[0]) *pulsoutLO[6]=pulsouts[6];	 // right
   else *pulsoutHI[6]=pulsouts[6];      
   if (gate[3].pulsestart[1]) *pulsoutLO[7]=pulsouts[7];	 
   else *pulsoutHI[7]=pulsouts[7];      

   gate[1].pulsek[0]=0; // so is refreshed
   gate[1].pulsek[1]=0;
   gate[2].pulsek[0]=0; // so is refreshed
   gate[2].pulsek[1]=0;
   gate[3].pulsek[0]=0; // so is refreshed
   gate[3].pulsek[1]=0;

   
   k = (int)((gate[2].kout+1.0)*2048.0); // 3 as feedback only or???
   //      k = (int)((gate[2].kout)*4095.0); 
   DAC_SetChannel1Data(DAC_Align_12b_R, k); // 1000/4096 * 3V3 == 0V8

  // any fake pulse norms?
  
}
