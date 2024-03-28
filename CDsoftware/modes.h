#define MAXDEL 64000
#define MAXN 7

typedef struct heavens_ { 
  uint32_t trigger;
  uint32_t changed;
  uint32_t pulsek[2], pulsekk[2]; // process of rising trigger on input - also for incoming output

  // reset and other options as 5 bits
  uint32_t opts;
  
  float kin;
  float kout;
  uint32_t pulsein[2];
  uint32_t pulseout[2];

  uint32_t pulseup[2];
  uint32_t pulselen[2];
  uint32_t pulsestart[2];
  uint32_t pulsecnt[2];
  uint32_t routein;
  
  // delay line
  uint32_t delay_line[MAXDEL/32]; // local
  float Rspeed; // for local delay line
  uint32_t Rlength;
  float Rcount;
  uint32_t Rstart;
  
  float Pspeed;
  uint32_t Plength;
  float Pcount;
  uint32_t Pstart;
  
  float SRspeed; // for shared delay line
  uint32_t SRlength;
  float SRcount;
  uint32_t SRstart;
  
  float SPspeed;
  uint32_t SPlength;
  float SPcount;
  uint32_t SPstart;
  //  float (*Nfunc)(float input, uint32_t reset);
  uint32_t neuron; // which function includes fake pass through
  uint32_t process;
  // and say 7 params for each of the x neurons
  float Nparam[MAXN][7];
  float Nstat[MAXN][4]; // statics which hold values
} heavens;


