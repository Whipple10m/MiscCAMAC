#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void            c_cdreg();
void            ccfini();
unsigned int    c_cccc();
unsigned int    c_cccz();
unsigned int    c_ccci();
unsigned long   c_cssa();
unsigned long   c_cfsa();
unsigned long   initusr();

enum F { F0,  F1,  F2,  F3,  F4,  F5,  F6,  F7,  F8,  F9, F10,
        F11, F12, F13, F14, F15, F16, F17, F18, F19, F20, F21,
        F22, F23, F24, F25, F26, F27, F28, F29, F30, F31};

enum N { N0,  N1,  N2,  N3,  N4,  N5,  N6,  N7,  N8,  N9, N10,
        N11, N12, N13, N14, N15, N16, N17, N18, N19, N20, N21,
        N22, N23, N24, N25 };

enum A {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10,
       A11, A12, A13, A14, A15};

typedef char    Boolean_op ;
#ifndef SUCCESS
#define SUCCESS    1
#define FAILURE    0
#endif

Boolean_op tst_status(status)
unsigned long status ;
{
  void LIB$SIGNAL() ;
  
  if(status & 1) {
    return(SUCCESS) ;
  }
  else {
    printf("\n");
    LIB$SIGNAL(status) ;
    return(FAILURE) ;
  }
}

int init_camac(int crate)      /* Buckley's */
{
  unsigned int    status=0,controller_ext=0;
  int             q=0,w_data=0;
  unsigned long   signal=0;
  int             branch=0;

  signal = initusr() ;    /* Initialize communication with crate cntrller */
  if(!tst_status(signal)) {
    return(0);
  }

  c_cdreg(&controller_ext,branch,crate,N24,0);
  c_cccz(controller_ext,0,1); /* !Generate dataway initialize - crate*/ 
  c_cccc(controller_ext,0,1); /* !Generate crate clear - crate */
  c_ccci(controller_ext,0,0,1);/* !Turn off Inhibit - crate */
	
  return(1);
}/**** end subroutine init_camac ****/

void c_wait_time(float delay)
{
        time_t	time() ;
	time_t	*ptime ;
        time_t  finaltime, currenttime ;

        time(&finaltime) ;
        finaltime += (time_t)(delay + 0.5) ;
        do {
          time(&currenttime) ;
        } while(currenttime<finaltime) ;
        
}	   	

int main(int argc, char** argv)
{
  int branch=0;

  int i;
  int reg[16];

  int crates[] = { 1,2,3,4,5 };
#define ncrates (sizeof(crates)/sizeof(*crates))
  int controller_reg[ncrates];
  int malu_reg[ncrates][2];
  int sob_event_gate_reg;
  int lp_reset_reg[ncrates];

  for(i=0;i<ncrates;i++)init_camac(crates[i]);
  for(i=0;i<ncrates;i++)c_cdreg(&controller_reg[i], branch, crates[i], N24,A0);
  for(i=0;i<ncrates;i++)c_cdreg(&malu_reg[i][0], branch, crates[i], N21, A0);
  for(i=0;i<ncrates;i++)c_cdreg(&malu_reg[i][1], branch, crates[i], N21, A1);
  c_cdreg(&sob_event_gate_reg, branch, 3, N22, A1);
  for(i=0;i<ncrates;i++)c_cdreg(&lp_reset_reg[i], branch, crates[i], N23, A0);

  for(i=0;i<ncrates;i++)
    {
      int value=1;
      int Q;

      /* Disable List Processors, which may compete to read in MALU */
      c_cssa(F24,lp_reset_reg[i],&value,&Q,0,1);

      /* Clear MALU */
      c_cssa(F9,malu_reg[i][0],&value,&Q,0,1);
    }

  while(1)
    {
      static crate_index = 0;
      int counts[2];
      int value;
      int Q;

      if(crate_index == 0)
	{
	  /* Open the event gate to let one event through */
	  value=1;
	  c_cssa(F25,sob_event_gate_reg,&value,&Q,0,1);
	}

      Q=0;
      while(Q==0)
	{
	  /* Test the LAM on the MALU */
	  c_cfsa(F8,malu_reg[crate_index][0],&value /*dummy*/,&Q, 0, 1);
	}

      /* Read MALU channels 1-16 */
      c_cfsa(F2,malu_reg[crate_index][0],&counts[0],&Q,0,1);

      /* Read MALU channels 1-16 */
      c_cfsa(F2,malu_reg[crate_index][1],&counts[1],&Q,0,1);

      printf("%d: %5d %5d -- %10d -- 0x%8.8x 0x%8.8x \n",crates[crate_index],
	     counts[1]&0xFFFF,counts[0]&0xFFFF,
	     ((counts[1]&0xFFFF)<<16)+counts[0]&0xFFFF,counts[1],counts[0]);
      
      crate_index++;
      if(crate_index==ncrates)
	{
	  crate_index=0;
	  printf("\n");
	}
    }

}
