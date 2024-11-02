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

enum A { A0, A1, A2, A3, A4, A5, A6, A7, 
         A8, A9,A10,A11,A12,A13,A14,A15};

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

  c_cdreg(&controller_ext,branch,crate,24,0);
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
  int crate;
  int slot;
  int branch=0;

  int i;
  int reg[16];
  int controller_reg;

  int sequence=0;

  int value;
  int Q;


  while(1)
    {
      fprintf(stdout, "Enter CRATE number which has PHILIPS scaler: ");
      if(fscanf(stdin,"%d",&crate) == 1)
	{
	  if(init_camac(crate))break;
	  fprintf(stdout,"Could not initialise crate %d\n",crate);
	}
    }

  c_cdreg(&controller_reg, branch, crate, 24, 0);

  while(1)
    {
      fprintf(stdout, "Enter SLOT number containing PHILIPS scaler: ");
      if(fscanf(stdin,"%d",&slot) == 1)
	if((slot>=1)&&(slot<=24))break;
      fprintf(stdout,"Invalid slot number %d\n",slot);
    }

  for(i=0; i<16; i++)c_cdreg(&reg[i], branch, crate, slot, i);

  value=0;
  c_cfsa(F11,reg[A0],&value /* dummy */,&Q,0,1);
  c_cfsa(F11,reg[A1],&value /* dummy */,&Q,0,1);
  c_cfsa(F11,reg[A2],&value /* dummy */,&Q,0,1);
  c_cfsa(F11,reg[A3],&value /* dummy */,&Q,0,1);
  c_cfsa(F11,reg[A5],&value /* dummy */,&Q,0,1);
  c_cfsa(F11,reg[A12],&value /* dummy */,&Q,0,1);
  c_cfsa(F11,reg[A13],&value /* dummy */,&Q,0,1);
  
  while(1)
    {
      int counts[32];
      int Qs[32];

      sequence++;

      /* Inhibit Crate */
      value=1;
      c_ccci(controller_reg,value /* inhibit */,0,1);

      /* Reset Scaler -- Configure for 32 x 24bit scalers */
      c_cfsa(F11,reg[A0],&value /* dummy */,&Q,0,1); /* i.e. F11, A0 */

      /* Clear Scaler */
      c_cfsa(F11,reg[A4],&value /* dummy */,&Q,0,1); /* i.e. F11, A4 */

      /* Un-Inhibit Crate */
      value=0;
      c_ccci(controller_reg,value /* inhibit */,0,1);

      c_wait_time(2.0);

      /* Inhibit Crate */
      value=1;
      c_ccci(controller_reg,value /* inhibit */,0,1);

      /* First reset the scaler bank selection register */      
      c_cfsa(F11,reg[A1],&value /* dummy */,&Q,0,1); /* i.e. F11, A1 */

       /* Next loop over all channels in unit and store scaler value */
      for(i=0;i<32;i++)
	{
	  if(i==0){value=0; c_cfsa(F17,reg[A1],&value,&Q,0,1); }
	  else if(i==16){value=1; c_cfsa(F17,reg[A1],&value,&Q,0,1); }

	  c_cfsa(F0,reg[i%16],&counts[i],&Qs[i],0,1); 
	}
	
      /*      fputc(12,stdout);
	      for(i=0;i<20;i++)fputs("\n",stdout);*/
      fprintf(stdout,"Sequence Number: %-8.8d\n",sequence);
      for(i=0;i<16;i++)
	fprintf(stdout,"%2d %8d (%1d)     %2d %8d (%1d)\n",
		i,counts[i]&0x00FFFFFF,Qs[i],
		i+16,counts[i+16]&0x00FFFFFF,Qs[i+16]);

      /* Un-Inhibit Crate */
      value=0;
      c_ccci(controller_reg,value /* inhibit */,0,1);
    }

}
