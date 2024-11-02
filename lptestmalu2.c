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
unsigned long   c_cfubr();
unsigned long   initusr();

typedef enum { F0,  F1,  F2,  F3,  F4,  F5,  F6,  F7,  F8,  F9, F10,
	       F11, F12, F13, F14, F15, F16, F17, F18, F19, F20, F21,
	       F22, F23, F24, F25, F26, F27, F28, F29, F30, F31} F;

typedef enum { N0,  N1,  N2,  N3,  N4,  N5,  N6,  N7,  N8,  N9, N10,
	       N11, N12, N13, N14, N15, N16, N17, N18, N19, N20, N21,
	       N22, N23, N24, N25 } N;

typedef enum {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10,
	      A11, A12, A13, A14, A15} A;

typedef enum { Q_REQUIRE, Q_IGNORE } QREQ;

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

int init_camac(int crate, N n_controller)      /* Buckley's */
{
  unsigned int    status=0,controller_ext=0;
  int             q=0,w_data=0;
  unsigned long   signal=0;
  int             branch=0;

  signal = initusr() ;    /* Initialize communication with crate cntrller */
  if(!tst_status(signal)) {
    return(0);
  }

  c_cdreg(&controller_ext,branch,crate,n_controller,0);
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

int lp_naf(F f, N n, A a, QREQ qreq)
{
  return ((qreq&0x0001)<<14)|((n&0x001F)<<9)|((a&0x00F)<<5)|(f&0x001F);
}

typedef enum { UNCOND, NOQ, GT, LT, AWT, NOQAWT, EQL, RWDP } JMPCOND;

int lp_jmpcond(JMPCOND cnd, int a)
{
  return ((0x0003)<<14)|((cnd&0x0007)<<11)|(a&0x07ff);
}

int lp_jmp(int a)
{
  return lp_jmpcond(UNCOND,a);
}

int jmp_noq(int a)
{
  return lp_jmpcond(NOQ,a);
}

int lp_jmpgt(int a)
{
  return lp_jmpcond(GT,a);
}

int lp_jmplt(int a)
{
  return lp_jmpcond(LT,a);
}

int lp_jmpawt(int a)
{
  return lp_jmpcond(AWT,a);
}

int lp_jmpnoqawt(int a)
{
  return lp_jmpcond(NOQAWT,a);
}

int lp_jmpeql(int a)
{
  return lp_jmpcond(EQL,a);
}

int lp_jmpdwdp(int a)
{
  return lp_jmpcond(RWDP,a);
}

int main(int argc, char** argv)
{
  FILE *fp;

  int branch=0;

  int i,j;
  int reg[16];

  int crates[] = { 1,2,3,4 };
#define ncrates (sizeof(crates)/sizeof(*crates))
  int malu_reg[ncrates][2];
  int sob_event_gate_reg;
  int lp_reg[ncrates][16];

#define N_TIME_SLICES 30
  int time_slice[ncrates][N_TIME_SLICES];
  for(i=0;i<ncrates;i++)for(j=0;j<N_TIME_SLICES;j++)time_slice[i][j]=0;

  /* Initialise crates */
  for(i=0;i<ncrates;i++)init_camac(crates[i],N23);

  /* Register addresses */
  for(i=0;i<ncrates;i++)                              /* All addresses in LP */
    for(j=0;j<16;j++)c_cdreg(&lp_reg[i][j], branch, crates[i], N23, j);
  for(i=0;i<ncrates;i++)                                /* A0 and A1 in MALU */
    for(j=0;j<2;j++)c_cdreg(&malu_reg[i][j], branch, crates[i], N21, j);
  c_cdreg(&sob_event_gate_reg, branch, 3, N22, A1);            /* A1 in SOB3 */

  for(i=0;i<ncrates;i++)
    {
      int value=1;
      int Q;

      /* Disable List Processors */
      c_cssa(F24,lp_reg[i][0],&value,&Q,0,1);

      /* Reset LP - Booking/LSR/Trigger Enable/Go Flag etc.. */
      c_cssa(F25,lp_reg[i][15],&value,&Q,0,1);

      /* Book LP */
      c_cssa(F27,lp_reg[i][0],&value,&Q,0,1);
      if(Q==0)
	{
	  fprintf(stderr,"Could not book LP %d",crates[i]);
	  exit(EXIT_FAILURE);
	}

      /* Write Instruction Pointer - set to zero */
      value=0;
      c_cfsa(F17,lp_reg[i][0],&value,&Q,0,1);

      /* Write the program */
      value=lp_jmp(7);
      c_cfsa(F16,lp_reg[i][0],&value,&Q,0,1);

      for(j=1;j<7;j++)
	{
	  value=lp_jmpawt(0);
	  c_cfsa(F16,lp_reg[i][0],&value,&Q,0,1);
	}

      for(j=0;j<N_TIME_SLICES;j++)
	{
	  value=lp_naf(F0,N21,A0,Q_REQUIRE);
	  c_cfsa(F16,lp_reg[i][0],&value,&Q,0,1);
	}

      value=lp_jmpawt(0);
      c_cfsa(F16,lp_reg[i][0],&value,&Q,0,1);

      /* Write Instruction Pointer - set to zero */
      value=0;
      c_cfsa(F17,lp_reg[i][0],&value,&Q,0,1);

      /* Write Write-Data Pointer - set to zero */
      value=0;
      c_cfsa(F17,lp_reg[i][1],&value,&Q,0,1);

      /* Write Read-Data Pointer - set to zero */
      value=0;
      c_cfsa(F17,lp_reg[i][2],&value,&Q,0,1);

      /* Enable Triggers */
      value=1;
      c_cfsa(F26,lp_reg[i][0],&value,&Q,0,1);

      /* Clear MALU */
      c_cssa(F9,&malu_reg[i][0],&value,&Q,0,1);
    }

  fp=fopen("lptestmalu2.dat","w");
  if(fp==NULL)
    {
      fprintf(stderr,"Could not open lptestmalu2.dat\n");
      perror("Error is");
      exit(EXIT_FAILURE);
    }

  for(i=0;i<10000;i++)
    {
      int multi_value[N_TIME_SLICES];
      int value;
      int Q;

      fprintf(stderr,"Iteration %-5d : ",i+1);

      /* Open the event gate to let one event through */
      value=1;
      c_cssa(F25,sob_event_gate_reg,&value,&Q,0,1);

      fprintf(stderr,"Final MALUs : ");

      for(j=0;j<ncrates;j++)
	{
	  int k;
	  int read_ptr;
	  int write_ptr;

	  int counts[2];

	  int cb[4] = { N_TIME_SLICES, 0, 0, 0 };
	
	  Q=0;
	  while(Q==0)
	    {
	      fprintf(stderr,".");
	      /* Test the LAM on the MALU */
	      c_cfsa(F8,malu_reg[j][0],&value /*dummy*/,&Q, 0, 1);
	    }

	  /* Read MALU channels 1-16 */
	  c_cfsa(F2,malu_reg[j][0],&counts[0],&Q,0,1);

	  /* Read MALU channels 17-32 */
	  c_cfsa(F2,malu_reg[j][1],&counts[1],&Q,0,1);

	  fprintf(stderr,"  %d",((counts[1]&0xFFFF)<<16)+counts[0]&0xFFFF);

	  /* Transfer the data */
	  c_cfubr(F0,lp_reg[j][1],multi_value,cb,0,1);

	  //	  fprintf(stderr, "%d %d %d %d\n",cb[0],cb[1],cb[2],cb[3]);

	  for(k=0; k<N_TIME_SLICES; k++)
	    {
	      if(multi_value[k]==counts[0])
		{ 
		  time_slice[j][k]++; 
		  break;
		}
	    }
	}
      fprintf(stderr,"\n");      
    }

  for(i=0;i<ncrates;i++)
    {
      for(j=0;j<N_TIME_SLICES-1;j++)
	fprintf(fp,"%d ",time_slice[i][j]);
      fprintf(fp,"%d\n",time_slice[i][j]);
    }
  
  fclose(fp);
}
