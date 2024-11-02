#define NO_SCALERS

/* cfds.c
   JQ (980130)
   Program to set and read cfd settings and to take bias curve.
*/

#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<time.h>
#include	<math.h>
#include        <ctype.h>
#include	<unixio.h>
#include	<file.h>
#include        <descrip.h>   

/* Set up CAMAC function codes (F) and subadresses (A) */
enum F { F0,  F1,  F2,  F3,  F4,  F5,  F6,  F7,  F8,  F9, F10,
        F11, F12, F13, F14, F15, F16, F17, F18, F19, F20, F21,
        F22, F23, F24, F25, F26, F27, F28, F29, F30, F31};

enum A {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10,
       A11, A12, A13, A14, A15};
/* End function codes and subadresses */


int Branch=0;   /* dummy argument for C_CDREG subroutine call */
int Q_resp; 
int Dumint;     /* dummy float for calls to c_cfsa */ 
int Wide=0;    /* print results in "wide" mode */ 

#ifndef MAX_CHAN
#define MAX_CHAN 336
#endif

typedef char    Boolean_op ;
#ifndef SUCCESS
#define SUCCESS    1
#define FAILURE    0
#endif

#ifndef DC
#define DC         0           /* used to indicate whether CFD thresholds */
#define MV         1           /* are in d.c. or mV.                      */
#endif
int  Thresh_mode=MV;
char Thresh_scale[80];

#ifndef MASK_ENCODE_ON         /* used by cfd_mask_encode() to determine */
#define MASK_ENCODE_ON   0     /* whether to mask off tubes (...OFF) or  */ 
#define MASK_ENCODE_OFF  1     /* to turn them back on (...ON).          */
#endif  

#define CCONTROL	24

/*------------------------   CFD information   ------------------------*/
#define CFD_CRATE	6
#define NCFD		21
#define CFD1		1   
#define N_TRIG          10    /* Number of times to trigger CFDs via CAMAC */

int Cfd_crate_online;
int Extcfd[16*NCFD];           /* 16 channels X 21 modules */
int Cfd_dc_thresh[16*NCFD];
int Cfd_mv_thresh[16*NCFD];
int Cfd_width_dead[NCFD];
int Cfd_mask[NCFD];
int Cfd_online[NCFD]={1,1,1,1,1,   /* 1 means on-line, 0 off-line */
		      1,1,1,1,1,
		      1,1,1,1,1,
		      1,1,1,1,1,
		      1};

#define DEFAULT_MV_THRESH     90
#define DEFAULT_WIDTH_DEAD  0x05
#define DEFAULT_MASK           0



/*-----------------------  Old 4413 discriminators  ----------------------*/
#define CRATE_4413 4
#define N_4413     6
int Ext4413[N_4413];           
int L4413_online[N_4413] = {1,1,1,1,1,1};
int L4413_slot[N_4413] = {1,2,3,4,5,6};

/*-------------- SCALER used for co-incidence bias curve --------------*/
#define COINC_SCAL_CRATE     4
#define COINC_SCAL_SLOT     19
#define COINC_SCAL_CHAN      8
#define COINC_SCAL_CHAN2    10
int Ext_coinc_scal;                    
int Ext_coinc_scal2;  
int Coinc_scal_online=1;

#ifndef NO_SCALERS
/*-------------------------  SCALER information   ---------------------*/
#define SCAL_CRATE           5    /* scalers are in crate 5 */

int Scal_crate_online=0;

/* LeCroy scaler stuff */
#define N_LC_SCAL      3          /* 4 no. of LeCroy scalers */
#define LC_SCAL_SLOT1  1          /* first scaler slot */
#define LC_CHAN1       1          /* channels no. of first LeCroy scaler */
int Lc_scal_cdreg[N_LC_SCAL];     /* only need one per unit */
int Lc_scal_val[2*16*N_LC_SCAL];  /* 32 channels per unit */
int Lc_scal_online[N_LC_SCAL] = {1,1,1}; 
int Lc_scal_slot[N_LC_SCAL]=      /* scaler positions in crates */
               { 1, 2, 3};

  

/* Phillips scaler stuff */
#define N_PHIL_SCAL      8        /* 7 no. of Phillips scalers */
#define PHIL_SCAL_SLOT1  4        /* first scaler slot */
#define PHIL_CHAN1       97      /* channels no. of first Phillips scaler */
int Phil_scal_cdreg[16*N_PHIL_SCAL];   /* 16 subadresses per unit */
int Phil_scal_val[2*16*N_PHIL_SCAL];   /* 32 channels per unit */
int Phil_scal_online[N_PHIL_SCAL] = {1,1,1,1,1,1,1,1};  
int Phil_scal_slot[N_PHIL_SCAL]=     /* scaler positions in crates */
        { 4, 5, 6, 7, 8, 9, 10, 11};
 
#define MAX_SCALER_CHAN  352
#endif // NO_SCALERS

/* SOB */
#define SOB_SLOT 22
int Sob_cdreg;

/* 1-fold */

#define N_1FOLD 50
#define N_TIME 20


/*------------------------  Function prototypes  -------------------------*/

/* HYTEC Crate Controller routines */
void            c_cdreg();
void            ccfini();
unsigned int    c_cccc();
unsigned int    c_cccz();
unsigned int    c_ccci();
unsigned long   c_cssa();
unsigned long   c_cfsa();
unsigned long   initusr();

/* User defined functions */
Boolean_op      tst_status();
void            LIB$SIGNAL();
void            c_wait_time(float);
void            init_cfds();
void            usage();
int             init_camac(int);
void            cfd_get_maskchannels(); 
void            cfd_get_maskpixels(); 
void            cfd_mask_encode(int , int *, int);
void            cfd_get_modmask(int );
void            cfd_write_mask(int );
void            cfd_mask_non_verbose(int );
void            cfd_get_width_dead(unsigned int *);
void            cfd_write_width_dead(int, unsigned int);
void            cfd_get_module(int *);
void            cfd_get_modchannel(int *);
void            cfd_get_channel(int *, int *); 
void            cfd_get_threshold(int *);
void            cfd_write_threshold(int, int, int);
void            cfd_read_threshold(int, int);
void            cfd_read_width_dead(int);
void            cfd_read_masks();
void            cfd_get_thresh_mode();
void            cfd_get_thresh_mode();
void            do_coinc_bias(int);
#ifndef NO_SCALERS
void            do_chan_bias();
#endif
void            do_both_bias();
void            do_cfd_calibration();
void            trigger_cfds(int);
void            test_1fold();
void            single_chan_rate();
int             cfd_mv2dc(int *, int *);              /* from gpr10_c.c */
int             cfd_dc2mv(int *, int *);              /* from gpr10_c.c */


/*---------------------  end function prototypes --------------------------*/



int main(int argc, char **argv)
{
  
  int i;
  int j;
  int option;
  unsigned int width_dead=0x55;
  int mask=0;
  int module=-1;
  int modchan=-1;                          /* channel within module (0-15) */
  int channel;                             /* total channel no. (1-336)    */
  int threshold=DEFAULT_MV_THRESH;
  char line[80];
  char coption[80];


  /*--------------------   Initialize CAMAC  ----------------------------*/
  printf("Initialising CAMAC to crate %d ... ",CFD_CRATE);
  if ((Cfd_crate_online=init_camac(CFD_CRATE)))
    printf("done\n");
  else{
    printf("Error, exiting....\n");
    exit(FAILURE);
  }

#ifndef NO_SCALERS  
  printf("Initialising CAMAC to crate %d ... ",SCAL_CRATE);

  if ((Scal_crate_online=init_camac(SCAL_CRATE)))
    printf("done\n");
  else{
    printf("Error initialising connection to Scaler crate, ");
    printf("continuing anyway.\n");
  }
#endif

  printf("Initialising CAMAC to crate %d ... ",COINC_SCAL_CRATE);
  if ((Coinc_scal_online=init_camac(COINC_SCAL_CRATE)))
    printf("done\n");
  else{
    printf("Error initialising connection to Coinc. scaler crate, ");
    printf("continuing anyway.\n");
  }
  
  
  
  /*-----------------  Set up external register coding  ----------------*/
  /* CFDs: */
  for(j=0;j<NCFD;j++)     
    for(i=0;i<16;i++) 
      c_cdreg(&(Extcfd[j*16+i]),Branch,CFD_CRATE,CFD1+j,i) ;

  /* 4413 Discriminators: */
  for(j=0;j<N_4413;j++)
    if (L4413_online[j])
      c_cdreg(&(Ext4413[j]),Branch,CRATE_4413,j+1,A0) ;
  

  /* Coincidence Scaler: */
  if (Coinc_scal_online){
    c_cdreg(&Ext_coinc_scal,Branch,COINC_SCAL_CRATE,COINC_SCAL_SLOT,
            COINC_SCAL_CHAN);
    c_cdreg(&Ext_coinc_scal2,Branch,COINC_SCAL_CRATE,COINC_SCAL_SLOT,
	    COINC_SCAL_CHAN2);
  }

#ifndef NO_SCALERS
  if(Scal_crate_online){
    /* LeCroy Scalers */
    for(i=0;i<N_LC_SCAL;i++)
      c_cdreg(&Lc_scal_cdreg[i],Branch,SCAL_CRATE,
	      Lc_scal_slot[i],A0);
    /* Phillips Scalers */
    for(i=0;i<N_PHIL_SCAL;i++)
      for(j=0;j<16;j++)
	c_cdreg(&Phil_scal_cdreg[i*16+j],Branch,SCAL_CRATE,
		Phil_scal_slot[i],j);

  /*----- initialilse external register codes for SOB, A2 out ------*/
    printf("Setting up external register coding for SOB....");
    c_cdreg(&Sob_cdreg,Branch,SCAL_CRATE,SOB_SLOT,A2);
    printf("done\n");
  }
#endif


  /*-----------------------  Set CFD masks to 0 -------------------------*/ 
  for (i=0;i<NCFD;i++)
    Cfd_mask[i]=0;

  /*--------------------------   Main Loop  -----------------------------*/ 
  for(;;){
    printf("\nCFD Options:\n");
    printf("1.  Set threshold (by module/modchan)\n");
    printf("2.  Set threshold (by channel)\n");
    printf("3.  Set width/deadtime\n");
    printf("4.  Set mask (by module/modchan)\n");
    printf("5.  Mask off (more) channels (by channel)\n");
    printf("6.  Restore (some) masked off channels\n");
    printf("7.  Restore all masked off channels\n");
    printf("8.  Read thresholds\n");
    printf("9.  Read width/deadtime\n");
    printf("10. Read masks\n");
    printf("11. Initialize all CFDs to default values\n");
    printf("12. Take bias curve (co-incidence)\n");
#ifndef NO_SCALERS
    printf("13. Take bias curve (channels)\n");
#endif
    printf("14. Take bias curve (CFDs and 4413s)\n");
    printf("15. Trigger CFDs %d times\n",N_TRIG);
    printf("16. Check for 1-fold sensitivity\n");
    printf("17. Measure 1-fold rates with multiplicity scaler\n");
    printf("18. PST bias curve with LP ( < kHz rate)\n");
    printf("19. Do CFD calibration using 1/2 scaler\n");
    printf("Q.  Quit\n\n");
    /*    printf("W.  Wide display format\n\n"); */
    printf("Please enter option: ");

    fgets(line,80,stdin);
    sscanf(line,"%s",coption);
    if((strcmp(coption,"Q")==0) || (strcmp(coption,"q")==0))
      break;          
    else if((strcmp(coption,"W")==0) || (strcmp(coption,"w")==0)){
      printf("Wide\n");
      Wide=1;
    }
    else if (sscanf(line,"%d",&option)==1){
      switch(option){
      case 1:
	/*Set threshold (by module/modchan) */
	printf("Setting thresholds...\n");
	cfd_get_thresh_mode();
	cfd_get_module(&module);
	cfd_get_modchannel(&modchan);
	cfd_get_threshold(&threshold);
	cfd_write_threshold(module,modchan,threshold);
	break;
      case 2:
	/* Set threshold (by channel) */
	printf("Setting thresholds...\n");
	cfd_get_thresh_mode();
	cfd_get_channel(&module,&modchan);
	cfd_get_threshold(&threshold);
	cfd_write_threshold(module,modchan,threshold);
	break;
      case 3:
	/* Set width/deadtime */
	printf("Setting width/deadtime...\n");
	cfd_get_module(&module);
	cfd_get_width_dead(&width_dead);              
	cfd_write_width_dead(module,width_dead);  
	break;
      case 4:
	/* Set mask (by module/channel) */
	printf("Setting masks...\n");
	cfd_get_module(&module);
	cfd_get_modmask(module);
	cfd_write_mask(module);       /* just started cfd_get_masks */
	break;
      case 5:
	/* Mask off channels (by channel number) */
	printf("Masking off (more) channels...\n");
	cfd_get_maskchannels(MASK_ENCODE_OFF);
	cfd_write_mask(-1);
	break;
      case 6:
	/* Restore (some) masked off channels */
	printf("Restoring (some) masked channels...\n");
	cfd_get_maskchannels(MASK_ENCODE_ON);       
	cfd_write_mask(-1);
	break;
      case 7:
        /* Restore all masked off channels */
	printf("Un-masking all channels...\n");
        for (i=0;i<NCFD;i++)
          Cfd_mask[i]=0;
        cfd_write_mask(-1);
        break;
      case 8:
	/* read thresholds */
	printf("Reading thresholds...\n");
	cfd_get_module(&module);
	cfd_get_modchannel(&modchan);
	cfd_read_threshold(module,modchan);
	break;
      case 9:
	/* read width/deadtime */
	printf("Reading width/deadtime...\n");
	cfd_get_module(&module);
	cfd_read_width_dead(module);
	break;
      case 10:
	printf("Reading masks...\n");
	cfd_read_masks();
	break;
      case 11:
	/* initialise CFDs to default values */
	printf("Initialising CFDs to default values...\n");
	cfd_write_threshold(-1,-1,DEFAULT_MV_THRESH);
	cfd_write_width_dead(-1,DEFAULT_WIDTH_DEAD);  
        for (i=0;i<NCFD;i++)
          Cfd_mask[i]=DEFAULT_MASK;
        cfd_write_mask(-1);
	break;
      case 12:
	printf("Taking co-incidence bias curve...\n");
	do_coinc_bias(1);
	break;
#ifndef NO_SCALERS
      case 13:
	printf("Taking channel-by-channel bias curve...\n");
	do_chan_bias();
	break;
#endif
      case 14:
	printf("Taking bias curve with CFDs and 4413s...\n");
	do_both_bias();
	break;
      case 15:
	printf("Triggering CFDs....");
	trigger_cfds(N_TRIG);
        printf(" done\n");
	break;
      case 16:
	test_1fold();
	break;  
      case 17:
        single_chan_rate();
        break;  	
      case 18:
        printf(" \n");
        printf("To take PST bias curve without LPs use program 'PST_BIAS'\n");
        printf("otherwise, now start Granite. Start a run. Press 'Panic'\n");
        printf("the LPs will keep running but won't be read out'\n");
	do_coinc_bias(2);
	break;
      case 19:
	puts("OK then, what we are going to do is use one scalar (or");
	puts("actually half of one scalar) which has 16 functioning channels");
	puts("to calibrate the cfds. I want you to frad in a signal into all");
	puts("16 channels of the cfd at the same time (a linear fan-out is");
	puts("useful, there are loads of them on the top shelf in the wooden");
	puts("cabinet -- or at least there were loads of them there in 1999");
	puts(":-). make note of the peak voltage on each channel and i'll");
	puts("step through the cfd settings and tell you what one (in");
	puts("digital counts) corresponds to that voltage");
	do_cfd_calibration();
	break;
      default:
	printf("Unknown option %d, please try again.\n",option);
	break;
      }
    }
  }
      

  ccfini();          /* free resources */ 

  return EXIT_SUCCESS;
}

#ifndef NO_SCALERS
typedef enum { LCROY_SC, PHIL_SC } ScalerType;

void read_scaler(ScalerType scaler_type,int *cdreg,int data[32])
{
  const int dt=1; /* time to wait -- in seconds */
  unsigned int controller_ext;
  int inhibit;
  int wdata;
  int chan;

  c_cdreg(&controller_ext,Branch,SCAL_CRATE,CCONTROL,0);

  /* inhibit crate */
  inhibit=1;
  c_ccci(controller_ext,inhibit,0,1);

  /* Clear the scaler */
  switch(scaler_type)
    {
    case LCROY_SC:
      /* first clear LeCroy scaler */
      wdata=64;
      c_cfsa(F16,*(cdreg+A0),&wdata,&Q_resp,0,1); 

      /* problem with crate 5. Camac clear does not work any more */
      /* for LeCroy scalers. So, now try SOB clear.               */
      wdata=0; 
      c_cfsa(F25,Sob_cdreg,&wdata,&Q_resp,0,1); 
      break;
    case PHIL_SC:
      c_cfsa(F11,*(cdreg+A4),&Dumint,&Q_resp,0,1);
      printf("clear phillips scaler  F11 A4 Q-resp: %d \n",Q_resp);
	/* i.e. F11, A4 */
      break;
    }
  
  inhibit=0;
  c_ccci(controller_ext,inhibit,0,1);
  
  /* wait time */
  c_wait_time(dt);
  
  /*--------------  Read out scaler ----------------*/

  /* inhibit crate */
  inhibit=1;
  c_ccci(controller_ext,inhibit,0,1);

  switch(scaler_type)
    {
    case LCROY_SC:
      /* load LeCroy scaler contents into internal buffer and start readout */
      wdata=7968;
      c_cfsa(F16,*(cdreg+A0),&wdata,&Q_resp,0,1);
      /* i.e. F16, A0 */
      
      /* Read LeCroy scalers using F(2)A(0) */

      /* The first address (FA) and number to read (NR) have already */
      /* been selected with the F16,A0,W=7968 above */

      for(chan=0;chan<32;chan++){
	c_cfsa(F2,*(cdreg+A0),&data[chan],&Q_resp,0,1);
        /* printf("%d %d %d %d\n",i,j,lc_scal_val[i*32+j],q_resp); */ 
        /* i.e. F2, A0 */
      }
      break;
      
    case PHIL_SC:
      /* First reset the scaler bank selection register */
      c_cfsa(F11,*(cdreg+A1),&Dumint,&Q_resp,0,1);
      /* i.e. F11, A1 */
      
      /* Next loop over all channels in unit and store scaler value */
      for(chan=0;chan<32;chan++){
	c_cfsa(F4,*(cdreg+A15),&data[chan],&Q_resp,0,1);
	printf("%d %d %d\n",chan,data[chan],Q_resp);
      /* i.e. F4, A15 */
      }
      break;
    }

  /* uninhibit crate */
  inhibit=0;
  c_ccci(controller_ext,inhibit,0,1);
  
  return;
}

void do_cfd_calibration()
{
  char line[80];
  
  int scaler;
  int *scaler_cdreg;
  ScalerType scaler_type;
  int scaler_channel_start=0;
  
  const int untriggerable_threshold_dc=200;
  const int fireforsure_threshold_dc=3;
  const int firstguess_threshold_dc=40;

  const int crossed_channel_counts=40;

  while(1)
    {
      int i;

      fputs("What scalar do you want to use (counting from 1): ",stdout);  
      fflush(stdout);
      fgets(line,80,stdin);
      if(sscanf(line,"%d",&scaler)!=1)
	{
	  printf("Please enter a scaler number, i don't understand: %s",line);
	  continue;
	}

      /* is it a LeCroy scaler ? */
      for(i=0;i<N_LC_SCAL;i++)if(scaler==Lc_scal_slot[i])break;
      if(i<N_LC_SCAL)
	{
	  puts("Its a LeCroy scaler... oh well");
	  if(!Lc_scal_online[i])
	    {
	      puts("....but its not online, we can't use it");
	      continue;
	    }
	  scaler_type=LCROY_SC;
	  scaler_cdreg=Lc_scal_cdreg+i;
	  break;
	}

      /* perhaps its a Phillips one then */
      for(i=0;i<N_PHIL_SCAL;i++)if(scaler==Phil_scal_slot[i])break;
      if(i<N_PHIL_SCAL)
	{
	  puts("Its a Phillips scaler... yippee");
	  if(!Phil_scal_online[i])
	    {
	      puts("....but its not online, we can't use it");
	      continue;
	    }
	  scaler_type=PHIL_SC;
	  scaler_cdreg=Phil_scal_cdreg+i*16;
	  break;
	}
      
      /* its not a valid scaler then i suppose */
      puts("I don't have a record of the scaler you asked for");
    }
  
  while(1)
    {
      fputs("Do you want to use the (L)ower or "
	    "(U)pper channels of the scaler:",stdout);
      fflush(stdout);
      if(fgets(line,80,stdin)==0)continue;
      
      if(tolower(*line)=='u')
	{
	  scaler_channel_start=16;
	  break;
	}
      else if(tolower(*line)=='l')
	{
	  scaler_channel_start=0;;
	  break;
	}
    }

  Thresh_mode=DC;
  strcpy(Thresh_scale,"d.c.");

  /* ensure we have stopped triggering on all channels */
  cfd_write_threshold(-1,-1,untriggerable_threshold_dc);

  /*::::::::::::::::::::::::::: main loop ::::::::::::::::::::::::::*/
  while(1)
    {
      int module;
      int modchan;
      int counts[2][32];
      static int threshold_dc=40 /*firstguess_threshold_dc*/;
      int levels_dc[16];
      int ncounts[16];
      int bcounts[16];
      int crosstalk[16][16];
      int crosstalk_tries;

      static char filename[80]="none";

      FILE* outfp;

      fputs("What CFD would you like to calibrate (-1 to end): ",stdout);
      fflush(stdout);
      
      fgets(line,80,stdin);
      if(sscanf(line,"%d",&module)!=1)
	{
	  printf("Please enter a module number, i don't understand: %s",line);
	  continue;
	}

      if(module==-1)break;

      if((module-CFD1)>=NCFD)
	{
	  printf("CFD module must be between %d and %d inclusive.",
		 CFD1,NCFD);
	  continue;
	}

      modchan=-1;
      cfd_write_threshold(module,-1,untriggerable_threshold_dc);
      c_wait_time(1);
      
      for(modchan=0;modchan<16;modchan++)
	{
	  int channel;
	  int nominal_count=0;
	  int boundary_count=0;
	  enum { false, true } was_triggering;

	  for(channel=0;channel<16;channel++)	  
	    crosstalk[modchan][channel]=0;
	  levels_dc[modchan]=-1;

	  if(threshold_dc==-1)threshold_dc=40;

#define CTRIES 3
#define SCS scaler_channel_start

	  printf(
"----------------------------------- Channel %d ---------------------------\n",
modchan+1);
	  puts("  Testing for crosstalk and calculating nominal counts..");
	  cfd_write_threshold(module,modchan,fireforsure_threshold_dc);
	  for(crosstalk_tries=0;crosstalk_tries<CTRIES;crosstalk_tries++)
	    {
	      read_scaler(scaler_type,scaler_cdreg,&counts[0][0]);
	      for(channel=0;channel<16;channel++)
		printf("%3d%c",counts[0][channel+SCS],channel==15?'\n':' ');
	      for(channel=0;channel<16;channel++)
		if(counts[0][channel+SCS]>crossed_channel_counts)
		  crosstalk[modchan][channel]++;
	      nominal_count+=counts[0][modchan+SCS];
	    }
	  ncounts[modchan]=nominal_count/=CTRIES;
	  bcounts[modchan]=boundary_count=nominal_count/100;	  
	  printf("Nominal counts: %d, boundary set at: %d\n",nominal_count,
		 boundary_count);

	  if(nominal_count<crossed_channel_counts)
	    {
	      puts(" Very few counts .. channel probably dead !");
	      cfd_write_threshold(module,modchan,untriggerable_threshold_dc);
	      continue;
	    }

	  /* Here we go then -- see if we trigger then either go up or
	     down until that changes */
	  cfd_write_threshold(module,modchan,threshold_dc);

	  /* This loop takes two sets of counts and if they are both
	     either over the boundary count or both under it it proceeds.
	     this is to counter the times you get noise from the scalers */
	  while(1)
	    {
	      read_scaler(scaler_type,scaler_cdreg,&counts[0][0]);
	      for(channel=0;channel<16;channel++)
		printf("%3d%c",counts[0][channel+SCS],channel==15?'\n':' ');
	      for(channel=0;channel<16;channel++)
		if(counts[0][channel+SCS]>crossed_channel_counts)
		  crosstalk[modchan][channel]++;

	      read_scaler(scaler_type,scaler_cdreg,&counts[1][0]);
	      for(channel=0;channel<16;channel++)
		printf("%3d%c",counts[1][channel+SCS],channel==15?'\n':' ');
	      for(channel=0;channel<16;channel++)
		if(counts[1][channel+SCS]>crossed_channel_counts)
		  crosstalk[modchan][channel]++;
	      crosstalk_tries+=2;
	      if(!( (counts[0][modchan+SCS]>boundary_count) ^ /* XOR */
		    (counts[1][modchan+SCS]>boundary_count)) )break;
	    }
	  
	  if(counts[0][modchan+SCS]>boundary_count)was_triggering=true;
	  else was_triggering=false;

	  /* loop until we trigger, or we don't depeding on whether we
	     are weren't or were ... if you see what i mean */

	  while(1)
	    {
	      if(was_triggering == true)threshold_dc++;
	      else threshold_dc--;

	      if((threshold_dc<fireforsure_threshold_dc) ||
		 (threshold_dc>=untriggerable_threshold_dc))
		{
		  puts("Threshold out of range.... giving up!");
		  threshold_dc=-1;
		  break;
		}

	      cfd_write_threshold(module,modchan,threshold_dc);
	      
	      /* This loop takes two sets of counts and if they are both
		 either over the boundary count or both under it it proceeds.
		 this is to counter the times you get noise from the scalers */
	      while(1)
		{
		  read_scaler(scaler_type,scaler_cdreg,&counts[0][0]);
		  for(channel=0;channel<16;channel++)
		    printf("%3d%c",counts[0][channel+SCS],
			   channel==15?'\n':' ');
		  for(channel=0;channel<16;channel++)
		    if(counts[0][channel+SCS]>crossed_channel_counts)
		      crosstalk[modchan][channel]++;
		  read_scaler(scaler_type,scaler_cdreg,&counts[1][0]);
		  for(channel=0;channel<16;channel++)
		    printf("%3d%c",counts[1][channel+SCS],
			   channel==15?'\n':' ');
		  for(channel=0;channel<16;channel++)
		    if(counts[1][channel+SCS]>crossed_channel_counts)
		      crosstalk[modchan][channel]++;
		  crosstalk_tries+=2;
		  if(!( (counts[0][modchan+SCS]>boundary_count) ^ /* XOR */
			(counts[1][modchan+SCS]>boundary_count)) )break;
		}
	      
	      if(( (counts[0][modchan+SCS]>boundary_count) && 
		   (was_triggering==false) ) ||
		 ( (counts[0][modchan+SCS]<=boundary_count) && 
		   (was_triggering==true) ) )break;
	    }

	  
	  if((threshold_dc!=-1)&&(was_triggering == false))threshold_dc++;
	  
	  levels_dc[modchan]=threshold_dc;
	
	  /* turn off the triggering channel by setting the treshold high */
	  cfd_write_threshold(module,modchan,untriggerable_threshold_dc);
	}

      for(modchan=0;modchan<16;modchan++)
	{
	  int crossed_triggers;
	  int n_crosstalking=0;
	  int channel;
	  printf("Channel %2d, Threshold: %d dc (>%d counts)\n",
		 modchan+1,levels_dc[modchan],bcounts[modchan]);
	  crossed_triggers=crosstalk[modchan][modchan]/2;
	  if(ncounts[modchan]<crossed_channel_counts)
	    {
	      printf(" Nominal counts %d < %d -- CHANNEL PROBABLY DEAD\n",
		     ncounts[modchan],crossed_channel_counts);
	    }
	  else
	    {
	      for(channel=0;channel<16;channel++)
		if((modchan!=channel)&&
		   (crosstalk[modchan][channel]>=crossed_triggers))
		  {
		    if(n_crosstalking++ == 0)printf(" Cross-talk: ");
		    printf("%d ",channel+1);
		  }
	      if(n_crosstalking)puts("");
	    }
	}

      while(1)
	{
	  fprintf(stdout, "Enter output filename (or \"none\") [%s]: ",
		  filename);
	  fflush(stdout);	  
	  fgets(line,80,stdin);

	  line[79]='\0';
	  line[strlen(line)-1]='\0';

	  outfp=NULL;

	  /* If user enters nothing then use old file name as default */
	  if(strcmp(line,"")==0)strcpy(line,filename);
	  if(strcmp(line,"none")==0)break;
	  
	  outfp=fopen(line,"a");
	  if(outfp == NULL)
	    {
	      fprintf(stdout,"\007... Could not open file \"%s\"\n",line);
	      continue;
	    }

	  strcpy(filename,line);
	  break;
	}

      if(outfp==NULL)continue;

      for(modchan=0;modchan<16;modchan++)
	fprintf(outfp,"%3d ",levels_dc[modchan]);

      fprintf(outfp," %% CFD#%-2d",module);

      for(modchan=0;modchan<16;modchan++)
	{
	  int crossed_triggers;
	  int n_crosstalking=0;
	  int channel;

	  crossed_triggers=crosstalk[modchan][modchan]/2;
	  if(ncounts[modchan]<crossed_channel_counts)
	    {
	      fprintf(outfp," ; %d DEAD",modchan+1);
	    }
	  else
	    {
	      for(channel=0;channel<16;channel++)
		if((modchan!=channel)&&
		   (crosstalk[modchan][channel]>=crossed_triggers))
		  {
		    if(n_crosstalking++ == 0)
		      fprintf(outfp," ; %d CROSS",modchan+1);
		    fprintf(outfp,"%c%d",
			    ((n_crosstalking==1)?' ':','),channel+1);
		  }
	    }
	}
      
      fprintf(outfp,"\n");

      fclose(outfp);
    }
  
  return;
}
#endif // NO_SCALERS


void read_scaler(int *reg,int controller_reg, int upper, int data[32])
{
  const int dt=1; /* time to wait -- in seconds */

  int i;
  int value;
  int Q;

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
  
  c_wait_time(dt);
  
  /* Inhibit Crate */
  value=1;
  c_ccci(controller_reg,value /* inhibit */,0,1);
  
  /* First reset the scaler bank selection register */      
  c_cfsa(F11,reg[A1],&value /* dummy */,&Q,0,1); /* i.e. F11, A1 */
  
  if(upper){ value=1; c_cfsa(F17,reg[A1],&value,&Q,0,1); }
  else { value=0; c_cfsa(F17,reg[A1],&value,&Q,0,1); }

  /* Next loop over all channels in unit and store scaler value */
  for(i=0;i<16;i++)
    c_cfsa(F0,reg[i],&data[i],&Q,0,1); 

  /* Un-Inhibit Crate */
  value=0;
  c_ccci(controller_reg,value /* inhibit */,0,1);
  
  return;
}


void do_cfd_calibration()
{
  char line[80];
  
  int scaler_channel_start=0;
  
  const int untriggerable_threshold_dc=200;
  const int fireforsure_threshold_dc=3;
  const int firstguess_threshold_dc=40;

  const int crossed_channel_counts=40;

  int crate;
  int slot;
  int branch=0;

  int i;
  int reg[16];
  int controller_reg;

  while(1)
    {
      fprintf(stdout, "Enter CRATE number which has PHILIPS scaler: ");
      fflush(stdout);
      fgets(line,80,stdin);
      if(sscanf(line,"%d",&crate) == 1)
	{
	  if(init_camac(crate))break;
	  fprintf(stdout,"Could not initialise crate %d\n",crate);
	}
    }
  
  c_cdreg(&controller_reg, branch, crate, 24, 0);
  
  while(1)
    {
      fprintf(stdout, "Enter SLOT number containing PHILIPS scaler: ");
      fgets(line,80,stdin);
      if(sscanf(line,"%d",&slot) == 1)
	if((slot>=1)&&(slot<=24))break;
	  fprintf(stdout,"Invalid slot number %d\n",slot);
    }

  for(i=0; i<16; i++)c_cdreg(&reg[i], branch, crate, slot, i);
  
  while(1)
    {
      fputs("Do you want to use the (L)ower 1-16 or "
	    "(U)pper 17-32 channels of the scaler:",stdout);
      fflush(stdout);
      if(fgets(line,80,stdin)==0)continue;
      
      if(tolower(*line)=='u')
	{
	  scaler_channel_start=1;
	  break;
	}
      else if(tolower(*line)=='l')
	{
	  scaler_channel_start=0;;
	  break;
	}
    }

  Thresh_mode=DC;
  strcpy(Thresh_scale,"d.c.");

  /* ensure we have stopped triggering on all channels */
  cfd_write_threshold(-1,-1,untriggerable_threshold_dc);

  /*::::::::::::::::::::::::::: main loop ::::::::::::::::::::::::::*/
  while(1)
    {
      int module;
      int modchan;
      int counts[2][32];
      static int threshold_dc=40 /*firstguess_threshold_dc*/;
      int levels_dc[16];
      int ncounts[16];
      int bcounts[16];
      int crosstalk[16][16];
      int crosstalk_tries;

      static char filename[80]="none";

      FILE* outfp;

      fputs("What CFD would you like to calibrate (-1 to end): ",stdout);
      fflush(stdout);
      
      fgets(line,80,stdin);
      if(sscanf(line,"%d",&module)!=1)
	{
	  printf("Please enter a module number, i don't understand: %s",line);
	  continue;
	}

      if(module==-1)break;

      if((module-CFD1)>=NCFD)
	{
	  printf("CFD module must be between %d and %d inclusive.",
		 CFD1,NCFD);
	  continue;
	}

      modchan=-1;
      cfd_write_threshold(module,-1,untriggerable_threshold_dc);
      c_wait_time(1);
      
      for(modchan=0;modchan<16;modchan++)
	{
	  int channel;
	  int nominal_count=0;
	  int boundary_count=0;
	  enum { false, true } was_triggering;

	  for(channel=0;channel<16;channel++)	  
	    crosstalk[modchan][channel]=0;
	  levels_dc[modchan]=-1;

	  if(threshold_dc==-1)threshold_dc=40;

#define CTRIES 3
#define SCS scaler_channel_start

	  printf(
"----------------------------------- Channel %d ---------------------------\n",
modchan+1);
	  puts("  Testing for crosstalk and calculating nominal counts..");
	  cfd_write_threshold(module,modchan,fireforsure_threshold_dc);
	  for(crosstalk_tries=0;crosstalk_tries<CTRIES;crosstalk_tries++)
	    {
	      read_scaler(reg,controller_reg,scaler_channel_start,
			  &counts[0][0]);
	      for(channel=0;channel<16;channel++)
		printf("%3d%c",counts[0][channel+SCS],channel==15?'\n':' ');
	      for(channel=0;channel<16;channel++)
		if(counts[0][channel+SCS]>crossed_channel_counts)
		  crosstalk[modchan][channel]++;
	      nominal_count+=counts[0][modchan+SCS];
	    }
	  ncounts[modchan]=nominal_count/=CTRIES;
	  bcounts[modchan]=boundary_count=nominal_count/100;	  
	  printf("Nominal counts: %d, boundary set at: %d\n",nominal_count,
		 boundary_count);

	  if(nominal_count<crossed_channel_counts)
	    {
	      puts(" Very few counts .. channel probably dead !");
	      cfd_write_threshold(module,modchan,untriggerable_threshold_dc);
	      continue;
	    }

	  /* Here we go then -- see if we trigger then either go up or
	     down until that changes */
	  cfd_write_threshold(module,modchan,threshold_dc);

	  /* This loop takes two sets of counts and if they are both
	     either over the boundary count or both under it it proceeds.
	     this is to counter the times you get noise from the scalers */
	  while(1)
	    {
	      read_scaler(reg,controller_reg,scaler_channel_start,
			  &counts[0][0]);
	      for(channel=0;channel<16;channel++)
		printf("%3d%c",counts[0][channel+SCS],channel==15?'\n':' ');
	      for(channel=0;channel<16;channel++)
		if(counts[0][channel+SCS]>crossed_channel_counts)
		  crosstalk[modchan][channel]++;

	      read_scaler(reg,controller_reg,scaler_channel_start,
			  &counts[1][0]);
	      for(channel=0;channel<16;channel++)
		printf("%3d%c",counts[1][channel+SCS],channel==15?'\n':' ');
	      for(channel=0;channel<16;channel++)
		if(counts[1][channel+SCS]>crossed_channel_counts)
		  crosstalk[modchan][channel]++;
	      crosstalk_tries+=2;
	      if(!( (counts[0][modchan+SCS]>boundary_count) ^ /* XOR */
		    (counts[1][modchan+SCS]>boundary_count)) )break;
	    }
	  
	  if(counts[0][modchan+SCS]>boundary_count)was_triggering=true;
	  else was_triggering=false;

	  /* loop until we trigger, or we don't depeding on whether we
	     are weren't or were ... if you see what i mean */

	  while(1)
	    {
	      if(was_triggering == true)threshold_dc++;
	      else threshold_dc--;

	      if((threshold_dc<fireforsure_threshold_dc) ||
		 (threshold_dc>=untriggerable_threshold_dc))
		{
		  puts("Threshold out of range.... giving up!");
		  threshold_dc=-1;
		  break;
		}

	      cfd_write_threshold(module,modchan,threshold_dc);
	      
	      /* This loop takes two sets of counts and if they are both
		 either over the boundary count or both under it it proceeds.
		 this is to counter the times you get noise from the scalers */
	      while(1)
		{
		  read_scaler(reg,controller_reg,scaler_channel_start,
			      &counts[0][0]);
		  for(channel=0;channel<16;channel++)
		    printf("%3d%c",counts[0][channel+SCS],
			   channel==15?'\n':' ');
		  for(channel=0;channel<16;channel++)
		    if(counts[0][channel+SCS]>crossed_channel_counts)
		      crosstalk[modchan][channel]++;
		  read_scaler(reg,controller_reg,scaler_channel_start,
			      &counts[1][0]);
		  for(channel=0;channel<16;channel++)
		    printf("%3d%c",counts[1][channel+SCS],
			   channel==15?'\n':' ');
		  for(channel=0;channel<16;channel++)
		    if(counts[1][channel+SCS]>crossed_channel_counts)
		      crosstalk[modchan][channel]++;
		  crosstalk_tries+=2;
		  if(!( (counts[0][modchan+SCS]>boundary_count) ^ /* XOR */
			(counts[1][modchan+SCS]>boundary_count)) )break;
		}
	      
	      if(( (counts[0][modchan+SCS]>boundary_count) && 
		   (was_triggering==false) ) ||
		 ( (counts[0][modchan+SCS]<=boundary_count) && 
		   (was_triggering==true) ) )break;
	    }

	  
	  if((threshold_dc!=-1)&&(was_triggering == false))threshold_dc++;
	  
	  levels_dc[modchan]=threshold_dc;
	
	  /* turn off the triggering channel by setting the treshold high */
	  cfd_write_threshold(module,modchan,untriggerable_threshold_dc);
	}

      for(modchan=0;modchan<16;modchan++)
	{
	  int crossed_triggers;
	  int n_crosstalking=0;
	  int channel;
	  printf("Channel %2d, Threshold: %d dc (>%d counts)\n",
		 modchan+1,levels_dc[modchan],bcounts[modchan]);
	  crossed_triggers=crosstalk[modchan][modchan]/2;
	  if(ncounts[modchan]<crossed_channel_counts)
	    {
	      printf(" Nominal counts %d < %d -- CHANNEL PROBABLY DEAD\n",
		     ncounts[modchan],crossed_channel_counts);
	    }
	  else
	    {
	      for(channel=0;channel<16;channel++)
		if((modchan!=channel)&&
		   (crosstalk[modchan][channel]>=crossed_triggers))
		  {
		    if(n_crosstalking++ == 0)printf(" Cross-talk: ");
		    printf("%d ",channel+1);
		  }
	      if(n_crosstalking)puts("");
	    }
	}

      while(1)
	{
	  fprintf(stdout, "Enter output filename (or \"none\") [%s]: ",
		  filename);
	  fflush(stdout);	  
	  fgets(line,80,stdin);

	  line[79]='\0';
	  line[strlen(line)-1]='\0';

	  outfp=NULL;

	  /* If user enters nothing then use old file name as default */
	  if(strcmp(line,"")==0)strcpy(line,filename);
	  if(strcmp(line,"none")==0)break;
	  
	  outfp=fopen(line,"a");
	  if(outfp == NULL)
	    {
	      fprintf(stdout,"\007... Could not open file \"%s\"\n",line);
	      continue;
	    }

	  strcpy(filename,line);
	  break;
	}

      if(outfp==NULL)continue;

      for(modchan=0;modchan<16;modchan++)
	fprintf(outfp,"%3d ",levels_dc[modchan]);

      fprintf(outfp," %% CFD#%-2d",module);

      for(modchan=0;modchan<16;modchan++)
	{
	  int crossed_triggers;
	  int n_crosstalking=0;
	  int channel;

	  crossed_triggers=crosstalk[modchan][modchan]/2;
	  if(ncounts[modchan]<crossed_channel_counts)
	    {
	      fprintf(outfp," ; %d DEAD",modchan+1);
	    }
	  else
	    {
	      for(channel=0;channel<16;channel++)
		if((modchan!=channel)&&
		   (crosstalk[modchan][channel]>=crossed_triggers))
		  {
		    if(n_crosstalking++ == 0)
		      fprintf(outfp," ; %d CROSS",modchan+1);
		    fprintf(outfp,"%c%d",
			    ((n_crosstalking==1)?' ':','),channel+1);
		  }
	    }
	}
      
      fprintf(outfp,"\n");

      fclose(outfp);
    }
  
  return;
}






/*:::::::::::::::::::::::::    test_1fold    :::::::::::::::::::::*/
void test_1fold()
{
  int nchan=NCFD*16;
  int rate[NCFD*16];
  int i=0;
  int chan;
  char line[80];
  char filename[80];
  FILE *fp;

  /* first, mask off all channels */
  for(i=0;i<NCFD;i++)
    Cfd_mask[i]=0xFFFF;
  cfd_write_mask(-1);

  printf("Triggering each channel %d times\n",N_1FOLD);
  
  for(i=0;i<nchan;i++){

    /* chose channel to test */
    chan= i + 1;

    /* now turn it on */
    cfd_mask_encode(1,&chan,MASK_ENCODE_ON);
    cfd_write_mask(-1);

    /* clear scaler */
    c_cfsa(F9,Ext_coinc_scal,&Dumint,&Q_resp,0,1);

    /* trigger CFDs N times */
    trigger_cfds(N_1FOLD);

    /* read out co-incidence scaler */
    c_cfsa(F0,Ext_coinc_scal,&rate[i],&Q_resp,0,1);

    /* turn channel off again */
    cfd_mask_encode(1,&chan,MASK_ENCODE_OFF);
    cfd_write_mask(-1); 

  }

  /* print value: */
  for(i=0;i<nchan;i++)
    printf("Channel %d, triggered: %d\n",i+1,rate[i]);

  
  printf("Enter filename for data save: ");
  fgets(line,80,stdin);
  if(sscanf(line,"%s",filename)==-1)
    printf("O.K. don't save the data.\n");
  else{
    if((fp=fopen(filename,"w"))==NULL){
      printf("Error, cannot open %s, data cannot be saved.\n",filename);
      return;
    }
    for(i=0;i<nchan;i++)
        fprintf(fp,"%d %d\n",i+1,rate[i]);

    fclose(fp);
    printf("Data saved successfully.\n");
  }

}


/*::::::::::::::::::::::::: single_chan_rate  :::::::::::::::::::::*/
void single_chan_rate()
{
  int nchan=NCFD*16;
  int rate[NCFD*16];
  int i=0;
  int chan;
  int temp;
  int nread;
  float n_time;
  char line[80];
  char filename[80];
  FILE *fp;

  printf("\nRemember to set multiplicity discriminator threshold for single folds...");

  printf("\nEnter time for each channel to run free (in secs): ");
  fgets(line,80,stdin);
  sscanf(line,"%f",&n_time);

  /* first, mask off all channels */
  for(i=0;i<NCFD;i++)
    Cfd_mask[i]=0xFFFF;
  cfd_mask_non_verbose(-1);

  do{
    printf("Test all pixels? (-1 for all): ");
    fgets(line,80,stdin);
    if (strlen(line)==1){
      break;             
    }                    
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<-1) || (temp==0));

  if (temp>-1){
    cfd_get_maskpixels(MASK_ENCODE_ON);
    cfd_mask_non_verbose(-1);

    /* clear scaler */
    c_cfsa(F9,Ext_coinc_scal,&Dumint,&Q_resp,0,1);

    /* wait for n_time seconds */
    c_wait_time(n_time);

    /* read out co-incidence scaler */
    c_cfsa(F0,Ext_coinc_scal,&rate[i],&Q_resp,0,1);
    /* print value: */
    printf("rate is: %f Hz",i+1,rate[i]/n_time);
    /* turn channel off again */
    cfd_mask_encode(1,&chan,MASK_ENCODE_OFF);
    cfd_mask_non_verbose(-1); 

  }
  else {  

 
  for(i=0;i<nchan;i++){

    /* choose channel to test */
    chan= i + 1;

    /* now turn it on */
    cfd_mask_encode(1,&chan,MASK_ENCODE_ON);
    cfd_mask_non_verbose(-1);

    /* clear scaler */
    c_cfsa(F9,Ext_coinc_scal,&Dumint,&Q_resp,0,1);

    /* wait for n_time seconds */
    c_wait_time(n_time);

    /* read out co-incidence scaler */
    c_cfsa(F0,Ext_coinc_scal,&rate[i],&Q_resp,0,1);
    /* print value: */
    printf("Pixel %d rate: %f Hz",i+1,rate[i]/n_time);
    /* turn channel off again */
    cfd_mask_encode(1,&chan,MASK_ENCODE_OFF);
    cfd_mask_non_verbose(-1); 

  }
  

  /* print value: */
  for(i=0;i<nchan;i++)
    printf("Channel %d rate: %f Hz\n",i+1,rate[i]/n_time);
  
  
  printf("Enter filename for data save: ");
  fgets(line,80,stdin);
  if(sscanf(line,"%s",filename)==-1)
    printf("O.K. don't save the data.\n");
  else{
    if((fp=fopen(filename,"w"))==NULL){
      printf("Error, cannot open %s, data cannot be saved.\n",filename);
      return;
    }
    for(i=0;i<nchan;i++)
        fprintf(fp,"%d %d\n",i+1,rate[i]);

    fclose(fp);
    printf("Data saved successfully.\n");
  }
  }
}


/*:::::::::::::::::::::::::     trigger_cfds    :::::::::::::::::::::*/
void trigger_cfds(int ntimes)
{
  int i;
  int j;
  
  for (i=0;i<NCFD;i++){
    if (Cfd_online[i]==0){
      printf("Module %d is not online, skipping...\n",i+i);
      continue;
    }  
    for(j=0;j<ntimes;j++){
      c_cfsa(F25,Extcfd[i*16+A0],&Dumint,&Q_resp,0,1);
    }
    printf("Finished module %d, Q: %d\n",i+1,Q_resp);
  }

}


/*::::::::::::::::::::::::: cfd_get_thresh_mode :::::::::::::::::::::*/
void cfd_get_thresh_mode()
{
  char line[80];
  int temp;
  int nread;

  do{
    printf("Threshold Mode? [%d]\n",(Thresh_mode==MV ? 1 : 2));
    printf("1. mV\n");
    printf("2. d.c.\n");
    fgets(line,80,stdin);
    if (strlen(line)==1){                           /* default! */              
      temp=(Thresh_mode==MV ? 1 : 2);
      break;                 
    }                        
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<1) || (temp>2));

  if (temp==1){
    Thresh_mode=MV;
    strcpy(Thresh_scale,"mV");
  }
  else{
    Thresh_mode=DC;
    strcpy(Thresh_scale,"d.c.");
  }

  return;
}  
   

#ifndef NO_SCALERS
/*::::::::::::::::::::::::::  do_chan_bias  :::::::::::::::::::::::::*/
void do_chan_bias()
{
  int nsteps;
  int i;
  int j;
  int k;
  int chan;
  int data[MAX_SCALER_CHAN];
  int coinc;
  int inhibit;
  int wdata=0;
  unsigned int controller_ext;
  float thresh_start;
  float thresh_final;
  float thresh_step;
  int ithresh;
  float dt;
  float *bias_data;
  int *bias_thresh;
  FILE *fp;
  char answer[80];
  char line[80];
  char filename[80];


  c_cdreg(&controller_ext,Branch,SCAL_CRATE,CCONTROL,0);
  
  cfd_get_thresh_mode();

  printf("\nEnter Starting Discriminator Value (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_start);
  printf("\nEnter Final Discriminator Value (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_final);
  printf("\nEnter Discriminator Step Size (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_step);
  printf("\nEnter time for each step (in secs): ");
  fgets(line,80,stdin);
  sscanf(line,"%f",&dt);


  do{
    printf("\nEnter filename for data save: ");
    fgets(line,80,stdin);    
  }while(sscanf(line,"%s",filename)==-1);
  
  if((fp=fopen(filename,"w"))==NULL){
    printf("Error, cannot open %s, data cannot be saved.\n",filename);
    return;
  }


  nsteps=(int) ((thresh_final - thresh_start)/thresh_step);
  if (nsteps < 0){
    printf("Step size %f has wrong sign, changing to %f\n",
	   thresh_step, -1*thresh_step);
    thresh_step*=-1;
    nsteps*=-1;
  }
  
  nsteps++;       /* e.g. (2-1)/1=1 whereas 2 pts! */


  /*::::::::::::::::::::::::::  main loop  ::::::::::::::::::::::::::*/
  for(i=0;i<nsteps;i++){

    printf("Starting main loop\n");

    /* set discriminator threshold */
    ithresh=(int) (thresh_start+(float)i*thresh_step);
    cfd_write_threshold(-1,-1,ithresh);
    
    
    /*--------- Clear all scalers ------------*/

    /* inhibit crate */
    inhibit=1;
    c_ccci(controller_ext,inhibit,0,1);

    /* first clear LeCroy scalers */
    wdata=64;
    for(j=0;j<N_LC_SCAL;j++){
      c_cfsa(F16,Lc_scal_cdreg[j+A0],&wdata,&Q_resp,0,1); 
      /* i.e. F16, A0 */
      /* printf("scaler: %d  Q: %d  w_data: %d\n",i,q_resp,wdata); */
    } 

    /* problem with crate 5. Camac clear does not work any more */
    /* for LeCroy scalers. So, now try SOB clear.               */
    wdata=0; 
    c_cfsa(F25,Sob_cdreg,&wdata,&Q_resp,0,1); 

    /* now clear Phillips scalers */
    for(j=0;j<N_PHIL_SCAL;j++)
      c_cfsa(F11,Phil_scal_cdreg[j*16+A4],&Dumint,&Q_resp,0,1);

    /* uninhibit crate */
    inhibit=0;
    c_ccci(controller_ext,inhibit,0,1);


    /* clear co-incidence scaler - F9, A? */
    c_cfsa(F9,Ext_coinc_scal,&Dumint,&Q_resp,0,1);

    /*---------------- Wait a while..... --------------*/

    /* wait time */
    c_wait_time(dt);

    /*--------------  Read out scalers ----------------*/

    /* read co-inc scaler */
    c_cfsa(F0,Ext_coinc_scal,&coinc,&Q_resp,0,1);


    /* inhibit crate */
    inhibit=1;
    c_ccci(controller_ext,inhibit,0,1);

    /* load LeCroy scaler contents into internal buffer and start readout */
    wdata=7968;
    for(j=0;j<N_LC_SCAL;j++)
      c_cfsa(F16,Lc_scal_cdreg[j+A0],&wdata,&Q_resp,0,1);
    /* i.e. F16, A0 */

    
    chan=0;              /* channel index */

    /* Read LeCroy scalers using F(2)A(0) */
    for(j=0;j<N_LC_SCAL;j++){

      /* The first address (FA) and number to read (NR) have already */
      /* been selected with the F16,A0,W=7968 above */

      for(k=0;k<32;k++){
	c_cfsa(F2,Lc_scal_cdreg[j+A0],&data[chan++],
	       &Q_resp,0,1);
        /* printf("%d %d %d %d\n",i,j,lc_scal_val[i*32+j],q_resp); */ 
        /* i.e. F2, A0 */
      }
    }


    /* Now read Phillips scalers */
     for(j=0;j<N_PHIL_SCAL;j++){

      /* First reset the scaler bank selection register */
      c_cfsa(F11,Phil_scal_cdreg[j*16+A1],&Dumint,&Q_resp,0,1);
      /* i.e. F11, A1 */
      
      /* Next loop over all channels in unit and store scaler value */
      for(k=0;k<32;k++){
	c_cfsa(F4,Phil_scal_cdreg[j*16+A15],&data[chan++],
	       &Q_resp,0,1);
	/*	printf("%d %d %d %d\n",i,j,phil_scal_val[i*32+j],q_resp);*/
        /* i.e. F4, A15 */
      }

    }  /* Finished Q-Block scaler read for all Phillips modules */   


    /* uninhibit crate */
    inhibit=0;
    c_ccci(controller_ext,inhibit,0,1);

    printf("chan=%d\n\n",chan);
    
    /*-----------  write data to file  --------------------------*/
    fprintf(fp,"%d ",ithresh);
    for (chan=0;chan<MAX_CHAN;chan++)
      fprintf(fp,"%.1f ",(float) data[chan]/dt);
    fprintf(fp,"%.1f ",(float) coinc/dt);    
    fprintf(fp,"\n");
    
    printf("** Successfully wrote data for threshold %d %s \n\n",
	   ithresh,Thresh_scale);
    printf(" nsteps= %d  i= %d\n\n",nsteps,i);
  }
 
  fclose(fp);

  return;
}
#endif // NO_SCALERS  

/*:::::::::::::::::::::::::::::  do_both_bias  ::::::::::::::::::::::::::*/
void do_both_bias()
{
  int nsteps;
  int i;
  int j;
  int data;
  float thresh_start;
  float thresh_final;
  float thresh_step;
  int ithresh;
  float dt;
  float *bias_data;
  float *bias_data2;


  int *bias_thresh;
  FILE *fp;
  char answer[80];
  char line[80];
  char filename[80];
  
  /* Set 4413s to remote mode */
  for (j=0;j<N_4413;j++)
    if (L4413_online[j]){
      c_cfsa(F26,Ext4413[j],&Dumint,&Q_resp,0,1);
      printf("Set remote, 4413 #%d, Q: %d\n",j,Q_resp);
    }

  /* Force threshold scale to be mV */
  Thresh_mode=MV;
  strcpy(Thresh_scale,"mV");

  printf("\nEnter Starting Discriminator Value (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_start);
  printf("\nEnter Final Discriminator Value (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_final);
  printf("\nEnter Discriminator Step Size (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_step);
  printf("\nEnter time for each step (in secs): ");
  fgets(line,80,stdin);
  sscanf(line,"%f",&dt);

  nsteps=(int) ((thresh_final - thresh_start)/thresh_step);
  if (nsteps < 0){
    printf("Step size %f has wrong sign, changing to %f\n",
	   thresh_step, -1*thresh_step);
    thresh_step*=-1;
    nsteps*=-1;
  }
  
  nsteps++;       /* e.g. (2-1)/1=1 whereas 2 pts! */
  bias_data=(float *)malloc(sizeof(float)*(nsteps+1));  /* cfds */
  bias_data2=(float *)malloc(sizeof(float)*(nsteps+1)); /* 4413s */
  bias_thresh=(int *)malloc(sizeof(int)*(nsteps+1));

  
  for(i=0;i<nsteps;i++){

    /* set discriminator threshold */
    ithresh=(int) (thresh_start+(float)i*thresh_step);
    cfd_write_threshold(-1,-1,ithresh);
    for (j=0;j<N_4413;j++)
      if (L4413_online[j]){
	c_cfsa(F17,Ext4413[j],&ithresh,&Q_resp,0,1);
	printf("Setting threshold on 4413 #%d, Q: %d\n",j+1,Q_resp);
      }
    
    
    /* print bias curve values so far */
    for (j=0;j<i;j++)
      printf("Threshold: %3d %s, Rate CFD (Hz.): %f, Rate 4413 (Hz.): %.1f\n",
	     *(bias_thresh+j),Thresh_scale,*(bias_data+j),*(bias_data2+j));
    
    
    /* clear scaler - F9, A? */
    c_cfsa(F9,Ext_coinc_scal2,&Dumint,&Q_resp,0,1);
    c_cfsa(F9,Ext_coinc_scal,&Dumint,&Q_resp,0,1);

    /* wait time */
    c_wait_time(dt);

    /* Read out scaler */
    c_cfsa(F0,Ext_coinc_scal,&data,&Q_resp,0,1);
    *(bias_data+i)=(float) data/dt;
    *(bias_thresh+i)=ithresh;
    c_cfsa(F0,Ext_coinc_scal2,&data,&Q_resp,0,1);
    *(bias_data2+i)=(float) data/dt;
    
  }

  printf("Threshold: %3d %s, Rate CFD (Hz.): %.1f  Rate 4413 (Hz.) %.1f\n",
         *(bias_thresh+i-1),Thresh_scale,*(bias_data+i-1),*(bias_data2+i-1));
  

  printf("Enter filename for data save: ");
  fgets(line,80,stdin);
  if(sscanf(line,"%s",filename)==-1)
    printf("OK, don't save the data.\n");
  else{
    if((fp=fopen(filename,"w"))==NULL){
      printf("Error, cannot open %s, data cannot be saved.\n",filename);
      return;
    }
    for(i=0;i<nsteps;i++)
      fprintf(fp,"%d %f %f\n",*(bias_thresh+i),*(bias_data+i),
	      *(bias_data2+i));

    fclose(fp);
    printf("Data saved successfully.\n");
  }

  /* Set 4413s to local? */

  return;
}



/*::::::::::::::::::::::::::  do_coinc_bias  :::::::::::::::::::::::::*/
void do_coinc_bias(int n)
/* n=1 means read scaler channel 1 */
/* n=2 means read scaler channels 1 and 2 */
{
  int nsteps;
  int i;
  int j;
  int data;
  float thresh_start;
  float thresh_final;
  float thresh_step;
  int ithresh;
  float dt;
  float *bias_data;
  float *bias_data2;


  int *bias_thresh;
  FILE *fp;
  char answer[80];
  char line[80];
  char filename[80];
  
  cfd_get_thresh_mode();

  printf("\nEnter Starting Discriminator Value (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_start);
  printf("\nEnter Final Discriminator Value (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_final);
  printf("\nEnter Discriminator Step Size (%s): ",Thresh_scale);
  fgets(line,80,stdin);
  sscanf(line,"%f",&thresh_step);
  printf("\nEnter time for each step (in secs): ");
  fgets(line,80,stdin);
  sscanf(line,"%f",&dt);

  nsteps=(int) ((thresh_final - thresh_start)/thresh_step);
  if (nsteps < 0){
    printf("Step size %f has wrong sign, changing to %f\n",
	   thresh_step, -1*thresh_step);
    thresh_step*=-1;
    nsteps*=-1;
  }
  
  nsteps++;       /* e.g. (2-1)/1=1 whereas 2 pts! */
  bias_data=(float *)malloc(sizeof(float)*(nsteps+1));
  if (n==2)
    bias_data2=(float *)malloc(sizeof(float)*(nsteps+1));

  bias_thresh=(int *)malloc(sizeof(int)*(nsteps+1));

  for(i=0;i<nsteps;i++){

    /* set discriminator threshold */
    ithresh=(int) (thresh_start+(float)i*thresh_step);
    cfd_write_threshold(-1,-1,ithresh);
    
    /* print bias curve values so far */
    for (j=0;j<i;j++)
      if (n==1)
        printf("Threshold: %3d %s,  Rate (Hz.): %.1f Q: %d\n",
	       *(bias_thresh+j),Thresh_scale,*(bias_data+j),Q_resp);
      else if (n==2){
        printf("Threshold: %3d %s, Any 2 rate (Hz.): %f,  PST rate (Hz.): %.1f",
	       *(bias_thresh+j),Thresh_scale,*(bias_data+j),*(bias_data2+j));
        printf("  Q: %d\n",Q_resp);
      }
    
    /* clear scaler - F9, A? */
    if (n==2)
      c_cfsa(F9,Ext_coinc_scal2,&Dumint,&Q_resp,0,1);
    
    c_cfsa(F9,Ext_coinc_scal,&Dumint,&Q_resp,0,1);

    /* wait time */
    c_wait_time(dt);

    /* Read out scaler */
    c_cfsa(F0,Ext_coinc_scal,&data,&Q_resp,0,1);
    *(bias_data+i)=(float) data/dt;
    *(bias_thresh+i)=ithresh;
    if (n==2){
      c_cfsa(F0,Ext_coinc_scal2,&data,&Q_resp,0,1);
      *(bias_data2+i)=(float) data/dt;
    }
  }

  if (n==1)
    printf("Threshold: %3d %s,  Rate (Hz.): %.1f Q: %d\n\n",
      	 *(bias_thresh+i-1),Thresh_scale,*(bias_data+i-1),Q_resp);
  else if (n==2){
    printf("Threshold: %3d %s, Any 2 rate (Hz.): %.1f  PST rate (Hz.) %.1f",
         *(bias_thresh+i-1),Thresh_scale,*(bias_data+i-1),*(bias_data2+i-1));
    printf("  Q: %d\n",Q_resp);
  }

  printf("Enter filename for data save: ");
  fgets(line,80,stdin);
  if(sscanf(line,"%s",filename)==-1)
    printf("O.K. don't save the data.\n");
  else{
    if((fp=fopen(filename,"w"))==NULL){
      printf("Error, cannot open %s, data cannot be saved.\n",filename);
      return;
    }
    for(i=0;i<nsteps;i++)
      if(n==1)
        fprintf(fp,"%d %f\n",*(bias_thresh+i),*(bias_data+i));
      else if (n==2)
        fprintf(fp,"%d %f %f\n",*(bias_thresh+i),*(bias_data+i),
               *(bias_data2+i));

    fclose(fp);
    printf("Data saved successfully.\n");
  }

  return;
}

/*::::::::::::::::::::::::::::: cfd_read_masks :::::::::::::::::::::::*/
void cfd_read_masks()
{
  unsigned int mask;
  unsigned int tempmask;
  char binmask[17];
  int i;
  int j;
  int ntoff=0;
  int *toff;
  
  toff=(int *)malloc(1*sizeof(float));  /* needed before reallocing */

  for (i=0;i<NCFD;i++){
    if (Cfd_online[i]==0){
      printf("Module %d is not online, skipping...\n",i+i);
      continue;
    }
    strcpy(binmask,"0000000000000000");
    c_cfsa(F1,Extcfd[i*16+A0],&mask,&Q_resp,0,1);
    mask=mask&0xFFFF;
    tempmask=mask;
    for(j=0;j<16;j++){
      if (tempmask & 1){
	binmask[15-j]='1';
	ntoff++;
	if((toff=realloc(toff,sizeof(int)*ntoff))==NULL){
	  printf("Cannot allocate Memory, exiting...\n");
	  exit(EXIT_FAILURE);
	}
	*(toff+ntoff-1)=i*16+j+1;
      }
      tempmask=tempmask >> 1;
    }

    printf("Module %2d, read mask: 0x%04X (%d)(%s), Q: %d\n",
	   i+1,mask,mask,binmask,Q_resp);
  }

  printf("\nChannels off:\n");
  for(i=0;i<ntoff;i+=10){
    for(j=0;(j<10)&&((i+j)<ntoff);j++)
      printf("%3d ",*(toff+i+j));
    printf("\n");
  }
}



/*::::::::::::::::::::::::: cfd_read_width_dead :::::::::::::::::::::*/

void cfd_read_width_dead(int module)
{
  int i;
  int j;
  int width_dead;
  
  if (module==-1){                        /* module==-1 */
    for (i=0;i<NCFD;i++){
      if (Cfd_online[i]==0){
	printf("Module %d is not online, skipping...\n",i+i);
	continue;
      }
      c_cfsa(F1,Extcfd[i*16+A1],&width_dead,&Q_resp,0,1);
      printf("Module %2d, read width/deadtime: 0x%02X (%3d), Q: %d\n",
	     i+1,width_dead,width_dead,Q_resp);  /* can also do %0#4X */
    }                                          /* but doesn't look as nice */
  }
  else {                                   /* module!=-1 */
    if (Cfd_online[module-1]==0){
      printf("Module %d is not online, skipping...\n",i+i);
      return;
    }           
    c_cfsa(F1,Extcfd[(module-1)*16+A1],&width_dead,&Q_resp,0,1);
    printf("Module %2d, read width/deadtime: 0x%02X (%3d), Q: %d\n",
	   module,width_dead,width_dead,Q_resp);
  }
   
  return;
}



/*:::::::::::::::::::::::::: cfd_read_threshold :::::::::::::::::::::::*/

void cfd_read_threshold(int module, int modchan)
{
  int i;
  int j;
  int channel;
  int thresh_in_dc;
  int thresh_in_mv;
  char scale[80];

  printf("\n\nThreshold values read via CAMAC:\n");
  
  if (module==-1){                        /* module==-1 */
    for (i=0;i<NCFD;i++){
      if (Cfd_online[i]==0){
	printf("Module %d is not online, skipping...\n",i+i);
	continue;
      }
      if (modchan==-1){                   /* module==-1 && channel==-1 */
	for (j=0;j<16;j++){
	  channel=i*16+j+1;
	  c_cfsa(F0,Extcfd[channel-1],&thresh_in_dc,&Q_resp,0,1);
	  thresh_in_mv=cfd_dc2mv(&channel,&thresh_in_dc);
	  printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
		 channel,i+1,j,thresh_in_dc);
	  printf("(%3d mV)  Q: %d\n",thresh_in_mv,Q_resp);
	}
      }
      else{                                /* module==-1 && channel!=-1 */
	channel=i*16+modchan+1;
	c_cfsa(F0,Extcfd[channel-1],&thresh_in_dc,&Q_resp,0,1);
	thresh_in_mv=cfd_dc2mv(&channel,&thresh_in_dc);
	printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
	       channel,i+1,modchan,thresh_in_dc);
	printf("(%3d mV)  Q: %d\n",thresh_in_mv,Q_resp);
      }
    }
  }
  else {                                   /* module!=-1 */
    if (Cfd_online[module-1]==0){
      printf("Module %d is not online, skipping...\n",i+i);
      return;
    }           
    if (modchan==-1){                      /* module!=-1 && channel==-1 */
      for (j=0;j<16;j++){
	channel=(module-1)*16+j+1;
	c_cfsa(F0,Extcfd[channel-1],&thresh_in_dc,&Q_resp,0,1);
	thresh_in_mv=cfd_dc2mv(&channel,&thresh_in_dc);
	printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
	       channel,module,j,thresh_in_dc);
	printf("(%3d mV)  Q: %d\n",thresh_in_mv,Q_resp);
      }
    }
    else{                                  /* module!=-1 && channel!=-1 */
      channel=(module-1)*16+modchan+1;
      c_cfsa(F0,Extcfd[channel-1],&thresh_in_dc,&Q_resp,0,1);
      thresh_in_mv=cfd_dc2mv(&channel,&thresh_in_dc);
      printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
	     channel,module,modchan,thresh_in_dc);
      printf("(%3d mV)  Q: %d\n",thresh_in_mv,Q_resp);
    }
  }
  
  printf("\n");

  return;
}


/*::::::::::::::::::::::::  cfd_get_maskpixels  ::::::::::::::::::::::::*/

void cfd_get_maskpixels(int encode_mode)
{
  char line[10000];
  char templine[10000];
  int nread;
  int temp;
  int nskip;
  int intdum;
  int *toff;
  int ntoff;
  int i;

  do {
    printf("Please enter pixel to turn %s: ",
	   (encode_mode==MASK_ENCODE_OFF ? "off" : "on"));
    fgets(line,10000,stdin);
  }
  while(sscanf(line,"%s",templine)==-1);

  line[strlen(templine)]='\0';   /* in case trailing blank spaces*/ 

  ntoff=0;                 /* needed malloc before reallocing */
  if((toff=(int *)malloc(1*sizeof(float)))==NULL){
      printf("Cannot allocate Memory, exiting...\n");
      exit(EXIT_FAILURE);
    }    

  if(*(line+strlen(line)-1)!=',')
    strcat(line,",");           /* hack to eliminate special case of */
                                /* reading last entry of string      */
  while(sscanf(line,"%d,%n",&intdum,&nskip)!=EOF){
    if((toff=realloc(toff,sizeof(int)*(ntoff+1)))==NULL){
      printf("Cannot allocate Memory, exiting...\n");
      exit(EXIT_FAILURE);
    }
    sscanf(line,"%d,",toff+ntoff);
    strcpy(line,line+nskip);     /* chop off first entry each time */
    ntoff++;
  }

  /* Now have list of ntoff tubes off in aray toff */
  /* Next call routine which coded this array into masks */

  cfd_mask_encode(ntoff,toff,encode_mode);
  printf("Pixel %d....",*toff);
  return;
}

/*::::::::::::::::::::::::  cfd_get_maskchannels  ::::::::::::::::::::::::*/

void cfd_get_maskchannels(int encode_mode)
{
  char line[10000];
  char templine[10000];
  int nread;
  int temp;
  int nskip;
  int intdum;
  int *toff;
  int ntoff;
  int i;

  do {
    printf("Please enter channels to turn %s (comma separated list): ",
	   (encode_mode==MASK_ENCODE_OFF ? "off" : "on"));
    fgets(line,10000,stdin);
  }
  while(sscanf(line,"%s",templine)==-1);

  line[strlen(templine)]='\0';   /* in case trailing blank spaces*/ 

  ntoff=0;                 /* needed malloc before reallocing */
  if((toff=(int *)malloc(1*sizeof(float)))==NULL){
      printf("Cannot allocate Memory, exiting...\n");
      exit(EXIT_FAILURE);
    }    

  if(*(line+strlen(line)-1)!=',')
    strcat(line,",");           /* hack to eliminate special case of */
                                /* reading last entry of string      */
  while(sscanf(line,"%d,%n",&intdum,&nskip)!=EOF){
    if((toff=realloc(toff,sizeof(int)*(ntoff+1)))==NULL){
      printf("Cannot allocate Memory, exiting...\n");
      exit(EXIT_FAILURE);
    }
    sscanf(line,"%d,",toff+ntoff);
    strcpy(line,line+nskip);     /* chop off first entry each time */
    ntoff++;
  }

  /* Now have list of ntoff tubes off in aray toff */
  /* Next call routine which coded this array into masks */

  cfd_mask_encode(ntoff,toff,encode_mode);

  return;
}

/*::::::::::::::::::::::::: cfd_mask_encode :::::::::::::::::::::::::*/

void cfd_mask_encode(int ntoff, int *toff, int encode_mode)
{
  
  int i;
  int module;
  /*  int modules[NCFD];*/
  int modchan;
  int bit;


  /* loop through every tube */
  for (i=0;i<ntoff;i++){
    /* first determine which module tube belongs to */
    module=(int) (*(toff+i)-1)/16 + 1;
    /* next determine which tube in module it is */
    if(module>1)
      modchan=(int) fmod(*(toff+i)-1,16*(module-1));
    else
      modchan=*(toff+i)-1;
    /* now, figure out which bit corresponds to channel */
    bit=(int) pow((double) 2,(double) modchan);
    /* next, turn that bit on in mask by OR */
    Cfd_mask[module-1]=Cfd_mask[module-1] | bit;   /* OR */
    if (encode_mode==MASK_ENCODE_ON)
      Cfd_mask[module-1]=Cfd_mask[module-1] ^ bit;   /* Exclusive OR */
  }
  
  /* Above code looks funny. Works as follows:                         */
  /* By default the bit is set for the channel to be turned off, which */
  /* is ok when we want to switch a tube off (or when a tube is        */
  /* already off). Next we check if we actually want to turn the tube  */
  /* on, using an XOR. Now we are guaranteed that the tube is already  */
  /* off, otherwise (if user tried to turn on a tube that was not off  */
  /* then its mode would just be toggled.)                             */

  return;
}

/*:::::::::::::::::::::::::::::  cfd_get_modmask  ::::::::::::::::::::::*/

void cfd_get_modmask(int module)
{
  char line[80];
  int nread;
  int temp;
  int i;
  int do_all=0;

  if (module==-1){
    module=1;
    do_all=1;
  }

  temp=Cfd_mask[module-1];

  do{
    printf("Mask? (0-65535) [%d]: ",Cfd_mask[module-1]);
    fgets(line,80,stdin);
    if (strlen(line)==1){                           /* default! */    
      temp=Cfd_mask[module-1];          
      break;                 
    }                        
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<0) || (temp>65535));

  
  if (do_all)
    for(i=0;i<NCFD;i++)
      Cfd_mask[i]=temp;
  else
    Cfd_mask[module-1]=temp;

  return;
}


/*:::::::::::::::::::::::::::  cfd_write_mask ::::::::::::::::::::::::*/

void cfd_write_mask(int module)
{
  int i;
  int tmp_mask;   /* for debugging */


  if (module==-1){
    for (i=0;i<NCFD;i++){
      if (Cfd_online[i]==0){
	printf("Module: %2d is not online, skipping...\n",i+i);
	continue;
      }
      tmp_mask=Cfd_mask[i];
      c_cfsa(F17,Extcfd[i*16+A0],&Cfd_mask[i],&Q_resp,0,1);
      printf("Module: %2d, wrote mask: 0x%04X, Q: %d\n",
	     i+1,Cfd_mask[i],Q_resp);
      if(tmp_mask!=Cfd_mask[i]){
	printf("** ERROR - camac module has overwritten o/p data\n");
	exit;
      }
    }
  }
  else {                                   /* module!=-1 */
    if (Cfd_online[module-1]==0){
      printf("Module: %2d is not online, skipping...\n",i+i);
      return;
    }           
    c_cfsa(F17,Extcfd[(module-1)*16+A0],&Cfd_mask[module-1],&Q_resp,0,1);
    printf("Module: %2d, wrote mask: 0x%04X, Q: %d\n",
	   module,Cfd_mask[module-1],Q_resp);
  }
  
  printf("\n");
  
  return;
}
      
/*:::::::::::::::::::::::::::  cfd_mask_non_verbose ::::::::::::::::::::::::*/

void cfd_mask_non_verbose(int module)
{
  int i;
  int tmp_mask;   /* for debugging */


  if (module==-1){
    for (i=0;i<NCFD;i++){
      if (Cfd_online[i]==0){
	printf("Module: %2d is not online, skipping...\n",i+i);
	continue;
      }
      tmp_mask=Cfd_mask[i];
      c_cfsa(F17,Extcfd[i*16+A0],&Cfd_mask[i],&Q_resp,0,1);
/*      printf("Module: %2d, wrote mask: 0x%04X, Q: %d\n",
	     i+1,Cfd_mask[i],Q_resp);  */
      if(tmp_mask!=Cfd_mask[i]){
	printf("** ERROR - camac module has overwritten o/p data\n");
	exit;
      }
    }
  }
  else {                                   /* module!=-1 */
    if (Cfd_online[module-1]==0){
      printf("Module: %2d is not online, skipping...\n",i+i);
      return;
    }           
    c_cfsa(F17,Extcfd[(module-1)*16+A0],&Cfd_mask[module-1],&Q_resp,0,1);
    printf("Module: %2d, wrote mask: 0x%04X, Q: %d\n",
	   module,Cfd_mask[module-1],Q_resp);
  }
  
  printf("\n");
  
  return;
}
      

/*:::::::::::::::::::::::::: cfd_get_width_dead ::::::::::::::::::::::*/

void cfd_get_width_dead(unsigned int *width_dead)
{
  char line[80];
  int nread;
  int temp;
  unsigned int width;
  unsigned int dead; 

  dead=  *width_dead & 0x0F;
  width=(*width_dead & 0xF0)>>4;

  temp=width;

  do{
    printf("Width? (0-15) [%d]: ",width);
    fgets(line,80,stdin);
    if (strlen(line)==1){                           /* default! */    
      temp=width;          
      break;                 
    }                        
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<0) || (temp>15));

  width=temp;

  temp=dead;

  do{
    printf("Dead time? (0-15) [%d]: ",dead);
    fgets(line,80,stdin);
    if (strlen(line)==1){                           /* default! */    
      temp=dead;          
      break;                 
    }                        
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<0) || (temp>15));

  dead=temp;

  *width_dead=(width << 4)+dead;

  return;
}


/*::::::::::::::::::::::::: cfd_write_width_dead :::::::::::::::::::::*/

void cfd_write_width_dead(int module, unsigned int width_dead)
{
  int i;
  int j;
  
  if (module==-1){                        /* module==-1 */
    for (i=0;i<NCFD;i++){
      if (Cfd_online[i]==0){
	printf("Module %d is not online, skipping...\n",i+i);
	continue;
      }
      c_cfsa(F17,Extcfd[i*16+A1],&width_dead,&Q_resp,0,1);
      printf("Module %2d, wrote width/deadtime: 0x%02X, Q: %d\n",
	     i+1,width_dead,Q_resp);  /* can also do %0#4X but */
    }                                            /* doesn't look as nice. */
  }
  else {                                   /* module!=-1 */
    if (Cfd_online[module-1]==0){
      printf("Module %d is not online, skipping...\n",i+i);
      return;
    }           
    c_cfsa(F17,Extcfd[(module-1)*16+A1],&width_dead,&Q_resp,0,1);
    printf("Module %2d, wrote width/deadtime: 0x%02X, Q: %d\n",
	   module,width_dead,Q_resp);
  }

  printf("\n");
   
  return;
}


/*:::::::::::::::::::::::::::: cfd_get_module :::::::::::::::::::::::::*/

void cfd_get_module(int *module)
{
  char line[80];
  int nread;
  int temp;

  temp=*module;

  do{
    printf("Module? (1-%d, -1 for all) [%d]: ",NCFD,*module);
    fgets(line,80,stdin);
    if (strlen(line)==1){     /* special case - nothing entered so use the  */
      temp=*module;           /* default value. Copying *module to temp may */
      break;                  /* seem superfluous here but is needed in case*/
    }                         /* a previous valid sscanf (e.g. -2) has been */
                              /* performed and then user goes with default. */
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<-1) || (temp>NCFD) || (temp==0));

  *module=temp;

  return;
}



/*:::::::::::::::::::::::::::: cfd_get_modchannel ::::::::::::::::::::::*/

void cfd_get_modchannel(int *modchan)
{
  char line[80];
  int nread;
  int temp;

  temp=*modchan;

  do{
    printf("Channel? (0-15, -1 for all) [%d]: ",*modchan);
    fgets(line,80,stdin);
    if (strlen(line)==1){
      temp=*modchan;    
      break;             
    }                    
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<-1) || (temp>15));

  *modchan=temp;

  return;
}


/*:::::::::::::::::::::::::::::: cfd_get_channel :::::::::::::::::::::::*/

void cfd_get_channel(int *module, int *modchan) 
/* asks user for channel (1-336) number and returns module and channel */
/* (0-15) in module (eg., 21 => 2,4)                                   */  
{
  char line[80];
  int nread;
  int channel;
  int temp;

  if ((*module==-1) || (*modchan==-1)){
    *module=-1;
    *modchan=-1;
    temp=-1;
    channel=-1;
  }
  else{
    channel=(*module-1)*16+*modchan+1;
    temp=channel;
  }

  do{
    printf("Channel? (1-%d, -1 for all) [%d]: ",MAX_CHAN,channel);
    fgets(line,80,stdin);
    if (strlen(line)==1){
      break;             
    }                    
    nread=sscanf(line,"%d",&temp);
  }
  while((nread!=1) || (temp<-1) || (temp>MAX_CHAN) || (temp==0));

  if (temp==-1){
    *module=-1;
    *modchan=-1;
  }
  else {
    *module=(int) (temp-1)/16+1;
    if (*module>1)
      *modchan=(int) fmod((temp-1),16*(*module-1));
    else 
      *modchan=temp-1;
  }

  return;
}



/*:::::::::::::::::::::::::::: cfd_get_threshold :::::::::::::::::::::::::*/

void cfd_get_threshold(int *threshold)
{
  char line[80];
  int nread;
  
  do{
    printf("Threshold? (%s): ",Thresh_scale);
    fgets(line,80,stdin);
    nread=sscanf(line,"%d",threshold);
  }
  while((nread!=1) || (*threshold<0));

  return;
}


/*:::::::::::::::::::::::::: cfd_write_threshold :::::::::::::::::::::::*/

void cfd_write_threshold(int module, int modchan, int threshold)
{
  int i;
  int j;
  int channel;
  int thresh_out;

  printf("\n\nThreshold values written via CAMAC:\n");
  
  
  if (module==-1){                        /* module==-1 */
    for (i=0;i<NCFD;i++){
      if (Cfd_online[i]==0){
	printf("Module %d is not online, skipping...\n",i+i);
	continue;
      }
      if (modchan==-1){                   /* module==-1 && channel==-1 */
	for (j=0;j<16;j++){
	  channel=i*16+j+1;
	  if (Thresh_mode==DC)
	    thresh_out=threshold;
	  else if (Thresh_mode==MV){
	    thresh_out=cfd_mv2dc(&channel,&threshold);
	  }
	  c_cfsa(F16,Extcfd[channel-1],&thresh_out,&Q_resp,0,1);
	  printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
		 channel,i+1,j,thresh_out);
	  printf("(%3d mV)  Q: %d\n",cfd_dc2mv(&channel,&thresh_out),Q_resp);
	}
      }
      else{                                /* module==-1 && channel!=-1 */
	channel=i*16+modchan+1;
	if (Thresh_mode==DC)
	  thresh_out=threshold;
	else if (Thresh_mode==MV){
	  thresh_out=cfd_mv2dc(&channel,&threshold);
	}
	c_cfsa(F16,Extcfd[channel-1],&thresh_out,&Q_resp,0,1);
	printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
	       channel,i+1,modchan,thresh_out);
	printf("(%3d mV)  Q: %d\n",cfd_dc2mv(&channel,&thresh_out),Q_resp);
      }
    }
  }
  else {                                   /* module!=-1 */
    if (Cfd_online[module-1]==0){
      printf("Module %d is not online, skipping...\n",i+i);
      return;
    }           
    if (modchan==-1){                      /* module!=-1 && channel==-1 */
      for (j=0;j<16;j++){
	channel=(module-1)*16+j+1;
	if (Thresh_mode==DC)
	  thresh_out=threshold;
	else if (Thresh_mode==MV){
	  thresh_out=cfd_mv2dc(&channel,&threshold);
	}
	c_cfsa(F16,Extcfd[channel-1],&thresh_out,&Q_resp,0,1);
	printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
	       channel,module,j,thresh_out);
	printf("(%3d mV)  Q: %d\n",cfd_dc2mv(&channel,&thresh_out),Q_resp);
      }
    }
    else{                                  /* module!=-1 && channel!=-1 */
      channel=(module-1)*16+modchan+1;
      if (Thresh_mode==DC)
	thresh_out=threshold;
      else if (Thresh_mode==MV){
	thresh_out=cfd_mv2dc(&channel,&threshold);
      }
      c_cfsa(F16,Extcfd[channel-1],&thresh_out,&Q_resp,0,1);
      printf("Chan: %3d  Mod: %2d  Modchan: %2d  thresh.: %3d dc ",
	     channel,module,modchan,thresh_out);
      printf("(%3d mV)  Q: %d\n",cfd_dc2mv(&channel,&thresh_out),Q_resp);

    }
  }
  
  printf("\n");

  return;
}




  
/*::::::::::::::::::::::::::::   tst_status   ::::::::::::::::::::::::::*/

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



/*::::: Subroutine to initialise camac connection to desired crate :::::*/

int init_camac(int crate)      /* Buckley's */
{
  unsigned int    status=0,controller_ext;
  int             q=0,w_data=0;
  unsigned long   signal ;

  signal = initusr() ;    /* Initialize communication with crate cntrller */
  if(!tst_status(signal)) {
    return(0);
  }

  c_cdreg(&controller_ext,Branch,crate,CCONTROL,0);
  c_cccz(controller_ext,0,1); /* !Generate dataway initialize - crate*/ 
  c_cccc(controller_ext,0,1); /* !Generate crate clear - crate */
  c_ccci(controller_ext,0,0,1);/* !Turn off Inhibit - crate */
	
  return(1);
}/**** end subroutine init_camac ****/


void init_cfds()
{
  int     Q_resp ;
  int     data ;
  int     i,j ;
  int     option ;
  int	  q=0,w_data=0;
  unsigned long	signal ;
  int threshold;

  for(j=0;j<NCFD;j++) {
    
    for(i=0;i<16;i++) 
      c_cdreg(&(Extcfd[j*16+i]),Branch,CFD_CRATE,CFD1+j,i) ;
    

      /* Set thresholds */
    for(i=0;i<16;i++){ 
      c_cfsa(F16,Extcfd[j*16+i],&threshold,&Q_resp,0,1) ;
      c_cfsa(F0,Extcfd[j*16+i],&threshold,&Q_resp,0,1);
      printf("CFD %d, CH %d, Set Threshold: %d, Q: %d\n",j+1,i+1,
	     threshold,Q_resp) ;
    }
    
      /* Set input masks */ /* 1-mask off that channel, 0-on */
    data = 0x0000 ;
    c_cfsa(F17,Extcfd[j*16+A0],&data,&Q_resp,0,1) ;
    printf("CFD %d, set input mask to %x, Q: %d\n",j+1,data,Q_resp) ;
    
    /* Set gate-width/deadtime */  
    /* 0-minimum, f-maximum, low 4-bits deadtime, high 4-bits width */
    data = 0x0055 ; /* binary: 01010101, from dacq */
    c_cfsa(F17,Extcfd[j*16+A1],&data,&Q_resp,0,1) ;
    printf("CFD %d, set width/deadtime mask to 0x%x, Q: %d\n",j+1,data,
	   Q_resp) ;
  
    printf("\n");
  } /* end for j */
}
     

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







