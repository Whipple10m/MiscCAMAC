/* 
	simple_acq.c
	J. Buckley/K. Harris
	5/25/95

	Simple 11m data acquisition and trouble-shooting program
	Modified by kh to do single channel/single unit adc reads
*/

#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<unixio.h>
#include	<file.h>
#include	<time.h>
#include	<math.h>
/* #include	<fcntl.h> */

/* Switch between compiling for 10m and 11m */
/* #define	TENM */
#define ELEVENM

typedef char    Boolean_op ;
#define true    1
#define false   0

#define LINELENGTH      300

#define LITLEN          100
#define NLITS           50
#define CMDLEN          100
#define NCMDS           50

/*
 An event is stored in an array of 16 bit words rather than	
 a structure to (1) avoid alignment problems, (2) avoid
 problems from machine dependent type sizes, (3) speed up
 processing time.  The elements of the event array are found
 using the following index definitions:
*/

typedef unsigned int	Evt_Buffer ;	/* Each element of the event	*/
					/* buffer is of this type	*/
#define E_EVTNO		0
#define E_CODE 		1
#define E_SATTIME0	2
#define E_SATTIME1	3
#define E_SATTIME2	4
#define E_EVTTIME	5
#define E_PMTS 		6

#define NTUBES		109
#define NADCS		10
#define EVTSIZE		(E_PMTS+NTUBES)

/* Event codes */
#define C_FRAMEEVT	0
#define C_NORMEVT	1
#define C_INJPED	2	
#define DISPLAY_INTERVAL	100

#define FRAME_INTERVAL	100	/* Take a frame event every 100 events 	*/
#define PED_INTERVAL	10      /* Take an injected pedistal event 	*/
                                /* every 10 events			*/
/* ADC function codes */
#define F_ENABLELAM	26	/* Enables adc LAM */
#define F_TESTSTATUS	27
#define F_EXECUTE	25	/* Clears the adc when ch11 is read */
#define F_TESTLAM	8	/* Tests the LAM (Look At Me) status */
#define F_CLEARLAM	10	/* Clears the adc lam */
#define F_CLEAR		9	/* Clears the adc */
#define F_TEST_GATE	25	/* Uses internal 80ns gate to test adc */
#define F_READ_CLEAR	2	/* Clears the adc when ch11 is read */

/* MALU function codes */
#define F_READ_INPUT_PATTERN	0

#ifdef TENM
#define CRATE1	3
#define CRATE2	4
#endif
#ifdef ELEVENM
#define CRATE1  1
#define CRATE2  2
#endif

/* Slot assignment for CAMAC crate 1			*/
#ifdef TENM
#define	SCAL1	8	/* N/A					*/
#define SCAL2   9
#define SCAL3   10
#define SCAL4	11
#define SCAL5   12	/* Lecroy scal:Singles rates		*/
#define SCAL6   13      /* Lecroy scal:Singles rates            */
#define SCAL7   14      /* Lecroy scal:Singles rates            */
#define SCAL8   15      /* Lecroy scal:Singles rates            */
#define PUR_SCAL 16	/* Purdue livetime/abort/frame scaler   */
#define TDCPHS	17	/* event clock phase TDC		*/
#define EVTCLK  18      /* event clock                          */
#define EVTMRK  19      /* event memory (event marker)          */
#define GPSMRK  20      /* event memory (GPS second marker)     */
#define SATCLK  14      /* satellite clock interface            */
#define SOBBOX1 22      /* Purdue Son-Of-Boxus Input/ouput reg. */
#define LISTPROC1 23    /* Hytec Listprocessor                  */
#define CCONTROL1 24    /* Crate controler 1                    */
#endif

#ifdef ELEVENM
#define PUR_SCAL 8 	/* Purdue livetime/abort/frame scaler	*/
#define TDCPHS	9 	/* event clock phase TDC		*/
#define EVTCLK	10 	/* event clock				*/
#define EVTMRK	11 	/* event memory (event marker)		*/
#define CALMRK	12 	/* event memory (transit calib. marker)	*/
#define GPSMRK	13 	/* event memory (GPS second marker)	*/
#define SATCLK	14 	/* satellite clock interface		*/
#define SCAL1   15      /* Mich scal:Current monitor            */
#define SCAL2   16      /* Mich scal:Current monitor            */
#define SCAL3   17      /* Mich scal:Current monitor            */
#define SCAL4   18      /* Mich scal:Current monitor            */
#define SCAL5   19      /* Mich scal:Singles rates              */
#define SCAL6   20      /* Mich scal:Singles rates              */
#define SCAL7   21      /* Mich scal:Singles rates              */
#define SOBBOX1 22      /* Purdue Son-Of-Boxus Input/ouput reg. */
#define LISTPROC1 23    /* Hytec Listprocessor                  */
#define CCONTROL1 24    /* Crate controler 1                    */
#endif

/* Slot assignment for CAMAC crate 2                    */
#ifdef TENM
#define PHSDLY	20	/* Dual phase delay			*/
#define BURSCL	21	/* Burst scaler				*/
#define ADC1    8       /* 12-channel adc                       */
#define ADC2    9       /* 12-channel adc                       */
#define ADC3    10      /* 12-channel adc                       */
#define ADC4    11      /* 12-channel adc                       */
#define ADC5    12      /* 12-channel adc                       */
#define ADC6    13      /* 12-channel adc                       */
#define ADC7    14      /* 12-channel adc                       */
#define ADC8    15      /* 12-channel adc                       */
#define ADC9    16      /* 12-channel adc                       */
#define ADC10   17      /* 12-channel adc                       */
#define intrpt  18      /* Spare slot used for interupt testing */
#endif
#ifdef ELEVENM
#define TDC1	1  	/* PMT TDC(8 channels)			*/
#define TDC2	2 	/* PMT TDC(8 channels)			*/
#define TDC3	3 	/* PMT TDC(8 channels)			*/
#define TDC4	4 	/* PMT TDC(8 channels)			*/
#define TDC5	5 	/* PMT TDC(8 channels)			*/
#define TDC6	6 	/* PMT TDC(8 channels)			*/
#define TDC7    7       /* PMT TDC(8 channels)			*/
#define PHSDLY  9       /* Dual phase delay			*/
#define BURSCL  10      /* Burst scaler                         */
#define ADC1	11 	/* 12-channel adc			*/
#define ADC2	12 	/* 12-channel adc			*/
#define ADC3	13 	/* 12-channel adc			*/
#define ADC4	14 	/* 12-channel adc			*/
#define ADC5	15 	/* 12-channel adc			*/
#define ADC6	16 	/* 12-channel adc			*/
#define ADC7	17 	/* 12-channel adc			*/
#define ADC8	18 	/* 12-channel adc			*/
#define ADC9	19 	/* 12-channel adc			*/
#define ADC10	20 	/* 12-channel adc			*/
#endif
#define SOBBOX2	22	/* Purdue Son-Of-Boxus Input/ouput reg.	*/
#define LISTPROC2 23	/* Hytec Listprocessor			*/
#define CCONTROL2 24	/* Crate controler 2			*/

long int	branch ;
float		peds[NTUBES+5] ;
/*-----------------------------------------------------------------------
*-- GV @ FLWO           SHOW109         ! NOV 17, 1988 !
*-----------------------------------------------------------------------
*-- Displays REAL HRC events in an hexagonal pattern.
*-- A few changes from PWK version (rtpmt109).
*-- Numbering convention (zones 0, 1 and 2):
*-- 
*-- Converted to C by J. Buckley 
*--
*--            16    17    18
*--
*--          15     6     7    19
*--
*--      14     5     1     2     8
*--
*--         13     4     3     9
*--
*--            12    11    10
*--
*-----------------------------------------------------------------------*/

void rshow109(event)
Evt_Buffer	*event ;
{
	static Boolean_op	first_time = true ;
	static char	f1[200],f2[200],f3[200],f4[200],f5[200],
			f6[200],f7[200],f8[200] ;
	register int	i ;

	if(first_time) {	/* Initialize the format strings the first */
			/* time the function is called		   */
	  first_time = false ;
	  strcat(f1,"                   ");
	  for(i=0;i<4;i++) 
	    strcat(f1,"%5d      ") ;
	  strcat(f1,"\n\n") ;

	  strcat(f2,"                     ");
	  for(i=0;i<6;i++) 
	    strcat(f2,"%5d ") ;
	  strcat(f2,"\n\n") ;

	  strcat(f3,"             ");
	  for(i=0;i<9;i++) 
	    strcat(f3,"%5d ") ;
	  strcat(f3,"\n\n") ;

	  strcat(f4,"                ");
	  for(i=0;i<8;i++) 
	    strcat(f4,"%5d ") ;
	  strcat(f4,"\n\n") ;

	  strcat(f5,"       ");
	  for(i=0;i<11;i++) 
	    strcat(f5,"%5d ") ;
	  strcat(f5,"\n\n") ;

	  strcat(f6,"          ");
	  for(i=0;i<10;i++) 
	    strcat(f6,"%5d ") ;
	  strcat(f6,"\n\n") ;

	  strcat(f7," ");
	  for(i=0;i<13;i++) 
	    strcat(f7,"%5d ") ;
	  strcat(f7,"\n\n") ;

	  strcat(f8,"                   ");
	  for(i=0;i<4;i++) 
	    strcat(f8,"%5d      ") ;
	  strcat(f8,"\n\n") ;
	}

        printf(f1,event[104],event[105],event[106],event[107]) ;
        printf(f2,event[82],event[83],event[84],event[85],event[86],
	 event[87]) ;
        printf(f3,event[103],event[81],event[54],event[55],event[56],event[57],
	 event[58],event[88],event[108]) ;
        printf(f4, event[80],event[53],event[32],event[33],event[34],event[35],
	 event[59],event[89]) ;
        printf(f5,event[102],event[79],event[52],event[31],event[16],event[17],
	 event[18],event[36],event[60],event[90],event[109]) ;
        printf(f6,event[78],event[51],event[30],event[15],event[6],event[7],
	 event[19],event[37],event[61],event[91]) ;
        printf(f7,event[101],event[77],event[50],event[29],event[14],event[5],
	 event[1],event[2],event[8],event[20],event[38],event[62],
	 event[92]) ;
        printf(f6,event[76],event[49],event[28],event[13],event[4],event[3],
	 event[9],event[21],event[39],event[63]) ;
        printf(f5,event[100],event[75],event[48],event[27],event[12],event[11],
	 event[10],event[22],event[40],event[64],event[93]) ;
        printf(f4,event[74],event[47],event[26],event[25],event[24],event[23],
	 event[41],event[65]) ;
        printf(f3,event[99],event[73],event[46],event[45],event[44],event[43],
	 event[42],event[66],event[94]) ;
        printf(f2,event[72],event[71],event[70],event[69],event[68],event[67]);
        printf(f8,event[98],event[97],event[96],event[95]) ;
}

/* void tst_status(errmsg)	
char	*errmsg ;
{
	void	c_ctstat() ;
	unsigned long	status ;

	c_ctstat(&status) ;
	if(status) {
	    printf("ERROR: %s\n",errmsg) ;
	    exit(0) ;
	}
} */

void init_camac()
{

	unsigned int	status=0,controller_ext1,controller_ext2;
	int		q=0,w_data=0;
	void		c_cdreg() ;
	unsigned int	c_cccc();
	unsigned int	c_cccz();
	unsigned int	c_ccci();
	unsigned long	c_cssa() ;
	unsigned long	initusr() ;
	unsigned long	signal ;
	Boolean_op	tst_status() ;


	signal = initusr() ;	/* Initialize communication with CAMAC */
	if(!tst_status(signal)) {
	    printf("Exiting acq11m.\n") ;
	    exit(0) ;
	}

/* get crate external address */	
	c_cdreg(&controller_ext1,branch,CRATE1,CCONTROL1,0);
	c_cdreg(&controller_ext2,branch,CRATE2,CCONTROL2,0);

/* !Init crate 1 */
        c_cccz(controller_ext1,0,1);   /* !Generate dataway initialize - crate #1 */
        c_cccc(controller_ext1,0,1);   /* !Generate crate clear - crate #3 */
        c_ccci(controller_ext1,0,0,1); /* !Turn off Inhibit - crate#2 */

/* !Init crate 2 */
        c_cccz(controller_ext2,0,1);   /* !Generate dataway initialize - crate #2 */
        c_cccc(controller_ext2,0,1);   /* !Generate crate clear - crate #2 */
        c_ccci(controller_ext2,0,0,1); /* !Turn off Inhibit - crate#2 */
   
}/*end subroutine init_camac*/

Boolean_op	read_evt_clock(evt)
Evt_Buffer	*evt ;
{
}

Boolean_op	read_sat_clock(evt)	/* Function under construction! */
Evt_Buffer	*evt ;
{
	static Boolean_op first_time = true ;
	static long int ext_satclk0, ext_satclk1, ext_satclk2 ;
	long int	q_resp0, q_resp1, q_resp2 ;
	void	c_cdreg() ;
	unsigned long	c_cssa() ;

	if(first_time) {	/* The first time the function is called */
			/* combine the crate no., station no. and */
			/* subaddresses into "external register" */
			/* codes.  Do once to save time(?)	 */
	  first_time = false ;
	  c_cdreg(&ext_satclk0, branch, CRATE1, SATCLK, 0) ; 
	  c_cdreg(&ext_satclk1, branch, CRATE1, SATCLK, 1) ; 
	  c_cdreg(&ext_satclk2, branch, CRATE1, SATCLK, 2) ; 
	}
	c_cssa(0,ext_satclk0,&(evt[E_SATTIME0]),&q_resp0) ;
	c_cssa(0,ext_satclk1,&(evt[E_SATTIME1]),&q_resp1) ;
	c_cssa(0,ext_satclk2,&(evt[E_SATTIME2]),&q_resp2) ;
	printf("sat clock: %d, %d, %d\n",evt[E_SATTIME0],
	  evt[E_SATTIME1],evt[E_SATTIME2]) ;
	return((Boolean_op)(q_resp0 && q_resp1 && q_resp2)) ;
}


Boolean_op enable_adc_lams()
{
        register int    i ;
        static Boolean_op first_time = true ;
        static long int ext_adc1 ;
        int        q_resp ;
	int		w_data ;	/* dummy */
	void    	c_cdreg() ;
        unsigned long   c_cfsa() ;

        if(first_time) {     /* The first time the function is called */
                        /* combine the crate no., station no. and */
                        /* subaddresses into "external register" */
                        /* codes.  Do once to save time(?)       */
          first_time = false ;
          c_cdreg(&ext_adc1, branch, CRATE2, ADC1, 0) ;
	}
	c_cfsa(F_ENABLELAM,ext_adc1,&w_data,&q_resp,0,1) ;
/*        printf("enable lam q_resp:%d\n",q_resp) ;*/
	return((Boolean_op)q_resp) ;
}
	
Boolean_op clear_scalers()	/* Function under construction! */
{
}

void	gen_test_trigger() 	/* Function under construction! */
{
}

Boolean_op clear_adcs()
{
	int	i, j ;
	static Boolean_op first_time = true ;
        static int ext_adc[11] ;
        int		q_resp,q_resptot ;
	int		w_data ;	/* dummy */
	void    	c_cdreg() ;
        unsigned long   c_cfsa() ;


        if(first_time) {     /* The first time the function is called */
                        /* combine the crate no., station no. and */
                        /* subaddresses into "external register" */
                        /* codes.  Do once to save time(?)       */
/*          printf("Generate adc extensions for clear\n") ;
          printf("adcs are in crate:%d starting at slot:%d\n",CRATE2,ADC1) ;*/
          first_time = false ;
	  for(i=0;i<NADCS;i++) {
            for(j=0;j<10000;j++) {}
            c_cdreg(&ext_adc[i], branch, CRATE2, ADC1+i, 0) ;
	  }
	}
	q_resptot = true ;
	/* printf("clear adcs and adc lams\n") ; */
	for(i=0;i<NADCS;i++) {
	  c_cfsa(F_CLEARLAM,ext_adc[i],&w_data,&q_resp,0,1) ;
/*          printf("adc[%d] clear lam q:%d\n",i,(int)q_resp) ;*/
	  q_resptot = q_resptot && q_resp ;
          c_cfsa(F_CLEAR,ext_adc[i],&w_data,&q_resp,0,1) ;
/*          printf("adc[%d] clear q:%d\n",i,(int)q_resp) ;*/
          q_resptot = q_resptot && q_resp ;
	}
/*        printf("total q_resp:%d\n",q_resptot) ;*/
	return((Boolean_op)q_resptot) ;
}
	
Boolean_op read_scalers(evt)
Evt_Buffer	*evt ;
{
}

Boolean_op read_adcs(evt)	/* Function under construction! */
Evt_Buffer	*evt ;
{	
        int     naf() ;
        int     q,n,a ;
	int	q_resp ;
	Evt_Buffer	*padc ;

        q_resp = true ;
        for(n=ADC1,padc=evt[E_PMTS];n<=ADC10;n++,padc++) {
          /* printf("adc #%d    n: data,q\n",n) ;*/
           for(a=0;a<12;a++) {
               q = naf(CRATE2,n,a,0,padc) ;
               q_resp = q && q_resp ;
               /*printf("      %d: %d,%d\n",a,*padc,q) ;*/
           } /* end for a */
        } /* end for n */
        return(q_resp) ;
}

void write_evt(fp,pevt)		/* Function under construction! */
int		fp ;
Evt_Buffer	*pevt ;
{
	write(fp,pevt,EVTSIZE) ;
}

Boolean_op awt_trigger()
{
	int	q_resp ;
	int	w_data ;	
	static Boolean_op	first_time = true ;
	static long int extsob1_6 ;	/* SOB1, input #6 is the new */
					/* trigger input register */
	void    	c_cdreg() ;
        unsigned long   c_cssa() ;

	if(first_time) {
	  first_time = false ;	
	  c_cdreg(&extsob1_6,branch,CRATE1,SOBBOX1,6) ;
	}
	do{
	  c_cssa(27,extsob1_6,&w_data,&q_resp) ;
	} while(!q_resp) ;
}

Boolean_op awt_scaler_lam()
{
        int     q_resp ;
        int     w_data ;
        static Boolean_op  first_time = true ;
        static long int extscal1_0 ;     /* Ext for first ADC    */
        void            c_cdreg() ;
        unsigned long   c_cfsa() ;

        if(first_time) {
          first_time = false ;
          c_cdreg(&extscal1_0,branch,CRATE2,SCAL1,0) ;
        }
        do{
          c_cfsa(F_TESTLAM,extscal1_0,&w_data,&q_resp,0,1) ;
        } while(!q_resp) ;
}

Boolean_op awt_adc_lam()
{
        int     q_resp ;
        int     w_data ;
        static Boolean_op  first_time = true ;
        static long int extadc1_0 ;     /* Ext for first ADC	*/
	void   		c_cdreg() ;
        unsigned long   c_cfsa() ;
          
        if(first_time) {
          first_time = false ;
          c_cdreg(&extadc1_0,branch,CRATE2,ADC1,0) ;
        }
        do{
          c_cfsa(F_TESTLAM,extadc1_0,&w_data,&q_resp,0,1) ;
	/*printf("awt_adc q_resp=%d\n",q_resp);*/
        } while(!q_resp) ;
}

void	ped_set()
{
	char    *cmd_str[NCMDS+1] ;
	char	*lit_str[NLITS+1] ;
	char	line[LINELENGTH] ;
	int	ncmd, nlit,single_channel=0,adc_channel=1,crate_slot=8 ;
        int     q_resp,i=0,x=0,go=0,events_taken=0,test_adc=0,repeat=0 ;
        int     w_data,events=10,adc_number=1,camac_slot=0,loop=0 ;
	float	fevents,fadc;
	long int data[12];
        long int extadc1_0 ;

	void    	c_cdreg() ;
        unsigned long   c_cfsa() ;
	unsigned int	c_cccc();	/*generate crate clear*/
	unsigned int	c_cccz();	/*generate dataway initialize*/
	unsigned long	c_reset();	/*initialize the crate controller*/
	void		init_camac();


	init_camac();

	for(i=0;i<NCMDS;i++) {
	  cmd_str[i]=(char *)calloc((size_t)CMDLEN,(size_t)sizeof(char));
        }
        for(i=0;i<NLITS;i++) {
          lit_str[i]=(char *)calloc((size_t)LITLEN,(size_t)sizeof(char));
        }

        printf("\n\n\nADC #		To look at all channels of an ADC\n");
	printf("SLOT #		To test an ADC in an unused slot\n");
        printf("EVENT #		To set the number of events\n");
	printf("CHANNEL #	To look at an individual channel\n");
	printf("STATUS		To list PEDSET variables\n");
	printf("TEST		Toggles between internal/external gates\n");
	printf("<return>	To start collecting\n");
        printf("EXIT		To end program\n\n\n");


	for(;;){

	 do{ /*get commands until operator says go*/

	  printf("PEDSET>");
	  go=0; /* Stay in loop until go command is received */

	  if(getline(line,sizeof(line))<0) break ;
	  extract(line,cmd_str,&ncmd,lit_str,&nlit) ;

	  if(ncmd > 0) {
	    if((strncmp(cmd_str[0],"EVENTS",3)==0)||
	     (strncmp(cmd_str[0],"events",3)==0)){ 
	      if(ncmd >= 2) {
	        sscanf(cmd_str[1],"%d",&events) ;
		if(events<1 || events>10000){
		  printf("valid range is 1-10000\n");
		  events=10;
		}
                printf("Events to be taken: %d\n\n\n",events) ;
	      }
	      else
		printf("Format is...EVENTS ###  ie. EVENTS 100 <return>\n\n");
	    } /* end if strncmp(cmd_str[0] */

            else if((strncmp(cmd_str[0],"ADC",2)==0)||
             (strncmp(cmd_str[0],"adc",2)==0)){
              if(ncmd >= 2) {
                sscanf(cmd_str[1],"%d",&adc_number) ;
		single_channel=0;	/*normal adc read out mode */
		camac_slot=0;	/*testing a normal adc position*/
		crate_slot=adc_number+ADC1-1;
		if(adc_number<=0 || adc_number > 10){
		  printf("valid values are 1-10\n");
		  adc_number=1;
		  crate_slot=8;
		}/*end if adc_number<=0*/
                printf("ADC unit %d is being tested\n\n\n",adc_number) ;
              }/*end if ncmd>=2*/
	    }/*end if strncmp ADC*/

		/*slot is used for testing an adc in a non-normal position*/
            else if((strncmp(cmd_str[0],"SLOT",3)==0)||
             (strncmp(cmd_str[0],"slot",3)==0)){
              if(ncmd >= 2) {
                sscanf(cmd_str[1],"%d",&camac_slot) ;
		single_channel=0;
		crate_slot=camac_slot;
		if(camac_slot<=0 || camac_slot > 24){
		  printf("valid values are 1-24\n");
		  camac_slot=8;
		  crate_slot=8;
		}/*end if camac_slot<=0*/
                printf("SLOT #  %d is being tested\n\n\n",camac_slot) ;
              }/*end if ncmd>=2*/
              else
                printf("Format is...ADC #   ie. ADC 1 <return>\n\n");
	    } /* else if strncmp(cmd_str[0],"ADC" */

            else if((strncmp(cmd_str[0],"CHANNEL",4)==0)||
             (strncmp(cmd_str[0],"channel",4)==0)){
              if(ncmd >= 2) {
                sscanf(cmd_str[1],"%d",&adc_channel) ;
		adc_number=(int)((adc_channel-1)/12)+1;/*get adc unit number*/
		single_channel=1;
		crate_slot=adc_number+ADC1-1;
		if(adc_channel<=0 || adc_channel > 120){
		  printf("valid values are 1-120\n");
		  adc_channel=1;
		  adc_number=1;
		  crate_slot=8;
		} /* end if adc_channel */
                printf("ADC channel %d is being tested\n\n\n",adc_channel) ;
             } /*end if ncmd >=2 */
              else
                printf("Format is...ADC #   ie. ADC 10 <return>\n\n\n");
	    } /* else if strncmp(cmd_str[0],"CHANNEL" */

            else if((strncmp(cmd_str[0],"TEST",3)==0)||
             (strncmp(cmd_str[0],"test",3)==0)){

	     if(test_adc==0){
		printf("TEST mode is on. Using internal 80ns gate.\n");
		test_adc=1;
	     }
	     else {
		printf("TEST mode is off. Using external adc gate.\n");
		test_adc=0;
	     }
	    } /* else if strncmp(cmd_str[0],"TEST" */

            else if((strncmp(cmd_str[0],"EXIT",3)==0)||
             (strncmp(cmd_str[0],"exit",3)==0)||
	     (strncmp(cmd_str[0],"QUIT",3)==0)||
	     (strncmp(cmd_str[0],"quit",3)==0)){
	      return;
            }

            else if((strncmp(cmd_str[0],"INIT",2)==0)||
             (strncmp(cmd_str[0],"init",2)==0)||
	     (strncmp(cmd_str[0],"CLEAR",3)==0)||
	     (strncmp(cmd_str[0],"clear",3)==0)){

		init_camac();
		printf("crates initialized!\n\n");
            }

            else if((strncmp(cmd_str[0],"STATUS",3)==0)||
	     (strncmp(cmd_str[0],"status",3)==0)){
	      if (test_adc==1)
		printf("TEST MODE:  ");
	      if (single_channel==0 && camac_slot==0){
		printf("CRATE=%d  SLOT=%d  ADC=%d  CHANNELS=all  EVENTS=%d\n\n",
			CRATE2,crate_slot,adc_number,events);
	      }/*end if*/
	      else if (single_channel>0 && camac_slot==0 ) {
		printf("CRATE=%d  SLOT=%d  ADC=%d  CHANNEL=%d EVENTS=%d\n\n",
			CRATE2,crate_slot,adc_number,adc_channel,events);
	      }
	      else if (single_channel>0 && camac_slot>0 ) {
		printf("CRATE=%d  SLOT=%d  CHANNEL=%d EVENTS=%d\n\n",
			CRATE2,crate_slot,adc_channel,events);
	      }
	      else {
		printf("CRATE=%d  SLOT=%d  CHANNELS=all  EVENTS=%d\n\n",
			CRATE2,crate_slot,events);
	      }
            }
            else {

	      printf("\n\n\nADC #		To look at all channels of an ADC\n");
	      printf("SLOT #		To test an ADC in an unused slot\n");
              printf("EVENT #		To set the number of events\n");
	      printf("CHANNEL #	To look at an individual channel\n");
	      printf("STATUS		To list PEDSET variables\n");
	      printf("TEST		Toggles between internal/external gates\n");
	      printf("<return>	To start collecting\n");
              printf("EXIT		To end program\n\n\n");

	    }

	  } /* end if ncmd >0 TOP IF IN LOOP*/
	  else /*if ncmd !>0*/
	    go=1; /*No command was entered...data taking is A OK!*/
	}while(!go);

	/*clear screen */
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	printf("COLLECTING DATA\n\n");

	/* Zero data array */
	for(i=0;i<12;i++)
	  data[i]=0;

	/* Get command address for the ADC entered. */

        c_cdreg(&extadc1_0,branch,CRATE2,crate_slot,0) ;
	/* Clear the adc then enable the lam */

        c_cfsa(F_CLEAR,extadc1_0,w_data,&q_resp,0,1) ;
        c_cfsa(F_ENABLELAM,extadc1_0,w_data,&q_resp,0,1) ;

        for(events_taken=0;events_taken<events;events_taken++){
	  x=0;
          do{
	     x++;
             c_cfsa(F_ENABLELAM,extadc1_0,w_data,&q_resp,0,1) ;
	     if(test_adc==1){	/* This uses the internal adc gate 80ns */
               c_cfsa(F_TEST_GATE,extadc1_0,w_data,&q_resp,0,1) ;
	     }
             c_cfsa(F_TESTLAM,extadc1_0,w_data,&q_resp,0,1) ;
/*printf("q_resp=%d X=%d\n",q_resp,x);*/
	  }while(!q_resp && x<100);

	  /* Read and add adc values to the data array */
          for(i=0;i<12;i++)
	  {
            q_resp=naf(CRATE2,crate_slot,i,2,&w_data);
	    data[i]+=w_data;
          }/*end for loop*/

	} /* end for(events...)*/

	if(single_channel){      
         printf("Events averaged=%d\n\n",events);
	 printf("channel %d	%.1f\n\n\n",adc_channel,
	       (float)data[(int)((adc_channel-1)%12)]/events);

	}
	else{
	  if (camac_slot==0){
            printf("ADC unit=%d    Events averaged=%d\n\n",adc_number,events);
            for(i=0;i<12;i++){
             printf("channel %d	%.1f\n",((adc_number-1)*12)+1+i,(float)data[i]/events);
	     /*printf("	raw=%d\n",data[i]);*/
            }/*end for i=0 i<12 */
	  }/*end if camac_slot==0*/
	  else{
            printf("SLOT =%d    Events averaged=%d\n\n",crate_slot,events);
            for(i=0;i<12;i++){
            printf("channel%d	%.1f\n",((adc_number-1)*12)+1+i,(float)data[i]/events);
	    /*printf("	raw=%d\n",data[i]);*/
            }/*end for i=0 i<12 */
	  }/*end else camac_slot==0*/
	  printf ("\n\n\n");
        }/*end else*/

     } /* End outer for loop */
}

Boolean_op close_evt_gate()
{
        int     q_resp ;
        int     w_data ;
        static Boolean_op       first_time = true ;
        static long int extsob1_1 ;
        void    c_cdreg() ;
        unsigned long   c_cssa() ;
              
	if(first_time) {
	  first_time = false ;	
	  c_cdreg(&extsob1_1,branch,CRATE1,SOBBOX1,1) ;
	}
	c_cssa(25,extsob1_1,&w_data,&q_resp) ;
	return((Boolean_op)q_resp) ;
}

Boolean_op open_evt_gate()
{
	int	q_resp ;
	int	w_data ;	
	static Boolean_op	first_time = true ;
	static long int 	extsob1_0 ;
	void    c_cdreg() ;
        unsigned long   c_cfsa() ;
          
	if(first_time) {
	  first_time = false ;
          printf("sob event gate: crate:%d, slot:%d\n",CRATE1,SOBBOX1) ;	
	  c_cdreg(&extsob1_0,branch,CRATE1,SOBBOX1,0) ; 
	}
	c_cfsa(25,extsob1_0,&w_data,&q_resp,0,1) ;
	return((Boolean_op)q_resp) ;
}

Boolean_op tst_status(status)
unsigned long status ;
{
	void LIB$SIGNAL() ;

	if(status & 1) {
	    return(true) ;
	}
	else {
	    LIB$SIGNAL(status) ;
	    return(false) ;
	}
}

read_purscal()
{
	int     q_resp ;
        int     data ;
        int     extadc[12] ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        unsigned long   status ;
        void    c_ctstat() ;
        Boolean_op clear_adcs(), open_evt_gate(), enable_adc_lams() ;
        void    quit() ;
        Boolean_op      tst_status() ;
        void    lib$wait() ;
        int     naf() ;
        int     q,n,a,scal_val ;
	int	answer ;
	float	count_time ;

	printf("Input count time (sec):") ;
	scanf("%f",&count_time) ;

	for(a=0;a<12;a++) {
           q=naf(CRATE1,PUR_SCAL,a,0,&scal_val) ;
	}
        printf("cleared scalar\n") ;

        for(;;) {
            lib$wait(count_time) ;
            for(a=0;a<12;a++) {
               q = naf(CRATE1,PUR_SCAL,a,2,&scal_val) ;
               printf("%d: %d   q=%d\n",a,scal_val,q) ;
            } /* end for a */
            printf("Enter 1 to continue, 0 to quit:") ;
            scanf("%d",&answer) ;
            if(answer == 0) quit() ;
        } /* end for ever */
} /* end read_purscal */

void calc_peds()	/* Calculate pedestals */
{
        int     i ;
        int     q_resp ;
        int     data ;
        int     extadc[12] ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        unsigned long   status ;
        void    c_ctstat() ;
        Boolean_op clear_adcs(), open_evt_gate(), enable_adc_lams() ;
        Boolean_op      tst_status(), close_evt_gate() ;
        void    lib$wait() ;
        int     naf() ;
        int     q,n,a,adc_val ;
        int     answer ;
        int	iadc ;
	double	adc_sum[NTUBES+20] ;
        double  adc_sqsum[NTUBES+20] ;
        long int  n_events ;
        long int  max_n_events ;
        double  mean, sigma ;
        double  sqrt() ;

        printf("Number of events to accumulate :") ;
        scanf("%ld",&max_n_events) ;

        clear_adcs() ;
        enable_adc_lams() ;    
        for(i=0;i<NTUBES;i++) {
            adc_sum[i] = adc_sqsum[i] = 0.0 ;
        }
        for(n_events=0;n_events<max_n_events;) {
            n_events++ ;
            clear_adcs() ;
            enable_adc_lams() ;
            open_evt_gate() ; 
            c_ctstat(&status) ;
            tst_status(status) ;
            awt_adc_lam() ;
            for(iadc=0,n=ADC1;n<=ADC10;n++) {
               for(a=0;a<12;a++,iadc++) {
                   q = naf(CRATE2,n,a,0,&adc_val) ;
                   if(q != 1) {
                       printf("Bad q value, abort\n") ;
                   }	
                   adc_sum[iadc] += (double)adc_val ;
                   adc_sqsum[iadc] += ((double)adc_val*(double)adc_val) ;
              } /* end for a */
            } /* end for n */
          } /* end for n_events */
          printf("\n") ;
          printf("channel number: mean,sigma)\n") ;
          for(iadc=0;iadc<NTUBES;iadc++) {
            mean = adc_sum[iadc]/(double)n_events ;
            sigma = sqrt(adc_sqsum[iadc]/(double)n_events - mean*mean) ;
            peds[iadc] = mean ;
            printf("Ped %d: %3.2lf, Pedvar: %3.2lf \t",iadc+1,mean,sigma) ;
            if((iadc+1)%2 == 0) {
               printf("\n") ;
            }
          }
          printf("\n") ;            
} /* end calc_peds */


void accum_adcs()	/* Accumulate adc means and variances */
{
        int     i ;
        int     q_resp ;
        int     data ;
        int     extadc[12] ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        unsigned long   status ;
        void    c_ctstat() ;
        Boolean_op clear_adcs(), open_evt_gate(), enable_adc_lams() ;
        Boolean_op      tst_status(), close_evt_gate() ;
        void    lib$wait() ;
        int     naf() ;
        int     q,n,a,adc_val ;
        int     answer ;
        int	iadc ;
	double	adc_sum[NTUBES+20] ;
        double  adc_sqsum[NTUBES+20] ;
        long int  n_events ;
        long int  max_n_events ;
        double  mean, sigma ;
        double  sqrt() ;

        printf("Number of events to accumulate :") ;
        scanf("%ld",&max_n_events) ;

	for(;;) {
          while(!close_evt_gate) ;
          printf("closed event gate\n") ;
          clear_adcs() ;
          printf("cleared adcs\n") ;
          enable_adc_lams() ;
          printf("enabled adcs\n") ;

          for(i=0;i<NTUBES;i++) {
            adc_sum[i] = adc_sqsum[i] = 0.0 ;
          }
          for(n_events=0;n_events<max_n_events;) {
            n_events++ ;
            clear_adcs() ;
            enable_adc_lams() ;
            for(i=0;i<1000;i++) ;
            while(!open_evt_gate()) ;
            c_ctstat(&status) ;
            tst_status(status) ;
            awt_adc_lam() ;

            for(i=0;i<10000;i++) {} ;
            for(iadc=0,n=ADC1;n<=ADC10;n++) {
               for(a=0;a<12;a++,iadc++) {
                   q = naf(CRATE2,n,a,0,&adc_val) ;
                   if(q != 1) {
                       printf("Bad q value, abort\n") ;
                       return ;
                   }	
                   adc_sum[iadc] += (double)adc_val ;
                   adc_sqsum[iadc] += ((double)adc_val*(double)adc_val) ;
              } /* end for a */
            } /* end for n */
          } /* end for n_events */
          printf("\n") ;
          printf("channel number: mean,sigma)\n") ;
          for(iadc=0;iadc<NTUBES;iadc++) {
            mean = adc_sum[iadc]/(double)n_events ;
            sigma = sqrt(adc_sqsum[iadc]/(double)n_events - mean*mean) ;
            printf("%d:	%3.2lf,	%3.2lf		\t",iadc+1,mean,sigma) ;
            if((iadc+1)%2 == 0) {
               printf("\n") ;
            }
          }
          printf("\n") ;            
          printf("Number of events to accumulate (-1 to quit) :" );
          scanf("%ld",&max_n_events) ;
          if(max_n_events == -1)
            return ;
        } /* end for ever */
} /* end accum_adcs */

#define MAXPED	100.0

void nitrogen_gains()	/* Accumulate adc means and variances */
{
        int     i,j ;
        int     q_resp ;
        int     data ;
        int     extadc[12] ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        unsigned long   status ;
        void    c_ctstat() ;
        Boolean_op clear_adcs(), open_evt_gate(), enable_adc_lams() ;
        Boolean_op      tst_status(), close_evt_gate() ;
        void    lib$wait() ;
        int     naf() ;
        int     q,n,a,adc_val ;
        int     answer ;
        int	iadc ;
	double	adc_sum[NTUBES+20] ;
        double  adc_sqsum[NTUBES+20] ;
        double  hvin[NTUBES+20] ;
        double  hvout[NTUBES+20] ;
        double	gains[NTUBES+20] ;
        double  adc_vals[NTUBES+20] ;
        double  gfact, expo1 ;
        double	vlimit ;
        long int nsum[NTUBES+20] ;
        long int  n_events ;
        long int  max_n_events ;
        double  mean, sigma ;
        double  sqrt() ;
        double  meangain ;
        FILE	*fp, *fopen() ;
        FILE	*fphvin, *fphvout ;
	char	fname_hvin[100], fname_hvout[100] ;
        int	ngood ;
         

	gfact = 1.0 ;
	expo1 = 0.148 ;
        vlimit = 1400.0 ;

        printf("Input HV filename :") ;
        scanf("%s",fname_hvin) ;
        printf("Output HV filename:") ;
        scanf("%s",fname_hvout) ;
        if((fphvin=fopen(fname_hvin,"r")) != NULL) {
          printf("Opened HV file: %s\n",fname_hvin) ;
          for(i=0;i<NTUBES;i+=10) {
            for(j=i;j<(i+10);j++) {
              fscanf(fphvin,"%lf",&(hvin[j])) ;
              printf("%4d ",(int)(hvin[j]+0.5)) ;
            }
            printf("\n") ;
          }
          fclose(fphvin) ;
        }

        printf("Number of events to accumulate :") ;
        scanf("%ld",&max_n_events) ;

	for(;;) {
          while(!close_evt_gate()) ;
          printf("closed event gate\n") ;
          clear_adcs() ;
          printf("cleared adcs\n") ;
          enable_adc_lams() ;
          printf("enabled adcs\n") ;
          
          for(i=0;i<NTUBES;i++) {
            adc_sum[i] = adc_sqsum[i] = 0.0 ;
            nsum[i] = 0 ;
          }
          for(n_events=0;n_events<max_n_events;) {
            n_events++ ;
            clear_adcs() ;
            enable_adc_lams() ;
            for(i=0;i<1000;i++) ;
            while(!open_evt_gate()) ;
            c_ctstat(&status) ;
            tst_status(status) ;
            awt_adc_lam() ;

            for(i=0;i<10000;i++) {} ;
            for(iadc=0,n=ADC1,ngood=0;n<=ADC10;n++) {
               for(a=0;a<12;a++,iadc++) {
                   q = naf(CRATE2,n,a,0,&adc_val) ;
                   if(q != 1) {
                       printf("Bad q value, abort\n") ;
                       return ;
                   }
                   adc_vals[iadc] = adc_val ;
                   if(adc_val > MAXPED) ngood++ ;
              } /* end for a */
            } /* end for n */
            printf("ngood: %d",ngood) ;
            if(ngood > 70) {
              printf("*\n") ;
              for(iadc=0;iadc<NTUBES;iadc++) {
                adc_sum[iadc] += (double)adc_vals[iadc] ;
                adc_sqsum[iadc] += 
                 ((double)adc_vals[iadc]*(double)adc_vals[iadc]) ;
                nsum[iadc]++ ;
              }
            }
            else {
              printf("\n") ;
            }
          } /* end for n_events */
         
          printf("\n") ;
          printf("channel number: mean,sigma)\n") ;
          for(iadc=0,meangain=0.0;iadc<NTUBES;iadc++) {
            if(nsum[iadc]>0) {
              mean = adc_sum[iadc]/(double)nsum[iadc] ;
              sigma = sqrt(adc_sqsum[iadc]/(double)nsum[iadc] - mean*mean) ;
            }
            else {
              mean = 0.0 ;
              sigma = 0.0 ;
            }
            printf("%d:	%3.2lf,	%3.2lf		\t",iadc+1,mean,sigma) ;
            if((iadc+1)%2 == 0) {
               printf("\n") ;
            }
            meangain += mean ;
          }
          meangain /= (double)NTUBES ;
          printf("\n") ; 
          if((fp=fopen("n2gains.out","w")) == NULL) {
            printf("Cannot open n2gains.out\n") ;
          }
          else {
            printf("Gain correction factors:\n") ;
            for(iadc=0;iadc<NTUBES;iadc+=12) {
               for(i=iadc;i<iadc+12;i++) {
                  if((nsum[i]>0)&&(adc_sum[i]>0)) {           
                    fprintf(fp,"%3.2lf ",
                     gains[i] = meangain/(adc_sum[i]/(double)nsum[i]) ) ;
                    printf("%3.2lf ",gains[i]) ;
                  }
                  else {
                    fprintf(fp,"0.00 ") ;
                    printf("0.00 ") ;
                    gains[i] = 1.0 ;
                  }
               }
               fprintf(fp,"\n") ; 
               printf("\n") ;
            }
            fclose(fp) ;
          }
          for(i=0;i<NTUBES;i++) {
            hvout[i] = hvin[i]*pow((gfact*gains[i]),expo1) ;
            if(hvout[i] > vlimit) hvout[i] = vlimit ;
          }          
          if((fphvout=fopen(fname_hvout,"w")) != NULL) {
            printf("Opened HV output file: %s\n",fname_hvout) ;
            for(i=0;i<NTUBES;i+=10) {
              for(j=i;j<(i+10);j++) {
                fprintf(fphvout,"%4d ",(int)(hvout[j]+0.5)) ;
                printf("%4d ",(int)(hvout[j]+0.5)) ;
              }
              fprintf(fphvout,"\n") ;
              printf("\n") ;
            }
            fclose(fphvout) ;
          }
          printf("Number of events to accumulate (-1 to quit) :" );
          scanf("%ld",&max_n_events) ;
          if(max_n_events == -1)
            return ;
        } /* end for ever */
} /* end accum_adcs */
  
show_adcs(pedsub)
int pedsub ;
{
	Evt_Buffer	event[NTUBES+5] ;
        int     q_resp ;
        int     data ;
        int     extadc[12] ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        unsigned long   status ;
        void    c_ctstat() ;
        Boolean_op clear_adcs(), open_evt_gate(), enable_adc_lams() ;
        Boolean_op      tst_status() ;
        void    lib$wait() ;
        int     naf() ;
        int     q,n,a,adc_val ;
        int     answer ;
	int	i ;
	int	iadc ;
        int     ievent ;

        clear_adcs() ;
        enable_adc_lams() ;
        answer=1 ;
        while(answer==1) {
            clear_adcs() ;
            enable_adc_lams() ;
            open_evt_gate() ;
            c_ctstat(&status) ;
            tst_status(status) ;
            awt_adc_lam() ;
            for(i=0;i<10000;i++) {}
            for(iadc=1,n=ADC1;n<=ADC10;n++) {
               for(a=0;a<12;a++) {
                   q = naf(CRATE2,n,a,0,&adc_val) ;
                   /* printf("      %d: %d,%d\n",a,adc_val,q) ; */
                   event[iadc++] = adc_val ;
               } /* end for a */
            } /* end for n */
            if(pedsub == 1) {
              for(iadc=0;iadc<NTUBES;iadc++) {
                ievent = event[iadc] ;
                ievent = 
                 ievent - (int)(peds[iadc]+0.5) ;
                if(ievent<0) event[iadc] = 0 ;
                else event[iadc] = ievent ;
              }
            } 
            rshow109(event) ;
            printf("Enter 1 to continue, 0 to quit:") ;
            scanf("%d",&answer) ;
       } /* end while */
} /* end show_adcs */
  
void test_adcs() 
{
	int	q_resp ;
	int	data ;
	int	extadc[12] ;
	void	c_cdreg() ;
	unsigned long	c_cfsa() ;
	unsigned long	status ;
	void	c_ctstat() ;
	Boolean_op clear_adcs(), open_evt_gate(), enable_adc_lams() ;
	Boolean_op	tst_status() ;
	void	lib$wait() ;
 	int	naf() ;
	int	q,n,a,adc_val ;
	int	answer ;

	clear_adcs() ;
	enable_adc_lams() ;

	for(;;) {
	    clear_adcs() ;
            enable_adc_lams() ;
	    printf("cleared adc lams\n") ;
	    open_evt_gate() ;
	    printf("opened event gate\n") ;
	    c_ctstat(&status) ;
	    tst_status(status) ;
	 printf("status=%d\nwaiting for lam\n",status);
	    awt_adc_lam() ;
	    printf("got adc lam\n") ;
	    lib$wait(0.010) ;
	    printf("waited 0.010 sec\n") ;
	    for(n=ADC1;n<=ADC10;n++) {
	       printf("adc #%d    n: data,q\n",n) ;
	       for(a=0;a<12;a++) {
	           q = naf(CRATE2,n,a,0,&adc_val) ;
	           printf("      %d: %d,%d\n",a,adc_val,q) ;
	       } /* end for a */
	    } /* end for n */
	    printf("Enter 1 to continue, 0 to quit:") ;
	    scanf("%d",&answer) ;
	    if(answer == 0) break ;
	} /* end for ever */
} /* end test_adcs */

void show_currents()
{
        Evt_Buffer      event[NTUBES+5] ;
        int	ievt ;
        int     q_resp ;
        int     data ;
        int     scaldat[65] ;
        int     extscal0[5] ;
        int     extscal1[5] ;
        int     extscal15[5] ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        void    c_ctstat() ;
        void    c_wait_time() ;
        unsigned long   status ;
        int     crate ;
        int     station ;
        float	timebase ;
        int     i,j,k ;
        int     dummy ;
        Boolean_op      tst_status() ;
        void    quit() ;
        unsigned long   c_waitef() ;
        int     efn ;
        char	answer[50] ;
  
        ievt = 0 ;

        timebase=1 ;
        crate=CRATE1 ;
        station=SCAL1 ;
        c_cdreg(&extscal0[0],branch,crate,station,0) ;
        c_cdreg(&extscal1[0],branch,crate,station,1) ;
        c_cdreg(&extscal15[0],branch,crate,station,15);
        station=SCAL2 ;
        c_cdreg(&extscal0[1],branch,crate,station,0) ;
        c_cdreg(&extscal1[1],branch,crate,station,1) ;
        c_cdreg(&extscal15[1],branch,crate,station,15);
        station=SCAL3 ;
        c_cdreg(&extscal0[2],branch,crate,station,0) ;
        c_cdreg(&extscal1[2],branch,crate,station,1) ;
        c_cdreg(&extscal15[2],branch,crate,station,15);
        station=SCAL4 ;
        c_cdreg(&extscal0[3],branch,crate,station,0) ;
        c_cdreg(&extscal1[3],branch,crate,station,1) ;
        c_cdreg(&extscal15[3],branch,crate,station,15);
        
        for(;;) {
          for(i=0;i<4;i++) {
            c_cfsa(26,extscal0[i],&data,&q_resp,0,1) ;	/* enable LAM */
            printf("q_resp for LAM enable: %d",q_resp) ;
            c_cfsa(26,extscal1[i],&data,&q_resp,0,1) ;	/* enable counting */
            printf("q_resp for counting enable: %d",q_resp) ;
            c_cfsa(9,extscal0[i],&data,&q_resp,0,1) ; /* clear scaler */
          }
          c_wait_time(1.0) ;
          printf("start counting for %f seconds...\n",timebase) ;
          for(i=0;i<4;i++) {
            c_cfsa(26,extscal1[i],&data,&q_resp,0,1) ; /* enable counting */
          }
          c_wait_time(timebase) ;
          for(i=0;i<4;i++) {
            c_cfsa(24,extscal1[i],&data,&q_resp,0,1) ; /* disable counting */
          }
          for(i=0;i<4;i++) {
            c_cfsa(8,extscal15[i],&data,&q_resp,0,1) ; /* test */
            c_cfsa(11,extscal0[i],&data,&q_resp,0,1) ; /* reset address */
            for(j=0;j<64;j++) {
              c_cfsa(0,extscal0[i],&(scaldat[j]),&q_resp,0,1) ;
            }
            for(j=0;j<64;j+=2) {
              /* event[ievt++] = scaldat[j]+65536*scaldat[j+1] ; */               
              event[ievt++] = scaldat[j] ;
              if(ievt>NTUBES) ievt = NTUBES ;
            } /* end for j */
          } /* end for i */
          printf("\n\nCURRENTS:\n") ;
          rshow109(event) ;
          printf("Enter 1 to continue, 0 to quit: ") ;
          scanf("%f",&timebase) ;
          if(timebase == 0.0) break ;
        }
}

void show_rates()
{
        Evt_Buffer      event[NTUBES+5] ;
        int	ievt ;
        int     q_resp ;
        int     data ;
        int     scaldat[65] ;
        int     extscal0[4] ;
        int     extscal1[4] ;
        int     extscal15[4] ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        void    c_ctstat() ;
        void    c_wait_time() ;
        unsigned long   status ;
        int     crate ;
        int     station ;
        float	timebase ;
        int     i,j,k ;
        int     dummy ;
        Boolean_op      tst_status() ;
        void    quit() ;
        unsigned long   c_waitef() ;
        int     efn ;
        char	answer[50] ;
  
        ievt = 0 ;

        printf("Enter time base: ") ;
        scanf("%f",&timebase) ;
        crate=CRATE1 ;
        station=SCAL5 ;
        c_cdreg(&extscal0[0],branch,crate,station,0) ;
        c_cdreg(&extscal1[0],branch,crate,station,1) ;
        c_cdreg(&extscal15[0],branch,crate,station,15);
        station=SCAL6 ;
        c_cdreg(&extscal0[1],branch,crate,station,0) ;
        c_cdreg(&extscal1[1],branch,crate,station,1) ;
        c_cdreg(&extscal15[1],branch,crate,station,15);
        station=SCAL7 ;
        c_cdreg(&extscal0[2],branch,crate,station,0) ;
        c_cdreg(&extscal1[2],branch,crate,station,1) ;
        c_cdreg(&extscal15[2],branch,crate,station,15);
        
        for(;;) {
          for(i=0;i<3;i++) {
            c_cfsa(26,extscal0[i],&data,&q_resp,0,1) ;	/* enable LAM */
            printf("q_resp for LAM enable: %d",q_resp) ;
            c_cfsa(26,extscal1[i],&data,&q_resp,0,1) ;	/* enable counting */
            printf("q_resp for counting enable: %d",q_resp) ;
            c_cfsa(9,extscal0[i],&data,&q_resp,0,1) ; /* clear scaler */
          }
          c_wait_time(1.0) ;
          printf("start counting for %f seconds...\n",timebase) ;
          for(i=0;i<3;i++) {
            c_cfsa(26,extscal1[i],&data,&q_resp,0,1) ; /* enable counting */
          }
          c_wait_time(timebase) ;
          for(i=0;i<3;i++) {
            c_cfsa(24,extscal1[i],&data,&q_resp,0,1) ; /* disable counting */
          }
          for(i=0;i<3;i++) {
            c_cfsa(8,extscal15[i],&data,&q_resp,0,1) ; /* test */
            c_cfsa(11,extscal0[i],&data,&q_resp,0,1) ; /* reset address */
            for(j=0;j<64;j++) {
              c_cfsa(0,extscal0[i],&(scaldat[j]),&q_resp,0,1) ;
            }
            for(j=0;j<64;j+=2) {
              /* event[ievt++] = scaldat[j]+65536*scaldat[j+1] ; */               
              event[ievt++] = scaldat[j] ;
            } /* end for j */
          } /* end for i */
          printf("\n\nSINGLES RATES:\n") ;
          rshow109(event) ;
          printf("Time base (sec) (-1 to quit): ") ;
          scanf("%f",&timebase) ;
          if(timebase < 0.0) break ;
        }
}

void test_scalers()
{
        int     q_resp ;
        int     data ;
        int     scaldat[65] ;
        int     extscal0 ;
        int     extscal1 ;
        int     extscal15 ;
        void    c_cdreg() ;
        unsigned long   c_cfsa() ;
        void    c_ctstat() ;
        void    c_wait_time() ;
        unsigned long   status ;
        int     crate ;
        int     station ;
        float	timebase ;
        int     i,j,k ;
        int     dummy ;
        Boolean_op      tst_status() ;
        void    quit() ;
        unsigned long   c_waitef() ;
        int     efn ;
        char	answer[50] ;

        printf("Scaler crate number: ") ;
        scanf("%d",&crate) ;
        printf("Scaler station number: ") ;
        scanf("%d",&station) ;
        printf("Time base (sec): ") ;
        scanf("%f",&timebase) ;
 
        c_cdreg(&extscal0,branch,crate,station,0) ;
        c_cdreg(&extscal1,branch,crate,station,1) ;
        c_cdreg(&extscal15,branch,crate,station,15);

        c_cfsa(26,extscal0,&data,&q_resp,0,1) ;	/* enable LAM */
        printf("q_resp for LAM enable: %d",q_resp) ;
        c_cfsa(26,extscal1,&data,&q_resp,0,1) ;	/* enable counting */
        printf("q_resp for counting enable: %d",q_resp) ;
        for(;;) {
          c_cfsa(9,extscal0,&data,&q_resp,0,1) ; /* clear scaler */
          c_wait_time(1.0) ;
          printf("start counting for %f seconds...\n",timebase) ;
          c_cfsa(26,extscal1,&data,&q_resp,0,1) ; /* enable counting */
          c_wait_time(timebase) ;
          c_cfsa(24,extscal1,&data,&q_resp,0,1) ; /* disable counting */
          c_cfsa(8,extscal15,&data,&q_resp,0,1) ; /* test */
          c_cfsa(11,extscal0,&data,&q_resp,0,1) ; /* reset address */
          for(i=0;i<64;i++) {
            c_cfsa(0,extscal0,&(scaldat[i]),&q_resp,0,1) ;
          }
          for(i=0;i<64;i+=2) {
            printf("%d: (%ld,%ld) %ld\n",i/2,scaldat[i],scaldat[i+1],
             scaldat[i]+65536*scaldat[i+1]) ;
          }
          printf("Time base (sec) (-1 to quit): ") ;
          scanf("%f",&timebase) ;
          if(timebase < 0.0) break ;
        }
}
	    
void test_sobs()
{
	int     q_resp ;
        int     data ;
        int 	extsob1[10] ;
        void	c_cdreg() ;
        unsigned long   c_cfsa() ;
	void	c_ctstat() ;
	void	lib$wait() ;
	unsigned long	status ;
	int	crate ;
	int	i,j,k ;
	int	dummy ;
	Boolean_op	tst_status() ;
	void	quit() ;
	unsigned long	c_waitef() ;
	int	efn ;

	printf("Enter crate number: ") ;
	scanf("%d",&crate) ;
	printf("crate number: %d\n",crate);
	for(i=0;i<8;i++) {
	    c_cdreg(&(extsob1[i]),branch,crate,SOBBOX1,i) ;
	    printf("gen ext for sob%d, output%d\n",crate,i) ;
	}

	for(k=0;k<1000;k++) {
	  printf("c\n") ;
	  for(i=0;i<8;i++) {
	    c_cfsa(25,extsob1[i],&data,&q_resp,0,1) ;
	    printf("pulsed sob%d output%d\n",crate,i) ;
	    /* do{
	      c_waitef(efn,&status) ;
	    } while((status & 1) != 1) ;
	    printf("wait status:%d\n",(int)status) ;
	    tst_status(status) ; */
	    c_ctstat(&status) ;
	    if(!tst_status(status)) {
	      printf("Bad status, hit a key to continue:") ;
	      scanf("%d",&dummy) ;
	    }
            printf("status:%d,q:%d\n",(int)status,(int)q_resp) ;
                                     
	    lib$wait(0.003) ;
	    /*  printf("waited for 0.001 sec\n") ;
	    printf("?") ;
	    scanf("%d",&dummy) ; */
          }
        }
}

void c_wait_time(delay)
float delay ;
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

/* void c_wait_time(delay)
float delay ;
{
	void	ftime() ;
	timeb_t	**ptime ;
	float	final_time, current_time ;

	ftime(*ptime) ;
        final_time = (float)(*ptime)->time
         + (float)(*ptime)->millitm/1000.0 + delay ;
        printf("final time: %f\n",final_time) ;
        do {
          ftime(*ptime) ;
          current_time = (float)(*ptime)->time +
            (float)(*ptime)->millitm/1000.0 ;
        } while(current_time < final_time) ;
} */
	
void acquire_data(fp,duration)
int	fp ;		
int	duration ;
{
	Evt_Buffer	evt[EVTSIZE+1] ;
	long int	i ;
	int		start_time, last_time ;
	Boolean_op	awt_trigger(),awt_adc_lam(),close_evt_gate() ;
	Boolean_op 	read_adcs(), read_scalers() ;
	void		write_evt(), rshow109() ;
	Boolean_op	clear_adcs(), open_evt_gate() ;
	Boolean_op	read_sat_clock(), read_evt_clock() ;
	time_t		time() ;
        int		dummy ;

	last_time = start_time = time(NULL) ;
	for(i=0L;last_time<(start_time+duration);i++) {
            printf("last_time: %f, start_time: %f, duration: %f\n",
              last_time,start_time,duration) ;
            open_evt_gate() ;
            if(!i%PED_INTERVAL) {
              printf("Pedistal event\n") ;
              gen_test_trigger() ;
              evt[E_CODE] = C_INJPED ;
              awt_adc_lam() ;
              printf("got adc lam\n") ;
              evt[E_EVTNO] = i ;
              printf("event number: %d\n",i) ;
              read_sat_clock(&evt) ;
              printf("read sat clock\n") ;
              read_evt_clock(&evt) ;
              printf("read evt clock\n") ;
              read_adcs(&evt) ;
              printf("read adcs\n") ;
              read_scalers(&evt) ;

            }
	    else if(!i%FRAME_INTERVAL) {
              printf("Frame event\n") ;
              evt[E_CODE] = C_FRAMEEVT ;
              evt[E_EVTNO] = i ;
              /* awt_scaler_lam() ; */
              printf("event number: %d\n",i) ;
              read_sat_clock(&evt) ;
              printf("read sat clock\n") ;
              read_evt_clock(&evt) ;
              printf("read evt clock\n") ;
              read_scalers(&evt) ;
              printf("read scalers\n") ;
              write_evt(fp,&evt) ;
              printf("        PMT CURRENTS\n") ;
              rshow109(&(evt[E_PMTS])) ;
              clear_scalers(&evt) ;
              printf("cleared scalers\n") ;
            }
            else {
              printf("Normal event\n") ;
              evt[E_CODE] = C_NORMEVT ;             
              /* printf("await trigger\n") ;
	      awt_trigger() ;
	      printf("got trigger\n") ; */
	      awt_adc_lam() ;
	      printf("got adc lam\n") ;
				/* Note: Event gate is automatically */
                                /* closed by NIM trigger logic       */
	      /* close_evt_gate() ; */
	      evt[E_EVTNO] = i ;
              printf("event number: %d\n",i) ;
	      read_sat_clock(&evt) ;
	      printf("read sat clock\n") ;
	      read_evt_clock(&evt) ;
              printf("read evt clock\n") ;
	      read_adcs(&evt) ;
              printf("read adcs\n") ;
	      write_evt(fp,&evt) ;
	      printf("        PMT SIGNALS\n") ;
	      rshow109(&(evt[E_PMTS])) ;	        
	      clear_adcs() ;
              printf("cleared ADCs\n") ;
            }
	    last_time = time(NULL) ;
            printf("last_time: %f\n",last_time) ;
            printf("input any number to continue:") ;
            scanf("%d",&dummy) ;
	} /* end for i */
} /* end acquire */

int getline(s,lim)
char    s[] ;
int     lim ;
{
        int     c, i ;

        for (i=0; i<lim-1 && (c=getchar())!=EOF && c!='\n'; ++i)
            s[i] = c ;
        if (c == EOF) {
            exit(0) ;
        }
        if (c == '\n') {
            s[i] = c ;
            ++i ;
        }
        s[i] = '\0' ;
        return i ;
}
		/* Simple command parser	*/
extract(oldline,cmd_str,ncmd,lit_str,nlit)
		/* Extract command strings and */
                /* literal strings from command line */
char    oldline[] ;
char    *cmd_str[] ;
int     *ncmd ;
char    *lit_str[] ;
int     *nlit ;
{
        char    line[LINELENGTH] ;
        char    newline[LINELENGTH] ;
        char    *tok ;
	char	*toknocomm ;

        *ncmd = 0 ;
        *nlit = 0 ;

        if(oldline[0] == '#') return ;
	if((toknocomm = strtok(oldline,"#\n\0"))==NULL)
	    return ;
 
        strncpy(line,toknocomm,sizeof(line)-1) ;
			/* Read up to comment           */
        if(strlen(line) == 0) return ;
        strncpy(newline,tok = strtok(line,"\""),sizeof(newline)-1) ;
        while(tok != NULL) {	/* Extract all of the           */
				/* literal strings		*/
            if(*nlit<NLITS) {
                tok = strtok(NULL,"\"") ;
                if((tok != NULL)&&(strlen(tok) != 0))
                    strncpy(lit_str[(*nlit)++],tok,(size_t)LITLEN) ;
                if(tok == NULL) break ;
            }
            else break ;

            if((tok = strtok(NULL,"\"")) == NULL) break ;
            else strcat(newline,tok) ;
        }
        tok=strtok(newline," \t,:") ;
	if((tok != NULL)&&(strlen(tok) != 0))
	strncpy(cmd_str[(*ncmd)++],tok,(size_t)CMDLEN) ;
        while(tok != NULL) {
            if(*ncmd<NCMDS) {
                tok = strtok(NULL," \t,:") ;
                if((tok != NULL)&&(strlen(tok) != 0))
                    strncpy(cmd_str[(*ncmd)++],tok,(size_t)CMDLEN) ;
            }
            else break ;
        }
}

void	display_commands()
{
/* This subroutine displays possible commands for the main routine */
   printf("\n\n\nCOMMANDS: \n");
   printf("calcpeds \n") ;
   printf("pedset \n") ;
/*   printf("start \n");
   printf("open [file]\n"); */
   printf("show [events,rates,currents,evtmped] \n");
/*   printf("read [purscal,adcs] \n"); */
   printf("test [sobs,aadcs,nitrogen,adcs,tdcs,scalers,evtgate]\n");
   printf("quit or exit\n\n\n");
}

void init_main()
{
	if(sizeof(unsigned int) != 4) {
	    printf("ERROR: Your machine has a different int size\n");
	    printf("than the VAXstation 4000-90 (i.e. not 4 bytes).\n");
	    printf("You must edit the data\n") ;
	    printf("acquisition program and recompile.  Sorry\n") ;
	    printf("for the inconvenience.\n") ;
	    exit(0) ;
	}
}

main()
{
	char    *cmd_str[NCMDS+1] ;
	char	*lit_str[NLITS+1] ;
	char	line[LINELENGTH] ;
	int	ncmd, nlit ;
	int	duration ;	
	int	fp ;
	FILE	*frun_number ;
	char	fname[100] ;
	char	default_fname[100] ;
	unsigned int	i ;
        int	j ;
	float	fduration ;
	void	acquire_data() ;
	void	init_camac() ;
	void	init_main() ;
	void	test_sobs() ;
	void	test_adcs() ;
	void	ped_set();
        void    accum_adcs() ;
        /* void    show_adcs() ; */
        void    show_rates() ;
        void    show_currents() ;
	void	display_commands();
        void    calc_peds() ;
	void	quit() ;

#ifdef TENM
        printf("SIMPLEACQ running in 10m configuration\n\n") ;
#endif
#ifdef ELEVENM
        printf("SIMPLEACQ running in 11m configuration\n\n") ;
#endif
	init_main() ;
	init_camac() ;

	duration = 0.500 ;
	
	for(i=0;i<NCMDS;i++) {
	  cmd_str[i]=(char *)calloc((size_t)CMDLEN,(size_t)sizeof(char));
        }
        for(i=0;i<NLITS;i++) {
          lit_str[i]=(char *)calloc((size_t)LITLEN,(size_t)sizeof(char));
        }

	init_camac() ;

	display_commands();
	for(;;) {
	  printf("* ") ;
	  if(getline(line,sizeof(line))<0) break ;
	  extract(line,cmd_str,&ncmd,lit_str,&nlit) ;
	  /* printf("line=%s\n ncmd=%d,nlit=%d\n",line,ncmd,nlit) ; */
	  if(ncmd > 0) {
	    if((strncmp(cmd_str[0],"DURATION",3)==0)||
	     (strncmp(cmd_str[0],"duration",3)==0)) {
              printf("DURATION\n") ;
	      if(ncmd >= 2) {
	        sscanf(cmd_str[1],"%f",&fduration) ;
		duration = (int)(fduration * 60.0 + 0.5) ;
                printf("duration: %f\n",duration) ;
	      }
	    } /* end if */
	    else if((strncmp(cmd_str[0],"START",4)==0)||
	     (strncmp(cmd_str[0],"start",4)==0)) {
                printf("START\n") ;
	        if(fp == -1) {
		 printf("ERROR: Ouput file has not been opened.\n") ;
		 printf("Use the OPEN command before starting a run.\n") ;
		}
		else {
	         acquire_data(fp,duration) ;
		}
	    } /* end else if */
	    else if((strncmp(cmd_str[0],"OPEN",2)==0)||
	     (strncmp(cmd_str[0],"open",2)==0)) {
	      if(ncmd >= 2) {
		sscanf(cmd_str[1],"%s",fname) ;
		if((fp = open(fname,O_RDWR,0))==-1) {
		  printf("Error opening %s\n",fname) ;
		}
	      }
 	      /* else {
	        printf("default fname: %s",default_fname) ;
	      } */
            } /* end else if */
	    else if((strncmp(cmd_str[0],"SHOW",4)==0)||
             (strncmp(cmd_str[0],"show",4)==0)) {
              if(ncmd > 1) {
                if(strncmp(cmd_str[1],"events",4)==0) {
                 printf("Single Event Display:\n") ;
                 show_adcs(0) ;
                 printf("done with single event display\n") ;
                }
                if(strncmp(cmd_str[1],"currents",4)==0) {
                 printf("PMT Current Display:\n") ;
                 show_currents() ;
                }
                if(strncmp(cmd_str[1],"rates",4)==0) {
                 printf("Singles Rate Display\n") ;
                 show_rates() ;
                }
                if(strncmp(cmd_str[1],"evtmped",4)==0) {
                 printf("Singles Rate Display (Peds subtracted)\n") ;
                 show_adcs(1) ;
                }                
              } /* end if ncmd */
            } /* end else if */
	    else if((strncmp(cmd_str[0],"READ",4)==0)||
             (strncmp(cmd_str[0],"read",4)==0)) {
              printf("READ\n") ;
              if(ncmd > 1) {
		if(strncmp(cmd_str[1],"purscal",3)==0) {
                 printf("READ PURDUE SCALER\n") ;
                 read_purscal() ;
                }
                else if(strncmp(cmd_str[1],"adcs",3)==0) {
                 printf("ADCS\n") ;
                 test_adcs() ;
                }
              } /* end if ncmd */
            } /* end else if READ */

            else if((strncmp(cmd_str[0],"TEST",4)==0)||
             (strncmp(cmd_str[0],"test",4)==0)) {
              printf("TEST\n") ;
              if(ncmd > 1) {
                if(strncmp(cmd_str[1],"sobs",3)==0) {
                 printf("SOBS\n") ;
                 test_sobs() ;
                }
                else if(strncmp(cmd_str[1],"aadcs",4)==0) {
                 printf("ACCUMULATE ADC MEANS AND VARIANCES\n") ;
                 accum_adcs() ;
                }
                else if(strncmp(cmd_str[1],"adcs",3)==0) {
                 printf("ADCS\n") ;
                 test_adcs() ;
                }
                else if(strncmp(cmd_str[1],"tdcs",3)==0) {
                }
                else if(strncmp(cmd_str[1],"scalers",3)==0) {
                 printf("SCALERS\n") ;
                 test_scalers() ;
                }
                else if(strncmp(cmd_str[1],"nitrogen",3)==0) {
                 printf("NITROGEN\n") ;
                 nitrogen_gains() ;
                }
                else if(strncmp(cmd_str[1],"evtgate",4)==0) {
                 printf("EVT GATE\n") ;
                 for(i=0;i<10000;i++) {
                   open_evt_gate() ;
                   for(j=0;j<10000;j++) ;
                 }
                }
              } /* end if ncmd */
            } /* end else if TEST */

            else if((strncmp(cmd_str[0],"PEDSET",4)==0)||
             (strncmp(cmd_str[0],"pedset",4)==0)) {
              ped_set() ;
            }

            else if((strncmp(cmd_str[0],"CALCPEDS",5)==0)||
             (strncmp(cmd_str[0],"calcpeds",5)==0)) {
              calc_peds() ;
            }

            else if((strncmp(cmd_str[0],"quit",4)==0)||
             (strncmp(cmd_str[0],"exit",4)==0)) {
              quit() ;
            }

            else if((strncmp(cmd_str[0],"help",2)==0)||
             (strncmp(cmd_str[0],"HELP",2)==0)) {
              display_commands() ;
            }
            else {
              printf("invalid command\n") ;
            }
	  } /* end if ncmd */
	  /* If no command was given assume help was needed. */
          display_commands() ;
	} /* end for */
	quit() ;
} /* end main */ 

void quit()
{
	unsigned long	ccfini();

	ccfini() ;
	printf("ACQ11M: Exiting normally\n") ;
	exit(0) ;
}

int naf(c,n,a,f,data)	/* Issue a single CAMAC command	*/
			/* 16-bit data version		*/
int	c ;	/* CAMAC crate number	*/
int	n ;	/* CAMAC module number	*/
int	a ;	/* Module subaddress	*/
int	f ;	/* Function code	*/
int	*data ;		/* Data			*/
{
	int	ext ;
	int	q_resp ;
	void	c_cdreg() ;
	unsigned long c_cfsa() ;

	c_cdreg(&ext, branch, c, n, a) ;
	c_cfsa(f,ext,data,&q_resp,0,1) ;
	return(q_resp) ;
}

