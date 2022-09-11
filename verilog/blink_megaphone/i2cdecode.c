#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>

unsigned char scl_char=0;
unsigned char sda_char=0;

int i2c_state=0;

int main(int argc,char **argv)
{
  if (argc!=2) {
    fprintf(stderr,"usage: i2cdecode <file.vcd>\n");
    exit(-1);
  }

  FILE *f=fopen(argv[1],"r");
  if (!f) {
    fprintf(stderr,"ERROR: Could not read from '%s'\n",argv[1]);
    exit(-1);
  }

  long long timestamp=0;
  int scl_val=0;
  int sda_val=0;
  int scl_last=0;
  int sda_last=0;

  int i2c_bit=0;
  int i2c_byte=0;
  
  char line[1024];
  line[0]=0; fgets(line,1024,f);
  while(line[0]) {
    int n;
    char c;
    char sig[1024];
    if (sscanf(line,"$scope module %s $end",sig)==1) {
      fprintf(stderr,"INFO: Found module scope '%s'\n",sig);
    }
    if (sscanf(line,"$var wire 1 %c %s $end%n",&c,sig,&n)==2) {
      if (!strcmp(sig,"scl")) {
	scl_char=c;
	fprintf(stderr,"INFO: SCL signal is indicated by %c\n",scl_char);
      }
      if (!strcmp(sig,"sda")) {
	sda_char=c;
	fprintf(stderr,"INFO: SDA signal is indicated by %c\n",sda_char);
      }      
    }

    long long nexttimestamp;
    if (sscanf(line,"#%lld",&nexttimestamp)==1) {
      //      fprintf(stderr,"INFO: SCL=%c, SDA=%c @ %lld\n",scl_val,sda_val,timestamp);

#define IDLE 0
#define START 1
#define XFER 2
      switch(i2c_state) {
      case IDLE:
	if (scl_val=='1'&&sda_val=='0'&&sda_last=='1') {
	  fprintf(stderr,"DEBUG: %16lld I2C START\n",timestamp);
	  i2c_state=START;
	}
	break;
      case START:
	if (scl_val=='0'&&sda_val=='0'&&scl_last=='1') {
	  // Clock gone low during start: Now we begin
	  fprintf(stderr,"DEBUG: %16lld I2C START CLOCK LOW\n",timestamp);
	  i2c_state=XFER;
	  i2c_bit=0;
	  i2c_byte=0;
	}
	break;
      case XFER:
	if (scl_val=='1'&&scl_last=='1'&&sda_val=='0'&&sda_last=='1') {
	  // RE-START detected
	  fprintf(stderr,"DEBUG: %16lld I2C RE-START\n",timestamp);
	  i2c_state=START;
	}
	if (scl_val=='1'&&scl_last=='1'&&sda_val=='1'&&sda_last=='0') {
	  // STOP detected
	  fprintf(stderr,"DEBUG: %16lld I2C STOP\n",timestamp);
	  i2c_state=IDLE;
	}
	if (scl_val=='1'&&scl_last=='0') {
	  // Clock gone high, latch bit
	  i2c_byte=i2c_byte<<1;
	  i2c_byte|=(sda_val&1);
	  i2c_bit++;
	  if (i2c_bit==9) {
	    // End of byte
	    fprintf(stderr,"DEBUG: %16lld I2C byte = $%02x (nack=%d)\n",
		    timestamp,i2c_byte>>1,i2c_byte&1);
	    i2c_state=XFER;
	    i2c_bit=0;
	    i2c_byte=0;
	  }
	}
      }

      sda_last=sda_val;
      scl_last=scl_val;
      
      timestamp=nexttimestamp;
    }
    if (line[1]==scl_char&&line[2]<' ') {
      scl_val=line[0];
    }
    if (line[1]==sda_char&&line[2]<' ') {
      sda_val=line[0];
    }

    line[0]=0; fgets(line,1024,f);
  }
  
}
