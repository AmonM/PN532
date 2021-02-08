#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

unsigned char getFW[] = {0x0, 0xFF, 0x02, 0xFE,  0xD4, 0x02, 0x2A, 0x00};
unsigned char handshake[] = {0x55, 0x55, 0x00, 0x00, 0x00};
unsigned char samConfig[] = {0x0, 0xFF, 0x05, 0xFB, 0xD4, 0x14, 0x01, 0x00, 0x01, 0x16, 0x0};
unsigned char longPreamble[] =
        "\x55\x55"
        "\x00\x00\x00\x00\x00\x00\x00\x00"
        "\x00\x00\x00\x00\x00\x00\x00\x00"
        "\x00\x00\x00\x00\x00\x00\x00\x00"
        "\x00\xFF\x02\xFE\xD4\x02\x2A\x00"
       ;
unsigned char readPassiveTargetID[] = {0x00, 0xFF, 0x04, 0xFC, 0xD4, 0x4A, 0x01, 0x00, 0xE1, 0x00};
unsigned char authCard[] = { 0x00, 0xFF, 0x0F, 0xF1, 0xD4, 0x40, 0x01, 0x60, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEA, 0xED, 0x72, 0x85, 0xC1, 0x0};
unsigned char ack[] = "\x00\x00\xFF\x00\xFF\x00";

struct message {
        unsigned char buf[255];
        unsigned char len;
};

struct message sendCMD(int fd, unsigned char* command, int cmdLen){
    unsigned char buf [100];
    struct message sporocilo;

    write(fd, command, cmdLen);
	  usleep ((cmdLen + 25) * 100);             // sleep enough to transmit the 7 plus

        int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read

        if(n <= 6){
					sporocilo.len = 0;
					return sporocilo;
        }

        for(int i = 0; i < 6; i++){
          if(ack[i] != buf[i]){
            sporocilo.len = 0;
            sporocilo.buf[0] = '\0';
            return sporocilo;
          }
        }

        for(int i = 6; i < n; i++){

                sporocilo.buf[i] = buf[i];

                if(i<n-1){
                        //printf("0x%02X ",buf[i]);
                }else{
                       // printf("0x%02X\n", buf[i]);
                }
        }

        sporocilo.len = n-6;       
        return sporocilo;
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 2;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        /*if (tcsetattr (fd, TCSANOW, &tty) != 0)
                error_message ("error %d setting term attributes", errno);*/
}

void detectCards(int fd){
  struct message sporocilo;
  sporocilo = sendCMD(fd, readPassiveTargetID, sizeof readPassiveTargetID);
  if(sporocilo.len >= 19){
    unsigned char messageLen = sporocilo.buf[9];
    unsigned char UUIDlen = sporocilo.buf[18];
    printf("Card ID: ");
    for(int i = 18; i < 18+UUIDlen; i++){
      if(i<17+UUIDlen)
        printf("0x%02X ",sporocilo.buf[i]);
      else{
        printf("0x%02X\n",sporocilo.buf[i]);
      }
    }
  }
}

int main(){
    fprintf(stdout, "HANDSHAKE!\n");
    struct message sporocilo;
    char *portname = "/dev/ttyUSB0";
            
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
			//error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
			return 1;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 1);                // set blocking

    sporocilo = sendCMD(fd, longPreamble, sizeof longPreamble);
    usleep(1000);
    sporocilo = sendCMD(fd, samConfig, sizeof samConfig);
    
    while(1){
      usleep(100000);
      detectCards(fd);
    }    
    usleep(1000000);
    printf("\n");
    sporocilo = sendCMD(fd, authCard, sizeof authCard);
    

    //sendCMD(fd, rPin, sizeof rPin);

    return 0;
}
