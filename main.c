#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#define ARGU_FILENAME_OFFSET 1
#define PROPER_ARGU_CNT 2
#define BAUDRATE 115200
#define STOP_BIT 1
#define DATA_LEN 8
#define IS_PARITY 0
#define BUFFER_SIZE 512
#define LEN_OFFSET 1
#define PID_OFFSET 2
#define DATA_OFFSET 3
#define CHECKSUM_OFFSET 4
#define HEADER 0x23
#define PID 0x01
#define LEN_BYTE 0x22 // higher 4 bits are protocol, which is CAN_ISO-15031, and the lower 4 bits are (PID + LEN)

//#define DEBUG

volatile sig_atomic_t sig_flag = 0;
uint8_t buffer[BUFFER_SIZE] = {0};

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,};

int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200, 300,};

typedef enum {
	FILE_ERR = -5,
	ARGUS_ERR,
	SERIAL_SET_ERR,
	INDEX_ERR
} err_type;

void quit_program_cb(int sig) 
{
	sig_flag = 1;
}

int open_serial(char* dev)
{
	return open(dev, O_RDWR);
}

int set_speed(int fd, int speed)
{
	int i;
	int status;
	struct termios Opt;
	tcgetattr(fd, &Opt);

  	for ( i= 0; i < sizeof(speed_arr) / sizeof(int); i++) {
		if (speed == name_arr[i]) {
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
	        	status = tcsetattr(fd, TCSANOW, &Opt);

		      if (status != 0) {
		        	perror("tcsetattr fd1a");
				return SERIAL_SET_ERR;
			}

	        	return 0;
        	}
		tcflush(fd,TCIOFLUSH);
	}
	
	return SERIAL_SET_ERR;
}

int set_parity(int fd,int databits,int stopbits,int parity)
{

      struct termios options;

      if  ( tcgetattr( fd,&options)  !=  0) {
            perror("Setting serial error");
            return SERIAL_SET_ERR;
      }

      options.c_cflag &= ~CSIZE;

      switch (databits) {
            case 7:
                  options.c_cflag |= CS7;
                  break;
            case 8:
                  options.c_cflag |= CS8;
                  break;
            default:
                  fprintf(stderr,"Unsupported data size\n");
                  return SERIAL_SET_ERR;
      }

      switch (parity) {
            case 'n':
            case 'N':
                  options.c_cflag &= ~PARENB;   /* Clear parity enable */
                  options.c_iflag &= ~INPCK;     /* Enable parity checking */
                  break;
            case 'o':
            case 'O':
                  options.c_cflag |= (PARODD | PARENB);
                  options.c_iflag |= INPCK;
                  break;
            case 'e':
            case 'E':
                  options.c_cflag |= PARENB;     /* Enable parity */
                  options.c_cflag &= ~PARODD;
                  options.c_iflag |= INPCK;       /* Disnable parity checking */
                  break;
            case 'S':
            case 's':
                  options.c_cflag &= ~PARENB;
                  options.c_cflag &= ~CSTOPB;
                  break;
            default:
                  fprintf(stderr,"Unsupported parity\n");
                  return SERIAL_SET_ERR;
      }

      switch (stopbits) {
            case 1:
                  options.c_cflag &= ~CSTOPB;
                  break;
            case 2:
                  options.c_cflag |= CSTOPB;
            	  break;
            default:
                  fprintf(stderr,"Unsupported stop bits\n");
                  return SERIAL_SET_ERR;
      }

      /* Set input parity option */
      if (parity != 'n')
            options.c_iflag |= INPCK;

      options.c_cc[VTIME] = 15; // 15 seconds

      options.c_cc[VMIN] = 64;

      options.c_lflag = 0;
      //cfmakeraw(&options); this func causes options.c_cc[VMIN] = 0, which then lead to always has "no valid data" at startup
      tcflush(fd,TCIFLUSH); /* Update the options and do it *NOW* */

      if (tcsetattr(fd,TCSANOW,&options) != 0) {
            perror("Setting serial error");
            return SERIAL_SET_ERR;

      }

      return 0;
 }

int serial_conf(int fd)
{
	if (set_speed(fd, BAUDRATE) == SERIAL_SET_ERR)
            return SERIAL_SET_ERR;
	if (set_parity(fd, DATA_LEN, STOP_BIT, (IS_PARITY ? 'Y' : 'N')) == SERIAL_SET_ERR)
            return SERIAL_SET_ERR;

      return 0;
}

int16_t find_speed_data_pattern(uint16_t size)
{
#ifdef DEBUG
	puts("debugging");
	return buffer[2]; // arbitrary byte
#endif
	uint16_t base = 0, correctness = 0;
	
	size -= 1; // for safe traverse the array

	while (base < size) {
		if ((size - base) < 5) // no pattern matched at the end of the buffer
			return INDEX_ERR;
		if (buffer[base] == HEADER) { // header matched
varifying:		correctness++;
			switch (correctness) {
				case 1:
					if (buffer[base + LEN_OFFSET] == LEN_BYTE)
						goto varifying; // LEN matched
					else
						goto start_over;
				case 2:
					if (buffer[base + PID_OFFSET] == PID)
						goto varifying; // PID matched
					else
						goto start_over;
				case 3:
					if (buffer[base + CHECKSUM_OFFSET] \
					== ((PID + buffer[base + DATA_OFFSET]) % 256))
						return buffer[base + DATA_OFFSET]; // pass, return speed_data
					else
						goto start_over;
			}
		}
		else {
start_over:		correctness = 0;
			base++;
		}
	}
	return INDEX_ERR;
}

int main(int argc, char** argv)
{
	if (argc != PROPER_ARGU_CNT) {
		puts("Too many or too few arguments!");
		puts("Should be `./executable {FILENAME}`, whereas should have 2 args.");
		exit(ARGUS_ERR);
	}

	int fd_serial;

	fd_serial = open_serial(*(argv + ARGU_FILENAME_OFFSET));
      
	if (fd_serial == -1) {
		puts("Device not exist or currently not available!");
		puts("Do you forget to execute the program with `sudo` ?");
		exit(FILE_ERR);
	}

	if (serial_conf(fd_serial) == SERIAL_SET_ERR) {
		puts("Serial port configuration failed!");
		exit(SERIAL_SET_ERR);
	}

	signal(SIGINT, quit_program_cb);
	
	uint32_t recv_cnt;
	int16_t speed_data;
	while ((recv_cnt = read(fd_serial, buffer, BUFFER_SIZE)) > 0) {
		printf("%d\n", recv_cnt);
		speed_data = find_speed_data_pattern(recv_cnt);
		speed_data == INDEX_ERR ? puts("No valid data."): printf("%d km/h\n", speed_data);
		sleep(1);
		/* ctrl+c, program ending handler */
		if (sig_flag) {
			printf("\nExit status: %d\n", close(fd_serial));
			puts("Terminated by user.");
			exit(0);
		}
	}

	puts("Unexpected error occured!");
	exit(-1);
}
