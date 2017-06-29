#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static const int speed_map_arr[][2] = {
    {230400, B230400}, {115200, B115200}, {57600, B57600}, {38400, B38400},
    {19200, B19200},   {9600, B9600},     {4800, B4800},   {2400, B2400},
    {1800, B1800},     {1200, B1200},     {600, B600},     {300, B300},
    {200, B200},       {150, B150},       {134, B134},     {110, B110},
    {75, B75},         {50, B50},         {0, B0},
};

static int getSpeed(int baudRate, speed_t *speed) {
  unsigned i, arr_size = sizeof(speed_map_arr) / sizeof(speed_map_arr[0]) - 1;
  for (i = 0; i < arr_size; i++) {
    if (baudRate == speed_map_arr[i][0]) {
      *speed = (speed_t)speed_map_arr[i][1];
      return (0);
    }
  }
  return (-1);
}

int setupPort(int fd, int baudRate, int dataBits, int stopBits, int parity) {
  speed_t speed;
  struct termios options;

  if (fd == -1) {
    fprintf(stderr, "Invalid file descriptor, is the serial port open?");
    return (-1);
  }

  if (getSpeed(baudRate, &speed) != 0) {
    fprintf(stderr, "Unsupported baudRate %d\n", baudRate);
    return (-1);
  }

  /* Get current serial port settings */
  if (tcgetattr(fd, &options) != 0) {
    perror("tcgetattr(fd, &options) failed");
    return (-1);
  }

  /* Setting raw mode / no echo / binary */
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);

  options.c_oflag &= ~(OPOST | ONLCR | OCRNL);
#ifdef OLCUC
  options.c_oflag &= ~OLCUC;
#endif

  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
  options.c_iflag &= ~IUCLC;
#endif
#ifdef PARMRK
  options.c_iflag &= ~PARMRK;
#endif

/* Setting the baud rate */
#ifdef __USE_MISC
  cfsetspeed(&options, speed);
#else
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
#endif

  /* Setting the data bits */
  options.c_cflag &= ~CSIZE; /* Mask the character size bits */
  switch (dataBits) {
  case 5:
    options.c_cflag |= CS5; /* Select 5 data bits */
    break;
  case 6:
    options.c_cflag |= CS6; /* Select 6 data bits */
    break;
  case 7:
    options.c_cflag |= CS7; /* Select 7 data bits */
    break;
  case 8:
    options.c_cflag |= CS8; /* Select 8 data bits */
    break;
  default:
    fprintf(stderr, "Unsupported data size %d\n", dataBits);
    return (-1);
  }

  /* Setting the stop bits */
  switch (stopBits) {
  case 1:
    options.c_cflag &= ~CSTOPB; /* 1 stop bit */
    break;
  case 2:
    options.c_cflag |= CSTOPB; /* 2 stop bits */
    break;
  default:
    fprintf(stderr, "Unsupported stop bits %d\n", stopBits);
    return (-1);
  }

  /* Setting parity checking */
  options.c_iflag &= ~(INPCK | ISTRIP);
  switch (parity) {
  case 0: /* No parity */
    options.c_cflag &= ~(PARENB | PARODD);
    break;
  case 1: /* Odd parity */
    options.c_cflag |= (PARENB | PARODD);
    break;
  case 2: /* Even parity */
    options.c_cflag &= ~(PARODD);
    options.c_cflag |= (PARENB);
    break;
#ifdef CMSPAR
  case 3: /* Mark parity */
    options.c_cflag |= (PARENB | CMSPAR | PARODD);
    break;
  case 4: /* Space parity */
    options.c_cflag |= (PARENB | CMSPAR);
    options.c_cflag &= ~(PARODD);
    break;
#else
  case 3:
  case 4:
#endif
  default: /* invalid parity */
    fprintf(stderr, "Unsupported parity %d\n", parity);
    return (-1);
  }

  /* Software flow control is disabled */
  options.c_iflag &= ~(IXON | IXOFF | IXANY);

  /* 15 seconds timeout */
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 150;

  tcflush(fd, TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    perror("tcsetattr(fd, TCSANOW, &options) failed");
    return (-1);
  }

  return (0);
}
