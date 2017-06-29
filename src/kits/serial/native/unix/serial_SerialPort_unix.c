#include "sedona.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>

#define PORT_NAME_FORMAT "/dev/ttymxc%d"
#define HANDLE_ARRAY_LEN 128

//#define DUMP_SERIAL_DATA 1

typedef struct _SerialData {
  char name[32];
  int hFile;
} SerialData;

int setupPort(int fd, int baudRate, int dataBits, int stopBits, int parity);

int writeBytes(SerialData *pData, uint8_t *pu8Buf, int32_t nbytes);
int readBytes(SerialData *pData, uint8_t *pu8Buf, int32_t nbytes);

void printBytes(char *msgp, unsigned length);

SerialData *pSd[HANDLE_ARRAY_LEN];

Cell errCell = {-2};

// Set the serial parameters for the port.  If any params are bad, do
//  nothing and return -1, otherwise return 0.
//  int SerialPort doInit(int port, int baud, int datab, int stopb, int par, int
//  rts)
Cell serial_SerialPort_doInit(SedonaVM *vm, Cell *params) {
  SerialData *pData;
  int32_t portNum = params[1].ival;
  int32_t baudRate = params[2].ival;
  int32_t dataBits = params[3].ival;
  int32_t stopBits = params[4].ival;
  int32_t parity = params[5].ival;
  int32_t rtsLevel = params[6].ival;

  //
  // Initialize physical port here
  //
  if (portNum < 0 || portNum >= HANDLE_ARRAY_LEN) {
    printf("Invalid port %d\n", portNum);
    return negOneCell;
  }

  pData = (SerialData *)malloc(sizeof(SerialData));
  memset(pData, 0, sizeof(SerialData));

  sprintf(pData->name, PORT_NAME_FORMAT, portNum);
  pData->hFile = open(pData->name, O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
  if (pData->hFile == -1) {
    printf("Error: Cannot open serial port %s, err=%d\n", pData->name, errno);
    return negOneCell;
  }

  pSd[portNum] = pData;

  setupPort(pData->hFile, baudRate, dataBits, stopBits, parity);

  // Return zero if nothing went wrong
  return zeroCell;
}

// Shut down the serial port.  Return 0 if successful.
// int SerialPort.doClose(int port)
Cell serial_SerialPort_doClose(SedonaVM *vm, Cell *params) {
  int32_t portNum = params[1].ival;
  SerialData *pData = pSd[portNum];
  //
  // Shut down physical port here
  //
  printf("SerialPort.doClose %s on platform 'unix'.\n", pData->name);

  close(pData->hFile);
  free(pData);
  pSd[portNum] = NULL;
  return zeroCell;
}

// Read one byte from port.  Return byte value, or -1 if no byte was
// available.  (non-blocking)
// int  SerialPort.doRead(int port)
Cell serial_SerialPort_doRead(SedonaVM *vm, Cell *params) {
  uint8_t ch;
  int32_t bytesRead;
  Cell ret;
  int32_t portNum = params[1].ival;

  bytesRead = readBytes(pSd[portNum], &ch, 1);

  if (bytesRead < 0)
    return errCell;

  if (bytesRead != 1)
    return negOneCell;

  ret.ival = ch;
  return ret;
}

// Write one byte to port.  Return -1 if any error, or 0 if successful.
// int  SerialPort.doWrite(int port, int c)
Cell serial_SerialPort_doWrite(SedonaVM *vm, Cell *params) {
  int32_t portNum = params[1].ival;
  uint8_t ch = (uint8_t)params[2].ival;
  int32_t bytesWritten;

  bytesWritten = writeBytes(pSd[portNum], &ch, 1);
  if (bytesWritten != 1)
    return negOneCell;

  return zeroCell;
}

// Read up to n bytes from port into array y.  Return number of bytes
// read, or -1 if an error occurred.  (non-blocking)
// int  SerialPort.doReadBytes(int port, byte[] y, int off, int len)
Cell serial_SerialPort_doReadBytes(SedonaVM *vm, Cell *params) {
  Cell ret;
  int32_t portNum = params[1].ival;
  uint8_t *pu8Buf = params[2].aval;
  int32_t off = params[3].ival;
  int32_t nbytes = params[4].ival;

  int32_t bytesRead;

  pu8Buf = pu8Buf + off;

  bytesRead = readBytes(pSd[portNum], pu8Buf, nbytes);
#ifdef DUMP_SERIAL_DATA
  if (bytesRead > 0) {
    printf("Rx[%d]:\t", bytesRead);
    printBytes(pu8Buf, bytesRead);
  }
#endif

  ret.ival = bytesRead;
  return ret;
}

// Write up to n bytes to port from array y.  Return number of bytes
// written, or -1 if an error occurred.
// int  SerialPort.doWriteBytes(int port, byte[] y, int off, int len)
Cell serial_SerialPort_doWriteBytes(SedonaVM *vm, Cell *params) {
  Cell ret;
  int32_t portNum = params[1].ival;
  uint8_t *pu8Buf = params[2].aval;
  int32_t off = params[3].ival;
  int32_t nbytes = params[4].ival;

  int32_t bytesWritten;

  pu8Buf = pu8Buf + off;

  bytesWritten = writeBytes(pSd[portNum], pu8Buf, nbytes);
#ifdef DUMP_SERIAL_DATA
  if (bytesWritten > 0) {
    printf("Tx[%d]:\t", bytesWritten);
    printBytes(pu8Buf, nbytes);
  }
#endif

  if (bytesWritten == -1)
    return negOneCell;

  ret.ival = bytesWritten;
  return ret;
}

// return number of bytes read
int readBytes(SerialData *pData, uint8_t *pu8Buf, int32_t nbytes) {
  int32_t bytesRead;

  bytesRead = read(pData->hFile, pu8Buf, nbytes);
  if (bytesRead == -1 && errno == EAGAIN)
    bytesRead = 0;

  return bytesRead;
}

// return number of bytes written -
int writeBytes(SerialData *pData, uint8_t *pu8Buf, int32_t nbytes) {
  int32_t bytesWritten;

  bytesWritten = write(pData->hFile, pu8Buf, nbytes);

  return bytesWritten;
}

void printBytes(char *msgp, unsigned length) {
  unsigned i;

  for (i = 0; i < length; i++) {
    printf("%2.2X ", msgp[i] & 0x0ff);
  }
  printf("\n");
}
