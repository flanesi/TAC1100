#ifdef __cplusplus
extern "C" {
#endif

/*
 * TAC1100: ModBus RTU client to read TAIYE TAC1100 series smart mini power meter registers
 *
 * Copyright (C) 2026 Flavio Anesi <www.flanesi.it>
 *
 * Partially code from sdm120c ModBus Mini Smart Meter by Gianfranco Di Prinzio <gianfrdp@inwind.it>
 * Locking code partially from aurora by Curtronis.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

// Enable checks for inter-lock problems debug

#include <sys/types.h>
#include <sys/file.h>
#include <sys/time.h>

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <getopt.h>
#include <syslog.h>


#include <modbus-version.h>
#include <modbus.h>

#define DEFAULT_RATE 9600

// ====================================
// TAC1100 MODBUS REGISTER ADDRESSES
// ====================================

#define VOLTAGE   0x0000    // Voltage (Float, V)
#define CURRENT   0x0006    // Current (Float, A)
#define POWER     0x000C    // Active power (Float, W)
#define APOWER    0x0012    // Reactive power (Float, var)
#define RAPOWER   0x0018    // Apparent power (Float, VA)
#define PFACTOR   0x001E    // Power factor (Float)
#define PANGLE    0x0024    // Phase angle (Float, Degrees)
#define FREQUENCY 0x0030    // Frequency (Float, Hz)
#define IAENERGY  0x0500    // Total import active energy (Float, kWh)
#define EAENERGY  0x0502    // Total export active energy (Float, kWh)
#define TAENERGY  0x0504    // Total active energy (Float, kWh)
#define IRAENERGY 0x0508    // Total import reactive energy (Float, kvarh)
#define ERAENERGY 0x050A    // Total export reactive energy (Float, kvarh)
#define TRENERGY  0x050C    // Total reactive energy (Float, kvarh)

// ====================================
// TAC1100 MODBUS REGISTER ADDRESSES - WRITE
// ====================================

#define KPPA           0x5000   // Key Parameter Programming Authorization (Password per abilitare programmazione)
#define DEMAND_PERIOD  0x5002   // Demand period (0-60 minuti, default 60)
#define SLIDE_TIME     0x5003   // Slide time (1 to Demand Period-1, default 1)
#define DEVICE_ID      0x5005   // Modbus address (1-247, default 1) - R nel protocollo ma scrivibile via pulsanti/modbus
#define BAUD_RATE      0x5006   // Network Baud Rate (0=1200, 1=2400, 2=4800, 3=9600, 4=19200) - R nel protocollo ma scrivibile
#define NPARSTOP       0x5007   // Parity and stop bit (0=N1, 1=E1, 2=O1, 3=N2) - R nel protocollo ma scrivibile
#define PASSWORD       0x5008   // Password (default 0000, richiede KPPA)
#define TIME_DISP      0x5018   // Automatic Scroll Display Time (0-60 secondi, 0=stop scroll, default 0)
#define BACKLIT_TIME   0x5019   // Backlit time (0-120 o 255 minuti, 0=always on, 255=always off, default 60)
#define SYSTEM_TIME    0x501A   // System time (BCD format: Year-Month-Date-Week-Hour-Minute-Second)
#define TARIFF         0x501E   // Tariff configuration (BCD format: Tariff number-Min-Hour)
#define RESET_HIST     0x5600   // Reset historical data (0=reset max demand, 8=reset monthly, 9=reset daily) - SOLO SCRITTURA
#define METER_CODE     0x5601   // Meter code - SOLA LETTURA
#define SERIAL_NUM     0x5602   // Serial number - SOLA LETTURA (4 bytes)
#define SW_VERSION     0x5604   // Software version - SOLA LETTURA
#define HW_VERSION     0x5605   // Hardware version - SOLA LETTURA
#define DISP_VERSION   0x5606   // Display version - SOLA LETTURA
#define FAULT_CODE     0x5607   // Fault code (0=No fault, 1=Battery low voltage) - SOLA LETTURA

#define BR1200  0
#define BR2400  1
#define BR4800  2
#define BR9600  3
#define BR19200 4

#define MAX_RETRIES 100

#define E_PARITY 'E'
#define O_PARITY 'O'
#define N_PARITY 'N'

#define RESTART_TRUE  1
#define RESTART_FALSE 0

#define DEBUG_STDERR 1
#define DEBUG_SYSLOG 2

int debug_mask     = 0; //DEBUG_STDERR | DEBUG_SYSLOG; // Default, let pass all
int debug_flag     = 0;
int trace_flag     = 0;

int metern_flag    = 0;

const char *version     = "1.0";
char *programName;
const char *ttyLCKloc   = "/var/lock/LCK.."; /* location and prefix of serial port lock file */

#define CMDLINESIZE 128            /* should be enough for debug */
char cmdline[CMDLINESIZE]="";    
long unsigned int PID;

long unsigned int PPID;
char *PARENTCOMMAND = NULL;

static int yLockWait = 0;          /* Seconds to wait to lock serial port */
static time_t command_delay = -1;  // = 30;  /* MilliSeconds to wait before sending a command */
static time_t settle_time = -1;    // us to wait line to settle before starting chat

char *devLCKfile = NULL;
char *devLCKfileNew = NULL;
FILE *fdModbusExclusiveLock = NULL;  // Exclusive lock file descriptor for ModBus communication

// Forward declarations
void releaseModbusExclusiveLock(void);
void ClrSerLock(long unsigned int PID);
void AddSerLock(const char *szttyDevice, const char *devLCKfile, const long unsigned int PID, const char *COMMAND, int debug_flag);
void exit_error(modbus_t *ctx);

void usage(char* program) {
    printf("TAC1100c %s: ModBus RTU client to read TAC1100 series smart mini power meter registers\n",version);
    printf("Copyright (C) 2026 Flavio Anesi\n");
    printf("Complied with libmodbus %s\n\n", LIBMODBUS_VERSION_STRING);
    printf("Usage: %s [-a address] [-d n] [-x] [-p] [-v] [-c] [-e] [-i] [-t] [-f] [-g] [-T] [[-m]|[-q]] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -s new_address device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -r baud_rate device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -N parity device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -Q current_password -K new_password device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -L demand_period device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -U slide_time device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -R scroll_time device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -G backlit_time device\n", program);
    printf("       %s [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -Q current_password -H reset_type device\n\n", program);
    printf("Required:\n");
    printf("\tdevice\t\tSerial device (i.e. /dev/ttyUSB0)\n");
    printf("Connection parameters:\n");
    printf("\t-a address \tMeter number (1-247). Default: 1\n");
    printf("\t-b baud_rate \tUse baud_rate serial port speed (1200, 2400, 4800, 9600, 19200)\n");
    printf("\t\t\tDefault: 9600\n");
    printf("\t-P parity \tUse parity (E, N, O). Default: N\n");
    printf("\t-S bit \t\tUse stop bits (1, 2). Default: 1\n");
    printf("Reading parameters (no parameter = retrieves all values):\n");
    printf("\t-p \t\tGet power (W)\n");
    printf("\t-v \t\tGet voltage (V)\n");
    printf("\t-c \t\tGet current (A)\n");
    printf("\t-l \t\tGet apparent power (VA)\n");
    printf("\t-n \t\tGet reactive power (VAR)\n");
    printf("\t-f \t\tGet frequency (Hz)\n");
    printf("\t-o \t\tGet phase angle (Degree)\n");
    printf("\t-g \t\tGet power factor\n");
    printf("\t-i \t\tGet imported energy (Wh)\n");
    printf("\t-e \t\tGet exported energy (Wh)\n");
    printf("\t-t \t\tGet total energy (Wh)\n");
    printf("\t-A \t\tGet imported reactive energy (VARh)\n");
    printf("\t-B \t\tGet exported reactive energy (VARh)\n");
    printf("\t-C \t\tGet total reactive energy (VARh)\n");
    printf("\t-T \t\tGet Time for automatic scroll display (0=no rotation)\n");
    printf("\t-m \t\tOutput values in IEC 62056 format ID(VALUE*UNIT)\n");
    printf("\t-q \t\tOutput values in compact mode\n");
    printf("Writing new settings parameters:\n");
    printf("\t-s new_address \tSet new meter number (1-247)\n");
    printf("\t-r baud_rate \tSet baud_rate meter speed (1200, 2400, 4800, 9600, 19200)\n");
    printf("\t-N parity \tSet parity and stop bits (0-3) [REQUIRES RESTART]\n");
    printf("\t\t\t0: N1, 1: E1, 2: O1, 3: N2\n");
    printf("\t-Q password \tCurrent password for KPPA authorization (default: 0000)\n");
    printf("\t-K password \tSet new password (0-9999) [REQUIRES KPPA: use -Q]\n");
    printf("\t-L demand_period Set demand period (0-60 minutes, 0=update every second, default 60)\n");
    printf("\t-U slide_time \tSet slide time (1 to Demand_Period-1, default 1)\n");
    printf("\t-R scroll_time \tSet automatic scroll display time (0-60 seconds, 0=no rotation, default 0)\n");
    printf("\t-G backlit_time Set backlit time (0-120 or 255 minutes, 0=always on, 255=off, default 60)\n");
    printf("\t-H reset_type \tReset historical data [REQUIRES KPPA: use -Q]\n");
    printf("\t\t\t0=max demand, 8=monthly energy, 9=daily energy\n");
    printf("\n");
    printf("KPPA = Key Parameter Programming Authorization\n");
    printf("Operations marked [REQUIRES KPPA] need -Q option with current password.\n");
    printf("Operations marked [REQUIRES RESTART] need meter restart after modification.\n");
    printf("\n");
    printf("Examples:\n");
    printf("  Read all values:        %s /dev/ttyUSB0\n", program);
    printf("  Change password:        %s -Q 0000 -K 1234 /dev/ttyUSB0\n", program);
    printf("  Reset max demand:       %s -Q 1234 -H 0 /dev/ttyUSB0\n", program);
    printf("  Change meter address:   %s -s 5 /dev/ttyUSB0\n", program);
    printf("\n");
    printf("Fine tuning & debug parameters:\n");
    printf("\t-z num_retries\tTry to read max num_retries times on bus before exiting\n");
    printf("\t\t\twith error. Default: 1 (no retry)\n");
    printf("\t-j 1/10 secs\tResponse timeout. Default: 2=0.2s\n");
    printf("\t-D 1/1000 secs\tDelay before sending commands. Default: 0ms\n");
    printf("\t-w seconds\tTime to wait to lock serial port (1-30s). Default: 0s\n");
    printf("\t-W 1/1000 secs\tTime to wait for 485 line to settle. Default: 0ms\n");
    printf("\t-y 1/1000 secs\tSet timeout between every bytes (1-500). Default: disabled\n");
    printf("\t-d debug_level\tDebug (0=disable, 1=debug, 2=errors to syslog, 3=both)\n");
    printf("\t\t\tDefault: 0\n");
    printf("\t-x \t\tTrace (libmodbus debug on)\n");
}

/*--------------------------------------------------------------------------
    tv_diff
----------------------------------------------------------------------------*/
static long inline tv_diff(struct timeval const * const t1, struct timeval const * const t2)
{
    struct timeval res;
    timersub(t1, t2, &res);
    return res.tv_sec*1000000 + res.tv_usec;
}

/*--------------------------------------------------------------------------
        rnd_usleep
----------------------------------------------------------------------------*/
static long inline rnd_usleep(const useconds_t usecs)
{
    long unsigned rnd10 = 10.0*rand()/(RAND_MAX+1.0) + 1;
    if (usleep(usecs*rnd10) == 0)
        return usecs*rnd10;
    else
        return -1;
}

/*--------------------------------------------------------------------------
    getCurTime
----------------------------------------------------------------------------*/
char* getCurTime()
{
    time_t curTimeValue;
    struct tm *ltime;
    static struct timeval _t;
    static struct timezone tz;
    static char retTimeValue[85];  // Aumentato da 24 a 85 per evitare overflow

    gettimeofday(&_t, &tz);
    curTimeValue = _t.tv_sec;
    ltime = localtime(&curTimeValue);
    sprintf(retTimeValue, "%04d-%02d-%02d %02d:%02d:%02d.%06ld",
            ltime->tm_year + 1900,
            ltime->tm_mon + 1,
            ltime->tm_mday,
            ltime->tm_hour,
            ltime->tm_min,
            ltime->tm_sec,
            _t.tv_usec);

    return retTimeValue;
}

/*--------------------------------------------------------------------------
    log_message
----------------------------------------------------------------------------*/
void log_message(int type, char *format, ...)
{
    char fmt[256];
    va_list ap;

    if (type & debug_mask) {

        snprintf(fmt, sizeof(fmt), "%s %s[%lu]: %s\n", getCurTime(), programName, PID, format);
        //snprintf(fmt, sizeof(fmt), "%s %s[%lu(%lu)]<%s>: %s\n", getCurTime(), programName, PID, PPID, (PARENTCOMMAND == NULL ? "" : PARENTCOMMAND), format);

        va_start(ap, format);
        if (type & DEBUG_STDERR) {
            vfprintf(stderr, fmt, ap);
            fflush(stderr);
        }
        va_end(ap);

        va_start(ap, format);
        if (type & DEBUG_SYSLOG) {
            vsyslog(LOG_INFO, fmt, ap);
        }
        va_end(ap);
    }
}

/*--------------------------------------------------------------------------
    getCmdLine
----------------------------------------------------------------------------*/
int getCmdLine() {
    int fdcmd;
    size_t bytes_read;
    
    *cmdline = '\0';
    snprintf(cmdline, sizeof(cmdline), "/proc/%lu/cmdline", PID);
    
    if ((fdcmd = open(cmdline, O_RDONLY)) < 0) {
        *cmdline = '\0';
        return 0;
    }
    
    *cmdline = '\0';
    if ((bytes_read = read(fdcmd, cmdline, sizeof(cmdline)-1)) <= 0) {
        close(fdcmd);
        *cmdline = '\0';
        return 0;
    }
    close(fdcmd);
    
    cmdline[bytes_read] = '\0';
    
    size_t i;
    for (i = 0; i < bytes_read; i++) {
        if (cmdline[i] == '\0') cmdline[i] = ' ';
    }
    cmdline[bytes_read-1] = '\0';
    
    return bytes_read;
}

/*--------------------------------------------------------------------------
    getMemPtr
----------------------------------------------------------------------------*/
void *getMemPtr(size_t sSize)
{
    void *pMem;
    if ((pMem = calloc(sSize, sizeof(char))) == NULL) {
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "getMemPtr(): calloc(): %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    return pMem;
}

/*--------------------------------------------------------------------------
    ClrSerLock
----------------------------------------------------------------------------*/
void ClrSerLock(long unsigned int PID)
{
    FILE *fdserlck;
    long unsigned int LckPID;
    char *COMMAND = NULL;
    int fLen = 0;
    int curChar = 0;

    // Always release exclusive ModBus lock before clearing serial lock
    releaseModbusExclusiveLock();

    if ((fdserlck = fopen(devLCKfile, "r")) == NULL) {
        log_message(debug_flag | DEBUG_SYSLOG, "ClrSerLock(): can't open lock file: %s for read.", devLCKfile);
        return;
    }

    // Acquire exclusive lock before clearing - wait for all shared locks to release
    // Note: During normal operation, we use LOCK_SH (shared lock) which allows
    // multiple processes to access the serial port. This works well when processes
    // communicate with different ModBus addresses. However, if multiple instances
    // try to communicate simultaneously (even with the same address), there's a small
    // chance of bus collisions. Use -z option for retries to handle this gracefully.
    log_message(debug_flag, "Acquiring exclusive lock on %s to clear...", devLCKfile);
    flock(fileno(fdserlck), LOCK_EX);   // Will wait to acquire lock then continue
    log_message(debug_flag, "Exclusive lock on %s acquired.", devLCKfile);

    while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n' && curChar != ' ') fLen++;
    fLen = 0;
    if (curChar == ' ') while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n') fLen++;

    rewind(fdserlck);
    
    COMMAND = getMemPtr(fLen+1);
    COMMAND[0] = '\0';
    LckPID = 0;
    
    int bRead = fscanf(fdserlck, "%lu%*[ ]%[^\n]\n", &LckPID, COMMAND);
    
    if (bRead != EOF && LckPID > 0) {
        if (LckPID != PID) {
            log_message(debug_flag | DEBUG_SYSLOG, "Lock held by process (%lu)%s, can't clear for process (%lu)", LckPID, COMMAND, PID);
            fclose(fdserlck);  // Release exclusive lock
            free(COMMAND);
            return;
        }
    }

    // Keep file open with exclusive lock while deleting to prevent race conditions
    if (unlink(devLCKfileNew) != 0 && errno != ENOENT) {
        log_message(DEBUG_SYSLOG, "ClrSerLock(): unlink(%s): (%d) %s", devLCKfileNew, errno, strerror(errno));
    }

    if (PID == LckPID && unlink(devLCKfile) == -1) {
        log_message(DEBUG_SYSLOG, "ClrSerLock(): unlink(%s): (%d) %s", devLCKfile, errno, strerror(errno));
    }

    log_message(debug_flag, "ClrSerLock(%lu) removed lock file: %s", PID, devLCKfile);
    
    fclose(fdserlck);  // This also releases the exclusive lock
    free(COMMAND);
}

/*--------------------------------------------------------------------------
    AddSerLock
----------------------------------------------------------------------------*/
void AddSerLock(const char *szttyDevice, const char *devLCKfile, const long unsigned int PID, const char *COMMAND, int debug_flag)
{
    int fddevlock;
    int retry = 0;
    char PIDstr[24];

    snprintf(PIDstr, sizeof(PIDstr), "%lu", PID);

    while (retry < 3) {
        if ((fddevlock = open(devLCKfileNew, O_WRONLY | O_CREAT | O_EXCL, 0644)) > 0) break;
        
        if (errno == EEXIST) {
            log_message(debug_flag, "AddSerLock(): lock file exists: %s, retry %d", devLCKfileNew, retry);
            rnd_usleep(25000);
            retry++;
        } else {
            log_message(DEBUG_SYSLOG, "AddSerLock(): open(%s): (%d) %s", devLCKfileNew, errno, strerror(errno));
            exit(2);
        }
    }

    if (retry >= 3) {
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "AddSerLock(): Unable to create lock file %s after %d retries", devLCKfileNew, retry);
        exit(2);
    }

    if (write(fddevlock, PIDstr, strlen(PIDstr)) != (ssize_t)strlen(PIDstr)) {
        log_message(DEBUG_SYSLOG, "AddSerLock(): write(): (%d) %s", errno, strerror(errno));
        close(fddevlock);
        exit(2);
    }

    if (COMMAND != NULL && strlen(COMMAND) > 0) {
        if (write(fddevlock, " ", 1) != 1 ||
            write(fddevlock, COMMAND, strlen(COMMAND)) != (ssize_t)strlen(COMMAND)) {
            log_message(DEBUG_SYSLOG, "AddSerLock(): write(command): (%d) %s", errno, strerror(errno));
        }
    }

    if (write(fddevlock, "\n", 1) != 1) {
        log_message(DEBUG_SYSLOG, "AddSerLock(): write(newline): (%d) %s", errno, strerror(errno));
    }

    close(fddevlock);

    if (link(devLCKfileNew, devLCKfile) == -1) {
        if (errno != EEXIST) {
            log_message(DEBUG_SYSLOG, "AddSerLock(): link(%s, %s): (%d) %s", devLCKfileNew, devLCKfile, errno, strerror(errno));
            exit(2);
        }
    }

    if (unlink(devLCKfileNew) == -1) {
        log_message(DEBUG_SYSLOG, "AddSerLock(): unlink(%s): (%d) %s", devLCKfileNew, errno, strerror(errno));
    }

    log_message(debug_flag, "AddSerLock(%lu) to %s", PID, devLCKfile);
}

void releaseModbusExclusiveLock(void)
{
    if (fdModbusExclusiveLock != NULL) {
        log_message(debug_flag, "Releasing exclusive ModBus lock...");
        fclose(fdModbusExclusiveLock);
        fdModbusExclusiveLock = NULL;
    }
}

void exit_error(modbus_t *ctx)
{
/*
      log_message(debug_flag, "Flushing modbus buffer");
      log_message(debug_flag, "Flushed %d bytes", modbus_flush(ctx));
*/
      modbus_close(ctx);
      modbus_free(ctx);
      ClrSerLock(PID);
      free(devLCKfile);
      free(devLCKfileNew);
      if (!metern_flag) {
        printf("NOK\n");
        log_message(debug_flag | DEBUG_SYSLOG, "NOK");
      }
      free(PARENTCOMMAND);
      exit(EXIT_FAILURE);
}

inline int bcd2int(int val)
{
    return((((val & 0xf0) >> 4) * 10) + (val & 0xf));
}

int int2bcd(int val)
{
    return(((val / 10) << 4) + (val % 10));
}

int bcd2num(const uint16_t *src, int len)
{
    int n = 0;
    int m = 1;
    int i = 0;
    int shift = 0;
    int digit = 0;
    int j = 0;
    for (i = 0; i < len; i++) {
        for (j = 0; j < 4; j++) {
            digit = ((src[len-1-i]>>shift) & 0x0F) * m;
            n += digit;
            m *= 10;
            shift += 4;
        }
    }
    return n;
}

// Funzione per leggere valori in formato Float (usata per letture)
float getMeasureFloat(modbus_t *ctx, int address, int retries, int nb) {

    uint16_t tab_reg[nb * sizeof(uint16_t)];
    int rc = -1;
    int i;
    int j = 0;
    int exit_loop = 0;
    int errno_save=0;
    struct timeval tvStart, tvStop;

    while (j < retries && exit_loop == 0) {
      j++;

      if (command_delay) {
        log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
        usleep(command_delay);
      }

      log_message(debug_flag, "%d/%d. Register Address %d [%04X]", j, retries, 30000+address+1, address);
      gettimeofday(&tvStart, NULL); 
      rc = modbus_read_input_registers(ctx, address, nb, tab_reg);
      errno_save = errno;
      gettimeofday(&tvStop, NULL); 

      if (rc == -1) {
        if (trace_flag) fprintf(stderr, "%s: ERROR (%d) %s, %d/%d\n", programName, errno_save, modbus_strerror(errno_save), j, retries);
        log_message(debug_flag | ( j==retries ? DEBUG_SYSLOG : 0), "ERROR (%d) %s, %d/%d, Address %d [%04X]", errno_save, modbus_strerror(errno_save), j, retries, 30000+address+1, address);
        log_message(debug_flag | ( j==retries ? DEBUG_SYSLOG : 0), "Response timeout gave up after %ldus", tv_diff(&tvStop, &tvStart));
        if (command_delay) {
          log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
          usleep(command_delay);
        }
      } else {
        log_message(debug_flag, "Read time: %ldus", tv_diff(&tvStop, &tvStart));
        exit_loop = 1;
      }

    }

    if (rc == -1) {
      exit_error(ctx);
    }

    if (debug_flag) {
       for (i=0; i < rc; i++) {
          log_message(debug_flag, "reg[%d/%d]=%d (0x%X)", i, (rc-1), tab_reg[i], tab_reg[i]);
       }
    }

    // swap LSB and MSB
    uint16_t tmp1 = tab_reg[0];
    uint16_t tmp2 = tab_reg[1];
    tab_reg[0] = tmp2;
    tab_reg[1] = tmp1;

    float value = modbus_get_float(&tab_reg[0]);

    return value;

}

// Funzione per leggere configurazioni in formato UINT (Holding Registers)
int getConfigUINT(modbus_t *ctx, int address, int retries, int nb) {

    uint16_t tab_reg[nb * sizeof(uint16_t)];
    int rc = -1;
    int i;
    int j = 0;
    int exit_loop = 0;

    while (j < retries && exit_loop == 0) {
      j++;

      if (command_delay) {
        log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
        usleep(command_delay);
      }

      log_message(debug_flag, "%d/%d. Register Address %d [%04X]", j, retries, 40000+address+1, address);
      rc = modbus_read_registers(ctx, address, nb, tab_reg);

      if (rc == -1) {
        log_message(debug_flag | ( j==retries ? DEBUG_SYSLOG : 0), "ERROR (%d) %s, %d/%d, Address %d [%04X]", errno, modbus_strerror(errno), j, retries, 40000+address+1, address);
        if (command_delay) {
          log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
          usleep(command_delay);
        }
      } else {
        exit_loop = 1;
      }
    }

    if (rc == -1) {
      exit_error(ctx);
    }

    if (debug_flag) {
       for (i=0; i < rc; i++) {
          log_message(debug_flag, "reg[%d/%d]=%d (0x%X)", i, (rc-1), tab_reg[i], tab_reg[i]);
       }
    }

    int value = tab_reg[0];

    return value;

}

// Funzione per abilitare KPPA (Key Parameter Programming Authorization)
// Requires current password to enable writing to protected parameters
int enableKPPA(modbus_t *ctx, int current_password)
{
    uint16_t tab_reg[1];
    tab_reg[0] = (uint16_t)current_password;

    if (command_delay) {
      log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
      usleep(command_delay);
    }

    log_message(debug_flag, "Enabling KPPA with password %d (0x%04X)", current_password, current_password);
    
    // Write password to KPPA register to enable authorization
    int n = modbus_write_registers(ctx, KPPA, 1, tab_reg);
    if (n != -1) {
        log_message(debug_flag, "KPPA enabled successfully");
        return 0;  // Success
    } else {
        log_message(debug_flag | DEBUG_STDERR, "KPPA enable failed: (%d) %s", errno, modbus_strerror(errno));
        return -1;  // Failure
    }
}

// Funzione per scrivere configurazioni in formato UINT (per TAC1100)
void changeConfigUINT(modbus_t *ctx, int address, int new_value, int restart)
{
    uint16_t tab_reg[1];
    tab_reg[0] = (uint16_t)new_value;

    if (command_delay) {
      log_message(debug_flag, "Sleeping command delay: %ldus", command_delay);
      usleep(command_delay);
    }

    log_message(debug_flag, "Writing value %d (0x%04X) to register 0x%04X", new_value, new_value, address);
    
    // TAC1100 requires Function Code 0x10 (Write Multiple Registers) even for single register
    int n = modbus_write_registers(ctx, address, 1, tab_reg);
    if (n != -1) {
        printf("New value %d for address 0x%X successfully written\n", new_value, address);
        if (restart == RESTART_TRUE) {
            printf("\n");
            printf("*******************************************************\n");
            printf("*** ATTENTION: METER RESTART REQUIRED               ***\n");
            printf("*** Please restart the meter to apply the changes   ***\n");
            printf("*******************************************************\n");
            printf("\n");
        }
    } else {
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Write error: (%d) %s, n=%d", errno, modbus_strerror(errno), n);
        if (errno == EMBXILFUN) { // Illegal function
            log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Tip: Parameter may be read-only or password authorization (KPPA) required");
            fprintf(stderr, "\n");
            fprintf(stderr, "ERROR: Write operation failed.\n");
            fprintf(stderr, "This parameter may require KPPA (Key Parameter Programming Authorization).\n");
            fprintf(stderr, "Please ensure the meter is in programming mode with correct password.\n");
            fprintf(stderr, "\n");
        }
        exit_error(ctx);
    }
}

/*--------------------------------------------------------------------------
    getIntLen
----------------------------------------------------------------------------*/
int getIntLen(long value){
  long l=!value;
  while(value) { l++; value/=10; }
  return l;
}

/*--------------------------------------------------------------------------
    getPIDcmd
----------------------------------------------------------------------------*/
void *getPIDcmd(long unsigned int PID)
{
    int fdcmd;
    char *COMMAND = NULL;
    size_t cmdLen = 0;
    size_t length;
    char buffer[1024];
    char cmdFilename[getIntLen(PID)+14+1];

    // Generate the name of the cmdline file for the process
    *cmdFilename = '\0';
    snprintf(cmdFilename,sizeof(cmdFilename),"/proc/%lu/cmdline",PID);
    
    // Read the contents of the file
    if ((fdcmd  = open(cmdFilename, O_RDONLY)) < 0) return NULL;
    if ((length = read(fdcmd, buffer, sizeof(buffer))) <= 0) {
        close(fdcmd); return NULL;
    }     
    close(fdcmd);
    
    // read does not NUL-terminate the buffer, so do it here
    buffer[length] = '\0';
    // Get 1st string (command)
    cmdLen=strlen(buffer)+1;
    if((COMMAND = getMemPtr(cmdLen)) != NULL ) {
        strncpy(COMMAND, buffer, cmdLen);
        COMMAND[cmdLen-1] = '\0';
    }

    return COMMAND;
}

/*--------------------------------------------------------------------------
    lockSer
----------------------------------------------------------------------------*/
void lockSer(const char *szttyDevice, const long unsigned int PID, int debug_flag)
{
    char *pos;
    FILE *fdserlck = NULL;
    char *COMMAND = NULL;
    long unsigned int LckPID;
    struct timeval tLockStart, tLockNow;
    int bRead;
    int errno_save = 0;
    int fLen = 0;
    int curChar = 0;
    char *LckCOMMAND = NULL;
    char *LckPIDcommand = NULL;

    pos = strrchr(szttyDevice, '/');
    if (pos > 0) {
        pos++;
        devLCKfile = getMemPtr(strlen(ttyLCKloc)+(strlen(szttyDevice)-(pos-szttyDevice))+1);
        devLCKfile[0] = '\0';
        strcpy(devLCKfile,ttyLCKloc);
        strcat(devLCKfile, pos);
        devLCKfile[strlen(devLCKfile)] = '\0';
        devLCKfileNew = getMemPtr(strlen(devLCKfile)+getIntLen(PID)+2);	/* dot & terminator */
        devLCKfileNew[0] = '\0';
        strcpy(devLCKfileNew,devLCKfile);
        sprintf(devLCKfileNew,"%s.%lu",devLCKfile,PID);
        devLCKfileNew[strlen(devLCKfileNew)] = '\0';
    } else {
        devLCKfile = NULL;
    }

    log_message(debug_flag, "szttyDevice: %s",szttyDevice);
    log_message(debug_flag, "devLCKfile: <%s>",devLCKfile);
    log_message(debug_flag, "devLCKfileNew: <%s>",devLCKfileNew);
    log_message(debug_flag, "PID: %lu", PID);    

    COMMAND = getPIDcmd(PID);
    AddSerLock(szttyDevice, devLCKfile, PID, COMMAND, debug_flag);

    LckPID = 0;
    long unsigned int oldLckPID = 0;
    int staleLockRetries = 0;
    int const staleLockRetriesMax = 2;
    long unsigned int clrStaleTargetPID = 0;    
    int missingPidRetries = 0;
    int const missingPidRetriesMax = 2;
    int totalLockAttempts = 0;
    int const maxLockAttempts = 100; // Prevent infinite loop
    
    gettimeofday(&tLockStart, NULL);
    tLockNow=tLockStart;

    if (debug_flag) log_message(debug_flag, "Checking for lock");
    while(LckPID != PID && tv_diff(&tLockNow, &tLockStart) <= yLockWait*1000000L) {
        
        totalLockAttempts++;
        if (totalLockAttempts > maxLockAttempts) {
            free(LckCOMMAND);
            free(LckPIDcommand);
            free(COMMAND);
            ClrSerLock(PID);
            log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Exceeded maximum lock attempts (%d). Lock file may be corrupted: %s", maxLockAttempts, devLCKfile);
            log_message(DEBUG_STDERR, "Try removing the lock file manually: sudo rm -f %s", devLCKfile);
            free(devLCKfile); free(devLCKfileNew); free(PARENTCOMMAND);
            exit(2);
        }

        do {
            fdserlck = fopen(devLCKfile, "r");
            if (fdserlck == NULL) {
                log_message(debug_flag | DEBUG_SYSLOG, "Problem locking serial device, can't open lock file: %s for read.",devLCKfile);
                exit(2);
            }
            errno = 0;
            if (flock(fileno(fdserlck), LOCK_SH | LOCK_NB) == 0) break;      // Lock Acquired 
            errno_save=errno;
            
            if (errno_save == EWOULDBLOCK) {
                log_message(debug_flag, "Would block %s, retry (%d) %s...", devLCKfile, errno_save, strerror(errno_save));
                rnd_usleep(25000);
                fclose(fdserlck);
            } else {
                log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Problem locking serial device, can't open lock file: %s for read. (%d) %s", devLCKfile, errno_save, strerror(errno_save));
                exit(2);
            }
        } while (errno_save == EWOULDBLOCK);

        fLen = 0;
        while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n' && curChar != ' ') fLen++;
        fLen = 0;
        if (curChar == ' ') while ((curChar = fgetc(fdserlck)) != EOF && curChar != '\n') fLen++;

        rewind(fdserlck);
        
        LckCOMMAND = getMemPtr(fLen+1);
        LckCOMMAND[0] = '\0';
        LckPID=0;
        
        errno = 0;
        bRead = fscanf(fdserlck, "%lu%*[ ]%[^\n]\n", &LckPID, LckCOMMAND);
        errno_save = errno;
        fclose(fdserlck);
        if (LckPID != oldLckPID) {
            log_message(debug_flag | (bRead==EOF || errno_save != 0 ? DEBUG_SYSLOG : 0), "errno=%i, bRead=%i PID=%lu LckPID=%lu", errno_save, bRead, PID, LckPID);
            log_message(debug_flag, "Checking process %lu (%s) for lock", LckPID, LckCOMMAND);
        }
        if (bRead == EOF || LckPID == 0 || errno_save != 0) {
            log_message(debug_flag | DEBUG_SYSLOG, "Problem locking serial device, can't read PID from lock file: %s.",devLCKfile);
            log_message(debug_flag | DEBUG_SYSLOG, "errno=%i, bRead=%i PID=%lu LckPID=%lu", errno_save, bRead, PID, LckPID);
            if (errno_save != 0) {
                log_message(debug_flag | DEBUG_SYSLOG, "(%u) %s", errno_save, strerror(errno_save));
                free(LckCOMMAND); free(LckPIDcommand); free(COMMAND);
                exit(2);
            } else {
                if (missingPidRetries < missingPidRetriesMax) {
                    missingPidRetries++;
                    log_message(debug_flag, "%s miss process self PID from lock file?",devLCKfile);
                    rnd_usleep(250000); // Wait 250ms before retry
                } else if (missingPidRetries >= missingPidRetriesMax) {
                    log_message(debug_flag | DEBUG_SYSLOG, "%s miss process self PID from lock file, amending.",devLCKfile);
                    AddSerLock(szttyDevice, devLCKfile, PID, COMMAND, debug_flag);
                    missingPidRetries = 0;
                }
            }
            oldLckPID = LckPID;
        } else { //fread OK
          
          missingPidRetries = 0;
          
          LckPIDcommand = getPIDcmd(LckPID);
          
          if (LckPID != oldLckPID) {
              log_message(debug_flag, "PID: %lu COMMAND: \"%s\" LckPID: %lu LckCOMMAND: \"%s\" LckPIDcommand \"%s\"%s", PID, COMMAND
                                          , LckPID, LckCOMMAND, LckPIDcommand
                                          , LckPID == PID ? " = me" : "");
              oldLckPID = LckPID;              
          }
          
          if ((PID != LckPID && LckPIDcommand == NULL) || (LckCOMMAND[0]!='\0' && strcmp(LckPIDcommand,LckCOMMAND) != 0) || strcmp(LckPIDcommand,"") == 0) {
                // Stale lock: process doesn't exist or command mismatch
                if (staleLockRetries < staleLockRetriesMax) {
                    staleLockRetries++;
                    clrStaleTargetPID = LckPID;
                    log_message(debug_flag | (staleLockRetries > 1 ? DEBUG_SYSLOG : 0), "Stale pid lock(%d)? PID=%lu, LckPID=%lu, LckCOMMAND='%s', LckPIDCommand='%s'", staleLockRetries, PID, LckPID, LckCOMMAND, LckPIDcommand);
                } else if (LckPID == clrStaleTargetPID && staleLockRetries >= staleLockRetriesMax) {
                    log_message(debug_flag | DEBUG_SYSLOG, "Clearing stale serial port lock. (%lu)", LckPID);
                    ClrSerLock(LckPID);
                    staleLockRetries = 0;
                    clrStaleTargetPID = 0;
                }
          } else {
                // Valid lock: process exists and command matches
                staleLockRetries = 0;
                clrStaleTargetPID = 0;
                
                // Check if it's a compatible ModBus client (sdm120c, aurora, tac1100, etc.)
                if (PID != LckPID && LckPIDcommand != NULL) {
                    // Lock held by another process - check if it's ModBus compatible
                    if (strstr(LckPIDcommand, "sdm120") != NULL || 
                        strstr(LckPIDcommand, "tac1100") != NULL || 
                        strstr(LckPIDcommand, "aurora") != NULL ||
                        strstr(LckPIDcommand, "modbus") != NULL) {
                        // Compatible ModBus client - we can share the bus
                        log_message(debug_flag, "Compatible ModBus client detected: %s (PID %lu). Sharing bus.", LckPIDcommand, LckPID);
                        // Accept this as valid - we have shared lock, exit loop
                        LckPID = PID;  // Set to our PID to exit the loop
                    }
                }
          } 
        }

        if (yLockWait > 0 && LckPID != PID) {
             rnd_usleep(25000);
        }

        // Cleanup and loop        
        free(LckCOMMAND);
        free(LckPIDcommand);
        LckCOMMAND = NULL;
        LckPIDcommand = NULL;

        gettimeofday(&tLockNow, NULL);
    }

    free(LckCOMMAND);
    free(LckPIDcommand);
    free(COMMAND);

    if (LckPID != PID) {
        ClrSerLock(PID);
        log_message(DEBUG_STDERR, "Problem locking serial device %s.",szttyDevice);
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Unable to get lock on serial %s for %lu in %ds: still locked by %lu.",szttyDevice,PID,(yLockWait)%30,LckPID);
        log_message(DEBUG_STDERR, "Try a greater -w value (eg -w%u).", (yLockWait+2)%30);
        free(devLCKfile); free(devLCKfileNew); free(PARENTCOMMAND);        
        exit(2);
    }
    
    // We have shared lock now. Before opening ModBus connection, upgrade to exclusive lock
    // to prevent bus collisions when multiple processes communicate simultaneously.
    log_message(debug_flag, "Upgrading to exclusive lock for ModBus communication...");
    fdModbusExclusiveLock = fopen(devLCKfile, "r");
    if (fdModbusExclusiveLock != NULL) {
        if (flock(fileno(fdModbusExclusiveLock), LOCK_EX) != 0) {
            log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Failed to acquire exclusive lock: (%d) %s", errno, strerror(errno));
            fclose(fdModbusExclusiveLock);
            fdModbusExclusiveLock = NULL;
            ClrSerLock(PID);
            free(devLCKfile); free(devLCKfileNew); free(PARENTCOMMAND);
            exit(2);
        }
        log_message(debug_flag, "Exclusive lock acquired. Ready for ModBus communication.");
        // Keep this file open during all ModBus operations
        // It will be closed when program exits or in exit_error()
    } else {
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Failed to open lock file for exclusive access");
    }
}

int main(int argc, char* argv[])
{
    int device_address = 1;
    
    // Flags for reading parameters
    int power_flag     = 0;
    int volt_flag      = 0;
    int current_flag   = 0;
    int pangle_flag    = 0;
    int freq_flag      = 0;
    int pf_flag        = 0;
    int apower_flag    = 0;
    int rapower_flag   = 0;
    int export_flag    = 0;
    int import_flag    = 0;
    int total_flag     = 0;
    int rexport_flag   = 0;
    int rimport_flag   = 0;
    int rtotal_flag    = 0;
    int time_disp_flag = 0;
    
    // Flags for writing common SDM120/TAC1100 parameters
    int new_address        = 0;
    int new_baud_rate      = -1;
    int new_parity_stop    = -1;
    
    // Flags for writing TAC1100 parameters
    int password_flag       = 0;
    int password_value      = 0;
    int current_password    = 0;  // Current password for KPPA
    int current_password_set = 0; // Flag if current password was specified
    int demand_period_flag  = 0;
    int demand_period       = 0;
    int slide_time_flag     = 0;
    int slide_time          = 0;
    int scroll_time_flag    = 0;
    int scroll_time         = 0;
    int backlit_time_flag   = 0;
    int backlit_time        = 0;
    int reset_hist_flag     = 0;
    int reset_hist_type     = 0;
    
    int compact_flag   = 0;
    int count_param    = 0;
    int num_retries    = 1;
    
#if LIBMODBUS_VERSION_MAJOR >= 3 && LIBMODBUS_VERSION_MINOR >= 1 && LIBMODBUS_VERSION_MICRO >= 2
    uint32_t resp_timeout = 2;
    uint32_t byte_timeout = -1;    
#else
    time_t resp_timeout = 2;
    time_t byte_timeout = -1;
#endif
    char *szttyDevice  = NULL;

    int c;
    int speed          = 0;
    int bits           = 0;
    int read_count     = 0;

    const char *EVEN_parity = "E";
    const char *NONE_parity = "N";
    const char *ODD_parity  = "O";
    char *c_parity     = NULL;
    
    int baud_rate      = 0;
    int stop_bits      = 1;  // Default: 1 stop bit
    char parity        = N_PARITY;  // Default: None
    
    programName        = argv[0];

    if (argc == 1) {
        usage(programName);
        exit(EXIT_FAILURE);
    }

    srand(getpid()^time(NULL));      // Init random numbers

    PID = getpid();
    getCmdLine();

    PPID = getppid();
    PARENTCOMMAND = getPIDcmd(PPID); 

    opterr = 0;

    // Opzioni: a b c d D e f g i j K L m n N o p P q Q r R s S t T U v w W x y z A B C G H
    while ((c = getopt (argc, argv, "a:Ab:BcCd:D:efgij:K:lL:mN:no OpP:qQ:r:R:s:S:tTU:vw:W:xy:z:G:H:")) != -1) {
        log_message(debug_flag | DEBUG_SYSLOG, "optind = %d, argc = %d, c = %c, optarg = %s", optind, argc, c, optarg);

        switch (c)
        {
            case 'a':
                device_address = atoi(optarg);
                if (!(0 < device_address && device_address <= 247)) {
                    fprintf (stderr, "%s: Address must be between 1 and 247.\n", programName);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "device_address = %d", device_address);
                break;
                
            case 'v':
                volt_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "volt_flag = %d, count_param = %d", volt_flag, count_param);
                break;
                
            case 'p':
                power_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "power_flag = %d, count_param = %d", power_flag, count_param);
                break;
                
            case 'c':
                current_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "current_flag = %d, count_param = %d", current_flag, count_param);
                break;
                
            case 'e':
                export_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "export_flag = %d, count_param = %d", export_flag, count_param);
                break;
                
            case 'i':
                import_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "import_flag = %d, count_param = %d", import_flag, count_param);
                count_param++;
                break;
                
            case 't':
                total_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "total_flag = %d, count_param = %d", total_flag, count_param);
                break;
                
            case 'A':
                rimport_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "rimport_flag = %d, count_param = %d", rimport_flag, count_param);
                break;
                
            case 'B':
                rexport_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "rexport_flag = %d, count_param = %d", rexport_flag, count_param);
                break;
                
            case 'C':
                rtotal_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "rtotal_flag = %d, count_param = %d", rtotal_flag, count_param);
                break;
                
            case 'f':
                freq_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "freq_flag = %d, count_param = %d", freq_flag, count_param);
                break;
                
            case 'g':
                pf_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "pf_flag = %d, count_param = %d", pf_flag, count_param);
                break;
                
            case 'l':
                apower_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "apower_flag = %d, count_param = %d", apower_flag, count_param);
                break;
                
            case 'n':
                rapower_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "rapower_flag = %d, count_param = %d", rapower_flag, count_param);
                count_param++;
                break;
                
            case 'o':
                pangle_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "pangle_flag = %d, count_param = %d", pangle_flag, count_param);
                count_param++;
                break;
                
            case 'd':
                switch (*optarg) {
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                        debug_flag = atoi(optarg) & DEBUG_STDERR;
                        debug_mask = atoi(optarg);
                        break;
                    default:
                         fprintf (stderr, "%s: Debug value must be one of 0,1,2,3.\n", programName);
                         exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "debug_flag = %d", debug_flag);
                break;
                
            case 'x':
                trace_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "trace_flag = %d, count_param = %d", trace_flag, count_param);
                break;
                
            case 'b':
                speed = atoi(optarg);
                if (speed == 1200 || speed == 2400 || speed == 4800 || speed == 9600 || speed == 19200) {
                    baud_rate = speed;
                } else {
                    fprintf (stderr, "%s: Baud Rate must be one of 1200, 2400, 4800, 9600, 19200\n", programName);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "speed = %d, count_param = %d", speed, count_param);
                break;
                
            case 'P':
                c_parity = strdup(optarg);
                if (strcmp(c_parity,EVEN_parity) == 0) {
                    parity = E_PARITY;
                } else if (strcmp(c_parity,NONE_parity) == 0) {
                    parity = N_PARITY;
                } else if (strcmp(c_parity,ODD_parity) == 0) {
                    parity = O_PARITY;
                } else {
                    fprintf (stderr, "%s: Parity must be one of E, N, O\n", programName);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "c_parity = %s, count_param = %d", c_parity, count_param);
                free(c_parity);
                break;
                
            case 'S':
                bits = atoi(optarg);
                if (bits == 1 || bits == 2) {
                    stop_bits = bits;
                } else {
                    fprintf (stderr, "%s: Stop bits can be one of 1, 2\n", programName);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "bits = %d, count_param = %d", bits, count_param);
                break;
                
            // ==========================================
            // TAC1100 SPECIFIC WRITE PARAMETERS
            // ==========================================
            
            case 's': // Set new meter address
                new_address = atoi(optarg);
                if (!(0 < new_address && new_address <= 247)) {
                    fprintf (stderr, "%s: New address (%d) out of range, 1-247.\n", programName, new_address);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "new_address = %d", new_address);
                break;
                
            case 'r': // Set new baud rate
                speed = atoi(optarg);
                switch (speed) {
                    case 1200:
                        new_baud_rate = BR1200;
                        break;
                    case 2400:
                        new_baud_rate = BR2400;
                        break;
                    case 4800:
                        new_baud_rate = BR4800;
                        break;
                    case 9600:
                        new_baud_rate = BR9600;
                        break;
                    case 19200:
                        new_baud_rate = BR19200;
                        break;
                    default:
                        fprintf (stderr, "%s: Baud Rate must be one of 1200, 2400, 4800, 9600, 19200\n", programName);
                        exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "new_baud_rate = %d", new_baud_rate);
                break;
                
            case 'N': // Set new parity/stop bits
                new_parity_stop = atoi(optarg);
                if (!(0 <= new_parity_stop && new_parity_stop <= 3)) {
                    fprintf (stderr, "%s: New parity/stop (%d) out of range, 0-3.\n", programName, new_parity_stop);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "new_parity_stop = %d", new_parity_stop);
                break;
            
            case 'K': // Set Password
                password_flag = 1;
                password_value = atoi(optarg);
                if (!(0 <= password_value && password_value <= 9999)) {
                    fprintf (stderr, "%s: Password (%d) out of range, 0-9999.\n", programName, password_value);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "password_flag = %d, password_value = %d", password_flag, password_value);
                break;
                
            case 'Q': // Current Password (for KPPA authorization)
                current_password_set = 1;
                current_password = atoi(optarg);
                if (!(0 <= current_password && current_password <= 9999)) {
                    fprintf (stderr, "%s: Current password (%d) out of range, 0-9999.\n", programName, current_password);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "current_password_set = %d, current_password = %d", current_password_set, current_password);
                break;
                
            case 'L': // Set Demand Period
                demand_period_flag = 1;
                demand_period = atoi(optarg);
                if (!(0 <= demand_period && demand_period <= 60)) {
                    fprintf (stderr, "%s: Demand period (%d) out of range, 0-60 minutes.\n", programName, demand_period);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "demand_period_flag = %d, demand_period = %d", demand_period_flag, demand_period);
                break;
                
            case 'U': // Set Slide Time
                slide_time_flag = 1;
                slide_time = atoi(optarg);
                if (slide_time < 1) {
                    fprintf (stderr, "%s: Slide time (%d) must be >= 1.\n", programName, slide_time);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "slide_time_flag = %d, slide_time = %d", slide_time_flag, slide_time);
                break;
                
            case 'R': // Set Automatic Scroll Display Time
                scroll_time_flag = 1;
                scroll_time = atoi(optarg);
                if (!(0 <= scroll_time && scroll_time <= 60)) {
                    fprintf (stderr, "%s: Scroll time (%d) out of range, 0-60 seconds.\n", programName, scroll_time);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "scroll_time_flag = %d, scroll_time = %d", scroll_time_flag, scroll_time);
                break;
                
            case 'G': // Set Backlit Time (cambiato da -B per evitare conflitto con -B reactive export)
                backlit_time_flag = 1;
                backlit_time = atoi(optarg);
                if (!((0 <= backlit_time && backlit_time <= 120) || backlit_time == 255)) {
                    fprintf (stderr, "%s: Backlit time (%d) out of range, 0-120 or 255.\n", programName, backlit_time);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "backlit_time_flag = %d, backlit_time = %d", backlit_time_flag, backlit_time);
                break;
                
            case 'H': // Reset Historical Data
                reset_hist_flag = 1;
                reset_hist_type = atoi(optarg);
                if (!(reset_hist_type == 0 || reset_hist_type == 8 || reset_hist_type == 9)) {
                    fprintf (stderr, "%s: Reset type (%d) must be 0 (max demand), 8 (monthly) or 9 (daily).\n", programName, reset_hist_type);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "reset_hist_flag = %d, reset_hist_type = %d", reset_hist_flag, reset_hist_type);
                break;
                
            case 'm':
                metern_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "metern_flag = %d, count_param = %d", metern_flag, count_param);
                break;
                
            case 'q':
                compact_flag = 1;
                log_message(debug_flag | DEBUG_SYSLOG, "compact_flag = %d, count_param = %d", compact_flag, count_param);
                break;
                
            case 'z':
                num_retries = atoi(optarg);
                if (!(0 < num_retries && num_retries <= MAX_RETRIES)) {
                    fprintf (stderr, "%s: num_retries (%d) out of range, 1-%d.\n", programName, num_retries, MAX_RETRIES);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "num_retries = %d, count_param = %d", num_retries, count_param);
                break;
                
            case 'j':
                resp_timeout = atoi(optarg);
                if (resp_timeout < 1 || resp_timeout > 500) {
                    fprintf(stderr, "%s: -j Response timeout (%lu) out of range, 0-500.\n",programName,(long unsigned)resp_timeout);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "resp_timeout = %d, count_param = %d", resp_timeout, count_param);
                break;
                
            case 'y':
                byte_timeout = atoi(optarg);
                if (byte_timeout < 1 || byte_timeout > 500) {
                    fprintf(stderr, "%s: -y Byte timeout (%lu) out of range, 1-500.\n",programName,(long unsigned)byte_timeout);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "byte_timeout = %d, count_param = %d", byte_timeout, count_param);
                break;
                
            case 'w':
                yLockWait = atoi(optarg);
                if (yLockWait < 1 || yLockWait > 30) {
                    fprintf(stderr, "%s: -w Lock Wait seconds (%d) out of range, 1-30.\n",programName,yLockWait);
                    exit(EXIT_FAILURE);
                }
                log_message(debug_flag | DEBUG_SYSLOG, "yLockWait = %d, count_param = %d", yLockWait, count_param);
                break;
                
            case 'W':
                settle_time = atoi(optarg);
                log_message(debug_flag | DEBUG_SYSLOG, "settle_time = %d, count_param = %d", settle_time, count_param);
                break;
                
            case 'D':
                command_delay = atoi(optarg);
                log_message(debug_flag | DEBUG_SYSLOG, "command_delay = %d, count_param = %d", command_delay, count_param);
                break;
                
            case 'T':
                time_disp_flag = 1;
                count_param++;
                log_message(debug_flag | DEBUG_SYSLOG, "time_disp_flag = %d, count_param = %d", time_disp_flag, count_param);
                break;
                
            case '?':
                if (isprint (optopt)) {
                    fprintf (stderr, "%s: Unknown option `-%c'.\n", programName, optopt);
                    usage(programName);
                    exit(EXIT_FAILURE);
                } else {
                    fprintf (stderr,"%s: Unknown option character `\\x%x'.\n",programName, optopt);
                    usage(programName);
                    exit(EXIT_FAILURE);
                }
            default:
                fprintf (stderr, "%s: Unknown option `-%c'.\n", programName, optopt);
                usage(programName);
                exit(EXIT_FAILURE);
        }
    }

    log_message(debug_flag, "cmdline=\"%s\"", cmdline);
        
    if (optind < argc) {               /* get serial device name */
        szttyDevice = argv[optind];
     } else {
        log_message(debug_flag, "optind = %d, argc = %d", optind, argc);
        usage(programName);
        fprintf(stderr, "%s: No serial device specified\n", programName);
        exit(EXIT_FAILURE);
    }

    if (compact_flag == 1 && metern_flag == 1) {
        fprintf(stderr, "%s: Parameter -m and -q are mutually exclusive\n", programName);
        usage(programName);
        exit(EXIT_FAILURE);
    }

    lockSer(szttyDevice, PID, debug_flag);

    modbus_t *ctx;
    
    // Baud rate
    if (baud_rate == 0) baud_rate = DEFAULT_RATE;

    // Response timeout
    if (resp_timeout == -1) {
        resp_timeout = 2;       // default 2 = 0.2s
    } else {
        resp_timeout *= 100;    // from 1/10s to ms
        log_message(debug_flag, "resp_timeout=%lums", (long unsigned)resp_timeout);
    }
    resp_timeout *= 1000;  // Convert to microseconds
    log_message(debug_flag, "resp_timeout=%luus", (long unsigned)resp_timeout);
    
    // Command delay
    if (command_delay == -1) {
        command_delay = 0;      // default = no command delay
    } else { 
        command_delay *= 1000;        
        log_message(debug_flag, "command_delay=%ldus", command_delay);
    }

    // Settle time delay
    if (settle_time == -1)
        settle_time = 0;        // default = no settle time
    else {
        settle_time *= 1000;
        log_message(debug_flag, "settle_time=%ldus", settle_time);
    }

    if (stop_bits == 0) {
        if (parity != N_PARITY)
            stop_bits=1;     // Default if parity != N
        else
            stop_bits=2;     // Default if parity == N        
    }

    //--- Modbus Setup start ---
    
    ctx = modbus_new_rtu(szttyDevice, baud_rate, parity, 8, stop_bits);
    if (ctx == NULL) {
        log_message(debug_flag | DEBUG_SYSLOG, "Unable to create the libmodbus context\n");
        ClrSerLock(PID);
        exit(EXIT_FAILURE);
    } else {
        log_message(debug_flag, "Libmodbus context open (%d%s%d)",
                baud_rate,
                (parity == E_PARITY) ? EVEN_parity :
                (parity == N_PARITY) ? NONE_parity :
                ODD_parity,
                stop_bits);
    }

#if LIBMODBUS_VERSION_MAJOR >= 3 && LIBMODBUS_VERSION_MINOR >= 1 && LIBMODBUS_VERSION_MICRO >= 2

    // Considering to get those values from command line
    if (byte_timeout == -1) {
        modbus_set_byte_timeout(ctx, -1, 0);
        log_message(debug_flag, "Byte timeout disabled.");
    } else {
        modbus_set_byte_timeout(ctx, 0, byte_timeout);
        log_message(debug_flag, "New byte timeout: %ds, %dus", 0, byte_timeout);
    }
    modbus_set_response_timeout(ctx, 0, resp_timeout);
    log_message(debug_flag, "New response timeout: %ds, %dus", 0, resp_timeout);

#else

    struct timeval timeout;

    if (byte_timeout == -1) {
        timeout.tv_sec = -1;
        timeout.tv_usec = 0;
        modbus_set_byte_timeout(ctx, &timeout);
        log_message(debug_flag, "Byte timeout disabled.");
    } else {
        timeout.tv_sec = 0;
        timeout.tv_usec = byte_timeout;
        modbus_set_byte_timeout(ctx, &timeout);
        log_message(debug_flag, "New byte timeout: %ds, %dus", timeout.tv_sec, timeout.tv_usec);
    }
    
    timeout.tv_sec = 0;
    timeout.tv_usec = resp_timeout;
    modbus_set_response_timeout(ctx, &timeout);
    log_message(debug_flag, "New response timeout: %ds, %dus", timeout.tv_sec, timeout.tv_usec);

#endif

    modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_NONE);
    
    if (settle_time) {
      // Wait for line settle
      log_message(debug_flag, "Sleeping %ldus for line settle...", settle_time);
      usleep(settle_time);
    }
    
    if (trace_flag == 1) {
        modbus_set_debug(ctx, 1);
    }

    modbus_set_slave(ctx, device_address);

    if (modbus_connect(ctx) == -1) {
        log_message(DEBUG_STDERR | DEBUG_SYSLOG, "Connection failed: (%d) %s\n", errno, modbus_strerror(errno));
        modbus_free(ctx);
        ClrSerLock(PID);
        exit(EXIT_FAILURE);
    }

    float voltage     = 0;
    float current     = 0;
    float power       = 0;
    float apower      = 0;
    float rapower     = 0;
    float pf          = 0;
    float pangle      = 0;
    float freq        = 0;
    float imp_energy  = 0;
    float exp_energy  = 0;
    float tot_energy  = 0;
    float impr_energy = 0;
    float expr_energy = 0;
    float totr_energy = 0;
    int   time_disp   = 0;

    // =============================================
    // GESTIONE SCRITTURA PARAMETRI TAC1100
    // =============================================
    
    // Verifica mutua esclusione tra parametri di scrittura
    if (new_address > 0 && new_baud_rate >= 0) {
        log_message(DEBUG_STDERR, "Parameter -s and -r are mutually exclusive\n\n");
        usage(programName);
        exit_error(ctx);
    } else if ((new_address > 0 || new_baud_rate >= 0) && new_parity_stop >= 0) {
        log_message(DEBUG_STDERR, "Parameter -s, -r and -N are mutually exclusive\n\n");
        usage(programName);
        exit_error(ctx);
    }
    
    if (new_address > 0) {
        // Change Meter Address
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting new meter address to %d", new_address);
            changeConfigUINT(ctx, DEVICE_ID, new_address, RESTART_FALSE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (new_baud_rate >= 0) {
        // Change Baud Rate
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting new baud rate to %d", new_baud_rate);
            changeConfigUINT(ctx, BAUD_RATE, new_baud_rate, RESTART_FALSE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (new_parity_stop >= 0) {
        // Change Parity/Stop bits
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting new parity/stop to %d", new_parity_stop);
            changeConfigUINT(ctx, NPARSTOP, new_parity_stop, RESTART_TRUE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (password_flag > 0) {
        // Set Password
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting password to %d", password_value);
            
            // Verifica se la password corrente è stata fornita
            if (!current_password_set) {
                fprintf(stderr, "\nERROR: Current password required for KPPA authorization.\n");
                fprintf(stderr, "Usage: %s -Q <current_password> -K <new_password> /dev/ttyUSB0\n", programName);
                fprintf(stderr, "Example: %s -Q 0000 -K 1234 /dev/ttyUSB0\n\n", programName);
                modbus_close(ctx);
                modbus_free(ctx);
                ClrSerLock(PID);
                exit(EXIT_FAILURE);
            }
            
            // Abilita KPPA con la password corrente
            printf("Enabling KPPA authorization with current password...\n");
            if (enableKPPA(ctx, current_password) == -1) {
                fprintf(stderr, "\nERROR: Failed to enable KPPA.\n");
                fprintf(stderr, "Please verify:\n");
                fprintf(stderr, "  1. Current password (-Q) is correct (default: 0000)\n");
                fprintf(stderr, "  2. Meter is accessible via Modbus\n");
                fprintf(stderr, "  3. No communication errors\n\n");
                modbus_close(ctx);
                modbus_free(ctx);
                ClrSerLock(PID);
                exit(EXIT_FAILURE);
            }
            
            printf("KPPA enabled. Changing password...\n");
            
            // Ora scrivi la nuova password
            changeConfigUINT(ctx, PASSWORD, password_value, RESTART_FALSE);
            
            printf("\nPassword changed successfully from %d to %d.\n", current_password, password_value);
            printf("IMPORTANT: Remember your new password!\n");
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (demand_period_flag > 0) {
        // Set Demand Period
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting demand period to %d minutes", demand_period);
            changeConfigUINT(ctx, DEMAND_PERIOD, demand_period, RESTART_FALSE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (slide_time_flag > 0) {
        // Set Slide Time
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting slide time to %d", slide_time);
            changeConfigUINT(ctx, SLIDE_TIME, slide_time, RESTART_FALSE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (scroll_time_flag > 0) {
        // Set Automatic Scroll Display Time
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting automatic scroll display time to %d seconds", scroll_time);
            changeConfigUINT(ctx, TIME_DISP, scroll_time, RESTART_FALSE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (backlit_time_flag > 0) {
        // Set Backlit Time
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Setting backlit time to %d minutes", backlit_time);
            changeConfigUINT(ctx, BACKLIT_TIME, backlit_time, RESTART_FALSE);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
        
    } else if (reset_hist_flag > 0) {
        // Reset Historical Data
        if (count_param > 0) {
            usage(programName);
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            exit(EXIT_FAILURE);
        } else {
            log_message(debug_flag, "Resetting historical data, type %d", reset_hist_type);
            
            // Verifica se la password corrente è stata fornita
            if (!current_password_set) {
                fprintf(stderr, "\nERROR: Current password required for KPPA authorization.\n");
                fprintf(stderr, "Usage: %s -Q <current_password> -H <reset_type> /dev/ttyUSB0\n", programName);
                fprintf(stderr, "Example: %s -Q 0000 -H 0 /dev/ttyUSB0\n", programName);
                fprintf(stderr, "\nReset types:\n");
                fprintf(stderr, "  0 = Reset maximum demand\n");
                fprintf(stderr, "  8 = Reset monthly energy consumption\n");
                fprintf(stderr, "  9 = Reset daily energy consumption\n\n");
                modbus_close(ctx);
                modbus_free(ctx);
                ClrSerLock(PID);
                exit(EXIT_FAILURE);
            }
            
            // Abilita KPPA con la password corrente
            printf("Enabling KPPA authorization with password...\n");
            if (enableKPPA(ctx, current_password) == -1) {
                fprintf(stderr, "\nERROR: Failed to enable KPPA.\n");
                fprintf(stderr, "Please verify:\n");
                fprintf(stderr, "  1. Password (-Q) is correct (default: 0000)\n");
                fprintf(stderr, "  2. Meter is accessible via Modbus\n");
                fprintf(stderr, "  3. No communication errors\n\n");
                modbus_close(ctx);
                modbus_free(ctx);
                ClrSerLock(PID);
                exit(EXIT_FAILURE);
            }
            
            printf("KPPA enabled. Sending reset command...\n");
            
            // RESET_HIST richiede KPPA authorization
            changeConfigUINT(ctx, RESET_HIST, reset_hist_type, RESTART_FALSE);
            
            printf("\nHistorical data reset command executed successfully.\n");
            printf("Reset type: ");
            switch(reset_hist_type) {
                case 0: printf("Maximum demand reset\n"); break;
                case 8: printf("Monthly energy consumption reset\n"); break;
                case 9: printf("Daily energy consumption reset\n"); break;
            }
            modbus_close(ctx);
            modbus_free(ctx);
            ClrSerLock(PID);
            return 0;
        }
    }
    
    // =============================================
    // IMPOSTAZIONE FLAG DI LETTURA SE NESSUN PARAMETRO SPECIFICATO
    // =============================================
    
    if (power_flag   == 0 &&
        apower_flag  == 0 &&
        rapower_flag == 0 &&
        volt_flag    == 0 &&
        current_flag == 0 &&
        pf_flag      == 0 &&
        pangle_flag  == 0 &&
        freq_flag    == 0 &&
        export_flag  == 0 &&
        import_flag  == 0 &&
        total_flag   == 0 &&
        rexport_flag == 0 &&
        rimport_flag == 0 &&
        rtotal_flag  == 0 &&
        time_disp_flag == 0
    ) {
        // if no parameter, retrieve all values
        power_flag   = 1;
        apower_flag  = 1;
        rapower_flag = 1;
        volt_flag    = 1;
        current_flag = 1;
        pangle_flag  = 1;
        freq_flag    = 1;
        pf_flag      = 1;
        export_flag  = 1;
        import_flag  = 1;
        total_flag   = 1;
        rexport_flag  = 1;
        rimport_flag  = 1;
        rtotal_flag   = 1;
        count_param  = power_flag + apower_flag + rapower_flag + volt_flag + 
                       current_flag + pangle_flag + freq_flag + pf_flag + 
                       export_flag + import_flag + total_flag +
                       rexport_flag + rimport_flag + rtotal_flag;
    }

    // =============================================
    // LETTURA PARAMETRI
    // =============================================
    
    if (volt_flag == 1) {
        voltage = getMeasureFloat(ctx, VOLTAGE, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_V(%3.2f*V)\n", device_address, voltage);
        } else if (compact_flag == 1) {
            printf("%3.2f ", voltage);
        } else {
            printf("Voltage: %3.2f V \n",voltage);
        }
    }

    if (current_flag == 1) {
        current  = getMeasureFloat(ctx, CURRENT, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_C(%3.2f*A)\n", device_address, current);
        } else if (compact_flag == 1) {
            printf("%3.2f ", current);
        } else {
            printf("Current: %3.2f A \n",current);
        }
    }

    if (power_flag == 1) {
        power = getMeasureFloat(ctx, POWER, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_P(%3.2f*W)\n", device_address, power);
        } else if (compact_flag == 1) {
            printf("%3.2f ", power);
        } else {
            printf("Power: %3.2f W \n", power);
        }
    }

    if (apower_flag == 1) {
        apower = getMeasureFloat(ctx, RAPOWER, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_VA(%3.2f*VA)\n", device_address, apower);
        } else if (compact_flag == 1) {
            printf("%3.2f ", apower);
        } else {
            printf("Apparent Power: %3.2f VA \n", apower);
        }
    }

    if (rapower_flag == 1) {
        rapower = getMeasureFloat(ctx, APOWER, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_VAR(%3.2f*VAR)\n", device_address, rapower);
        } else if (compact_flag == 1) {
            printf("%3.2f ", rapower);
        } else {
            printf("Reactive Power: %3.2f VAR \n", rapower);
        }
    }

    if (pf_flag == 1) {
        pf = getMeasureFloat(ctx, PFACTOR, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_PF(%3.2f*F)\n", device_address, pf);
        } else if (compact_flag == 1) {
            printf("%3.2f ", pf);
        } else {
            printf("Power Factor: %3.2f \n", pf);
        }
    }

    if (pangle_flag == 1) {
        pangle = getMeasureFloat(ctx, PANGLE, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_PA(%3.2f*Dg)\n", device_address, pangle);
        } else if (compact_flag == 1) {
            printf("%3.2f ", pangle);
        } else {
            printf("Phase Angle: %3.2f Degree \n", pangle);
        }
    }

    if (freq_flag == 1) {
        freq = getMeasureFloat(ctx, FREQUENCY, num_retries, 2);
        read_count++;
        if (metern_flag == 1) {
            printf("%d_F(%3.2f*Hz)\n", device_address, freq);
        } else if (compact_flag == 1) {
            printf("%3.2f ", freq);
        } else {
            printf("Frequency: %3.2f Hz \n", freq);
        }
    }

    if (import_flag == 1) {
        imp_energy = getMeasureFloat(ctx, IAENERGY, num_retries, 2) * 1000;
        read_count++;
        if (metern_flag == 1) {
            printf("%d_IE(%d*Wh)\n", device_address, (int)imp_energy);
        } else if (compact_flag == 1) {
            printf("%d ", (int)imp_energy);
        } else {
            printf("Import Active Energy: %d Wh \n", (int)imp_energy);
        }
    }

    if (export_flag == 1) {
        exp_energy = getMeasureFloat(ctx, EAENERGY, num_retries, 2) * 1000;
        read_count++;
        if (metern_flag == 1) {
            printf("%d_EE(%d*Wh)\n", device_address, (int)exp_energy);
        } else if (compact_flag == 1) {
            printf("%d ", (int)exp_energy);
        } else {
            printf("Export Active Energy: %d Wh \n", (int)exp_energy);
        }
    }

    if (total_flag == 1) {
        tot_energy = getMeasureFloat(ctx, TAENERGY, num_retries, 2) * 1000;
        read_count++;
        if (metern_flag == 1) {
            printf("%d_TE(%d*Wh)\n", device_address, (int)tot_energy);
        } else if (compact_flag == 1) {
            printf("%d ", (int)tot_energy);
        } else {
            printf("Total Active Energy: %d Wh \n", (int)tot_energy);
        }
    }

    if (rimport_flag == 1) {
        impr_energy = getMeasureFloat(ctx, IRAENERGY, num_retries, 2) * 1000;
        read_count++;
        if (metern_flag == 1) {
            printf("%d_IRE(%d*VARh)\n", device_address, (int)impr_energy);
        } else if (compact_flag == 1) {
            printf("%d ", (int)impr_energy);
        } else {
            printf("Import Reactive Energy: %d VARh \n", (int)impr_energy);
        }
    }

    if (rexport_flag == 1) {
        expr_energy = getMeasureFloat(ctx, ERAENERGY, num_retries, 2) * 1000;
        read_count++;
        if (metern_flag == 1) {
            printf("%d_ERE(%d*VARh)\n", device_address, (int)expr_energy);
        } else if (compact_flag == 1) {
            printf("%d ", (int)expr_energy);
        } else {
            printf("Export Reactive Energy: %d VARh \n", (int)expr_energy);
        }
    }

    if (rtotal_flag == 1) {
        totr_energy = getMeasureFloat(ctx, TRENERGY, num_retries, 2) * 1000;
        read_count++;
        if (metern_flag == 1) {
            printf("%d_TRE(%d*VARh)\n", device_address, (int)totr_energy);
        } else if (compact_flag == 1) {
            printf("%d ", (int)totr_energy);
        } else {
            printf("Total Reactive Energy: %d VARh \n", (int)totr_energy);
        }
    }

    if (time_disp_flag == 1) {
        time_disp = getConfigUINT(ctx, TIME_DISP, num_retries, 1);
        read_count++;
        if (compact_flag == 1) {
            printf("%d ", (int) time_disp);
        } else {
            printf("Automatic scroll display time: %d seconds\n", (int) time_disp);
        }
    }

    if (read_count == count_param) {
        modbus_close(ctx);
        modbus_free(ctx);
        ClrSerLock(PID);
        free(devLCKfile);
        free(devLCKfileNew);
        free(PARENTCOMMAND);
        if (!metern_flag) printf("OK\n");
    } else {
        exit_error(ctx);
    }

    return 0;
}

#ifdef __cplusplus
}
#endif
