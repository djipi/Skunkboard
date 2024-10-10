/* jcp2.c : Move data to Jaguar by way of EZ-HOST RDB connection 
	Skunkboard support - http://www.harmlesslion.com

Design:
	Data is copied at peak speed via RDB into two buffers in EZ memory (at $2800 and $1800)
	The 68K polls EZ memory for new buffers, copying them into Jaguar memory as they appear

Memory map relative to base address $2800:
	$37E0:	Base address (in 68K memory) for this block
	$37E4:	Address where 68K execution begins once this block is complete
			-1 if more blocks will follow (i.e., stay in the reader loop)
	$37E8:	Base address of next block in EZ-HOST (usually $1800)
	$37EA:	Amount (in bytes) to copy (up to 4064 bytes)
			-1 if the block is not ready for copying
			The 68K sets this at $2800 and $1800 before transfer begins
			After each block is moved, the 68K sets this to -1 again. Some
			sequences are bi-directional.

	Note:   the values FxFF, excluding FFFF, are reserved as flag values
			for future, incompatible versions of JCP, to allow detection.

	OTG has 16k of internal RAM:
	0000 - 04A3 - Interrupt vectors, HW Registers and USB buffers
	04A4 - 0FEF - Free, reserved for GDB
	0FF0 - 102B - turbow (accelerated USB block copy stub)
	102C - 17FF - Free
	1800 - 27FF - Transmit buffer 1
	2800 - 37FF - Transmit buffer 2
	3800 - 3FFF - Free
	4000 - BFFF - external memory (not implemented)
	C000 - C0FF - control registers
	C100 - DFFF - external memory (not implemented)
	E000 - FFFF - EZ-Host ROM BIOS 

SB Rev 2:
	Special flags are added to indicate bank instructions:

	During flash, bits are added to the erase block count:
	0x80000000;		// set high bit to flag it for slow (word) write - doesn't require 9v (on SB rev 1 too)
	0x40000000;		// set next bit to flag bank 2 (instead of bank 1)

	During boot, bits are added to the start address:
	0x10000000;		// set bank 2 (instead of bank 1)
	0x70000000;		// set 6MB mode (signed, avoid high bit)

Observations:
	 The X command seems to have a limit of 4080 bytes per transfer
		This applies even with escaping!  (Which makes 9120 bytes...)
	 Roundtrips hurt -- compare 10 seconds/megabyte @ 4080 to 13 seconds/megabyte @ 2048
	 We currently use 'middle endian' because the CPLD does not byteswap 'data regions'

Lots and lots of tweaks by Tursi, sorry, not all documented, though I've updated what
I changed above.

This program and associated binaries are copyright by Mike Brent, http://www.harmlesslion.com
Commercial use prohibited. All rights reserved. Copyright 2009 by Mike Brent.

linux link with -lusb and -lrt - requires libusb to be installed!
Apple link with only -lusb

May require root priviledges to see the device.

Thanks to Belboz for the OSX patch!
Thanks to SebRmv for ideas/fixes in the Skunklib support code

Additional coding by Dilinger @ AA (2014-2020), no Copyright has been broken.
*/

/* if you don't want to include the BIOS upgrades, comment this out */
/* the upgrade for rev 3 boards also works fine on rev 2 */
#define INCLUDE_BIOS_10204
#define INCLUDE_BIOS_30002
/* uncomment this to try automatic mode (experimental) */
// JCP_AUTO doesn't work too well at the moment.. need to rearchitect. 
// should detect the filetype FIRST, then decide how to upload it. Too
// hacky trying to guess first.
//#define JCP_AUTO

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#ifndef __APPLE__
#include <malloc.h>
#endif
#include <errno.h>
#include <time.h>
#if defined(WIN32) || defined(WIN64)
#include <process.h>
#ifdef LIBUSB_1
#include "libusb-1.0/libusb.h"
#else
#include "winusb.h"
#endif
#else
#include <unistd.h>
#include <usb.h>
#include <sys/time.h>
#endif
#include "turbow.h"
#include "univbin.h"
#include "romdump.h"
#include "flashstub.h"
#include "dumpver.h"
#include "standard_values.h"
#include "readver.h"
#ifdef INCLUDE_BIOS_10204
#include "upgrade10204.h"
#endif
#ifdef INCLUDE_BIOS_30002
#include "upgrade30002.h"
#endif
#ifdef REMOVERS
#ifndef _MSC_VER
#pragma message "Enabling Removers extensions."
#endif
#include "jcp_handler.h"
#endif

#if defined(INCLUDE_BIOS_10204) || defined(INCLUDE_BIOS_30002)
#define JCP_U_VERSION "[-U]"
#else
#define JCP_U_VERSION ""
#endif

#ifdef WIN64
#define	PLATFORM_NAME	"x64"
#else
#ifdef WIN32
#define	PLATFORM_NAME	"Win32"
#endif
#endif

/* version major.minor.rev */
#define JCP2VERSION 0x020800
#define	JCP2_VERSION	"2.08.00"
/* ROM based address that we can blindly send dummy data to */
#define DUMMYBASE 0xFFE000
/* size of the work buffer (maximum ROM size plus slack) */
#define BUFSIZE (6*1024*1024+0x2000)
/* size of a RAM buffer (for ELF file loading) */
#define RAMBUFSIZE (2*1024*1024)

#define	ENBIGEND(_x) (((_x)[0] << 24) + ((_x)[1] << 16) + ((_x)[2] << 8) + (_x)[3])
#define	HALFBIGEND(_x) (((_x)[0] << 8) + (_x)[1])
#define	ENMIDEND(_x) (((_x)[0] << 16) + ((_x)[1] << 24) + ((_x)[2]) + ((_x)[3] << 8) )
#define	HALFLITTLEEND(_x) (((_x)[0]) + ((_x)[1] << 8) )

/* data for the SerialBig function */
char *szNumDat[16] = {
	" XXX  ",
	"X   X ",
	"  X   ",
	"XXX   ",
	"XXXXX ",
	"    X ",
	"  XX  ",
	" X    ",
	"X     ",
	"  XX  ",
	"X  X  ",
	"   X  ",
	" XXX  ",
	"XXXX  ",
	" XXXX ",
    "      ",
};

int nDigits[12][7] = {
	{ 0,1,1,1,1,1,0 },
	{ 2,3,2,2,2,2,4 },
	{ 12,1,5,6,7,8,4 },
	{ 0,1,5,9,5,1,0 },
	{ 8,10,10,4,11,11,11 },
	{ 4,8,8,0,5,1,0 },
	{ 0,8,8,13,1,1,0 },
	{ 4,5,5,11,2,2,2 },
	{ 0,1,1,0,1,1,0 },
	{ 0,1,1,14,5,5,0 },
	// space, accessed as -1
	{ 15,15,15,15,15,15,15 },
	// period, accessed as -2
	{ 15,15,15,15,15,15,6 }
};

#ifndef uchar
#define uchar unsigned char
#endif
#ifndef bool
#define bool int
#endif
#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif

#if !defined(WIN32) && !defined(WIN64)
/* linux compatibility with Windows terms */
#define DWORD unsigned int
#define _stricmp strcasecmp
#define Sleep(x) usleep(x*1000)
#define _execlp execlp

/* returns a count in ms - this one should be osx compat */
DWORD GetTickCount() {
	struct timeval now;
	DWORD ret;

	int rv = gettimeofday(&now, NULL);

	if (rv != 0) {
		return 0;	/* we got nothing */
	}

	ret=now.tv_sec*1000+(now.tv_usec/1000);
	return ret;
}
#endif

#ifdef LIBUSB_1
libusb_device_handle *findEZ(bool fInstallTurboW, bool fAbortOnFail);
#else
usb_dev_handle *findEZ(bool fInstallTurboW, bool fAbortOnFail);
#endif
void Reattach(void);
void bye(char* msg);
void SendFile(int flen, uchar *fptr, int curbase, int base);
int  DoFile(uchar *fdata, int base, int flen, int skip, bool builtin);
void LockBothBuffers(void);
bool TestIfBuffersLocked(void);
void WaitForBothBuffers(void);
void DoResetAndReconnect(bool bForce);
void DoResetAndBoot(void);
void DoFlash(int nLen);
void DoDump(char *pszName);
void DoSerialInfo(void);
void DoSerialBig(void);
void DoBiosUpdate(void);
void DoReset(void);
void HandleConsole(void);
void FilenameSanitize(char *buf);
int ParseAddress(const char *pBuf);
int HandleTransfer(uchar *fdata, int base, int flen, int skip, bool part2of6mb);
void CatVal(char *szOut, int nBufLen, int nVal, int nRow);
bool DetermineFileInfo(bool bMute, uchar *fdata, int *base, int *flen, int *skip);

/* globals */
int nextez = 0x1800;
#ifdef LIBUSB_1
libusb_device_handle *udev = NULL;
libusb_context *ctx;
#else
usb_dev_handle *udev = NULL;
#endif
uchar *fdata = NULL;
char g_szFilename[256];
FILE *fp=NULL;
bool g_FirstFileSent=false;
char g_pszExtShell[256];
int nSpinner = 0;
char szSpin[] = "\\|/-";
char USBBusName[10];					/* USB bus name to use */
int USBBus = 0;							/* USB bus */
int USBPort = 0;						/* USB port */
unsigned short SkunkboardSerial = 0;	/* Skunkboard serial number */
int ComTimeout = 1000;					/* Communications timeout */

bool g_OptDoFlash=false;
bool g_OptDoSlowFlash=false;
bool g_OptOverrideFlash=false;
bool g_OptEraseAllBlocks=false;
bool g_OptDoDump=false;
bool g_OptFlashActive=false;
bool g_OptNoBoot=false;
bool g_OptOnlyBoot=false;
bool g_OptOverride=false;
bool g_OptConsole=false;
bool g_OptOnlyConsole=false;
bool g_OptConsoleUp=false;			/* set to true when console actually starts */
bool g_OptSilentConsole=false;
bool g_OptVerbose=false;
bool g_OptAutoMode=false;			/* used to enable JCP auto mode - this is experimental */
int nCartBank=0;					/* 0=bank 1, 1=bank 2, -1=6MB mode (latter 2 not on SB rev 1) */
bool g_SixMegWrite=false;			/* used to help systems that need to know when it's 6mb! */
int  g_HeaderSkip=0;				/* number of bytes to skip in header - manual set */
bool g_OptQuietMode=false;			/* Quiet mode for skunkGUI. no spin and extra text output */
bool g_skipwait=false;				/* used during file transfer to skip waiting for boot signature */
bool g_OptDoReset = false;
bool g_OptDoSerialInfo = false;
bool g_OptDoSerialBig = false;


/* Main function - entry point */
int main(int argc, char* argv[])
{
	int	base, flen, skip;
	int	nArg;
	int	nPos;
	FILE *fp;
	int	nUsed;
	bool fExitLoop;
	bool bOldConsole;
#ifdef LIBUSB_1
	const struct libusb_version *libusbver;
#endif

	// Display version
#if defined(WIN32) || defined(WIN64)
	printf("\njcp2 v%s - %s - built on %s\n", JCP2_VERSION, PLATFORM_NAME, __DATE__);
#else
	printf("jcp2 v%02X.%02X.%02X built on %s\n", ((JCP2VERSION&0xFF0000)>>16), ((JCP2VERSION&0xFF00)>>8), (JCP2VERSION&0xFF), __DATE__);
#endif
#ifdef REMOVERS
	printf("Compiled with the Removers extensions\n");
#endif
#ifndef LIBUSB_1
	printf("Use libusb 0.1\n\n");
#else
	libusbver = libusb_get_version();
	printf("Use libusb %i.%i.%i.%i%s    %s\n\n", libusbver->major, libusbver->minor, libusbver->micro, libusbver->nano, libusbver->rc, libusbver->describe);
#endif

	// Display options & arguments
	if ((argc<2) || ((argc>1) && (strchr(argv[1],'?'))))
	{
		printf("jcp2 [-?] [-2|6] [-b] [-c] [-d] [-e] [-f] [-h={count}] [-n] [-o] [-q] [-r] [-s]\n");
		printf("     [-serial=xxxx] [-t={value}] %s [-ubus={1|..}] [-uport={0|..}] [-w]\n", JCP_U_VERSION);
		printf("     [-x={external console}] [filename] [{$|0x}base]\n");
		printf("\nValues by default\n");
		printf("Skunkboard memory bank set as 1\n");
		printf("$base, or 0xbase, set as $4000\n");
		printf("Communication timeout set as 1000\n");
		printf("filename set as NULL\n");
		printf("\nArguments without parameters - can also be attached together (i.e: -ef, etc.)\n");
		printf("-? : This display information (optional)\n");
		printf("-2 : Use Skunkboard memory bank 2 instead of bank 1\n");
		printf("-6 : Use the 6MB mode instead of the banked mode\n");
		printf("-b : Boot address with {$base}\n");
		printf("-c : Launch console (incompatible with the '-n' option)\n");
		printf("-d : Dump Skunkboard memory flash to filename\n");				// , then reset the Skunkboard
		printf("-e : Erase whole Skunkboard memory flash\n");
		printf("-f : Flash {filename} to Skunkboard memory bank at {$base (default: $802000)}\n");
		printf("-n : No boot after the Skunkboard memory flash\n");
		printf("-o : Override address (pass filename and base)\n");
		printf("-q : Quiet mode (useful for SkunkGUIs)\n");
		printf("-r : Reset the Jaguar only\n");
		printf("-s : Display only Skunkboard version and his serial number\n");
#if defined(INCLUDE_BIOS_10204) || defined(INCLUDE_BIOS_30002)
		printf("-U : Upgrade Skunkboard BIOS, then reset the Jaguar\n");
#endif
		printf("-w : Word flash (slow flash operation, to be used if '-f' alone fails)\n");
		printf("\nArguments with parameters\n");
		printf("-h={count}            : Override the header skip count\n");
		printf("-serial={xxxx}        : Use Skunkboard serial number (4 digits) to connect\n");
		printf("-t={value}            : Communication timeout (must be above 0)\n");
		printf("-ubus={1|..}          : Force USB bus to be used\n");
		printf("-uport={0|..}         : Force USB port to be used\n");
		printf("-x={external console} : Shell to external console application\n");
		printf("\nUndocumented arguments\n");
		printf("-! : Override flash\n");
		printf("-* : Display the Skunkboard version and his serial number as a banner form\n");
		printf("-v : Verbose mode\n");
		printf("\n");
	}
	else
	{
#ifdef LIBUSB_1
		if (!libusb_init(&ctx))
		{
			libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_NONE);
#endif
			// Default basic initialization
			fdata = (uchar*)malloc(BUFSIZE);	// 6MB + header
			memset(fdata, 0, BUFSIZE);
			base = 0x4000;
			flen = 0;
			skip = 0;
			strcpy(g_szFilename, "");
			strcpy(g_pszExtShell, "");
			strcpy(USBBusName, "");
#ifdef JCP_AUTO
			g_OptAutoMode = true;
#endif

			// loop on arguments option
			nArg = 0;
			while (++nArg < argc)
			{
				// option detection ?
				if (argv[nArg][0] == '-')
				{
					nPos = 1;
					fExitLoop = false;

					// arguments options detection
					while ((argv[nArg][nPos]) && !fExitLoop)
					{
						switch (argv[nArg][nPos++])
						{
							// quiet mode (for SkunkGUI)
						case 'q':
							g_OptQuietMode = true;
							break;

							// verbose mode
						case 'v':
							g_OptVerbose = true;
							break;

							// flash {filename} to Skunkboard
						case 'f':
							g_OptDoFlash = true;
							base = 0x802000;
							break;

							// Word flash
						case 'w':
							g_OptDoSlowFlash = true;
							break;

							// Erase whole Skunkboard memory flash
						case 'e':
							g_OptEraseAllBlocks = true;
							break;

							// Dump Skunkboard memory flash
						case 'd':
							g_OptDoDump = true;
							break;

							// Reset the Jaguar
						case 'r':
							g_OptDoReset = true;
							break;

							// No boot after the Skunkboard memory flash
						case 'n':
							g_OptNoBoot = true;
							break;

							// Boot address
						case 'b':
							g_OptOnlyBoot = true;
							break;

							// Override address
						case 'o':
							g_OptOverride = true;
							break;

							// Launch console
						case 'c':
							g_OptConsole = true;
							break;

							// -s : Display Skunkboard version & serial info
							// -serial= : Use Skunkboard version
						case 's':
							if (!argv[nArg][nPos])
							{
								g_OptDoSerialInfo = true;
							}
							else
							{
								if (!strncmp(&argv[nArg][nPos], "erial=", 6))
								{
									if (strlen(&argv[nArg][nPos + 6]) == 4)
									{
										SkunkboardSerial = (argv[nArg][nPos + 6] - '0') << 12;
										SkunkboardSerial |= (argv[nArg][nPos + 7] - '0') << 8;
										SkunkboardSerial |= (argv[nArg][nPos + 8] - '0') << 4;
										SkunkboardSerial |= (argv[nArg][nPos + 9] - '0');
										fExitLoop = true;
									}
									else
									{
										bye("Error: Serial number must be in 4 digits");
									}
								}
								else
								{
									bye("Error: Option is not -s or -serial either");
								}
							}
							break;

							// Communication timeout
						case 't':
							if ((ComTimeout = atoi((char *)&(argv[nArg][nPos + 1]))) <= 0)
							{
								bye("Error: Communication timeout must be above 0");
							}
							else
							{
								fExitLoop = true;
							}
							break;

							// USB port
						case 'u':
							if (argv[nArg][nPos])
							{
								if (!strncmp(&argv[nArg][nPos], "port=", 5))
								{
									USBPort = atoi(&argv[nArg][nPos + 5]);
									fExitLoop = true;
								}
								else
								{
									if (!strncmp(&argv[nArg][nPos], "bus=", 4))
									{
										sprintf(USBBusName, "bus-%s", &argv[nArg][nPos + 4]);
										USBBus = atoi(&argv[nArg][nPos + 4]);
										fExitLoop = true;
									}
									else
									{
										bye("Error: Option is not -uport or -ubus either");
									}
								}
							}
							break;

							// BIOS update
						case 'U':
#if defined(INCLUDE_BIOS_10204) || defined(INCLUDE_BIOS_30002)
							DoBiosUpdate();
							Sleep(100);
							DoReset();
							bye("");
#endif
							break;

							// Override flash
							// undocumented! Don't require a flash even if in flash range
						case '!':
							g_OptOverrideFlash = true;
							break;

							// undocumented! banner serial! Used in test script. :)
						case '*':
							g_OptDoSerialBig = true;
							break;

							// bank 2 selected
						case '2':
							nCartBank = 1;
							printf("Using bank 2\n");
							break;

							// bank 6 selected
						case '6':
							nCartBank = -1;
							g_SixMegWrite = true;
							printf("Using 6MB flash mode\n");
							break;

							// get the forced header offset
						case 'h':
							if (argv[nArg][nPos] == '=')
							{
								g_HeaderSkip = atoi(&argv[nArg][++nPos]);
								fExitLoop = true;
							}
							else
							{
								bye("Error: -h option requires the number of bytes to skip");
							}
							break;

							// external console shell
						case 'x':
							if (argv[nArg][nPos] != '=')
							{
								bye("Error: -x option requires external shell filename application to follow");
							}
							else
							{
								g_OptConsole = true;
								strcpy(g_pszExtShell, &argv[nArg][++nPos]);
								fExitLoop = true;
							}
							break;

							// default
						default:
							bye("Error: Unknown option");
						}
					}
				}
				else
				{
					// $base detection
					if (argv[nArg][0] == '$')
					{
						if (1 != sscanf(&argv[nArg][1], "%x", &base))
						{
							printf("Could not scan address '%s'\n", argv[nArg]);
							bye("Error: Failed to parse address.");
						}
					}
					else
					{
						if ((argv[nArg][0] == '0') && (argv[nArg][1] == 'x'))
						{
							if (1 != sscanf(&argv[nArg][2], "%x", &base))
							{
								printf("Could not scan address '%s'\n", argv[nArg]);
								bye("Error: Failed to parse address.");
							}
						}
						else
						{
							strcpy(g_szFilename, &argv[nArg][0]);
						}
					}
				}
			}

			// Display the Bios & Serial in a simple text
			if (g_OptDoSerialInfo)
			{
				DoSerialInfo();
			}
			else
			{
				// Display the Bios & Serial in a banner form
				if (g_OptDoSerialBig)
				{
					DoSerialBig();
				}
				else
				{
					// Do Jaguar Reset
					if (g_OptDoReset)
					{
						DoReset();
					}
					else
					{
						// Boot only not selected
						if (!g_OptOnlyBoot)
						{
							if (!strlen(g_szFilename) && (!g_OptConsole))
							{
								bye("Error: No filename was specified");
							}
							else
							{
								if (!strlen(g_szFilename))
								{
									// user seems to want to try to attach the console, so don't try to load stuff
									if (!g_OptOnlyBoot)
									{
										g_OptOnlyConsole = true;
									}
								}
								else
								{
									if (!g_OptDoDump)
									{
										fp = fopen(g_szFilename, "rb");

										if (NULL == fp || (flen = (int)fread(fdata, 1, BUFSIZE, fp)) < 1)
										{
											bye("Error: Couldn't read file");
										}

										fclose(fp);

										fp = NULL;
									}
									else
									{
										flen = 0;
									}
								}
							}
						}

						// Skunkboard memory flash dump
						if (g_OptDoDump)
						{
							DoDump(g_szFilename);
							bye("Process: Dump complete.");
						}

						// Bit of a hack, preparse the file to figure out its true length and address
						DetermineFileInfo(false, fdata, &base, &flen, &skip);

						// 6MB is not really necessary since the filename is smaller than 4MB
						if ((nCartBank == -1) && (flen <= (4 * 1024 * 1024 - 0x2000)) && (!g_OptOnlyBoot))
						{
							printf("6MB mode not required, will flash bank 1 instead\n");
							nCartBank = 0;
						}

						// 6MB can be selectioned due to the filename size even if 6MB argument is not set
						if ((g_OptAutoMode) && (nCartBank != -1) && (flen > (4 * (1024 * 1024))))
						{
							printf("Assuming 6MB mode...\n");
							nCartBank = -1;
						}

						// Flash
						if (g_OptDoFlash)
						{
							if (flen == 0)
							{
								bye("Error: File must be specified with flash!");
							}
							else
							{
								if (g_OptNoBoot)
								{
									printf("Warning: -n (no boot) option not supported during flashing\n");
									g_OptNoBoot = false;
								}

								// this is an estimate, headers may make it possible, or the 2k bios
								// gap may make it impossible!
								if ((flen > (4 * (1024 * 1024))) && (-1 != nCartBank))
								{
									bye("Warning: File is too large to be flashed to a 4MB bank, try 6MB mode\n");
								}
							}
						}

						// we handle 6MB mode as two separate uploads, since we can't run it directly
						if (nCartBank == -1)
						{
							if (!g_OptOnlyBoot)
							{
								bOldConsole = g_OptConsole;
								// tweak things up a bit to make this work
								printf("6MB mode will require two transfers...\n");
								nCartBank = 0;
								g_OptNoBoot = true;
								g_OptConsole = false;
								nUsed = HandleTransfer(fdata, base, flen, skip, false);
								WaitForBothBuffers();

								printf("Flashing second bank...\n");
								nCartBank = 1;
								g_OptConsole = bOldConsole;
								g_OptFlashActive = false;	// this is necessary because we have to load the flasher stub again
								HandleTransfer(fdata + nUsed, 0x800000, flen - nUsed, 0, true);
								WaitForBothBuffers();
							}

							printf("Requesting start...\n");
							g_OptNoBoot = false;
							g_OptOnlyBoot = true;
							nCartBank = -1;
							DoFile(fdata, base, 0, 0, true);
						}
						else
						{
							HandleTransfer(fdata, base, flen, skip, false);
						}
					}
				}
			}

			free(fdata);
			fdata = NULL;

			if (NULL != udev)
			{
#ifdef LIBUSB_1
				libusb_close(udev);
#else
				usb_close(udev);
#endif
			}
#ifdef LIBUSB_1
			libusb_exit(ctx);
		}
#endif
	}
	
	return 0;
}


/* handles the transfer and the flash portion */
/* returns the number of bytes actually processed (not necessarily sent, includes headers) */
int HandleTransfer(uchar *fdata, int base, int flen, int skip, bool part2of6mb)
{
	bool bOldNoBoot;

	if (g_OptDoFlash)
	{
		bOldNoBoot=g_OptNoBoot;		// loading the flash program ALWAYS requires NoBoot to be false
		g_OptNoBoot=false;
		
		DoFlash(flen);

		g_OptNoBoot=bOldNoBoot;
		g_OptFlashActive=true;
	}

	return DoFile(fdata, base, flen, skip, part2of6mb);
}


/* Generate a little text spinner */
void Spin(void)
{
	if (!g_OptQuietMode)
    {
		if (++nSpinner > (sizeof(szSpin)-1))
		{
			nSpinner = 0;
		}

		putchar(szSpin[nSpinner]);
		putchar('\b');
		fflush(stdout);
	}
}


/* reset the Jaguar then reconnect to it - pass true not to wait on the buffers */
void DoResetAndReconnect(bool bForce)
{
	if (!bForce) 
	{
		WaitForBothBuffers();	// make sure the Jag is done the last command
	}

	DoReset();
	Sleep(2000);			// takes the Jag about 2s to come up

	while (NULL == udev)
	{
		Sleep(100);
		udev = findEZ(true, false);
	}

	WaitForBothBuffers();	// when the Jag clears the buffers, we're up

	// reset pointer
	nextez = 0x1800;
}


/* synch reset the Jag, then send a start packet for the cart */
void DoResetAndBoot(void)
{
	DWORD tmp=0;

	DoResetAndReconnect(false);

	// send the fake boot command
	g_OptOnlyBoot = true;
	g_OptFlashActive = false;

	DoFile((uchar*)&tmp, 0x802000, 0, 0, true);
}


/* mark both buffers as blocked - don't use during upload! */
void LockBothBuffers(void)
{
	unsigned short tmp = 0;

	for (;;)
	{
#ifndef LIBUSB_1
		if (usb_control_msg(udev, 0x40, 0xfe, 4080, 0x1800+0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#else
		if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, 0x1800 + 0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#endif
		{
			break;
		}

		Reattach();
	}

	for (;;)
	{
#ifndef LIBUSB_1
		if (usb_control_msg(udev, 0x40, 0xfe, 4080, 0x2800+0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#else
		if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, 0x2800 + 0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#endif
		{
			break;
		}

		Reattach();
	}
}


/* returns true if both buffers are locked */
/* Note: locked diffs from 'in-use' in that locked uses a length of 0 */
bool TestIfBuffersLocked(void)
{
	volatile short poll = 0;
	bool bRet = false;

#ifndef LIBUSB_1
	if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x1800+0xFEA, (char*)&poll, 2, ComTimeout) == 2)
#else
	if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x1800 + 0xFEA, (char*)&poll, 2, ComTimeout) == 2)
#endif
	{
		if (poll == 0)
		{
#ifndef LIBUSB_1
			if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800+0xFEA, (char*)&poll, 2, ComTimeout) == 2)
#else
			if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800 + 0xFEA, (char*)&poll, 2, ComTimeout) == 2)
#endif
			{
				if (poll == 0)
				{
					bRet = true;
				}
			}
			else
			{
				bRet = true;	// say locked for now
				Reattach();
			}
		}
	}
	else
	{
		bRet = true;	// say locked for now
		Reattach();
	}

	return bRet;
}


/* wait for the Jag to mark both buffers as free */
void WaitForBothBuffers(void)
{
	volatile short poll = 0;

	// first buffer
	do
	{
		Spin();

#ifndef LIBUSB_1
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x1800+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x1800 + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		}
		else
		{
			Sleep(100);
		}
	}
	while (-1 != poll);

	printf(".");
	
	// second buffer
	do
	{
		Spin();

#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800 + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		}
		else
		{
			Sleep(100);
		}
	}
	while (-1 != poll);

	printf(".\n");
}


/* Reset the Jaguar */
void DoReset(void)
{
	// We have to use scan mode to access registers (TurboWrite uses DMA engine)
	unsigned char cmd[10] = {	0xB6, 0xC3, 0x04, 0x00, 0x00, 0x28, 0xC0, 0x02, 0x00, 0x00	};

	if ( (g_OptVerbose) || (!g_OptSilentConsole) )
	{
		printf("Resetting jaguar...\n");
	}

	// Reset is 0xc028=2, 0xc028=0
	if (NULL == udev)
	{
		udev = findEZ(true, true);		// we used to not load turbow here, but for better reconnect, we want to lock buffers first!
	}

	LockBothBuffers();		// carries through the reset

	// Send command
#ifdef LIBUSB_1
	if (libusb_control_transfer(udev, 0x40, 0xff, 10, 0x304C, (char*)cmd, 10, ComTimeout) != 10)
#else
	if (usb_control_msg(udev, 0x40, 0xff, 10, 0x304C, (char*)cmd, 10, ComTimeout) != 10)
#endif
	{
		bye("Error: Reset assert failed to send.");
	}
	else
	{
		// brief delay!
		Sleep(50);

		cmd[7] = 0;

		// Send command
#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0x40, 0xff, 10, 0x304C, (char*)cmd, 10, ComTimeout) != 10)
#else
		if (usb_control_msg(udev, 0x40, 0xff, 10, 0x304C, (char*)cmd, 10, ComTimeout) != 10)
#endif
		{
			bye("Error: Reset release failed to send.");
		}
		else
		{
			/* in case it's used elsewhere */
			if (NULL != udev)
			{
#ifdef LIBUSB_1
				libusb_close(udev);
#else
				usb_close(udev);
#endif
			}

			udev = NULL;
		}
	}
}


/* Prepare the Jaguar to receive a flash file */
void DoFlash(int nLen)
{
	volatile short poll=0;
	int idx;
	unsigned int nBlocks;

	g_OptSilentConsole = true; 

	// due to the flash layout, we can't do a straight sector erase
	// we compromise some and erase only half if we don't need it
	// all - that seems to work okay.
	if (nLen <= (2*(1024*1024)))
	{
		nBlocks = 32;
	}
	else
	{
		nBlocks = 62;
	}

	// just in case it's needed
	if (g_OptEraseAllBlocks)
	{
		nBlocks=62;
	}

	// restrict to the legal range
	if (nBlocks > 62)
	{
		nBlocks = 62;
	}
	else
	{
		if (nBlocks < 1)
		{
			nBlocks = 1;
		}
	}

	if (g_OptVerbose)
	{
		printf("Going to erase %d blocks\n", nBlocks);
	}

	if (g_OptDoSlowFlash)
	{
		printf("Using slow flash (experimental...)\n");
		nBlocks |= 0x80000000;	// set high bit to flag it
	}
	
	if (nCartBank == 1)
	{
		nBlocks |= 0x40000000;	// set next bit to flag bank 2
	}

	{
		// FLASHSTUB needs to be non-const for this to work
		static int nFlashStubOffset = -1;

		if (-1 == nFlashStubOffset)
		{
			for (idx = 0; idx<SIZE_OF_FLASHSTUB; idx++)
			{
				if ((FLASHSTUB[idx] == 0x0a) && (FLASHSTUB[idx+1] == 0xbc) && (FLASHSTUB[idx+2] == 0xde) && (FLASHSTUB[idx+3] == 0xf0))
				{
					nFlashStubOffset = idx;
					break;
				}
			}
		}

		if (-1 == nFlashStubOffset)
		{
			bye("Error: Failed to find signature - internal error.");
		}
		else
		{
			FLASHSTUB[nFlashStubOffset] = (nBlocks & 0xff000000) >> 24;
			FLASHSTUB[nFlashStubOffset + 1] = (nBlocks & 0xff0000) >> 16;
			FLASHSTUB[nFlashStubOffset + 2] = (nBlocks & 0xff00) >> 8;
			FLASHSTUB[nFlashStubOffset + 3] = (nBlocks & 0xff);
		}
	}

	DoFile((uchar*)FLASHSTUB, 0x4100, SIZE_OF_FLASHSTUB, 168, true);

	// Don't scan for the buffers to be ready till they are zeroed, indicates start of flash
	// first buffer
	do
	{
		Spin();

#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x1800 + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x1800+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		}
		else
		{
			Sleep(100);
		}
	}
	while (0 != poll);
	
	printf(".");
	
	// second buffer
	do
	{
		Spin();

#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800 + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		} 
		else 
		{
			Sleep(100);
		}
	}
	while (0 != poll);
	
	printf(".\n");

	// blocks are 64k each, and each takes about 300ms (somewhat less) to erase
	printf("Waiting for erase to complete (about %ds)", ((nBlocks+1)*300)/1000);

	// Both buffers will be marked ready when the
	// Jag is ready to proceed.
	WaitForBothBuffers();

	g_OptSilentConsole = false;

	// reset pointer
	nextez = 0x1800;
}


/* Request the Jaguar to dump the flash (uses the console to collect it) */
void DoDump(char *pszName)
{
	int nCnt, nRes;
	DWORD nEnd, nStart = GetTickCount();

	// First we need to open the file and write the universal header to it
	fp = fopen(pszName, "wb");

	if (NULL == fp)
	{
		bye("Error: Can't open output file!");
	}
	else
	{
		nCnt = (int)fwrite(univbin, 1, sizeof(univbin), fp);

		// binary doesn't include the useless 0xff padding to 8k, so do that here
		// except, there is a tiny block at 0x400 we need to fill in with standard
		// values.
		while (nCnt < 0x400)
		{
			fputc(0xff, fp);
			nCnt++;
		}

		// standard values - 32-bit cart, start at 0x802000, show logo
		nCnt += (int)fwrite(standard_values, 1, sizeof(standard_values), fp);

		// now the rest of the padding
		while (nCnt < 8192)
		{
			fputc(0xff, fp);
			nCnt++;
		}

		// This will make DoFile shell out to the console before returning
		g_OptConsole = true;
		g_OptSilentConsole = true;

		// we hack ROMDUMP to set the second bank, if needed. This requires ROMDUMP
		// not to be const!
		if (nCartBank == 1)
		{
			ROMDUMP[0xab] = 1;
		}

		printf("Beginning dump to '%s'...\n", pszName);
		DoFile((uchar*)ROMDUMP, 0x10000, SIZE_OF_ROMDUMP, 168, true);

		nEnd = GetTickCount();
		nRes = (nEnd - nStart) / 1000;
		if (nRes > 0)
		{
			printf("\nDumped 4MB in %ds - %dKB/s\n", nRes, (4 * 1024 - 8) / nRes);
		}
		else
		{
			printf("Dump time <1s\n");
		}
	}
}


/* Request the Jaguar to print out serial number (uses the console to collect it) */
void DoSerialInfo(void)
{
	unsigned char SerBuf[12];
	unsigned short poll;
    DWORD curtime,endtime;

	// Open socket to Jaguar
	if (NULL == udev)
	{
		udev = findEZ(true, true);
	}

	// On the newer boards, we can get this information without uploading a program, so try that first
	// First, make sure that the $2800 buffer is marked as ready
	endtime = GetTickCount() + 2000;

	do
	{
		Spin();

#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800 + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		}
//			printf("Polled 0x%08x\n", poll);
		// poll is unsigned. But a value over 0xf0xx is reserved for future use
		// and we can't count on the lower bytes to be correct since the high
		// byte can change first in rare race conditions.
        curtime=GetTickCount();
        if (curtime > endtime)
		{
            bye("Error: can't connect with skunkboard.");
        }
	}
	while (0xf0ff != (poll&0xf0ff));

	if (poll == 0xffff)
	{
		// now, get the lower 12 bytes of that buffer - should contain the serial number
		// If it fails, fall back on the old approach
#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12)
#endif
		{
			// check for the magic word at the beginning (note the funky byte swapping!)
			if (!memcmp(SerBuf, "\x57\xfa\x0d\xf0", 4))
			{
				// that is what we are looking for! The next 8 bytes are the revision and serial number
				// Note that they are in BCD!
				printf("Boot version %02x.%02x.%02x, Serial %02x%02x\n", SerBuf[6], SerBuf[5], SerBuf[4],  SerBuf[9], SerBuf[8]);
				return;
			}
		}
	}

	// Else, fall back on the old system. You can tell which one happened by
	// looking for the 'Sending...' output of the upload. :)
	// This will make DoFile shell out to the console before returning
	g_OptConsole = true;
	g_OptSilentConsole = true;
	DoFile((uchar*)DUMPVER, 0x5000, SIZE_OF_DUMPVER, 168, true);
	printf("\n");
}


/* Request the Jaguar to print out serial number in big text (uses the console to collect it) */
/* Sort of an Easter Egg, but meant to help in test. */
void DoSerialBig(void)
{
	unsigned char SerBuf[12];
	unsigned short poll;
	char szOut[128];
	int i;

	// Open socket to Jaguar
	if (NULL == udev)
	{
		udev = findEZ(true, true);
	}

	// On the newer boards, we can get this information without uploading a program, so try that first
	// First, make sure that the $2800 buffer is marked as ready
	do
	{
		Spin();

#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800 + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		}
//			printf("Polled 0x%08x\n", poll);
		// poll is unsigned. But a value over 0xf0xx is reserved for future use
		// and we can't count on the lower bytes to be correct since the high
		// byte can change first in rare race conditions.
	}
	while (0xf0ff != (poll&0xf0ff));

	if (poll == 0xffff)
	{
		// now, get the lower 12 bytes of that buffer - should contain the serial number
		// If it fails, fall back on the old approach
#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12)
#endif
		{
			// check for the magic word at the beginning (note the funky byte swapping!)
			if (0 == memcmp(SerBuf, "\x57\xfa\x0d\xf0", 4)) 
			{
				// that is what we are looking for! The next 8 bytes are the revision and serial number
				// Note that they are in BCD!
				// build each row - converting 2 digit BCD for the most part (we drop the first 0)
				for (i=0; i<7; i++)
				{
					szOut[0]='\0';
					CatVal(szOut, sizeof(szOut), SerBuf[6]%16, i);
					CatVal(szOut, sizeof(szOut), -2, i);
					CatVal(szOut, sizeof(szOut), SerBuf[5]/16, i);
					CatVal(szOut, sizeof(szOut), SerBuf[5]%16, i);
					CatVal(szOut, sizeof(szOut), -2, i);
					CatVal(szOut, sizeof(szOut), SerBuf[4]/16, i);
					CatVal(szOut, sizeof(szOut), SerBuf[4]%16, i);
					CatVal(szOut, sizeof(szOut), -1, i);
					CatVal(szOut, sizeof(szOut), SerBuf[9]/16, i);
					CatVal(szOut, sizeof(szOut), SerBuf[9]%16, i);
					CatVal(szOut, sizeof(szOut), SerBuf[8]/16, i);
					CatVal(szOut, sizeof(szOut), SerBuf[8]%16, i);
					printf("%s\n", szOut);
				}

				return;
			}
		}
	}

	// This one won't fall back on the old system
	bye("Error: FAILED TO GET SERIAL NUMBER BY NEW SYSTEM");
}


/* Do a BIOS update (if available) */
void DoBiosUpdate(void)
{
	unsigned char SerBuf[12];
	int currentRev = 0;
	bool bForce = false;
	
#if defined(INCLUDE_BIOS_10204) || defined(INCLUDE_BIOS_30002)
	if (g_OptDoFlash)
	{
		// overriding this flash to mean force update, to repair a damaged BIOS
		bForce = true;
		g_OptDoFlash = false;	// don't confuse the poor upload code
	}
	
	if (g_OptOverrideFlash)
	{
		// force the old rev 1 updater -- necessary with really old rev 1
		// boards (for instance, 1.01.00 has trouble). Should be okay with
		// rev 2 boards, so no override provided for that. This still won't
		// load rev 1 BIOS on a rev 2 or 3, so it's okay.
		printf("Forcing upgrade to assume Rev 1 board.\n");
		currentRev=1;
	}
	else
	{
		// first, find out what board we currently have so we know what to load
		// to do this across all the various revisions, we need to upload some code
		printf("Examining current board...\n");
		g_OptConsole=false;
		g_skipwait=true;
		DoFile((uchar*)readver, 0x5000, SIZE_OF_READVER, 168, true);
		g_skipwait=false;
		Sleep(500);

		// after the program runs, we should be able to just read the current information
		// Open socket to Jaguar
	//	Reattach();

		// readver copies the version data in the same way as the latest BIOSs do, so just read it
		// we don't check for the buffer again, because that doesn't work with the rev 1 board
		// we delayed long enough above that all should be well.
		// now, get the lower 12 bytes of that buffer - should contain the serial number
#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12)
#endif
		{
			// check for the magic word at the beginning (note the funky byte swapping!)
			if (0 == memcmp(SerBuf, "\x57\xfa\x0d\xf0", 4))
			{
				// that is what we are looking for! The next 8 bytes are the revision and serial number
				// Note that they are in BCD!
				// Major - SerBuf[6]
				// Minor - SerBuf[5]
				// Rev   - SerBuf[4]
				currentRev = SerBuf[6];
			}
			else
			{
				bye("\n\nNot able to read current board revision\n");
			}
		}
	}
	
	if (currentRev == 1)
	{
		#if !defined(INCLUDE_BIOS_10204)
		bye("\n\nBIOS Upgrade for Rev 1 boards not included in this version of JCP2\n");
		#endif
	}
	else
	{
		if ((currentRev == 2)||(currentRev == 3))
		{
			#if !defined(INCLUDE_BIOS_30002)
			bye("\n\nBIOS Upgrade for Rev 2 or 3 boards not included in this version of JCP2\n");
			#endif
		}
		else
		{
			if ((currentRev < 1) || (currentRev > 3))
			{
				printf("\n\nCurrent rev is '%02x'\n", currentRev);
				bye("Upgrade is not supported by this version of JCP2\n");
			}
		}
	}
	
	DoResetAndReconnect(true);
	
	g_OptConsole = true;
	g_OptSilentConsole = true;

	// off we go now!
	if (currentRev == 1)
	{
#ifdef INCLUDE_BIOS_10204
		// Major - SerBuf[6]
		// Minor - SerBuf[5]
		// Rev   - SerBuf[4]
		if (!bForce)
		{
			if ((SerBuf[6] == 1) && (SerBuf[5] == 2) && (SerBuf[4] == 4))
			{
				bye("Board is already on revision 1.02.04 - upgrade not required (-fu to force).\n");
			}
		}

		printf("\n\nGoing to upgrade Rev 1 board to 1.02.04\n\n");	
		DoFile((uchar*)Upgrade10204, 0x80000, SIZE_OF_UPGRADE10204, 168, true);
#endif
	}
	else
	{
		// 2 and 3
#ifdef INCLUDE_BIOS_30002
		// Major - SerBuf[6]
		// Minor - SerBuf[5]
		// Rev   - SerBuf[4]
		if (!bForce)
		{
			if ((SerBuf[6] == 3) && (SerBuf[5] == 0) && (SerBuf[4] == 2))
			{
				bye("Board is already on revision 3.00.02 - upgrade not required (-fu to force).\n");
			}
		}

		printf("\n\nGoing to upgrade Rev %d board to Rev 3 BIOS 3.00.02 (okay for Rev2)\n\n", currentRev);
		DoFile((uchar*)upgrayyd30002, 0x80000, SIZE_OF_UPGRAYYD30002, 168, true);
#endif
	}
#else
	bye("Error: BIOS update not included in this build of JCP2");
#endif
}


/* Writes a block to the Jaguar */
/* uchar points to data to write, curbase is the base to load at, 
   start is the start address or -1 if not starting yet, and len
   is the number of bytes to load. len should be even or at least
   the buffer must be an even size!
   This function writes into the other-than-current block */
void WriteABlock(uchar *data, int curbase, int start, int len)
{
	uchar block[4080];
	int i;
	volatile unsigned short poll;
	DWORD curtime,endtime;

	// check for cartridge header space
	if ( ((curbase >= 0x800000) && (curbase < 0x802000)) ||	((curbase+len >= 0x800000) && (curbase+len < 0x802000)) )
	{
		if ((nCartBank != 1) || (g_SixMegWrite == false)) 
		{
			printf("\n* Skipping block at 0x%08X - unwritable.\n", curbase);
			return;
		}
	}

	// verify flash memory if needed
	if ((g_OptDoFlash) && (g_OptFlashActive))
	{
		if (curbase < 0x800000)
		{
			printf("\n* Skipping block at 0x%08X - not flash!\n", curbase);
			return;
		}
	}

	// skip the RAM check if this is our magic block
	if ((curbase == DUMMYBASE) && ((g_OptOnlyBoot) || (g_OptNoBoot) || (g_OptConsoleUp)))
	{
		// do nothing
	} 
	else
	{
		if (!g_OptDoFlash)
		{
			if (curbase >= 0x200000)
			{
				printf("\n* Skipping block at 0x%08X - not RAM!\n", curbase);
				return;
			}

			if (curbase <= 0x2800) {
				printf("\n* Skipping block at 0x%08X - protected RAM!\n", curbase);
				return;
			}
		}
	}

	memset(block, 0, 4080);

	// 'Fix' the byte order for the next block of file data
	for (i = 0; i < len; i += 2)
	{
		block[i+1] = *data++;
		block[i] = *data++;
	}

	// Set up block trailer
	block[0xFE2] = curbase & 255;
	block[0xFE3] = (curbase >> 8) & 255;
	block[0xFE0] = (curbase >> 16) & 255;
	block[0xFE1] = (curbase >> 24) & 255;

	block[0xFE6] = start & 255;
	block[0xFE7] = (start >> 8) & 255;
	block[0xFE4] = (start >> 16) & 255;
	block[0xFE5] = (start >> 24) & 255;

	block[0xFE8] = 0;
	block[0xFE9] = nextez >> 8;
	nextez = (0x1800 == nextez) ? 0x2800 : 0x1800;

	block[0xFEA] = len & 255;
	block[0xFEB] = (len >> 8) & 255;

	if (g_OptVerbose)
	{
		printf("ez: %04x  start: %08x  len: %04x  base: %08x/%08x\n", nextez, ENMIDEND(block+0xfe4), HALFLITTLEEND(block+0xfea), ENMIDEND(block+0xfe0), curbase);
	}

	// Wait for the block to come free (handshake with 68K).
	poll=0;
	endtime = GetTickCount() + 2000;

	do
	{
		Spin();

#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0xC0, 0xff, 4, nextez + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
		if (usb_control_msg(udev, 0xC0, 0xff, 4, nextez+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		}
//			printf("Polled 0x%08x\n", poll);
		// poll is unsigned. But a value over 0xf0xx is reserved for future use
		// and we can't count on the lower bytes to be correct since the high
		// byte can change first in rare race conditions.

		curtime=GetTickCount();
        if (curtime > endtime)
		{
            bye("Error: can't connect with skunkboard");
        }
	}
	while (0xf0ff != (poll&0xf0ff));

	// any value except 0xffff indicates a future use, possibly that
	// this is a new firmware. We only need this here because this is
	// always the first block sent to the Jaguar.
	if (poll != 0xffff)
	{
		if (g_OptVerbose)
		{
			printf("value %04X", poll);
		}

		bye("Error: Got invalid value from block synchronization. Please use a newer JCP.");
	}

	// Send off the finished block.
#ifdef LIBUSB_1
	if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, nextez, (char*)block, 4080, ComTimeout) != 4080)
#else
	if (usb_control_msg(udev, 0x40, 0xfe, 4080, nextez, (char*)block, 4080, ComTimeout) != 4080)
#endif
	{
		Reattach();
	}

	// check for successful start, except for 'internal' utilities. These may
	// boot so fast and return to command mode so quick we can't catch it.
	if ((!g_skipwait) && (-1 != start) && (-2 != start))
	{
		// since this block should have triggered a boot on Jag, we wait for it
		// Jag should set to 0000 or 8888 (with a possible ffff intermediate)
		// Wait for the block to change to a valid setting (handshake with 68K).
		poll=0;

		do
		{
			Spin();

#ifdef LIBUSB_1
			if (libusb_control_transfer(udev, 0xC0, 0xff, 4, nextez + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
			if (usb_control_msg(udev, 0xC0, 0xff, 4, nextez+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
			{
				Reattach();
			}

//			printf("Polled 0x%08x\n", poll);
		}
		while ((0x0000 != poll) && (0x8888 != poll));

		if (poll == 0x8888)
		{
			bye("Error: Unauthorized. You must flash a different rom to proceed.\n(Remember to reset the jag with 'jcp2 -r'!)");
		}
		else
		{
			if ( (g_OptVerbose) || (!g_OptSilentConsole) )
			{
				printf("Jag accepted start request at $%08X.\n", start&0x00ffffff);
			}
		}
	}
}


/* Send a file to the Jaguar */
void SendFile(int flen, uchar *fptr, int curbase, int base)
{
	int dotty=0;
	int len;
	int start;
	DWORD dummy;

	while (flen > 0)
	{
		start = (flen <= 4064) ? base : -1;

		if (-1 != start)
		{
			if (g_OptNoBoot)
			{
				if (g_OptFlashActive)
				{
					// flash technically can't no-boot, so this is the first half
					// of a 6MB copy. Tell the flash program to return to cmd mode
					start = -2;
				}
				else
				{
					// it really is no-boot
					start = -1;
				}
			}
			else
			{
				dotty = 0;

				if (g_OptVerbose)
				{
					printf(" \nBooting address $%X\n", start);
				}
				else
				{
					printf(" \n");
				}

				/* add modifiers only when the flash is proceeding */
				if ((g_OptFlashActive)||(g_OptOnlyBoot))
				{
					if (nCartBank == 1)
					{
						start|=0x10000000;		// set bank 2
					}
					if (nCartBank == -1)
					{
						start|=0x70000000;		// set 6MB mode (signed, avoid high bit)
					}
				}
			}
		}

		len = (flen <= 4064) ? flen : 4064;

		WriteABlock(fptr, curbase, start, len);

		fptr += 4064;
		curbase += 4064;
		flen -= 4064;

		dotty = (dotty + 1) & 7;

		if ((0 == dotty) && (!g_OptQuietMode)) 
		{
			putchar('.');
		}
	}

	// if this is a no-boot case, we need to make sure the next block will
	// be at $2800, as that's the only address jcp polls to start up. So
	// if needed, we'll send a little dummy block here, like with -b
	if ((g_OptNoBoot) && (nextez != 0x1800))
	{
		dummy = 0;
		WriteABlock((unsigned char*)&dummy, DUMMYBASE, -1, 4);
	}
}


/* Parse a file for headers, and prepare to send it to the Jaguar */
/* returns the number of bytes processed including headers */
/* note: builtin files DO NOT autodetect - make sure your base and */
/* skip values are correct! (You can run the file manually to get them) */
int DoFile(uchar *fdata, int base, int flen, int skip, bool builtin)
{
	// Send the data to the Jaguar, 4064 bytes at a time
	int curbase;
	int ticks, oldlen;
	int nRet = 0;
	uchar *fptr;
	int res;

	if (!g_OptOnlyBoot)
	{
		if (!builtin)
		{
			// this one mutes so we don't print the info twice
			DetermineFileInfo(true, fdata, &base, &flen, &skip);
		
			if (g_HeaderSkip > 0)
			{
				printf("Forcing a manual header skip of %d bytes (was %d)\n", g_HeaderSkip, skip);
				skip = g_HeaderSkip;
			}
		}

		curbase = base;
		flen -= skip;

		if ((base >= 0x800000) || (base+flen >= 0x800000))
		{
			if ((!g_OptDoFlash) && (!g_OptOverrideFlash))
			{
				if (!g_OptAutoMode)
				{
					bye("Error: This upload requires the -f option to prepare the flash!");
				}

				/* if the exe was renamed, then we WILL do flash */
				g_OptDoFlash = true;
				DoFlash(flen+skip);
				g_OptFlashActive = true;
			}
		}
	}
	else
	{
		// the only-boot mode, we send a dummy block to the top of unused ROM space and then boot the address
		curbase = DUMMYBASE;
		flen = 4;		// smallest transfer size
		skip = 0;
	}

	fptr = fdata + skip;

	// Open socket to Jaguar


	if (NULL == udev)
	{
		udev = findEZ(true, true);
	}

	/* if this is the first file, and we are in auto mode, check if reset is needed */

	if ((g_OptAutoMode) && (!g_FirstFileSent))
	{
		if (TestIfBuffersLocked())
		{
			// buffers locked - probably from a previous upload, so reset
			DoResetAndReconnect(true);
		}
	}

	g_FirstFileSent=true;
	ticks = GetTickCount();
	oldlen = flen;

	if (!g_OptOnlyConsole)
	{
		printf("Sending...");
		if ((g_SixMegWrite) && (!g_OptOnlyBoot)) 
		{
			if ((curbase+flen) >= 0xc00000)
			{
				// trim it down and only send what's needed
				flen -= (curbase+flen)-0xc00000;
			}
		}

		nRet = flen+skip;
		SendFile(flen, fptr, curbase, base);
	}

	if (!g_OptOnlyBoot)
	{
		ticks = GetTickCount() - ticks;

		if (ticks > 0)
		{
			res=oldlen/ticks;
		} 
		else
		{
			res=0;
		}

		if ( (g_OptVerbose) || (!g_OptSilentConsole) ) 
		{
			printf(" \nFinished in %d millis, %dKB/second.\n", ticks, res);
		}
	}

	if (g_OptConsole)
	{
		HandleConsole();
	}

	return nRet;
}


// Abort nicely(?)
void bye(char* msg)
{
	if (msg[0] != '\0')
	{
		printf("* %s\n", msg);
	}

	if (NULL != udev)
	{
#ifdef LIBUSB_1
		libusb_close(udev);
#else
		usb_close(udev);
#endif
	}

	if (NULL != fdata)
	{
		free(fdata);
	}

	exit(1);
}


/* Locate the Jaguar on the USB bus, open it, get a handle, and upload the turboW tool */
#ifdef LIBUSB_1
libusb_device_handle *findEZ(bool fInstallTurbo, bool fAbortOnFail)
#else
usb_dev_handle *findEZ(bool fInstallTurbo, bool fAbortOnFail)
#endif
{	
#ifdef LIBUSB_1
	libusb_device_handle *udev = NULL;
	struct libusb_device_descriptor desc;
	libusb_device **devlist;
	libusb_device *device;
	uint8_t PortList[256];
	ssize_t cnt;
	unsigned char SerBuf[12];

	if ((cnt = libusb_get_device_list(ctx, &devlist)) >= 0)
	{
		for (ssize_t i = 0; i < cnt; i++)
		{
			device = devlist[i];

			if (!USBBus || (libusb_get_bus_number(device) == USBBus))
			{
				int j = libusb_get_port_numbers(device, PortList, sizeof(PortList));

				while (j--)
				{
					if (!USBPort || (PortList[j] == USBPort))
					{
						if (!libusb_get_device_descriptor(device, &desc))
						{
							if ((desc.idVendor == 0x4b4) && (desc.idProduct == 0x7200))
							{
								if (!libusb_open(device, &udev))
								{
									if (!SkunkboardSerial || ((libusb_control_transfer(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12) && (SerBuf[8] == (SkunkboardSerial & 0x00ff)) && (SerBuf[9] == ((SkunkboardSerial & 0xff00) >> 8))))
									{
										if (fInstallTurbo)
										{
											// load turbow from array
											if (libusb_control_transfer(udev, 0x40, 0xff, 0, 0x304c, (char*)turbow, SIZE_OF_TURBOW, ComTimeout) != SIZE_OF_TURBOW)
											{
												printf("Failed to install turbow.bin!\n");
											}
											else
											{
												if (g_OptVerbose)
												{
													printf("Installed turbow.bin\n");
												}
											}
										}

										return udev;
									}
									else
									{
										libusb_close(udev);
									}
								}
								else
								{
									bye("Error: Skunkboard found, but can't open EZ-HOST. In use or not ready? ");
								}
							}
						}
					}
				}
			}
		}

		libusb_free_device_list(devlist, 1);
	}
#else
	usb_dev_handle *udev = NULL;
	struct usb_bus *bus;
	struct usb_device *dev;
	int nTriesLeft = 3;
	int ret;
	unsigned char SerBuf[12];

	while (nTriesLeft--) 
	{
		usb_init();
		usb_set_debug(0);
		usb_find_busses();
		usb_find_devices();

		for (bus = usb_get_busses(); bus; bus = bus->next)
		{
			if (!strlen(USBBusName) || !strcmp(USBBusName, bus->dirname))
			{
				for (dev = bus->devices; dev; dev = dev->next)
				{
					if ((dev->descriptor.idVendor == 0x4b4) && (dev->descriptor.idProduct == 0x7200))
					{
						if (!(udev = usb_open(dev)))
						{
							bye("Error: - Found, but can't open, EZ-HOST. In use or not ready? ");
//							udev = NULL;
//							break;		/* should quickly fall out of outer loop too, unless there's another! */
						}
						else
						{
							if (!SkunkboardSerial || ((usb_control_msg(udev, 0xC0, 0xff, 4, 0x2800, (char*)SerBuf, 12, ComTimeout) == 12) && (SerBuf[8] == (SkunkboardSerial & 0x00ff)) && (SerBuf[9] == ((SkunkboardSerial & 0xff00)>>8))))
							{
								if (fInstallTurbo)
								{
									// load turbow from array
									ret = usb_control_msg(udev, 0x40, 0xff, 0, 0x304c, (char*)turbow, SIZE_OF_TURBOW, ComTimeout);

									if (ret < 1)
									{
										printf("Failed to install turbow.bin!\n");
									}
									else
									{
										if (g_OptVerbose)
										{
											printf("Installed turbow.bin: %d scan codes sent\n", ret);
										}
									}
								}

								return udev;
							}
							else
							{
								usb_close(udev);
								udev = NULL;
							}
						}
					}
				}
			}
		}

		if (udev == NULL)
		{
			if (nTriesLeft > 0)
			{
				printf(".. retrying ..\n");
				Sleep(1000);
			}
		}
	}
#endif
	if (fAbortOnFail)
	{
		bye("Error: Can't open EZ-HOST.\n");
		// does not return
	}

	return NULL;
}


/* called from a failed attempt to access the jag */
void Reattach(void)
{
	printf("Waiting to handshake with 68k (control-c to abort)\n");
	udev=NULL;
	Sleep(1000);
	udev = findEZ(true, true);
}


/* Does all the console functions */
void HandleConsole(void)
{
#ifndef REMOVERS
	int nTotalFileLength=0;		// bytes written
	char buf[4064];
	int nLength;
	int nDummy;
#endif
	uchar block[4080];
	unsigned short tmp;
	int i, len;
#ifdef WIN32
	char *p, *oldp;
#endif
	int x;

	// If the user requested an external console, then we just have to shell out to it here
	if (strlen(g_pszExtShell))
	{
		printf(" \nStarting external console...\n");
		if (-1 == _execlp(g_pszExtShell, g_pszExtShell, NULL))
		{
			printf("Could not exec external shell, error %d\n", errno);
			bye("Error: External shell failed to launch.");
		}
		else
		{
			// if it returns
			exit(0);
		}
	}

	// our basic job is to relay text from the Jaguar. Extension commands are marked with 0xffff as the first two bytes
	// In text mode, we accept text from either buffer, so we scan them both
	if ( (g_OptVerbose) || (!g_OptSilentConsole) )
	{
		printf(" \nStarting console...\n");
	}

	if (g_OptDoDump)
	{
		printf(" \nReceiving flash...\n");
	}

	// flag console as up
	g_OptConsoleUp = true;

	// blank both buffers
	memset(block, 0, 4080);

	// Set up block trailer
	block[0xFE8] = 0;
	block[0xFE9] = nextez >> 8;
	nextez = (0x1800 == nextez) ? 0x2800 : 0x1800;

	block[0xFEA] = 0xff;
	block[0xFEB] = 0xff;

	// to handshake with the jaguar, we clear the blocks from this end
	// that way the Jag knows we're up and ready.
	tmp = 0xffff;

	for (;;)
	{
#ifdef LIBUSB_1
		if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, 0x1800 + 0xFEA, (char*)&tmp, 2, ComTimeout) != 2)
#else
		if (usb_control_msg(udev, 0x40, 0xfe, 4080, 0x1800+0xFEA, (char*)&tmp, 2, ComTimeout) != 2)
#endif
		{
			Reattach();
		}
		else
		{
#ifdef LIBUSB_1
			if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, 0x2800 + 0xFEA, (char*)&tmp, 2, ComTimeout) != 2)
#else
			if (usb_control_msg(udev, 0x40, 0xfe, 4080, 0x2800 + 0xFEA, (char*)&tmp, 2, ComTimeout) != 2)
#endif
			{
				Reattach();
			}
			else
			{
				break;
			}
		}
	}

	// now we can start the main loop
	for (;;)
	{
		// Wait for the block to be used (handshake with 68K).
		volatile short poll=0;

		do 
		{
			nextez = (0x1800 == nextez) ? 0x2800 : 0x1800;

			// It's actually faster to check this small block and
			// do two reads than to read the whole block just to test
#ifdef LIBUSB_1
			if (libusb_control_transfer(udev, 0xC0, 0xff, 4, nextez + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
			if (usb_control_msg(udev, 0xC0, 0xff, 4, nextez+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
			{
				Reattach();
			}
		}
		while (-1 == poll);

		// Read in the finished block.
		for (;;)
		{
#ifdef LIBUSB_1
			if (libusb_control_transfer(udev, 0xC0, 0xff, 4080, nextez, (char*)block, 4080, ComTimeout) == 4080)
#else
			if (usb_control_msg(udev, 0xC0, 0xff, 4080, nextez, (char*)block, 4080, ComTimeout) == 4080)
#endif
			{
				break;
			}

			Reattach();
		}

		// acknowledge the buffer as read to delag the jag
		tmp = 0xffff;

		for (;;)
		{
#ifdef LIBUSB_1
			if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, nextez + 0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#else
			if (usb_control_msg(udev, 0x40, 0xfe, 4080, nextez+0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#endif
			{
				break;
			}

			Reattach();
		}

		// deswap the block (including the header)
		for (i=0; i<4080; i+=2)
		{
			x = block[i+1];
			block[i+1] = block[i];
			block[i] = x;
		}

		if (g_OptVerbose) 
		{
			printf("Read block from %x, len %d, first bytes: %02x %02x %02x %02x\n", nextez, ((block[0xfea]<<8)|block[0xfeb]), block[0], block[1], block[2], block[3]);
		}
		
		if (0 == ((block[0xfea]<<8)|block[0xfeb]))
		{
			// bad block (left over flag from booting), ignore
			continue;
		}

		// Now do something with it
		if ((block[0]==0xff) && (block[1]==0xff))
		{
			// escape command (16 bit command)
			switch ((block[2]<<8)|block[3])
			{
#ifndef REMOVERS
					// nop - can be handy for synchronizing?
				case 0:		
					if (g_OptVerbose)
					{
						printf("NOP received.\n");
					}
					break;

					// terminate console
				case 1:		
					if ( (g_OptVerbose) || (!g_OptSilentConsole) )
					{
						printf("Console terminating.\n");
					}
					return;

					// receive input
				case 2:		
					// get input from the user
					printf("> ");
						if (!fgets(buf, 4064, stdin)) {
							printf("Failed to read from stdin, code %d\n", errno);
							buf[0]='\0';
						} else {
					buf[4063] = '\0';
						}

					// strip EOL
					i = (int)strlen(buf) - 1;
					while (i > 0)
					{
						if (buf[i] < ' ')
						{
							buf[i] = '\0';
							i--;
						} 
						else
						{
							break;
						}
					}

					// write that input to the jag in the alternate buffer
					WriteABlock((unsigned char*)buf, DUMMYBASE, -1, (int)strlen(buf)+1);

					if (g_OptVerbose)
					{
						printf("Wait for Jag to clear %04X\n", nextez);
					}

					// now we must not proceed from this point until the Jaguar
					// acknowledges that block by clearing its length
					do 
					{
#ifdef LIBUSB_1
						if (libusb_control_transfer(udev, 0xC0, 0xff, 4, nextez + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
						if (usb_control_msg(udev, 0xC0, 0xff, 4, nextez+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
						{
							Reattach();
						}

						Sleep(500);
					}
					while (0 != poll);

					// Now clear the buffer back to 0xffff so the Jag can use it again
					tmp  =0xffff;

					for (;;)
					{
#ifdef LIBUSB_1
						if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, nextez + 0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#else
						if (usb_control_msg(udev, 0x40, 0xfe, 4080, nextez+0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#endif
						{
							break;
						}

						Reattach();
					}

					break;

					// Open a file for writing
				case 3:		
					for (i = 0; i < 4060; i++)
					{
						buf[i] = block[i+4];
						if (buf[i] == '\0')
						{
							break;
						}
					}

					buf[i] = '\0';					// makesure
					FilenameSanitize(buf);

					if (NULL != fp)
					{
						printf("Closing file...\n");
						fclose(fp);
						fp = NULL;
					}

					fp = fopen(buf, "wb");
					if (NULL != fp)
					{
						printf("Opened %s for writing...\n", buf);
						nTotalFileLength=0;
					}
					else
					{
						printf("Error: Failed to open %s for writing, code %d\n", buf, errno);
					}

					break;

				// open a file for reading
				case 4:		
					for (i=0; i<4060; i++) 
					{
						buf[i]=block[i+4];
						if (buf[i] == '\0')
						{
							break;
						}
					}

					buf[i]='\0';	// makesure
					FilenameSanitize(buf);

					if (NULL != fp) 
					{
						printf("Closing file...\n");
						fclose(fp);
						fp=NULL;
					}

					fp=fopen(buf, "rb");
					if (NULL != fp)
					{
						printf("Opened %s for reading...\n", buf);
					}
					else
					{
						printf("Error: Failed to open %s for reading, code %d\n", buf, errno);
					}
					
					break;

					// write a block to the open file
				case 5:		
					if (NULL != fp) 
					{
						Spin();

						nLength=((block[0xfea]<<8)|block[0xfeb])-4;
						if (nLength > 4060)
						{
							nLength = 4060;
						}

						fwrite(&block[4], 1, nLength, fp);
						nTotalFileLength += nLength;

						if (g_OptVerbose)
						{
							printf("Wrote %d bytes, total %d\n", nLength, nTotalFileLength);
						}
					}
					break;

					// read a block from a file
				case 6:		
					if (NULL != fp)
					{
						Spin();

						nLength=(block[0xfea]<<8)|block[0xfeb];
						if (g_OptVerbose)
						{
							printf("Read requested %d -", nLength);
						}

						if (nLength > 4064)
						{
							nLength = 4064;
						}

						nLength=(int)fread(buf, 1, nLength, fp);
						if (g_OptVerbose)
						{
							printf(" got %d\n", nLength);
						}

						// write that input to the jag in the alternate buffer
						WriteABlock((unsigned char*)buf, DUMMYBASE, -1, nLength);

						if (g_OptVerbose)
						{
							printf("Wait for Jag to clear %04X\n", nextez);
						}
					}
					else
					{
						// need to send an empty reply back to the Jag
						nDummy = 0;
						// write that input to the jag in the alternate buffer
						WriteABlock((unsigned char*)&nDummy, DUMMYBASE, -1, 0);
					}

					// now we must not proceed from this point until the Jaguar
					// acknowledges that block by clearing its length
					do
					{
#ifdef LIBUSB_1
						if (libusb_control_transfer(udev, 0xC0, 0xff, 4, nextez + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
						if (usb_control_msg(udev, 0xC0, 0xff, 4, nextez+0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#endif
						{
							Reattach();
						}
					}
					while (0 != poll);

					// Now clear the buffer back to 0xffff so the Jag can use it again
					tmp = 0xffff;
					for (;;)
					{
#ifdef LIBUSB_1
						if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, nextez + 0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#else
						if (usb_control_msg(udev, 0x40, 0xfe, 4080, nextez+0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#endif
						{
							break;
						}

						Reattach();
					}
					break;

					// close a file
				case 7:		
					if (NULL != fp)
					{
						printf("Closing file...\n");
						fclose(fp);
						fp=NULL;
					}
					break;
#else
					case 1:
						serve_request((char *)block + 4, NULL);
						break;

					case 2:
					{
						char buf[4064];

						serve_request((char *)block + 4, buf);
						int nLength = MSGHDRSZ + get_message_length(buf); // add header size to content length

						// write that input to the jag in the alternate buffer
						WriteABlock((unsigned char*)buf, DUMMYBASE, -1, nLength);

						// now we must not proceed from this point until the Jaguar
						// acknowledges that block by clearing its length
						do
						{
#ifdef LIBUSB_1
							if (libusb_control_transfer(udev, 0xC0, 0xff, 4, nextez + 0xFEA, (char*)&poll, 2, ComTimeout) != 2)
#else
							if ((usb_control_msg(udev, 0xC0, 0xff, 4, nextez + 0xFEA, (char*)&poll, 2, ComTimeout) != 2))
#endif
							{
								Reattach();
							}
						}
						while (0 != poll);

						// Now clear the buffer back to 0xffff so the Jag can use it again
						tmp = 0xffff;
						for (;;)
						{
#ifdef LIBUSB_1
							if (libusb_control_transfer(udev, 0x40, 0xfe, 4080, nextez + 0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#else
							if (usb_control_msg(udev, 0x40, 0xfe, 4080, nextez + 0xFEA, (char*)&tmp, 2, ComTimeout) == 2)
#endif
							{
								break;
							}
							Reattach();
						}

						break;
					}
#endif
				default:
					printf("Warning: Unimplemented command 0x%04X\n", (block[2]<<8)|block[3]);
					break;
			}
			continue;	
		}

		// else get the length and reformat as a string
		len = block[0xfea]|(block[0xfeb]<<8);
		if (len > 4064)
		{
			len = 4064;
		}
		block[len] = '\0';

		// Check for and trap formfeed characters
		// use them to clear the screen, like an old-school terminal
		// Not sure if Linux/Apple will support this?
#if defined(WIN32) || defined(WIN64)
		p = block;
		oldp = p;

		while (NULL != (p = strchr(p, '\xc')))
		{
			*p = '\0';
			printf("%s", oldp);
			system("cls");
			oldp = p+1;
		}

		printf("%s", oldp);
#else
		printf("%s", (const char*)block);
#endif
	}
}


// remove any path information from buf
void FilenameSanitize(char *buf)
{
	char *ptr;

	ptr=strrchr(buf, '/');

	if (NULL != ptr)
	{
		memmove(buf, ptr, strlen(ptr)+1);
	}

	ptr=strrchr(buf, '\\');

	if (NULL != ptr)
	{
		memmove(buf, ptr, strlen(ptr)+1);
	}
}


// parse a string address (used by command line) 
// Does not return on failure!
int ParseAddress(const char *pBuf) {
	int base;

	if ((pBuf[0] != '$') && ((pBuf[0]!='0')||(pBuf[1]!='x'))) {
		printf("Could not parse address '%s'\n", pBuf);
		bye("Address must start with '$' or 0x. (Linux may require '\\$')");
	}
	if (pBuf[0] == '$') {
		if (1 != sscanf(&pBuf[1], "%x", &base)) {
			printf("Could not scan address '%s'\n", pBuf);
			bye("Failed to parse address.");
		}
	} else {
		if (1 != sscanf(&pBuf[2], "%x", &base)) {
			printf("Could not scan address '%s'\n", pBuf);
			bye("Failed to parse address..");
		}
	}

	return base;
}


// helper for the SerialBig function
void CatVal(char *szOut, int nBufLen, int nVal, int nRow)
{
	if ((nRow < 0) || (nRow > 6))
	{
		// out of range, return
		return;
	}

	if ((nVal < -2) || (nVal > 9)) 
	{
		// out of range, clamp value
		nVal=0;
	}

	if ((signed)(strlen(szOut) + strlen(szNumDat[0])) > nBufLen-1)
	{
		// too long!
		return;
	}

	if (nVal == -1) 
	{
		nVal = 10;
	}
	else
	{
		if (nVal == -2)
		{
			nVal = 11;
		}
	}

	// digit row
	strcat(szOut, szNumDat[nDigits[nVal][nRow]]);
}


// Looks at the data to determine the true base, length, and skip
// Pass in what you already know, values may be updated. Return
// is true if file type was recognized, false otherwise. Fields
// may still be changed even if false is returned!
// Will only set values to absolute settings (ie: it must be safe
// to call this function multiple times with updated values)
bool DetermineFileInfo(bool bMute, uchar *fdata, int *base, int *flen, int *skip)
{
	bool ret = true;	// assume it will be true
	char *pTmp;
	int fPadded;
	int idx, nFirst;

	// Check file header
	if ((*flen > 0x2000) && (0x802000 == ENBIGEND(fdata+0x404)))
	{
		if (!bMute)
		{
			if ( (g_OptVerbose) || (!g_OptSilentConsole) )
			{
				printf("Cart ROM:  ");
			}
		}

		if (!g_OptOverride)
		{
			*base = 0x802000;
		}

		*skip = 0x2000;
	} else if ((*flen > 0x2200) && (0x802000 == ENBIGEND(fdata+0x604))) {
		if (!bMute) {
			if ( (g_OptVerbose) || (!g_OptSilentConsole) ) {
				printf("Cart ROM + 512:  ");
			}
		}
		if (!g_OptOverride) *base = 0x802000;
		*skip = 0x2200;
	} else if ((*flen > 72) && (fdata[0] == 0x01) && (fdata[1] == 0x50)) {
		if (!bMute) {
			if ( (g_OptVerbose) || (!g_OptSilentConsole) ) {
				printf("COFF File:  ");
			}
		}
		if (!g_OptOverride) *base = ENBIGEND(fdata+56);
		*skip = ENBIGEND(fdata+68);
		if (*flen <= *skip) {
			bye("Error: Detection error or corrupt file.\n");
		}
	} else if ((*flen > 0x30) && (fdata[0] == 0x7f) && (fdata[1] == 'E') && (fdata[2] == 'L') && (fdata[3] == 'F')) {
		int loadbase;
		int secs, seclen;
		uchar *img, *secptr;

		if (!bMute)
		{
			if ( (g_OptVerbose) || (!g_OptSilentConsole) )
			{
				printf("ELF File:  ");
			}
		}

		if ((fdata[5] != 0x2) || (0x20004 != ENBIGEND(fdata+0x10))) 
		{
			bye("Error: Not 68K executable.");
		}

		*skip = loadbase = ENBIGEND(fdata+0x18);
		*flen = 0;
	
		// Map all the sections into a new memory image. Not necessarily entirely safe.
		secs = HALFBIGEND(fdata+0x30);
		seclen = HALFBIGEND(fdata+0x2e);
		img = (uchar*)malloc(RAMBUFSIZE);
		secptr = fdata+ENBIGEND(fdata+0x20);
		memset(img, 0, 2048*1024);
		while (secs-- >= 0) {
			int sadr = ENBIGEND(secptr+0xc), slen = ENBIGEND(secptr+0x14);
			uchar* fptr = fdata+ENBIGEND(secptr+0x10);
			if (0 != sadr) {		// 0 is debug info, so ignore it
				if (sadr < loadbase)
					bye("Error: Section has base address below entry point.  See readelf for details.");
				if (sadr+slen > *flen)
					*flen = sadr+slen;
				if (*flen >= 2048*1024 || loadbase < 0)
					bye("Error: Section falls outside Jaguar memory.  See readelf for details.");
				if (1 == ENBIGEND(secptr+0x4))	// Progbits, so copy them
					memcpy(img+sadr, fptr, slen);
			}
			secptr+=seclen;
		}
		// copy the data across to the true buffer
		memcpy(fdata, img, RAMBUFSIZE);
		free(img);
		if (!g_OptOverride) *base=loadbase;
	} else if ((*flen > 0x2e) && (fdata[0x1c] == 'J') && (fdata[0x1d] == 'A') && (fdata[0x1e] == 'G') && (fdata[0x1f] == 'R')) {
		if (!bMute) {
			if ( (g_OptVerbose) || (!g_OptSilentConsole) ) {
				printf("Jag Server Exe: ");
			}
		}
		if (!g_OptOverride) *base=ENBIGEND(fdata+0x22);
		*skip=0x2e;
	} else if ((*flen > 0x24) && (fdata[0] == 0x60) && (fdata[1] == 0x1b)) {
		if (!bMute) {
			if ( (g_OptVerbose) || (!g_OptSilentConsole) ) {
				printf("DRI ABS File:  ");
			}
		}
		*skip = 0x24;
		*base = ENBIGEND(fdata+0x16);
		*flen = ENBIGEND(fdata+0x6) + ENBIGEND(fdata+0x2) + *skip;
	}
	else
	{
		if ((*flen > 0xa8) && (fdata[0] == 0x01) && (fdata[1] == 0x50))
		{
			if (!bMute)
			{
				if ((g_OptVerbose) || (!g_OptSilentConsole))
				{
					printf("Alcyon ABS File:  ");
				}
			}

			*skip = 0xa8;
			*base = ENBIGEND(fdata + 0x28);		// Right now, the code below assumes base = run address.
			// run = ENBIGEND(fdata + 0x24);	// But these files can have different run and base addresses.
			*flen = ENBIGEND(fdata + 0x18) + ENBIGEND(fdata + 0x1c) + *skip;
		}
		else
		{
			// check for headerless ROM padded with a solid value (normally would be FF)
			// skip the first 8 bytes as some vendors put data there
			fPadded = false;

			if (*flen > 0x2000)
			{
				nFirst = fdata[8];

				for (idx = 9; idx < 8192; idx++)
				{
					if (fdata[idx] != nFirst)
					{
						break;
					}
				}

				// first byte at $2000 in is actual data (could still misread with weird padding bytes)
				if ((idx >= 8192) && (fdata[8192] != nFirst))
				{
					fPadded = true;
				}
			}

			if (fPadded)
			{
				if (!bMute)
				{
					if ((g_OptVerbose) || (!g_OptSilentConsole))
					{
						printf("Headerless padded ROM: ");
					}
				}

				if (!g_OptOverride)
				{
					*base = 0x802000;
				}

				*skip = 0x2000;
			}
			else
			{
				// if all else failed, and the extension is .ROM, assume $802000 load address 
				pTmp = strrchr(g_szFilename, '.');

				if (NULL != pTmp)
				{
					if (_stricmp(pTmp, ".rom") == 0)
					{
						if ((g_OptVerbose) || (!g_OptSilentConsole))
						{
							if (!bMute)
							{
								// check for exact multiple of 2MB or 4MB, and warn user if so
								if ((*flen == (1024 * 1024 * 2)) || (*flen == (1024 * 1024 * 4)))
								{
									printf("Warning: ROM size is suspicious but no common header found. Full file being uploaded.\nIf it fails, consider adding '-h 8192' to skip a 2k header\n\n");
								}

								printf("Headerless ROM: ");
							}
						}

						if (!g_OptOverride)
						{
							*base = 0x802000;
						}

						*skip = 0;
					}
				}

				ret = false;		// either way, if we got here we are guessing
			}
		}
	}

	if (!bMute) 
	{
		if ( (g_OptVerbose) || (!g_OptSilentConsole) )
		{
			printf("Skip %d bytes, base addr is $%X, length is %d bytes\n", *skip, *base, *flen-*skip);
		}
	}

	return ret;
}
