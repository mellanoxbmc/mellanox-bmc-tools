/************************************************************************
*       Copyright, 2000-2003 Lattice Semiconductor Corp.                *
*                               SVF2VME.c                               *
*       This Program Converts An SVF File From ASCII Source Form Into   *
*       Binary Form To Reduce Size And To Improve Processing Time.      *
*                                                                       *
*       The conversion is done by converting Token into 8 bits Code.    *
*       The 8 bits Token are always followed by 16 bits field.          * 
*       Some 8 bits Token has data field follows the 16 bits field.     *
*       SDI MASK is not supported due to its rare appearence on ISP     *
*       application.                                                    *
*       The data field follows the SDR token are compressed when 0x00   *
*       or 0xFF bytes are detected. The compression is done by replacing*
*       0xFF, 0xFF, 0xFF by 0xFF, 0x02 etc.                             *
*       Single level Huffman encoding scheme is used to compress the    *
*       data and compare with the result produced above. The winner is  *
*       is used to compress the data.                                   *
*       Compression is done on row by row basis.                        *
*       The RUNTEST Token is followed by a 16 bits field express in     *
*       uS if the most significant bit is 0, in mS if the most          *
*       significant bit is 1.                                           *
*       The FREQUENCY Token is followed by a 16 bits field express in   *
*       KHZ if the most significant bit is 0, in MHZ if the most        *
*       significant bit is 1.                                           *
*                                                                       *
* Note: This program must be compiled as a 32 bits application if the   *
*       SVF file is known to contain more than more than 64000 bits     *
*       in a single scan.                                               *
*                                                                       *
*                                                                       *
*************************************************************************
* Rev. 13 Mellanox based on 12.2  2017/03/12 by Oleksandr Shamray       *
* <oleksandrs@mellanox.com>                                             *
*		1. Added possibility to direct CPLD programming trough          *
*		   JTAG driver interface.                                       *
*		2. Added handlers to support Aspeed AST25xx controller JTAG     *
*		   interface.                                                   *
*		3. Added new command line key to define JTAG interface          *
*		   device path:                                                 *
*		   "-prog /dev/aspeed-jtag"                                     *
*                                                                       *
*************************************************************************
* Rev. 12.2		2008/04/04 by Chuo Liu									*
*		1. Frequency statement update									*
*			- If -c flag is not applied, keep frequency in the SVF file	*
*			  if available, or default to 1 MHz							*
*			- If -c flag is applied, apply new frequency to the VME		*
*			  file only if the -c frequency is lower than SVF file		*
*			  Frequency.												*
*		2. Max memory required update									*
*			- Search both SDR and SIR size for max memory required.		*
*		3. TCK to time update											*
*			- Add new option under -max_tck flag: -max_tck no		 	*
*			  If -max_tck no is present, do not convert TCK to time.	*
*			- If TCK amount is greater than the value after -max_tck,	*
*			  or 1000 by default, convert TCK to wait time to			*
*			  microseconds, and leave the remainder clock as TCK.		*
*			  If frequency is overwrited by -c flag, use the new		*
*			  frequency to determine /	convert	TCK to wait time.		*
*		4. Maximum Buffer size											*
*			- Add new option under -max_size flag [optional] 		 	*
*			  Specifies the maximum value allowed to allocate memory    *
*			  for a row of data in KBytes. Ex. 8,16,32,64				*
*             Default: 64 KBytes.										*
*************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <fcntl.h>
#include <sys/stat.h>
#ifndef DISABLE_JTAG_PROG
#include <uapi/linux/ioctl.h>
#include <uapi/linux/aspeed-jtag.h>
#endif
#include "vmopcode.h"
#include "utilities.h"
#include "svf2vme.h"
#include "jtag_handlers.h"

FILE * g_pSVFFile, * g_pVMEFile;
int g_JTAGFile;
char  buffer[strmax];   /*memory to store a string temporary          */
char  * g_pszSVFString;   /*pointer to current token string*/
long int g_iFrequency = 0; /* Stores the active frequency (in Hz) */
int g_iSVFLineIndex = 0;            /*keeps the svfline number read*/
int headIR = 0, tailIR = 0, headDR = 0, tailDR = 0;
int CurEndDR = DRPAUSE, CurEndIR = IRPAUSE;
long int g_iMaxSize = 0;            /*the maximum row size of all the SVF files*/
unsigned char g_ucComment = 0;
/*02/16/07 Nguyen added to support header*/
unsigned char g_ucHeader = 0;
char g_cHeader[strmax];/*memory to store a header string */

/* Variables used in programming foreign devices */
int g_iVendor = 0;
long int g_iMaxBufferSize = SCANMAX;      /*the maximum value allowed to allocate memory*/

/*********************************************************************
*
* Register used to control how SVF is converted to VME.
*
*********************************************************************/

unsigned short g_usFlowControlRegister = 0;

/*********************************************************************
*
* Buffer used to hold intelligent programming data and index used to
* hold the size.
*
*********************************************************************/

unsigned char * g_ucIntelBuffer = 0;
unsigned short g_usIntelBufferSize = 2;
unsigned short g_usIntelBufferIndex = 0;

/*********************************************************************
*
* Function prototypes.
*
*********************************************************************/

void EncodeCRC( const char * a_pszVMEFilename );
void LCOUNTCom();
void writeIntelProgramData();

int (*write_handler)(unsigned  char reset, char data) = null_handler;
int g_errStatus = 0;
char g_direct_prog = 0;

static struct stableState 
{
	char * text;
	int state;
} stableStates[ StableStateMax ] = {
	{ "RESET", RESET },
	{ "IDLE", IDLE },
	{ "IRPAUSE", IRPAUSE },
	{ "DRPAUSE", DRPAUSE },
	{ "DRCAPTURE", DRCAPTURE} /*11/15/05 Nguyen changed to support DRCAPTURE*/
};

static struct scanToken 
{
	char * text;
	int token;
} scanTokens[] = 
{
	/*********************************************************************
	*
	* Supported SVF keywords.
	*
	*********************************************************************/

	{ ";", ENDDATA },
	{ "SIR", SIR }, { "SDR", SDR },
	{ "TDI", TDI }, { "TDO", TDO }, { "MASK", MASK },
	{ "STATE", STATE },
	{ "TCK", TCK, }, { "WAIT", WAIT },
	{ "XSDR", XSDR }, { "XTDI", XTDI }, { "XTDO", XTDO },
	{ "ENDDR", ENDDR },
	{ "ENDIR", ENDIR },
	{ "HIR", HIR }, { "TIR", TIR }, { "HDR", HDR }, { "TDR", TDR },
	{ "MEM", MEM },
	{ "RUNTEST", RUNTEST },
	{ "ENDSTATE", ENDSTATE },
	{ "TRST", TRST },
	{ "FREQUENCY", FREQUENCY },
	{ "SEC", SEC },
	{ "SMASK", SMASK },
	{ "MAXIMUM", MAX },
	{"ON", ON},{"OFF", OFF},{"ISPEN",ispEN}, {"HIGH", HIGH},{"LOW", LOW},
	{ "SETFLOW", SETFLOW }, { "RESETFLOW", RESETFLOW },
	{ "REPEAT", REPEAT }, { "ENDLOOP", ENDLOOP },
	{ "(", LEFTPAREN }, 
	{ "CRC", CRC },
	{ "CMASK", CMASK },
	{ "RMASK", RMASK },
	{ "READ", READ },
	{ "DMASK", DMASK },
	{ "VUES",  VUES },
	{ "LCOUNT", LCOUNT },
	{ "LDELAY", LDELAY },
	{ "LSDR", LSDR },
	{ "LVDS", LVDS },
	{ "LOOP", LOOP }
};

int ScanTokenMax = sizeof( scanTokens ) / sizeof( scanTokens[ 0 ] );

/* 3 scan nodes is reserved:
   0 is for SIR, 1 is for SDR and 2 is to store previous SDR */ 
/* 4/26/2001 ht Add 1 scan nodes to store the HIR,TIR,HDR and TDR info */
struct scanNode 
{
 long int numbits;
 unsigned char *tdi;
 unsigned char *tdo;
 unsigned char *mask;
 unsigned char *crc;
 unsigned char *cmask;
 unsigned char *rmask;
 unsigned char *read;
 unsigned char *dmask;

} scanNodes[4] = {{0, NULL, NULL, NULL, NULL, NULL, NULL, NULL,NULL},
{0, NULL, NULL, NULL, NULL, NULL, NULL, NULL,NULL},
{0, NULL, NULL, NULL, NULL, NULL, NULL, NULL,NULL},
{0, NULL, NULL, NULL, NULL, NULL, NULL, NULL,NULL}
};

CFG * cfgChain = NULL;



void print_progress(unsigned int pos, unsigned int total)
{
	static char last_progress = -1;
	char progress = 0;

	if (pos > total)
		return;
	progress = (char)((pos * 100) / total);
	if (progress != last_progress){
		last_progress = progress;
		printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\bprogress %3d%%", progress);
		fflush(stdout);
	}
}

/*********************************************************************
*                                                                    *
* IsStableState                                                      *
*                                                                    *
* Returns the index of the stable state.  If the stable state is not *
* found or invalid, then return -1.                                  *
*                                                                    *
*********************************************************************/

short int IsStableState( const char * a_pszState )
{
	short int siIndex = 0;

	for ( siIndex = 0; siIndex < StableStateMax; siIndex++ ) {
		if ( !stricmp( a_pszState, stableStates[ siIndex ].text ) ) {

			/*********************************************************************
			*                                                                    *
			* Matches stable state.                                              *
			*                                                                    *
			*********************************************************************/

			return siIndex;
		}
	}

	/*********************************************************************
	*                                                                    *
	* Failed to match a stable state.                                    *
	*                                                                    *
	*********************************************************************/

	return -1;
}

/*************************************************************************
*                                                                        *
*              Convert Number to VME format                              *
*                                                                        *
* A large number is converted into cascading of bytes                    *
* If the most significant bit of a byte is 0, then                       *
* it is the last byte to form the number.                                *
* Input:                                                                 *
* number-----the decimal number to be converted                          *
* Return:                                                                *
* the number of bytes the number is converted into.                      *       
*                                                                        *
*************************************************************************/
void ConvNumber( long int number )
{
	while ( number > 0x007F ) {
		g_errStatus |= write( ( unsigned char ) ( ( number & 0x007F ) + 0x0080 ) );
        number = number >> 7;
	}
	
	g_errStatus |= write( ( unsigned char ) number );
}

/*********************************************************************
*                                                                    *
* Token                                                              *
*                                                                    *
* Stores a token in the variable g_pszSVFString.  Returns 1 when EOF or  *
* 0 when passing.                                                    *
*                                                                    *
*********************************************************************/

int Token( const char * delimiters )
{
	int iStringIndex, iTempIndex, iStringLength, iCommentIndex;

	if ( ( g_pszSVFString == NULL ) || ( ( g_pszSVFString = strtok( NULL, delimiters ) ) == NULL ) || ( g_pszSVFString[ 0 ] == '\n' ) || 
		 ( g_pszSVFString[ 0 ] == '!' || ( g_pszSVFString[ 0 ] == '/' ) ) ) {
		do {

			/*********************************************************************
			*                                                                    *
			* Get a string from the SVF file and increment the line index.       *
			*                                                                    *
			*********************************************************************/

			g_iSVFLineIndex++;
			if ( fgets( buffer, strmax, g_pSVFFile ) == NULL ) {

				/*********************************************************************
				*                                                                    *
				* File has reached EOF, return 1.                                    *
				*                                                                    *
				*********************************************************************/

				return ( 1 );
			}

			iStringLength = strlen( buffer ) + 1;
			if ( iStringLength > 2 ) {
				for ( iStringIndex = 0; iStringIndex < iStringLength; iStringIndex++ ) {
					switch ( buffer[ iStringIndex ] ) {
					case '\t':
						
						/*********************************************************************
						*                                                                    *
						* Replace tab with space.                                            *
						*                                                                    *
						*********************************************************************/

						buffer[ iStringIndex ] = ' ';
						break;
					case '\r':
						
						/*********************************************************************
						*                                                                    *
						* Replace return carriage with space.                                *
						*                                                                    *
						*********************************************************************/

						buffer[ iStringIndex ] = ' ';
						break;
					case ';':
					case '(':
					case ')':

						/*********************************************************************
						*                                                                    *
						* Insert spaces before the characters ";()" to allow tokenizing.     *
						*                                                                    *
						*********************************************************************/

						for ( iTempIndex = iStringLength + 2; iTempIndex > iStringIndex + 2; iTempIndex-- ) {
							buffer[ iTempIndex ] = buffer[ iTempIndex - 2 ];
						}

						buffer[ iTempIndex-- ] = ' ';
						buffer[ iTempIndex-- ] = buffer[ iStringIndex ];
						buffer[ iStringIndex ] = ' ';
						iStringLength += 2;
						iStringIndex += 2;
						break;
					case '/':
						if ( buffer[ iStringIndex + 1 ] != '/' ) {
							return FILE_ERROR;
						}
					case '!':
						if ( !g_ucComment ) {

							/*********************************************************************
							*                                                                    *
							* Comment option not passed in via command line.  Skip the comment   *
							* by setting iStringIndex = iStringLength.                           *
							*                                                                    *
							*********************************************************************/

							iStringIndex = iStringLength;
							break;
						}

						/*********************************************************************
						*                                                                    *
						* Write the comment into the VME file.                               *
						*                                                                    *
						*********************************************************************/
						
						write( COMMENT );
						ConvNumber( iStringLength - iStringIndex - 2 );
						for ( iCommentIndex = iStringIndex; buffer[ iCommentIndex ] != '\n'; iCommentIndex++ ) {
							write( buffer[ iCommentIndex ] );

							/*********************************************************************
							*                                                                    *
							* Replace the character with space.  This is done to prevent the     *
							* comment from being written more than once.  For example, if the    *
							* character is not replaced with a space, then the following legal   *
							* SVF comments would be written more than once:                      *
							*                                                                    *
							* !! SVF Comment.                                                    *
							* !!! SVF Comment.                                                   *
							* /// SVF Comment.                                                   *
							* //// SVF Comment.                                                  *
							*                                                                    *
							*********************************************************************/

							buffer[ iCommentIndex ] = ' ';
						}
						break;
					default:
						break;
					}
				}
			}
			
			/*********************************************************************
			*                                                                    *
			* Get the first token of g_pszSVFString.                                 *
			*                                                                    *
			*********************************************************************/
			
			g_pszSVFString = strtok( buffer, delimiters );

		} while ( ( g_pszSVFString == NULL ) || ( g_pszSVFString[ 0 ] == '\r' ) || ( g_pszSVFString[ 0 ] == '\n' ) ||
			      ( g_pszSVFString[ 0 ] == '!' ) || ( g_pszSVFString[ 0 ] == '/' ) );
    }
	
	return ( 0 );
}


/*************************************************************************
* ispsvf_convert()                                                       *
* Read the svf file line by line and convert line by line into a token   *
* based ispSTREAM file:                                                  *
* FIELD:  token  :size    :   data                                       *
* WIDTH:  8 bits :16 bits :   variable(in binary)                        *
* 8/28/01 define options:                                                *
*          bit 0: no compress is 0, comression is 1.                     *
*          bit 1: stop if fail is 0, stop if pass is 1.                  *
*************************************************************************/
short int ispsvf_convert( int chips, CFG * chain, char * vmefilename, bool compress )
{
	short int  i, j, rcode = 0, rcode_prog = 0;
	int device;
	char filler = 0;
	long int scan_len;
	char opcode;
	int temp;
	char xchar;
	int iStringIndex,iStringLength;
	unsigned long int SVFfile_size = 0;
	unsigned long int SVFfile_pos = 0;
	struct stat filestat;
	static struct header 
	{
		unsigned char types;
		int           value;
	} headers[ 4 ] = { { TDR, 0 }, { TIR, 0 }, { HDR, 0 }, { HIR, 0 } };
	
	if (g_direct_prog == 0){
		if ( ( g_pVMEFile = fopen( vmefilename, "wb" ) ) == NULL ) {
			return FILE_NOT_FOUND;
		}
		/*********************************************************************
		*
		* Write the VME version.
		*
		*********************************************************************/

		fprintf( g_pVMEFile, "____13" );
		if ( compress ) {

			/*********************************************************************
			*
			* Full VME.
			*
			*********************************************************************/

			write( 0xF1 );
		}
		else  {

			/*********************************************************************
			*
			* Compressed VME.
			*
			*********************************************************************/

			write( 0xF2 );
		}
	}
	/*02/16/07 Nguyen added to support header*/
	if(g_ucHeader && strcmp(g_cHeader,""))
	{
		/*********************************************************************
		*
		* Write the VME header.
		*
		*********************************************************************/
		iStringLength = strlen( g_cHeader ) + 1;
		if ( iStringLength > 2 ) {
			for ( iStringIndex = 0; iStringIndex < iStringLength; iStringIndex++ ) {
				switch ( g_cHeader[ iStringIndex ] ) {
				case '\t':				
					/*********************************************************************
					*                                                                    *
					* Replace tab with space.                                            *
					*                                                                    *
					*********************************************************************/

					g_cHeader[ iStringIndex ] = ' ';
					break;
				case '\r':
					/*********************************************************************
					*                                                                    *
					* Replace return carriage with space.                                *
					*                                                                    *
					*********************************************************************/

					g_cHeader[ iStringIndex ] = ' ';
					break;
				default:
					break;
				}
			}
		}
		write( COMMENT );
		ConvNumber( iStringLength);
		for ( iStringIndex = 0; iStringIndex < iStringLength; iStringIndex++ ) {
			write( g_cHeader[ iStringIndex ] );
		}
	}
	/*********************************************************************
	*
	* Write the maximum memory size needed.
	*
	*********************************************************************/

	write( MEM );
	ConvNumber( g_iMaxSize );
	
	/*********************************************************************
	*
	* For-loop based on the number of SVF files.
	*
	*********************************************************************/

	for (device = 0; device < chips; device++) {
		printf("Process SVF config file(%s) %d of %d\n", chain[device].Svffile, device+1, chips);
		rcode = 0;
		
		/*********************************************************************
		*
		* Write the VENDOR opcode.
		*
		*********************************************************************/

		write( VENDOR );
		if ( !stricmp( chain[ device ].Vendor, "lattice" ) ) {

			/*********************************************************************
			*
			* Vendor is LATTICE.
			*
			*********************************************************************/

			g_iVendor = LATTICE;
			write( LATTICE );
		}
		else if ( !stricmp( chain[ device ].Vendor, "altera" ) ) {

			/*********************************************************************
			*
			* Vendor is ALTERA.
			*
			*********************************************************************/

			g_iVendor = ALTERA;
			write( ALTERA );
		}
		else if ( !stricmp( chain[ device ].Vendor, "xilinx" ) ) {

			/*********************************************************************
			*
			* Vendor is XILINX.
			*
			*********************************************************************/

			g_iVendor = XILINX;
			write( XILINX );
		}
		
		if ( stricmp( chain[ device ].name, "SVF" ) == 0 ) {    
			if ( ( g_pSVFFile = fopen( chain[ device ].Svffile, "r" ) ) == NULL ) {
				if (g_direct_prog == 0)
					fclose( g_pVMEFile );
				return FILE_NOT_FOUND;
			}
			
			stat(chain[ device ].Svffile, &filestat);
			SVFfile_size = filestat.st_size;
			SVFfile_pos = 0;

			/*********************************************************************
			*
			* Get the frequency.
			*
			*********************************************************************/

			g_iFrequency = chain[ device ].Frequency;
			
			if ( chips > 1 ) {
				for ( i = 0; i < 4; i++ ) {

					/*********************************************************************
					*
					* Default values to 0.
					*
					*********************************************************************/

					headers[ i ].value = 0;
				}

				for ( i = 0; i < device; i++ ) {
					
					/*********************************************************************
					*
					* Calculate the trailer stream.
					*
					*********************************************************************/

					headers[ 0 ].value += 1;
					headers[ 1 ].value += chain[ i ].inst;
				}

				for ( i = device + 1; i < chips; i++ ) {
					
					/*********************************************************************
					*
					* Calculate the header stream.
					*
					*********************************************************************/

					headers[ 2 ].value += 1;
					headers[ 3 ].value += chain[ i ].inst;
				}

				for ( i = 0; i < 4; i++ ) {
					write( headers[ i ].types );
					if ( headers[ i ].value ) {
						write( ( unsigned char ) ( headers[ i ].value ) );
						write( TDI );
						if ( headers[ i ].value % 8 ) {
							temp = headers[ i ].value / 8 + 1;
						}
						else {
							temp = headers[ i ].value / 8;
						}

						if ( ( headers[ i ].types == HIR ) || ( headers[ i ].types == TIR ) ) {
							xchar = ( char ) 0xFF;
						}
						else {
							xchar = ( char ) 0x00;
						}

						for ( j = 0; j < temp; j++ ) {
							write( xchar );
						}
						write( CONTINUE );
					}
					else {
						write( 0x00 );
					}
				}
			}
			
			/*********************************************************************
			*
			* Begin iterating through the SVF.  The active SVF token shall be stored
			* in the variable g_pszSVFString.
			*
			*********************************************************************/
			
			g_pszSVFString = NULL;
			while ( ( rcode == 0 ) /*&& (rcode_prog == 0)*/ && ( ( rcode = Token( " \n" ) ) == 0 ) ) {
				
				/*********************************************************************
				*
				* Check if there exists an opcode for the corresponding SVF token.
				*
				*********************************************************************/
				for ( i = 0; i < ScanTokenMax; i++ ) {
					if ( stricmp( scanTokens[ i ].text, g_pszSVFString ) == 0 ) {  
						break;
					}
				}
				if ( i >= ScanTokenMax ) {

					/*********************************************************************
					*
					* No opcode was found based on the extracted token, return error.
					*
					*********************************************************************/
					if (g_direct_prog == 0)
						fclose( g_pVMEFile );
					return FILE_ERROR;
				}
				SVFfile_pos = ftell(g_pSVFFile);
				print_progress(SVFfile_pos, SVFfile_size);
				opcode = scanTokens[ i ].token; 
				switch (opcode){
                case SDR:
                	write_handler = jtag_cmd_handler;
                	write_handler(WRITE_HANDLER_INIT_CMD, SDR);
					rcode = ScanCom( 1, compress );
					rcode_prog = write_handler(WRITE_HANDLER_SEND_CMD, 0);
					break;
                case SIR:
                	write_handler = jtag_cmd_handler;
                	write_handler(WRITE_HANDLER_INIT_CMD, SIR);
                	rcode = ScanCom( 0, compress );
                	rcode_prog = write_handler(WRITE_HANDLER_SEND_CMD, 0);
					break;
                case STATE: 
					rcode = STATECom(); 
					break;
                case RUNTEST:
                	write_handler = runtest_handler;
                	write_handler(WRITE_HANDLER_INIT_CMD, RUNTEST);
					rcode = RUNTESTCom( chain[ device ].MaxTCK, chain[ device ].noMaxTCK ); 
					rcode_prog = write_handler(WRITE_HANDLER_SEND_CMD, 0);
					break;	
                case ENDIR: 
					rcode = Token( " " );
					write( opcode );
					for ( j = 0; j < StableStateMax; j++ ) {
						if ( !stricmp( g_pszSVFString, stableStates[ j ].text ) ) {
							break;
						}
					}
					if ( j == StableStateMax ) {
						write( IRPAUSE );
					}
					else { 
						write( ( char ) j );
						CurEndIR = j;
					}
					break;
				case ENDDR: 
					rcode = Token( " " );
					write( opcode );
					for ( j = 0; j < StableStateMax; j++ ) {
						if ( stricmp( g_pszSVFString, stableStates[ j ].text ) == 0 ) {
							break;
						}
					}
					if ( j == StableStateMax ) {
						write( DRPAUSE );
					}
					else {
						write( ( char ) j );
						CurEndDR = j;
					}
					break;
				case HDR: 
                case HIR:
                case TIR:
                case TDR:
                	write_handler = jtag_cmd_handler;
                	write_handler(WRITE_HANDLER_INIT_CMD, opcode);

					if ( chips > 1 ) {
						rcode = Token( ";" );
						rcode_prog = write_handler(WRITE_HANDLER_SEND_CMD, 0);
						break;
					}
					
					rcode = Token( " " );
					write( opcode );  /* Write the header/trailer opcode. */
					scan_len = atol( g_pszSVFString );
					if ( scan_len == 0 ) {
						ConvNumber( scan_len );
					}
					else {
						if ( scan_len > ( long int ) g_iMaxBufferSize ) {
						
							/*********************************************************************
							*
							* Number can't be larger than the buffer size limit.
							*
							*********************************************************************/

							if (g_direct_prog == 0)
								fclose( g_pVMEFile );
							return FILE_ERROR;
						}

						/*********************************************************************
						*
						* Turn off compression for header and trailer because compression is
						* off if chips is greater than 1.
						*
						*********************************************************************/

						rcode = TDIToken( scan_len, 3, 0 );
					}
					rcode_prog = write_handler(WRITE_HANDLER_SEND_CMD, 0);
					break;
				case FREQUENCY:
					rcode = FREQUENCYCom();
					break;
                case ENDDATA: 
					break;
				case TDI:
					rcode = Token( ")" );
					break;
				case LCOUNT:
					LCOUNTCom();

					/*********************************************************************
					*
					* Initialize intelligent programming related variables.
					*
					*********************************************************************/

					g_usFlowControlRegister |= INTEL_PRGM;
					g_ucIntelBuffer = ( unsigned char * ) calloc( g_usIntelBufferSize + 1, sizeof( unsigned char ) );
					break;
				case LDELAY:
					rcode = RUNTESTCom( chain[ device ].MaxTCK, chain[ device ].noMaxTCK  );
					break;
				case LSDR:
					rcode = ScanCom( 1, compress );

					/*********************************************************************
					*
					* Write intelligent programming data from the buffer to the file.
					*
					*********************************************************************/

					writeIntelProgramData();
					break;
				case LOOP:
					LCOUNTCom();

					/*********************************************************************
					*
					* Initialize intelligent programming related variables.
					*
					*********************************************************************/

					g_usFlowControlRegister |= INTEL_PRGM;
					g_ucIntelBuffer = ( unsigned char * ) calloc( g_usIntelBufferSize + 1, sizeof( unsigned char ) );
					break;
				case ENDLOOP:
					/*********************************************************************
					*
					* Write intelligent programming data from the buffer to the file.
					*
					*********************************************************************/

					writeIntelProgramData();
					break;
				case VUES:

					/*********************************************************************
					*
					* Write Verify USERCODE opcode.
					*
					*********************************************************************/

					write( opcode );
					break;
				case LVDS:

					/*********************************************************************
					*
					* Support LVDS SVF file.
					*
					*********************************************************************/

					write( opcode );
					rcode = LVDSCom();
					break;
				/* 03/14/06 Support Toggle ispENABLE signal*/
				case ispEN:
					write( opcode );
					Token( " ;" );
					if((!strcmp(g_pszSVFString ,"ON")) || (!strcmp(g_pszSVFString ,"HIGH")))
						write( 0x01 );
					else
						write(0x00);
					break;
				case TRST: 	/* 05/24/06 Support Toggle TRST pin*/
					write( opcode ); 
					Token( " ;" );
					if(!strcmp(g_pszSVFString ,"ON"))
						write( 0x00 );
					else if(!strcmp(g_pszSVFString ,"OFF"))
						write(0x01);
					else
						write( 0x00 );
					break;
				default:

					/*********************************************************************
					*
					* Invalid opcode, return FILE_ERROR.
					*
					*********************************************************************/

					if (g_direct_prog == 0)
						fclose( g_pVMEFile );
					return FILE_ERROR;
				}
			}
			
			if ( scanNodes[ 0 ].mask != NULL ) {
				free( scanNodes[ 0 ].mask ); 
				scanNodes[ 0 ].mask = NULL;
			}
			
			if ( scanNodes[ 1 ].mask != NULL ) {
				free( scanNodes[ 1 ].mask );
				scanNodes[ 1 ].mask = NULL;
			}
			
			for ( i = 0; i < 4; i++ ) {
				if ( scanNodes[ i ].tdi != NULL ) {
					free( scanNodes[ i ].tdi );
					scanNodes[ i ].tdi = NULL;
				}
			}
			fclose( g_pSVFFile );
		}
		else if ( stricmp( chain[ device ].name, "JTAG" ) == 0 ) {
		
		}
		else {
			if (g_direct_prog == 0)
				fclose( g_pVMEFile );
			return FILE_ERROR;
		}
		printf("\n");
	}
	
	/*********************************************************************
	*
	* Write the ENDVME opcode and close the file pointer.
	*
	*********************************************************************/

	write( ENDVME );
	if (g_direct_prog == 0)
		fclose( g_pVMEFile );

	if (!g_direct_prog)
		EncodeCRC( vmefilename );

	return rcode;
}

/************************************************************************
*  
* ExeSTATECommand()                                               
* execute a subset of the SVF STATE command             
*                                                               
* SVF STATE command sends the TAP to the selected *stable* state.    
* The stable state are RESET, IDLE, IRPAUSE, DRPAUSE.                     
* 9/28/00 ht added support to multiple transistion among stable states.
*            example: STATE RESET IDLE; It is translated into
*            STATE RESET;
*            STATE IDLE;
*                                                         
*************************************************************************/

short int STATECom() 
{
	short int i, rcode;                    
	rcode = Token(" ");   
	while ( ( rcode == 0 ) && ( g_pszSVFString[ 0 ] != ';' ) ) {
		for ( i = 0; i < StableStateMax; i++ ) {
			if ( stricmp( g_pszSVFString, stableStates[ i ].text ) == 0) {
				break;
			}
		}
		
		if ( i == StableStateMax ) {}
		else {
			write( STATE );
			write( ( char ) i );
		}
		
		rcode = Token(" ");
	}
	return rcode;
}

/*********************************************************************
*                                                                    *
* RUNTESTCom                                                         *
*                                                                    *
* Convert the SVF RUNTEST command into VME format.                   *
*                                                                    *
*********************************************************************/

short int RUNTESTCom( unsigned int max_tck , short int noMaxTCK ) 
{
	short int siRetCode = 0;
	short int siIndex = 0;
	float fTime = 0;
	unsigned long ulTime = 0;
	int iState = -1;
	
	while ( ( siRetCode = Token( " " ) ) == 0 ) {

		if ( ( siIndex = IsStableState( g_pszSVFString ) ) >= 0 ) {

			/*********************************************************************
			*                                                                    *
			* Write STATE command.                                               *
			*                                                                    *
			*********************************************************************/

			write( STATE );
			iState = stableStates[ siIndex ].state;
			write( ( unsigned char ) iState );
		}
		else if ( !stricmp( g_pszSVFString, "MAXIMUM" ) ) {

			/*********************************************************************
			*                                                                    *
			* Discard MAXIMUM command and burn its following two parameters.     *
			*                                                                    *
			*********************************************************************/

			for ( siIndex = 0; siIndex < 2; siIndex++ ) {
				Token( " " );
			}
		}
		else if ( !stricmp( g_pszSVFString, "ENDSTATE" ) ) {

			/*********************************************************************
			*                                                                    *
			* Found ENDSTATE parameter.                                          *
			*                                                                    *
			*********************************************************************/

			Token( " " );
			if ( ( siIndex = IsStableState( g_pszSVFString ) ) >= 0 ) {

				/*********************************************************************
				*                                                                    *
				* Write STATE command.                                               *
				*                                                                    *
				*********************************************************************/

				write( STATE );
				write( ( unsigned char ) stableStates[ siIndex ].state );
			}
		}
		else if ( ( strchr( g_pszSVFString, 'E' ) != NULL ) || ( strchr( g_pszSVFString, 'e' ) != NULL ) ) {

			/*********************************************************************
			*                                                                    *
			* Number parameter passed in as exponential.                         *
			* Example: 2.00E-2                                                   *
			*                                                                    *
			*********************************************************************/

			fTime = ( float ) atof( g_pszSVFString );

			Token( " " );
			if ( ( fTime > 0 ) && !stricmp( g_pszSVFString, "SEC" ) ) {
				// Rev. 12.2 Chuo add changing wait time to TCK if -max_tck no appears
				if(noMaxTCK){
					fTime = fTime * g_iFrequency;
					while(fTime > 0xFFFF){
						write( TCK );
						ConvNumber(0xFFFF);
						fTime -= 0xFFFF;
					}
					write( TCK );
					ConvNumber( fTime );
				}

				else{
					/*********************************************************************
					*                                                                    *
					* Convert delay from seconds to microseconds.                        *
					*                                                                    *
					*********************************************************************/

					fTime = fTime * 1000000;

					/*********************************************************************
					*                                                                    *
					* If the delay time is greater than 1000 microseconds, then convert  *
					* it back to milliseconds and set the MSB to 1.  By expressing the   *
					* delay in milliseconds, it reduces having to encode large delays    *
					* as microseconds.                                                   *
					*                                                                    *
					*********************************************************************/

					if ( fTime > 1000 ) {

						/*********************************************************************
						*                                                                    *
						* Convert delay from microseconds to milliseconds.                   *
						*                                                                    *
						*********************************************************************/

						fTime = fTime / 1000;

						/*********************************************************************
						*                                                                    *
						* If the delay exceeds the ranger of signed short integers, which is *
						* 0x7FFF, then it must be split over several delays.                 *
						*                                                                    *
						*********************************************************************/

						while ( fTime > ( unsigned long ) 0x7FFF ) {
							
							/*********************************************************************
							*                                                                    *
							* Write the WAIT opcode, and write delay of 0x7FFF + 0x8000, or      *
							* 0xFFFF into the VME file.                                          *
							*                                                                    *
							*********************************************************************/

							write( WAIT );
							ConvNumber( 0x7FFF + 0x8000 );

							/*********************************************************************
							*                                                                    *
							* Decrement the delay by 0x7FFF.                                     *
							*                                                                    *
							*********************************************************************/

							fTime -= 0x7FFF;
						}

						/*********************************************************************
						*                                                                    *
						* Set the MSB of the delay to 1.                                     *
						*                                                                    *
						*********************************************************************/

						fTime += 0x8000;
					}
					
					write( WAIT );
					ConvNumber( ( long ) fTime );
				}
			}
			else {

				/*********************************************************************
				*                                                                    *
				* Invalid parameter.                                                 *
				*                                                                    *
				*********************************************************************/

				siRetCode = FILE_ERROR;
			}
		}
		else if ( atoi( g_pszSVFString ) > 0 ) {

			if ( iState == -1 ) {

				/*********************************************************************
				*                                                                    *
				* If the safe state is not provided, used default state IDLE.        *
				*                                                                    *
				*********************************************************************/

				write( STATE );
				write( IDLE );
			}

			/*********************************************************************
			*                                                                    *
			* Number parameter passed in as integer.                             *
			*                                                                    *
			*********************************************************************/

			ulTime = atoi( g_pszSVFString );

			Token( " " );
			if ( !stricmp( g_pszSVFString, "TCK" ) ) {

				// Rev. 12.2 Chuo add noMaxTCK option to keep TCK the same if flag is provided
				if(noMaxTCK){
					while ( ulTime > ( unsigned long ) 0xFFFF ) {
						/*********************************************************************
						*                                                                    *
						* Write the first chunk of 0xFFFF TCK toggles.                       *
						*                                                                    *
						*********************************************************************/
						write( TCK );
						ConvNumber( ( unsigned long ) 0xFFFF );
						ulTime -= 0xFFFF;
					}
					/*********************************************************************
					*                                                                    *
					* Write the TCK toggles.                                             *
					*                                                                    *
					*********************************************************************/
					write( TCK );
					ConvNumber( ulTime );
				}
				else{
					/*********************************************************************
					*                                                                    *
					* Check if the TCK size is less than or equal to the maximum TCK     *
					* allowed.                                                           *
					*                                                                    *
					*********************************************************************/

					if ( ulTime <= max_tck ) {

						/*********************************************************************
						*                                                                    *
						* Write TCK command by breaking down the TCK count if it exceeds     *
						* 0xFFFF since the VME Embedded cannot decode numbers greater than   *
						* 16-bit.                                                            *
						*                                                                    *
						*********************************************************************/

						max_tck = ulTime;
						while ( ulTime > ( unsigned long ) 0xFFFF ) {

							/*********************************************************************
							*                                                                    *
							* Write the first chunk of 0xFFFF TCK toggles.                       *
							*                                                                    *
							*********************************************************************/

							write( TCK );
							ConvNumber( ( unsigned long ) 0xFFFF );
							ulTime -= 0xFFFF;
						}

						/*********************************************************************
						*                                                                    *
						* Write the TCK toggles.                                             *
						*                                                                    *
						*********************************************************************/

						write( TCK );
						ConvNumber( ulTime );
						ulTime = max_tck;

						// Rev. 12.2 Chuo remove writing wait time if TCK is not converted
						/*if ( !g_iFrequency ) {

							printf( "Warning: failed to convert %d TCK cycles to delay time due to unspecified frequency.\n", ulTime );
						}
						else {

							//*********************************************************************
							//*                                                                    *
							//* Convert given TCK to delay time in microseconds.                   *
							//*                                                                    *
							//*********************************************************************

							fTime = ( float ) ulTime / ( float ) g_iFrequency;
							fTime = fTime * 1000000;

							//*********************************************************************
							//*                                                                    *
							//* If the delay time is greater than 1000 microseconds, then convert  *
							//* it back to milliseconds and set the MSB to 1.  By expressing the   *
							//* delay in milliseconds, it reduces having to encode large delays    *
							//* as microseconds.                                                   *
							//*                                                                    *
							//*********************************************************************

							if ( fTime > 1000 ) {

								//*********************************************************************
								//*                                                                    *
								//* Convert delay from microseconds to milliseconds.                   *
								//*                                                                    *
								//*********************************************************************

								fTime = fTime / 1000;

								//*********************************************************************
								//*                                                                    *
								//* If the delay exceeds the ranger of signed short integers, which is *
								//* 0x7FFF, then it must be split over several delays.                 *
								//*                                                                    *
								//*********************************************************************

								while ( fTime > ( unsigned long ) 0x7FFF ) {
									
									//*********************************************************************
									//*                                                                    *
									//* Write the WAIT opcode, and write delay of 0x7FFF + 0x8000, or      *
									//* 0xFFFF into the VME file.                                          *
									//*                                                                    *
									//*********************************************************************

									write( WAIT );
									ConvNumber( 0x7FFF + 0x8000 );

									//*********************************************************************
									//*                                                                    *
									//* Decrement the delay by 0x7FFF.                                     *
									//*                                                                    *
									//*********************************************************************

									fTime -= 0x7FFF;
								}

								//*********************************************************************
								//*                                                                    *
								//* Set the MSB of the delay to 1.                                     *
								//*                                                                    *
								//*********************************************************************

								fTime += 0x8000;
							}
							
							write( WAIT );
							ConvNumber( ( long ) fTime );
						}*/
					}
					else {

						if ( !g_iFrequency ) {

							printf( "Warning: failed to convert %d TCK cycles to delay time due to unspecified frequency.\n", ulTime );

							/*********************************************************************
							*                                                                    *
							* Since no frequency was given, write TCK command by breaking down   *
							* the TCK count if it exceeds 0xFFFF since the VME Embedded cannot   *
							* decode numbers greater than 16-bit.                                *
							*                                                                    *
							*********************************************************************/

							while ( ulTime > ( unsigned long ) 0xFFFF ) {

								/*********************************************************************
								*                                                                    *
								* Write the first chunk of 0xFFFF TCK toggles.                       *
								*                                                                    *
								*********************************************************************/

								write( TCK );
								ConvNumber( ( unsigned long ) 0xFFFF );
								ulTime -= 0xFFFF;
							}

							/*********************************************************************
							*                                                                    *
							* Write the TCK toggles.                                             *
							*                                                                    *
							*********************************************************************/

							write( TCK );
							ConvNumber( ulTime );
						}
						else {

							/*********************************************************************
							*                                                                    *
							* Write TCK command by breaking down the TCK count if it exceeds     *
							* 0xFFFF since the VME Embedded cannot decode numbers greater than   *
							* 16-bit.                                                            *
							*                                                                    *
							*********************************************************************/

							while ( max_tck > ( unsigned long ) 0xFFFF ) {

								/*********************************************************************
								*                                                                    *
								* Write the first chunk of 0xFFFF TCK toggles.                       *
								*                                                                    *
								*********************************************************************/

								write( TCK );
								ConvNumber( ( unsigned long ) 0xFFFF );
								max_tck -= 0xFFFF;
							}

							/*********************************************************************
							*                                                                    *
							* Convert given TCK to delay time in microseconds.                   *
							*                                                                    *
							*********************************************************************/
							
							fTime = ( float ) ulTime / ( float ) g_iFrequency;
							fTime = fTime * 1000000;
							
							//Rev. 12.2 Chuo updated max_tck option to write remainder TCK instead of max tck
							//if(g_iFrequency >= 1000000){
							//	temp = g_iFrequency / 1000000;
							//	TCK2time = ulTime / temp;
							//	temp = ulTime % temp;
							//}
							//fTime = ( float ) TCK2time;

							/*********************************************************************
							*                                                                    *
							* Write the TCK toggles.                                             *
							*                                                                    *
							*********************************************************************/

							write( TCK );
							ConvNumber(max_tck);
							//ConvNumber( temp );
							//Rev. 12.2 Chuo updated max_tck option to write remainder TCK instead of max tck
							//if(temp){
							//	write( TCK );
							//	ConvNumber( temp );
							//}
							
							
							/*********************************************************************
							*                                                                    *
							* If the delay time is greater than 1000 microseconds, then convert  *
							* it back to milliseconds and set the MSB to 1.  By expressing the   *
							* delay in milliseconds, it reduces having to encode large delays    *
							* as microseconds.                                                   *
							*                                                                    *
							*********************************************************************/

							if ( fTime > 1000 ) {

								/*********************************************************************
								*                                                                    *
								* Convert delay from microseconds to milliseconds.                   *
								*                                                                    *
								*********************************************************************/

								fTime = fTime / 1000;

								/*********************************************************************
								*                                                                    *
								* If the delay exceeds the ranger of signed short integers, which is *
								* 0x7FFF, then it must be split over several delays.                 *
								*                                                                    *
								*********************************************************************/

								while ( fTime > ( unsigned long ) 0x7FFF ) {
									
									/*********************************************************************
									*                                                                    *
									* Write the WAIT opcode, and write delay of 0x7FFF + 0x8000, or      *
									* 0xFFFF into the VME file.                                          *
									*                                                                    *
									*********************************************************************/

									write( WAIT );
									ConvNumber( 0x7FFF + 0x8000 );

									/*********************************************************************
									*                                                                    *
									* Decrement the delay by 0x7FFF.                                     *
									*                                                                    *
									*********************************************************************/

									fTime -= 0x7FFF;
								}

								/*********************************************************************
								*                                                                    *
								* Set the MSB of the delay to 1.                                     *
								*                                                                    *
								*********************************************************************/

								fTime += 0x8000;
							}
							
							write( WAIT );
							ConvNumber( ( long ) fTime );
						}
					}
				}
			}
			else if ( !stricmp( g_pszSVFString, "SEC" ) ) {
				// Rev. 12.2 Chuo add changing wait time to TCK if -max_tck no appears
				if(noMaxTCK){
					ulTime = ulTime * g_iFrequency;
					while(ulTime > 0xFFFF){
						write( TCK );
						ConvNumber(0xFFFF);
						ulTime -= 0xFFFF;
					}

					write( TCK );
					ConvNumber( ulTime );
				}
				else{
					/*********************************************************************
					*                                                                    *
					* Convert delay from seconds to microseconds.                        *
					*                                                                    *
					*********************************************************************/

					ulTime = ulTime * 1000000;

					/*********************************************************************
					*                                                                    *
					* If the delay time is greater than 1000 microseconds, then convert  *
					* it back to milliseconds and set the MSB to 1.  By expressing the   *
					* delay in milliseconds, it reduces having to encode large delays    *
					* as microseconds.                                                   *
					*                                                                    *
					*********************************************************************/

					if ( ulTime > 1000 ) {

						/*********************************************************************
						*                                                                    *
						* Convert delay from microseconds to milliseconds.                   *
						*                                                                    *
						*********************************************************************/

						ulTime = ulTime / 1000;

						/*********************************************************************
						*                                                                    *
						* If the delay exceeds the ranger of signed short integers, which is *
						* 0x7FFF, then it must be split over several delays.                 *
						*                                                                    *
						*********************************************************************/

						while ( ulTime > ( unsigned long ) 0x7FFF ) {
							
							/*********************************************************************
							*                                                                    *
							* Write the WAIT opcode, and write delay of 0x7FFF + 0x8000, or      *
							* 0xFFFF into the VME file.                                          *
							*                                                                    *
							*********************************************************************/

							write( WAIT );
							ConvNumber( 0x7FFF + 0x8000 );

							/*********************************************************************
							*                                                                    *
							* Decrement the delay by 0x7FFF.                                     *
							*                                                                    *
							*********************************************************************/

							ulTime -= 0x7FFF;
						}

						/*********************************************************************
						*                                                                    *
						* Set the MSB of the delay to 1.                                     *
						*                                                                    *
						*********************************************************************/

						ulTime += 0x8000;
					}
					
					write( WAIT );
					ConvNumber( ulTime );
				}
			}
			else {

				/*********************************************************************
				*                                                                    *
				* Invalid parameter.                                                 *
				*                                                                    *
				*********************************************************************/

				siRetCode = FILE_ERROR;
			}
		}
		else if ( !stricmp( g_pszSVFString, ";" ) ) {

			/*********************************************************************
			*                                                                    *
			* Found the end of the RUNTEST command.  Break.                      *
			*                                                                    *
			*********************************************************************/

			break;
		}
		else {

			/*********************************************************************
			*                                                                    *
			* Invalid parameter.                                                 *
			*                                                                    *
			*********************************************************************/

			siRetCode = FILE_ERROR;
		}

		if ( siRetCode < 0 ) {

			/*********************************************************************
			*                                                                    *
			* Break if siRetCode is less than 0.                                 *
			*                                                                    *
			*********************************************************************/

			break;
		}
	}

	return siRetCode;
}  
 
/******************************************************************************
*													*
* ExeFREQUENCYCommand() 									*
*													*
* The frequency is expressed in 16 bits integer in KHZ or MHZ			*
******************************************************************************/

short int FREQUENCYCom() 
{
	int iRetCode;
	float fFrequency;

	/******************************************************************************
	*
	* Get the frequency from the file.
	*
	******************************************************************************/

	iRetCode = Token( ";" );
	fFrequency = ( float ) atof( g_pszSVFString );

	/******************************************************************************
	*
	* If the current frequency is the default of 0 MHz, then overwrite it with the
	* frequency from the file. Otherwise if it's not 0 MHz, then we know the user
	* passed in a custom frequency via the command line.
	*
	******************************************************************************/

	if ( g_iFrequency == 0 ) {
		g_iFrequency = ( long ) fFrequency;
	}
	// Rev. 12.2 Chuo add overwriting custom frequency if the custom one is greater than the one in SVF
	else if(g_iFrequency > ( long ) fFrequency){
		g_iFrequency = ( long ) fFrequency;
	}
	
	write( FREQUENCY );
	ConvNumber( g_iFrequency );
	
	return iRetCode;
}

 
/******************************************************************************************
*															*
* Execute the Shift Data Register										*
*                                                                             		*
* syntax: SDR or SIR 128[SMASK - Optional][TDI Optional][SMASK - Optional]             *
*[TDO - optional]                                            	*
*															*
* Howard Tang 02/02/2000 When encounter SDR > 64000, split the SDR into				*
*                        mulitple SDR with 64000 bits maximum per each shift			*
*                        Example:                                                         *
*                        SDR 84000 TDI();                                               *
*                        is splitted up into:                                             *
*                        SDR 64000 TDI ........ SDR 20000 TDI ........                    *
* Howard Tang 03/07/2000 Give special treatment to a 1 bit long SDR, i.e. SDR 1(0);      *
*                        It shows up primarily for an auto address increment. It is       *
*                        converted blindly into STATE SHIFTDR; STATE PAUSEDR;             *
*                        To achieve minimum file size, the TDO stream is compared with the*
*                        the TDI stream of the previous DRSCAN, if it is the same,        *
*                        opcode XSDR is used to launch simultaneous shift in and out      *
*                                                                                         *
*******************************************************************************************/

short int ScanCom( char scan_type, bool compress )
{
	int rcode = 0;
	long int DR_Length;
	
	/* Read the TDI data */
	rcode = Token( " (" ); 
	
	/* Read the number of bits for SDR */
	DR_Length = atoi( g_pszSVFString );
	if ( DR_Length == 0 ) {
		return FILE_NOT_VALID;
	}
	
	if ( DR_Length > ( long int ) g_iMaxBufferSize) {
		/* Put the device into DRPAUSE before processing cascading frames */
		if ( scan_type ) {
			write( STATE );
			write( DRPAUSE );
		}
	}
	
	/* Read the TDI and TDO data from the SVF file and convert */
	rcode = TDIToken( DR_Length, scan_type, compress );

	/* TDO may exist and compression turns on */
	if ( ( scan_type ) && ( DR_Length > ( long int ) g_iMaxBufferSize ) ) {
		/* Put the device into ENDDR after processing cascading frames */
		write( STATE );
		write( ( char ) CurEndDR );
	}

	return ( rcode );
}
 

/***********************************************************************
*                                                                      *
* Generic routine to read data from the SVF file.                      *
*                                                                      *
* Parses the SVF command token "TDI(11223344)"                         *
*                                                                      *
* SDR 128 TDI(00302043003010430030304300308043);  Program             *
*                                                                      *
* SDR 128 TDI(ffffffffffffffffffffffffffffffff)                       *
*         TDO(00302043003010430030304300308043);  Verify              *
*                                                                      *
* Var                                                                  *
* numbits       The number of bits of data to be processed.            *
* g_pszSVFString    The string contains the whole or part of the data read *
*               from SVF                                               *
* sdr           0 is sir. 1,2 is sdr 3 is hir,hdr,tir,tdr              *
* compress      0 = no compress by default. e.g. SIR data                *
*               1 = compress                                             *
************************************************************************/

short int TDIToken(long int  numbits, char sdr, bool compress )
{
	int            i;
	int            rcode = 0;
	int            bits, bit;
	char           Nodes[4]= {SIR, SDR, XSDR, HIR};
	char           option = 0, Done = 0;
	char           ioshift = 0;         /*simutaneously shift in and out*/

	if ( sdr == 0 ) {

		/****************************************************************************
		*
		* Currently processing SIR, potential for XTDO is lost so delete previous 
		* SDR-TDI data if it exists.
		*
		*****************************************************************************/

		if ( scanNodes[ 1 ].tdi ) {
			free( scanNodes[ 1 ].tdi );
			scanNodes[ 1 ].tdi = NULL;
		}
		
	}

	if ( ( sdr <= 1 ) && ( scanNodes[ sdr ].numbits != numbits ) ) {
		
		/****************************************************************************
		*
		* Data size has changed, old MASK cannot be carried-over.
		*
		*****************************************************************************/

		if ( scanNodes[ sdr ].mask != NULL ) {
			free( scanNodes[ sdr ].mask );
			scanNodes[ sdr ].mask = NULL;
		}

		/****************************************************************************
		*
		* Data size has changed, potential for XTDO is lost so delete previous 
		* SDR-TDI data if it exists.
		*
		*****************************************************************************/

		if ( scanNodes[ 1 ].tdi ) {
			free( scanNodes[ 1 ].tdi );
			scanNodes[ 1 ].tdi = NULL;
		}

		if ( scanNodes[ 2 ].tdi  ) {
			free( scanNodes[ 2 ].tdi );
			scanNodes[ 2 ].tdi = NULL;
		}
	}
	
	if ( ( sdr == 1 ) && ( scanNodes[ sdr ].tdi != NULL ) && ( scanNodes[ sdr ].numbits == numbits ) ) {
		
		/****************************************************************************
		*
		* Copy over previous SDR-TDI to check for possible XTDO later.
		*
		*****************************************************************************/

		if ( scanNodes[ 2 ].tdi != NULL ) {
			free( scanNodes[ 2 ].tdi );
		}

		if ( ( scanNodes[ 2 ].tdi = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
			return OUT_OF_MEMORY;
		}

		for ( i = 0; i < numbits / 8 + 1; i++ ) {
			scanNodes[ 2 ].tdi[ i ] = scanNodes[ 1 ].tdi[ i ];
			scanNodes[ 2 ].numbits = numbits;
		}
	}

	scanNodes[ sdr ].numbits = numbits;
	Done = 0;
	
	/****************************************************************************
	*
	* Read SIR/SDR and convert TDI, TDO, MASK, etc, into binary streams.
	*
	*****************************************************************************/
     
	while ( ( !Done ) && ( rcode == 0 ) ) {
		rcode = Token( " " );
		
		for ( i = 0; i < ScanTokenMax; i++ ) {

			/****************************************************************************
			*
			* Check if g_pszSVFString is a valid token.
			*
			*****************************************************************************/

			if ( stricmp( scanTokens[ i ].text, g_pszSVFString ) == 0 ) {
				break;
			}
		}
		switch ( scanTokens[i].token ) {
		case TDI:

			/****************************************************************************
			*
			* De-allocate existing TDI and convert current TDI to binary stream.
			*
			*****************************************************************************/

			if ( scanNodes[ sdr ].tdi != NULL ) {
				free( scanNodes[ sdr ].tdi );
			}

			if ( ( scanNodes[ sdr ].tdi = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
				return OUT_OF_MEMORY;
			}
			
			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].tdi );
			break;
		case SMASK:

			/****************************************************************************
			*
			* Discard SMASK.
			*
			*****************************************************************************/

			rcode = ConvertFromHexString( numbits, NULL );
			break;
		case TDO:

			/****************************************************************************
			*
			* Convert current TDO to binary stream.
			*
			*****************************************************************************/

			if ( ( scanNodes[ sdr ].tdo = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
				return OUT_OF_MEMORY;
			}

			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].tdo );
			
			if ( ( sdr == 1 ) && ( scanNodes[ 2 ].numbits == scanNodes[ 1 ].numbits ) &&
				 ( scanNodes[ 2 ].tdi != NULL ) ) {
				
				/****************************************************************************
				*
				* Check if current TDO is the same as previous TDI. If this is true, then
				* use XTDO.
				*
				*****************************************************************************/

				for ( i = 0; i < numbits / 8 + 1; i++ ) {
					if ( scanNodes[ 2 ].tdi[ i ] != scanNodes[ sdr ].tdo[ i ] ) {
						break;
					}
				}
				
				if ( i == numbits / 8 + 1 ) {

					/****************************************************************************
					*
					* Bingo. TDO is the same as previous TDI.
					*
					*****************************************************************************/

					if (!g_direct_prog)
						ioshift = 1;
				}
			}
			break;
		case MASK:

			/****************************************************************************
			*
			* De-allocate existing MASK and convert current MASK to binary stream.
			*
			*****************************************************************************/

			if ( scanNodes[ sdr ].mask == NULL ) {
				if ( ( scanNodes[ sdr ].mask = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
					return OUT_OF_MEMORY;
				}
			}
			
			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].mask );
			break;
		case CRC:

			/****************************************************************************
			*
			* De-allocate existing CRC and convert current CRC to binary stream.
			*
			*****************************************************************************/

			if ( scanNodes[ sdr ].crc == NULL ) {
				if ( ( scanNodes[ sdr ].crc = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
					return OUT_OF_MEMORY;
				}
			}

			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].crc );
			break;
		case CMASK:

			/****************************************************************************
			*
			* De-allocate existing CMASK and convert current CMASK to binary stream.
			*
			*****************************************************************************/

			if ( scanNodes[ sdr ].cmask == NULL ) {
				if ( ( scanNodes[ sdr ].cmask = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
					return OUT_OF_MEMORY;
				}
			}

			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].cmask );
			break;
		case READ:

			/****************************************************************************
			*
			* De-allocate existing READ and convert current READ to binary stream.
			*
			*****************************************************************************/

			if ( scanNodes[ sdr ].read == NULL ) {
				if ( ( scanNodes[ sdr ].read = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
					return OUT_OF_MEMORY;
				}
			}
			
			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].read );
			break;
		case RMASK:

			/****************************************************************************
			*
			* De-allocate existing RMASK and convert current RMASK to binary stream.
			*
			*****************************************************************************/

			if ( scanNodes[ sdr ].rmask == NULL ) {
				if ( ( scanNodes[ sdr ].rmask = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
					return OUT_OF_MEMORY;
				}
			}
			
			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].rmask );
			break;
		case DMASK:

			/****************************************************************************
			*
			* De-allocate existing DMASK and convert current DMASK to binary stream.
			*
			*****************************************************************************/

			if ( scanNodes[ sdr ].dmask == NULL ) {
				if ( ( scanNodes[ sdr ].dmask = ( unsigned char * ) malloc( numbits / 8 + 2 ) ) == NULL ) {
					return OUT_OF_MEMORY;
				}
			}
			
			rcode = ConvertFromHexString( numbits, scanNodes[ sdr ].dmask );
			break;
		case ENDDATA:

			/****************************************************************************
			*
			* End of SIR/SDR command.
			*
			*****************************************************************************/

		default:
			Done = 1;
			break;
		}
	}
	
	/****************************************************************************
	*
	* Write the current SIR/SDR stream into VME file format. Split them into
	* multiple SDR streams if necessary.
	*
	*****************************************************************************/

	/*TODO Put JTAG ioctl here */
	bit = 0;
	option = compress + 1; 
	do {
		if ( numbits > ( long int ) g_iMaxBufferSize ) {
			
			/****************************************************************************
			*
			* Enable cascading since data size exceeded allowable limit.
			*
			*****************************************************************************/

			write( SETFLOW );
			ConvNumber( CASCADE );
			bits = ( int ) g_iMaxBufferSize; 
		}
		else {
			bits = ( int ) numbits;
		}
		
		if ( ioshift ) {

			/****************************************************************************
			*
			* Write XSDR to indicate that TDO data is the same as previous TDI.
			*
			*****************************************************************************/

			write( Nodes[ 2 ] );
		}
		else if ( sdr < 3 ) {

			/****************************************************************************
			*
			* Write regular SIR/SDR.
			*
			*****************************************************************************/

			write( Nodes[ sdr ] );
		}
		
		/****************************************************************************
		*
		* Write data size.
		*
		*****************************************************************************/

		ConvNumber( bits );
		
		if ( scanNodes[ sdr ].tdi != NULL ) {

			/****************************************************************************
			*
			* Write TDI.
			*
			*****************************************************************************/

			write( TDI );
			rcode = convertToispSTREAM( bits, &scanNodes[ sdr ].tdi[ bit / 8 ], option );
		}
		
		if ( scanNodes[ sdr ].tdo != NULL ) {

			/****************************************************************************
			*
			* Write TDO/XTDO.
			*
			*****************************************************************************/

			if ( ioshift ) {
				write( XTDO );
			}
			else {
				write( TDO );
				rcode =  convertToispSTREAM( bits, &scanNodes[ sdr ].tdo[ bit / 8 ], option );
			}
		}
		
		if ( ( scanNodes[ sdr ].tdo != NULL ) && ( scanNodes[ sdr ].mask != NULL ) ) {

			/****************************************************************************
			*
			* Write MASK.
			*
			*****************************************************************************/
			
			write( MASK );
			rcode = convertToispSTREAM( bits, &scanNodes[ sdr ].mask[ bit / 8 ], option );
		}
		
		if ( scanNodes[ sdr ].crc != NULL ) {

			/****************************************************************************
			*
			* Write CRC.
			*
			*****************************************************************************/

			write( CRC );
			rcode = convertToispSTREAM( bits, &scanNodes[ sdr ].crc[ bit / 8 ], option );
		}
		
		if ( scanNodes[ sdr ].cmask != NULL ) {

			/****************************************************************************
			*
			* Write CMASK.
			*
			*****************************************************************************/

			write( CMASK );
			rcode = convertToispSTREAM( bits, &scanNodes[ sdr ].cmask[ bit / 8 ], option );
		}
		
		if ( scanNodes[ sdr ].read != NULL ) {

			/****************************************************************************
			*
			* Write READ.
			*
			*****************************************************************************/

			write( READ );
			rcode = convertToispSTREAM( bits, &scanNodes[ sdr ].read[ bit / 8], option );
		}
		
		if ( scanNodes[ sdr ].rmask != NULL ) {

			/****************************************************************************
			*
			* Write RMASK.
			*
			*****************************************************************************/

			write( RMASK );
			rcode = convertToispSTREAM( bits, &scanNodes[ sdr ].rmask[ bit / 8 ], option );
		}
		
		if ( scanNodes[ sdr ].dmask != NULL ) {

			/****************************************************************************
			*
			* Write DMASK.
			*
			*****************************************************************************/

			write( DMASK );
			rcode = convertToispSTREAM( bits, &scanNodes[ sdr ].dmask[ bit / 8 ], option );
		}

        write( CONTINUE );
        bit += ( long int ) g_iMaxBufferSize;
		
		if ( numbits > (long int) g_iMaxBufferSize ) {
			
			/****************************************************************************
			*
			* Disable cascading.
			*
			*****************************************************************************/

			write( RESETFLOW );
			ConvNumber( CASCADE );
		}
	} while ( ( rcode == 0 ) && ( ( numbits -= ( long int ) g_iMaxBufferSize ) > 0 ) );
	
	/****************************************************************************
	*
	* De-allocate TDO.
	*
	*****************************************************************************/

	if ( scanNodes[ sdr ].tdo != NULL ) {
		free( scanNodes[ sdr ].tdo );
		scanNodes[ sdr ].tdo = NULL;
	}
	
	/****************************************************************************
	*
	* De-allocate CMASK.
	*
	*****************************************************************************/

	if ( scanNodes[ sdr ].cmask != NULL ) {
		free( scanNodes[ sdr ].cmask );
		scanNodes[ sdr ].cmask = NULL;
	}
	
	/****************************************************************************
	*
	* De-allocate CRC.
	*
	*****************************************************************************/

	if ( scanNodes[ sdr ].crc != NULL ) {
		free( scanNodes[ sdr ].crc );
		scanNodes[ sdr ].crc = NULL;
	}
	
	/****************************************************************************
	*
	* De-allocate READ.
	*
	*****************************************************************************/

	if ( scanNodes[ sdr ].read != NULL ) {
		free( scanNodes[ sdr ].read );
		scanNodes[ sdr ].read = NULL;
	}
	
	/****************************************************************************
	*
	* De-allocate RMASK.
	*
	*****************************************************************************/

	if ( scanNodes[ sdr ].rmask != NULL ) {
		free( scanNodes[ sdr ].rmask );
		scanNodes[ sdr ].rmask = NULL;
	}
	
	/****************************************************************************
	*
	* De-allocate DMASK.
	*
	*****************************************************************************/

	if ( scanNodes[ sdr ].dmask != NULL ) {
		free( scanNodes[ sdr ].dmask );
		scanNodes[ sdr ].dmask = NULL;
	}
	
	return ( rcode );
}

/******************************************************************************
*													*
* ConvertFromHexString										*
*													*
* Converts the Hex String[00302043003010430030304300308043] into			*
* binary bits.											*
* It start by searching for open '(' and terminate only by seeing a		*
* close ')'. It will read another string in from the file if necessary. 	*
* The binary order is exactly the reverse of the SVF Hex format.			*
* 													*
* numbits:          is the number of bits of data need to convert.		*
* HexString:        is the data string to convert.					*
* end:              is to tell if the semicolon is encountered following	*
*                   the close ')'.    							*
* options:          0 = data are not save. e.g. smask data				*
*                   1 = save selectively. e.g. mask data 				*
*                   2 = compress data. e.g. SDR TDI TDO only. SIR never.	*
*                   3 = save selectively and compressed.				*
*													*
* This routine will return a string of given number of bits. 			*
*													*
******************************************************************************/

short int  ConvertFromHexString(long int numbits, unsigned char *data_buf)
{  
	int          charcount;           /* counts during pass over the string */
	int          dataIdx;             /* cursor into dataarray[]        */
	int          i, j;                   /* hex value extract from the char         */
	char           cur_char;
	unsigned char  char_val;
	short int             rcode = 0;
	char                  *work_buf = NULL;          /*working memory*/
	
	charcount = dataIdx = 0;  
	
	if ( data_buf ) {
		if ( ( work_buf = ( char * ) malloc( numbits / 4 + 2 ) ) == NULL ) {
			return OUT_OF_MEMORY;
		}
	}
  
	/*search for the open bracket then close bracket*/
	while ( rcode == 0 ) {
		/*read next string if necessary*/
        rcode = Token( " (" );

		for ( i = 0; i < (int) strlen( g_pszSVFString ); i++ )
		{
            if ( ( cur_char = g_pszSVFString[ i ] ) == ')' ) {
				break;  /*end of current stream*/
			}

            if ( (data_buf) && ( isxdigit( cur_char ) ) ) { 
                work_buf[ charcount++ ] = cur_char;
			}
		}

        if ( cur_char == ')' ) {
			break;   /*done with the current row*/
		}
	}
	
	/*data_buf now contains the entire hex string. reverse it and convert it to binary*/
	if (data_buf) {   
		if ( charcount * 4 < numbits ) {
			dataIdx = numbits / 4;
			for ( i = charcount-1; i >= 0; i-- ) { /*push data forward*/
				work_buf[ dataIdx-- ] = work_buf[ i ];
			}
			
			while ( dataIdx >= 0 ) {
				work_buf[ dataIdx-- ] = '0'; /*fill the lower data with 0*/
			}
			charcount = numbits / 4 + 1;
		}
		
		dataIdx = 0;
		j = 0;
		
		for ( i = charcount - 1; i >= 0; i-- ) {
			char_val = ( unsigned char ) CharToHex( work_buf[ i ] );
			if ( j % 2 == 0 ) {
				data_buf[ dataIdx ] = 0x00;
				data_buf[ dataIdx ] |= ( unsigned char )( reverse( char_val ) << 4 );
			}
			else 
			{
				data_buf[ dataIdx ] |= ( unsigned char ) reverse( char_val );
				dataIdx++;
			}
			j++;
		}
	}
	
	if (work_buf != NULL) {
		free(work_buf);
	}

	return rcode;
}

/***********************************************************************
*
* convertToispSTREAM()
* converts SVF ASCII data stream to Bin stream
*
* 
************************************************************************/
short int convertToispSTREAM(long int numbits, unsigned char *data_buf, char options )
{
	int foundFF = 0;
	int FFcount;
	int bytes;
	int i;
	int j;
	char opt;
	char mode;
	unsigned char cur_char;
	unsigned char compr_char = 0x00;
	
	if ( numbits % 8 ) {
		bytes = numbits / 8 + 1;
	}
	else {
		bytes = numbits / 8;
	}
	opt = options;
	j = 0;
	mode = 0;

	/* Determine the compression mode recommended */
	if ( options >= Minimize ) {
		mode = ( char ) compressToispSTREAM( bytes, data_buf, &opt );
		if (opt == NoSave) {
			return OK;
		}

		if ( mode == 1 ) {
			compr_char = 0x00;
		}
		else if ( mode == 2 ) {
			compr_char = 0xFF;
		}
		
		write( mode );   
		if ( ( mode >= 3 ) && ( mode % 2 ) ) {
			/* For compress by nibble, if mode is odd, inc it to fit byte boundary */
			mode++;  
		}
	}
	if ( opt ) {  
		foundFF = FFcount = 0;
		for ( i = 0; i < bytes; i++ ) {
			/* Turn off the store when enough nibbles are stored */
			if ( ( mode >= 3 ) && ( i >= mode / 2 ) ) {
				opt = NoSave;
			}
			
			cur_char = data_buf[ i ];
			if ( ( opt >= Minimize ) && ( foundFF ) && ( cur_char == compr_char ) ) {
				/* 0xFF again */
				FFcount++;
			}

			else if ( ( opt >= Minimize ) && ( foundFF ) ) {
				/* More than 1 and last 0xFF found */
				ConvNumber( FFcount );

				/* Number of 0xFF bytes */  
				foundFF = FFcount = 0;
				if ( cur_char == compr_char ) {
					foundFF = 1;
				}
				write( cur_char );
			}
			else if ( ( opt >= Minimize ) && ( cur_char == compr_char ) && ( mode < 3 ) && ( mode != -93 ) ) {
				foundFF = 1;    
				write( cur_char );
			}
			else if ( opt ) {
				write( cur_char );
			}
		}
	}

	if ( foundFF ) {
		/* More than 1 and last 0xFF found */
		if ( cur_char == compr_char ) {
			/* Last nibble also is key */
			FFcount++;
		}

		/* Number of 0xFF or 0x00 bytes */
		ConvNumber( FFcount );
	}

	return 0;
}

/***********************************************************************
*
* compressispSTREAM()
* This is to analyze the incoming stream to determine if compression is
* feasible:
* Return  0=> No Compression is recommended.
*         1=> Compression by 0x00.
*         2=> Compression by 0xFF.
*       >=3=> compression by nibble count. 
*             Example: given stream 012301230123 
*             Return value should be 4 since the stream can be compressed
*             as 012302. 
* 
************************************************************************/
short int compressToispSTREAM( int bytes, unsigned char * data_buf, char * options )
{	
	int ori_bytes;
	int i;
	int j;
	int k;
	int m;
	int bits;
	int foundFF;
	int table[ 16];
	int bytetable[ 256 ];
	int bytecount;
	int occurance;
	int rcode;
	int comprkey;
	int count00 = 0;
	int countff = 0;
	unsigned char key;
	unsigned char FFcount = 0;
	unsigned char cur_char;
	unsigned char compr_char;
	unsigned char xch;
	
	ori_bytes = bytes;
	rcode = 0;

	/* If length is less than 3 bytes, don't waste time to compress */
	if (bytes < 3) {
		*options = Store;
		return 0x00;
	}
	
	/* Intitialize the lookup table */
	for ( i = 0; i < 16; i++ ) {
		table[ i ] = 0;   
	}
	for ( i = 0; i < 256; i++ ) {
		bytetable[ i ] = 0;  
	}
	
	/* Build the data occurance frequency table */
	for ( i = 0; i < bytes; i++ ) {
		table[ ( int ) ( data_buf[ i ] >> 4 ) ] += 1;    /* Check the high nibble */
		table[ ( int ) ( data_buf[ i ] & 0x0F ) ] += 1;   /* Check the low nibble */
		if ( data_buf[ i ] == 0x00 ) {
			count00++;
		}
		if ( data_buf[ i ] == 0xFF ) {
			countff++;
		}
	}

	/* Perform compression dry run to see if saving is compress by 0x00 and 0xFF */
	if ( count00 > countff ) {
		compr_char = 0x00;
	}
	else {
		compr_char = 0xFF;
	}
	
	foundFF = FFcount = 0;
	bytecount = 0;
	cur_char = 0x00;
	for ( i = 0; i < bytes; i++ ) {
		cur_char = data_buf[ i ];
		if ( ( foundFF ) && ( cur_char == compr_char ) && ( FFcount < 0xFF ) ) {
			/* 0xFF again */
			FFcount++;
		}
		else if ( foundFF ) {
			/* More than 1 and last 0xFF found */
			bytecount++;
			foundFF = FFcount = 0;
			if ( cur_char == compr_char ) {
				foundFF = 1; 
			}
			bytecount++;
		}
		else if ( cur_char == compr_char ) {
			foundFF = 1;    
			bytecount++;
		}
		else 
		{
			bytecount++;
		}

		bytetable[ ( int ) cur_char ] += 1;
	}      
	
	j = bytetable[ 0 ];
	key = 0;
	for ( i = 0; i < 256; i++ ) {
		if ( bytetable[ i ] > j ) {
			j = bytetable[ i ]; 
			key = i;
		}
	}

	/* Calculate the bytes needed to perform compression using the highest occurance byte as the key */
	comprkey =( ( ( bytes - j ) * ( 8 + 1 ) + j ) / 8 ) + 1;
	if ( bytecount < bytes ) { 
		/* Compress by 0xFF or 0x00 recommended */
		if ( compr_char == 0x00 ) {
			rcode = 1;
		}
		else {
			rcode = 2;
		}
	}
    else {
		/* Try multiple nibble alternative */
		occurance = 0;
		
		/* Find the lowest number of occurance */
		for ( i = 0; i < 16; i++ ) {
			if ( table[ i] > 0 ) {
				if ( occurance == 0 ) {
					occurance = table[ i ];
				}
				else if ( table[ i ] < occurance ) {
					occurance = table[ i ];
				}
			}
		}
		
		/* The number of nibbles as the key */
		bits = bytes * 2 / occurance;
		
		for ( i = 1; i < occurance; i++ ) {
			/* Check if the stream can be mapped into keys */
			for ( j = 0; j < bits; j++ ) {
				/* The next first nibble of the key */
				m = j + i * bits;
				if ( ( 0x0F & ( data_buf[ j / 2 ] >> 4 * ( 1 - j % 2 ) ) ) != ( 0x0F & ( data_buf[ m / 2 ] >> 4 * ( 1 - m % 2 ) ) ) ) {
					/* No compression possible */
					*options = Store;
					rcode = 0;
					break;
				}
			}
		}

		if ( ( *options == Store ) || ( occurance == 1 ) ) {
			rcode = 0x00;
		}
		else {
			/* tnt 10/19/02: if the number of nibbles is the same 
							size as the orginal number of bytes, then 
							do not compress.  This is done to fix the 
							problem with 5512VE part */
			if ( bits == ori_bytes ) {
				*options = Store;
				rcode = 0x00;
			}
			else if( ((bytes * 2) % occurance))
			{
				*options = Store;
				rcode = 0x00;
			}
			else {
				rcode = bits;
			}
		}
	}
	
	/* If all fail and compress by key is the best */
	if ( ( rcode < 3 ) && ( comprkey < bytecount - 1 ) && ( comprkey < bytes ) ) {
		write( 0xFF );
		write( ( unsigned char ) key );
		xch = 0x00;
		bits = 0;
		cur_char = 0x00;
		for ( i = 0; i < bytes; i++ ) {
			cur_char = data_buf[ i ];
			if ( cur_char == key ) {
				bits++;
				m = 0; 
			}
			else {
				xch |= 0x80 >> bits++; 
				m = 8;
			}
			k = 0;
			do {
				if ( bits > 7 ) {
					write( xch ); 
					bits = 0; 
					xch = 0x00;
				}
				if ( m ) { 
					xch |= ( ( cur_char << k ) & 0x80 ) >> bits; 
					bits++;
				}
			} while ( ++k < m );
			
			if (bits > 7) {
				write( xch );
				bits = 0; 
				xch = 0x00;
			}
		}
		
		if ( i % 8 ) {
			write( xch ); 
			bits = 0; 
			xch = 0x00;
		}

		/* Return back that it is all done */
		*options = NoSave; 
		return 0x00;
	}

	if ( rcode == 0x00 ) {
		*options = Store;
	}
	return ( rcode );
}

/************************************************************************
*                                                                       *
* CharToHex()                                                           *
* converts ASCII letter, assumed to be a Hex digit to an integer        *
*                                                                       *
************************************************************************/
short int CharToHex( char hexNibbleChar ) 
{
	int hexdigit;
	
	if ( hexNibbleChar >= '0' && hexNibbleChar <= '9' ) {
		hexdigit = hexNibbleChar - '0';
	}
	else if ( hexNibbleChar >= 'a' && hexNibbleChar <= 'f' ) {
		hexdigit = hexNibbleChar - 'a' + 10;
	}
	else if ( hexNibbleChar >= 'A' && hexNibbleChar <='F' ) {
		hexdigit = hexNibbleChar - 'A' +10;
	}
	else {
		hexdigit = 0;
	}
	
	return hexdigit;
}

/************************************************************************
*                                                                      	*
* write()		  									                    *
* write data to a file or to a port;                                   	*
* counts the number of bytes written if inside a repeat loop;           *
* counts the number of total bytes in the VME file                      *
*												                        *
* Token  It is to distinguish between data or Token code.    		    * 
*        Token code is not sent to the parallel port               	    *
*        0=>it is pure data                                            	*
*        1=>it is a token                                              	*
* data   the content of data to be written to a file.				    *
************************************************************************/
int write(unsigned char data)
{
	if ((g_direct_prog) && (write_handler != NULL))
		return write_handler(WRITE_HANDLER_BYTE_CMD, data);


	if ( g_usFlowControlRegister & INTEL_PRGM ) {

		/*********************************************************************
		*
		* If intelligent programming flag is set, then write data into a buffer.
		*
		*********************************************************************/

		if ( g_usIntelBufferSize <= g_usIntelBufferIndex ) {			
			g_usIntelBufferSize *= 2;
			g_ucIntelBuffer = ( unsigned char * ) realloc( g_ucIntelBuffer, g_usIntelBufferSize * sizeof( unsigned char ) );
		}

		g_ucIntelBuffer[ g_usIntelBufferIndex++ ] = data;
	}
	else {

		/*********************************************************************
		*
		* Write data immediately to file.
		*
		*********************************************************************/

		fputc(data, g_pVMEFile);
	}
	return 0;
} 

/************************************************************************
*												*
* reverse()											*
* swap the 4 bits Hex number								*
*												*
************************************************************************/
unsigned char reverse( unsigned char a_cDigit )
{
	char cBitIndex;
	unsigned char cCurChar = 0x00;

	for ( cBitIndex = 0; cBitIndex < 4; cBitIndex++ ) {
		if ( ( a_cDigit >> cBitIndex ) & 0x01 ) {
			cCurChar |= 0x01 << ( 3 - cBitIndex );
		}
	}

	return ( cCurChar );
}

/************************************************************************ 																		*
* AllocateCFGMemory()									*
* Allocate memory for chain structure                                   *
* * Input:							                        *
*		Total of devices in chain						*
* * Output:											*
*		0 if false									*
*												*
************************************************************************/ 

int AllocateCFGMemory( int struct_size )
{
	if ( cfgChain != NULL ) {
		free( cfgChain );
	}
	cfgChain = NULL;
	if ( ( cfgChain =( CFG * ) calloc( struct_size + 1, sizeof( CFG ) ) ) == NULL ) {
		free( cfgChain );
		printf( "\nOut of Memory!\n" );
		return false;               
	}
	return true;
}

/************************************************************************ 																		*
* DeAllocateCFGMemory()							                		*
* Deallocate memory used to store chain information                     *
* Input:							                                    *
*																		*
* Output:																*
*																		*
*																		*
************************************************************************/ 

void DeAllocateCFGMemory()
{
	if ( cfgChain != NULL ) {
		free( cfgChain );
	}
}


/************************************************************************
*												*
* PrintHelp()										*
* Print the useage menu on console							*
*												*
************************************************************************/
void PrintHelp(void)
{
	printf( "Usage: svf2vme [ -help |\n" );
	printf( "               [ -full ]\n" );
	printf( "                 -infile  < input file path >  [ -clock < frequency > ]\n" );
	printf( "                                               [ -vendor < altera | xilinx > ]\n" );
	printf( "                                               [ -max_tck < max_tck > ]\n" );
	printf( "               [ -infile  < input file path >  [ -clock < frequency > ]\n" );
	printf( "                                               [ -vendor < altera | xilinx > ]\n" );
	printf( "                                               [ -max_tck < max_tck > ]\n" );
	printf( "                                               [ -max_size < max_buffer_size > ]\n" );
	printf( "               [ -bypass < instruction register length > ]\n" );
	printf( "               [ -outfile < output file path > ]\n" );
	printf( "               [ -prog  < jtag program interface path > ]\n" );
	printf( "               [ -comment ]\n" );
	printf( "               [ -header < header string > ]\n" );
	printf( "               ]\n" );
	printf( "Descriptions:           \n" );
	printf( "    -help:    Displays usage.\n" );
	printf( "    -full:    Disables compression.\n" );
	printf( "              Default: compression is on.\n" );
	printf( "    -infile:  Specifies the input SVF file.\n" );
	printf( "    -clock:   Overwrite the frequency of the SVF file.\n" );
	printf( "              Default: frequency based on SVF file or 1 MHz if not provided.\n" );
	printf( "    -vendor:  Specifies the vendor of the SVF file.\n" );
	printf( "              Default: JTAG standard.\n" );
	printf( "    -max_tck: Specifies the maximum TCK. Any remaining TCK will be converted to delay.\n" );
	printf( "              Default: no maximum TCK.\n" );
	printf( "    -max_size: Specifies the maximum value allowed to allocate memory for a row of data in Kbytes.\n" );
	printf( "              Ex. 8,16,32,64...Default: 64 KBytes.\n" );
	printf( "    -bypass:  Specifies the instruction register length of the bypassed device.\n" );
	printf( "    -outfile: Specifies the output VME file name.\n" );
	printf( "              Default: Uses the input file name with *.vme extension.\n" );
	printf( "    -comment: Generates VME file with the SVF comments displayed during processing.\n" );
	printf( "              Default: comments are off.\n" );
	printf( "    -header:  Generates VME file with the specified header.\n" );
	printf( "              Default: header are off.\n" );
	printf( "    -prog:    Run direct device program instead of generate vme file.\n" );

	printf( "Examples:               \n" );
	printf( "    svf2vme -infile c:\\file.svf -clock 10K -max_tck 1000 -max_size 64\n" );
	printf( "    svf2vme -infile c:\\file1.svf -clock 25M -infile c:\\file2.svf -vendor altera\n" );
	printf( "    svf2vme -bypass 8 -infile c:\\file.svf -clock 10K -outfile c:\\file.vme \n" );
	printf( "    svf2vme -infile c:\\file.svf -header \"CREATED BY:ispVM System Version 17.3\"\n" );
	printf( "\n" );
	printf( "See the readme.txt for more information.               \n\n" );
	
}
/**************************************************************************
*                            MAIN                            		  *
*                                                            		  *
* 3/2/2000 Howard Tang  support multiple svf files conversion:		  *
*                       example: svf2vme svf1.svf 6 svf2.svf ...          *
* 5/31/2000 H. Tang  V2.01 Add support to short form SVF and space or     *
*                          no space between TDx and xMASK with ( and ).   *
* 12/19/00  H. Tang  V2.02 Read a new line if reaches end of line.        *
* 01/02/01  H. Tang  V2.03 Fix memory problem when splitting long SDR     *
* 03/28/01  Nguyen   V2.04 Accept \t as delimiter on strtok() calls.      *
*                          The SVF files generated by ispVM 9.0.1 have    *
*                          TABs in it.                                    *
* 04/26/01  H. Tang  V3.00 Add support to multiple devices in a single    *
*                          SVF file generated by ispVM 9.0.x              *
* 08/28/01  H. Tang  V9.00 change it to support VME V9.0 format.          *
* 5/24/06   H. Tang        Support TRST pin toggling.                     *
***************************************************************************/

int main( int argc, char *argv[] )
{
	int iRetCode;
	int iCommandLineIndex;
	int iFullVMEOption = 0;
	int iSVFCount = 0;
	int iBypassCount = 0;
	int iCurrentSVFCount = 0;
	int iTemp = 0;
	char * szTmp = NULL;
	char szVMEFilename[ 1024 ] = { 0 };
	char szCommandLineArg[1024] = { 0 };
	char szErrorMessage[ 1024 ] = { 0 };
	char JTAGpath[ 1024 ] = { 0 };
	FILE * fptrVMEFile = NULL;
	int JTAGfrq;
	struct aspeed_jtag_runtest_idle runtest;

	printf( "              Mellanox Technologies Ltd.\n" );
	printf( "     JTAG svf player Version %s Copyright 2017\n\n", VME_VERSION_NUMBER );
		
	if ( argc < 2 )
	{
		PrintHelp();
		exit( ERR_COMMAND_LINE_SYNTAX );
	}

	write_handler = null_handler;

	/* Pre-process the command line arguments to count the number of SVF files and bypasses given */
	for ( iCommandLineIndex = 1; iCommandLineIndex < argc; iCommandLineIndex++ ) {
		strcpy( szCommandLineArg, argv[ iCommandLineIndex ] );
		if ( !stricmp( szCommandLineArg, "-infile" ) || !stricmp( szCommandLineArg, "-if" ) ) {
			iSVFCount++;
		}
		else if ( !strcmp( szCommandLineArg, "-bypass" ) || !strcmp( szCommandLineArg, "-by" ) ) {
			iBypassCount++;
		}
	}

	if ( iSVFCount <= 0 ) {
		sprintf( szErrorMessage, "Error: missing required argument -infile < input file >.\n\n" );
		printf( "%s", szErrorMessage );
		PrintHelp();
		exit( ERR_COMMAND_LINE_SYNTAX );
	}
	iSVFCount += iBypassCount;

	/* Allocate memory for chain setup */
	iRetCode = AllocateCFGMemory( iSVFCount );
	if ( !iRetCode ) {
		sprintf( szErrorMessage, "Error: system out of memory.\n\n" );
		printf( "%s", szErrorMessage );
		exit( OUT_OF_MEMORY );
	}

	for ( iCommandLineIndex = 1; iCommandLineIndex < argc; iCommandLineIndex++ ) {
		strcpy( szCommandLineArg, argv[ iCommandLineIndex ] );
		strlwr( szCommandLineArg );
		if ( !strcmp( szCommandLineArg, "-help" ) || !strcmp( szCommandLineArg, "-h" ) ) {
			PrintHelp();
			exit( OK_SHOW_HELP );
		}
		else if ( !strcmp( szCommandLineArg, "-full" ) || !strcmp( szCommandLineArg, "-f" ) ) {
			iFullVMEOption = 1;
		}
		else if ( !strcmp( szCommandLineArg, "-infile" ) || !strcmp( szCommandLineArg, "-if" ) ) {
			iRetCode = GetSVFInformation( &iCommandLineIndex, &iCurrentSVFCount, argc, argv, szErrorMessage );
			if ( iRetCode < 0 ) {
				printf( "%s", szErrorMessage );
				if ( iRetCode == ERR_COMMAND_LINE_SYNTAX ) {
					PrintHelp();
				}
				exit( iRetCode );
			}
		}
		else if ( !strcmp( szCommandLineArg, "-bypass" ) || !strcmp( szCommandLineArg, "-by" ) ) {
			if ( ++iCommandLineIndex >= argc ) {
				sprintf( szErrorMessage, "Error: missing bypass length.\n\n" );
				printf( "%s", szErrorMessage );
				exit( ERR_COMMAND_LINE_SYNTAX );
			}

			strcpy( szCommandLineArg, argv[ iCommandLineIndex ] );
			for ( iTemp = 0; iTemp < ( signed int ) strlen( szCommandLineArg ); iTemp++ ) {
				if ( !isdigit( szCommandLineArg[ iTemp ] ) ) {
					sprintf( szErrorMessage, "Error: bypass length %s is not a number.\n\n", szCommandLineArg );
					printf( "%s", szErrorMessage );
					exit( ERR_COMMAND_LINE_SYNTAX );
				}
			}

			strcpy( cfgChain[ iCurrentSVFCount ].name, "JTAG" );
			strcpy( cfgChain[ iCurrentSVFCount ].Svffile, "" );
			cfgChain[ iCurrentSVFCount ].inst = atoi( szCommandLineArg );   
			strcpy( cfgChain[ iCurrentSVFCount ].Vendor, "lattice" );
			cfgChain[ iCurrentSVFCount ].Frequency = 0;
			iCurrentSVFCount++;
		}
		else if ( !strcmp( szCommandLineArg, "-outfile" ) || !strcmp( szCommandLineArg, "-of" ) ) {
			if ( ++iCommandLineIndex >= argc ) {
				sprintf( szErrorMessage, "Error: missing output file name.\n\n" );
				printf( "%s", szErrorMessage );
				exit( ERR_COMMAND_LINE_SYNTAX );
			}

			strcpy( szVMEFilename, argv[ iCommandLineIndex ] );
			fptrVMEFile = fopen( szVMEFilename, "w" );
			if ( fptrVMEFile == NULL ) {
				sprintf( szErrorMessage, "Error: unable to write to output file %s\n\n", szVMEFilename );
				printf( "%s", szErrorMessage );
				exit( FILE_NOT_VALID );
			}
			remove( szVMEFilename );
		}
		else if ( !strcmp( szCommandLineArg, "-comment" ) ) {
			g_ucComment = 1;			
		}
		else if(!strcmp( szCommandLineArg, "-header" ))
		{
			if ( ++iCommandLineIndex >= argc ) {
				sprintf( szErrorMessage, "Error: missing header string.\n\n" );
				printf( "%s", szErrorMessage );
				exit( ERR_COMMAND_LINE_SYNTAX );
			}
			g_ucHeader = 1;
			strcpy(g_cHeader,argv[ iCommandLineIndex ]);
		}else if(!strcmp( szCommandLineArg, "-prog" )){
			if ( ++iCommandLineIndex >= argc ) {
				sprintf( szErrorMessage, "Error: missing jtag interface path.\n\n" );
				printf( "%s", szErrorMessage );
				exit( ERR_COMMAND_LINE_SYNTAX );
			}
			strcpy(JTAGpath, argv[ iCommandLineIndex ]);
#ifdef DISABLE_JTAG_PROG
			g_direct_prog = 1;
#else
			g_JTAGFile = open(JTAGpath, O_RDWR);
			if(g_JTAGFile == -1){
				sprintf( szErrorMessage, "Error: can't open JTAG interface file.\n\n" );
				printf( "%s", szErrorMessage );
				exit( ERR_COMMAND_LINE_SYNTAX );
			} else {
				g_direct_prog = 1;
				iFullVMEOption = 1;
			}
#endif
		}
		else {
			sprintf( szErrorMessage, "Error: %s is an unrecognized or misplaced argument.\n\n", szCommandLineArg );
			printf( "%s", szErrorMessage );
			PrintHelp();
			exit( ERR_COMMAND_LINE_SYNTAX );
		}
	}

	if ( szVMEFilename[0] == '\0' ) {
		/* If no output file then use the name of the first SVF file */
        for ( iTemp = 0; iTemp < iCurrentSVFCount; iTemp++ ) {
            if ( !stricmp( cfgChain[ iTemp ].name, "SVF" ) ) {
				break;
			}
		}

		strcpy( szCommandLineArg, cfgChain[ iTemp ].Svffile );
		szTmp = &szCommandLineArg[ strlen( szCommandLineArg ) - 4 ];
		if ( szTmp ) {
			*szTmp = '\0';
		}
		sprintf( szVMEFilename, "%s.vme", szCommandLineArg );
	}

	for ( iTemp = 0; iTemp < iCurrentSVFCount; iTemp++ ) {
		if ( !stricmp( cfgChain[ iTemp ].name, "SVF" ) ) {

			if ( ( g_pSVFFile = fopen( cfgChain[ iTemp ].Svffile, "r" ) ) == NULL )
			{
				printf( szErrorMessage, "Error: svf file %s cannot be read.\n\n", cfgChain[ iTemp ].Svffile );
				exit( FILE_NOT_FOUND );
			}

			/* Get instruction length */
			while ( fgets( buffer, strmax, g_pSVFFile ) != NULL ) {
				g_pszSVFString = strtok( buffer, "\t " );
				if ( !stricmp( g_pszSVFString, "SIR" ) ) {
					g_pszSVFString = strtok(NULL, "\t (");
					cfgChain[ iTemp ].inst = atoi( g_pszSVFString );
					break;
				}
			}

			/* Pre-process the file to find the working memory size */
			rewind( g_pSVFFile );
			while ( fgets( buffer, strmax, g_pSVFFile ) != NULL ) {
				g_pszSVFString = strtok( buffer, " \t" );
				if ( !stricmp( g_pszSVFString, "SDR" ) ) {
					g_pszSVFString = strtok( NULL, "\t (" );
			
					if ( atol( g_pszSVFString ) > g_iMaxSize ) {
						g_iMaxSize = atol(g_pszSVFString); /* Keep the largest */
					}
				}

				// Rev. 12.2 Chuo add checking memory size for SIR
				if ( !stricmp( g_pszSVFString, "SIR" ) ) {
					g_pszSVFString = strtok( NULL, "\t (" );
			
					if ( atol( g_pszSVFString ) > g_iMaxSize ) {
						g_iMaxSize = atol(g_pszSVFString); /* Keep the largest */
					}
				}
			}
			if ( g_iMaxSize >( long int ) g_iMaxBufferSize ) {
				g_iMaxSize =( long int ) g_iMaxBufferSize;   /* Maximum memory needed for a row of data */
			}
			fclose( g_pSVFFile );
		}
	}

	jtag_handlers_init();

#ifndef DISABLE_JTAG_PROG
	if (g_direct_prog){
		iFullVMEOption = 1;
		g_direct_prog = 1;

		JTAGfrq = 20000;
		ioctl(g_JTAGFile, ASPEED_JTAG_SIOCFREQ, &JTAGfrq);

		runtest.end = 0;
		runtest.mode = ASPEED_JTAG_XFER_SW_MODE;
		runtest.reset = 0;
		runtest.tck = 0;
		ioctl(g_JTAGFile, ASPEED_JTAG_IOCRUNTEST, &runtest);
	}
#endif

	if ( iFullVMEOption ) {
		if (!g_direct_prog){
			printf( "Begin generating the full VME file \n(%s)......\n\n", szVMEFilename );
		}
		iRetCode = ispsvf_convert( iSVFCount, cfgChain, szVMEFilename, false ); 
	}	
	else
	{ 
		printf( "Begin generating the compressed VME file \n(%s)......\n\n", szVMEFilename );
		iRetCode = ispsvf_convert( iSVFCount, cfgChain, szVMEFilename, true ); 
	}
#ifndef DISABLE_JTAG_PROG
	close(g_JTAGFile);
#endif
	/* Free chain memory */
	DeAllocateCFGMemory();

	if ( iRetCode < 0 )
	{
		remove( szVMEFilename );
		printf( "Failed at SVF line %d in generating the VME file......\n\n", g_iSVFLineIndex );
		printf( "+-------+\n" );
		printf( "| FAIL! |\n" );
		printf( "+-------+\n\n" );
	}
	else
	{
		printf( "+=======+\n" );
		printf( "| PASS! |\n" );
		printf( "+=======+\n\n" );
		iRetCode = OK;
	} 
	exit( iRetCode );
}

/***********************************************************************************
*GetSVFInformation()     														   *
*																				   *
*This function is used to parse the incoming commandline SVF files                 *
*																				   *
************************************************************************************/

int GetSVFInformation( int * a_piCommandLineIndex, int * a_piCurrentSVFCount, int a_iArgc, char * a_cArgv[], char * a_szErrorMessage )
{
	int iTemp;
	char szTmp[ 1024 ] = { 0 };
	char szCommandLineArg[ 1024 ] = { 0 };
	char szSVFFilename[ 1024 ] = { 0 };
	FILE * fptrSVFFile = NULL;

	if ( ++*a_piCommandLineIndex >= a_iArgc ) {
		sprintf( a_szErrorMessage, "Error: missing input file name.\n\n" );
		return ( ERR_COMMAND_LINE_SYNTAX );
	}

	strcpy( szSVFFilename, a_cArgv[ *a_piCommandLineIndex ] );
	strcpy( szTmp,  &szSVFFilename[ strlen( szSVFFilename ) - 4 ] );
	if ( stricmp( szTmp, ".svf" ) ) {
		sprintf( a_szErrorMessage, "Error: input file %s must have *.svf extension.\n\n", szSVFFilename );
		return ( ERR_COMMAND_LINE_SYNTAX );
	}

	fptrSVFFile = fopen( szSVFFilename, "r" );
	if ( fptrSVFFile == NULL ) {
		sprintf( a_szErrorMessage, "Error: svf file %s cannot be read.\n\n", szSVFFilename );
		return ( FILE_NOT_VALID );
	}
	fclose( fptrSVFFile );
	strcpy( cfgChain[ *a_piCurrentSVFCount ].Svffile, szSVFFilename );

	/* Set default SVF file name, frequency, vendor, and max tck */
	strcpy( cfgChain[ *a_piCurrentSVFCount ].name, "SVF" );
	cfgChain[ *a_piCurrentSVFCount ].Frequency = 0;
	strcpy( cfgChain[ *a_piCurrentSVFCount ].Vendor, "lattice" );
	cfgChain[ *a_piCurrentSVFCount ].MaxTCK = 1000;
	cfgChain[ *a_piCurrentSVFCount ].noMaxTCK = 0;

	/* Parse for arguments to the SVF file */
	while ( ++*a_piCommandLineIndex < a_iArgc ) {
		strcpy( szCommandLineArg, a_cArgv[ *a_piCommandLineIndex ] );
		if ( !stricmp( szCommandLineArg, "-vendor" ) || !stricmp( szCommandLineArg, "-v" ) ) {
			if ( ++*a_piCommandLineIndex >= a_iArgc ) {
				sprintf( a_szErrorMessage, "Error: missing vendor input.\n\n" );
				return ( ERR_COMMAND_LINE_SYNTAX );
			}
			strcpy( szCommandLineArg, a_cArgv[ *a_piCommandLineIndex ] );
			if ( !stricmp( szCommandLineArg, "altera" ) ) {
				strcpy( cfgChain[ *a_piCurrentSVFCount ].Vendor, "altera" );
			}
			else if ( !stricmp( szCommandLineArg, "xilinx" ) ) {
				strcpy( cfgChain[ *a_piCurrentSVFCount ].Vendor, "xilinx" );
			}
			else if ( !stricmp( szCommandLineArg, "lattice" ) ) {
				strcpy( cfgChain[ *a_piCurrentSVFCount ].Vendor, "lattice" );
			}
			else {
				sprintf( a_szErrorMessage, "Error: %s is an unrecognized vendor.\n\n", szCommandLineArg );
				return ( ERR_COMMAND_LINE_SYNTAX );
			}
		}
		else if ( !stricmp( szCommandLineArg, "-clock" ) || !stricmp( szCommandLineArg, "-c" ) ) {
			if ( ++*a_piCommandLineIndex >= a_iArgc ) {
				sprintf( a_szErrorMessage, "Error: missing frequency input.\n\n" );
				return ( ERR_COMMAND_LINE_SYNTAX );
			}
			strcpy( szCommandLineArg, a_cArgv[ *a_piCommandLineIndex ] );
			strlwr( szCommandLineArg );
			if ( szCommandLineArg[ strlen( szCommandLineArg ) -1 ] == 'k' || szCommandLineArg[ strlen( szCommandLineArg ) -1 ] == 'm' ) {
				for ( iTemp = 0; iTemp < ( signed int ) strlen( szCommandLineArg ) - 1; iTemp++ ) {
					if ( !isdigit( szCommandLineArg[ iTemp ] ) ) {
						sprintf( a_szErrorMessage, "Error: %s is an invalid frequency setting.\n\n", szCommandLineArg );
						return ( ERR_COMMAND_LINE_SYNTAX );
					}
				}
				if ( strchr( szCommandLineArg, 'k' ) ) {
					cfgChain[ *a_piCurrentSVFCount ].Frequency = atoi( szCommandLineArg ) * 1000;
				}
				else {
					cfgChain[ *a_piCurrentSVFCount ].Frequency = atoi( szCommandLineArg ) * 1000000;
				}
			}
			else {
				sprintf( a_szErrorMessage, "Error: %s is an unrecognized frequency.\n\n", szCommandLineArg );
				return ( ERR_COMMAND_LINE_SYNTAX );
			}
		}
		else if ( !stricmp( szCommandLineArg, "-max_tck" ) ) {
			if ( ++*a_piCommandLineIndex >= a_iArgc ) {
				sprintf( a_szErrorMessage, "Error: missing max_tck input.\n\n" );
				return ( ERR_COMMAND_LINE_SYNTAX );
			}

			strcpy( szCommandLineArg, a_cArgv[ *a_piCommandLineIndex ] );
			strlwr(szCommandLineArg);
			if(!strcmp(szCommandLineArg, "no")){
				cfgChain[ *a_piCurrentSVFCount ].noMaxTCK = 1;
			}
			else{
				if ( atoi( szCommandLineArg ) > 0 ) {
					cfgChain[ *a_piCurrentSVFCount ].MaxTCK = atoi( szCommandLineArg );
				}
				else {
					sprintf( a_szErrorMessage, "Error: max_tck must be greater than 0.\n\n" );
					return ( ERR_COMMAND_LINE_SYNTAX );
				}
			}
		}
		else if ( !stricmp( szCommandLineArg, "-max_size" ) ) {
			if ( ++*a_piCommandLineIndex >= a_iArgc ) {
				sprintf( a_szErrorMessage, "Error: missing maximum buffer size input.\n\n" );
				return ( ERR_COMMAND_LINE_SYNTAX );
			}
			
			strcpy( szCommandLineArg, a_cArgv[ *a_piCommandLineIndex ] );
			strlwr(szCommandLineArg);
			if ( atoi( szCommandLineArg ) > 0 ) {
				g_iMaxBufferSize  = atoi( szCommandLineArg );
				if((g_iMaxBufferSize != 8)&&
				   (g_iMaxBufferSize != 16)&&
				   (g_iMaxBufferSize != 32)&&
				   (g_iMaxBufferSize != 64)&&
				   (g_iMaxBufferSize != 128)&&
				   (g_iMaxBufferSize != 256))
				{
					sprintf( a_szErrorMessage, "Error: max_size must be equal to 8,16,32,64,128 or 256.\n\n" );
					return ( ERR_COMMAND_LINE_SYNTAX );
				}
				g_iMaxBufferSize = g_iMaxBufferSize * 1000;
			}
			else {
				sprintf( a_szErrorMessage, "Error: max_size must be greater than 0.\n\n" );
				return ( ERR_COMMAND_LINE_SYNTAX );
			}
		}
		else {
			( *a_piCommandLineIndex )--;
			break;
		}
	}

	( *a_piCurrentSVFCount )++;
	return OK;
}

void CalculateCRC( const unsigned char * a_pVMEBuffer, unsigned int a_iLength, unsigned short * a_pCalculatedCRC )
{
	unsigned int a_iIndex;
	unsigned char ucData;
	unsigned char ucTempData;
	unsigned char ucByteIndex;
	unsigned short usCRCTableEntry, usCalculatedCRC = 0;
	unsigned short crc_table[ 16 ] = {
		0x0000, 0xCC01, 0xD801,
		0x1400, 0xF001, 0x3C00,
		0x2800, 0xE401, 0xA001,
		0x6C00, 0x7800, 0xB401,
		0x5000, 0x9C01, 0x8801,
		0x4400
	};

	*a_pCalculatedCRC = 0;
	for ( a_iIndex = 0; a_iIndex < a_iLength; a_iIndex++ ) {
		ucData = 0;
		ucTempData = a_pVMEBuffer[ a_iIndex ];
		for ( ucByteIndex = 0; ucByteIndex < 8; ucByteIndex++ ) {
			ucData <<= 1;
			if ( ucTempData & 0x01 ) {
				ucData |= 0x01;
			}
			ucTempData >>= 1;
		}

		usCRCTableEntry = crc_table[ usCalculatedCRC & 0xF ];
		usCalculatedCRC = ( usCalculatedCRC >> 4 ) & 0x0FFF;
		usCalculatedCRC = usCalculatedCRC ^ usCRCTableEntry ^ crc_table[ ucData & 0xF ];
		usCRCTableEntry = crc_table[ usCalculatedCRC & 0xF ];
		usCalculatedCRC = ( usCalculatedCRC >> 4 ) & 0x0FFF;
		usCalculatedCRC = usCalculatedCRC ^ usCRCTableEntry ^ crc_table[ ( ucData >> 4 ) & 0xF ];
	}

	*a_pCalculatedCRC = usCalculatedCRC;
}

void EncodeCRC( const char * a_pszVMEFilename )
{
	FILE * pVMEFile;
	unsigned char * pVMEBuffer;
	unsigned int uiVMEBufferIndex = 0;
	unsigned int uiVMEFileSize = 0;
	unsigned short usCalculatedCRC = 0;
	int iReadChar;
	
	/*********************************************************************
	*
	* Count the size of the VME file and allocate a buffer based on the size.
	*
	*********************************************************************/

	pVMEFile = fopen( a_pszVMEFilename, "rb" );
	while( ( iReadChar = getc( pVMEFile ) ) != EOF) {
		uiVMEFileSize++;
	}
	pVMEBuffer = ( unsigned char * ) calloc( uiVMEFileSize + 1, sizeof( unsigned char ) );

	/*********************************************************************
	*
	* Read the VME file into a buffer.
	*
	*********************************************************************/

	rewind( pVMEFile );
	uiVMEBufferIndex = 0;
	while( ( iReadChar = getc( pVMEFile ) ) != EOF) {
		pVMEBuffer[ uiVMEBufferIndex++ ] = ( unsigned char ) iReadChar;
	}
	fclose( pVMEFile );
	pVMEFile = 0;

	/*********************************************************************
	*
	* Calculate the 16-bit crc of the VME buffer.
	*
	*********************************************************************/

	CalculateCRC( pVMEBuffer, uiVMEFileSize, &usCalculatedCRC );

	/*********************************************************************
	*
	* Open the file pointer to write the VME file.
	*
	*********************************************************************/

	pVMEFile = fopen( a_pszVMEFilename, "wb" );

	/*********************************************************************
	*
	* Write the FILE_CRC opcode followed by the 16-bit CRC.
	*
	*********************************************************************/

	fputc( FILE_CRC, pVMEFile );
	fputc( usCalculatedCRC >> 8, pVMEFile );
	fputc( usCalculatedCRC, pVMEFile );

	/*********************************************************************
	*
	* Write the entire VME buffer into the VME file.
	*
	*********************************************************************/

	for ( uiVMEBufferIndex = 0; uiVMEBufferIndex < uiVMEFileSize; uiVMEBufferIndex++ ) {
		fputc( pVMEBuffer[ uiVMEBufferIndex ], pVMEFile );
	}

	/*********************************************************************
	*
	* Close the file pointer and free memory.
	*
	*********************************************************************/

	fclose( pVMEFile );
	free( pVMEBuffer );
	pVMEFile = 0;
	pVMEBuffer = 0;
}

/*********************************************************************
*
* LCOUNTCom
*
* Convert the SVF LCOUNT command into VME format.
*
*********************************************************************/

void LCOUNTCom()
{
	long lCount; 

	write( LCOUNT );
	Token( "" );
	lCount = atol( g_pszSVFString );
	ConvNumber( lCount );
}

/*********************************************************************
*
* writeIntelProgramData
*
* Write the intelligent programming buffer into the file.
*
*********************************************************************/

void writeIntelProgramData()
{
	unsigned short usBufferIndex;

	/*********************************************************************
	*
	* Write the ENDLOOP opcode to signal the end of the intel buffer.
	*
	*********************************************************************/

	write( ENDLOOP );

	/*********************************************************************
	*
	* Set the intelligenet programming flag to false so that data will be
	* written onto the file instead of the buffer.
	*
	*********************************************************************/

	g_usFlowControlRegister &= ~INTEL_PRGM;

	ConvNumber( g_usIntelBufferIndex );
	for ( usBufferIndex = 0; usBufferIndex < g_usIntelBufferIndex; usBufferIndex++ ) {
		write( g_ucIntelBuffer[ usBufferIndex ] );
	}

	/*********************************************************************
	*
	* Re-initialize intelligent programming related variables.
	*
	*********************************************************************/

	free( g_ucIntelBuffer );
	g_ucIntelBuffer = 0;
	g_usIntelBufferIndex = 0;
}

/*********************************************************************
*
* LVDSCom
*
* Convert LVDS-SVF file to VME format.
*
*********************************************************************/

short int LVDSCom()
{
	short int siRetCode = 0;
	long int iLVDSPairCount = 0;
	long int iLVDSIndex = 0;
	bool bNumberConversion;
	bool * pbLVDSIndices = NULL;
	int iMaxSizeIndex = 0;

	Token( "(" );
	
	/*********************************************************************
	*
	* Get the number of LVDS pairs. Return error if the number is less
	* than or equal to zero.
	*
	*********************************************************************/

	iLVDSPairCount = get_number( g_pszSVFString, &bNumberConversion );
	if ( iLVDSPairCount <= 0 || !bNumberConversion ) {
		return FILE_NOT_VALID;
	}
	ConvNumber( iLVDSPairCount );

	/*********************************************************************
	*
	* Allocate memory to represent the LVDS indices. By default, initialize
	* them all to false. Later when reading the LVDS indices, set the
	* location to true. If the SVF tries to set the location more than
	* once then report an error.
	*
	*********************************************************************/

	pbLVDSIndices = (bool*) malloc( g_iMaxSize );
	for ( iMaxSizeIndex = 0; iMaxSizeIndex < g_iMaxSize; iMaxSizeIndex++ ) {
		pbLVDSIndices[ iMaxSizeIndex ] = false;
	}

	/*********************************************************************
	*
	* Iterate through the LVDS pairs in the SVF file.
	*
	*********************************************************************/

	for ( ; iLVDSPairCount > 0; iLVDSPairCount-- ) {
		
		siRetCode = Token( ":" );
		if ( siRetCode ) {
			siRetCode = FILE_NOT_VALID;
			break;
		}

		/*********************************************************************
		*
		* Convert number to integer.
		*
		*********************************************************************/

		iLVDSIndex = get_number( g_pszSVFString, &bNumberConversion );
		if ( iLVDSIndex < 0 || !bNumberConversion ) {
			siRetCode = FILE_NOT_VALID;
			break;
		}

		/*********************************************************************
		*
		* Check to make sure index location is unused.
		*
		*********************************************************************/

		if ( pbLVDSIndices[ iLVDSIndex ] ) {
			siRetCode = FILE_NOT_VALID;
			break;
		}

		/*********************************************************************
		*
		* Set index location to used (true) and write the number to the VME
		* file.
		*
		*********************************************************************/

		pbLVDSIndices[ iLVDSIndex ] = true;
		ConvNumber( iLVDSIndex );
		
		if ( iLVDSPairCount > 1 ) {
			siRetCode = Token( "," );
			if ( siRetCode ) {
				siRetCode = FILE_NOT_VALID;
				break;
			}

			/*********************************************************************
			*
			* Convert number to integer.
			*
			*********************************************************************/

			iLVDSIndex = get_number( g_pszSVFString, &bNumberConversion );
		}
		else {
			siRetCode = Token( ")" );
			if ( siRetCode ) {
				siRetCode = FILE_NOT_VALID;
				break;
			}

			/*********************************************************************
			*
			* Convert number to integer.
			*
			*********************************************************************/

			iLVDSIndex = get_number( g_pszSVFString, &bNumberConversion );
		}

		if ( iLVDSIndex < 0 || !bNumberConversion ) {
			siRetCode = FILE_NOT_VALID;
			break;
		}

		/*********************************************************************
		*
		* Check to make sure index location is unused.
		*
		*********************************************************************/

		if ( pbLVDSIndices[ iLVDSIndex ] ) {
			siRetCode = FILE_NOT_VALID;
			break;
		}

		/*********************************************************************
		*
		* Set index location to used (true) and write the number to the VME
		* file.
		*
		*********************************************************************/

		pbLVDSIndices[ iLVDSIndex ] = true;
		ConvNumber( iLVDSIndex );
	}

	/*********************************************************************
	*
	* Free dynamic memory.
	*
	*********************************************************************/

	free( pbLVDSIndices );
	pbLVDSIndices = 0;

	return ( siRetCode );
}
