#include <string.h>
#include <ctype.h>
#include <malloc.h>
#include <stdlib.h>
#include "utilities.h"

int stricmp( const char * a_szFirst, const char * a_szSecond )
{
	int iRetCode;
	char * pszFirst = NULL;
	char * pszSecond = NULL;

	if ( !a_szFirst || !a_szSecond ) {
		return -1;
	}

	pszFirst = ( char * ) malloc( strlen( a_szFirst ) + 1 );
	pszSecond = ( char * ) malloc( strlen( a_szSecond ) + 1 );
	
	if ( !pszFirst || !pszSecond ) {
		return -1;
	}

	strcpy( pszFirst, a_szFirst );
	strcpy( pszSecond, a_szSecond );
	strlwr( pszFirst );
	strlwr( pszSecond );

	iRetCode = strcmp( pszFirst, pszSecond );

	free( pszFirst );
	free( pszSecond );
	pszFirst = NULL;
	pszSecond = NULL;
	return ( iRetCode );
}

char * strlwr( char * a_pszString )
{
	unsigned int iIndex = 0;
	if ( !a_pszString ) {
		return a_pszString;
	}

	for( iIndex = 0; iIndex < strlen( a_pszString ); iIndex++ ) {
		a_pszString[ iIndex ] = tolower( a_pszString[ iIndex ] );
	}

	return a_pszString;
}

void trim_right( char * a_pszLine )
{
	int iIndex;

	if (!a_pszLine) {
		return;
	}

	for ( iIndex = strlen( a_pszLine ); iIndex >= 0; iIndex-- ) {
		if ( ( a_pszLine[ iIndex ] < 0x21 ) || ( a_pszLine[ iIndex ] > 0x7E ) ) {
			a_pszLine[ iIndex ] = '\0';
		}
		else {
			break;
		}
	}
}

void trim_left( char * a_pszLine )
{
	int iTemp;
	char * pszTempLine;

	if ( !a_pszLine ) {
		return;
	}

	pszTempLine = ( char * ) malloc( strlen( a_pszLine ) + 1 );
	strcpy( pszTempLine, a_pszLine );
	
	for ( iTemp = 0; iTemp < ( int ) strlen( a_pszLine ); iTemp++ ) {
		if ( ( a_pszLine[ iTemp ] >= 0x21 ) && ( a_pszLine[ iTemp ] <= 0x7E ) ) {
			break;
		}
	}

	strcpy( a_pszLine, pszTempLine + iTemp );
	free( pszTempLine );
	pszTempLine = NULL;
}

int get_number( const char * a_pszString, bool * a_bReturnValue )
{
	char * pszNumber = 0;
	int iIndex = 0;

	*a_bReturnValue = true;
	if ( !a_pszString ) {
		return false;
	}

	/*********************************************************************
	*
	* Allocate memory to store the number as a string. Trim white spaces.
	*
	*********************************************************************/

	pszNumber = ( char * ) malloc( strlen( a_pszString ) + 1 );
	strcpy( pszNumber, a_pszString );
	trim_left( pszNumber );
	trim_right( pszNumber );

	for( iIndex = 0; iIndex < ( int ) strlen( pszNumber ); iIndex++ ) {

		/*********************************************************************
		*
		* Check to make sure each character is a digit.
		*
		*********************************************************************/

		switch ( pszNumber[ iIndex ] ) {
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			break;
		default:
			*a_bReturnValue = false;
		}

		/*********************************************************************
		*
		* Found non-digit.
		*
		*********************************************************************/

		if ( *a_bReturnValue == false ) {
			break;
		}
	}

	iIndex = atoi( pszNumber );
	free( pszNumber );
	pszNumber = 0;

	return iIndex;
}
static unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

unsigned char reverse_bits(unsigned char n) {
   // Reverse the top and bottom nibble then swap them.
   return (lookup[n&0b1111] << 4) | lookup[n>>4];
}
