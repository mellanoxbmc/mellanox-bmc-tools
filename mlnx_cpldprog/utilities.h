#ifndef __UTILITIES_H__
#define __UTILITIES_H__

typedef unsigned char bool;

int stricmp( const char * a_szFirst, const char * a_szSecond );
char * strlwr( char * a_pszString );
int get_number( const char * a_pszString, bool * a_bReturnValue );
void trim_left( char * a_pszLine );
void trim_right( char * a_pszLine );
unsigned char reverse_bits(unsigned char n);

/* Constants */
#define strmax 1028      
#define NoSave   0x00
#define Store    0x01
#define Minimize 0x02
#define StableStateMax 5

#define    false                        0
#define    true                         1
#define		OK_SHOW_HELP				1
#define		OK          				0
#define		OUT_OF_MEMORY              -1
#define		FILE_NOT_FOUND             -8
#define		FILE_NOT_VALID             -9
#define		FILE_ERROR                -18
#define		ERR_COMMAND_LINE_SYNTAX	  -20

#endif /*__UTILITIES_H__*/
