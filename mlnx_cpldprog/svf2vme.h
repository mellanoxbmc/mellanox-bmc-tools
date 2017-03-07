
typedef struct {
    char name[ 30 ];				/* Device name */
    char Svffile[ 1024 ];			/* SVF Filenme */
    int inst;					    /* IR - Length */
    long int Frequency;             /* Frequency (in Hz) */
	char Vendor[ 100 ];             /* Vendor name */
	unsigned int MaxTCK;            /* Maximum TCK */
	// Rev. 12.2 Chuo add isMaxTCK flag
	unsigned char noMaxTCK;			/* Indicates Max TCK is set */
} CFG;						/*Chain configuration setup structure*/

short int ispsvf_convert(int chips, CFG * chain, char *vmefilename, bool compress );
short int ENDIRCom();
short int ENDDRCom();
short int HDRCom();
short int HIRCom();
short int  TDRCom();    
short int  TIRCom();
short int ScanCom( char types, bool compress );
short int RUNTESTCom( unsigned int max_tck, short int noMaxTCK );
short int FREQUENCYCom();
short int RESETCom(void);
short int STATECom();
short int LVDSCom();
short int TDIToken(long int  numbits,
               char types,
               bool compress );
short int  ConvertFromHexString(long int numbits,
                           unsigned char *Hexstring);
short int CharToHex(char hexNibbleChar);
short int compressToispSTREAM(int bytes, unsigned char *data_buf, char *options);
int write(unsigned char data);
unsigned char reverse( unsigned char a_cDigit );
short int convertToispSTREAM(long int charcount, unsigned char *data, char options );
int GetSVFInformation( int * a_piCommandLineIndex, int * a_piCurrentSVFCount, int a_iArgc, char * a_cArgv[], char * a_szErrorMessage );
