// Definitionen für Debugging über die serielle Schnittstell

#ifndef DEBUGDEFS
#define DEBUGDEFS

//#define DEBUG
#ifdef DEBUG
    #define DB_PRINT( x, ... ) { sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DB_PRINT_( x, ... ) { sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
    //#define DB_PRINT( x, ... ) { sprintf( dbgbuf,   x , __VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    extern char dbgbuf[60];
        #ifdef __AVR_MEGA__
            extern unsigned int __heap_start;
            extern char *__malloc_heap_start;
            extern char *__brkval;
            int freeMemory();
        #endif
#else
    #define DB_PRINT( x, ... ) ;
    #define DB_PRINT_( x, ... ) ;
#endif

#endif
