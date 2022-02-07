// Definitionen für Debugging über die serielle Schnittstell

#ifndef DEBUGDEFS
#define DEBUGDEFS

#define DEBUG
//#define SERVODBG
//#define SIGNALDBG
//#define COILDBG
#define STATICDBG

#ifdef __AVR_MEGA__

#ifdef DEBUG
    #define DB_PRINT( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DB_PRINT_( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
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
#ifdef SIGNALDBG
    #define DBSG_PRINT( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBSG_PRINT_( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBSG_PRINT( x, ... ) ;
    #define DBSG_PRINT_( x, ... ) ;
#endif

#ifdef SERVODBG
    #define DBSV_PRINT( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBSV_PRINT_( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBSV_PRINT( x, ... ) ;
    #define DBSV_PRINT_( x, ... ) ;
#endif

#ifdef COILDBG
    #define DBCL_PRINT( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBCL_PRINT_( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBCL_PRINT( x, ... ) ;
    #define DBCL_PRINT_( x, ... ) ;
#endif

#ifdef STATICDBG
    #define DBST_PRINT( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBST_PRINT_( x, ... ) { char dbgbuf[60];sprintf_P( dbgbuf, (const char*) F( x ), ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBST_PRINT( x, ... ) ;
    #define DBST_PRINT_( x, ... ) ;
#endif

#else // für STM32F1
#ifdef DEBUG
    #define DB_PRINT( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DB_PRINT_( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DB_PRINT( x, ... ) ;
    #define DB_PRINT_( x, ... ) ;
#endif
#ifdef SIGNALDBG
    #define DBSG_PRINT( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBSG_PRINT_( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBSG_PRINT( x, ... ) ;
    #define DBSG_PRINT_( x, ... ) ;
#endif

#ifdef SERVODBG
    #define DBSV_PRINT( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBSV_PRINT_( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBSV_PRINT( x, ... ) ;
    #define DBSV_PRINT_( x, ... ) ;
#endif

#ifdef COILDBG
    #define DBCL_PRINT( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBCL_PRINT_( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBCL_PRINT( x, ... ) ;
    #define DBCL_PRINT_( x, ... ) ;
#endif

#ifdef STATICDBG
    #define DBST_PRINT( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.println( dbgbuf ); }
    #define DBST_PRINT_( x, ... ) { char dbgbuf[60];sprintf( dbgbuf, x, ##__VA_ARGS__ ) ; Serial.print( dbgbuf ); }
#else
    #define DBST_PRINT( x, ... ) ;
    #define DBST_PRINT_( x, ... ) ;
#endif

#endif // endif AVR/STM32
#endif
