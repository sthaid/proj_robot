#ifndef __UTIL_MISC_H__
#define __UTIL_MISC_H__

#include <stdbool.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

// -----------------  GENERAL  -----------------------------------

// show the value of a define
#define SHOW_DEFINE(x) INFO("define %s = %s\n", #x, SHOW_DEFINE_STR(x))
#define SHOW_DEFINE_STR(x) #x

// for use in call to madvise
#define PAGE_SIZE 4096L
#define ROUND_UP(x,n) (((uint64_t)(x) + ((uint64_t)(n) - 1)) & ~((uint64_t)(n) - 1))  // x must be pwr 2

// -----------------  LOGGING  -----------------------------------

//#define ENABLE_LOGGING_AT_DEBUG_LEVEL

#define INFO(fmt, args...) \
    do { \
        logmsg("INFO", __func__, fmt, ## args); \
    } while (0)
#define WARN(fmt, args...) \
    do { \
        logmsg("WARN", __func__, fmt, ## args); \
    } while (0)
#define ERROR(fmt, args...) \
    do { \
        logmsg("ERROR", __func__, fmt, ## args); \
    } while (0)

#ifdef ENABLE_LOGGING_AT_DEBUG_LEVEL
    #define DEBUG(fmt, args...) \
        do { \
            logmsg("DEBUG", __func__, fmt, ## args); \
        } while (0)
#else
    #define DEBUG(fmt, args...) 
#endif

#define BLANK_LINE \
    do { \
        INFO("#"); \
    } while (0)

#define FATAL(fmt, args...) \
    do { \
        logmsg("FATAL", __func__, fmt, ## args); \
        exit(1); \
    } while (0)

#define INFO_INTERVAL(us, fmt, args...) \
    do { \
        uint64_t now = microsec_timer(); \
        static uint64_t last; \
        if (now - last > (us)) { \
            INFO(fmt, args); \
            last = now; \
        } \
    } while (0)

#define ERROR_INTERVAL(us, fmt, args...) \
    do { \
        uint64_t now = microsec_timer(); \
        static uint64_t last; \
        if (now - last > (us)) { \
            ERROR(fmt, args); \
            last = now; \
        } \
    } while (0)

#define WARN_INTERVAL(us, fmt, args...) \
    do { \
        uint64_t now = microsec_timer(); \
        static uint64_t last; \
        if (now - last > (us)) { \
            WARN(fmt, args); \
            last = now; \
        } \
    } while (0)

void logmsg(char * lvl, const char * func, char * fmt, ...) __attribute__ ((format (printf, 3, 4)));

// -----------------  TIME  --------------------------------------

#define MAX_TIME_STR 50

uint64_t microsec_timer(void);
uint64_t get_real_time_us(void);
char * time2str(char * str, int64_t us, bool gmt, bool display_ms, bool display_date);

// -----------------  CONFIG READ/WRITE  --------------------------------------------

#define MAX_CONFIG_VALUE_STR 100

typedef struct {
    const char * name;
    char         value[MAX_CONFIG_VALUE_STR];
} config_t;

int config_read(char * config_path, config_t * config, int config_version);
int config_write(char * config_path, config_t * config, int config_version);

// -----------------  NETWORKING  ----------------------------------------

int getsockaddr(char * node, int port, struct sockaddr_in * ret_addr);
char * sock_addr_to_str(char * s, int slen, struct sockaddr * addr);
int do_recv(int sockfd, void * recv_buff, size_t len);
int do_send(int sockfd, void * send_buff, size_t len);

#endif
