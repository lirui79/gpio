#ifndef LOGSAMPLE_COMLOG_H_
#define LOGSAMPLE_COMLOG_H_


#define LOGF_START()   printf("start");
#define LOGF_END()     printf("end");
#define LOGF_TAG()     printf("tag");
#define LOGF_DBG(...)  printf(__VA_ARGS__);
#define LOGF_INF(...)  printf(__VA_ARGS__);
#define LOGF_WAR(...)  printf(__VA_ARGS__);
#define LOGF_ERR(...)  printf(__VA_ARGS__);

#endif
