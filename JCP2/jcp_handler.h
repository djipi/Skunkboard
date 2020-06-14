#ifndef __JCP_HANDLER_H
#define __JCP_HANDLER_H

#define SKUNK_WRITE_STDERR 1
#define SKUNK_READ_STDIN 2

#define SKUNK_FOPEN 3
#define SKUNK_FCLOSE 4
#define SKUNK_FREAD 5
#define SKUNK_FWRITE 6
#define SKUNK_FPUTC 7
#define SKUNK_FEOF 8
#define SKUNK_FFLUSH 9
#define SKUNK_FGETS 10
#define SKUNK_FGETC 11
#define SKUNK_FSEEK 12
#define SKUNK_FTELL 13

#define MSGHDRSZ 6

int get_message_length(char *message);

void serve_request(char *request, char *reply);

#endif
