#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "jcp_handler.h"

#define MSGLENMAX (4060-MSGHDRSZ)

#define TRUE 1
#define FALSE 0

static int readInt16(char *s) {
  int n = ((unsigned char)s[0])<<8;
  n |= (unsigned char)s[1];
  return n;
}

static void writeInt16(char *s, int n) {
  n &= 0xffff;
  s[0] = (char)((n & 0xff00)>>8);
  s[1] = (char)(n & 0xff);
}

static int readInt32(char *s) {
  int n = readInt16(s) << 16;
  n |= readInt16(s+2);
  return n;
}

static void writeInt32(char *s, int n) {
  writeInt16(s, (n & 0xffff0000) >> 16);
  writeInt16(s+2, n & 0xffff);
}

static char *decode_message(char *message, int *length, int *abstract) {
  *length = readInt16(message);
  message += 2;
  *abstract = readInt32(message);
  message += 4;
  return message;
}

int get_message_length(char *message) {
  return readInt16(message);
}

static char *read_arg(char *message, int *length) {
  char c = '\0';
  while((*length)-- > 0) {
    c = *message++;
    if(c == '\0') {
      break;
    }
  }
  if(c != '\0') {
    *--message = '\0';
  }
  return message;
}

#if(0)
static void push_string(char *message, char *arg) {
  int len = readInt16(message);
  char *content = message + MSGHDRSZ + len;
  strcpy(content, arg);
  len += 1+strlen(arg);
  writeInt16(message, len);
}
#endif

#define MAXFILES 64

static FILE *files[MAXFILES] = { NULL };
static int init_files = TRUE;

#define SKUNK_LOG_ACTIONS 1

#if(SKUNK_LOG_ACTIONS)
static FILE *logfile = NULL;

#ifdef _MSC_VER
#define LOG(...)
#else
#define LOG(args...) ({fprintf(logfile, ##args); fflush(logfile);})
#endif
#else
#define LOG(...)
#endif

void serve_request(char *request, char *reply) {

#if(SKUNK_LOG_ACTIONS)
  if(logfile == NULL) {
    logfile = fopen("/tmp/jcp.log","w");
    assert(logfile != NULL);
  }
#endif

  if(init_files) {
    int i = 0;
    files[i++] = stdin;
    files[i++] = stderr;
    for(; i < MAXFILES; i++) {
      files[i]= NULL;
    }
    init_files = FALSE;
  }
  

  int length;
  int abstract;
  char *content = decode_message(request, &length, &abstract);
  switch(abstract) {
  case SKUNK_WRITE_STDERR: {
    LOG("Skunk Write stderr Request\n");
    fwrite(content, 1, length, stderr);
    fflush(stderr);
    break;
  }
  case SKUNK_READ_STDIN: {
    LOG("Skunk Read stdin Request\n");
    fprintf(stdout,">");
    if (!fgets(reply+MSGHDRSZ,MSGLENMAX,stdin)) {
      LOG("Skunk Read stdin Request failed\n");
      reply[MSGHDRSZ] = '\0';
    }
    int len = 1+strlen(reply+MSGHDRSZ); // count the \0
    writeInt16(reply, len);
    break;
  }
  case SKUNK_FOPEN: {
    LOG("Skunk fopen Request\n");
    writeInt32(reply+2,-1);  // error 
    writeInt16(reply,0); // size of reply message
    if(length <= 0) {
      break;
    }
    char *filename = content;
    content = read_arg(filename, &length);
    if(length <= 0) {
      break;
    }
    char *mode = content;
    content = read_arg(mode, &length);
    int fd;
    for(fd = 2; fd < MAXFILES; fd++) {
      if(files[fd] == NULL) {
	break;
      }
    }
    
    if(fd < MAXFILES) {
      LOG("%d = fopen(\"%s\", \"%s\");\n", fd, filename, mode);
      FILE *f = fopen(filename, mode);
      if(f != NULL) {
	LOG("OK\n");
	files[fd] = f;
	writeInt16(reply,0); // no reply content
	writeInt32(reply+2,fd);  // return file descriptor
      }
    } 
    break;
  }
  case SKUNK_FCLOSE: {
    LOG("Skunk fclose Request\n");
    assert(length == 2);
    writeInt32(reply+2,-1);  // error 
    writeInt16(reply,0); // size of reply message
    int fd = readInt16(content);
    content += 2;
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL)) {
      LOG("fclose(%d);\n", fd);
      int res = fclose(files[fd]);
      files[fd] = NULL;
      if(res != EOF) {
	LOG("OK\n");
	writeInt16(reply,0); // no reply content
	writeInt32(reply+2,res);
      }
    }
    break;
  }
  case SKUNK_FREAD: {
    LOG("Skunk fread request\n");
    assert(length == 10);
    size_t size = readInt32(content);
    content += 4;
    size_t nmemb = readInt32(content);
    content += 4;
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, 0); // no read
    writeInt16(reply, 0); // size of reply message
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL) && (size * nmemb <= MSGLENMAX)) {
      size_t nb = fread(reply+MSGHDRSZ, size, nmemb, files[fd]);
      LOG("%zd = fread(%zd, %zd, %d)\n", nb, size, nmemb, fd);
      writeInt16(reply, size * nb); // must fit on 16 bits
      writeInt32(reply+2, nb); // result
    }
    break;
  }
  case SKUNK_FWRITE: {
    LOG("Skunk fwrite request\n");
    assert(length >= 10);
    size_t size = readInt32(content);
    content += 4;
    size_t nmemb = readInt32(content);
    content += 4;
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, 0); // no read
    writeInt16(reply, 0); // size of reply message
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL) && (size * nmemb <= MSGLENMAX-10)) {
      size_t nb = fwrite(content, size, nmemb, files[fd]);
      LOG("%zd = fwrite(%zd, %zd, %d)\n", nb, size, nmemb, fd);
      writeInt16(reply, 0); // no reply content
      writeInt32(reply+2, nb); // result
    }
    break;
  }
  case SKUNK_FPUTC: {
    LOG("Skunk fputc request\n");
    assert(length == 4);
    int c = readInt16(content);
    content += 2;
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, -1); // no read
    writeInt16(reply, 0); // size of reply message
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL)) {
      int res = fputc(c, files[fd]);
      LOG("%d = fputc(%d, %d)\n", res, c, fd);
      writeInt16(reply, 0); // no reply content
      writeInt32(reply+2, res); // result
    }
    break;
  }
  case SKUNK_FEOF: {
    LOG("Skunk feof request\n");
    assert(length == 2);
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, 0); // eof 
    writeInt16(reply, 0);
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL)) {
      int res = feof(files[fd]);
      LOG("%d = feof(%d)\n", res, fd);
      writeInt16(reply, 0); // no reply content
      writeInt32(reply+2, res); // result
    }  
    break;
  }
  case SKUNK_FFLUSH: {
    LOG("Skunk fflush request\n");
    assert(length == 2);
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, -1); // eof 
    writeInt16(reply, 0);
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL)) {
      int res = fflush(files[fd]);
      LOG("%d = fflush(%d)\n", res, fd);
      writeInt16(reply, 0); // no reply content
      writeInt32(reply+2, res); // result
    }  
    break;
  }
  case SKUNK_FGETS: {
    LOG("Skunk fgets request\n");
    assert(length == 6);
    int size = readInt32(content);
    content += 4;
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, -1); // error
    writeInt16(reply, 0);
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL) && (size <= MSGLENMAX)) {
      char *res = fgets(reply+MSGHDRSZ, size, files[fd]);
      if(res != NULL) {
	LOG("fgets(%d, %d)\n", size, fd);
	int n = 1+strlen(reply+MSGHDRSZ);
	writeInt16(reply, n); // reply content length
	writeInt32(reply+2, 0); // result
      }
    }  
    break;
  }
  case SKUNK_FGETC: {
    LOG("Skunk fgetc request\n");
    assert(length == 2);
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, -1); // error
    writeInt16(reply, 0);
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL)) {
      int res = fgetc(files[fd]);
      LOG("%d = fgetc(%d)\n", res, fd);
      writeInt16(reply, 0); // no reply content
      writeInt32(reply+2,res); // result
    }
    break;
  }
  case SKUNK_FSEEK: {
    LOG("Skunk fseek request\n");
    assert(length == 8);
    long offset = readInt32(content);
    content += 4;
    int whence = readInt16(content);
    content += 2;
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, -1); // error
    writeInt16(reply, 0);
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL)) {
      int res = fseek(files[fd], offset, whence);
      LOG("%d = fseek(%d, %ld, %d)\n", res, fd, offset, whence);
      writeInt16(reply, 0); // no reply content
      writeInt32(reply+2,res); // result
    }
    break;
  }
  case SKUNK_FTELL: {
    LOG("Skunk ftell request\n");
    assert(length == 2);
    int fd = readInt16(content);
    content += 2;
    writeInt32(reply+2, -1); // error
    writeInt16(reply, 0);
    if((0 <= fd) && (fd < MAXFILES) && (files[fd] != NULL)) {
      long res = ftell(files[fd]);
      LOG("%ld = ftell(%d)\n", res, fd);
      writeInt16(reply, 0); // no reply content
      writeInt32(reply+2,res); // result
    }
    break;
  }
  default: {
    if(reply != NULL) {
      writeInt16(reply, 0);
    }
    break;
  }
  }
}
