#ifndef SAMPLE_COMMON_UTILS_HPP_
#define SAMPLE_COMMON_UTILS_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "../../include/Mv3dRgbdApi.h"
#include "../../include/Mv3dRgbdDefine.h"
#include "../../include/Mv3dRgbdImgProc.h"

#ifndef ASSERT
#define ASSERT(x)   do{ \
                if(!(x)) { \
                    LOGE("Assert failed at %s:%d", __FILE__, __LINE__); \
                    LOGE("    : " #x ); \
                    abort(); \
                } \
            }while(0)
#endif

#ifndef ASSERT_OK
#define ASSERT_OK(x)    do{ \
                int err = (x); \
				if(err != MV3D_RGBD_OK) { \
				LOGE("Assert failed: error %#x at %s:%d", err, __FILE__, __LINE__); \
                    LOGE("Source Code: " #x ); \
                    abort(); \
                } \
            }while(0)
#endif


#ifdef _WIN32
# include <windows.h>
# include <time.h>
  static inline char* getLocalTime()
  {
      static char local[26] = {0};
      SYSTEMTIME wtm;
      struct tm tm;
      GetLocalTime(&wtm);
      tm.tm_year     = wtm.wYear - 1900;
      tm.tm_mon     = wtm.wMonth - 1;
      tm.tm_mday     = wtm.wDay;
      tm.tm_hour     = wtm.wHour;
      tm.tm_min     = wtm.wMinute;
      tm.tm_sec     = wtm.wSecond;
      tm.tm_isdst    = -1;
  
      strftime(local, 26, "%Y-%m-%d %H:%M:%S", &tm);

      return local;
  }

#else
# include <sys/time.h>
# include <unistd.h>
  static inline char* getLocalTime()
  {
      static char local[26] = {0};
      time_t time;

      struct timeval tv;
      gettimeofday(&tv, NULL);

      time = tv.tv_sec;
      struct tm* p_time = localtime(&time); 
      strftime(local, 26, "%Y-%m-%d %H:%M:%S", p_time);  

      return local; 
  }
#endif


#if defined(_WIN32) || defined(WIN32) || defined(__WIN32__)
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
int _kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	/* set the nonblock */
	fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0));
	if (ch != EOF)
	{
		ungetc(ch, stdin); /* back the ch to stdin */
		return 1;
	}
	return 0;
}
#endif


#if defined(_WIN32) || defined(WIN32) || defined(__WIN32__)
#else
char _getch(void)
{
	struct termios tmtemp,tm;
	char c;
	int fd=0;
	if(tcgetattr(fd,&tm) != 0)
	{      /*获取当前的终端属性设置，并保存到tm结构体中*/
		return -1;
	}

	tmtemp=tm;
	cfmakeraw(&tmtemp);     /*将tetemp初始化为终端原始模式的属性设置*/
	if(tcsetattr(fd,TCSANOW,&tmtemp) != 0)
	{     /*将终端设置为原始模式的设置*/
		return -1;
	}
	c=getchar();
	if(tcsetattr(fd,TCSANOW,&tm) != 0)
	{      /*接收字符完毕后将终端设置回原来的属性*/
		return 0;
	}
	return c;
}
#endif


void msleep(int ms)
{
#if defined(_WIN32) || defined(WIN32) || defined(__WIN32__)
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}

#define LOG(fmt,...)  printf(fmt "\n", ##__VA_ARGS__)
#define LOGD(fmt,...)  printf("(%s) " fmt "\n", getLocalTime(), ##__VA_ARGS__)
#define LOGE(fmt,...)  printf("(%s) Error: " fmt "\n", getLocalTime(), ##__VA_ARGS__)


#endif
