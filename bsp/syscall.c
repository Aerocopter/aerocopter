#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "bsp_uart.h"

#undef errno
extern int errno;

// 环境变量相关
char *__env[1] = { 0 };
char **environ = __env;

// 退出程序
void _exit(int status)
{
    while (1); // 无限循环，或者执行系统复位
}

// 进程相关
int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

// 文件操作
int _close(int file)
{
    return -1;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _open(const char *name, int flags, int mode)
{
    return -1;
}

int _read(int file, char *ptr, int len)
{
    return 0;
}

// 我们已有的 _write 函数（放在同一文件中或外部）
int _write(int file, char *ptr, int len)
{
    // 你的实现，调用 uart_send_char
    // 确保包含 uart_send_char 的声明或实现
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') uart_send_char('\r');
        uart_send_char(ptr[i]);
    }
    return len;
}

// 堆内存管理
caddr_t _sbrk(int incr)
{
    extern char _end; // 由链接脚本定义
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }
    prev_heap_end = heap_end;
    // 检查堆栈碰撞（可选）
    // extern char _stack_top; // 如果链接脚本定义了栈顶
    // if (heap_end + incr > _stack_top) {
    //     errno = ENOMEM;
    //     return (caddr_t)-1;
    // }
    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

// 其他可能需要的函数
int _link(char *old, char *new)
{
    return -1;
}

int _unlink(char *name)
{
    return -1;
}

int _stat(char *file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

// 时间相关
clock_t _times(struct tms *buf)
{
    return -1;
}

int _gettimeofday(struct timeval *tv, struct timezone *tz)
{
    tv->tv_sec = 0;
    tv->tv_usec = 0;
    return 0;
}
