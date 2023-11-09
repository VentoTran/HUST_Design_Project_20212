
#include "helperFunc.h"
#include "math.h"
#include "stm32f1xx_hal.h"

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>

/* Includes */
#include <errno.h>
#include <stdint.h>

/**
 * Pointer to the current high watermark of the heap usage
 */
static uint8_t *__sbrk_heap_end = NULL;

/**
 * @brief _sbrk() allocates memory to the newlib heap and is used by malloc
 *        and others from the C library
 *
 * @verbatim
 * ############################################################################
 * #  .data  #  .bss  #       newlib heap       #          MSP stack          #
 * #         #        #                         # Reserved by _Min_Stack_Size #
 * ############################################################################
 * ^-- RAM start      ^-- _end                             _estack, RAM end --^
 * @endverbatim
 *
 * This implementation starts allocating at the '_end' linker symbol
 * The '_Min_Stack_Size' linker symbol reserves a memory for the MSP stack
 * The implementation considers '_estack' linker symbol to be RAM end
 * NOTE: If the MSP stack, at any point during execution, grows larger than the
 * reserved size, please increase the '_Min_Stack_Size'.
 *
 * @param incr Memory size
 * @return Pointer to allocated memory
 */
void *_sbrk(ptrdiff_t incr)
{
    extern uint8_t _end;             /* Symbol defined in the linker script */
    extern uint8_t _estack;          /* Symbol defined in the linker script */
    extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    const uint8_t *max_heap = (uint8_t *)stack_limit;
    uint8_t *prev_heap_end;

    /* Initialize heap end at first call */
    if (NULL == __sbrk_heap_end)
    {
        __sbrk_heap_end = &_end;
    }

    /* Protect heap from growing into the reserved MSP stack */
    if (__sbrk_heap_end + incr > max_heap)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    prev_heap_end = __sbrk_heap_end;
    __sbrk_heap_end += incr;

    return (void *)prev_heap_end;
}

/* Variables */
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

char *__env[1] = {0};
char **environ = __env;

/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    _kill(status, -1);
    while (1)
    {
    } /* Make sure we hang here */
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        *ptr++ = __io_getchar();
    }

    return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        __io_putchar(*ptr++);
    }
    return len;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _open(char *path, int flags, ...)
{
    (void)path;
    (void)flags;
    /* Pretend like we always fail */
    return -1;
}

int _wait(int *status)
{
    (void)status;
    errno = ECHILD;
    return -1;
}

int _unlink(char *name)
{
    (void)name;
    errno = ENOENT;
    return -1;
}

int _times(struct tms *buf)
{
    (void)buf;
    return -1;
}

int _stat(char *file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(char *old, char *new)
{
    (void)old;
    (void)new;
    errno = EMLINK;
    return -1;
}

int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

int _execve(char *name, char **argv, char **env)
{
    (void)name;
    (void)argv;
    (void)env;
    errno = ENOMEM;
    return -1;
}

//-------------------------------------------------------------------------

void reverse(char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    if (x == 0)
    {
        str[i++] = '0';
    }
    else
    {
        while (x)
        {
            str[i++] = (x % 10) + '0';
            x = x / 10;
        }
    }

    // If number of digits required is more, then
    // add space at the beginning
    while (i < d)
        str[i++] = ' ';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
    // Extract floating part
    float fpart = n - (float)ipart;
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.'; // add dot
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

int intToStr0(int x, char str[], int d)
{
    int i = 0;
    if (x == 0)
    {
        str[i++] = '0';
    }
    else
    {
        while (x)
        {
            str[i++] = (x % 10) + '0';
            x = x / 10;
        }
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa0(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
    // Extract floating part
    float fpart = n - (float)ipart;
    // convert integer part to string
    int i = intToStr0(ipart, res, 0);
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.'; // add dot
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

// int random_number(int min_num, int max_num)
//{
//     int result = 0, low_num = 0, hi_num = 0;
//
//     if (min_num < max_num)
//     {
//         low_num = min_num;
//         hi_num = max_num + 1; // include max_num in output
//     } else {
//         low_num = max_num + 1; // include max_num in output
//         hi_num = min_num;
//     }
//
//     srand(HAL_GetTick());
//     result = (rand() % (hi_num - low_num)) + low_num;
//     return result;
// }

//--------------------------------------------------------------------------
