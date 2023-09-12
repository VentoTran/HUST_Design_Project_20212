
#include "helperFunc.h"
#include "math.h"
#include "stm32f1xx_hal.h"

//-------------------------------------------------------------------------


void reverse(char* str, int len)
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


void ftoa(float n, char* res, int afterpoint)
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


void ftoa0(float n, char* res, int afterpoint)
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

int random_number(int min_num, int max_num)
{
    int result = 0, low_num = 0, hi_num = 0;

    if (min_num < max_num)
    {
        low_num = min_num;
        hi_num = max_num + 1; // include max_num in output
    } else {
        low_num = max_num + 1; // include max_num in output
        hi_num = min_num;
    }

    srand(HAL_GetTick());
    result = (rand() % (hi_num - low_num)) + low_num;
    return result;
}

//--------------------------------------------------------------------------


