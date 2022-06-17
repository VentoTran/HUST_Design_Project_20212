
#ifndef _HELPER_FUNC_H_
#define _HELPER_FUNC_H_

/**
 * @brief 
 * Reverses a string.
 * @param str str[] to reverse
 * @param len str[] length
 */
void reverse(char* str, int len);

/**
 * @brief 
 * Converts a given integer x to string str[]. 
 * If d is more than the number of digits in x, then 0s are added at the beginning.
 * @param x a given integer.
 * @param str str[] to write into.
 * @param d the number of digits required in the output.
 * @return <int> is the length of the new str[] created.
 */
int intToStr(int x, char str[], int d);

/**
 * @brief 
 * Converts a floating-point/double number to a string.
 * @param n float input.
 * @param res str[] output.
 * @param afterpoint number of digit after the point.
 */
void ftoa(float n, char* res, int afterpoint);



#endif