#include "string.h";
#include "string_func.h"

int strtoint(char* str){
    int num = 0;
    for (int i = 0; str[i] != '\0'; i++) {
        num = num * 10 + (str[i] - 48);
    }
    return num;
}

void slice(const char* str, char* result, size_t start, size_t end)
{
	int length = strlen(result);
    strncpy(result, str + start, end - start);
    result[end-start] = '\0'; //we're all stupid
}
