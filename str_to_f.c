#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define NOT_A_NUM (__builtin_nanf (""))


int tokenize_string(char str[], char delimiters[], double tokens[][3])
{

    char *p = str;
    int i;
    for (i = 0; p /*if ptr not a null*/; i++)
    {

        p = strtok(i == 0 ? str : NULL, delimiters); //only the first call pass str
        tokens[i/3][i%3] = (p) ? atof(p) : NOT_A_NUM;
        
    }

    return i/3;

}


int main()
{
    int row;
    char str[] = "[[1.1111, 2.2222, 3.3333]\n\
                   [3.3333, 4.4444, 5.5555]\n\
                   [6.6666, 7.7777, 8.8888]\n\
                   [9.9999, 11.111, 12.121]]";

    double tokens[100][3];
    char delimiter[] = " ,[]\n";
    row =  tokenize_string(str,delimiter,tokens);

    printf("The array has %d rows\n",row);

    return 0;

}
