#include <stdio.h>
#include <stdlib.h>
 
int main(int argc, char const *argv[])
{
    srand((unsigned) time(NULL));
    for (int i = 0; i < 10; ++i)
    {
        printf("%d\n", rand()%10);
    }

}
