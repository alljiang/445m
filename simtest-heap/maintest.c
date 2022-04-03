#include <stdio.h>
#include "heap.h"

void printstats() {
    struct heap_stats stats;
    int rv = Heap_Stats(&stats);
    printf("%d\n", rv);
    printf("size: %d\n", stats.size);
    printf("used: %d\n", stats.used);
    printf("free: %d\n", stats.free);
}

int main() {
    Heap_Init();

    printstats();

    int *test1 = Heap_Malloc(100);
    int *test2 = Heap_Malloc(100);
    int *test3 = Heap_Malloc(100);
    int *test4 = Heap_Malloc(100);
    printstats();
    Heap_Free(test3);
    printstats();
    Heap_Free(test2);
    printstats();
    Heap_Free(test4);
    printstats();
    Heap_Free(test1);
    printstats();

    printstats();

    return 0;
}