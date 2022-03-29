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

    int *p = Heap_Calloc(sizeof(int));
    *p = 42;
    Heap_Free(p);

    printstats();

    return 0;
}