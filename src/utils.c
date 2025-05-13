#include <utils.h>

void lstdir(char* path) {
    DIR *dp;
    struct dirent *ep;     
    dp = opendir (path);
    if (dp != NULL)
    {
        while ((ep = readdir (dp)) != NULL)
        puts (ep->d_name);
            
        (void) closedir (dp);
        return;
    }
    else
    {
        perror ("Couldn't open the directory");
        return;
    }
}


void print_memory_info() {
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    printf("\n[DRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_EXEC);
    printf("[IRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_DMA);
    printf("[DMA-capable] Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);
}