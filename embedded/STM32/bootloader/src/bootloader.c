#include <stdint.h>

//ROM begins at 0x08000000
#define BOOTLOADER_SIZE     0x8000
#define MAIN_START_ADDR     0x08000000 + BOOTLOADER_SIZE

static void jump_to_main() 
{
    //define function pointer
    typedef void (*void_func)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_START_ADDR + 4U);
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);

    void_func jump_func = (void_func)reset_vector;

    //jump to main in app
    jump_func();

}

int main()
{

    jump_to_main();

    //function will never return
    return 0;
}

