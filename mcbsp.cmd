SECTIONS
{
    .init_array:     load >> L1DSRAM
    .mcbsp:          load >> DSP_PROG
    .mcbspSharedMem: load >> DSP_PROG
//     platform_lib:   load >> DDR
    systemHeap:     load >> IRAM
}

