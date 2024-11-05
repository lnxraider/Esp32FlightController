px4_add_module(
    MODULE drivers__uart__sc16is752
    MAIN sc16is752
    COMPILE_FLAGS
        ${MAX_CUSTOM_OPT_LEVEL}
    SRCS
        sc16is752.cpp
    DEPENDS
        px4_work_queue
)

