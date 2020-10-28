set(COMPONENT_SRCS 
        "mpu60x0_driver/src/mpu60x0.c" 
        "mpu60x0_driver/src/mpu60x0_i2c.c"
    )

set(COMPONENT_ADD_INCLUDEDIRS "mpu60x0_driver/include")

register_component()

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)

# if(${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU6000")
#     component_compile_options(PUBLIC -DCONFIG_MPU_6000)
# elseif(${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU6050")
#     component_compile_options(PUBLIC -DCONFIG_MPU_6050)
# endif()