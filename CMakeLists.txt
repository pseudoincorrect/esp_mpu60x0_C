set(COMPONENT_SRCS "mpu60x0_driver/src/mpu60x0.c")
set(COMPONENT_ADD_INCLUDEDIRS "mpu60x0_driver/include")

register_component()

# if(${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU6000")
#     component_compile_options(PUBLIC -DCONFIG_MPU6000)
# elseif(${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU6050")
#     component_compile_options(PUBLIC -DCONFIG_MPU6050)
# endif()