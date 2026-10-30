#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/**
 * @brief TI Function Code
 */
enum TI_FUNCTION_CODE
{
    I2C_FN_CODE = 0,
    SPI_FN_CODE,
    UART_FN_CODE,
    GPIO_FN_CODE,
    INT_FN_CODE,
    PWM_FN_CODE,
    CANFD_FN_CODE,
    DDR_MEM_FN_CODE,
    EMMC_MEM_FN_CODE,
    OSPI_FLASH_FN_CODE,
};


/**
 * @brief TI I2C Command Code
 */
enum TI_I2C_COMMAND_CODE
{
    I2C_CONFIG_CMD_CODE = 0,
    I2C_REG_WRITE_CMD_CODE,
    I2C_REG_READ_CMD_CODE,
    I2C_MULTIREG_WRITE_CMD_CODE,
    I2C_MULTIREG_READ_CMD_CODE,
};

/**
 * @brief TI SPI Command Code
 */
enum TI_SPI_COMMAND_CODE
{
    SPI_CONFIG_CMD_CODE = 0,
    SPI_READWRITE_CMD_CODE,
};

/**
 * @brief TI UART Command Code
 */
enum TI_UART_COMMAND_CODE
{
    UART_CONFIG_CMD_CODE = 0,
    UART_WRITE_CMD_CODE,
    UART_READ_CMD_CODE,
};

/**
 * @brief TI GPIO Command Code
 */
enum TI_GPIO_COMMAND_CODE
{
    GPIO_CONFIG_CMD_CODE = 0,
    GPIO_WRITE_STATE_CMD_CODE,
    GPIO_READ_STATE_CMD_CODE,
    GPIO_SET_PORT_CMD_CODE,
    GPIO_WRITE_PORT_CMD_CODE,
    GPIO_READ_PORT_CMD_CODE,
    GPIO_WRITE_PULSE_CMD_CODE
};

/**
 * @brief TI INT Command Code
 */
enum TI_INT_COMMAND_CODE
{
    INT_CONFIG_CMD_CODE = 0,
    INT_CHECK_RECEIVED_CMD_CODE,
    INT_SET_HANDLER_CMD_CODE
};

/**
 * @brief TI PWM Command Code
 */
enum TI_PWM_COMMAND_CODE
{
    PWM_CONFIG_CMD_CODE = 0,
};

/**
 * @brief TI CANFD Command Code
 */
enum TI_CANFD_COMMAND_CODE
{
    CANFD_GET_DEFAULT_PARAMS_CMD_CODE = 0,
    CANFD_CONFIG_CMD_CODE,
    CANFD_TX_OBJECT_CREATE_CMD_CODE,
    CANFD_WRITE_DATA_CMD_CODE,
    CANFD_RX_OBJECT_CREATE_CMD_CODE,
    CANFD_READ_DATA_CMD_CODE,
    CANFD_WRITE_CANCEL_CMD_CODE,
    CANFD_VALID_DATA_CMD_CODE,
    CANFD_GET_FAILURE_REASON_CMD_CODE,
    CANFD_CLOSE_CMD_CODE,
    CANFD_DLT_TX_OBJECT_CMD_CODE,
    CANFD_DLT_RX_OBJECT_CMD_CODE
};

/**
 * @brief TI DDR Memory Command Code
 */
enum TI_DDR_MEM_COMMAND_CODE
{
    DDR_MEM_TEST_CMD_CODE = 0,
    DDR_MEM_READWRITE_CMD_CODE,
};

/**
 * @brief TI EMMC Memory Command Code
 */
enum TI_EMMC_MEM_COMMAND_CODE
{
    EMMC_MEM_READWRITE_CMD_CODE = 0,
};

/**
 * @brief TI OSPI FLASH Command Code
 */
enum TI_OSPI_FLASH_COMMAND_CODE
{
    OSPI_FLASH_READWRITE_CMD_CODE = 0,
};

/*-------------------------------------- DATA ---------------------------------*/


/*--------------------------- UART CMD ERROR CODE --------------------------------- */
/**
 * @brief UART CMD Error Code
 */
#define SIGNATURE_ERROR                                   0x10