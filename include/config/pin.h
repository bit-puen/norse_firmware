#ifndef CONFIG_PIN_H
#define CONFIG_PIN_H

#if defined(MK_I) || defined(MK_II)
#define PIN_DYNAMIXEL_TX            PA_9
#define PIN_DYNAMIXEL_RX            PA_10
#define PIN_DYNAMIXEL_DIRECTION     PA_8
#define PIN_PROTOCOL_TX             USBTX
#define PIN_PROTOCOL_RX             USBRX
#endif

/*
                            ----------------
                            |3V3        GND|
                            |3V3     GPIO43|x---USB0TX 
                            |RST     GPIO44|x---USB0RX
 PIN_DYNAMIXEL_DIRECTION----|GPIO4    GPIO1|
  PIN_OBSTACLE_DETECTION----|GPIO5    GPIO2|
                        ----|GPIO6   GPIO42|x---MTMS
                        ----|GPIO7   GPIO41|x---MTDI
        PIN_DYNAMIXEL_TX----|GPIO15  GPIO40|x---MTDO
        PIN_DYNAMIXEL_RX----|GPIO16  GPIO39|x---MTCK
                        ----|GPIO17  GPIO38|----BUILDIN_LED
                        ----|GPIO18  GPIO37|
                     SDA----|GPIO8   GPIO36|
      STRAPPING_PIN_JTAG---x|GPIO3   GPIO35|
                     LOG---x|GPIO46   GPIO0|x---BOOT
                     SCL----|GPIO9   GPIO45|x---STRAPPING_PIN_VDD_SPI
                        ----|GPIO10  GPIO48|
                        ----|GPIO11  GPIO47|
                        ----|GPIO12  GPIO21|x---USB_D+
                        ----|GPIO13  GPIO20|x---USB_D-
                        ----|GPIO14  GPIO19|----PECTRAL_SENSOR_GPIO
                            |5V0        GND|
                            |GND        GND|
                            ----------------
*/
#ifdef MK_III
#define PIN_DYNAMIXEL_DIRECTION     GPIO_NUM_4
#define PIN_OBSTACLE_DETECTION      GPIO_NUM_5
#endif

#endif