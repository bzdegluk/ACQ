

#define SDK_SYSTEM_CONFIG()           { CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1); \
                                        SdkEvalLedInit(LED1); \
                                        SdkEvalLedInit(LED2); \
                                        SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT); \
                                        SpiritSpiInit(); \
                                      }
