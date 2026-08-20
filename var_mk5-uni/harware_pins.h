
#if esp32c6_brd
#define SDA_PIN 		20
#define SCL_PIN 		19
#define BUZZER_PIN 		18
#define LED_PIN 		15    
#define MODE_BUTTON_PIN 9
#endif

#if esp32s3_brd
#define SDA_PIN 		8
#define SCL_PIN 		9
#define BUZZER_PIN 		10
#define LED_PIN 		15    
#define MODE_BUTTON_PIN 0
#endif

#if nrf52802_E73brd
//#define SDA_PIN 99
//#define SCL_PIN 99
#define BUZZER_PIN    	04
#define MODE_BUTTON_PIN 14
#define LED_PIN     	17
#endif

#if nrf51802_sens
//#define SDA_PIN 99
//#define SCL_PIN 99
#define BUZZER_PIN  	04
#define MODE_BUTTON_PIN 18 
#define LED_PIN_R     	17 
#define LED_PIN_R     	19 
#define LED_PIN     	LED_PIN_R 
#endif

#if nrf52822_R40sens
#define SDA_PIN 11
#define SCL_PIN 12
#define BUZZER_PIN		04 
#define MODE_BUTTON_PIN 18 
#define LED_PIN_R     	17 
#define LED_PIN_B     	19 
#define LED_PIN     	LED_PIN_R 
#endif

#if nrf52840_brd 
#define BUZZER_PIN 		6 
#define MODE_BUTTON_PIN 2 
#define LED_PIN 		13 
#endif

// just template for copypaste. dont uncomment HERE
//#define esp32c6_brd true
//#define esp32s3_brd true
//#define nrf52802_E73brd true
//#define nrf51802_sens true
//#define nrf52822_R40sens true
//#define nrf52840_brd true 
