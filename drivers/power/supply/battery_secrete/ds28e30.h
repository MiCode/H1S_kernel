#ifndef __DS28E30_H__
#define __DS28E30_H__
#define ERROR_NO_DEVICE					-1
#define ERROR_R_STATUS					-2
#define ERROR_R_ROMID					-3
#define ERROR_R_PAGEDATA				-4
#define ERROR_COMPUTE_MAC				-5
#define ERROR_S_SECRET                  -6
#define ERROR_UNMATCH_MAC				-7
#define DS_TRUE						    1
#define DS_FALSE					   0
#define CMD_RELEASE_BYTE				0xAA
// 1-wire ROM commands
#define CMD_SEARCH_ROM					0xF0
#define CMD_READ_ROM					0x33
#define CMD_MATCH_ROM					0x55
#define CMD_SKIP_ROM					0xCC
#define CMD_RESUME_ROM					0xA5
// DS28E30 device commands
#define CMD_START					    0x66
#define CMD_WRITE_MEM					0x96
#define CMD_READ_MEM					0x44
#define CMD_READ_STATUS					0xAA
#define CMD_SET_PAGE_PROT				0xC3
#define CMD_COMP_READ_AUTH				0xA5
#define CMD_DECREMENT_CNT				0xC9
#define CMD_DISABLE_DEVICE				0x33
#define CMD_READ_DEVICE_PUBLIC_KEY      0xCB
#define CMD_AUTHENTICATE_PUBLIC_KEY     0x59
#define CMD_AUTHENTICATE_WRITE          0x89
// Result bytes
#define RESULT_SUCCESS					    0xAA
#define RESULT_FAIL_DEVICEDISABLED			0x88
#define RESULT_FAIL_PARAMETETER				0x77
#define RESULT_FAIL_PROTECTION				0x55
#define RESULT_FAIL_INVALIDSEQUENCE 		0x33
#define RESULT_FAIL_ECDSA					0x22
#define RESULT_FAIL_VERIFY					0x00
#define RESULT_FAIL_NONE				    0xFF
// Pages
#define PG_USER_EEPROM_0  			0
#define PG_USER_EEPROM_1  			1
#define PG_USER_EEPROM_2  			2
#define PG_USER_EEPROM_3  			3
#define PG_CERTIFICATE_R    		4
#define PG_CERTIFICATE_S    		5
#define PG_AUTHORITY_PUB_KEY_X    	6
#define PG_AUTHORITY_PUB_KEY_Y    	7
#define PG_DS28E30_PUB_KEY_X    	28
#define PG_DS28E30_PUB_KEY_Y    	29
#define PG_DS28E30_PRIVATE_KEY    	36
#define PG_DEC_COUNTER  			106
// Delays
#define DELAY_DS28E30_EE_WRITE_TWM     	100	//50       //maximal 10ms
#define DELAY_DS28E30_EE_READ_TRM      	100	//50          //30       //maximal 5ms
#define DELAY_DS28E30_ECDSA_GEN_TGES 	200	//maximal 130ms (tGFS)
#define DELAY_DS28E30_Verify_ECDSA_Signature_tEVS   200	//maximal 130ms (tGFS)
#define DELAY_DS28E30_ECDSA_WRITE 		350	//for ECDSA write EEPROM
// Protection bit fields
#define PROT_RP						0x01
#define PROT_WP						0x02
#define PROT_EM                  	0x04	// EPROM Emulation Mode
#define PROT_DC						0x08
#define PROT_AUTH                 	0x20	// AUTH mode for authority public key X&Y
#define PROT_ECH                  	0x40	// Encrypted read and write using shared key from ECDH
#define PROT_ECW                 	0x80	// Authentication Write Protection ECDSA (not applicable to KEY_PAGES)
// Generate key flags
#define ECDSA_KEY_LOCK           	0x80
#define ECDSA_USE_PUF            	0x01
// page number
#define PAGE0						0x00
#define PAGE1						0x01
#define DC_PAGE						0x6A
#define SECRET_PAGE					0x03
#define MAX_PAGENUM					0x6B
#define ANONYMOUS					1
//retry times config
#define VERIFY_SIGNATURE_RETRY		5
#define VERIFY_CERTIFICATE_RETRY	5
#define READ_STATUS_RETRY			5
/* N19A code for HQ-360184 by p-wumingzhu1 at 20240103 */
#define READ_ROMNO_RETRY			2
#define READ_PAGEDATA0_RETRY		5
#define READ_DEC_COUNTER_RETRY		5
#define MINUS_COUNTER_RETRY			5
#define WRITE_PG1_RETRY				5
/* N19A code for HQ-360184 by p-wumingzhu1 at 20240103 end */
#define GET_ROM_ID_RETRY			2
#define GET_USER_MEMORY_RETRY		20
#define GET_BLOCK_STATUS_RETRY				8
#define SET_BLOCK_STATUS_RETRY				8
// xiaomi's battery identity
#define FAMILY_CODE					0xDB	//0xDB for custom DS28E30? 0x5B? 0x9f?[llt-----------------------------]不同型号电池
#define MI_CID_LSB                  0xF0
#define MI_CID_MSB                  0x04
#define MI_MAN_ID_LSB               0xD4
#define MI_MAN_ID_MSB               0x00
#define DC_INIT_VALUE				0x1FFFF
#define AUTHENTIC_COUNT_MAX 		5
unsigned char last_result_byte;
// Common Functions

#endif				/* __DS28E30_H__ */
