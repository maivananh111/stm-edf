/*
 * lan87xx_macros.h
 *
 *  Created on: Nov 1, 2023
 *      Author: anh
 */

#ifndef DRVPERDEV_LAN87XX_MACROS_H_
#define DRVPERDEV_LAN87XX_MACROS_H_





#define LAN87XX_BCR     					 ((uint16_t)0x0000U)
#define LAN87XX_BSR     					 ((uint16_t)0x0001U)
#define LAN87XX_PHYI1R   					 ((uint16_t)0x0002U)
#define LAN87XX_PHYI2R  					 ((uint16_t)0x0003U)
#define LAN87XX_ANAR    					 ((uint16_t)0x0004U)
#define LAN87XX_ANLPAR  					 ((uint16_t)0x0005U)
#define LAN87XX_ANER    					 ((uint16_t)0x0006U)
#define LAN87XX_ANNPTR  					 ((uint16_t)0x0007U)
#define LAN87XX_ANNPRR  					 ((uint16_t)0x0008U)
#define LAN87XX_MMDACR  					 ((uint16_t)0x000DU)
#define LAN87XX_MMDAADR 					 ((uint16_t)0x000EU)
#define LAN87XX_ENCTR   					 ((uint16_t)0x0010U)
#define LAN87XX_MCSR    					 ((uint16_t)0x0011U)
#define LAN87XX_SMR     					 ((uint16_t)0x0012U)
#define LAN87XX_TPDCR   					 ((uint16_t)0x0018U)
#define LAN87XX_TCSR    					 ((uint16_t)0x0019U)
#define LAN87XX_SECR    					 ((uint16_t)0x001AU)
#define LAN87XX_SCSIR   					 ((uint16_t)0x001BU)
#define LAN87XX_CLR     					 ((uint16_t)0x001CU)
#define LAN87XX_ISFR    					 ((uint16_t)0x001DU)
#define LAN87XX_IMR     					 ((uint16_t)0x001EU)
#define LAN87XX_PHYSCSR 					 ((uint16_t)0x001FU)


#define LAN87XX_BCR_SOFT_RESET        		 ((uint16_t)0x8000U)
#define LAN87XX_BCR_LOOPBACK          		 ((uint16_t)0x4000U)
#define LAN87XX_BCR_SPEED_SELECT      		 ((uint16_t)0x2000U)
#define LAN87XX_BCR_AUTONEGO_EN       		 ((uint16_t)0x1000U)
#define LAN87XX_BCR_POWER_DOWN        		 ((uint16_t)0x0800U)
#define LAN87XX_BCR_ISOLATE           		 ((uint16_t)0x0400U)
#define LAN87XX_BCR_RESTART_AUTONEGO  		 ((uint16_t)0x0200U)
#define LAN87XX_BCR_DUPLEX_MODE       		 ((uint16_t)0x0100U)

#define LAN87XX_BSR_100BASE_T4      		 ((uint16_t)0x8000U)
#define LAN87XX_BSR_100BASE_TX_FD   		 ((uint16_t)0x4000U)
#define LAN87XX_BSR_100BASE_TX_HD   		 ((uint16_t)0x2000U)
#define LAN87XX_BSR_10BASE_T_FD     		 ((uint16_t)0x1000U)
#define LAN87XX_BSR_10BASE_T_HD     		 ((uint16_t)0x0800U)
#define LAN87XX_BSR_100BASE_T2_FD   		 ((uint16_t)0x0400U)
#define LAN87XX_BSR_100BASE_T2_HD   		 ((uint16_t)0x0200U)
#define LAN87XX_BSR_EXTENDED_STATUS  		 ((uint16_t)0x0100U)
#define LAN87XX_BSR_AUTONEGO_CPLT    		 ((uint16_t)0x0020U)
#define LAN87XX_BSR_REMOTE_FAULT     		 ((uint16_t)0x0010U)
#define LAN87XX_BSR_AUTONEGO_ABILITY 		 ((uint16_t)0x0008U)
#define LAN87XX_BSR_LINK_STATUS      		 ((uint16_t)0x0004U)
#define LAN87XX_BSR_JABBER_DETECT    		 ((uint16_t)0x0002U)
#define LAN87XX_BSR_EXTENDED_CAP     		 ((uint16_t)0x0001U)

#define LAN87XX_PHYI1R_OUI_3_18           	 ((uint16_t)0xFFFFU)

#define LAN87XX_PHYI2R_OUI_19_24          	 ((uint16_t)0xFC00U)
#define LAN87XX_PHYI2R_MODEL_NBR          	 ((uint16_t)0x03F0U)
#define LAN87XX_PHYI2R_REVISION_NBR       	 ((uint16_t)0x000FU)

#define LAN87XX_ANAR_NEXT_PAGE               ((uint16_t)0x8000U)
#define LAN87XX_ANAR_REMOTE_FAULT            ((uint16_t)0x2000U)
#define LAN87XX_ANAR_PAUSE_OPERATION         ((uint16_t)0x0C00U)
#define LAN87XX_ANAR_PO_NOPAUSE              ((uint16_t)0x0000U)
#define LAN87XX_ANAR_PO_SYMMETRIC_PAUSE      ((uint16_t)0x0400U)
#define LAN87XX_ANAR_PO_ASYMMETRIC_PAUSE     ((uint16_t)0x0800U)
#define LAN87XX_ANAR_PO_ADVERTISE_SUPPORT    ((uint16_t)0x0C00U)
#define LAN87XX_ANAR_100BASE_TX_FD           ((uint16_t)0x0100U)
#define LAN87XX_ANAR_100BASE_TX              ((uint16_t)0x0080U)
#define LAN87XX_ANAR_10BASE_T_FD             ((uint16_t)0x0040U)
#define LAN87XX_ANAR_10BASE_T                ((uint16_t)0x0020U)
#define LAN87XX_ANAR_SELECTOR_FIELD          ((uint16_t)0x000FU)

#define LAN87XX_ANLPAR_NEXT_PAGE             ((uint16_t)0x8000U)
#define LAN87XX_ANLPAR_REMOTE_FAULT          ((uint16_t)0x2000U)
#define LAN87XX_ANLPAR_PAUSE_OPERATION       ((uint16_t)0x0C00U)
#define LAN87XX_ANLPAR_PO_NOPAUSE            ((uint16_t)0x0000U)
#define LAN87XX_ANLPAR_PO_SYMMETRIC_PAUSE    ((uint16_t)0x0400U)
#define LAN87XX_ANLPAR_PO_ASYMMETRIC_PAUSE   ((uint16_t)0x0800U)
#define LAN87XX_ANLPAR_PO_ADVERTISE_SUPPORT  ((uint16_t)0x0C00U)
#define LAN87XX_ANLPAR_100BASE_TX_FD         ((uint16_t)0x0100U)
#define LAN87XX_ANLPAR_100BASE_TX            ((uint16_t)0x0080U)
#define LAN87XX_ANLPAR_10BASE_T_FD           ((uint16_t)0x0040U)
#define LAN87XX_ANLPAR_10BASE_T              ((uint16_t)0x0020U)
#define LAN87XX_ANLPAR_SELECTOR_FIELD        ((uint16_t)0x000FU)

#define LAN87XX_ANER_RX_NP_LOCATION_ABLE     ((uint16_t)0x0040U)
#define LAN87XX_ANER_RX_NP_STORAGE_LOCATION  ((uint16_t)0x0020U)
#define LAN87XX_ANER_PARALLEL_DETECT_FAULT   ((uint16_t)0x0010U)
#define LAN87XX_ANER_LP_NP_ABLE              ((uint16_t)0x0008U)
#define LAN87XX_ANER_NP_ABLE                 ((uint16_t)0x0004U)
#define LAN87XX_ANER_PAGE_RECEIVED           ((uint16_t)0x0002U)
#define LAN87XX_ANER_LP_AUTONEG_ABLE         ((uint16_t)0x0001U)

#define LAN87XX_ANNPTR_NEXT_PAGE         	 ((uint16_t)0x8000U)
#define LAN87XX_ANNPTR_MESSAGE_PAGE      	 ((uint16_t)0x2000U)
#define LAN87XX_ANNPTR_ACK2              	 ((uint16_t)0x1000U)
#define LAN87XX_ANNPTR_TOGGLE            	 ((uint16_t)0x0800U)
#define LAN87XX_ANNPTR_MESSAGGE_CODE     	 ((uint16_t)0x07FFU)

#define LAN87XX_ANNPTR_NEXT_PAGE         	 ((uint16_t)0x8000U)
#define LAN87XX_ANNPRR_ACK               	 ((uint16_t)0x4000U)
#define LAN87XX_ANNPRR_MESSAGE_PAGE      	 ((uint16_t)0x2000U)
#define LAN87XX_ANNPRR_ACK2              	 ((uint16_t)0x1000U)
#define LAN87XX_ANNPRR_TOGGLE            	 ((uint16_t)0x0800U)
#define LAN87XX_ANNPRR_MESSAGGE_CODE     	 ((uint16_t)0x07FFU)

#define LAN87XX_MMDACR_MMD_FUNCTION       	 ((uint16_t)0xC000U)
#define LAN87XX_MMDACR_MMD_FUNCTION_ADDR  	 ((uint16_t)0x0000U)
#define LAN87XX_MMDACR_MMD_FUNCTION_DATA  	 ((uint16_t)0x4000U)
#define LAN87XX_MMDACR_MMD_DEV_ADDR       	 ((uint16_t)0x001FU)

#define LAN87XX_ENCTR_TX_ENABLE              ((uint16_t)0x8000U)
#define LAN87XX_ENCTR_TX_TIMER               ((uint16_t)0x6000U)
#define LAN87XX_ENCTR_TX_TIMER_1S            ((uint16_t)0x0000U)
#define LAN87XX_ENCTR_TX_TIMER_768MS         ((uint16_t)0x2000U)
#define LAN87XX_ENCTR_TX_TIMER_512MS         ((uint16_t)0x4000U)
#define LAN87XX_ENCTR_TX_TIMER_265MS         ((uint16_t)0x6000U)
#define LAN87XX_ENCTR_RX_ENABLE              ((uint16_t)0x1000U)
#define LAN87XX_ENCTR_RX_MAX_INTERVAL        ((uint16_t)0x0C00U)
#define LAN87XX_ENCTR_RX_MAX_INTERVAL_64MS   ((uint16_t)0x0000U)
#define LAN87XX_ENCTR_RX_MAX_INTERVAL_256MS  ((uint16_t)0x0400U)
#define LAN87XX_ENCTR_RX_MAX_INTERVAL_512MS  ((uint16_t)0x0800U)
#define LAN87XX_ENCTR_RX_MAX_INTERVAL_1S     ((uint16_t)0x0C00U)
#define LAN87XX_ENCTR_EX_CROSS_OVER          ((uint16_t)0x0002U)
#define LAN87XX_ENCTR_EX_MANUAL_CROSS_OVER   ((uint16_t)0x0001U)

#define LAN87XX_MCSR_EDPWRDOWN        		 ((uint16_t)0x2000U)
#define LAN87XX_MCSR_FARLOOPBACK      		 ((uint16_t)0x0200U)
#define LAN87XX_MCSR_ALTINT           		 ((uint16_t)0x0040U)
#define LAN87XX_MCSR_ENERGYON         		 ((uint16_t)0x0002U)

#define LAN87XX_SMR_MODE       				 ((uint16_t)0x00E0U)
#define LAN87XX_SMR_PHY_ADDR   				 ((uint16_t)0x001FU)

#define LAN87XX_TPDCR_DELAY_IN               ((uint16_t)0x8000U)
#define LAN87XX_TPDCR_LINE_BREAK_COUNTER     ((uint16_t)0x7000U)
#define LAN87XX_TPDCR_PATTERN_HIGH           ((uint16_t)0x0FC0U)
#define LAN87XX_TPDCR_PATTERN_LOW            ((uint16_t)0x003FU)

#define LAN87XX_TCSR_TDR_ENABLE           	 ((uint16_t)0x8000U)
#define LAN87XX_TCSR_TDR_AD_FILTER_ENABLE 	 ((uint16_t)0x4000U)
#define LAN87XX_TCSR_TDR_CH_CABLE_TYPE    	 ((uint16_t)0x0600U)
#define LAN87XX_TCSR_TDR_CH_CABLE_DEFAULT 	 ((uint16_t)0x0000U)
#define LAN87XX_TCSR_TDR_CH_CABLE_SHORTED 	 ((uint16_t)0x0200U)
#define LAN87XX_TCSR_TDR_CH_CABLE_OPEN    	 ((uint16_t)0x0400U)
#define LAN87XX_TCSR_TDR_CH_CABLE_MATCH   	 ((uint16_t)0x0600U)
#define LAN87XX_TCSR_TDR_CH_STATUS        	 ((uint16_t)0x0100U)
#define LAN87XX_TCSR_TDR_CH_LENGTH        	 ((uint16_t)0x00FFU)

#define LAN87XX_SCSIR_AUTO_MDIX_ENABLE    	 ((uint16_t)0x8000U)
#define LAN87XX_SCSIR_CHANNEL_SELECT      	 ((uint16_t)0x2000U)
#define LAN87XX_SCSIR_SQE_DISABLE         	 ((uint16_t)0x0800U)
#define LAN87XX_SCSIR_XPOLALITY           	 ((uint16_t)0x0010U)

#define LAN87XX_CLR_CABLE_LENGTH       	 	 ((uint16_t)0xF000U)

#define LAN87XX_INT_8       				 ((uint16_t)0x0100U)
#define LAN87XX_INT_7       				 ((uint16_t)0x0080U)
#define LAN87XX_INT_6       				 ((uint16_t)0x0040U)
#define LAN87XX_INT_5       				 ((uint16_t)0x0020U)
#define LAN87XX_INT_4       				 ((uint16_t)0x0010U)
#define LAN87XX_INT_3       				 ((uint16_t)0x0008U)
#define LAN87XX_INT_2       				 ((uint16_t)0x0004U)
#define LAN87XX_INT_1       				 ((uint16_t)0x0002U)

#define LAN87XX_PHYSCSR_AUTONEGO_DONE   	 ((uint16_t)0x1000U)
#define LAN87XX_PHYSCSR_HCDSPEEDMASK    	 ((uint16_t)0x001CU)
#define LAN87XX_PHYSCSR_10BT_HD         	 ((uint16_t)0x0004U)
#define LAN87XX_PHYSCSR_10BT_FD         	 ((uint16_t)0x0014U)
#define LAN87XX_PHYSCSR_100BTX_HD       	 ((uint16_t)0x0008U)
#define LAN87XX_PHYSCSR_100BTX_FD       	 ((uint16_t)0x0018U)

#define LAN87XX_STATUS_READ_ERROR            ((int32_t)-5)
#define LAN87XX_STATUS_WRITE_ERROR           ((int32_t)-4)
#define LAN87XX_STATUS_ADDRESS_ERROR         ((int32_t)-3)
#define LAN87XX_STATUS_RESET_TIMEOUT         ((int32_t)-2)
#define LAN87XX_STATUS_ERROR                 ((int32_t)-1)
#define LAN87XX_STATUS_OK                    ((int32_t) 0)
#define LAN87XX_STATUS_LINK_DOWN             ((int32_t) 1)
#define LAN87XX_STATUS_100MBITS_FULLDUPLEX   ((int32_t) 2)
#define LAN87XX_STATUS_100MBITS_HALFDUPLEX   ((int32_t) 3)
#define LAN87XX_STATUS_10MBITS_FULLDUPLEX    ((int32_t) 4)
#define LAN87XX_STATUS_10MBITS_HALFDUPLEX    ((int32_t) 5)
#define LAN87XX_STATUS_AUTONEGO_NOTDONE      ((int32_t) 6)

#define LAN87XX_WOL_IT                       LAN87XX_INT_8
#define LAN87XX_ENERGYON_IT                  LAN87XX_INT_7
#define LAN87XX_AUTONEGO_COMPLETE_IT         LAN87XX_INT_6
#define LAN87XX_REMOTE_FAULT_IT              LAN87XX_INT_5
#define LAN87XX_LINK_DOWN_IT                 LAN87XX_INT_4
#define LAN87XX_AUTONEGO_LP_ACK_IT           LAN87XX_INT_3
#define LAN87XX_PARALLEL_DETECTION_FAULT_IT  LAN87XX_INT_2
#define LAN87XX_AUTONEGO_PAGE_RECEIVED_IT    LAN87XX_INT_1



#endif /* DRVPERDEV_LAN87XX_MACROS_H_ */
