/*
 * Model_Config.h
 *
 *  Created on: 25 Jul 2020
 *      Author: Ralim
 */

#ifndef BSP_MINIWARE_MODEL_CONFIG_H_
#define BSP_MINIWARE_MODEL_CONFIG_H_
/*
 * Lookup for mapping features <-> Models
 */

#if defined(MODEL_OtterIron_Pro) > 1
#error "Multiple models defined!"
#elif defined(MODEL_OtterIron_Pro) == 0
#error "No model defined!"
#endif

#ifdef MODEL_OtterIron_Pro
#define POW_PD
#define TEMP_NTC
#define LIS_ORI_FLIP
#define BATTFILTERDEPTH 8
#endif

#endif /* BSP_MINIWARE_MODEL_CONFIG_H_ */
