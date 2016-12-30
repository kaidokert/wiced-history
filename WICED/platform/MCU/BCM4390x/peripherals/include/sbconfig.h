/*
 * Copyright 2016, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 * Broadcom SiliconBackplane hardware register definitions.
 *
 * $Id: sbconfig.h 456342 2014-02-18 16:09:19Z weitsan $
 */

#ifndef	_SBCONFIG_H
#define	_SBCONFIG_H

/* cpp contortions to concatenate w/arg prescan */
#ifndef PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif

/* enumeration in SB is based on the premise that cores are contiguos in the
 * enumeration space.
 */
#define SB_BUS_SIZE		0x10000		/* Each bus gets 64Kbytes for cores */
#define SB_BUS_BASE(b)		(SI_ENUM_BASE + (b) * SB_BUS_SIZE)
#define	SB_BUS_MAXCORES		(SB_BUS_SIZE / SI_CORE_SIZE)	/* Max cores per bus */

/*
 * Sonics Configuration Space Registers.
 */
#define	SBCONFIGOFF		0xf00		/* core sbconfig regs are top 256bytes of regs */
#define	SBCONFIGSIZE		256		/* sizeof (sbconfig_t) */

#define SBIPSFLAG		0x08
#define SBTPSFLAG		0x18
#define	SBTMERRLOGA		0x48		/* sonics >= 2.3 */
#define	SBTMERRLOG		0x50		/* sonics >= 2.3 */
#define SBADMATCH3		0x60
#define SBADMATCH2		0x68
#define SBADMATCH1		0x70
#define SBIMSTATE		0x90
#define SBINTVEC		0x94
#define SBTMSTATELOW		0x98
#define SBTMSTATEHIGH		0x9c
#define SBBWA0			0xa0
#define SBIMCONFIGLOW		0xa8
#define SBIMCONFIGHIGH		0xac
#define SBADMATCH0		0xb0
#define SBTMCONFIGLOW		0xb8
#define SBTMCONFIGHIGH		0xbc
#define SBBCONFIG		0xc0
#define SBBSTATE		0xc8
#define SBACTCNFG		0xd8
#define	SBFLAGST		0xe8
#define SBIDLOW			0xf8
#define SBIDHIGH		0xfc

/* All the previous registers are above SBCONFIGOFF, but with Sonics 2.3, we have
 * a few registers *below* that line. I think it would be very confusing to try
 * and change the value of SBCONFIGOFF, so I'm definig them as absolute offsets here,
 */

#define SBIMERRLOGA		0xea8
#define SBIMERRLOG		0xeb0
#define SBTMPORTCONNID0		0xed8
#define SBTMPORTLOCK0		0xef8

#if !defined(_LANGUAGE_ASSEMBLY) && !defined(__ASSEMBLY__)

typedef volatile struct _sbconfig {
	uint32	PAD[2];
	uint32	sbipsflag;		/* initiator port ocp slave flag */
	uint32	PAD[3];
	uint32	sbtpsflag;		/* target port ocp slave flag */
	uint32	PAD[11];
	uint32	sbtmerrloga;		/* (sonics >= 2.3) */
	uint32	PAD;
	uint32	sbtmerrlog;		/* (sonics >= 2.3) */
	uint32	PAD[3];
	uint32	sbadmatch3;		/* address match3 */
	uint32	PAD;
	uint32	sbadmatch2;		/* address match2 */
	uint32	PAD;
	uint32	sbadmatch1;		/* address match1 */
	uint32	PAD[7];
	uint32	sbimstate;		/* initiator agent state */
	uint32	sbintvec;		/* interrupt mask */
	uint32	sbtmstatelow;		/* target state */
	uint32	sbtmstatehigh;		/* target state */
	uint32	sbbwa0;			/* bandwidth allocation table0 */
	uint32	PAD;
	uint32	sbimconfiglow;		/* initiator configuration */
	uint32	sbimconfighigh;		/* initiator configuration */
	uint32	sbadmatch0;		/* address match0 */
	uint32	PAD;
	uint32	sbtmconfiglow;		/* target configuration */
	uint32	sbtmconfighigh;		/* target configuration */
	uint32	sbbconfig;		/* broadcast configuration */
	uint32	PAD;
	uint32	sbbstate;		/* broadcast state */
	uint32	PAD[3];
	uint32	sbactcnfg;		/* activate configuration */
	uint32	PAD[3];
	uint32	sbflagst;		/* current sbflags */
	uint32	PAD[3];
	uint32	sbidlow;		/* identification */
	uint32	sbidhigh;		/* identification */
} sbconfig_t;

#endif /* !_LANGUAGE_ASSEMBLY && !__ASSEMBLY__ */

/* sbipsflag */
#define	SBIPS_INT1_MASK		0x3f		/* which sbflags get routed to mips interrupt 1 */
#define	SBIPS_INT1_SHIFT	0
#define	SBIPS_INT2_MASK		0x3f00		/* which sbflags get routed to mips interrupt 2 */
#define	SBIPS_INT2_SHIFT	8
#define	SBIPS_INT3_MASK		0x3f0000	/* which sbflags get routed to mips interrupt 3 */
#define	SBIPS_INT3_SHIFT	16
#define	SBIPS_INT4_MASK		0x3f000000	/* which sbflags get routed to mips interrupt 4 */
#define	SBIPS_INT4_SHIFT	24

/* sbtpsflag */
#define	SBTPS_NUM0_MASK		0x3f		/* interrupt sbFlag # generated by this core */
#define	SBTPS_F0EN0		0x40		/* interrupt is always sent on the backplane */

/* sbtmerrlog */
#define	SBTMEL_CM		0x00000007	/* command */
#define	SBTMEL_CI		0x0000ff00	/* connection id */
#define	SBTMEL_EC		0x0f000000	/* error code */
#define	SBTMEL_ME		0x80000000	/* multiple error */

/* sbimstate */
#define	SBIM_PC			0xf		/* pipecount */
#define	SBIM_AP_MASK		0x30		/* arbitration policy */
#define	SBIM_AP_BOTH		0x00		/* use both timeslaces and token */
#define	SBIM_AP_TS		0x10		/* use timesliaces only */
#define	SBIM_AP_TK		0x20		/* use token only */
#define	SBIM_AP_RSV		0x30		/* reserved */
#define	SBIM_IBE		0x20000		/* inbanderror */
#define	SBIM_TO			0x40000		/* timeout */
#define	SBIM_BY			0x01800000	/* busy (sonics >= 2.3) */
#define	SBIM_RJ			0x02000000	/* reject (sonics >= 2.3) */

/* sbtmstatelow */
#define	SBTML_RESET		0x0001		/* reset */
#define	SBTML_REJ_MASK		0x0006		/* reject field */
#define	SBTML_REJ		0x0002		/* reject */
#define	SBTML_TMPREJ		0x0004		/* temporary reject, for error recovery */

#define	SBTML_SICF_SHIFT	16		/* Shift to locate the SI control flags in sbtml */

/* sbtmstatehigh */
#define	SBTMH_SERR		0x0001		/* serror */
#define	SBTMH_INT		0x0002		/* interrupt */
#define	SBTMH_BUSY		0x0004		/* busy */
#define	SBTMH_TO		0x0020		/* timeout (sonics >= 2.3) */

#define	SBTMH_SISF_SHIFT	16		/* Shift to locate the SI status flags in sbtmh */

/* sbbwa0 */
#define	SBBWA_TAB0_MASK		0xffff		/* lookup table 0 */
#define	SBBWA_TAB1_MASK		0xffff		/* lookup table 1 */
#define	SBBWA_TAB1_SHIFT	16

/* sbimconfiglow */
#define	SBIMCL_STO_MASK		0x7		/* service timeout */
#define	SBIMCL_RTO_MASK		0x70		/* request timeout */
#define	SBIMCL_RTO_SHIFT	4
#define	SBIMCL_CID_MASK		0xff0000	/* connection id */
#define	SBIMCL_CID_SHIFT	16

/* sbimconfighigh */
#define	SBIMCH_IEM_MASK		0xc		/* inband error mode */
#define	SBIMCH_TEM_MASK		0x30		/* timeout error mode */
#define	SBIMCH_TEM_SHIFT	4
#define	SBIMCH_BEM_MASK		0xc0		/* bus error mode */
#define	SBIMCH_BEM_SHIFT	6

/* sbadmatch0 */
#define	SBAM_TYPE_MASK		0x3		/* address type */
#define	SBAM_AD64		0x4		/* reserved */
#define	SBAM_ADINT0_MASK	0xf8		/* type0 size */
#define	SBAM_ADINT0_SHIFT	3
#define	SBAM_ADINT1_MASK	0x1f8		/* type1 size */
#define	SBAM_ADINT1_SHIFT	3
#define	SBAM_ADINT2_MASK	0x1f8		/* type2 size */
#define	SBAM_ADINT2_SHIFT	3
#define	SBAM_ADEN		0x400		/* enable */
#define	SBAM_ADNEG		0x800		/* negative decode */
#define	SBAM_BASE0_MASK		0xffffff00	/* type0 base address */
#define	SBAM_BASE0_SHIFT	8
#define	SBAM_BASE1_MASK		0xfffff000	/* type1 base address for the core */
#define	SBAM_BASE1_SHIFT	12
#define	SBAM_BASE2_MASK		0xffff0000	/* type2 base address for the core */
#define	SBAM_BASE2_SHIFT	16

/* sbtmconfiglow */
#define	SBTMCL_CD_MASK		0xff		/* clock divide */
#define	SBTMCL_CO_MASK		0xf800		/* clock offset */
#define	SBTMCL_CO_SHIFT		11
#define	SBTMCL_IF_MASK		0xfc0000	/* interrupt flags */
#define	SBTMCL_IF_SHIFT		18
#define	SBTMCL_IM_MASK		0x3000000	/* interrupt mode */
#define	SBTMCL_IM_SHIFT		24

/* sbtmconfighigh */
#define	SBTMCH_BM_MASK		0x3		/* busy mode */
#define	SBTMCH_RM_MASK		0x3		/* retry mode */
#define	SBTMCH_RM_SHIFT		2
#define	SBTMCH_SM_MASK		0x30		/* stop mode */
#define	SBTMCH_SM_SHIFT		4
#define	SBTMCH_EM_MASK		0x300		/* sb error mode */
#define	SBTMCH_EM_SHIFT		8
#define	SBTMCH_IM_MASK		0xc00		/* int mode */
#define	SBTMCH_IM_SHIFT		10

/* sbbconfig */
#define	SBBC_LAT_MASK		0x3		/* sb latency */
#define	SBBC_MAX0_MASK		0xf0000		/* maxccntr0 */
#define	SBBC_MAX0_SHIFT		16
#define	SBBC_MAX1_MASK		0xf00000	/* maxccntr1 */
#define	SBBC_MAX1_SHIFT		20

/* sbbstate */
#define	SBBS_SRD		0x1		/* st reg disable */
#define	SBBS_HRD		0x2		/* hold reg disable */

/* sbidlow */
#define	SBIDL_CS_MASK		0x3		/* config space */
#define	SBIDL_AR_MASK		0x38		/* # address ranges supported */
#define	SBIDL_AR_SHIFT		3
#define	SBIDL_SYNCH		0x40		/* sync */
#define	SBIDL_INIT		0x80		/* initiator */
#define	SBIDL_MINLAT_MASK	0xf00		/* minimum backplane latency */
#define	SBIDL_MINLAT_SHIFT	8
#define	SBIDL_MAXLAT		0xf000		/* maximum backplane latency */
#define	SBIDL_MAXLAT_SHIFT	12
#define	SBIDL_FIRST		0x10000		/* this initiator is first */
#define	SBIDL_CW_MASK		0xc0000		/* cycle counter width */
#define	SBIDL_CW_SHIFT		18
#define	SBIDL_TP_MASK		0xf00000	/* target ports */
#define	SBIDL_TP_SHIFT		20
#define	SBIDL_IP_MASK		0xf000000	/* initiator ports */
#define	SBIDL_IP_SHIFT		24
#define	SBIDL_RV_MASK		0xf0000000	/* sonics backplane revision code */
#define	SBIDL_RV_SHIFT		28
#define	SBIDL_RV_2_2		0x00000000	/* version 2.2 or earlier */
#define	SBIDL_RV_2_3		0x10000000	/* version 2.3 */

/* sbidhigh */
#define	SBIDH_RC_MASK		0x000f		/* revision code */
#define	SBIDH_RCE_MASK		0x7000		/* revision code extension field */
#define	SBIDH_RCE_SHIFT		8
#define	SBCOREREV(sbidh) \
	((((sbidh) & SBIDH_RCE_MASK) >> SBIDH_RCE_SHIFT) | ((sbidh) & SBIDH_RC_MASK))
#define	SBIDH_CC_MASK		0x8ff0		/* core code */
#define	SBIDH_CC_SHIFT		4
#define	SBIDH_VC_MASK		0xffff0000	/* vendor code */
#define	SBIDH_VC_SHIFT		16

#define	SB_COMMIT		0xfd8		/* update buffered registers value */

/* vendor codes */
#define	SB_VEND_BCM		0x4243		/* Broadcom's SB vendor code */

#endif	/* _SBCONFIG_H */
