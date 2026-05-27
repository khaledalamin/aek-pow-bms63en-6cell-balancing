#include "AEK_POW_BMS63CHAIN_iso_mng.h"


#pragma GCC push_options
#pragma GCC optimize ("-O0")

extern AEK_POW_BMS63CHAIN_chain_t AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_CHAIN_NUM];

void AEK_POW_BMS63CHAIN_iso_init(AEK_POW_BMS63CHAIN_chain_name_t AEK_POW_BMS63CHAIN_chain_name)
{
    /*
     * DIS is active-high disable.
     * Passing DISABLE here means: do not disable the ISO-SPI transceiver.
     */
    AEK_POW_BMS63CHAIN_iso_setDis(
        AEK_POW_BMS63CHAIN_chain_name,
        AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS,
        AEK_POW_BMS63CHAIN_DISABLE
    );

    /*
     * Keep ISO-SPI frequency LOW for bench wiring robustness.
     */
    AEK_POW_BMS63CHAIN_iso_setISOFreq(
        AEK_POW_BMS63CHAIN_chain_name,
        AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS,
        AEK_POW_BMS63CHAIN_ISO_FREQ_LOW
    );

    /*
     * TXEN is controlled only if a real pin is mapped.
     * Port 0 / pin 4 is valid. 0 / 0 means unmapped.
     */
    if((AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name]
            .AEK_POW_BMS63CHAIN_ISOspi_chainConf
            .AEK_POW_BMS63CHAIN_iso_portTXEn != 0U) ||
       (AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name]
            .AEK_POW_BMS63CHAIN_ISOspi_chainConf
            .AEK_POW_BMS63CHAIN_iso_pinTXEn != 0U)){

        AEK_POW_BMS63CHAIN_iso_setTXEn(
            AEK_POW_BMS63CHAIN_chain_name,
            AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS,
            AEK_POW_BMS63CHAIN_ENABLE
        );
    }

    /*
     * TXAMP is intentionally not driven here.
     * Leave it to hardware/default configuration for this bench setup.
     */
}


void AEK_POW_BMS63CHAIN_iso_signalSetting(
    AEK_POW_BMS63CHAIN_chain_name_t AEK_POW_BMS63CHAIN_chain_name,
    AEK_POW_BMS63CHAIN_iso_isofreqSel_t AEK_POW_BMS63CHAIN_iso_isofreqSel,
    AEK_POW_BMS63CHAIN_iso_isoTxAmp_t AEK_POW_BMS63CHAIN_iso_isoTxAmp)
{
    (void)AEK_POW_BMS63CHAIN_iso_isoTxAmp;

    /*
     * Runtime signal tuning is limited to ISOFREQ only.
     * TXAMP is not controlled by firmware because TXAMP is unmapped in
     * this bench configuration.
     */
    AEK_POW_BMS63CHAIN_iso_setISOFreq(
        AEK_POW_BMS63CHAIN_chain_name,
        AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS,
        AEK_POW_BMS63CHAIN_iso_isofreqSel
    );

    if(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name]
            .AEK_POW_BMS63CHAIN_spi_chainConf
            .AEK_POW_BMS63CHAIN_spi_accessType == AEK_POW_BMS63CHAIN_SPI_DUALACCESS){

        AEK_POW_BMS63CHAIN_iso_setISOFreq(
            AEK_POW_BMS63CHAIN_chain_name,
            AEK_POW_BMS63CHAIN_SPI_DUALACCESS,
            AEK_POW_BMS63CHAIN_iso_isofreqSel
        );
    }
}


void AEK_POW_BMS63CHAIN_iso_setDis(AEK_POW_BMS63CHAIN_chain_name_t AEK_POW_BMS63CHAIN_chain_name, AEK_POW_BMS63CHAIN_spi_access_t AEK_POW_BMS63CHAIN_spi_access, AEK_POW_BMS63CHAIN_sts_t AEK_POW_BMS63CHAIN_disSts){
	if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS){
		if(AEK_POW_BMS63CHAIN_disSts == AEK_POW_BMS63CHAIN_ENABLE){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portDis, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinDis);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portDis, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinDis);
		}
	}
	else if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_DUALACCESS){
		if(AEK_POW_BMS63CHAIN_disSts == AEK_POW_BMS63CHAIN_ENABLE){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portDisDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinDisDual);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portDisDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinDisDual);
		}
	}
	else{}
}

void AEK_POW_BMS63CHAIN_iso_setISOFreq(AEK_POW_BMS63CHAIN_chain_name_t AEK_POW_BMS63CHAIN_chain_name, AEK_POW_BMS63CHAIN_spi_access_t AEK_POW_BMS63CHAIN_spi_access, AEK_POW_BMS63CHAIN_iso_isofreqSel_t AEK_POW_BMS63CHAIN_iso_isofreqSel){
	if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS){
		if(AEK_POW_BMS63CHAIN_iso_isofreqSel == AEK_POW_BMS63CHAIN_ISO_FREQ_HIGH){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portISOFreq, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinISOFreq);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portISOFreq, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinISOFreq);
		}
	}
	else if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_DUALACCESS){
		if(AEK_POW_BMS63CHAIN_iso_isofreqSel == AEK_POW_BMS63CHAIN_ISO_FREQ_HIGH){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portISOFreqDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinISOFreqDual);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portISOFreqDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinISOFreqDual);
		}
	}
	else{}
}

void AEK_POW_BMS63CHAIN_iso_setTXEn(AEK_POW_BMS63CHAIN_chain_name_t AEK_POW_BMS63CHAIN_chain_name, AEK_POW_BMS63CHAIN_spi_access_t AEK_POW_BMS63CHAIN_spi_access, AEK_POW_BMS63CHAIN_sts_t AEK_POW_BMS63CHAIN_TxSts){
	if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS){
		if(AEK_POW_BMS63CHAIN_TxSts == AEK_POW_BMS63CHAIN_ENABLE){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXEn, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXEn);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXEn, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXEn);
		}
	}
	else if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_DUALACCESS){
		if(AEK_POW_BMS63CHAIN_TxSts == AEK_POW_BMS63CHAIN_ENABLE){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXEnDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXEnDual);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXEnDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXEnDual);
		}
	}
	else{}
}

void AEK_POW_BMS63CHAIN_iso_setTxAmp(AEK_POW_BMS63CHAIN_chain_name_t AEK_POW_BMS63CHAIN_chain_name, AEK_POW_BMS63CHAIN_spi_access_t AEK_POW_BMS63CHAIN_spi_access, AEK_POW_BMS63CHAIN_iso_isoTxAmp_t AEK_POW_BMS63CHAIN_iso_isoTxAmp){
	if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS){
		if(AEK_POW_BMS63CHAIN_iso_isoTxAmp == AEK_POW_BMS63CHAIN_ISO_TXAMPHIGH){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXAmp, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXAmp);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXAmp, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXAmp);
		}
	}
	else if(AEK_POW_BMS63CHAIN_spi_access == AEK_POW_BMS63CHAIN_SPI_DUALACCESS){
		if(AEK_POW_BMS63CHAIN_iso_isoTxAmp == AEK_POW_BMS63CHAIN_ISO_TXAMPHIGH){
			pal_setpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXAmpDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXAmpDual);
		}
		else{
			pal_clearpad(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_portTXAmpDual, AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chain_name].AEK_POW_BMS63CHAIN_ISOspi_chainConf.AEK_POW_BMS63CHAIN_iso_pinTXAmpDual);
		}
	}
	else{}
}

#pragma GCC pop_options
