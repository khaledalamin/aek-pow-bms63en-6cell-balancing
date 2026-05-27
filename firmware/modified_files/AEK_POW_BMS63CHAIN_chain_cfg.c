


#include "AEK_POW_BMS63CHAIN_chain_cfg.h"



#define AEK_POW_BMS63CHAIN_CHAIN_NUM 1

	

AEK_POW_BMS63CHAIN_node_t  AEK_POW_BMS63CHAIN_chainNode0[] = {
		{
			AEK_POW_BMS63CHAIN_NODE_DEV1,
			AEK_POW_BMS63CHAIN_NODE_IDLE,
			100,
			AEK_POW_BMS63CHAIN_ENABLE,
			AEK_POW_BMS63CHAIN_ENABLE,
			AEK_POW_BMS63CHAIN_ENABLE,
			AEK_POW_BMS63CHAIN_ENABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_ENABLE,
			AEK_POW_BMS63CHAIN_ENABLE,
			0.1,
			0.1,
			0.1,
			0.1,
			0.1,
			0.1,
			0.1,
			1.5,
			1.5,
			1.5,
			1.5,
			0.9,
			0.9,
			0.9,
			0.9,
			0.9,
			0.9,
			0.9,
			4.5,
			4.5,
			4.5,
			4.5,
			AEK_POW_BMS63CHAIN_ANALOG_INPUT,
			AEK_POW_BMS63CHAIN_ANALOG_INPUT,
			AEK_POW_BMS63CHAIN_ANALOG_INPUT,
			AEK_POW_BMS63CHAIN_ANALOG_INPUT,
			AEK_POW_BMS63CHAIN_NOT_USED,
			AEK_POW_BMS63CHAIN_NOT_USED,
			AEK_POW_BMS63CHAIN_ANALOG_INPUT,
			AEK_POW_BMS63CHAIN_RATIOMODE,
			AEK_POW_BMS63CHAIN_RATIOMODE,
			AEK_POW_BMS63CHAIN_RATIOMODE,
			AEK_POW_BMS63CHAIN_RATIOMODE,
			AEK_POW_BMS63CHAIN_RATIOMODE,
			AEK_POW_BMS63CHAIN_RATIOMODE,
			AEK_POW_BMS63CHAIN_RATIOMODE,
			4.2,
			2.5,
			25.2,
			15.0,
			AEK_POW_BMS63CHAIN_DISABLE,
			AEK_POW_BMS63CHAIN_ENABLE,
			1.7,
			0.1,
			AEK_POW_BMS63CHAIN_ADCFILTER_290us,
			AEK_POW_BMS63CHAIN_ADCFILTER_290us,
			AEK_POW_BMS63CHAIN_ADCFILTER_290us,
			0,
			0,
			1,
			1,
			1,
			1,
			0,
			AEK_POW_BMS63CHAIN_ENABLE,
			AEK_POW_BMS63CHAIN_ENABLE,
			AEK_POW_BMS63CHAIN_DISABLE,
		}
};

AEK_POW_BMS63CHAIN_chain_t AEK_POW_BMS63CHAIN_chain[] = {
    {
        .AEK_POW_BMS63CHAIN_chain_name_t = AEK_POW_BMS63CHAIN_CHAIN0,
        .AEK_POW_BMS63CHAIN_node_comType = AEK_POW_BMS63CHAIN_COM_ISOSPI,

        .AEK_POW_BMS63CHAIN_spi_chainConf = {
            .AEK_POW_BMS63CHAIN_spi_singleAccessDrv  = &SPID1,
            .AEK_POW_BMS63CHAIN_spi_singleAccessConf = &spi_config_spi_config_AEK_POW_BMS63EN_dev00,
            .AEK_POW_BMS63CHAIN_spi_dualAccessDrv    = NULL,
            .AEK_POW_BMS63CHAIN_spi_dualAccessConf   = NULL,
            .AEK_POW_BMS63CHAIN_spi_txRxSts          = AEK_POW_BMS63CHAIN_SPI_RX_COMPLETED,
            .AEK_POW_BMS63CHAIN_spi_frameErrSts      = AEK_POW_BMS63CHAIN_SPI_FRAME_NO_ERROR,
            .AEK_POW_BMS63CHAIN_spi_GSWErrSts        = AEK_POW_BMS63CHAIN_SPI_GSW_NO_ERROR,
            .AEK_POW_BMS63CHAIN_spi_accessType       = AEK_POW_BMS63CHAIN_SPI_SINGLEACCESS
        },

        .AEK_POW_BMS63CHAIN_ISOspi_chainConf = {
            .AEK_POW_BMS63CHAIN_iso_portDis =
                PORT_PIN_AEK_POW_BMS63ENBoard00_CN2_DIS_GPIO2,
            .AEK_POW_BMS63CHAIN_iso_pinDis =
                PIN_AEK_POW_BMS63ENBoard00_CN2_DIS_GPIO2,

            .AEK_POW_BMS63CHAIN_iso_portISOFreq =
                PORT_PIN_AEK_POW_BMS63ENBoard00_CN2_ISOFREQ_GPIO3,
            .AEK_POW_BMS63CHAIN_iso_pinISOFreq =
                PIN_AEK_POW_BMS63ENBoard00_CN2_ISOFREQ_GPIO3,

            /*
             * BNE is not used in this bench configuration.
             */
            .AEK_POW_BMS63CHAIN_iso_portBNE = 0,
            .AEK_POW_BMS63CHAIN_iso_pinBNE  = 0,

            /*
             * TXEN is needed. In UDE it should appear as port 0, pin 4.
             * Port 0 is acceptable; it usually means GPIO port A/index 0.
             */
            .AEK_POW_BMS63CHAIN_iso_portTXEn =
                PORT_PIN_AEK_POW_BMS63ENBoard00_CN2_TXEN_GPIO4,
            .AEK_POW_BMS63CHAIN_iso_pinTXEn =
                PIN_AEK_POW_BMS63ENBoard00_CN2_TXEN_GPIO4,

            /*
             * TXAMP is not controlled by firmware for now.
             */
            .AEK_POW_BMS63CHAIN_iso_portTXAmp = 0,
            .AEK_POW_BMS63CHAIN_iso_pinTXAmp  = 0,

            /*
             * Dual-access pins are not used.
             */
            .AEK_POW_BMS63CHAIN_iso_portDisDual       = 0,
            .AEK_POW_BMS63CHAIN_iso_pinDisDual        = 0,
            .AEK_POW_BMS63CHAIN_iso_portISOFreqDual   = 0,
            .AEK_POW_BMS63CHAIN_iso_pinISOFreqDual    = 0,
            .AEK_POW_BMS63CHAIN_iso_portBNEDual       = 0,
            .AEK_POW_BMS63CHAIN_iso_pinBNEDual        = 0,
            .AEK_POW_BMS63CHAIN_iso_portTXEnDual      = 0,
            .AEK_POW_BMS63CHAIN_iso_pinTXEnDual       = 0,
            .AEK_POW_BMS63CHAIN_iso_portTXAmpDual     = 0,
            .AEK_POW_BMS63CHAIN_iso_pinTXAmpDual      = 0,

            .AEK_POW_BMS63CHAIN_isofreqSel =
                AEK_POW_BMS63CHAIN_ISO_FREQ_LOW,
            .AEK_POW_BMS63CHAIN_isoTxAmp =
                AEK_POW_BMS63CHAIN_ISO_TXAMPLOW,
            .AEK_POW_BMS63CHAIN_outResTx =
                AEK_POW_BMS63CHAIN_ISO_RDIFF1
        },

        .AEK_POW_BMS63CHAIN_node_num = 1,
        .AEK_POW_BMS63CHAIN_node = AEK_POW_BMS63CHAIN_chainNode0
    }
};


void AEK_POW_BMS63CHAIN_spi_clkChain0(SPIDriver *spip){
	(void) *spip;
	if(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_spi_chainConf.AEK_POW_BMS63CHAIN_spi_txRxSts == AEK_POW_BMS63CHAIN_SPI_TX_WAITING){
		AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_spi_chainConf.AEK_POW_BMS63CHAIN_spi_txRxSts = AEK_POW_BMS63CHAIN_SPI_RX_WAITING;
	}
	else if(AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_spi_chainConf.AEK_POW_BMS63CHAIN_spi_txRxSts == AEK_POW_BMS63CHAIN_SPI_RX_WAITING){
		AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_spi_chainConf.AEK_POW_BMS63CHAIN_spi_txRxSts = AEK_POW_BMS63CHAIN_SPI_RX_COMPLETED;
	}
	else{}
}




