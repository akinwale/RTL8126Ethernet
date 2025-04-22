/* RTL8126Hardware.cpp -- RTL8126 hardware initialzation methods.
*
* Copyright (c) 2020 Laura Müller <laura-mueller@uni-duesseldorf.de>
* Copyright (c) 2025 Akinwale Ariwodola <akinwale@gmail.com>
* All rights reserved.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation; either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* Driver for Realtek RTL8126 PCIe 5GB ethernet controllers.
*
* This driver is based on Realtek's r8126 Linux driver (10.015.00).
*/

#include "RTL8126Ethernet.hpp"

#pragma mark --- hardware initialization methods ---

bool RTL8126::initPCIConfigSpace(IOPCIDevice *provider)
{
    DebugLog("RTL8126::initPCIConfigSpace");
    
    IOByteCount pmCapOffset;
    UInt32 pcieLinkCap;
    UInt16 pcieLinkCtl;
    UInt16 cmdReg;
    UInt16 pmCap;
    bool result = false;
    
    /* Get vendor and device info. */
    pciDeviceData.vendor = provider->configRead16(kIOPCIConfigVendorID);
    pciDeviceData.device = provider->configRead16(kIOPCIConfigDeviceID);
    pciDeviceData.subsystem_vendor = provider->configRead16(kIOPCIConfigSubSystemVendorID);
    pciDeviceData.subsystem_device = provider->configRead16(kIOPCIConfigSubSystemID);
    
    /* Setup power management. */
    if (provider->extendedFindPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset)) {
        pmCap = provider->extendedConfigRead16(pmCapOffset + kIOPCIPMCapability);
        DebugLog("PCI power management capabilities: 0x%x.\n", pmCap);
        
        if (pmCap & kPCIPMCPMESupportFromD3Cold) {
            wolCapable = true;
            DebugLog("PME# from D3 (cold) supported.\n");
        }
        pciPMCtrlOffset = pmCapOffset + kIOPCIPMControl;
    } else {
        IOLog("PCI power management unsupported.\n");
    }
    provider->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
    
    /* Get PCIe link information. */
    if (provider->extendedFindPCICapability(kIOPCIPCIExpressCapability, &pcieCapOffset)) {
        pcieLinkCap = provider->configRead32(pcieCapOffset + kIOPCIELinkCapability);
        pcieLinkCtl = provider->configRead16(pcieCapOffset + kIOPCIELinkControl);
        DebugLog("PCIe link capabilities: 0x%08x, link control: 0x%04x.\n", pcieLinkCap, pcieLinkCtl);
        
        if (linuxData.configASPM == 0) {
            IOLog("Disable PCIe ASPM.\n");
            provider->setASPMState(this, 0);
        } else {
            IOLog("Warning: Enable PCIe ASPM.\n");
            provider->setASPMState(this, kIOPCIELinkCtlASPM | kIOPCIELinkCtlClkPM);
            linuxData.configASPM = 1;
        }
    }
    /* Enable the device. */
    cmdReg    = provider->configRead16(kIOPCIConfigCommand);
    cmdReg  &= ~kIOPCICommandIOSpace;
    cmdReg    |= (kIOPCICommandBusMaster | kIOPCICommandMemorySpace | kIOPCICommandMemWrInvalidate);
    provider->configWrite16(kIOPCIConfigCommand, cmdReg);
    //provider->configWrite8(kIOPCIConfigLatencyTimer, 0x40);
    
    baseMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress2, kIOMapInhibitCache);
    
    if (!baseMap) {
        IOLog("region #2 not an MMIO resource, aborting.\n");
        goto done;
    }
    baseAddr = reinterpret_cast<volatile void *>(baseMap->getVirtualAddress());
    linuxData.mmio_addr = baseAddr;
    result = true;
    
done:
    return result;
}

IOReturn RTL8126::setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    DebugLog("RTL8126::setPowerStateWakeAction");
    
    RTL8126 *ethCtlr = OSDynamicCast(RTL8126, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;
    
    if (ethCtlr && ethCtlr->pciPMCtrlOffset) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        val16 |= kPCIPMCSPowerStateD0;
        
        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

IOReturn RTL8126::setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    DebugLog("RTL8126::setPowerStateSleepAction");
    
    RTL8126 *ethCtlr = OSDynamicCast(RTL8126, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;

    if (ethCtlr && ethCtlr->pciPMCtrlOffset) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);

        if (ethCtlr->wolActive)
            val16 |= (kPCIPMCSPMEStatus | kPCIPMCSPMEEnable | kPCIPMCSPowerStateD3);
        else
            val16 |= kPCIPMCSPowerStateD3;
        
        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

/*
 * These functions have to be rewritten after every update
 * of the underlying Linux sources.
 */

IOReturn RTL8126::identifyChip()
{
    DebugLog("RTL8126::identifyChip");
    
    struct rtl8126_private *tp = &linuxData;
    IOReturn result = kIOReturnSuccess;
    UInt32 reg, val32;
    UInt32 version;

    val32 = ReadReg32(TxConfig);
    reg = val32 & 0x7c800000;
    version = val32 & 0x00700000;

    switch (reg) {
        case 0x64800000:
            if (version == 0x00000000) {
                tp->mcfg = CFG_METHOD_1;
            } else if (version == 0x100000) {
                tp->mcfg = CFG_METHOD_2;
            } else if (version == 0x200000) {
                tp->mcfg = CFG_METHOD_3;
            } else {
                tp->mcfg = CFG_METHOD_3;
                tp->HwIcVerUnknown = true;
            }
            tp->efuse_ver = EFUSE_SUPPORT_V4;
            break;

        default:
            tp->mcfg = CFG_METHOD_DEFAULT;
            tp->HwIcVerUnknown = true;
            tp->efuse_ver = EFUSE_NOT_SUPPORT;
            result = kIOReturnError;
            break;
    }
    return result;
}

bool RTL8126::initRTL8126()
{
    DebugLog("RTL8126::initRTL8126");
    
    struct rtl8126_private *tp = &linuxData;
    UInt32 i;
    UInt8 macAddr[MAC_ADDR_LEN];
    bool result = false;
    
    /* Identify chip attached to board. */
    if(identifyChip()) {
        IOLog("Unsupported chip found. Aborting...\n");
        goto done;
    }
    
    /* Setup EEE support. */
    //tp->eee_adv_t = eeeCap = (MDIO_EEE_100TX | MDIO_EEE_1000T);
    
    //tp->phy_reset_enable = rtl8126_xmii_reset_enable;
    //tp->phy_reset_pending = rtl8126_xmii_reset_pending;

    //tp->max_jumbo_frame_size = rtl_chip_info[tp->chipset].jumbo_frame_sz;
    
    tp->HwSuppDashVer = 0;

    //tp->AllowAccessDashOcp = rtl8126_is_allow_access_dash_ocp(tp);
    
    tp->HwPkgDet = rtl8126_mac_ocp_read(tp, 0xDC00);
    tp->HwPkgDet = (tp->HwPkgDet >> 3) & 0x07;
    
    tp->HwSuppNowIsOobVer = 1;

    tp->HwPcieSNOffset = 0x174;

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8126_get_realwow_hw_version(dev);
#endif //ENABLE_REALWOW_SUPPORT
    
    // force tp->DASH to 0
    tp->DASH = 0;

    if (linuxData.configASPM) {
        tp->org_pci_offset_99 = csiFun0ReadByte(0x99);
        tp->org_pci_offset_99 &= ~(BIT_5|BIT_6);

        tp->org_pci_offset_180 = csiFun0ReadByte(0x22c);
    }
    tp->org_pci_offset_80 = pciDevice->configRead8(0x80);
    tp->org_pci_offset_81 = pciDevice->configRead8(0x81);
    tp->use_timer_interrupt = true;

    tp->HwSuppMaxPhyLinkSpeed = 5000;

    if (tp->mcfg == CFG_METHOD_DEFAULT) {
        tp->use_timer_interrupt = false;
    }
    
    tp->ShortPacketSwChecksum = true;
    tp->UseSwPaddingShortPkt = true;
    
    tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V3;

    tp->HwSuppLinkChgWakeUpVer = 3;

    tp->HwSuppD0SpeedUpVer = 1;

    tp->HwSuppCheckPhyDisableModeVer = 3;
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
            tp->HwSuppTxNoCloseVer = 4;
            break;
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppTxNoCloseVer = 5;
            break;
    }
    
    switch (tp->HwSuppTxNoCloseVer) {
        case 5:
        case 6:
            tp->MaxTxDescPtrMask = MAX_TX_NO_CLOSE_DESC_PTR_MASK_V4;
            break;
        case 4:
            tp->MaxTxDescPtrMask = MAX_TX_NO_CLOSE_DESC_PTR_MASK_V3;
            break;
        case 3:
            tp->MaxTxDescPtrMask = MAX_TX_NO_CLOSE_DESC_PTR_MASK_V2;
            break;
        default:
            //tx_no_close_enable = 0;
            break;
    }
    
    //if (tp->HwSuppTxNoCloseVer > 0 && tx_no_close_enable == 1)
    //    tp->EnableTxNoClose = TRUE;

    switch (tp->mcfg) {
        case CFG_METHOD_1:
            tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_1;
            break;
        case CFG_METHOD_2:
            tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_2;
            break;
        case CFG_METHOD_3:
            tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_3;
            break;
    }
    
    if (tp->HwIcVerUnknown) {
        tp->NotWrRamCodeToMicroP = true;
        tp->NotWrMcuPatchCode = true;
    }

    tp->HwSuppMacMcuVer = 2;

    tp->MacMcuPageSize = RTL8126_MAC_MCU_PAGE_SIZE;

    tp->HwSuppNumTxQueues = 2;
    tp->HwSuppNumRxQueues = 4;
    
    //init interrupt
    switch (tp->mcfg) {
        case CFG_METHOD_1:
            tp->HwSuppIsrVer = 2;
            break;
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppIsrVer = 3;
            break;
        default:
            tp->HwSuppIsrVer = 1;
            break;
    }

    tp->HwCurrIsrVer = tp->HwSuppIsrVer;
    /*
    if (tp->HwCurrIsrVer > 1) {
        if (!(tp->features & RTL_FEATURE_MSIX) || tp->irq_nvecs < tp->min_irq_nvecs)
                tp->HwCurrIsrVer = 1;
    }

    // interrupt mask
    if (tp->HwCurrIsrVer == 5) {
        tp->intr_mask = ISRIMR_V5_LINKCHG | ISRIMR_V5_TOK_Q0;
        if (tp->num_tx_rings > 1)
                tp->intr_mask |= ISRIMR_V5_TOK_Q1;
        for (i = 0; i < tp->num_rx_rings; i++)
                tp->intr_mask |= ISRIMR_V5_ROK_Q0 << i;
    } else if (tp->HwCurrIsrVer == 4) {
        tp->intr_mask = ISRIMR_V4_LINKCHG;
        for (i = 0; i < tp->num_rx_rings; i++)
                tp->intr_mask |= ISRIMR_V4_ROK_Q0 << i;
    } else if (tp->HwCurrIsrVer == 3) {
        tp->intr_mask = ISRIMR_V2_LINKCHG;
        for (i = 0; i < max(tp->num_tx_rings, tp->num_rx_rings); i++)
                tp->intr_mask |= ISRIMR_V2_ROK_Q0 << i;
    } else if (tp->HwCurrIsrVer == 2) {
        tp->intr_mask = ISRIMR_V2_LINKCHG | ISRIMR_TOK_Q0;
        if (tp->num_tx_rings > 1)
                tp->intr_mask |= ISRIMR_TOK_Q1;

        for (i = 0; i < tp->num_rx_rings; i++)
                tp->intr_mask |= ISRIMR_V2_ROK_Q0 << i;
    } else {
        tp->intr_mask = LinkChg | RxDescUnavail | TxOK | RxOK | SWInt;
        tp->timer_intr_mask = LinkChg | PCSTimeout;

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH) {
                if (HW_DASH_SUPPORT_TYPE_3(tp)) {
                        tp->timer_intr_mask |= (ISRIMR_DASH_INTR_EN | ISRIMR_DASH_INTR_CMAC_RESET);
                        tp->intr_mask |= (ISRIMR_DASH_INTR_EN | ISRIMR_DASH_INTR_CMAC_RESET);
                }
        }
#endif
    }*/
    
    // rtl8126_setup_mqs_reg(tp); ?
    
    tp->HwSuppPtpVer = 2;

    switch (tp->mcfg) {
    case CFG_METHOD_1:
        tp->HwSuppIntMitiVer = 4;
        break;
    case CFG_METHOD_2:
    case CFG_METHOD_3:
        tp->HwSuppIntMitiVer = 5;
        break;
    }

    tp->HwSuppTcamVer = 2;

    tp->TcamNotValidReg = TCAM_NOTVALID_ADDR_V2;
    tp->TcamValidReg = TCAM_VALID_ADDR_V2;
    tp->TcamMaAddrcOffset = TCAM_MAC_ADDR_V2;
    tp->TcamVlanTagOffset = TCAM_VLAN_TAG_V2;

    tp->HwSuppExtendTallyCounterVer = 1;

    switch (tp->mcfg) {
        case CFG_METHOD_1:
            tp->HwSuppRxDescType = RX_DESC_RING_TYPE_3;
            break;
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppRxDescType = RX_DESC_RING_TYPE_4;
            break;
        default:
            tp->HwSuppRxDescType = RX_DESC_RING_TYPE_1;
            break;
    }

    tp->InitRxDescType = RX_DESC_RING_TYPE_1;
    tp->RxDescLength = RX_DESC_LEN_TYPE_1;
    switch (tp->HwSuppRxDescType) {
        case RX_DESC_RING_TYPE_3:
            if (tp->EnableRss) {
                    tp->InitRxDescType = RX_DESC_RING_TYPE_3;
                    tp->RxDescLength = RX_DESC_LEN_TYPE_3;
            }
            break;
        case RX_DESC_RING_TYPE_4:
            if (tp->EnableRss) {
                    tp->InitRxDescType = RX_DESC_RING_TYPE_4;
                    tp->RxDescLength = RX_DESC_LEN_TYPE_4;
            }
            break;
    }

    //tp->rtl8126_rx_config = rtl_chip_info[tp->chipset].RCR_Cfg;
    //if (tp->InitRxDescType == RX_DESC_RING_TYPE_3)
    //    tp->rtl8126_rx_config |= EnableRxDescV3;
    //else if (tp->InitRxDescType == RX_DESC_RING_TYPE_4)
    //    tp->rtl8126_rx_config &= ~EnableRxDescV4_1;

    tp->NicCustLedValue = RTL_R16(tp, CustomLED);

    tp->wol_opts = rtl8126_get_hw_wol(tp);
    tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;

    //rtl8126_set_link_option(tp, autoneg_mode, speed_mode, duplex_mode,
    //                        rtl8126_fc_full);

    tp->max_jumbo_frame_size = rtl_chip_info[tp->chipset].jumbo_frame_sz;
    

    //tp->eee_enabled = eee_enable;
    //tp->eee_adv_t = MDIO_EEE_1000T | MDIO_EEE_100TX;
    
    exitOOB();
    rtl8126_hw_init(tp);
    rtl8126_nic_reset(tp);
    
    /* Get production from EEPROM */
    rtl8126_eeprom_type(tp);

    if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
        rtl8126_set_eeprom_sel_low(tp);

    for (i = 0; i < MAC_ADDR_LEN; i++)
        macAddr[i] = ReadReg8(MAC0 + i);

    if(tp->mcfg == CFG_METHOD_1 ||
        tp->mcfg == CFG_METHOD_2 ||
        tp->mcfg == CFG_METHOD_3) {
            *(UInt32*)&macAddr[0] = ReadReg32(BACKUP_ADDR0_8125);
            *(UInt16*)&macAddr[4] = ReadReg16(BACKUP_ADDR1_8125);
    }
    

    if (is_valid_ether_addr((UInt8 *) macAddr)) {
        rtl8126_rar_set(tp, macAddr);
    } else {
        IOLog("Using fallback MAC.\n");
        rtl8126_rar_set(tp, fallBackMacAddr.bytes);
    }
    for (i = 0; i < MAC_ADDR_LEN; i++) {
        currMacAddr.bytes[i] = ReadReg8(MAC0 + i);
        origMacAddr.bytes[i] = currMacAddr.bytes[i]; /* keep the original MAC address */
    }
    IOLog("%s: (Chipset %d), %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
          rtl_chip_info[tp->chipset].name, tp->chipset,
          origMacAddr.bytes[0], origMacAddr.bytes[1],
          origMacAddr.bytes[2], origMacAddr.bytes[3],
          origMacAddr.bytes[4], origMacAddr.bytes[5]);
    
    //tp->fw_name = rtl_chip_fw_infos[tp->mcfg].fw_name;
    
    tp->cp_cmd = (ReadReg16(CPlusCmd) | RxChkSum);
    
    intrMaskRxTx = (SYSErr | LinkChg | RxDescUnavail | TxOK | RxOK);
    intrMaskTimer = (SYSErr | LinkChg | RxDescUnavail | PCSTimeout | RxOK);
    intrMaskPoll = (SYSErr | LinkChg);
    intrMask = intrMaskRxTx;
    
    /* Get the RxConfig parameters. */
    rxConfigReg = rtl_chip_info[tp->chipset].RCR_Cfg;
    rxConfigMask = rtl_chip_info[tp->chipset].RxConfigMask;
  
    /* Reset the tally counter. */
    WriteReg32(CounterAddrHigh, (statPhyAddr >> 32));
    WriteReg32(CounterAddrLow, (statPhyAddr & 0x00000000ffffffff) | CounterReset);

    rtl8126_disable_rxdvgate(tp);
    
#ifdef DEBUG
    
    if (wolCapable)
        IOLog("Device is WoL capable.\n");
    
#endif
    
    result = true;
    
done:
    return result;
}

void RTL8126::enableRTL8126()
{
    struct rtl8126_private *tp = &linuxData;
    
    setLinkStatus(kIONetworkLinkValid);
    
    intrMask = intrMaskRxTx;
    clear_bit(__POLL_MODE, &stateFlags);
    
    exitOOB();
    rtl8126_hw_init(tp);
    rtl8126_nic_reset(tp);
    rtl8126_powerup_pll(tp);
    rtl8126_hw_ephy_config(tp);
    configPhyHardware();
    setupRTL8126();
    
    setPhyMedium();
}

void RTL8126::disableRTL8126()
{
    struct rtl8126_private *tp = &linuxData;
    
    /* Disable all interrupts by clearing the interrupt mask. */
    WriteReg32(IMR0_8125, 0);
    WriteReg16(IntrStatus, ReadReg16(IntrStatus));

    rtl8126_nic_reset(tp);
    hardwareD3Para();
    powerDownPLL();
    
    if (test_and_clear_bit(__LINK_UP, &stateFlags)) {
        setLinkStatus(kIONetworkLinkValid);
        IOLog("Link down on en%u\n", netif->getUnitNumber());
    }
}

/* Reset the NIC in case a tx deadlock or a pci error occurred. timerSource and txQueue
 * are stopped immediately but will be restarted by checkLinkStatus() when the link has
 * been reestablished.
 */

void RTL8126::restartRTL8126()
{
    /* Stop output thread and flush txQueue */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    clear_bit(__LINK_UP, &stateFlags);
    setLinkStatus(kIONetworkLinkValid);
    
    /* Reset NIC and cleanup both descriptor rings. */
    rtl8126_nic_reset(&linuxData);
/*
    if (rxInterrupt(netif, kNumRxDesc, NULL, NULL))
        netif->flushInputQueue();
*/
    clearRxTxRings();

    /* Reinitialize NIC. */
    enableRTL8126();
}

void RTL8126::setupRTL8126()
{
    IOLog("RTL8126::setupRTL8126");
    
    struct rtl8126_private *tp = &linuxData;
    UInt32 i;
    UInt16 mac_ocp_data;
    
    // disable_rx_packet_filter
    WriteReg32(RxConfig, ReadReg32(RxConfig) & ~(AcceptErr | AcceptRunt |AcceptBroadcast | AcceptMulticast | AcceptMyPhys |  AcceptAllPhys));
    WriteReg32(TxConfig, (TX_DMA_BURST_unlimited << TxDMAShift) |
               (InterFrameGap << TxInterFrameGapShift));
    
    rtl8126_nic_reset(tp);
    
    WriteReg8(Cfg9346, ReadReg8(Cfg9346) | Cfg9346_Unlock);
    rtl8126_enable_force_clkreq(tp, 0);
    rtl8126_enable_aspm_clkreq_lock(tp, 0);

    rtl8126_set_eee_lpi_timer(tp);
    
    //keep magic packet only
    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xC0B6);
    mac_ocp_data &= BIT_0;
    rtl8126_mac_ocp_write(tp, 0xC0B6, mac_ocp_data);

    /* Fill tally counter address. */
    WriteReg32(CounterAddrHigh, (statPhyAddr >> 32));
    WriteReg32(CounterAddrLow, (statPhyAddr & 0x00000000ffffffff));

    switch (tp->HwSuppExtendTallyCounterVer) {
        case 1:
            rtl8126_set_mac_ocp_bit(tp, 0xEA84, (BIT_1 | BIT_0));
            break;
    }

    /* Setup the descriptor rings. */
    txTailPtr0 = txClosePtr0 = 0;
    txNextDescIndex = txDirtyDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    rxNextDescIndex = 0;
    
    WriteReg32(TxDescStartAddrLow, (txPhyAddr & 0x00000000ffffffff));
    WriteReg32(TxDescStartAddrHigh, (txPhyAddr >> 32));
    WriteReg32(RxDescAddrLow, (rxPhyAddr & 0x00000000ffffffff));
    WriteReg32(RxDescAddrHigh, (rxPhyAddr >> 32));

    /* Set DMA burst size and Interframe Gap Time */
    WriteReg32(TxConfig, (TX_DMA_BURST_unlimited << TxDMAShift) |
        (InterFrameGap << TxInterFrameGapShift));

    if (tp->EnableTxNoClose)
        WriteReg32(TxConfig, (ReadReg32(TxConfig) | BIT_6));

    //rtl8126_disable_double_vlan(tp);

    //rtl8126_set_l1_l0s_entry_latency(tp);

    //rtl8126_set_mrrs(tp);

    rtl8126_disable_l1_timeout(tp);

#ifdef ENABLE_RSS_SUPPORT
    rtl8126_config_rss(tp);
#else
    WriteReg32(RSS_CTRL_8125, 0x00);
#endif
    //rtl8126_set_rx_q_num(tp, rtl8126_tot_rx_rings(tp));

    WriteReg8(Config1, ReadReg8(Config1) & ~0x10);

    rtl8126_mac_ocp_write(tp, 0xC140, 0xFFFF);
    rtl8126_mac_ocp_write(tp, 0xC142, 0xFFFF);

    //new tx desc format
    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEB58);
    if (tp->mcfg == CFG_METHOD_2 || tp->mcfg == CFG_METHOD_3)
        mac_ocp_data &= ~(BIT_0 | BIT_1);
    mac_ocp_data |= (BIT_0);
    rtl8126_mac_ocp_write(tp, 0xEB58, mac_ocp_data);

    if (tp->HwSuppRxDescType == RX_DESC_RING_TYPE_4) {
        if (tp->InitRxDescType == RX_DESC_RING_TYPE_4)
            WriteReg8(0xd8, ReadReg8(0xd8) | EnableRxDescV4_0);
        else
            WriteReg8(0xd8, ReadReg8(0xd8) & ~EnableRxDescV4_0);
    }

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE614);
    mac_ocp_data &= ~(BIT_10 | BIT_9 | BIT_8);
    mac_ocp_data |= ((4 & 0x07) << 8);
    rtl8126_mac_ocp_write(tp, 0xE614, mac_ocp_data);

    //rtl8126_set_tx_q_num(tp, rtl8126_tot_tx_rings(tp));

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE63E);
    mac_ocp_data &= ~(BIT_5 | BIT_4);
    mac_ocp_data |= ((0x02 & 0x03) << 4);
    rtl8126_mac_ocp_write(tp, 0xE63E, mac_ocp_data);

    rtl8126_enable_mcu(tp, 0);
    rtl8126_enable_mcu(tp, 1);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xC0B4);
    mac_ocp_data |= (BIT_3 | BIT_2);
    rtl8126_mac_ocp_write(tp, 0xC0B4, mac_ocp_data);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEB6A);
    mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
    mac_ocp_data |= (BIT_5 | BIT_4 | BIT_1 | BIT_0);
    rtl8126_mac_ocp_write(tp, 0xEB6A, mac_ocp_data);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEB50);
    mac_ocp_data &= ~(BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5);
    mac_ocp_data |= (BIT_6);
    rtl8126_mac_ocp_write(tp, 0xEB50, mac_ocp_data);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE056);
    mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
    //mac_ocp_data |= (BIT_4 | BIT_5);
    rtl8126_mac_ocp_write(tp, 0xE056, mac_ocp_data);

    WriteReg8(TDFNR, 0x10);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE040);
    mac_ocp_data &= ~(BIT_12);
    rtl8126_mac_ocp_write(tp, 0xE040, mac_ocp_data);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEA1C);
    mac_ocp_data &= ~(BIT_1 | BIT_0);
    mac_ocp_data |= (BIT_0);
    rtl8126_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);

    rtl8126_mac_ocp_write(tp, 0xE0C0, 0x4000);

    rtl8126_set_mac_ocp_bit(tp, 0xE052, (BIT_6 | BIT_5));
    rtl8126_clear_mac_ocp_bit(tp, 0xE052, BIT_3 | BIT_7);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xD430);
    mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
    mac_ocp_data |= 0x45F;
    rtl8126_mac_ocp_write(tp, 0xD430, mac_ocp_data);

    //rtl8126_mac_ocp_write(tp, 0xE0C0, 0x4F87);
    if (!tp->DASH)
        WriteReg8(0xD0, ReadReg8(0xD0) | BIT_6 | BIT_7);
    else
        WriteReg8(0xD0, ReadReg8(0xD0) & ~(BIT_6 | BIT_7));

    rtl8126_disable_eee_plus(tp);

    mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEA1C);
    mac_ocp_data &= ~(BIT_2);
    if (tp->mcfg == CFG_METHOD_2 || tp->mcfg == CFG_METHOD_3)
        mac_ocp_data &= ~(BIT_9 | BIT_8);
    rtl8126_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);

    rtl8126_clear_tcam_entries(tp);

    WriteReg16(0x1880, ReadReg16(0x1880) & ~(BIT_4 | BIT_5));

    /* csum offload command for RTL8126 */
    //tp->tx_tcp_csum_cmd = TxTCPCS_C;
    //tp->tx_udp_csum_cmd = TxUDPCS_C;
    //tp->tx_ip_csum_cmd = TxIPCS_C;
    //tp->tx_ipv6_csum_cmd = TxIPV6F_C;

    /* config interrupt type for RTL8125B */
    //if (tp->HwSuppIsrVer > 1)
    //    rtl8126_hw_set_interrupt_type(tp, tp->HwCurrIsrVer);

    //other hw parameters
    rtl8126_hw_clear_timer_int(tp);

    rtl8126_hw_clear_int_miti(tp);

    /*if (tp->use_timer_interrupt &&
        (tp->HwCurrIsrVer > 1) &&
        (tp->HwSuppIntMitiVer > 3) &&
        (tp->features & RTL_FEATURE_MSIX)) {
            int i;
            for (i = 0; i < tp->irq_nvecs; i++)
                rtl8126_hw_set_timer_int_8125(tp, i, timer_count_v2);
    }*/

    //rtl8126_enable_exit_l1_mask(tp);

    rtl8126_mac_ocp_write(tp, 0xE098, 0xC302);

    if (aspm && (tp->org_pci_offset_99 & (BIT_2 | BIT_5 | BIT_6)))
        initPCIOffset99();
    else
        disablePCIOffset99();

    if (aspm && (tp->org_pci_offset_180 & rtl8126_get_l1off_cap_bits(tp)))
        rtl8126_init_pci_offset_180(tp);
    else
        rtl8126_disable_pci_offset_180(tp);

    tp->cp_cmd &= ~(EnableBist | Macdbgo_oe | Force_halfdup |
                Force_rxflow_en | Force_txflow_en | Cxpl_dbg_sel |
                ASF | Macdbgo_sel);

    WriteReg16(CPlusCmd, tp->cp_cmd);
    
    rtl8126_disable_rxdvgate(tp);

    /* Set receiver mode. */
    setMulticastMode(test_bit(__M_CAST, &stateFlags));
    
    //rtl8126_hw_set_rx_packet_filter(tp);

#ifdef ENABLE_DASH_SUPPORT
    if (tp->DASH && !tp->dash_printer_enabled)
            NICChkTypeEnableDashInterrupt(tp);
#endif

    WriteReg8(Cfg9346, ReadReg8(Cfg9346) & ~Cfg9346_Unlock);
    
    /* Enable all known interrupts by setting the interrupt mask. */
    //WriteReg32(IMR0_8125, intrMask);

    udelay(10);
}

void RTL8126::setPhyMedium()
{
    IOLog("RTL8126::setPhyMedium");
    
    struct rtl8126_private *tp = netdev_priv(&linuxData);
    int auto_nego = 0;
    int giga_ctrl = 0;
    int ctrl_2500 = 0;
    
    if (speed != SPEED_2500 && (speed != SPEED_1000) &&
        (speed != SPEED_100) && (speed != SPEED_10)) {
        duplex = DUPLEX_FULL;
        autoneg = AUTONEG_ENABLE;
    }
    /* Enable or disable EEE support according to selected medium. */
    /*
    if ((linuxData.eee_adv_t != 0) && (autoneg == AUTONEG_ENABLE)) {
        rtl8126_enable_eee(tp);
        DebugLog("Enable EEE support.\n");
    } else {
        rtl8126_disable_eee(tp);
        DebugLog("Disable EEE support.\n");
    }*/
    //Disable Giga Lite
    ClearEthPhyOcpBit(tp, 0xA428, BIT_9);
    ClearEthPhyOcpBit(tp, 0xA5EA, BIT_0);

    giga_ctrl = rtl8126_mdio_read(tp, MII_CTRL1000);
    giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
    
    ctrl_2500 = rtl8126_mdio_direct_read_phy_ocp(tp, 0xA5D4);
    ctrl_2500 &= ~(RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL);
    
    auto_nego = rtl8126_mdio_read(tp, MII_ADVERTISE);
    auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                   ADVERTISE_100HALF | ADVERTISE_100FULL |
                   ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

    if (autoneg == AUTONEG_ENABLE) {
        /* The default medium has been selected. */
        auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL);
        giga_ctrl |= ADVERTISE_1000FULL;
        ctrl_2500 |= RTK_ADVERTISE_2500FULL;
        ctrl_2500 |= RTK_ADVERTISE_5000FULL;
    } else if (speed == SPEED_2500) {
        ctrl_2500 |= RTK_ADVERTISE_2500FULL;
    } else if (speed == SPEED_1000) {
        if (duplex == DUPLEX_HALF) {
            giga_ctrl |= ADVERTISE_1000HALF;
        } else {
            giga_ctrl |= ADVERTISE_1000FULL;
        }
    } else if (speed == SPEED_100) {
        if (duplex == DUPLEX_HALF) {
            auto_nego |= ADVERTISE_100HALF;
        } else {
            auto_nego |=  ADVERTISE_100FULL;
        }
    } else { /* speed == SPEED_10 */
        if (duplex == DUPLEX_HALF) {
            auto_nego |= ADVERTISE_10HALF;
        } else {
            auto_nego |= ADVERTISE_10FULL;
        }
    }
    /* Set flow control support. */
    if (flowCtl == kFlowControlOn)
        auto_nego |= (ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

    tp->phy_auto_nego_reg = auto_nego;
    tp->phy_1000_ctrl_reg = giga_ctrl;

    tp->phy_2500_ctrl_reg = ctrl_2500;

    rtl8126_mdio_write(tp, 0x1f, 0x0000);
    rtl8126_mdio_write(tp, MII_ADVERTISE, auto_nego);
    rtl8126_mdio_write(tp, MII_CTRL1000, giga_ctrl);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA5D4, ctrl_2500);
    rtl8126_phy_restart_nway(tp);
    mdelay(20);

    tp->autoneg = AUTONEG_ENABLE;
    tp->speed = speed;
    tp->duplex = duplex;
}

/* Set PCI configuration space offset 0x79 to setting. */

void RTL8126::setOffset79(UInt8 setting)
{
    UInt8 deviceControl;
    
    DebugLog("setOffset79() ===>\n");
    
    if (!(linuxData.hwoptimize & HW_PATCH_SOC_LAN)) {
        deviceControl = pciDevice->configRead8(0x79);
        deviceControl &= ~0x70;
        deviceControl |= setting;
        pciDevice->configWrite8(0x79, deviceControl);
    }
    
    DebugLog("setOffset79() <===\n");
}

UInt8 RTL8126::csiFun0ReadByte(UInt32 addr)
{
    struct rtl8126_private *tp = &linuxData;
    UInt8 retVal = 0;
    
    if (tp->mcfg == CFG_METHOD_DEFAULT) {
        retVal = pciDevice->configRead8(addr);
    } else {
        UInt32 tmpUlong;
        UInt8 shiftByte;
        
        shiftByte = addr & (0x3);
        tmpUlong = rtl8126_csi_other_fun_read(&linuxData, 0, addr);
        tmpUlong >>= (8 * shiftByte);
        retVal = (UInt8)tmpUlong;
    }
    udelay(20);

    return retVal;
}

void RTL8126::csiFun0WriteByte(UInt32 addr, UInt8 value)
{
    struct rtl8126_private *tp = &linuxData;

    if (tp->mcfg == CFG_METHOD_DEFAULT) {
        pciDevice->configWrite8(addr, value);
    } else {
        UInt32 tmpUlong;
        UInt16 regAlignAddr;
        UInt8 shiftByte;
        
        regAlignAddr = addr & ~(0x3);
        shiftByte = addr & (0x3);
        tmpUlong = rtl8126_csi_other_fun_read(&linuxData, 0, regAlignAddr);
        tmpUlong &= ~(0xFF << (8 * shiftByte));
        tmpUlong |= (value << (8 * shiftByte));
        rtl8126_csi_other_fun_write(&linuxData, 0, regAlignAddr, tmpUlong );
    }
    udelay(20);
}

void RTL8126::enablePCIOffset99()
{
    struct rtl8126_private *tp = &linuxData;
    u32 csi_tmp;
    
    csiFun0WriteByte(0x99, 0x00);
    
    csi_tmp = rtl8126_mac_ocp_read(tp, 0xE032);
    csi_tmp &= ~(BIT_0 | BIT_1);
    if (tp->org_pci_offset_99 & (BIT_5 | BIT_6))
            csi_tmp |= BIT_1;
    if (tp->org_pci_offset_99 & BIT_2)
            csi_tmp |= BIT_0;
    rtl8126_mac_ocp_write(tp, 0xE032, csi_tmp);
}

void RTL8126::disablePCIOffset99()
{
    struct rtl8126_private *tp = &linuxData;

    rtl8126_mac_ocp_write(tp, 0xE032,  rtl8126_mac_ocp_read(tp, 0xE032) & ~(BIT_0 | BIT_1));

    csiFun0WriteByte(0x99, 0x00);
}

void RTL8126::initPCIOffset99()
{
    IOLog("RTL8126::initPCIOffset99");

    struct rtl8126_private *tp = &linuxData;
    u32 csi_tmp;

    switch (tp->mcfg) {
        case CFG_METHOD_1:
            rtl8126_mac_ocp_write(tp, 0xCDD0, 0x9003);
            rtl8126_set_mac_ocp_bit(tp, 0xE034, (BIT_15 | BIT_14));
            rtl8126_mac_ocp_write(tp, 0xCDD2, 0x889C);
            rtl8126_mac_ocp_write(tp, 0xCDD8, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDD4, 0x8C30);
            rtl8126_mac_ocp_write(tp, 0xCDDA, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDD6, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDDC, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDE8, 0x883E);
            rtl8126_mac_ocp_write(tp, 0xCDEA, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDEC, 0x889C);
            rtl8126_mac_ocp_write(tp, 0xCDEE, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDF0, 0x8C09);
            rtl8126_mac_ocp_write(tp, 0xCDF2, 0x9003);
            rtl8126_set_mac_ocp_bit(tp, 0xE032, BIT_14);
            rtl8126_set_mac_ocp_bit(tp, 0xE0A2, BIT_0);
            break;
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_mac_ocp_write(tp, 0xCDD0, 0x9003);
            rtl8126_set_mac_ocp_bit(tp, 0xE034, (BIT_15 | BIT_14));
            rtl8126_mac_ocp_write(tp, 0xCDD2, 0x8C09);
            rtl8126_mac_ocp_write(tp, 0xCDD8, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDD4, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDDA, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDD6, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDDC, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDE8, 0x887A);
            rtl8126_mac_ocp_write(tp, 0xCDEA, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDEC, 0x8C09);
            rtl8126_mac_ocp_write(tp, 0xCDEE, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDF0, 0x8A62);
            rtl8126_mac_ocp_write(tp, 0xCDF2, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDF4, 0x883E);
            rtl8126_mac_ocp_write(tp, 0xCDF6, 0x9003);
            rtl8126_set_mac_ocp_bit(tp, 0xE032, BIT_14);
            rtl8126_set_mac_ocp_bit(tp, 0xE0A2, BIT_0);
            break;
    }
    enablePCIOffset99();
}

void RTL8126::setPCI99_180ExitDriverPara()
{
    struct rtl8126_private *tp = &linuxData;
    
    if (tp->org_pci_offset_99 & BIT_2)
        rtl8126_issue_offset_99_event(tp);
    
    disablePCIOffset99();
}

void RTL8126::hardwareD3Para()
{
    struct rtl8126_private *tp = &linuxData;
    
    /* Set RxMaxSize register */
    WriteReg16(RxMaxSize, RX_BUF_SIZE);
    
    rtl8126_enable_force_clkreq(tp, 0);
    rtl8126_enable_aspm_clkreq_lock(tp, 0);
    
    rtl8126_disable_exit_l1_mask(tp);

    setPCI99_180ExitDriverPara();

    rtl8126_disable_rxdvgate(tp);
    rtl8126_disable_extend_tally_couter(tp);
}

UInt16 RTL8126::getEEEMode()
{
    struct rtl8126_private *tp = &linuxData;
    UInt16 eee = 0;
    UInt16 sup, adv, lpa, ena;

    if (eeeCap) {
        /* Get supported EEE. */
        sup = rtl8126_mdio_direct_read_phy_ocp(tp, 0xA5C4);
        DebugLog("EEE supported: %u\n", sup);

        /* Get advertisement EEE. */
        adv = rtl8126_mdio_direct_read_phy_ocp(tp, 0xA5D0);
        DebugLog("EEE advertised: %u\n", adv);

        /* Get LP advertisement EEE. */
        lpa = rtl8126_mdio_direct_read_phy_ocp(tp, 0xA5D2);
        DebugLog("EEE link partner: %u\n", lpa);

        ena = rtl8126_mac_ocp_read(tp, 0xE040);
        ena &= BIT_1 | BIT_0;
        DebugLog("EEE enabled: %u\n", ena);

        eee = (sup & adv & lpa);
    }
    return eee;
}
void RTL8126::exitOOB()
{
    IOLog("RTL8126::exitOOB");
    
    struct rtl8126_private *tp = &linuxData;
    UInt16 data16;
    
    WriteReg32(RxConfig, ReadReg32(RxConfig) & ~(AcceptErr | AcceptRunt | AcceptBroadcast | AcceptMulticast | AcceptMyPhys |  AcceptAllPhys));
    
    //Disable realwow  function
    rtl8126_mac_ocp_write(tp, 0xC0BC, 0x00FF);

    //rtl8126_nic_reset(dev);

    rtl8126_disable_now_is_oob(tp);

    data16 = rtl8126_mac_ocp_read(tp, 0xE8DE) & ~BIT_14;
    rtl8126_mac_ocp_write(tp, 0xE8DE, data16);
    //rtl8126_wait_ll_share_fifo_ready(dev);

    rtl8126_mac_ocp_write(tp, 0xC0AA, 0x07D0);
#ifdef ENABLE_LIB_SUPPORT
    rtl8126_mac_ocp_write(tp, 0xC0A6, 0x04E2);
#else
    rtl8126_mac_ocp_write(tp, 0xC0A6, 0x01B5);
#endif
    rtl8126_mac_ocp_write(tp, 0xC01E, 0x5555);

    //rtl8126_wait_ll_share_fifo_ready(dev);
}

void RTL8126::powerDownPLL()
{
    IOLog("RTL8126::powerDownPLL");
    
    struct rtl8126_private *tp = &linuxData;
    
    tp->check_keep_link_speed = 0;
    if (tp->wol_enabled == WOL_ENABLED || tp->DASH || tp->EnableKCPOffload) {
        int auto_nego;
        int giga_ctrl;
        u16 anlpar;
        u8 from_suspend = 0;
        
        rtl8126_set_hw_wol(tp, tp->wol_opts);
        WriteReg8(Config2, ReadReg8(Config2) | PMSTS_En);
        
        /* Enable the PME and clear the status */
        //rtl8126_set_pci_pme(tp, 1);
        
        if (rtl8126_keep_wol_link_speed(tp, from_suspend)) {
            tp->check_keep_link_speed = 1;
        } else {
            if (tp->D0SpeedUpSpeed != D0_SPEED_UP_SPEED_DISABLE) {
                //rtl8126_enable_d0_speedup(tp);
                tp->check_keep_link_speed = 1;
            }
            
            //rtl8126_set_wol_link_speed(tp);
        }
        
        WriteReg32(RxConfig, ReadReg32(RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
        
        return;
    }
    
    if (tp->DASH)
        return;
    
    rtl8126_phy_power_down(tp);
    
    if (!tp->HwIcVerUnknown)
        WriteReg8(PMCH, ReadReg8(PMCH) & ~BIT_7);
    
    WriteReg8(0xF2, ReadReg8(0xF2) & ~BIT_6);
}

void RTL8126::configPhyHardware()
{
    IOLog("RTL8126::configPhyHardware");
    
    struct rtl8126_private *tp = &linuxData;
    u64 flags;

    if (tp->resume_not_chg_speed) return;
    
    tp->phy_reset_enable(tp);
    
    if (HW_DASH_SUPPORT_TYPE_3(tp) && tp->HwPkgDet == 0x06) return;
    
    spin_lock_irqsave(&tp->phy_lock, flags);
    
    //rtl8126_set_hw_phy_before_init_phy_mcu(tp);
    
    rtl8126_init_hw_phy_mcu(tp);
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
            configPhyHardware8126a1();
            break;
        case CFG_METHOD_2:
            configPhyHardware8126a2();
            break;
        case CFG_METHOD_3:
            configPhyHardware8126a3();
            break;
    }
    
    //legacy force mode(Chap 22)
    ClearEthPhyOcpBit(tp, 0xA5B4, BIT_15);
    
    rtl8126_mdio_write(tp, 0x1F, 0x0000);
    
    /*if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
        if (tp->eee_enabled == 1)
            rtl8126_enable_eee(tp);
        else
            rtl8126_disable_eee(tp);
    }*/
    
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

void RTL8126::configPhyHardware8126a1()
{
    IOLog("RTL8126::configPhyHardware8126a1");
    
    struct rtl8126_private *tp = &linuxData;

    SetEthPhyOcpBit(tp, 0xA442, BIT_11);

    if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
        rtl8126_enable_phy_aldps(tp);
}

void RTL8126::configPhyHardware8126a2()
{
    IOLog("RTL8126::configPhyHardware8126a2");
    
    struct rtl8126_private *tp = &linuxData;

    SetEthPhyOcpBit(tp, 0xA442, BIT_11);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80BF);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0xED00);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80CD);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x1000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80D1);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0xC800);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80D4);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0xC800);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80E1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x10CC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80E5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4F0C);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8387);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x4700);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA80C,
                                          BIT_7 | BIT_6,
                                          BIT_7);


    ClearEthPhyOcpBit(tp, 0xAC90, BIT_4);
    ClearEthPhyOcpBit(tp, 0xAD2C, BIT_15);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8321);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    SetEthPhyOcpBit(tp, 0xACF8, (BIT_3 | BIT_2));
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8183);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x5900);
    SetEthPhyOcpBit(tp, 0xAD94, BIT_5);
    ClearEthPhyOcpBit(tp, 0xA654, BIT_11);
    SetEthPhyOcpBit(tp, 0xB648, BIT_14);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x839E);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x2F00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83F2);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0800);
    SetEthPhyOcpBit(tp, 0xADA0, BIT_1);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80F3);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x9900);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8126);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xC100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x893A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x8080);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8647);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xE600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x862C);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1200);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x864A);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xE600);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80A0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xBCBC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x805E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xBCBC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8056);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3077);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8058);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5A00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8098);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3077);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x809A);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5A00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8052);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3733);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8094);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3733);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x807F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C75);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x803D);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C75);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8036);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8078);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8031);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3300);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8073);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3300);


    ClearAndSetEthPhyOcpBit(tp,
                                          0xAE06,
                                          0xFC00,
                                          0x7C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89D1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0004);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FBD);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x0A00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FBE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0D09);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89CD);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0F0F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89CF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0F0F);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83A4);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83A6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83C0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83C2);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8414);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8416);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83F8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83FA);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);


    rtl8126_set_phy_mcu_patch_request(tp);

    ClearAndSetEthPhyOcpBit(tp,
                                          0xBD96,
                                          0x1F00,
                                          0x1000);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xBF1C,
                                          0x0007,
                                          0x0007);
    ClearEthPhyOcpBit(tp, 0xBFBE, BIT_15);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xBF40,
                                          0x0380,
                                          0x0280);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xBF90,
                                          BIT_7,
                                          (BIT_6 | BIT_5));
    ClearAndSetEthPhyOcpBit(tp,
                                          0xBF90,
                                          BIT_4,
                                          BIT_3 | BIT_2);

    rtl8126_clear_phy_mcu_patch_request(tp);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x843B);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x2000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x843D);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x2000);


    ClearEthPhyOcpBit(tp, 0xB516, 0x7F);


    ClearEthPhyOcpBit(tp, 0xBF80, (BIT_5 | BIT_4));


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8188);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0044);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00A8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00D6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00EC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00F6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00BC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0058);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x002A);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8015);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0800);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFD);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFF);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x7F00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFB);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE9);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0002);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FEF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x00A5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FF1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0106);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0102);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE3);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0400);


    SetEthPhyOcpBit(tp, 0xA654, BIT_11);
    ClearEthPhyOcpBit(tp, 0XA65A, (BIT_1 | BIT_0));

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xAC3A, 0x5851);
    ClearAndSetEthPhyOcpBit(tp,
                                          0XAC3C,
                                          BIT_15 | BIT_14 | BIT_12,
                                          BIT_13);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xAC42,
                                          BIT_9,
                                          BIT_8 | BIT_7 | BIT_6);
    ClearEthPhyOcpBit(tp, 0xAC3E, BIT_15 | BIT_14 | BIT_13);
    ClearEthPhyOcpBit(tp, 0xAC42, BIT_5 | BIT_4 | BIT_3);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xAC42,
                                          BIT_1,
                                          BIT_2 | BIT_0);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xAC1A, 0x00DB);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xADE4, 0x01B5);
    ClearEthPhyOcpBit(tp, 0xAD9C, BIT_11 | BIT_10);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814B);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814D);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814F);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0B00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8142);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8144);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8150);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8118);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811A);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811C);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0500);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x810F);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8111);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811D);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);

    SetEthPhyOcpBit(tp, 0xAC36, BIT_12);
    ClearEthPhyOcpBit(tp, 0xAD1C, BIT_8);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xADE8,
                                          0xFFC0,
                                          0x1400);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x864B);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x9D00);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8F97);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x003F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3F02);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x023C);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3B0A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);


    SetEthPhyOcpBit(tp, 0xAD9C, BIT_5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8122);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82C8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80EF);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82A0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0006);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x001A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03D8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0023);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0054);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0322);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x00DD);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03AB);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03DC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0027);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F9);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0012);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0001);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8018);
    SetEthPhyOcpBit(tp, 0xA438, BIT_13);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE4);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0000);

    ClearAndSetEthPhyOcpBit(tp,
                                          0xB54C,
                                          0xFFC0,
                                          0x3700);


    if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
            rtl8126_enable_phy_aldps(tp);
}

void RTL8126::configPhyHardware8126a3()
{
    DebugLog("RTL8126::configPhyHardware8126a3");
    
    struct rtl8126_private *tp = &linuxData;

    SetEthPhyOcpBit(tp, 0xA442, BIT_11);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8183);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x5900);
    SetEthPhyOcpBit(tp, 0xA654, BIT_11);
    SetEthPhyOcpBit(tp, 0xB648, BIT_14);
    SetEthPhyOcpBit(tp, 0xAD2C, BIT_15);
    SetEthPhyOcpBit(tp, 0xAD94, BIT_5);
    SetEthPhyOcpBit(tp, 0xADA0, BIT_1);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xAE06,
                                          BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10,
                                          BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8647);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xE600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8036);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8078);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89E9);
    SetEthPhyOcpBit(tp, 0xB87E, 0xFF00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFD);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFE);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0200);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFF);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0400);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8018);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x7700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8F9C);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0005);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0502);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0B00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0xD401);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FA8);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x2900);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814B);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814D);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814F);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0B00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8142);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8144);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8150);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8118);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811A);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811C);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0500);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x810F);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8111);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811D);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);


    SetEthPhyOcpBit(tp, 0xAD1C, BIT_8);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xADE8,
                                          BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6,
                                          BIT_12 | BIT_10);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x864B);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x9D00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x862C);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1200);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8566);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x003F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3F02);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x023C);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3B0A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);


    SetEthPhyOcpBit(tp, 0xAD9C, BIT_5);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8122);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82C8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80EF);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82A0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0006);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x001A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03D8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0023);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0054);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0322);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x00DD);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03AB);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03DC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0027);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F9);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0012);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0001);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);


    SetEthPhyOcpBit(tp, 0xA430, BIT_1 | BIT_0);


    ClearAndSetEthPhyOcpBit(tp,
                                          0xB54C,
                                          0xFFC0,
                                          0x3700);


    SetEthPhyOcpBit(tp, 0xB648, BIT_6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8082);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5D00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x807C);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x809D);
    ClearAndSetEthPhyOcpBit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5000);


    if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
            rtl8126_enable_phy_aldps(tp);
}
