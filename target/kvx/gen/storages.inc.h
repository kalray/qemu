struct Storages {
    uint64_t kv3_AESPC[1]; /* Arithmetic Exception Saved PC */
    uint64_t kv3_PM0[1]; /* Performance Monitor 0 */
    uint64_t kv3_PM1[1]; /* Performance Monitor 1 */
    uint64_t kv3_PM2[1]; /* Performance Monitor 2 */
    uint64_t kv3_PM3[1]; /* Performance Monitor 3 */
    uint64_t kv3_PMSA[1]; /* Performance Monitor Saved Address */
    uint64_t kv3_T0V[1]; /* Timer 0 value */
    uint64_t kv3_T1V[1]; /* Timer 1 value */
    uint64_t kv3_T0R[1]; /* Timer 0 reload value */
    uint64_t kv3_T1R[1]; /* Timer 1 reload value */
    uint64_t kv3_WDV[1]; /* Watchdog Value */
    uint64_t kv3_WDR[1]; /* Watchdog Reload Value */
    uint64_t kv3_MEN[1]; /* Miscellaneous External Notifications */
    uint64_t kv3_PC[1]; /* Program Counter */
    uint64_t kv3_PS[1]; /* Processing Status */
    uint64_t kv3_SPS[1]; /* Shadow Processing Status */
    uint64_t kv3_SPS_PL0[1]; /* Shadow Processing Status for Privilege Level 0 */
    uint64_t kv3_SPS_PL1[1]; /* Shadow Processing Status for Privilege Level 1 */
    uint64_t kv3_SPS_PL2[1]; /* Shadow Processing Status for Privilege Level 2 */
    uint64_t kv3_SPS_PL3[1]; /* Shadow Processing Status for Privilege Level 3 */
    uint64_t kv3_CS[1]; /* Compute Status */
    uint64_t kv3_DBA0[1]; /* Debug Breakpoint Address 0 */
    uint64_t kv3_DBA1[1]; /* Debug Breakpoint Address 1 */
    uint64_t kv3_DBA2[1]; /* Debug Breakpoint Address 2 */
    uint64_t kv3_DBA3[1]; /* Debug Breakpoint Address 3 */
    uint64_t kv3_DWA0[1]; /* Debug Watchpoint address 0 */
    uint64_t kv3_DWA1[1]; /* Debug Watchpoint address 1 */
    uint64_t kv3_DWA2[1]; /* Debug Watchpoint address 2 */
    uint64_t kv3_DWA3[1]; /* Debug Watchpoint address 3 */
    uint64_t kv3_CSIT[1]; /* Compute Status arithmetic Interrupt */
    uint64_t kv3_ES[1]; /* Exception Syndrome */
    uint64_t kv3_ES_PL0[1]; /* Exception Syndrome for Privilege Level 0 */
    uint64_t kv3_ES_PL1[1]; /* Exception Syndrome for Privilege Level 1 */
    uint64_t kv3_ES_PL2[1]; /* Exception Syndrome for Privilege Level 2 */
    uint64_t kv3_ES_PL3[1]; /* Exception Syndrome for Privilege Level 3 */
    uint64_t kv3_SID[1]; /* Coolidge V2, Stream ID */
    uint64_t kv3_SID_PL0[1]; /* Coolidge V2, Stream ID for Privilege Level 0 */
    uint64_t kv3_SID_PL1[1]; /* Coolidge V2, Stream ID for Privilege Level 1 */
    uint64_t kv3_SID_PL2[1]; /* Coolidge V2, Stream ID for Privilege Level 2 */
    uint64_t kv3_SID_PL3[1]; /* Coolidge V2, Stream ID for Privilege Level 3 */
    uint64_t kv3_IXC[1]; /* Coolidge V2, Inter-Extension Communications */
    uint64_t kv3_TEL[1]; /* TLB Entry Low */
    uint64_t kv3_TEH[1]; /* TLB Entry High */
    uint64_t kv3_TPCM0[1]; /* Trace PC Message 0 */
    uint64_t kv3_TPCM1[1]; /* Trace PC Message 1 */
    uint64_t kv3_TPCM2[1]; /* Trace PC Message 2 */
    uint64_t kv3_TPCMC[1]; /* Trace PC Message Control */
    uint64_t kv3_DC[1]; /* Debug Control */
    uint64_t kv3_DCV2_0[1]; /* Debug Control watchpoint/breakpoint 0 */
    uint64_t kv3_DCV2_1[1]; /* Debug Control watchpoint/breakpoint 1 */
    uint64_t kv3_DCV2_2[1]; /* Debug Control watchpoint/breakpoint 2 */
    uint64_t kv3_DCV2_3[1]; /* Debug Control watchpoint/breakpoint 3 */
    uint64_t kv3_SRS[512]; /* System Register Storage */
    uint64_t kv3_GRS[64]; /* General Register Storage */
    uint64_t kv3_XRS[256]; /* Extension Register Storage */
    uint64_t kv3_NPC[1]; /* Program Counter of Next Bundle */
    uint64_t kv3_TCR[1]; /* Timer Control Register */
    uint64_t kv3_PMC[1]; /* Performance Monitor Control Register */
    uint64_t kv3_PCR[1]; /* Processing Identification Register */
    uint64_t kv3_SYO[1]; /* SYscalls Owners */
    uint64_t kv3_HTO[1]; /* Hardware Trap Owners */
    uint64_t kv3_ITO[1]; /* Interrupt Owners */
    uint64_t kv3_ILE[1]; /* Interrupt Owners */
    uint64_t kv3_ILL[1]; /* Interrupt Owners */
    uint64_t kv3_ILR[1]; /* Interrupt Owners */
    uint64_t kv3_IPE[1]; /* Inter Process Event */
    uint64_t kv3_DO[1]; /* Debug Owners */
    uint64_t kv3_MO[1]; /* Miscellaneous Owners */
    uint64_t kv3_PSO[1]; /* Processor Status register Owners */
    uint64_t kv3_MMC[1]; /* Memory Management Control */
    uint64_t kv3_MES[1]; /* Memory Error Status */
    uint64_t kv3_WS[1]; /* Wake-up Status */
    uint64_t v2_SID[1]; /* Coolidge 1, Stream ID */
    uint64_t v2_SID_PL0[1]; /* Coolidge 1, Stream ID for Privilege Level 0 */
    uint64_t v2_SID_PL1[1]; /* Coolidge 1, Stream ID for Privilege Level 1 */
    uint64_t v2_SID_PL2[1]; /* Coolidge 1, Stream ID for Privilege Level 2 */
    uint64_t v2_SID_PL3[1]; /* Coolidge 1, Stream ID for Privilege Level 3 */
    uint64_t v2_IXC[1]; /* Coolidge 1, Inter-Extension Communications */
    uint64_t v2_DC[1]; /* Debug Control */
} storages;

