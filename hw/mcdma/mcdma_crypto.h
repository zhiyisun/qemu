#ifndef MCDMA_CRYPTO_H
#define MCDMA_CRYPTO_H

/*
 * MCDMA Crypto QEMU Backend
 *
 * A separate PCI device (vendor 0x1172, device 0x0001, class 0x1080) that
 * implements the same MCDMA descriptor-ring interface as the netdev backend
 * but processes crypto work items instead of network packets.
 *
 * The backend performs a loopback: it reads the plaintext from src_addr,
 * copies it to dst_addr, sets completion_status = SUCCESS, and advances
 * the consumed-head write-back pointer so the DPDK driver can dequeue
 * completed operations.  No actual encryption is performed – this is
 * sufficient to verify the DPDK MCDMA crypto PMD data-path end-to-end.
 *
 * PCI IDs
 * -------
 *   Vendor : 0x1172  (same as netdev)
 *   Device : 0x0001  (crypto, distinct from netdev 0x0000)
 *   Class  : 0x1080  (Encryption/Decryption controller)
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "mcdma.h"

#define TYPE_MCDMA_CRYPTO "mcdma-crypto"
OBJECT_DECLARE_SIMPLE_TYPE(McdmaCryptoState, MCDMA_CRYPTO)

/* Mirror of DPDK mcdma_crypto_hw_desc (must stay in sync with the driver) */
typedef struct McdmaCryptoHwDesc {
    uint64_t src_addr;           /* plaintext / ciphertext source    */
    uint64_t dst_addr;           /* ciphertext / plaintext dest      */
    uint64_t aad_addr;           /* additional auth data             */
    uint64_t tag_addr;           /* authentication tag               */
    uint32_t data_length;        /* bytes to process                 */
    uint32_t aad_length;
    uint32_t operation;          /* 0 = encrypt, 1 = decrypt         */
    uint32_t key_length;
    uint32_t digest_length;
    uint32_t request_id;
    uint32_t context_id;
    uint32_t completion_status;  /* written back by backend: 0=OK    */
    uint8_t  iv[16];
    uint8_t  key[32];
} QEMU_PACKED McdmaCryptoHwDesc;

QEMU_BUILD_BUG_ON(sizeof(McdmaCryptoHwDesc) != 112);

#define MCDMA_CRYPTO_STATUS_SUCCESS   0
#define MCDMA_CRYPTO_MAX_XFER_SIZE    (64 * 1024)  /* 64 KB per op */

typedef struct McdmaCryptoState {
    PCIDevice parent_obj;

    MemoryRegion bar0;
    MemoryRegion bar2;
    MemoryRegion bar4;

    uint32_t num_queues;
    McdmaQueue *tx_queues;   /* submission queues (TX CSR region, offset 0x80000) */

    QEMUBH *bh;
    bool bh_scheduled;

    uint32_t nr_vectors;
    bool msix_initialized;
} McdmaCryptoState;

#endif /* MCDMA_CRYPTO_H */
