## TISCI_MSG_SET_FWL_REGION

### Code Snippet
```C
uint16_t fwl_id            = 1;
uint16_t region            = 0;
/*
 * Firewall control register layout
 * +----------+------------+------------+----------+------+--------+
 * |  31:10   |    9:9     |    8:8     |   7:5    | 4:4  |  3:0   |
 * +----------+------------+------------+----------+------+--------+
 * | RESERVED | CACHE_MODE | BACKGROUND | RESERVED | LOCK | ENABLE |
 * +----------+------------+------------+----------+------+--------+
 * 
 * CACHE_MODE : Cache mode for region. Set to 1 to check cache permissions.
 *              Set to 0 to ignore cache permissions.
 * BACKGROUND : Background enable for region. There can be 1 backgroun region per
 *              FW and foreground regions can have overlapping addresses only
 *              with the background region.
 * LOCK       : Lock region. Once set region values cannot be modified.
 * ENABLE     : Enable region. A value of 0xA enables, others disable.
 */
uint32_t control           = 0x30A;
uint32_t n_permission_regs = 3;
/*
 * Firewall region permission register layout
 * +----------+---------+------------------+------------------+---------------+---------------+
 * |  31:24   |  23:16  |      15:12       |       11:8       |      7:4      |      3:0      |
 * +----------+---------+------------------+------------------+---------------+---------------+
 * | RESERVED | PRIV_ID | NONSEC_USER_DCRW | NONSEC_SUPV_DCRW | SEC_USER_DCRW | SEC_SUPV_DCRW |
 * +----------+---------+------------------+------------------+---------------+---------------+
 *
 * PRIV_ID     : Privilege Id
 *               https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/am62x/firewalls.html#list-of-priv-ids
 * NONSEC_USER : Non-secure User
 * NONSEC_SUPV : Non-secure Supervisor
 * SEC_USER    : Secure User
 * SEC_SUPV    : Secure Supervisor
 * DCRW        : Debug, Cache, Read, Write
 */
uint32_t permissions[] = { 0xC3FFFF, 0xC3FFFF, 0xC3FFFF };
uint32_t start_address = 0x80000000;
uint32_t end_address   = 0xFFFFFFFF;

/* Request & Response structures for TISCI_MSG_SET_FWL_REGION */
struct tisci_msg_fwl_set_firewall_region_req tisci_msg_req   = {0};
struct tisci_msg_fwl_set_firewall_region_resp tisci_msg_resp = {0};

/* Fill the request structure fields */
tisci_msg_req.fwl_id            = fwl_id;
tisci_msg_req.region            = region;
tisci_msg_req.control           = control;
tisci_msg_req.n_permission_regs = n_permission_regs;
tisci_msg_req.permissions[0]    = permissions[0];
tisci_msg_req.permissions[1]    = permissions[1];
tisci_msg_req.permissions[2]    = permissions[2];
tisci_msg_req.start_address     = start_address;
tisci_msg_req.end_address       = end_address;

int32_t status = Sciclient_firewallSetRegion(&tisci_msg_req, &tisci_msg_resp, SystemP_WAIT_FOREVER);

if(status == SystemP_SUCCESS) {
    DebugP_log("Firewall permissions set successfully\r\n");
    /* Response structure is empty for TISCI_MSG_SET_FWL_REGION */
} else {
    DebugP_logError("TISCI_MSG_SET_FWL_REGION request failed!!!\r\n");
}
```
