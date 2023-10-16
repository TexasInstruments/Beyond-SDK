## TISCI_MSG_GET_FWL_REGION

### Code Snippet
```C
/*
 * fwl_id            : Firewall Id
 * region            : Index of the region in the firewall
 * n_permission_regs : Number of permission registers (max 3)
 */
uint16_t fwl_id = 1;
uint16_t region = 0;
uint32_t n_permission_regs = 3;

/* Request & Response structures for TISCI_MSG_GET_FWL_REGION */
struct tisci_msg_fwl_get_firewall_region_req fwl_req   = {0};
struct tisci_msg_fwl_get_firewall_region_resp fwl_resp = {0};

/* Fill the request structure fields */
fwl_req.fwl_id = fwl_id;
fwl_req.region = region;
fwl_req.n_permission_regs = n_permission_regs;
    
int32_t status = Sciclient_firewallGetRegion(&fwl_req, &fwl_resp, SystemP_WAIT_FOREVER);
    
if(status == SystemP_SUCCESS) {
    DebugP_log("Firewall permissions retrieved successfully\r\n");
    DebugP_log("-------------------------------------------\r\n");
    DebugP_log("fwl_id: 0x%x\r\n", fwl_resp.fwl_id);
    DebugP_log("region: 0x%x\r\n", fwl_resp.region);
    DebugP_log("n_permission_regs: 0x%x\r\n", fwl_resp.n_permission_regs);
    for(int32_t i = 0; i < fwl_resp.n_permission_regs; i++) {
        DebugP_log("permissions[%d]: 0x%x\r\n", i, fwl_resp.permissions[i]);
    }
    DebugP_log("control: 0x%x\r\n", fwl_resp.control);
    DebugP_log("start_address: 0x%x\r\n", fwl_resp.start_address);
    DebugP_log("end_address: 0x%x\r\n", fwl_resp.end_address);
} else {
    DebugP_logError("TISCI_MSG_GET_FWL_REGION request failed!!!\r\n");
}
```
