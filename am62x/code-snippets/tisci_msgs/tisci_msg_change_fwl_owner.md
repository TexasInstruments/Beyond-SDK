## TISCI_MSG_CHANGE_FWL_OWNER

### Code Snippet
```C
/*
 * fwl_id      : Firewall Id
 * region      : Index of the region in the firewall
 * owner_index : Non-secure Host Id of the core
 */
uint16_t fwl_id = 1;
uint16_t region = 0;
uint8_t owner_index = TISCI_HOST_ID_MAIN_0_R5_1;

/* Request & Response structures for TISCI_MSG_CHANGE_FWL_OWNER */
struct tisci_msg_fwl_change_owner_info_req tisci_msg_req   = {0};
struct tisci_msg_fwl_change_owner_info_resp tisci_msg_resp = {0};

/* Fill the request structure fields */
tisci_msg_req.fwl_id = fwl_id;
tisci_msg_req.region = region;
tisci_msg_req.owner_index = owner_index;

int32_t status = Sciclient_firewallChangeOwnerInfo(&tisci_msg_req, &tisci_msg_resp, SystemP_WAIT_FOREVER);

if(status == SystemP_SUCCESS) {
    DebugP_log("Firewall ownership changed successfully\r\n");
    DebugP_log("---------------------------------------\r\n");
    DebugP_log("fwl_id: 0x%x\r\n", tisci_msg_resp.fwl_id);
    DebugP_log("region: 0x%x\r\n", tisci_msg_resp.region);
    DebugP_log("owner_index: 0x%x\r\n", tisci_msg_resp.owner_index);
} else {
    DebugP_logError("TISCI_MSG_CHANGE_FWL_OWNER request failed!!!\r\n");
}
```
