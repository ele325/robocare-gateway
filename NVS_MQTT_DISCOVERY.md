# NVS + MQTT Discovery Implementation Guide

## ✅ Completed in main.c

### 1. NVS Initialization
- `nvs_flash_init()` with auto-erase on version mismatch
- Functions: `nvs_read_uid()`, `nvs_write_uid()`
- MAC address generation: `get_device_mac()`

### 2. Global UID Buffer
- `g_firebase_uid[128]` — holds current UID
- Loaded from NVS at startup
- Falls back to `"AUTOMATIC_DISCOVERY"` if empty

### 3. Startup Flow
```
[0/7] NVS Init
  ├─ Get Device MAC : AA:BB:CC:DD:EE:FF
  ├─ Read UID from NVS
  │  ├─ If found → use it immediately
  │  └─ If not → set to AUTOMATIC_DISCOVERY
  └─ Pass to network_manager_set_uid(g_firebase_uid)

[4/7] Network Init
  └─ network_manager_start() MUST now:
     ├─ Subscribe to "robocare/discovery"
     ├─ Subscribe to "robocare/config/<MAC>"
     └─ Publish MAC on "robocare/discovery" if UID == AUTOMATIC_DISCOVERY
```

---

## ⏳ TO-DO: Modify network_manager.c

### In network_manager.c header:
```c
// Callback type for config reception
typedef void (*uid_config_callback_t)(const char *uid);

// Set callback when UID arrives
void network_manager_set_config_callback(uid_config_callback_t cb);
```

### In network_manager.c on_message():
```c
// Handle "robocare/discovery" → publish MAC if needed
if (msg.topic == "robocare/discovery") {
    if (strcmp(g_uid, "AUTOMATIC_DISCOVERY") == 0) {
        char mac[18];
        get_device_mac(mac, sizeof(mac));
        mqtt_client.publish("robocare/discovery", 
                           "{\"mac\":\"" + mac + "\"}", qos=1);
    }
    return;
}

// Handle "robocare/config/<MAC>" → extract UID, save to NVS
if (strncmp(msg.topic, "robocare/config/", 16) == 0) {
    const char *uid = (const char*)msg.payload;  // Assume simple string
    nvs_write_uid(uid);  // Save to NVS
    network_manager_set_uid(uid);  // Update RAM
    ESP_LOGI(TAG, "✓ UID received from bridge and saved to NVS");
    return;
}
```

### In app_main() call callback:
```c
//  After network_manager.start(), if UID is still AUTOMATIC_DISCOVERY:
if (strcmp(g_firebase_uid, "AUTOMATIC_DISCOVERY") == 0) {
    ESP_LOGI(TAG, "Waiting for UID from bridge...");
    // The MQTT callback will save it to NVS when received
}
```

---

## 🔄 Backend Flow (Python Bridge)

### Phase 1: Device announces itself
```
ESP32 publishes: robocare/discovery
Payload: (currently not used, but device is listening)
```

### Phase 2: Bridge discovers and claims
```
Firebase:
  → pending_devices/<MAC> = {status: "waiting", timestamp: now}
  
User claims device in app:
  → pending_devices/<MAC> = {status: "claimed", uid: "user123", ...}
```

### Phase 3: Bridge sends UID
```
Bridge listens to pending_devices changes:
  if status == "claimed":
    publish robocare/config/<MAC> = "user123"
    
ESP32 receives on robocare/config/AA:BB:CC:DD:EE:FF:
  → Saves to NVS
  → Works immediately
  → Persists after reboot
```

---

## 📋 Integration Checklist

- [ ] Modify `network_manager.c` to handle config messages
- [ ] Add `uid_config_callback_t` callback mechanism
- [ ] Test NVS read/write with actual ESP32
- [ ] Verify bridge publishes on correct topic
- [ ] Test device restart—UID should persist
- [ ] Test new device discovery flow end-to-end

---

## 🔗 Topic Reference (Aligned with Python Bridge)

| Topic | Direction | Payload | Purpose |
|-------|-----------|---------|---------|
| `robocare/discovery` | ESP → Bridge | - | Device announces existence |
| `robocare/config/<MAC>` | Bridge → ESP | UID string | UID assignment |
| `robocare/<uid>/zone/<node>/data` | Sensor → ESP | JSON | Sensor data |
| `robocare/<uid>/valve/control/<zone>` | Bridge → ESP | "0" or "1" | Relay command |

