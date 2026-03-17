/**
 * @file main.cpp
 * @brief Binary MQTT Sensor firmware.
 * @version 0.2
 * @author John Canty
 * @date 2025-10-28
 *
 * Overview
 * - Reads one binary input (active-low by default).
 * - Publishes sensor state to MQTT topic: <topicPrefix><deviceName>/DETECT.
 * - Publishes LWT status to: <topicPrefix><deviceName>/LWT.
 * - Hosts a configuration web UI for Wi-Fi and MQTT settings.
 * - Supports first-boot provisioning over SoftAP when Wi-Fi credentials are missing.
 * - Persists runtime settings in NVS namespaces: "wifi" and "mqtt".
 *
 * MQTT payload conventions
 * - DETECT topic publishes "OFF" when input is active (LOW).
 * - DETECT topic publishes "ON" when input is inactive (HIGH).
 * - LWT publishes retained "Online"/"Offline".
 *
 * Provisioning flow
 * - If Wi-Fi credentials are present, device boots in STA mode.
 * - If credentials are absent, device starts setup SoftAP and serves config page.
 * - Saving valid Wi-Fi credentials from the UI triggers reconnect to STA.
 *
 * Security notes
 * - Credentials and MQTT settings are stored in NVS.
 * - For stronger at-rest protection in production, combine with flash encryption.
 *
 * Maintenance notes
 * - Keep ISR handlers minimal and non-blocking.
 * - Keep loop() non-blocking; avoid long delays in normal control paths.
 */

#include <M5NanoC6.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WebServer.h>

#define NUM_LEDS 1
#define BUFFER_SIZE 64

Adafruit_NeoPixel strip(NUM_LEDS, M5NANO_C6_RGB_LED_DATA_PIN,
                        NEO_GRB + NEO_KHZ800);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences prefs;
WebServer webServer(80);

// Runtime configuration loaded from NVS (or defaults below).
// These values are the source of truth used by MQTT and web configuration UI.
String wifiSsid;
String wifiPassword;
String mqttServer = "10.10.11.55";
String deviceName = "BinarySensor"; // Used in topic: <topicPrefix><deviceName>/DETECT
String topicPrefix = "BinarySensors/";
String subscribeTopic = "";
String subscribeValue = "";
String dhcpHostname;
String setupApSsid;
bool webServerStarted = false;
bool webRoutesConfigured = false;
bool neoPixelToggleState = false;
bool provisioningApMode = false;

// Shared topic buffer for short MQTT topic composition.
char topic_buffer[BUFFER_SIZE];

// Binary sensor input pin and ISR-shared state.
// Variables touched by ISR are marked volatile and copied in loop() when used.
const int contactPin = 1;
volatile int lastContactState = HIGH;
volatile bool stateChanged = false;
volatile int currentContactState = HIGH; // HIGH=inactive, LOW=active for active-low wiring.
volatile unsigned long lastInterruptTime = 0;

int wifiRetryCount = 0;
unsigned long lastRebootMillis = 0;
unsigned long lastWiFiCheckMillis = 0;
unsigned long lastPublishMillis = 0;
unsigned long lastWiFiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastWiFiGoodMillis = 0;
unsigned long bootMillis = 0;
bool wifiHasEverConnected = false;
const unsigned long WIFI_CHECK_INTERVAL = 30000;  // Check WiFi every 30 seconds
const unsigned long WIFI_RECONNECT_INTERVAL = 5000; // Wait 5 seconds between reconnect attempts
const unsigned long WIFI_HARD_RESET_INTERVAL = 120000; // Re-init WiFi stack after 2 min down
const unsigned long MQTT_RECONNECT_INTERVAL = 3000; // MQTT retry every 3 seconds

// Forward declarations
void handleContactChange();
void connectToWiFi();
void callback(char* topic, byte* payload, unsigned int length);
void reConnect();
void buildTopic(char* buffer, const String& mqttDeviceName, const char* suffix);
void publishState(int state);
void scheduleDailyReboot();
void checkWiFiConnection();
void wifiEventHandler(WiFiEvent_t event, WiFiEventInfo_t info);
bool loadWiFiCredentials();
bool saveWiFiCredentials(const String& ssid, const String& passwordValue);
bool provisionWiFiCredentials();
bool loadMqttSettings();
bool saveMqttSettings(const String& mqttServerValue, const String& mqttDeviceName,
                      const String& topicPrefixValue, const String& subscribeTopicValue,
                      const String& subscribeValueValue);
void configureMqttClient();
void configureWebRoutes();
void startWebServer();
void stopWebServer();
void handleConfigPage();
void handleSaveConfig();
String buildDhcpHostname();
String normalizeTopicPrefix(const String& prefix);
String htmlEscape(const String& value);
void startSoftAPProvisioning();

/**
 * @brief Device initialization entry point.
 *
 * Startup sequence:
 * 1. Initialize board peripherals and LED.
 * 2. Initialize Wi-Fi stack and event hooks.
 * 3. Load persisted settings from NVS.
 * 4. Enter STA mode (if Wi-Fi creds exist) or setup SoftAP mode.
 * 5. Initialize MQTT client and binary input interrupt.
 */
void setup() {
    NanoC6.begin();
    bootMillis = millis();
    Serial.begin(115200);
    dhcpHostname = buildDhcpHostname();
    setupApSsid =  "Sensor-setup";
    
    // Enable RGB LED Power
    pinMode(M5NANO_C6_RGB_LED_PWR_PIN, OUTPUT);
    digitalWrite(M5NANO_C6_RGB_LED_PWR_PIN, HIGH);
    
    // Setup contact sensor pin
    pinMode(contactPin, INPUT_PULLUP);
    
    // Initialize LED to off
    strip.begin();
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
    
    // Configure WiFi for automatic reconnection and enable event handler
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false); // Reduce power-save related disconnects
    WiFi.persistent(false);  // Don't save credentials to flash
    WiFi.onEvent(wifiEventHandler);

    // Load stored credentials. If absent, start SoftAP setup mode.
    bool hasWiFiCredentials = loadWiFiCredentials();

    loadMqttSettings();
    configureWebRoutes();

    if (hasWiFiCredentials) {
        connectToWiFi();
    } else {
        startSoftAPProvisioning();
    }
    
    // Setup MQTT
    configureMqttClient();
    
    // Set MQTT keep-alive and socket timeout
    client.setKeepAlive(30);
    client.setSocketTimeout(10);  // 10 second socket timeout
    
    // Attach interrupt to the contactPin
    attachInterrupt(digitalPinToInterrupt(contactPin), handleContactChange, CHANGE);
    
    // Send initial state
    publishState(currentContactState);
}

/**
 * @brief ISR for binary sensor transitions with basic debounce.
 *
 * Contract:
 * - Keep execution short and deterministic.
 * - Do not perform network, heap-heavy, or blocking operations.
 */
void IRAM_ATTR handleContactChange() {
    // Simple debounce - ignore interrupts too close together
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 50) { // 50ms debounce
        currentContactState = digitalRead(contactPin);
        stateChanged = true;
        lastInterruptTime = interruptTime;
    }
}

/**
 * @brief Connect station interface to configured Wi-Fi credentials.
 *
 * Behavior:
 * - Configures STA mode and DHCP hostname.
 * - Performs short synchronous connect attempt.
 * - Leaves long-term recovery to periodic reconnect logic.
 */
void connectToWiFi() {
    if (wifiSsid.length() == 0) {
        Serial.println("No WiFi credentials available");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(dhcpHostname.c_str());
    
    // Connect to configured WiFi
    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    int retryCount = 0;
    
    while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
        delay(500);
        retryCount++;
    }

    // If failed, log and continue (don't restart - let WiFi auto-reconnect handle it)
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection failed - will retry automatically");
    } else {
        Serial.println("WiFi connected");
        provisioningApMode = false;
        wifiHasEverConnected = true;
        lastWiFiGoodMillis = millis();
    }
    
    wifiRetryCount = 0; // Reset retry count on successful connection
}

/**
 * @brief MQTT callback for optional command handling.
 *
 * Supported behaviors:
 * - Optional topic/value trigger toggles NeoPixel color.
 * - Backward-compatible "open"/"clos" payload handling.
 */
void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    message.trim();

    // Optional command: if topic and value are configured, matching payload toggles LED color.
    if (subscribeTopic.length() > 0 && subscribeValue.length() > 0) {
        String incomingTopic = String(topic);
        if (incomingTopic == subscribeTopic && message == subscribeValue) {
            neoPixelToggleState = !neoPixelToggleState;
            if (neoPixelToggleState) {
                strip.setPixelColor(0, strip.Color(255, 0, 0));
            } else {
                strip.setPixelColor(0, strip.Color(0, 255, 0));
            }
            strip.show();
            return;
        }
    }

    // Optional status LED command channel for external MQTT commands.
    // RED = active/alert, GREEN = inactive/normal.
    
    if (message == "open") {
        // Example command: "open" -> green.
        strip.setPixelColor(0, strip.Color(0, 255, 0));
        strip.show();
    } else if (message == "clos") {
        // Example command: "clos" -> red.
        strip.setPixelColor(0, strip.Color(255, 0, 0));
        strip.show();
    }
}

/**
 * @brief Maintain MQTT session while Wi-Fi is connected.
 *
 * Behavior:
 * - Rate-limits reconnect attempts.
 * - Publishes retained LWT on success.
 * - Resubscribes optional command topic.
 */
void reConnect() {
    // Handle WiFi disconnection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, attempting to reconnect...");
        WiFi.reconnect();  // Ask WiFi stack to reconnect
        return;  // Exit early, let WiFi auto-reconnect handle it
    }
    
    // Only attempt MQTT connection if WiFi is connected
    if (!client.connected()) {
        unsigned long currentMillis = millis();
        
        // Rate limit reconnection attempts to every 5 seconds
        if (currentMillis - lastMqttReconnectAttempt < MQTT_RECONNECT_INTERVAL) {
            return;
        }
        lastMqttReconnectAttempt = currentMillis;
        
        Serial.println("Attempting MQTT connection...");
        
        // Create a random client ID
        String clientId = "M5Stack-";
        clientId += String(random(0xffff), HEX);

        // Build the Last Will and Testament topic
        buildTopic(topic_buffer, deviceName, "LWT");

        // Attempt to connect with LWT
        if (client.connect(clientId.c_str(), topic_buffer, 0, true, "Offline")) {
            // Publish "Online" to the LWT topic after connecting
            client.publish(topic_buffer, "Online", true);

            if (subscribeTopic.length() > 0 && subscribeValue.length() > 0) {
                client.subscribe(subscribeTopic.c_str());
            }
            Serial.println("MQTT connected");
            wifiRetryCount = 0;  // Reset on successful connection
            publishState(currentContactState); // Republish current sensor state after reconnect
        } else {
            Serial.print("MQTT connection failed, rc=");
            Serial.println(client.state());
            wifiRetryCount++;
            
            // Only restart after MANY failed attempts (not just 3)
            if (wifiRetryCount > 10) {
                Serial.println("Too many MQTT failures, restarting device...");
                delay(1000);
                ESP.restart();
            }
        }
    }
}

/**
 * @brief Build an MQTT topic from current prefix/device and suffix.
 */
void buildTopic(char* buffer, const String& mqttDeviceName, const char* suffix) {
    int len = snprintf(buffer, BUFFER_SIZE, "%s%s/%s", topicPrefix.c_str(), mqttDeviceName.c_str(), suffix);
    if (len < 0 || len >= BUFFER_SIZE) {
        // Handle error - truncate if needed
        buffer[BUFFER_SIZE-1] = '\0';
    }
}

/**
 * @brief Publish binary sensor state and update status LED.
 */
void publishState(int state) {
    buildTopic(topic_buffer, deviceName, "DETECT");
    
    if (state == LOW) {
        // Active state (LOW for active-low wiring).
        client.publish(topic_buffer, "OFF");
        strip.setPixelColor(0, strip.Color(255, 0, 0)); // RED for alert
        strip.show();
        lastPublishMillis = millis();  // Record when we showed the alert
    } else {
        // Inactive state.
        client.publish(topic_buffer, "ON");
        strip.setPixelColor(0, strip.Color(0, 255, 0)); // GREEN for normal
        strip.show();
    }
}

/**
 * @brief Optional periodic reboot watchdog.
 *
 * Intended as a field-hardening fallback for long-uptime edge cases.
 */
void scheduleDailyReboot() {
    unsigned long currentMillis = millis();

    // Handle millis() overflow
    if (currentMillis < lastRebootMillis) {
        lastRebootMillis = 0; // Reset on overflow
    }

    // Check if 24 hours (86,400,000 milliseconds) have passed
    if (currentMillis - lastRebootMillis >= 86400000) {
        Serial.println("Scheduled daily reboot...");
        ESP.restart();
    }
}

/**
 * @brief Wi-Fi event hook to coordinate MQTT and web server lifecycle.
 */
void wifiEventHandler(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_START:
            Serial.println("WiFi STA started");
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("WiFi connected");
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("WiFi disconnected - waiting for reconnection...");
            if (client.connected()) {
                client.disconnect();
            }
            if (!provisioningApMode) {
                stopWebServer();
            }
            // WiFi will auto-reconnect due to setAutoReconnect(true)
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            Serial.print("DHCP hostname: ");
            Serial.println(WiFi.getHostname());
            wifiHasEverConnected = true;
            lastWiFiGoodMillis = millis();
            startWebServer();
            break;
        default:
            break;
    }
}

/**
 * @brief Periodic Wi-Fi health checks and staged recovery.
 *
 * Recovery escalation:
 * 1. Lightweight reconnect.
 * 2. Full STA re-initialize after sustained downtime.
 */
void checkWiFiConnection() {
    if (provisioningApMode) {
        return;
    }

    unsigned long currentMillis = millis();
    
    // Check WiFi status periodically
    if (currentMillis - lastWiFiCheckMillis >= WIFI_CHECK_INTERVAL) {
        lastWiFiCheckMillis = currentMillis;
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi status check: disconnected, triggering reconnect...");
            if (currentMillis - lastWiFiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
                lastWiFiReconnectAttempt = currentMillis;
                WiFi.reconnect();
            }

            // Escalate to full WiFi re-init if down too long
            unsigned long downDuration = wifiHasEverConnected
                ? (currentMillis - lastWiFiGoodMillis)
                : (currentMillis - bootMillis);
            if (downDuration >= WIFI_HARD_RESET_INTERVAL) {
                Serial.println("WiFi down too long - reinitializing WiFi stack...");
                WiFi.disconnect();
                delay(100);
                WiFi.mode(WIFI_STA);
                WiFi.setHostname(dhcpHostname.c_str());
                WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
                lastWiFiGoodMillis = currentMillis; // Prevent repeated hard resets in tight loop
            }
        } else {
            Serial.println("WiFi status check: connected");
            lastWiFiGoodMillis = currentMillis;
        }
    }
}

/**
 * @brief Load MQTT and optional subscription settings from NVS.
 */
bool loadMqttSettings() {
    prefs.begin("mqtt", true);
    mqttServer = prefs.getString("server", mqttServer);
    deviceName = prefs.getString("device", deviceName);
    topicPrefix = normalizeTopicPrefix(prefs.getString("prefix", topicPrefix));
    subscribeTopic = prefs.getString("subTopic", subscribeTopic);
    subscribeValue = prefs.getString("subValue", subscribeValue);
    subscribeTopic.trim();
    subscribeValue.trim();
    prefs.end();

    bool hasSettings = mqttServer.length() > 0 && deviceName.length() > 0 && topicPrefix.length() > 0;
    if (hasSettings) {
        Serial.println("Loaded MQTT settings from secure storage");
    }

    return hasSettings;
}

/**
 * @brief Persist MQTT and optional command-subscription settings to NVS.
 * @return true when values were saved and verified by immediate readback.
 */
bool saveMqttSettings(const String& mqttServerValue, const String& mqttDeviceName,
                      const String& topicPrefixValue, const String& subscribeTopicValue,
                      const String& subscribeValueValue) {
    String normalizedPrefix = normalizeTopicPrefix(topicPrefixValue);
    String normalizedSubTopic = subscribeTopicValue;
    String normalizedSubValue = subscribeValueValue;
    normalizedSubTopic.trim();
    normalizedSubValue.trim();
    if (mqttServerValue.length() == 0 || mqttDeviceName.length() == 0 || normalizedPrefix.length() == 0) {
        return false;
    }

    prefs.begin("mqtt", false);
    prefs.putString("server", mqttServerValue);
    prefs.putString("device", mqttDeviceName);
    prefs.putString("prefix", normalizedPrefix);
    prefs.putString("subTopic", normalizedSubTopic);
    prefs.putString("subValue", normalizedSubValue);
    String verifyServer = prefs.getString("server", "");
    String verifyDevice = prefs.getString("device", "");
    String verifyPrefix = prefs.getString("prefix", "");
    String verifySubTopic = prefs.getString("subTopic", "");
    String verifySubValue = prefs.getString("subValue", "");
    prefs.end();

    if (verifyServer != mqttServerValue || verifyDevice != mqttDeviceName || verifyPrefix != normalizedPrefix ||
        verifySubTopic != normalizedSubTopic || verifySubValue != normalizedSubValue) {
        return false;
    }

    mqttServer = mqttServerValue;
    deviceName = mqttDeviceName;
    topicPrefix = normalizedPrefix;
    subscribeTopic = normalizedSubTopic;
    subscribeValue = normalizedSubValue;
    Serial.println("MQTT settings saved to secure storage");
    return true;
}

/**
 * @brief Apply current MQTT endpoint and callback bindings.
 */
void configureMqttClient() {
    client.setServer(mqttServer.c_str(), 1883);
    client.setCallback(callback);
}

/**
 * @brief Register all HTTP routes for configuration web UI.
 */
void configureWebRoutes() {
    if (webRoutesConfigured) {
        return;
    }

    webServer.on("/", HTTP_GET, handleConfigPage);
    webServer.on("/save", HTTP_POST, handleSaveConfig);
    webServer.onNotFound([]() {
        webServer.sendHeader("Location", "/");
        webServer.send(302, "text/plain", "");
    });
    webRoutesConfigured = true;
}

/**
 * @brief Start HTTP server if not already running.
 */
void startWebServer() {
    if (!webRoutesConfigured) {
        configureWebRoutes();
    }

    if (!webServerStarted) {
        webServer.begin();
        webServerStarted = true;
        Serial.println("Web configuration server started");
    }
}

/**
 * @brief Stop HTTP server if running.
 */
void stopWebServer() {
    if (webServerStarted) {
        webServer.stop();
        webServerStarted = false;
        Serial.println("Web configuration server stopped");
    }
}

/**
 * @brief Render web-based configuration page.
 */
void handleConfigPage() {
    String page;
    page.reserve(2600);
    page += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
    page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    page += "<title>Binary MQTT Sensor Setup</title>";
    page += "<style>body{font-family:Helvetica,Arial,sans-serif;margin:0;background:#eef3f7;color:#102a43;}";
    page += ".wrap{max-width:680px;margin:24px auto;padding:24px;}";
    page += ".card{background:#fff;border-radius:16px;padding:24px;box-shadow:0 12px 32px rgba(16,42,67,.12);}";
    page += "h1{margin-top:0;font-size:1.8rem;}label{display:block;margin-top:16px;font-weight:600;}";
    page += "input{width:100%;padding:12px 14px;border:1px solid #bcccdc;border-radius:10px;font-size:1rem;box-sizing:border-box;}";
    page += "button{margin-top:20px;background:#0f766e;color:#fff;border:0;border-radius:999px;padding:12px 18px;font-size:1rem;}";
    page += ".meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;margin-bottom:18px;}";
    page += ".meta div{background:#f0f4f8;border-radius:12px;padding:12px;}small{display:block;color:#486581;margin-bottom:4px;}";
    page += "</style></head><body><div class='wrap'><div class='card'>";
    page += "<h1>Binary MQTT Sensor Setup</h1>";
    page += "<div class='meta'>";
    page += "<div><small>DHCP Hostname</small>" + htmlEscape(WiFi.getHostname() ? String(WiFi.getHostname()) : dhcpHostname) + "</div>";
    page += "<div><small>IP Address</small>" + htmlEscape(WiFi.localIP().toString()) + "</div>";
    page += "<div><small>WiFi SSID</small>" + htmlEscape(wifiSsid) + "</div>";
    page += "<div><small>MQTT Status</small>" + String(client.connected() ? "Connected" : "Disconnected") + "</div>";
    page += "</div>";
    if (provisioningApMode) {
        page += "<p><strong>Setup Mode:</strong> Connect this device to your WiFi and save. AP SSID: " + htmlEscape(setupApSsid) + "</p>";
    }
    page += "<form method='POST' action='/save'>";
    page += "<label for='wifi_ssid'>WiFi SSID</label>";
    page += "<input id='wifi_ssid' name='wifi_ssid' value='" + htmlEscape(wifiSsid) + "' required>";
    page += "<label for='wifi_password'>WiFi Password</label>";
    page += "<input id='wifi_password' name='wifi_password' type='password' placeholder='Leave blank to keep current password'>";
    page += "<label for='mqtt_server'>MQTT Server</label>";
    page += "<input id='mqtt_server' name='mqtt_server' value='" + htmlEscape(mqttServer) + "' required>";
    page += "<label for='device_name'>Device</label>";
    page += "<input id='device_name' name='device_name' value='" + htmlEscape(deviceName) + "' required>";
    page += "<label for='topic_prefix'>Topic Prefix</label>";
    page += "<input id='topic_prefix' name='topic_prefix' value='" + htmlEscape(topicPrefix) + "' required>";
    page += "<label for='subscribe_topic'>Subscribe Topic (optional)</label>";
    page += "<input id='subscribe_topic' name='subscribe_topic' value='" + htmlEscape(subscribeTopic) + "' placeholder='Example: BinarySensors/commands'>";
    page += "<label for='subscribe_value'>Toggle Value (optional)</label>";
    page += "<input id='subscribe_value' name='subscribe_value' value='" + htmlEscape(subscribeValue) + "' placeholder='Example: TOGGLE'>";
    page += "<button type='submit'>Save Configuration</button></form>";
    page += "</div></div></body></html>";
    webServer.send(200, "text/html", page);
}

/**
 * @brief Handle configuration form submission and apply changes.
 *
 * Side effects:
 * - Saves Wi-Fi/MQTT settings to NVS.
 * - Reconfigures MQTT connection.
 * - Reconnects Wi-Fi when credentials changed.
 */
void handleSaveConfig() {
    String newWifiSsid = webServer.arg("wifi_ssid");
    String newWifiPassword = webServer.arg("wifi_password");
    String newMqttServer = webServer.arg("mqtt_server");
    String newDeviceName = webServer.arg("device_name");
    String newTopicPrefix = webServer.arg("topic_prefix");
    String newSubscribeTopic = webServer.arg("subscribe_topic");
    String newSubscribeValue = webServer.arg("subscribe_value");
    newWifiSsid.trim();
    newMqttServer.trim();
    newDeviceName.trim();
    newTopicPrefix.trim();
    newSubscribeTopic.trim();
    newSubscribeValue.trim();

    if (newWifiSsid.length() == 0) {
        webServer.send(400, "text/plain", "WiFi SSID is required");
        return;
    }

    if (newWifiPassword.length() == 0 && newWifiSsid == wifiSsid) {
        newWifiPassword = wifiPassword;
    }

    bool wifiChanged = (newWifiSsid != wifiSsid) || (newWifiPassword != wifiPassword);
    if (wifiChanged && !saveWiFiCredentials(newWifiSsid, newWifiPassword)) {
        webServer.send(400, "text/plain", "Invalid WiFi configuration");
        return;
    }

    if (!saveMqttSettings(newMqttServer, newDeviceName, newTopicPrefix,
                          newSubscribeTopic, newSubscribeValue)) {
        webServer.send(400, "text/plain", "Invalid MQTT configuration");
        return;
    }

    configureMqttClient();
    if (client.connected()) {
        client.disconnect();
    }
    lastMqttReconnectAttempt = 0;

    String response = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2;url=/'></head><body>";
    response += "Configuration saved. Applying updates...";
    response += "</body></html>";
    webServer.send(200, "text/html", response);

    if (wifiChanged) {
        delay(150);
        provisioningApMode = false;
        WiFi.disconnect();
        delay(100);
        connectToWiFi();

        if (WiFi.status() != WL_CONNECTED) {
            startSoftAPProvisioning();
        }
    }
}

/**
 * @brief Build deterministic DHCP hostname using MAC suffix.
 */
String buildDhcpHostname() {
    uint64_t mac = ESP.getEfuseMac();
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "binary-sensor-%06llX", (mac & 0xFFFFFFULL));
    return String(buffer);
}

/**
 * @brief Normalize topic prefix to include trailing slash.
 */
String normalizeTopicPrefix(const String& prefix) {
    String normalized = prefix;
    normalized.trim();
    if (normalized.length() == 0) {
        return "";
    }
    if (!normalized.endsWith("/")) {
        normalized += "/";
    }
    return normalized;
}

/**
 * @brief HTML-escape dynamic text before rendering into web UI.
 */
String htmlEscape(const String& value) {
    String escaped = value;
    escaped.replace("&", "&amp;");
    escaped.replace("<", "&lt;");
    escaped.replace(">", "&gt;");
    escaped.replace("\"", "&quot;");
    escaped.replace("'", "&#39;");
    return escaped;
}

/**
 * @brief Start setup SoftAP for first-boot or fallback provisioning.
 */
void startSoftAPProvisioning() {
    provisioningApMode = true;
    WiFi.mode(WIFI_AP);
    bool started = WiFi.softAP(setupApSsid.c_str());
    if (!started) {
        Serial.println("Failed to start setup AP");
        return;
    }

    IPAddress apIp = WiFi.softAPIP();
    Serial.println("Setup AP started");
    Serial.print("AP SSID: ");
    Serial.println(setupApSsid);
    Serial.print("AP IP: ");
    Serial.println(apIp);
    startWebServer();
}

/**
 * @brief Load Wi-Fi credentials from NVS namespace "wifi".
 */
bool loadWiFiCredentials() {
    prefs.begin("wifi", true);
    wifiSsid = prefs.getString("ssid", "");
    wifiPassword = prefs.getString("pass", "");
    prefs.end();

    bool hasCreds = wifiSsid.length() > 0;
    if (hasCreds) {
        Serial.println("Loaded WiFi credentials from secure storage");
    } else {
        Serial.println("No WiFi credentials found in secure storage");
    }

    return hasCreds;
}

/**
 * @brief Persist Wi-Fi credentials to NVS with readback verification.
 */
bool saveWiFiCredentials(const String& ssid, const String& passwordValue) {
    if (ssid.length() == 0) {
        return false;
    }

    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", passwordValue);
    String verifySsid = prefs.getString("ssid", "");
    String verifyPass = prefs.getString("pass", "");
    prefs.end();

    bool okSsid = verifySsid == ssid;
    bool okPass = verifyPass == passwordValue;

    if (okSsid && okPass) {
        wifiSsid = ssid;
        wifiPassword = passwordValue;
        Serial.println("WiFi credentials saved to secure storage");
        return true;
    }

    return false;
}

/**
 * @brief Legacy serial provisioning path.
 *
 * NOTE: SoftAP web provisioning is the primary first-boot path.
 * This function remains as a maintenance fallback for serial-only recovery.
 */
bool provisionWiFiCredentials() {
    const unsigned long timeoutMs = 300000; // 5 minutes
    unsigned long startMs = millis();

    Serial.println("WiFi provisioning required.");
    Serial.println("Send one line in this format:");
    Serial.println("WIFI:<ssid>,<password>");

    String line;
    while (millis() - startMs < timeoutMs) {
        while (Serial.available() > 0) {
            char c = (char)Serial.read();
            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                line.trim();
                if (!line.startsWith("WIFI:")) {
                    Serial.println("Invalid format. Expected WIFI:<ssid>,<password>");
                    line = "";
                    continue;
                }

                String payload = line.substring(5);
                int comma = payload.indexOf(',');
                if (comma <= 0) {
                    Serial.println("Invalid format. Missing comma separator");
                    line = "";
                    continue;
                }

                String ssid = payload.substring(0, comma);
                String passwordValue = payload.substring(comma + 1);
                ssid.trim();
                passwordValue.trim();

                if (ssid.length() == 0) {
                    Serial.println("SSID cannot be empty");
                    line = "";
                    continue;
                }

                return saveWiFiCredentials(ssid, passwordValue);
            }

            if (line.length() < 256) {
                line += c;
            }
        }

        delay(20);
    }

    return false;
}

/**
 * @brief Main control loop.
 *
 * Responsibilities:
 * - Service web UI.
 * - Maintain Wi-Fi/MQTT connectivity.
 * - Publish sensor changes.
 * - Perform periodic housekeeping.
 */
void loop() {
    NanoC6.update();

    if (webServerStarted) {
        webServer.handleClient();
    }

    // Check WiFi connection periodically
    checkWiFiConnection();

    // Handle MQTT
    if (!client.connected()) {
        reConnect();
    } else {
        client.loop();  // Only loop MQTT when connected
    }

    // Check if the state has changed
    if (stateChanged) {
        stateChanged = false; // Reset the flag
        
        // Use a temporary variable to avoid race condition
        int currentState = currentContactState;
        publishState(currentState);
        
        // Update last state
        lastContactState = currentState;
    }
    
    // Turn off alert LED after 5 seconds
    if (currentContactState == LOW) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastPublishMillis > 5000) {
            strip.setPixelColor(0, strip.Color(0, 0, 0)); // Turn off
            strip.show();
        }
    }

    scheduleDailyReboot();
    
    delay(100); // Small delay to prevent busy waiting
}