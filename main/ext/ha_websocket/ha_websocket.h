#include "esp_log.h"
#include "esp_event.h"
#include "websoc_credentials.h"

constexpr char *ha_websoc_log_tag{COLOR_GRAY "HAWEBSOC" COLOR_WHITE};
constexpr uint16_t request_txrxbuffer_len{2048};
static char rxtx_buffer[request_txrxbuffer_len];



typedef struct {
    int data_len;    
    uint8_t op_code;                       
    char rx_buffer[request_txrxbuffer_len];
    bool processed;
  } esp_websocket_rx_data_t;

  esp_websocket_rx_data_t websoc_rx_data;
  

  enum class websoc_status_t{

    websoc_sts_not_initialized = 0,
    websoc_sts_not_connected,
    websoc_sts_not_disconnected,
    websoc_sts_attempting_connect,
    websoc_sts_connected,
    websoc_sts_connected_authed,
    websoc_sts_error,
    websoc_sts_error_wait_reset

  };

  websoc_status_t socket_status;

class Homeassistant_websocket
{

public:
  char* ha_websoc_header_template = "{\"id\":%%d,\"type\":\"call_service\",\"domain\":\""HA_INTEGRATION_DOMAIN"\",\"service\":\""HA_DEVICE_NAME".set_values\",\"service_data\":{%s}}";

private:
  static constexpr const char *ha_websoc_endpoint{CRED_WEBSOCK_URL};
  static constexpr const char *ha_websoc_token{CRED_WEBSOCK_HA_TOKEN};
  static constexpr const char *ha_websoc_auth_template{"{\"type\":\"auth\",\"access_token\":\"%s\"}"};
  static constexpr const char *ha_websoc_ping_template{"{\"id\":%d,\"type\":\"ping\"}"};
  static constexpr const char *const dbgResultLabels[] = {"Response "COLOR_GREEN"OK"COLOR_GRAY, "Response "COLOR_RED"NEGATIVE"COLOR_GRAY, COLOR_RED"TIMEOUT"COLOR_GRAY, COLOR_RED"ERROR"COLOR_GRAY};

  static constexpr uint16_t poll_rx_buffer_ms{1};

public:
  esp_websocket_client_config_t websocket_cfg = {};
  esp_websocket_client_handle_t websoc_client_handle;
  bool websoc_init_completed = false;
  uint16_t request_id = 1; /* this will be added into each JSON payload so HA can add it to response, has to be incremented with each msg sent */

private:

  static const char* op_code_dbg(uint8_t op_code) {

    static const std::array<const char*, 7> op_code_strings{
      "OP_COUNT",
      "OP_TEXT",
      "OP_BINARY",
      "OP_CON_CLOSE",
      "OP_PING",
      "OP_PONG",
      "OP_UNKNOWN"
    };

    return (op_code < 6) ? op_code_strings[op_code] : op_code_strings[6];
  }

  static void event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

  esp_err_t init(void);
  esp_err_t connect(void);
  esp_err_t waitForResponse(const char* positiveRespKey,const char* negativeRespKey, uint16_t timeout, uint16_t expected_resp_id);

public:
  Homeassistant_websocket(void) { websocket_cfg.uri = ha_websoc_endpoint; }
  esp_err_t send_text(char *tx_text, char *positive_response_key, char *negative_response_key, uint16_t response_timeout_ms, uint8_t reattempt_send, uint16_t pause_after_attempt);
  esp_err_t ping(uint8_t numberOfRetries, uint16_t pauseBetweenRetries);
  esp_err_t disconnect(void);
  esp_err_t connectAndAuthSocket(uint8_t numberOfRetries, uint16_t pauseBetweenRetries);

  /* @TODO: find better place for this method, very loosely related to websocket*/
  static esp_err_t sensor_report_period_rdy(time_t &now_unix, time_t &last_report_unix, const time_t &report_period_sec, bool set_last_report = true)
  {

    esp_err_t res = ESP_FAIL;

    if (0 != now_unix)
    {

      if (0 == last_report_unix)
      {
        if (now_unix >= report_period_sec)
        {
          last_report_unix = (now_unix - report_period_sec);
        }
        res = ESP_OK;
      }

      if ((now_unix - last_report_unix) >= report_period_sec)
      {
        /* not just checking, but intend to send it, setting the new send time: last_report_unix*/
        if (true == set_last_report)
        {
          last_report_unix = now_unix;
        }
        res = ESP_OK;
      }
    }
    return res;
  }
};