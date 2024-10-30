#include "esp_log.h"
#include "esp_event.h"

static constexpr uint16_t txrx_buffer_size{2048};
static char rxtx_buffer[txrx_buffer_size];
std::mutex txrx_buffer_mutex;
std::mutex websoc_event_handler_mutex;
static bool data_ready;

typedef struct {
    int data_len;    
    uint8_t op_code;                       
    char rx_buffer[txrx_buffer_size];
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

private:
  static constexpr const char *ha_websoc_log_tag{"HAWEBSOC"};

  static constexpr const char *ha_websoc_endpoint{"ws://192.168.1.199:8123/api/websocket"};
  static constexpr const char *ha_websoc_token{"eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJhNjY4OTk1ZGQ5Mzg0YWRiOGEwZTk1OGUxZmM0YWJlZiIsImlhdCI6MTcwNTI0MjcyOSwiZXhwIjoyMDIwNjAyNzI5fQ.VKxml-fWpsuBZExIHViE2WEg1ZqxDcbNFFKWJIyflhc"};

  static constexpr const char *ha_websoc_auth_template{"{\"type\":\"auth\",\"access_token\":\"%s\"}"};
  static constexpr const char *ha_websoc_ping_template{"{\"id\":%d,\"type\":\"ping\"}"};

  static constexpr uint16_t poll_rx_buffer_ms{10};

  

public:
  esp_websocket_client_config_t websocket_cfg = {};
  esp_websocket_client_handle_t websoc_client_handle;
  uint16_t request_id = 1; /* this will be added into each JSON payload so HA can add it to response, has to be incremented with each msg sent */

  bool websoc_init_completed = false;

private:

  static char* op_code_dbg(uint8_t op_code){

    switch(op_code){

      case 0:
        return "OP_COUNT";
      break;
      case 1:
        return "OP_TEXT";
      break;
      case 2:
        return "OP_BINARY";
      break;
      case 8:
        return "OP_CON_CLOSE";
      break;
      case 9:
        return "OP_PING";
      break;
      case 10:
        return "OP_PONG";
      break;
    }
    return "OP_UNKNOWN";
  }

  esp_err_t init(void){

    if(websoc_status_t::websoc_sts_not_initialized != socket_status){
      return ESP_FAIL;
    }

    websocket_cfg.uri = ha_websoc_endpoint;
    websocket_cfg.network_timeout_ms = 5000;
    websocket_cfg.disable_auto_reconnect = true;
    websocket_cfg.task_name = "ha_websoc";
    websocket_cfg.pingpong_timeout_sec = 60;
    websocket_cfg.disable_pingpong_discon = false;

    websoc_client_handle = esp_websocket_client_init(&websocket_cfg);

    if (websoc_client_handle == nullptr)
    {
      return ESP_FAIL;
    }
    esp_websocket_register_events(websoc_client_handle, WEBSOCKET_EVENT_ANY, this->event_handler, (void *)websoc_client_handle);

    socket_status = websoc_status_t::websoc_sts_not_connected;
    return ESP_OK;

  }

  esp_err_t connect(void){

    esp_err_t res = ESP_OK;

    if(websoc_status_t::websoc_sts_not_connected == socket_status){

      
      esp_websocket_client_stop(websoc_client_handle);
      res = esp_websocket_client_start(websoc_client_handle);
      if(ESP_OK == res){
        socket_status = websoc_status_t::websoc_sts_attempting_connect;
      }
    }

    if(websoc_status_t::websoc_sts_attempting_connect == socket_status){

        const constexpr uint16_t connection_attempt_timeout_ms{500};
        const constexpr uint16_t connection_attempt_pause{50/portTICK_PERIOD_MS};

        uint16_t i = 0;
        for(i = 0; ((i < (connection_attempt_timeout_ms/connection_attempt_pause)) && (!esp_websocket_client_is_connected(websoc_client_handle))); i++){
          vTaskDelay(connection_attempt_pause);
        }

        if(true == esp_websocket_client_is_connected(websoc_client_handle)){
          res = ESP_OK;
          socket_status = websoc_status_t::websoc_sts_connected;
        }
        else{
          res = ESP_ERR_TIMEOUT;
        }
    }

    if(websoc_status_t::websoc_sts_connected != socket_status){
      esp_websocket_client_stop(websoc_client_handle);
      socket_status = websoc_status_t::websoc_sts_not_connected;
    }

    return res;
  }

  static void event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
  {

    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id)
    {
    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGW(ha_websoc_log_tag, "EVENT_ERROR");
      break;
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(ha_websoc_log_tag, "EVENT_CONNECTED");
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGI(ha_websoc_log_tag, "EVENT_DISCONNECTED");
      socket_status = websoc_status_t::websoc_sts_not_connected;

      break;
    case WEBSOCKET_EVENT_DATA:
    {
      ESP_LOGI(ha_websoc_log_tag, "EVENT_DATA");
      ESP_LOGI(ha_websoc_log_tag, "got: %d bytes, OPcode: %s", data->data_len, op_code_dbg(data->op_code));

      websoc_rx_data.op_code = data->op_code;
          memset(websoc_rx_data.rx_buffer, '\0', txrx_buffer_size);
          snprintf(websoc_rx_data.rx_buffer, txrx_buffer_size, (char *)data->data_ptr);
          websoc_rx_data.processed = false;
    }
    break;
    }
  }

  esp_err_t waitForResponse(char* positiveRespKey, char* negativeRespKey, uint16_t timeout = 5000)
  {

    esp_err_t result = ESP_ERR_TIMEOUT;
    uint16_t responseTime = 0;

    for (; ((ESP_ERR_TIMEOUT == result) && (responseTime < (timeout / poll_rx_buffer_ms))); responseTime++)
    {

      if (false == websoc_rx_data.processed)
      {

        ESP_LOGI(ha_websoc_log_tag, "recieved: %s", websoc_rx_data.rx_buffer);

        if ((positiveRespKey != NULL) && (strstr(websoc_rx_data.rx_buffer, positiveRespKey) != NULL))
        {
          result = ESP_OK;
        }
        else if ((negativeRespKey != NULL) && (strstr(websoc_rx_data.rx_buffer, negativeRespKey) != NULL))
        {
          result = ESP_FAIL;
        }
        websoc_rx_data.processed = true;
      }
      else
      {
        vTaskDelay(poll_rx_buffer_ms / portTICK_PERIOD_MS);
      }
    }

    constexpr const char *const dbgResultLabels[] = {"POSITIVE", "NEGATIVE", "TIMEOUT", "ERROR"};
    ESP_LOGI(ha_websoc_log_tag, "%s response; time elapsed: %d", dbgResultLabels[(result & 0x03)],(responseTime * poll_rx_buffer_ms));

    return result;
  }

public:
  Homeassistant_websocket(void)
  {
    websocket_cfg.uri = ha_websoc_endpoint;
  }


   esp_err_t send_text(char* tx_text, char* positive_response_key, char* negative_response_key, uint16_t response_timeout_ms = 500, uint8_t reattempt_send = 10, uint16_t pause_after_attempt = 1500){

    esp_err_t res = ESP_ERR_TIMEOUT;

    if(websoc_status_t::websoc_sts_connected_authed == socket_status){

      for (uint16_t i = 0; ((res != ESP_OK) && (i < reattempt_send)); i++)
      {

        memset(rxtx_buffer, '\0', txrx_buffer_size);

        /* TODO: check if str has %d and only one %d to add request id + check if text is longer than what buffer can accept*/
        uint16_t len = snprintf(rxtx_buffer, txrx_buffer_size, tx_text, request_id);
        request_id++;

        ESP_LOGI(ha_websoc_log_tag, "send_text: %s", rxtx_buffer);

          int8_t result = esp_websocket_client_send_text(websoc_client_handle, rxtx_buffer, len, portMAX_DELAY);

          if(result >= 0){
            res = waitForResponse(positive_response_key, negative_response_key, response_timeout_ms);
          }
          else{
            ESP_LOGE(ha_websoc_log_tag, "send failed.");
            /* send failed, check what is the cause:*/
            /* check if disconnected */
            if(false == esp_websocket_client_is_connected(websoc_client_handle)){
              socket_status = websoc_status_t::websoc_sts_not_connected;
            }
          }

          /* TODO: add pause after failed attempt*/
      } 
    }
    else{
      ESP_LOGE(ha_websoc_log_tag, "socket is not connected or authenticated");
      res = ESP_FAIL;
    }

    return res;
  }

  esp_err_t ping(uint8_t numberOfRetries = 10, uint16_t pauseBetweenRetries = 1500){

    esp_err_t res = ESP_ERR_TIMEOUT;

    if(websoc_status_t::websoc_sts_connected_authed == socket_status){

      for (uint16_t i = 0; ((res != ESP_OK) && (i < numberOfRetries)); i++)
      {

        memset(rxtx_buffer, '\0', txrx_buffer_size);
        uint16_t len = snprintf(rxtx_buffer, txrx_buffer_size, ha_websoc_ping_template, request_id);
        request_id++;

        ESP_LOGI(ha_websoc_log_tag, "sending: %s", rxtx_buffer);

          int8_t result = esp_websocket_client_send_text(websoc_client_handle, rxtx_buffer, len, portMAX_DELAY);
          

          if(result >= 0){
            res = waitForResponse("pong", NULL, 2000);
          }
          else{
            /* send failed, check what is the cause:*/
            /* check if disconnected */
            if(false == esp_websocket_client_is_connected(websoc_client_handle)){
              socket_status = websoc_status_t::websoc_sts_not_connected;
            }
          }
      } 
    }
    else{
      res = ESP_FAIL;
    }

    return res;
  }

  esp_err_t disconnect(void){

    esp_err_t res = ESP_FAIL;

    if(socket_status >= websoc_status_t::websoc_sts_connected){
      esp_websocket_client_stop(websoc_client_handle);
      socket_status = websoc_status_t::websoc_sts_not_connected;
      res = ESP_OK;
    }

    return res;
  }

  esp_err_t connectAndAuthSocket(uint8_t numberOfRetries = 10, uint16_t pauseBetweenRetries = 1500)
  {

    esp_err_t res = ESP_OK;

    if(websoc_status_t::websoc_sts_not_initialized == socket_status){
      res = init();
      if(ESP_OK != res){
        return res;
      }
    }

    if((websoc_status_t::websoc_sts_connected != socket_status) && (socket_status >= websoc_status_t::websoc_sts_not_connected)){
      res = connect();
      if(ESP_OK != res){
        return res;
      }
    }
    
    if((websoc_status_t::websoc_sts_connected_authed != socket_status) && (socket_status >= websoc_status_t::websoc_sts_connected)){

      for (uint16_t i = 0; ((websoc_status_t::websoc_sts_connected == socket_status) && (i < numberOfRetries)); i++)
      {

        (void)waitForResponse("auth_required", NULL, 500); /* sometimes HA server will send welcome messege, ignore.*/

        memset(rxtx_buffer, '\0', txrx_buffer_size);
        uint16_t len = snprintf(rxtx_buffer, txrx_buffer_size, ha_websoc_auth_template, ha_websoc_token); /* move websoc out template with token into buff*/
        ESP_LOGI(ha_websoc_log_tag, "sending: %s", rxtx_buffer);

          int8_t result = esp_websocket_client_send_text(websoc_client_handle, rxtx_buffer, len, portMAX_DELAY);
          if(-1 == result){
            /* send failed, check what is the cause:*/
            /* check if disconnected */
            if(false == esp_websocket_client_is_connected(websoc_client_handle)){
              socket_status = websoc_status_t::websoc_sts_not_connected;
            }
            
          }

          if(ESP_OK == waitForResponse("auth_ok", NULL, 2000)){
            socket_status = websoc_status_t::websoc_sts_connected_authed;
            request_id = 0;
          }
      }

      if(websoc_status_t::websoc_sts_connected_authed != socket_status){
      /* Stop and destroy previous connection */
      /*esp_websocket_client_destroy(websoc_client_handle);*/


      esp_websocket_client_stop(websoc_client_handle);
      
      socket_status = websoc_status_t::websoc_sts_not_connected;
      vTaskDelay(pauseBetweenRetries / portTICK_PERIOD_MS);
    }

    }

    if(websoc_status_t::websoc_sts_connected_authed == socket_status){
       ESP_LOGI(ha_websoc_log_tag, "connected. ready to send.");
    }

    return ESP_OK;
  }
};