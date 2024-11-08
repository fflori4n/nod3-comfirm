#include "ha_websocket.h"

void Homeassistant_websocket::event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
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
    ESP_LOGI(ha_websoc_log_tag, "recieved bytes: %d, code: %s", data->data_len, op_code_dbg(data->op_code));

    websoc_rx_data.op_code = data->op_code;
    memset(websoc_rx_data.rx_buffer, '\0', txrx_buffer_size);
    snprintf(websoc_rx_data.rx_buffer, txrx_buffer_size, (char *)data->data_ptr);
    websoc_rx_data.processed = false;
    }
    break;
  }
}

esp_err_t Homeassistant_websocket::waitForResponse(const char *positiveRespKey, const char *negativeRespKey, uint16_t timeout = 5000)
{

  esp_err_t result = ESP_ERR_TIMEOUT;
  uint16_t responseTime = 0;

  for(responseTime = 0; ((ESP_ERR_TIMEOUT == result) && (responseTime < (timeout / poll_rx_buffer_ms))); responseTime++)
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

  ESP_LOGI(ha_websoc_log_tag, "%s after %d ms", dbgResultLabels[(result & 0x03)], (responseTime * poll_rx_buffer_ms));

  return result;
}

esp_err_t Homeassistant_websocket::init(void)
{

  if (websoc_status_t::websoc_sts_not_initialized != socket_status)
  {
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

  /* create event handler, note that this should only be called once, when init is finished websoc is initialized until next MCU reboot. */
  esp_websocket_register_events(websoc_client_handle, WEBSOCKET_EVENT_ANY, this->event_handler, (void *)websoc_client_handle);

  socket_status = websoc_status_t::websoc_sts_not_connected;
  return ESP_OK;
}

esp_err_t Homeassistant_websocket::connect(void)
{

  esp_err_t res = ESP_OK;

  if (websoc_status_t::websoc_sts_not_connected == socket_status)
  {

    esp_websocket_client_stop(websoc_client_handle);
    res = esp_websocket_client_start(websoc_client_handle);
    if (ESP_OK == res)
    {
      socket_status = websoc_status_t::websoc_sts_attempting_connect;
    }
  }

  if (websoc_status_t::websoc_sts_attempting_connect == socket_status)
  {

    const constexpr uint16_t connection_attempt_timeout_ms{500};
    const constexpr uint16_t connection_attempt_pause{50 / portTICK_PERIOD_MS};

    uint16_t i = 0;
    for (i = 0; ((i < (connection_attempt_timeout_ms / connection_attempt_pause)) && (!esp_websocket_client_is_connected(websoc_client_handle))); i++)
    {
      vTaskDelay(connection_attempt_pause);
    }

    if (true == esp_websocket_client_is_connected(websoc_client_handle))
    {
      res = ESP_OK;
      socket_status = websoc_status_t::websoc_sts_connected;
    }
    else
    {
      res = ESP_ERR_TIMEOUT;
    }
  }

  if (websoc_status_t::websoc_sts_connected != socket_status)
  {
    esp_websocket_client_stop(websoc_client_handle);
    socket_status = websoc_status_t::websoc_sts_not_connected;
  }

  return res;
}

esp_err_t Homeassistant_websocket::disconnect(void){

    esp_err_t res = ESP_FAIL;

    if(socket_status >= websoc_status_t::websoc_sts_connected){
      esp_websocket_client_stop(websoc_client_handle);
      socket_status = websoc_status_t::websoc_sts_not_connected;
      res = ESP_OK;
    }

    return res;
  }

  esp_err_t Homeassistant_websocket::connectAndAuthSocket(uint8_t numberOfRetries = 10, uint16_t pauseBetweenRetries = 1500)
  {

    esp_err_t res = ESP_OK;

    if (websoc_status_t::websoc_sts_not_initialized == socket_status)
    {
      res = init();
      if (ESP_OK != res)
      {
        return res;
      }
    }

    if ((websoc_status_t::websoc_sts_connected != socket_status) && (socket_status >= websoc_status_t::websoc_sts_not_connected))
    {
      res = connect();
      if (ESP_OK != res)
      {
        return res;
      }
    }

    if ((websoc_status_t::websoc_sts_connected_authed != socket_status) && (socket_status >= websoc_status_t::websoc_sts_connected))
    {

      for (uint16_t i = 0; ((websoc_status_t::websoc_sts_connected == socket_status) && (i < numberOfRetries)); i++)
      {

        (void)waitForResponse("auth_required", NULL, 500); /* sometimes HA server will send welcome messege, ignore.*/

        memset(rxtx_buffer, '\0', txrx_buffer_size);
        uint16_t len = snprintf(rxtx_buffer, txrx_buffer_size, ha_websoc_auth_template, ha_websoc_token); /* move websoc out template with token into buff*/
        ESP_LOGI(ha_websoc_log_tag, "sending: %s", rxtx_buffer);

        int8_t result = esp_websocket_client_send_text(websoc_client_handle, rxtx_buffer, len, portMAX_DELAY);
        if (-1 == result)
        {
          /* send failed, check what is the cause:*/
          /* check if disconnected */
          if (false == esp_websocket_client_is_connected(websoc_client_handle))
          {
            socket_status = websoc_status_t::websoc_sts_not_connected;
          }
        }

        if (ESP_OK == waitForResponse("auth_ok", NULL, 2000))
        {
          socket_status = websoc_status_t::websoc_sts_connected_authed;
          request_id = 0;
        }
      }

      if (websoc_status_t::websoc_sts_connected_authed != socket_status)
      {
        /* Stop and destroy previous connection */
        /*esp_websocket_client_destroy(websoc_client_handle);*/

        esp_websocket_client_stop(websoc_client_handle);

        socket_status = websoc_status_t::websoc_sts_not_connected;
        vTaskDelay(pauseBetweenRetries / portTICK_PERIOD_MS);
      }
    }

    if (websoc_status_t::websoc_sts_connected_authed == socket_status)
    {
      ESP_LOGI(ha_websoc_log_tag, "connected. ready to send.");
    }

    return ESP_OK;
  }

 esp_err_t Homeassistant_websocket::send_text(char* tx_text, char* positive_response_key, char* negative_response_key, uint16_t response_timeout_ms = 500, uint8_t reattempt_send = 10, uint16_t pause_after_attempt = 1500){

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

  esp_err_t Homeassistant_websocket::ping(uint8_t numberOfRetries = 10, uint16_t pauseBetweenRetries = 1500){

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