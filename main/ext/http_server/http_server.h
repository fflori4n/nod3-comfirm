#include "esp_log.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "string.h"
#include "lwip/dns.h"
//#include "captdns.h"

httpd_handle_t user_cfg_http_server;

const char index_html[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP C3: sens_FE5423</title>
  <style>
    body {
      font-family: sans-serif;
      background: #f4f4f4;
      display: flex;
      justify-content: center;
      align-items: center;
      flex-direction: column;
      height: 100vh;
      margin: 0;
    }
    h1 {
      margin-bottom: 0.5em;
      font-size: 1.2rem;
    }
    .warning {
      color: #b00;
      font-size: 0.9rem;
      margin-bottom: 1.5em;
      text-align: center;
      max-width: 300px;
    }
    form {
      background: #fff;
      padding: 1em;
      border-radius: 8px;
      box-shadow: 0 2px 6px rgba(0,0,0,0.1);
      display: flex;
      flex-direction: column;
      gap: 0.75em;
      width: 90%;
      max-width: 320px;
    }
    input, button {
      padding: 0.6em;
      font-size: 1em;
      border-radius: 6px;
      border: 1px solid #ccc;
    }
    button {
      background: #007bff;
      color: #fff;
      border: none;
    }
    button:hover {
      background: #0056b3;
    }
  </style>
</head>
<body>
  <h1>ESP C3: sens_FE5423</h1>
  <div class="warning">⚠️ Password will be sent over unsecured HTTP.</div>
  <form method="POST" action="/">
    <input name="ssid" placeholder="SSID" required>
    <input name="pass" type="password" placeholder="Password" required>
    <button type="submit">Connect</button>
  </form>
</body>
</html>
)rawliteral";

/* Our URI handler function to be called during GET /uri request */
esp_err_t get_handler(httpd_req_t *req)
{
    /* Send a simple response */
    const char index_html_start[] = "<!DOCTYPE html><html><head><title>ESP32</title></head><body><h1>Hello from ESP32!</h1></body></html>";
    
    ESP_LOGE("http","got get");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, strlen(index_html_start));
    return ESP_OK;
}

esp_err_t get_user_cfg_page(httpd_req_t *req)
{
    /* Send a page with form to fill out config. */
    //const char user_cfg_form_html[] = "<!DOCTYPE html><html><body><form method='POST' action='/'><input name='ssid' placeholder='SSID'><input name='pass' placeholder='Password' type='password'><button type='submit'>Connect</button></form></body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html, strlen(index_html));
    return ESP_OK;
}

/* Our URI handler function to be called during POST /uri request */
esp_err_t post_handler(httpd_req_t *req)
{
    ESP_LOGE("http","got post");
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    char content[100];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = MIN(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    /* Send a simple response */
    const char resp[] = "URI POST Response";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t form_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, MIN(req->content_len, sizeof(buf) - 1));
    if (ret >= 0)
    {
        buf[ret] = 0; // Null-terminate
        ESP_LOGI("POST", "Received: %s", buf);
        // Parse SSID and pass here (manual or with strtok)
        httpd_resp_sendstr(req, "Credentials received");

        memset(&(user_cfg_basic.ssid),'\0',sizeof(user_cfg_basic.ssid));
        memset(&(user_cfg_basic.passwd),'\0',sizeof(user_cfg_basic.passwd));

        char ssid_temp[40] = {'\0'};
        char passwd_temp[40] = {'\0'};

        if (sscanf(buf, "ssid=%39[^&]&pass=%39[^&]", ssid_temp, passwd_temp) != 2)  /* NOTE: TODO: this is a very janky solution, some passwords will not get correctly parsed, this has to be redone at some point.*/
        {
            ESP_LOGW("USR_CFG", "SSCANF parse failed.");
        }
        else
        {

            for(int i = 0; i < 40; i++){  /* TODO: woah wtf is this, what in the holy name of workarounds */
              if(ssid_temp[i] == '+'){
                ssid_temp[i] = ' ';
              }
              if(passwd_temp[i] == '+'){
                passwd_temp[i] = ' ';
              }
            }

            snprintf((char*)(user_cfg_basic.ssid), 40u, "%s", ssid_temp);
            snprintf((char*)(user_cfg_basic.passwd), 40u, "%s", passwd_temp);
            // sscanf(buf, "ssid=%s&pass=%s", user_cfg_basic.ssid, user_cfg_basic.passwd);
            user_cfg_basic.rdy_for_update = Nvs_Manager::config_in_use;

            ESP_LOGW("USR_CFG", "ssid: %s, pass: %s", user_cfg_basic.ssid, user_cfg_basic.passwd);

            save_user_config();

            ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");
            ESP_LOGW("USR_CFG", "NEW CONFIG WAS RECIEVED FROM HTTP PROVISONING");
            ESP_LOGW("USR_CFG", "CONFIG WAS WRITTEN TO FLASH, AND WILL BE USED ON NEXT WAKE");
            ESP_LOGW("USR_CFG", "FINGERS CROSSED 🤞");
            ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");
            ESP_LOGW("USR_CFG", "ESP IS EXECUTING SOFT RESET NOW");
            ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");
            ESP_LOGW("USR_CFG", "Device will reconnect on next wake...");
            esp_restart();
        }

        return ESP_OK;
    }

    return ESP_FAIL;
}

/* URI handler structure for GET /uri */
httpd_uri_t index_get = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = get_user_cfg_page,
    .user_ctx = NULL
};

/* URI handler structure for POST /uri */
httpd_uri_t uri_post = {
    .uri      = "/",
    .method   = HTTP_POST,
    .handler  = form_post_handler,
    .user_ctx = NULL
};

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &index_get);
        httpd_register_uri_handler(server, &uri_post);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server)
{
    if (server) {
        /* Stop the httpd server */
        httpd_stop(server);
    }
}