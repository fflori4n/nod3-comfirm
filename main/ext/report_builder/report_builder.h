#include <esp_err.h>
#include <algorithm>
#include <string>
#include <format>

enum class Report_group_e : const uint8_t
{
    MCU_INFO            = 0,
    DS18B20             = 1,
    DEV_INFO            = 2,
    DEV_WLAN_CON        = 3,
    SENSOR_HIVE_WEIGHT  = 4,
    SENSOR_BMX          = 5,
    REPORT_GROUP_E_MAX  = 6
};

struct Report_group_info_t
{
    const time_t cyclic_reporting_period_sec;
    time_t previous_report_unix;
    bool ready_to_be_sent;
};

RTC_FAST_ATTR Report_group_info_t group_report[]
{
    /* Report_group_e::MCU_INFO */ {60, 0, false},
    /* Report_group_e::DS18B20 */  {240, 0, false},
    /* Report_group_e::DEV_INFO */ {60, 0, false},
    /* Report_group_e::DEV_WLAN_CON */{240, 0, false},
    /* Report_group_e::SENSOR_HIVE_WEIGHT */ {180, 0, false},
    /* Report_group_e::SENSOR_BMX */ {240, 0, false}
};

enum class Report_status_e : const uint8_t
{
    empty_slot = 0u,
    invalid_key_error = 1u,
    out_of_range_error = 2u,
    waiting_for_transmit = 3u,
    ready_to_send = 4u,
    transmitted = 5u,
    recieved_by_server = 6u
};

constexpr char report_item_template_stl_str[]{"\"{0}\":\"{1}\","};
constexpr char report_item_template_float[]{"\"{0}\":{1:.2f},"};
constexpr char report_item_template_int[]{"\"{0}\":{1},"};

class Report_builder
{

public:

    typedef struct tag_value_t
    {
        Type_e type;
        char* key;
        void* value;
        Report_group_e group;
        Report_status_e status;

        uint16_t group_report_periods[(uint8_t)Report_group_e::REPORT_GROUP_E_MAX] = {0};

        static constexpr char* report_status_label[] = {"empty", "er_key", "er_range", "wait_send", "rdy_to_send", "transmitted", "recieved_by_server"};

        void debug_print(void){
            if(key != nullptr)
            {
                ESP_LOGW("Report_item_obj", "|%s| in group %d, status %s", key, (uint8_t)group, report_status_label[(uint8_t)status]);
            }   
        }

    }tag_value_t;

    constexpr static char* log_label_report_builder{COLOR_PINK"REP.BUILDER"COLOR_WHITE};
    static constexpr uint16_t report_item_list_length{30};
    tag_value_t report_items[report_item_list_length] = {};

    std::string service_data_str = "";
    inline static std::string json_data = ""; /* json_data.reserve(output_buffer_size); - wierd solution, but I want it to run out of memory here if alloc is bad */
    inline static std::string new_str = std::string(2048, ' ');

        
        //new_str.reserve(output_buffer_size);

    static constexpr uint8_t key_len_min{2};
    static constexpr uint8_t key_len_max{20};
    static constexpr uint8_t blacklist_len{5};
    static constexpr const char *char_blacklist{",. "};

    bool is_key_valid(std::string &key)
    {

        bool res = true;

        if ((key.length() < key_len_min) || key.length() > key_len_max)
        {
            res = false;
        }

        // for(uint16_t i=0; i< blacklist_len; i++)
        // {
        //     if(std::string::npos != key.find(char_blacklist[i])){
        //         res = false;
        //         break;
        //     }
        // }

        return res;
    }

    void debug_print_keyval_array(void){
        for(tag_value_t report_item : report_items){
            report_item.debug_print();
        }
    }

    void sort_keyval_array_by_group(void){

        std::sort(std::begin(report_items), std::end(report_items), [](const tag_value_t& first, const tag_value_t& second) {

            /* SORT: descending by status, ascemding by group, and abc */
            if(first.status != second.status)
            {
                return first.status > second.status;
            }
            else if(first.group != second.group)
            {
                return first.group < second.group;
            }
            else
            {
                return first.key < second.key;
            }
        });
    }

    void evaluate_send_time(time_t &now_unix)
    {
        ESP_LOGE("send report","checking send time");
        for (auto& group : group_report)
        {
            group.ready_to_be_sent = false;

            if (0 != now_unix)
            {

                if ((0 == group.previous_report_unix))
                {
                    group.ready_to_be_sent = true;
                    group.previous_report_unix = now_unix;
                }
                else
                {
                    if ((now_unix - group.previous_report_unix) >= group.cyclic_reporting_period_sec)
                    {
                        group.ready_to_be_sent = true;
                        group.previous_report_unix = now_unix;
                    }
                }
            }
        }

        /* now go over all tags, and set them to rdy to send if enough time has passed */
    }

    inline void evaluate_individual_ready_to_send(tag_value_t& item)
    {
        if((true == (group_report[(uint8_t)item.group].ready_to_be_sent)) && (Report_status_e::waiting_for_transmit == item.status)){
            item.status = Report_status_e::ready_to_send;
        }
    }

    esp_err_t add_keyval(const auto &group, const char *key, const void *value, const Type_e type, std::pair<float, float> range = {-9999.0f, 9999.0f})
    {
        /* ESP_ERR_NOT_FINISHED -- report item is not yet ready to be sent (based on it's report group cycle time) */
        /* ESP_OK -- proceed to next step */
        /* ESP_ERR_NOT_ALLOWED -- key invalid or value out of range*/
        /* ESP_FAIL -- other error*/

        esp_err_t res = ESP_OK;
        std::string new_item_keyval_str = "";
        ESP_LOGE(log_label_report_builder, "adding a new report item");

        tag_value_t &new_report_item = *std::find_if(std::begin(report_items), std::end(report_items), [](tag_value_t &report_item)
                                                     { return (Report_status_e::empty_slot == report_item.status); });

        if (&new_report_item == std::end(report_items))
        {
            /* no empty slot found! */
            ESP_LOGE(log_label_report_builder, "no empty slot to add item!");
            res = ESP_FAIL;
        }

        if (ESP_OK == res)
        {
            new_report_item.type = type;
            new_report_item.group = (Report_group_e)group;
            new_report_item.status = Report_status_e::waiting_for_transmit;

            /**/
            new_report_item.key = (char *)key;
            new_report_item.value = (void *)value;

            /* check range */
            switch (new_report_item.type)
            {
            case Type_e::FLOAT32:
            {
                if ((range.first > *((float *)value)) || (range.second < *((float *)value)))
                {
                    ESP_LOGW(log_label_report_builder, "%s's value is out of bounds! %.2f is not in range [%.2f : %.2f]", key, *((float *)value), range.first, range.second);
                    new_report_item.status = Report_status_e::out_of_range_error;
                    res = ESP_ERR_NOT_ALLOWED;
                }
            }
            break;

            case Type_e::INT32:
            {
                if ((range.first > (float)(*((int32_t *)value))) || (range.second < (float)(*((int32_t *)value))))
                {
                    ESP_LOGW(log_label_report_builder, "%s's value is out of bounds! %.2f is not in range [%.2f : %.2f]", key, (float)(*((int32_t *)value)), range.first, range.second);
                    new_report_item.status = Report_status_e::out_of_range_error;
                    res = ESP_ERR_NOT_ALLOWED;
                }
            }
            break;

            default:
            {
                /* do nothing */
            }
            break;
            }

            if (ESP_OK == res)
            {
                /* check if it is time to send this item */
                if (true == (group_report[(uint8_t)new_report_item.group].ready_to_be_sent))
                {
                    new_report_item.status = Report_status_e::ready_to_send;
                }
                else
                {
                    new_report_item.status = Report_status_e::waiting_for_transmit;
                    res = ESP_ERR_NOT_FINISHED;
                }
            }

            if (ESP_OK == res)
            {
                /* if value is in range, key ok and it is time to send, construct string and add it to outgoing buffer */
                switch (new_report_item.type)
                {
                case Type_e::PCHAR_STR:
                {
                    new_item_keyval_str = std::format(report_item_template_stl_str, new_report_item.key, ((const char *)value));
                }
                break;

                case Type_e::FLOAT32:
                {
                    new_item_keyval_str = std::format(report_item_template_float, new_report_item.key, *((float *)value));
                }
                break;

                case Type_e::INT32:
                {
                    new_item_keyval_str = std::format(report_item_template_int, new_report_item.key, *((int32_t *)value));
                }
                break;

                default:
                {
                    res = ESP_FAIL;
                }
                break;
                }

                if (ESP_OK == res)
                {
                    constexpr uint16_t json_data_size = 2000;
                    if (json_data.length() + new_item_keyval_str.length() < json_data_size)
                    {
                        json_data.append(new_item_keyval_str);
                        new_report_item.status = Report_status_e::transmitted;
                    }
                    else
                    {
                        ESP_LOGE(log_label_report_builder, "no more space in JSON data, %d + %d > %d bytes", json_data.length(), new_item_keyval_str.length(), json_data_size);
                        return ESP_FAIL;
                    }
                }
            }

            if (ESP_OK == res)
            {
                ESP_LOGI(log_label_report_builder, "new item was added: %s", new_item_keyval_str.c_str());
            }
            /*ESP_LOGI(log_label_report_builder, "json is now: %s", json_data.c_str());*/
        }
        
        return res;
    }

    esp_err_t add_keyval(const tag_value_t& new_item, const std::pair<float, float>& range = {-9999.0f, 9999.0f})
    {
        return this->add_keyval(new_item.group, new_item.key, new_item.value, new_item.type, range);
    }

    /* get the template, and fill it with as much JSON data as it can fit, then return the char* to be used by websoc feature */
    /* NOTE: where legacy meets STL, this will be an unholly combination of std::str and Cstr functions */
    uint16_t get_websoc_text(char* output_buffer, const uint16_t& output_buffer_size, char* ha_websocket_text_template)
    {
        uint16_t numof_available_bytes = output_buffer_size - strlen(ha_websocket_text_template);
        ESP_LOGW(log_label_report_builder, "available bytes in output: %d", numof_available_bytes);

        /* remove the last comma in service data str. */
        size_t pos = json_data.rfind(',');
        if (pos != std::string::npos)
        {
            json_data.erase(pos, 1);
        }

        this->debug_print_keyval_array();
        ESP_LOGW(log_label_report_builder, "JSON DATA IS: %s", json_data.c_str());
        snprintf(output_buffer, output_buffer_size, ha_websocket_text_template, json_data.c_str());

        return 0;
    }



    esp_err_t add_float_report_item(std::string key, auto value, const float &value_range_min, const float &value_range_max)
    {
        esp_err_t res = ESP_OK;

        // if(service_data_str.back() == '}'){
        //     ESP_LOGW(log_label,"%s cant be added, service data str is capped. clear str before appending new values!",key.c_str());
        //     res = ESP_FAIL;
        // }
        // else
        if (true != is_key_valid(key))
        {
            ESP_LOGW(log_label_report_builder, "%s is invalid key format", key.c_str());
            res = ESP_FAIL;
        }
        else if ((value < value_range_min) || (value > value_range_max))
        {
            ESP_LOGW(log_label_report_builder, "%s's value is out of bounds! [%.2f < %.2f < %.2f], will not be added to service data str!", key.c_str(), value_range_min, value, value_range_max);
            res = ESP_FAIL;
        }
        else
        {
            std::string new_item_str = std::format("\"{0}\":{1:.2f},", key, static_cast<float>(value));
            // ESP_LOGI(log_label,"%s",new_item_str.c_str());

            service_data_str.append(new_item_str);
        }

        return res;
    }

    esp_err_t add_uint_report_item(std::string key, auto value, const auto &value_range_min, const auto &value_range_max)
    {
        esp_err_t res = ESP_OK;

        if (true != is_key_valid(key))
        {
            ESP_LOGW(log_label_report_builder, "%s is invalid key format", key.c_str());
            res = ESP_FAIL;
        }
        else if ((value < 0) || (value < value_range_min) || (value > value_range_max))
        {
            ESP_LOGW(log_label_report_builder, "%s's value is out of bounds! [%.2f < %.2f < %.2f], will not be added to service data str!", key.c_str(), value_range_min, value, value_range_max);
            res = ESP_FAIL;
        }
        else
        {
            std::string new_item_str = std::format("\"{0}\":{1},", key, static_cast<uint32_t>(value));
            // ESP_LOGI(log_label,"%s",new_item_str.c_str());

            service_data_str.append(new_item_str);
        }

        return res;
    }

    esp_err_t add_cstr_report_item(std::string key, const char *value)
    {
        esp_err_t res = ESP_OK;
        std::string std_str_value(value);
        

        if (true != is_key_valid(key))
        {
            ESP_LOGW(log_label_report_builder, "%s is invalid key format", key.c_str());
            res = ESP_FAIL;
        }
        else if (true != is_key_valid(std_str_value))
        {
            ESP_LOGW(log_label_report_builder, "%s is invalid format", value);
            res = ESP_FAIL;
        }
        else
        {
            std::string new_item_str = std::format("\"{0}\":\"{1}\",", key, std_str_value);
            // ESP_LOGI(log_label,"%s",new_item_str.c_str());

            service_data_str.append(new_item_str);
        }

        return res;
    }

    std::string get_service_data_buffer(void)
    {
        /* remove the last comma in service data str. */
        size_t pos = service_data_str.rfind(',');
        if (pos != std::string::npos)
        {
            service_data_str.erase(pos, 1);
        }

        return service_data_str;
    }
};
