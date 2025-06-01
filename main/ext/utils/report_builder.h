#include <string>
#include <format>

class Report_builder
{

    static constexpr const char* log_label{"REP_BUILDER"};

    static constexpr uint8_t key_len_min{2};
    static constexpr uint8_t key_len_max{20};
    static constexpr uint8_t blacklist_len{5};
    static constexpr const char* char_blacklist{",. "};

    bool is_key_valid(std::string& key){

        bool res = true;

        if((key.length() < key_len_min) || key.length() > key_len_max)
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

public:

/*  @TODO: almost all of this should be static. */

    std::string service_data_str = "";

    esp_err_t add_float_report_item(std::string key, auto value, const float& value_range_min, const float& value_range_max)
    {
        esp_err_t res = ESP_OK;

        // if(service_data_str.back() == '}'){
        //     ESP_LOGW(log_label,"%s cant be added, service data str is capped. clear str before appending new values!",key.c_str()); 
        //     res = ESP_FAIL;
        // }
        // else
        if(true != is_key_valid(key))
        {
            ESP_LOGW(log_label,"%s is invalid key format",key.c_str()); 
            res = ESP_FAIL;
        }
        else if((value < value_range_min) || (value > value_range_max))
        {
             ESP_LOGW(log_label,"%s's value is out of bounds! [%.2f < %.2f < %.2f], will not be added to service data str!",key.c_str(), value_range_min, value, value_range_max); 
             res = ESP_FAIL;
        }
        else
        {
            std::string new_item_str = std::format("\"{0}\":{1:.2f},", key, static_cast<float>(value));
            //ESP_LOGI(log_label,"%s",new_item_str.c_str()); 

            service_data_str.append(new_item_str);
        }

        return res;
    }

    esp_err_t add_uint_report_item(std::string key, auto value, const auto& value_range_min, const auto& value_range_max)
    {
        esp_err_t res = ESP_OK;

        if(true != is_key_valid(key))
        {
            ESP_LOGW(log_label,"%s is invalid key format",key.c_str()); 
            res = ESP_FAIL;
        }
        else if((value < 0) || (value < value_range_min) || (value > value_range_max))
        {
             ESP_LOGW(log_label,"%s's value is out of bounds! [%.2f < %.2f < %.2f], will not be added to service data str!",key.c_str(), value_range_min, value, value_range_max); 
             res = ESP_FAIL;
        }
        else
        {
            std::string new_item_str = std::format("\"{0}\":{1},", key, static_cast<uint32_t>(value));
            //ESP_LOGI(log_label,"%s",new_item_str.c_str()); 

            service_data_str.append(new_item_str);
        }

        return res;
    }

    esp_err_t add_cstr_report_item(std::string key, const char* value)
    {
        esp_err_t res = ESP_OK;
        std::string std_str_value(value);

        if(true != is_key_valid(key))
        {
            ESP_LOGW(log_label,"%s is invalid key format",key.c_str()); 
            res = ESP_FAIL;
        }
        else if(true != is_key_valid(std_str_value))
        {
             ESP_LOGW(log_label,"%s is invalid format", value); 
             res = ESP_FAIL;
        }
        else
        {
            std::string new_item_str = std::format("\"{0}\":\"{1}\",", key, std_str_value);
            //ESP_LOGI(log_label,"%s",new_item_str.c_str()); 

            service_data_str.append(new_item_str);
        }

        return res;
    }

    std::string get_service_data_buffer(void){

        /* is str capped? if so, it ready to send, will not be modified until it is cleared.*/
        //if(service_data_str.back() != '}'){

            /* remove the last comma in service data str. */
            size_t pos = service_data_str.rfind(',');
            if (pos != std::string::npos) {
                service_data_str.erase(pos, 1);
            }
            /* cap the string */
            //service_data_str.append("}");
        //}

        return service_data_str;
    }

};